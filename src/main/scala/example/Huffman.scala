package example

import chisel3._
import chisel3.util._

// Idea: leave tree building to software. Don't bother implementing a hardware heap.

// Design: The Snappy encoding step will stop right before bit packing. 
// If Huffman code is to be used, it will use DMA to write the frequency
// into the memory. Each frequency is 15 bits (So the max size of a block
// is 32K), with highest bit as valid bits. It takes 64 cycles to complete transaction.
// (Note that the software can start adding freq to a priority queue during DMA access)

// Then, software execute an instruction to start encoding, with a pointer to the tree root.
// [63:56] of the tree root is the symbol, [55:28] for pointer to 1's child, [27:0] to 0's child.
// Higher 36 bits of the address is always the same. Take 256 cycles to read.

// Use the "huffman decoding hardware" paper's idea and build the lookup table for encoding.
// Encoding block for the tree: 
// d[7] = End
// d[5:4] = length (if a full block, = 00)
// d[3:0] = output.
// When we get a symbol, we check the memory region to determine the starting address, then start
// reading the block for translation.

class ShiftBuffer extends Module {
    val io = IO(new Bundle {
        val flush = Input(Bool())
        val in_en = Input(Bool())
        val shift_in = Input(UInt(4.W))
        val len_code = Input(UInt(2.W))
        val out = Flipped(Valid(UInt(8.W)))
    })

    val buffer_reg = Reg(UInt(12.W))
    val valid_reg = Reg(UInt(12.W))

    val out_valid = valid_reg(11, 8).orR
    val out_shamt = PriorityEncoder(Reverse(valid_reg(11, 8)))
    val out_bits = (valid_reg << out_shamt)(11, 4)

    // Flush logic: If in_en is high, flush will be ignore. 
    // Otherwise, if out_valid is high when flush is high, delay the flush to the next cycle.
    val delay_flush_reg = RegNext(io.flush && !io.in_en && out_valid)
    val flush_valid = valid_reg(7, 0).orR && ((io.flush && ~io.in_en) || delay_flush_reg)
    val flush_shamt = PriorityEncoder(Reverse(valid_reg(7, 0)))
    val flush_output = (buffer_reg << flush_shamt)(7, 0)
    assert(!delay_flush_reg || !io.in_en, "Data input during delayed flush; delay input for 1 cycle to fix")

    io.out.valid := out_valid || flush_valid
    io.out.bits := Mux(out_valid, out_bits, flush_output)

    when (io.in_en) {
        buffer_reg := Cat(buffer_reg(7, 0), io.shift_in) >> io.len_code
        valid_reg := (Cat(valid_reg(7, 0), io.shift_in) >> io.len_code) &
            (Cat(Fill(8, out_valid), 0.U(4.W)) >> out_shamt)
    } .elsewhen (io.flush) {
        buffer_reg := 0.U
        valid_reg := 0.U
    }

}

class HuffmanEncodeRequest extends Bundle {
    val head = UInt(32.W)
    val length = UInt(32.W)
}

class HuffmanIO(nRows: Int) extends Bundle {
    val tree_lut = UInt(32.W)
    val req = Flipped(Decoupled(new HuffmanEncodeRequest))
    val sp_io = new ScratchpadReadIO(nRows, 8)
    val resp = Valid(UInt(8.W))
}

// Encoder for static (or generated) Huffman tree. 
class HuffmanEncoder(nRows: Int) extends Module {
    val io = IO(new HuffmanIO(nRows))

    // States
    val s_ready :: s_symbol_read :: s_lut_read :: s_encode :: Nil = Enum(4)
    val state_reg = RegInit(s_ready)

    val addr_reg = Reg(UInt(32.W))
    val len_reg = Reg(UInt(32.W))
    val lut_idx_reg = Reg(UInt(8.W))

    val symbol_reg = Reg(UInt(8.W))

    val buffer = Module(new ShiftBuffer)

    io.sp_io.en := state_reg =/= s_ready && state_reg =/= s_end

    // Output align logic
    val flush_buffer = Wire(Bool())
    flush_buffer := false.B
    io.resp := buffer.io.out
    // The buffer flush is triggered on the s_ready cycle after session end to avoid input conflict
    buffer.io.flush := RegNext(flush_buffer)
    buffer.io.in_en := false.B
    buffer.io.shift_in := DontCare
    buffer.io.len_code := DontCare

    // State logic
    when (state_reg === s_ready && io.req.fire()) {
        state_reg := s_lut_read
        addr_reg := io.req.bits.head
        len_reg := io.req.bits.length - 1.U
    } 
    .elsewhen (state_reg === s_symbol_read) {
        io.sp_io.en := true.B
        io.sp_io.addr := addr_reg
        symbol_reg := io.sp_io.data
        state_reg := s_lut_read
    }
    .elsewhen (state_reg === s_lut_read) {
        io.sp_io.en := true.B
        io.sp_io.addr := io.tree_lut + symbol_reg
        lut_idx_reg := io.sp_io.data
        state_reg := s_encode
    }
    .elsewhen (state_reg === s_encode) {
        io.sp_io.en := true.B
        io.sp_io.addr := lut_idx_reg

        buffer.io.in_en := true.B
        // If we reach the end of the block...
        when (io.sp_io.data(7)) {
            // If we also reach the end of the request...
            when (len_reg === 0.U) {
                // Go back to ready state and flush the buffer
                state_reg := s_ready
                flush_buffer := true.B
            } .otherwise {
                // Otherwise, move address register and read the next symbol
                state_reg := s_symbol_read
                addr_reg := addr_reg + 1.U
                len_reg := len_reg - 1.U
            }
        } .otherwise {
            // If not, continue to read
            buffer.io.shift_in := io.sp_io.data(3, 0)
            buffer.io.len_code := 0.U
            lut_idx_reg := lut_idx_reg + 1.U
        }
    }
}

// Huffman tree walker. Read the tree in the memory and load them into scratchpad for encoding.
class HuffmanEncodingWalker(nRows: Int) extends Module {
    val io = IO(new Bundle {
        val sp_read = new ScratchpadReadIO(nRows, 8)
        val sp_write = new ScratchpadReadIO(nRows, 8)
        val stack_bottom = Input(sp_read.addr.cloneType)
        val tree_root = Input(sp_write.addr.cloneType)
        val tree_root = Input(sp_write.addr.cloneType)
    })

    // To avoid the stack consuming too much memory when traversing a extremely deep branch,
    // we will only search down for 4 level at a time. We will stop the search and store it
    // to a queue at level 4 if there are still more nodes under it. The queue start at 
    // (stack_bottom + 32 (# of possible entry) * 4 (entry length)).
}

// Huffman tree walker. Convert Huffman tree in compressed stream to the decoding format.
class HuffmanDecodingWalker extends Module {

}

// Huffman tree decoder.
class HuffmanDecoder extends Module {
    
}