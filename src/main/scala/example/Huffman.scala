package example

import chisel3._
import chisel3.util._

// Idea: leave tree building to software. Don't bother implementing a hardware heap.

// Design: The Snappy encoding step will stop right before bit packing. 
// If Huffman code is to be used, it will use DMA to write the frequency
// into the memory. Each frequency is 15 bits (So the max size of a block
// is 32K), with highest bit as valid bits. It takes 64 cycles to complete transaction.
// (Note that the software can start adding freq to a priority queue during DMA access)

// Then, software execute an instruction to start encoding.

class EncodeShiftBuffer extends Module {
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
    val canon_table = Input(UInt((32 - 8).W))
    val code_table = Input(Vec(2, UInt((32 - 8).W)))
    val table_idx_table = Input(UInt((32 - 8 - 1).W))

    val req = Flipped(Decoupled(new HuffmanEncodeRequest))
    val sp_read = Vec(2, new ScratchpadReadIO(nRows, 8))
    val resp = Valid(UInt(8.W))
}

// New encoding & decoding design:
// We have a canonical huffman encoding table to map the original symbol to 
// its canonical form; if we are already canonical, skip this step.
// (Canonical symbol: if 0 is the left branch and 1 is the right branch in the 
// tree, the leftmost leaf node will have a canonical symbol 0. It's very similar
// to BST rank, although this is not a BST)

// We group every four bits in the encoding together. For every of the 16 entries,
// store the max canonical symbol under the current tree. If we have a leave node,
// set all entries representing the nonexisting children of the leave node 
// to the canonical symbol.

// The entry is 16-bit wide, the lower 8 bits store the symbol as specified above. 
// The higher 8 bits store the table index (every table is of size 16). This keep the
// table representation compact. If we are at a leaf node, the table index will be 0,
// the root table. We will never go back to root table from any other table, so it's
// used to indicate a leaf node.

// Get 16 8-bit comparators. Two priority encoders, one for comparator output and one for
// reversed output. 

// Software needs to prepare the list of all symbol, in their order from left to right.
// Calculate symbol for every layer. 

// For decoding, use the 4 bits in the current group to index into the table for the 
// next layer's table. If the next level of table is 0, return the canonical symbol.
// We run the canonical symbol through the LUT and convert back to original symbol.

// Encoding requires two 64-bit ports. 

// Table format: Two 8*256 bits table for Canonical symbol translation (encode/decode).
// A series of 8*16 tables for max canonical symbols of each branch. The higher 4
// bits and lower 4 bits are stored separately as a 64-bit line in a larger table.
// A series of 8*16 tables for next table index, with regular table layout. 
// The big table should have 6 index bits (64 tables max) to fit in four table. 

// Encoder for static (or generated) Huffman tree. 
class HuffmanEncoder(nRows: Int) extends Module {
    val io = IO(new HuffmanIO(nRows))

    // States
    // When we receive a start request at s_ready, read the first symbol.
    // At s_symbol_read, convert symbol to canonical symbol and read the next symbol in.
    // At s_lut_read, read the max canonical symbol for every 4th-gen children.
    // At s_encode, write the 4-bit encoding to sp, read the next table index.
    val s_ready :: s_symbol_read :: s_lut_read :: s_encode :: Nil = Enum(4)
    val state_reg = RegInit(s_ready)

    io.req.ready := state_reg === s_ready
    io.sp_read(1).en := false.B
    io.sp_read(1).addr := DontCare
    io.sp_read(0).en := false.B
    io.sp_read(0).addr := DontCare

    val addr_reg = Reg(UInt(32.W))
    val len_reg = Reg(UInt(32.W))

    val symbol_reg = Reg(UInt(8.W))
    val canon_reg = Reg(UInt(8.W))
    val table_idx_reg = Reg(UInt(8.W))

    val buffer = Module(new EncodeShiftBuffer)

    // Output align logic
    val flush_buffer = Wire(Bool())
    flush_buffer := false.B
    io.resp := buffer.io.out
    // The buffer flush is triggered on the s_ready cycle after session end to avoid input conflict
    buffer.io.flush := RegNext(flush_buffer)
    buffer.io.in_en := false.B
    buffer.io.shift_in := DontCare
    buffer.io.len_code := DontCare

    // Symbol comparison logic
    val max_symbols = Wire(UInt(128.W))
    val max_symbols_vec = Wire(Vec(16, UInt(8.W)))
    max_symbols_vec := max_symbols
    val symbols_eq = Wire(Vec(16, Bool()))
    val symbols_lt = Wire(Vec(16, Bool()))
    for (i <- 0 until 16) {
        symbols_eq(i) := canon_reg === max_symbols_vec(i)
        symbols_lt(i) := canon_reg > max_symbols_vec(i)
    }
    val symbol_end_reg = RegNext(symbols_eq.asUInt().orR)
    val symbol_end_left = PriorityEncoder(symbols_eq)
    val symbol_end_right = PriorityEncoder(symbols_eq.reverse)
    val symbol_mask_reg = RegNext(~(symbol_end_left ^ symbol_end_right))
    val match_child_reg = RegNext(PriorityEncoder(symbols_lt))

    // State logic
    when (state_reg === s_ready && io.req.fire()) {
        state_reg := s_lut_read
        addr_reg := io.req.bits.head + 1.U
        len_reg := io.req.bits.length - 1.U
        table_idx_reg := 0.U

        io.sp_read(1).en := true.B
        io.sp_read(1).addr := io.req.bits.head
        symbol_reg := io.sp_read(1).data
    } 
    .elsewhen (state_reg === s_symbol_read) {
        state_reg := s_lut_read
        addr_reg := addr_reg + 1.U;

        io.sp_read(1).en := len_reg === 0.U
        io.sp_read(1).addr := addr_reg
        symbol_reg := io.sp_read(1).data

        io.sp_read(0).en := true.B
        io.sp_read(0).addr := Cat(io.canon_table, io.symbol_reg)
        canon_reg := io.sp_read(0).data
    }
    .elsewhen (state_reg === s_lut_read) {
        state_reg := s_encode
        
        io.sp_read(1).en := true.B
        io.sp_read(1).addr := Cat(io.code_table(1), table_idx_reg)
        max_symbols(127, 64) := io.sp_read(1).data

        io.sp_read(0).en := true.B
        io.sp_read(0).addr := Cat(io.code_table(0), table_idx_reg)
        max_symbols(63, 0) := io.sp_read(0).data
    }
    .elsewhen (state_reg === s_encode) {
        io.sp_read(1).en := true.B
        io.sp_read(1).addr := Cat(io.table_idx_table, table_idx_reg, match_child_reg(3))
        val next_table_vec = Wire(Vec(8, UInt(8.W)))
        next_table_vec := io.sp_read(1).data
        table_idx_reg := next_table_vec(match_child_reg(2, 0))
        
        buffer.io.in_en := true.B
        buffer.io.shift_in := match_child_reg
        // If we reach the end of the block...
        when (symbol_end_reg) {
            buffer.io.len_code := PriorityEncoder(symbol_mask_reg)
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
            buffer.io.len_code := 0.U
        }
    }
}

class DecodeShiftBuffer extends Module {
    val io = IO(new Bundle {
        val flush = Input(Bool())
        val refill = Flipped(Decoupled(UInt(8.W)))
        val shift = Input(UInt(3.W))
        val out = Valid(UInt(4.W))
    })

    val buffer_reg = Reg(UInt(16.W))
    val valid_reg = RegInit(0.U(16.W))
    
    io.out.valid := valid_reg(15, 12).andR
    io.out.bits := buffer_reg(15, 12)

    val shifted_buffer = buffer_reg << io.shift
    val shifted_valid = valid_reg << io.shift
    val shamt = PriorityEncoder(Reverse(shifted_valid(15, 8)))
    io.refill.ready := ~shifted_valid(7, 0).andR
    valid_reg := Cat(Fill(8, io.refill.fire()), 0.U(8.W)) >> shamt
    buffer_reg := shifted_buffer | Cat(io.refill.bits, 0.U(8.W)) >> shamt
}

// Huffman tree decoder
class HuffmanDecoder extends Module {
    val io = IO(new HuffmanIO(32))

    val s_ready :: s_busy :: Nil = Enum(2)
    val state_reg = RegInit(s_ready)

    val addr_reg = Reg(UInt(32.W))
    val len_reg = Reg(UInt(32.W))
    val table_idx_reg = Reg(UInt(8.W))
    
    io.sp_read(1).en := false.B
    io.sp_read(1).addr := DontCare
    io.sp_read(0).en := false.B
    io.sp_read(0).addr := DontCare
    io.resp.valid := false.B
    io.resp.bits := DontCare
    val buffer = Module(new DecodeShiftBuffer)
    buffer.io.flush := false.B
    buffer.io.shift := 0.U
    buffer.io.refill.valid := false.B
    buffer.io.refill.bits := DontCare

    when (state_reg === s_ready && io.req.fire()) {
        state_reg := s_busy
        addr_reg := io.req.bits.head
        len_reg := io.req.bits.length - 1.U
        table_idx_reg := 0.U
    } .elsewhen (state_reg === s_busy) {
        // If the buffer needs refill, refill it during this cycle
        when (buffer.io.refill.ready) {
            buffer.io.refill.valid := true.B
            io.sp_read(1).en := true.B
            io.sp_read(1).addr := addr_reg
            buffer.io.refill.bits := io.sp_read(1).data
            addr_reg := addr_reg + 1.U
        } .otherwise {
            // When busy, we read four bits from the buffer and look at the table.
            // Port 1 read the next table index, and port 0 read the max symbol. 
            // If port 1 indicates that this is a leaf, we will compare the max symbol 
            // in the current position with all other max symbol in port 0 to determine
            // the # of bits remaining in this leaf.
            // We send the bits info into the shift buffer. 
            io.sp_read(1).en := true.B
            io.sp_read(1).addr := Cat(io.table_idx_table, table_idx_reg, buffer.io.out.bits(3))
            val next_table_vec = Wire(Vec(8, UInt(8.W)))
            next_table_vec := io.sp_read(1).data
            val next_idx = next_table_vec(buffer.io.out.bits(2, 0))

            io.sp_read(0).en := true.B
            io.sp_read(0).addr := Cat(io.canon_table(buffer.io.out.bits(3)), table_idx_reg)
            val max_symbol_vec = Vec(8, UInt(8.W))
            max_symbol_vec := io.sp_read(0).data

            val max_symbol = max_symbol_vec(buffer.io.out.bits(2, 0))
            val max_symbol_eq_b0 = VecInit(max_symbol_vec map (max_symbol === _))
            val max_symbol_eq_b1 = VecInit(((max_symbol_eq_b0 grouped 2) map { a => a(1) && a(0) }).toSeq)
            val max_symbol_eq_b2 = VecInit(((max_symbol_eq_b1 grouped 2) map { a => a(1) && a(0) }).toSeq)
            val max_symbol_eq_b3 = max_symbol_eq_b2(1) && max_symbol_eq_b2(0)

            val symbol_mask = Cat(max_symbol_eq_b3, max_symbol_eq_b2(buffer.io.out.bits(2)),
                max_symbol_eq_b1(buffer.io.out.bits(2, 1)), max_symbol_eq_b0(buffer.io.out.bits(2, 0)))
            val symbol_shift = PriorityEncoder(Reverse(symbol_mask)) +& 1.U

            when (buffer.io.out.valid) {
                table_idx_reg := next_idx
                when (next_idx === 0.U) {
                    buffer.io.shift := symbol_shift
                    io.resp.valid := true.B
                    io.resp.bits := max_symbol
                    when (len_reg === 0.U) {
                        buffer.io.flush := true.B
                        state_reg := s_ready
                    } .otherwise {
                        len_reg := len_reg - 1.U
                    }
                } .otherwise {
                    buffer.io.shift := 4.U
                }
            }
        }
    }
}