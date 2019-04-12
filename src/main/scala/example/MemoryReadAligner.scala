package example

import chisel3._
import chisel3.core.dontTouch
import chisel3.util.experimental.loadMemoryFromFile
import chisel3.util.{Cat, Decoupled, MuxCase, MuxLookup}

class DecoupledMemoryAlignerIO(addressWidth: Int, dataWidth: Int) extends Bundle {
  val address = Flipped(Decoupled(UInt(addressWidth.W)))
  val data = Decoupled(UInt(dataWidth.W))

  override def cloneType: this.type = new DecoupledMemoryAlignerIO(addressWidth, dataWidth).asInstanceOf[this.type]
}

class MemoryAlignerIO(addressWidth: Int, dataWidth: Int) extends Bundle {
  val address = Input(UInt(addressWidth.W))
  val data = Output(UInt(dataWidth.W))

  override def cloneType: this.type = new MemoryAlignerIO(addressWidth, dataWidth).asInstanceOf[this.type]
}

/**
  * Couples large-width memory with reads that are byte-addressable and want to see a smaller data size.
  */
class MemoryReadAligner(readAddressWidth: Int, readDataWidth: Int, memAddressWidth: Int, memDataWidth: Int) extends Module {
  val io = IO(new Bundle {
    val readIO = new DecoupledMemoryAlignerIO(readAddressWidth, readDataWidth)
    val memIO = Flipped(new MemoryAlignerIO(memAddressWidth, memDataWidth))
  })

  // special case for beginning
  val initialized = RegInit(false.B)

  // hold and tag the last read so that boundary-crossing reads can use it
  val cachedData = RegInit(0.U(memDataWidth.W))
  val cachedAddress = RegInit(0.U(memAddressWidth.W))

  // bytes per read
  val memBytes: Int = memDataWidth / 8
  val readBytes: Int = readDataWidth / 8

  // do some math
  val lowerReadAddress: UInt = io.readIO.address.bits / memBytes.U
  val upperReadAddress: UInt = lowerReadAddress + 1.U
  val lowerReadOffset: UInt = io.readIO.address.bits % memBytes.U
  val bytesInLowerRead: UInt = Mux(lowerReadOffset + readBytes.U < memBytes.U,
    readBytes.U,
    memBytes.U - lowerReadOffset
  )
  val bytesInUpperRead: UInt = readBytes.U - bytesInLowerRead

  // select the correct read data
  val aggregateReadData: Vec[UInt] = Cat(cachedData, io.memIO.data).asTypeOf(Vec(memBytes*2, UInt(8.W)))

  // concatenate the appropriate bytes from the lower and upper read data into the final output
  //  io.readIO.data.bits := Cat(
  //    aggregateReadData(lowerReadOffset, lowerReadOffset + bytesInLowerRead),
  //    upperReadData(0.U, bytesInUpperRead)
  //  )
  //TODO: do this the correct, parameterized way
  io.readIO.data.bits := Cat(
    aggregateReadData((memBytes*2).U - lowerReadOffset - 1.U),
    aggregateReadData((memBytes*2).U - lowerReadOffset - 2.U),
    aggregateReadData((memBytes*2).U - lowerReadOffset - 3.U),
    aggregateReadData((memBytes*2).U - lowerReadOffset - 4.U),
  )

  // the cached data is the correct data if the cached address matches the address we want
  val cachedDataValid: Bool = lowerReadAddress === cachedAddress && initialized

  // the read data output of this module is only valid if the cached data was valid
  io.readIO.data.valid := cachedDataValid

  // we can always output the correct data and accept the next address when the cached data is valid
  io.readIO.address.ready := cachedDataValid

  // when the cached data is correct (the lower read data), then we can read the upper data
  // otherwise, we will have to read the lower data first
  when(cachedDataValid) {
    io.memIO.address := upperReadAddress
  }.otherwise {
    io.memIO.address := lowerReadAddress
  }

  // when we need to load up a new cached data, advance the "pipeline"
  when(!cachedDataValid) {
    initialized := true.B
    cachedData := io.memIO.data
    cachedAddress := lowerReadAddress
  }

  dontTouch(initialized)
  dontTouch(cachedData)
  dontTouch(cachedAddress)
  dontTouch(lowerReadAddress)
  dontTouch(upperReadAddress)
  dontTouch(lowerReadOffset)
  dontTouch(bytesInLowerRead)
  dontTouch(bytesInUpperRead)
  dontTouch(cachedDataValid)
}

class MemoryReadAlignerTestModule(readAddressWidth: Int, readDataWidth: Int, memAddressWidth: Int, memDataWidth: Int) extends Module {
  val io = IO(new Bundle {
    val readIO = new DecoupledMemoryAlignerIO(readAddressWidth, readDataWidth)
  })

  // create aligner and backing mem
  val aligner = Module(new MemoryReadAligner(readAddressWidth, readDataWidth, memAddressWidth, memDataWidth))
  dontTouch(aligner.io)
  val backingMem = Mem(Math.pow(2, memAddressWidth).toInt, UInt(memDataWidth.W))
  loadMemoryFromFile(backingMem, "memdata/alignerTestData.txt")

  // connect the memory to the aligner
  aligner.io.memIO.data := backingMem.read(aligner.io.memIO.address)

  // connect the aligner to the io
  aligner.io.readIO <> io.readIO
}