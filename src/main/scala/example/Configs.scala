package snappyaccl

import chisel3._
import freechips.rocketchip.config.{Config, Parameters}
import freechips.rocketchip.rocket.RocketCoreParams
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet}
import testchipip._

class WithCompressionAccelerator extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
      (p: Parameters) => {
         val snappy = LazyModule.apply(new CompressionAccelerator(OpcodeSet.custom3)(p))
         snappy
      }
    )
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site).map({
    case tp: RocketTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      core = tp.tileParams.core.copy(nPMPs = 0)))
    case other => other
  })
})
