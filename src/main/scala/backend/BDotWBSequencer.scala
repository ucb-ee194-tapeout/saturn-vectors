package saturn.backend

import chisel3._
import chisel3.util._
import chisel3.experimental.dataview._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import saturn.common._
import saturn.insns._
import scala.math._
import saturn.exu._

class BDotWBSequencerIO(pipe_depth: Int, acc_delay: Int)(implicit p: Parameters) extends SequencerIO(new BDotWBSequencerControl) with HasVectorParams {
  val rvd  = Decoupled(new VectorReadReq)
  val pipe_write_req = new VectorPipeWriteReqIO(1)
  val tail = Output(Bool())
  val in_flight = Input(UInt(32.W))
  val acc_intent = Output(UInt(32.W))
  val vbs_acc_intent = Input(UInt(32.W))
}

class BDotWBSequencer(pipe_depth: Int, acc_delay: Int)(implicit p: Parameters) extends Sequencer[BDotWBSequencerControl]()(p) with HasVectorParams with HasCoreParameters {

  val bdot_wb_insns = vParams.bdotWBInsns

  def accepts(inst: VectorIssueInst) = !inst.vmu && new VectorDecoder(inst, bdot_wb_insns, Nil).matched

  val io = IO(new BDotWBSequencerIO(pipe_depth, acc_delay))

  // Current instruction
  val valid = RegInit(false.B)
  val busy = RegInit(false.B)
  val inst = Reg(new BackendIssueInst)
  
  val eg_idx = RegInit(0.U(log2Ceil(maxVLMax).W))
  val head = Reg(Bool())

  val set_acc = RegInit(false.B)
  val set_acc_zero = Reg(Bool())
  val set_acc_bc = Reg(Bool())
  val writeback = RegInit(false.B)
  val acc_sel = Reg(UInt(5.W))

  val wvd_mask = Reg(UInt(egsTotal.W))
  val rvd_mask = Reg(UInt(egsTotal.W))
  val acc_mask = Reg(UInt(32.W))

  val emul = Reg(UInt(2.W))
  val in_eew = Reg(UInt(2.W))
  val renvd = Reg(Bool())

  val eg_idx_max = Mux(set_acc && set_acc_zero, 1.U, (vLen / dLen).U << emul)

  val next_eg_idx = eg_idx +& 1.U

  val tail = (next_eg_idx === eg_idx_max) && io.iss.fire

  io.dis.ready := (!busy || tail) && !io.dis_stall && io.iss.ready

  // New instruction
  when (io.dis.fire) {
    val dis_inst = io.dis.bits

    assert(dis_inst.vstart === 0.U)

    val dis_ctrl = new VectorDecoder(dis_inst, bdot_wb_insns, Seq(BDotSet, BDotSetZero, BDotSetBC, BDotWB))

    set_acc := dis_ctrl.bool(BDotSet)
    set_acc_zero := dis_ctrl.bool(BDotSetZero)
    set_acc_bc := dis_ctrl.bool(BDotSetBC)
    writeback := dis_ctrl.bool(BDotWB)

    val amul = dis_inst.rs1(2, 0)

    val dis_vd_arch_mask  = get_arch_mask(dis_inst.rd, amul) // TODO: Multi-vd outputs

    valid := true.B
    busy := true.B
    inst := io.dis.bits

    renvd := dis_ctrl.bool(BDotSet) && !dis_ctrl.bool(BDotSetZero)

    wvd_mask      := FillInterleaved(egsPerVReg, dis_vd_arch_mask)
    rvd_mask      := FillInterleaved(egsPerVReg, dis_vd_arch_mask)
    acc_mask      := get_arch_mask(dis_inst.rs2, amul)

    emul := amul // dis_inst.emul
    in_eew := 2.U // dis_inst.vconfig.vtype.vsew

    acc_sel := dis_inst.rs2

    head := true.B
  } .elsewhen (io.iss.fire) {
    valid := !tail
    head := false.B
  }

  when (tail && !io.dis.fire) {
    busy := false.B
  }
  io.tail := tail

  io.vat := inst.vat
  io.seq_hazard.valid := busy
  io.seq_hazard.bits.rintent := hazardMultiply(rvd_mask)
  io.seq_hazard.bits.wintent := hazardMultiply(wvd_mask)
  io.seq_hazard.bits.vat := inst.vat
  
  val wvd_eg = (inst.rd << log2Ceil(egsPerVReg)) + eg_idx

  val vd_read_oh  = Mux(renvd, UIntToOH(io.rvd.bits.eg), 0.U)
  val vd_write_oh = Mux(writeback, UIntToOH(wvd_eg), 0.U)

  val raw_hazard = (vd_read_oh & io.older_writes) =/= 0.U
  val waw_hazard = (vd_write_oh & io.older_writes) =/= 0.U
  val war_hazard = (vd_write_oh & io.older_reads) =/= 0.U
  val data_hazard = raw_hazard | waw_hazard | war_hazard

  val oldest = inst.vat === io.vat_head

  val current_rvd = (inst.rd << log2Ceil(egsPerVReg)) + eg_idx

  io.rvd.valid := valid && renvd
  io.rvd.bits.eg := current_rvd

  io.rvd.bits.oldest := oldest

  io.pipe_write_req.request := valid && writeback
  io.pipe_write_req.bank_sel := (if (vrfBankBits == 0) 1.U else UIntToOH(wvd_eg(vrfBankBits+log2Ceil(egsPerVReg)-1,log2Ceil(egsPerVReg))))
  io.pipe_write_req.pipe_depth := 0.U
  io.pipe_write_req.oldest := oldest
  io.pipe_write_req.fire := io.iss.fire

  when (io.iss.fire) {
    when (!tail) {
      eg_idx := next_eg_idx
      rvd_mask := rvd_mask & ~UIntToOH(current_rvd)
      wvd_mask := wvd_mask & ~UIntToOH(current_rvd)
      when (if (vLen == dLen) true.B else next_eg_idx(log2Ceil(vLen/dLen) - 1, 0) === 0.U) {
        acc_sel := acc_sel + 1.U
        acc_mask := acc_mask & ~UIntToOH(acc_sel)
      }
    } .otherwise {
      eg_idx := 0.U
    }
  }

  io.acc_intent := Mux(set_acc_bc, "hFFFFFFFF".U(32.W), acc_mask)

  io.iss.valid := (valid &&
    !data_hazard &&
    !(renvd && !io.rvd.ready) &&
    !(io.in_flight(acc_sel) || (set_acc_bc && io.in_flight.orR)) &&
    !(io.vbs_acc_intent(acc_sel) || (set_acc_bc && io.vbs_acc_intent.orR))
  )
  io.iss.bits.base_eg := wvd_eg
  io.iss.bits.in_eew := in_eew
  io.iss.bits.set_acc := set_acc
  io.iss.bits.set_acc_zero := set_acc_zero
  io.iss.bits.set_acc_bc := set_acc_bc
  io.iss.bits.writeback := writeback
  io.iss.bits.acc_sel := acc_sel

  io.busy := busy
  io.head := head
}