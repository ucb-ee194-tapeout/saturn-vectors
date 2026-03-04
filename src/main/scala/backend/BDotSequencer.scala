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

class BDotSequencerIO()(implicit p: Parameters) extends SequencerIO(new BDotSequencerControl) with HasVectorParams {
  val rvs1 = Decoupled(new VectorReadReq)
  val rvs2 = Decoupled(new VectorReadReq)
  val rvm  = Decoupled(new VectorReadReq)
  val batch_read_vs2 = Output(Bool())
  val tail = Output(Bool())
  val acc_intent = Output(UInt(32.W))
  val vbws_acc_intent = Input(UInt(32.W))
}

class BDotSequencer()(implicit p: Parameters) extends Sequencer[BDotSequencerControl]()(p) with HasVectorParams with HasCoreParameters {

  val bdot_insns = vParams.bdotInsns

  def accepts(inst: VectorIssueInst) = !inst.vmu && new VectorDecoder(inst, bdot_insns, Nil).matched

  val io = IO(new BDotSequencerIO)

  // Current instruction
  val valid = RegInit(false.B)
  val busy = RegInit(false.B)
  val inst = Reg(new BackendIssueInst)
  
  val eg_idx = RegInit(0.U(log2Ceil(maxVLMax).W))
  val head = Reg(Bool())

  val acc_sel = Reg(UInt(5.W))

  val rvs1_mask = Reg(UInt(egsTotal.W))
  val rvs2_mask = Reg(UInt(egsTotal.W))
  val rvm_mask = Reg(UInt(egsTotal.W))

  val fp = Reg(Bool())
  val altfmt = Reg(Bool())
  val signed = Reg(Bool())
  val batched = Reg(Bool())
  val widen = Reg(UInt(2.W))

  val emul = Reg(UInt(2.W))
  val in_eew = Reg(UInt(2.W))
  val renv1 = Reg(Bool())
  val renv2 = Reg(Bool())
  val renvm = Reg(Bool())
  val vl = Reg(UInt((1+log2Ceil(maxVLMax)).W))

  val eg_idx_max = (vLen / dLen).U << emul

  val next_eg_idx = eg_idx +& 1.U
  val elements_width = (dLen / 8).U >> in_eew
  val next_vl = Mux(vl >= elements_width, vl - elements_width, 0.U)

  val tail = (next_eg_idx === eg_idx_max) && io.iss.fire

  io.dis.ready := (!busy || tail) && !io.dis_stall && io.iss.ready

  // New instruction
  when (io.dis.fire) {
    val dis_inst = io.dis.bits

    assert(dis_inst.vstart === 0.U)

    val dis_ctrl = new VectorDecoder(dis_inst, bdot_insns, Seq(BDotSigned, BDotFP, BDotBatched, BDotWiden))

    fp := dis_ctrl.bool(BDotFP)
    altfmt := dis_inst.vconfig.vtype.altfmt
    signed := dis_ctrl.bool(BDotSigned)
    batched := dis_ctrl.bool(BDotBatched)
    widen := dis_ctrl.uint(BDotWiden)

    val dis_vs1_arch_mask = get_arch_mask(dis_inst.rs1, Mux(dis_ctrl.bool(BDotBatched), 0.U, dis_inst.emul))
    val dis_vs2_arch_mask = get_arch_mask(dis_inst.rs2, Mux(dis_ctrl.bool(BDotBatched), 3.U, dis_inst.emul))

    valid := true.B
    busy := true.B
    inst := io.dis.bits

    renv1 := true.B
    renv2 := true.B // !((dis_ctrl.bool(BDotSet) && !(dis_ctrl.bool(BDotSetZero))) || dis_ctrl.bool(BDotWB))
    renvm := false.B

    rvs1_mask     := FillInterleaved(egsPerVReg, dis_vs1_arch_mask)
    rvs2_mask     := FillInterleaved(egsPerVReg, dis_vs2_arch_mask)
    rvm_mask      := Mux(renvm, ~(0.U(egsPerVReg.W)), 0.U)

    emul := Mux(batched, 0.U, dis_inst.emul)
    in_eew := dis_inst.vconfig.vtype.vsew
    vl := dis_inst.vconfig.vl

    acc_sel := dis_inst.rd
    
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
  io.seq_hazard.bits.rintent := hazardMultiply(rvs1_mask | rvs2_mask | rvm_mask)
  io.seq_hazard.bits.wintent := 0.U
  io.seq_hazard.bits.vat := inst.vat

  val vs1_read_oh = Mux(renv1, UIntToOH(io.rvs1.bits.eg), 0.U)
  val vs2_oh = UIntToOH(io.rvs2.bits.eg)
  val vs2_oh_batched = vs2_oh.asTypeOf(Vec(4, Vec(8, UInt(egsPerVReg.W)))).map { grp =>
    Fill(8, grp(0))
  }.asUInt
  val vs2_read_oh = Mux(renv2, Mux(batched, vs2_oh_batched, vs2_oh), 0.U)
  val vm_read_oh  = Mux(renvm, UIntToOH(io.rvm.bits.eg), 0.U)

  val raw_hazard = ((vs1_read_oh | vs2_read_oh | vm_read_oh) & io.older_writes) =/= 0.U
  val data_hazard = raw_hazard

  val oldest = inst.vat === io.vat_head

  io.rvs1.valid := valid && renv1
  io.rvs1.bits.eg := (inst.rs1 << log2Ceil(egsPerVReg)) + eg_idx
  io.rvs2.valid := valid && renv2
  io.rvs2.bits.eg := (Mux(batched, inst.rs2 & "h1F".U(5.W), inst.rs2) << log2Ceil(egsPerVReg)) + eg_idx
  io.rvm.valid := valid && renvm
  io.rvm.bits.eg := eg_idx + (eg_idx >> in_eew)
  io.batch_read_vs2 := batched && valid && renv2

  io.rvs1.bits.oldest := oldest
  io.rvs2.bits.oldest := oldest
  io.rvm.bits.oldest := oldest

  when (io.iss.fire) {
    when (!tail) {
      eg_idx := next_eg_idx
      vl := next_vl
    } .otherwise {
      eg_idx := 0.U
    }
  }

  io.acc_intent := UIntToOH(acc_sel)

  io.iss.valid := (valid &&
    !data_hazard &&
    !(renv1 && !io.rvs1.ready) &&
    !(renv2 && !io.rvs2.ready) &&
    !(renvm && !io.rvm.ready) &&
    !io.vbws_acc_intent(acc_sel)
  )
  io.iss.bits.fp := fp
  io.iss.bits.altfmt := altfmt
  io.iss.bits.signed := signed
  io.iss.bits.batched := batched
  io.iss.bits.in_eew := in_eew
  io.iss.bits.acc_sel := acc_sel
  io.iss.bits.vl := vl

  io.busy := busy
  io.head := head
}