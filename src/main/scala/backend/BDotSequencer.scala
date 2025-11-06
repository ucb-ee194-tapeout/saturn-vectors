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

// class BDotCache(implicit p: Parameters) extends CoreModule with HasVectorParams {
//   // This thing should probably use a Mem or something

//   val io = IO(new Bundle {
//     val valid = Output(UInt(8.W))
//     val group = Output(UInt(2.W))

//     val new_group = Input(UInt(2.W))
//     val set_group = Input(Bool())

//     val write_data = Input(UInt(vLen.W))
//     val write_addr = Input(UInt(3.W))
//     val write_en = Input(Bool())

//     val read_data = Output(Vec(8, UInt(vLen.W)))
//   })

//   val valid = RegInit(VecInit(Seq.fill(8)(false.B)))
//   val group = RegInit(0.U(2.W))
//   when (io.set_group) {
//     group := io.new_group
//     valid.foreach { e => e := false.B }
//   }
//   io.valid := valid.asUInt

//   for (i <- 0 to 7) {
//     val register = Reg(UInt(vLen.W))
//     io.read_data(i) := register
//     when (io.write_en && io.write_addr === i.U) {
//       register := io.write_data
//       valid(i) := true.B
//     }
//   }
// }

class BDotSequencerIO(pipe_depth: Int, acc_delay: Int)(implicit p: Parameters) extends SequencerIO(new BDotSequencerControl) with HasVectorParams {
  val rvs1 = Decoupled(new VectorReadReq)
  val rvs2 = Decoupled(new VectorReadReq)
  val rvd  = Decoupled(new VectorReadReq)
  val rvm  = Decoupled(new VectorReadReq)
  val pipe_write_req = new VectorPipeWriteReqIO(pipe_depth + acc_delay * (vLen/dLen) * 8)
  val batch_read_vs2 = Output(Bool())
  val writeback_op = Decoupled(new BDotSequencerWritebackControl)
}

class BDotSequencer(pipe_depth: Int, acc_delay: Int)(implicit p: Parameters) extends Sequencer[BDotSequencerControl]()(p) with HasVectorParams with HasCoreParameters {

  val bdot_insns = vParams.bdotInsns

  def accepts(inst: VectorIssueInst) = !inst.vmu && new VectorDecoder(inst, bdot_insns, Nil).matched

  val io = IO(new BDotSequencerIO(pipe_depth, acc_delay))

  // Current instruction
  val valid = RegInit(false.B)
  val busy = RegInit(false.B)
  val inst = Reg(new BackendIssueInst)
  
  val read_eidx = RegInit(0.U(log2Ceil(maxVLMax).W))
  val write_eidx = RegInit(0.U(log2Ceil(maxVLMax).W))
  val head = Reg(Bool())

  val wvd_mask = Reg(UInt(egsTotal.W))
  val rvs1_mask = Reg(UInt(egsTotal.W))
  val rvs2_mask = Reg(UInt(egsTotal.W))
  val rvd_mask = Reg(UInt(egsTotal.W))
  val rvm_mask = Reg(UInt(egsTotal.W))

  val fp = Reg(Bool())
  val altfmt = Reg(Bool())
  val signed = Reg(Bool())
  val batched = Reg(Bool())
  val widen = Reg(UInt(2.W))

  val emul = Reg(UInt(2.W))
  val in_eew = Reg(UInt(2.W))
  val acc_eew = in_eew + widen
  val renvm = Reg(Bool())
  val vl = Reg(UInt((1+log2Ceil(maxVLMax)).W))

  val src_eidx_max = (vLen / dLen).U << emul
  val acc_eidx_max = Mux(batched, 1.U << (acc_eew - (log2Ceil(dLen) - 6).U), 1.U)
  val read_eidx_max = Mux(src_eidx_max > acc_eidx_max, src_eidx_max, acc_eidx_max)

  val next_read_eidx = read_eidx +& 1.U
  val next_write_eidx = write_eidx +& 1.U
  val elements_width = (dLen / 8).U >> in_eew
  val next_vl = Mux(vl >= elements_width, vl - elements_width, 0.U)

  val read_tail = next_read_eidx === read_eidx_max
  val write_tail = next_write_eidx === acc_eidx_max

  io.dis.ready := !busy /*(!valid || (tail && io.iss.fire))*/ && !io.dis_stall && io.iss.ready // TODO: Optimize

  // TODO: Should be write_tail, or optimize better
  val tail_pipe = Pipe(io.iss.fire && write_tail, 0.U, pipe_depth + acc_delay)

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

    val dis_vd_arch_mask  = get_arch_mask(dis_inst.rd, 0.U) // TODO: Multi-vd outputs
    val dis_vs1_arch_mask = get_arch_mask(dis_inst.rs1, Mux(dis_ctrl.bool(BDotBatched), 0.U, dis_inst.emul))
    val dis_vs2_arch_mask = get_arch_mask(dis_inst.rs2, Mux(dis_ctrl.bool(BDotBatched), 3.U, dis_inst.emul))

    valid := true.B
    busy := true.B
    inst := io.dis.bits

    renvm := false.B

    wvd_mask      := FillInterleaved(egsPerVReg, dis_vd_arch_mask)
    rvs1_mask     := FillInterleaved(egsPerVReg, dis_vs1_arch_mask)
    rvs2_mask     := FillInterleaved(egsPerVReg, dis_vs2_arch_mask)
    rvd_mask      := FillInterleaved(egsPerVReg, dis_vd_arch_mask)
    rvm_mask      := Mux(renvm, ~(0.U(egsPerVReg.W)), 0.U)

    emul := Mux(batched, 0.U, dis_inst.emul)
    in_eew := dis_inst.vconfig.vtype.vsew
    vl := dis_inst.vconfig.vl
    
    head := true.B
  } .elsewhen (io.iss.fire) {
    valid := !read_tail
    head := false.B
  }

  when (tail_pipe.valid) {
    busy := false.B
  }

  io.vat := inst.vat
  io.seq_hazard.valid := busy // TODO: Make this chain well between instructions
  io.seq_hazard.bits.rintent := hazardMultiply(rvs1_mask | rvs2_mask | rvd_mask | rvm_mask)
  io.seq_hazard.bits.wintent := hazardMultiply(wvd_mask)
  io.seq_hazard.bits.vat := inst.vat

  val wvd_eg = getEgId(inst.rd, write_eidx, acc_eew, inst.writes_mask)

  val vs1_read_oh = UIntToOH(io.rvs1.bits.eg)
  val vs2_read_oh = UIntToOH(io.rvs2.bits.eg)
  val vd_read_oh  = UIntToOH(io.rvd.bits.eg)
  val vm_read_oh  = Mux(renvm, UIntToOH(io.rvm.bits.eg), 0.U)
  val vd_write_oh = UIntToOH(wvd_eg)

  val raw_hazard = ((vs1_read_oh | vs2_read_oh | vd_read_oh | vm_read_oh) & io.older_writes) =/= 0.U
  val waw_hazard = (vd_write_oh & io.older_writes) =/= 0.U
  val war_hazard = (vd_write_oh & io.older_reads) =/= 0.U
  // val data_hazard = raw_hazard || waw_hazard || war_hazard

  val oldest = inst.vat === io.vat_head

  io.rvs1.valid := valid
  io.rvs1.bits.eg := getEgId(inst.rs1, read_eidx, in_eew, false.B)
  io.rvs2.valid := valid
  io.rvs2.bits.eg := getEgId(inst.rs2, read_eidx, in_eew, false.B)
  io.rvd.valid := valid
  io.rvd.bits.eg := getEgId(inst.rd, read_eidx, acc_eew, false.B)
  io.rvm.valid := valid && renvm
  io.rvm.bits.eg := getEgId(0.U, read_eidx, 0.U, true.B)
  io.batch_read_vs2 := batched && valid

  io.rvs1.bits.oldest := oldest
  io.rvs2.bits.oldest := oldest
  io.rvd.bits.oldest := oldest
  io.rvm.bits.oldest := oldest

  val exu_scheduler = Module(new PipeScheduler(1, pipe_depth + acc_delay * (vLen/dLen) * 8))
  exu_scheduler.io.reqs(0).request := valid
  exu_scheduler.io.reqs(0).fire := io.iss.fire
  exu_scheduler.io.reqs(0).depth := pipe_depth.U + acc_delay.U * ((vLen/dLen).U << emul)

  io.pipe_write_req.request := valid && read_tail && exu_scheduler.io.reqs(0).available
  io.pipe_write_req.bank_sel := (if (vrfBankBits == 0) 1.U else UIntToOH(wvd_eg(vrfBankBits,1)))
  io.pipe_write_req.pipe_depth := (pipe_depth + acc_delay).U
  io.pipe_write_req.oldest := oldest
  io.pipe_write_req.fire := io.iss.fire

  when (io.iss.fire) {
    when (!read_tail) {
      read_eidx := next_read_eidx
      vl := next_vl
    } .otherwise {
      read_eidx := 0.U
    }
  }

  when (io.writeback_op.fire) {
    when (!write_tail) {
      write_eidx := next_write_eidx
    } .otherwise {
      write_eidx := 0.U
    }
  }

  io.iss.valid := (valid &&
    !raw_hazard &&
    io.rvs1.ready &&
    io.rvs2.ready &&
    io.rvd.ready &&
    !(renvm && !io.rvm.ready)
  )
  io.iss.bits.base_eg := wvd_eg
  io.iss.bits.fp := fp
  io.iss.bits.altfmt := altfmt
  io.iss.bits.signed := signed
  io.iss.bits.batched := batched
  io.iss.bits.emul := emul
  io.iss.bits.in_eew := in_eew
  io.iss.bits.acc_eew := acc_eew
  // io.iss.bits.set_acc := read_eidx === 0.U
  io.iss.bits.head := head
  io.iss.bits.src_valid := read_eidx < src_eidx_max
  io.iss.bits.acc_valid := read_eidx < acc_eidx_max
  io.iss.bits.input_tail := next_read_eidx === src_eidx_max
  io.writeback_op.valid := (valid &&
    !(waw_hazard || war_hazard) &&
    io.pipe_write_req.available &&
    exu_scheduler.io.reqs(0).available
  )
  io.iss.bits.vl := vl

  io.busy := busy
  io.head := head
}