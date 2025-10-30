package saturn.exu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import chisel3.util.experimental.decode._
import saturn.common._
import saturn.backend._
import hardfloat._
import scala.math._

class IntegerDotPipe(pipe_depth: Int, acc_delay: Int, input_width: Int, output_width: Int)(implicit p: Parameters) extends CoreModule()(p) with HasVectorParams {
  val io = IO(new Bundle {
    val set_acc = Input(Bool())
    val acc_init = Input(UInt(output_width.W))
    val valid = Input(Bool())
    val signed_a = Input(Bool())
    val signed_b = Input(Bool())
    val in_a = Input(UInt(dLen.W))
    val in_b = Input(UInt(dLen.W))
    val vl = Input(UInt((1+log2Ceil(maxVLMax)).W))
    val out = Output(UInt(output_width.W))
  })

  val acc = Reg(UInt(output_width.W))

  val a = io.in_a.asTypeOf(Vec(dLen / input_width, UInt(input_width.W))).zipWithIndex.map { case (e, i) => Mux(io.vl > i.U, e, 0.U) }
  val b = io.in_b.asTypeOf(Vec(dLen / input_width, UInt(input_width.W))).zipWithIndex.map { case (e, i) => Mux(io.vl > i.U, e, 0.U) }

  val a_signs = a.map { e => io.signed_a && e(input_width - 1) }
  val b_signs = b.map { e => io.signed_b && e(input_width - 1) }
  val a_unsigned = a.zip(a_signs).map { case (e, signed) => Mux(signed, ~e + 1.U, e) }
  val b_unsigned = b.zip(b_signs).map { case (e, signed) => Mux(signed, ~e + 1.U, e) }

  val prods_unsigned = a_unsigned.zip(b_unsigned).map { case (x, y) => x * y }
  val prod_signs = a_signs.zip(b_signs).map { case (x, y) => x ^ y }
  val prods_short = prods_unsigned.zip(prod_signs).map { case (e, signed) => Mux(signed, ~e + 1.U, e) }
  val prods = prods_short.zip(prod_signs).map { case (e, sign) => Fill(output_width - e.getWidth, sign) ## e }
  val sum = prods.foldLeft(0.U) { (x, y) => x + y }
  
  val sum_pipe = Pipe(io.valid, sum, pipe_depth)

  when (io.set_acc) {
    acc := io.acc_init
  } .elsewhen (sum_pipe.valid) {
    acc := sum_pipe.bits +& acc
  }

  io.out := acc
}

class BDotSequencerControl(implicit p: Parameters) extends CoreBundle()(p) with HasVectorParams {
  val base_eg = Input(UInt(log2Ceil(32 * vLen / dLen).W))
  val fp = Input(Bool())
  val altfmt = Input(Bool())
  val signed = Input(Bool())
  val batched = Input(Bool())
  val emul = Input(UInt(2.W))
  val in_eew = Input(UInt(2.W))
  val acc_eew = Input(UInt(2.W))
  val set_acc = Input(Bool())
  val writeback = Input(Bool())
  val vl = Input(UInt((1+log2Ceil(maxVLMax)).W))
}

class BDotUnit(pipe_depth: Int, acc_delay: Int)(implicit p: Parameters) extends CoreModule()(p) with HasVectorParams {
  val total_depth = pipe_depth + (vLen/dLen)
  
  val io = IO(new Bundle {
    val op = Flipped(Decoupled(new BDotSequencerControl))
    val rvs1_data = Input(UInt(dLen.W))
    val rvs2_data = Input(UInt(dLen.W))
    val rvd_data = Input(UInt(dLen.W))
    val rvm_data = Input(UInt(dLen.W))
    val batch_vs2_data = Input(Vec(vParams.vrfBanking, UInt(dLen.W)))
    val write = Output(Valid(new VectorWrite(dLen)))
  })

  dontTouch(io.batch_vs2_data)

  val out = Wire(UInt(dLen.W))
  val ready = RegInit(true.B)

  val int8_out = Wire(Vec(4, UInt(32.W)))
  for (i <- 0 until 4) {
    val int8_pipe = Module(new IntegerDotPipe(pipe_depth, acc_delay, 8, 32))
    int8_pipe.io.set_acc := io.op.fire && !io.op.bits.fp && io.op.bits.in_eew === 0.U && io.op.bits.set_acc
    int8_pipe.io.acc_init := io.rvd_data((i + 1) * 32 - 1, i * 32)
    int8_pipe.io.valid := io.op.fire && !io.op.bits.fp && io.op.bits.in_eew === 0.U
    int8_pipe.io.signed_a := io.op.bits.altfmt
    int8_pipe.io.signed_b := io.op.bits.signed
    int8_pipe.io.in_a := io.rvs1_data
    int8_pipe.io.in_b := Mux(io.op.bits.batched, io.batch_vs2_data(i), if (i == 0) io.rvs2_data else 0.U)
    int8_pipe.io.vl := io.op.bits.vl
    int8_out(i) := int8_pipe.io.out
  }

  out := int8_out.asUInt

  val write_eg_pipe = Pipe(io.op.fire && io.op.bits.writeback, io.op.bits.base_eg, pipe_depth + acc_delay)
  val ready_pipe = Pipe(io.op.fire, 0.U, acc_delay)

  when (io.op.fire) {
    ready := false.B
  } .elsewhen (ready_pipe.valid) {
    ready := true.B
  }

  io.op.ready := ready

  // when (io.op.fire) {
  //   pipe_pos := 0.U
  // } .elsewhen (tail) {
  //   pipe_pos := 0.U
  //   ready := true.B
  // } .elsewhen(!ready) {
  //   pipe_pos := pipe_pos_next
  // }

  io.write.valid := write_eg_pipe.valid
  io.write.bits.eg := write_eg_pipe.bits
  io.write.bits.data := out
  io.write.bits.mask := Mux(io.op.bits.batched, ~(0.U(dLen.W)), "hFFFFFFFF".U(dLen.W))
}