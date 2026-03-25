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

class AccumulatorSelectPipe(depth: Int) extends Module {
  assert(depth > 0)

  val io = IO(new Bundle {
    val valid = Input(Bool())
    val acc_in = Input(UInt(5.W))
    val acc_out = Output(UInt(5.W))
    val in_flight = Output(UInt(32.W))
  })

  val data_regs = Reg(Vec(depth + 1, UInt(5.W)))
  val valid_regs = Reg(Vec(depth + 1, Bool()))

  data_regs(0) := io.acc_in
  valid_regs(0) := io.valid

  for (i <- 1 until depth + 1) {
    data_regs(i) := data_regs(i - 1)
    valid_regs(i) := valid_regs(i - 1)
  }

  io.acc_out := data_regs(depth - 1)
  io.in_flight := (0 until depth).map(i => Mux(valid_regs(i), UIntToOH(data_regs(i)), 0.U(32.W))).foldLeft(0.U)(_ | _)
}

class DotPipe(output_width: Int)(implicit p: Parameters) extends CoreModule()(p) with HasVectorParams {
  val io = IO(new Bundle {
    val valid = Input(Bool())
    val signed_a = Input(Bool())
    val signed_b = Input(Bool())
    val in_a = Input(UInt(dLen.W))
    val in_b = Input(UInt(dLen.W))
    val acc = Input(UInt(output_width.W))
    val vl = Input(UInt((1+log2Ceil(maxVLMax)).W))
    val out = Output(UInt(output_width.W))
    val out_en = Output(Bool())
  })
}

class IntegerDotPipe(pipe_depth: Int, acc_delay: Int, input_width: Int, output_width: Int)(implicit p: Parameters) extends DotPipe(output_width)(p) {

  val a = io.in_a.asTypeOf(Vec(dLen / input_width, UInt(input_width.W))).zipWithIndex.map { case (e, i) => Mux(io.vl > i.U || true.B, e, 0.U) }
  val b = io.in_b.asTypeOf(Vec(dLen / input_width, UInt(input_width.W))).zipWithIndex.map { case (e, i) => Mux(io.vl > i.U || true.B, e, 0.U) }

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
  val acc_pipe = Pipe(sum_pipe.valid, sum_pipe.bits +& io.acc, acc_delay - 1)

  io.out := acc_pipe.bits
  io.out_en := acc_pipe.valid
}

class BDotSequencerControl(implicit p: Parameters) extends CoreBundle()(p) with HasVectorParams {
  val fp = Input(Bool())
  val altfmt = Input(Bool())
  val signed = Input(Bool())
  val batched = Input(Bool())
  val in_eew = Input(UInt(2.W))
  val acc_sel = Input(UInt(5.W))
  val vl = Input(UInt((1+log2Ceil(maxVLMax)).W))
}

class BDotWBSequencerControl(implicit p: Parameters) extends CoreBundle()(p) with HasVectorParams {
  val base_eg = Input(UInt(log2Ceil(32 * vLen / dLen).W))
  val set_acc = Input(Bool())
  val set_acc_zero = Input(Bool())
  val set_acc_bc = Input(Bool())
  val writeback = Input(Bool())
  val in_eew = Input(UInt(2.W))
  val acc_sel = Input(UInt(5.W))
}

class BDotUnit(pipe_depth: Int, acc_delay: Int)(implicit p: Parameters) extends CoreModule()(p) with HasVectorParams {
  
  val io = IO(new Bundle {
    val op = Flipped(Decoupled(new BDotSequencerControl))
    val wb_op = Flipped(Decoupled(new BDotWBSequencerControl))
    val in_flight = Output(UInt(32.W))
    val rvs1_data = Input(UInt(dLen.W))
    val rvs2_data = Input(UInt(dLen.W))
    val rvd_data = Input(UInt(dLen.W))
    val rvm_data = Input(UInt(dLen.W))
    val batch_vs2_data = Input(Vec(vParams.vrfBanking, UInt(dLen.W)))
    val write = Output(Valid(new VectorWrite(dLen)))
  })

  val ready = RegInit(true.B)

  val accumulator = Reg(Vec(32, Vec(8, UInt(32.W))))

  val int8_out = Wire(Vec(8, UInt(32.W)))
  val int8_out_en = Wire(Bool())
  for (i <- 0 until 8) {
    val int8_pipe = Module(new IntegerDotPipe(pipe_depth, acc_delay, 8, 32))
    int8_pipe.io.valid := io.op.fire && !io.op.bits.fp && io.op.bits.in_eew === 0.U
    int8_pipe.io.signed_a := io.op.bits.altfmt
    int8_pipe.io.signed_b := io.op.bits.signed
    int8_pipe.io.in_a := io.rvs1_data
    int8_pipe.io.in_b := Mux(io.op.bits.batched, io.batch_vs2_data(i), if (i == 0) io.rvs2_data else 0.U)
    val acc_sel_pipe = Pipe(io.op.fire, io.op.bits.acc_sel, pipe_depth)
    int8_pipe.io.acc := accumulator(acc_sel_pipe.bits)(i.U)
    int8_pipe.io.vl := io.op.bits.vl
    int8_out(i) := int8_pipe.io.out
    if (i == 0) {
      int8_out_en := int8_pipe.io.out_en
    }
  }

  val ready_pipe = Pipe(io.op.fire, 0.U, acc_delay - 1)

  val acc_eidx = RegInit(0.U(3.W))
  val acc_write_eidx = RegInit(0.U(3.W))
  val acc_sel_pipe = Module(new AccumulatorSelectPipe(pipe_depth + acc_delay - 1))
  acc_sel_pipe.io.valid := io.op.fire
  acc_sel_pipe.io.acc_in := io.op.bits.acc_sel

  when (int8_out_en) {
    for (i <- 0 until 8) {
      accumulator(acc_sel_pipe.io.acc_out)(i) := int8_out(i) 
    }
  }

  when (io.op.fire) {
    ready := (acc_delay == 1).B
  } .elsewhen (ready_pipe.valid) {
    ready := true.B
  }

  when (io.wb_op.fire) {
    when (io.wb_op.bits.set_acc || io.wb_op.bits.writeback) {
      acc_eidx := Mux(io.wb_op.bits.set_acc && io.wb_op.bits.set_acc_zero, acc_eidx, acc_eidx + Mux(io.wb_op.bits.in_eew === 3.U, (dLen/64).U, (dLen/32).U))
    }
    when (io.wb_op.bits.set_acc) {
      for (x <- 0 until 32) {
        when (io.wb_op.bits.acc_sel === x.U || io.wb_op.bits.set_acc_bc) {
          for (i <- 0 until 8) {
            when (io.wb_op.bits.set_acc_zero) { // Zero
              accumulator(x.U)(i.U) := 0.U
            } .elsewhen (io.wb_op.bits.in_eew === 3.U) { // 64-bit

            } .otherwise { // 32-bit
              val group = (i / (dLen/32)) * (dLen/32)
              val idx = i % (dLen/32)
              when (acc_eidx === group.U) {
                accumulator(x.U)(i.U) := io.rvd_data((idx + 1) * 32 - 1, idx * 32)
              }
            }
          }
        }
      }
    }
  }

  val out = Wire(Vec(dLen / 32, UInt(32.W)))

  when (io.wb_op.bits.in_eew === 3.U) {
    for (i <- 0 until dLen/64) {
      out(2 * i) := accumulator(io.wb_op.bits.acc_sel)(acc_eidx + i.U)
      out(2 * i + 1) := accumulator(io.wb_op.bits.acc_sel + 1.U)(acc_eidx + i.U)
    }
  } .otherwise {
    for (i <- 0 until dLen/32) {
      out(i) := accumulator(io.wb_op.bits.acc_sel)(acc_eidx + i.U)
    }
  }

  io.op.ready := ready
  io.wb_op.ready := true.B // !(acc_sel_pipe.io.in_flight(io.wb_op.bits.acc_sel) || (io.wb_op.bits.set_acc_bc && acc_sel_pipe.io.in_flight.orR))
  io.in_flight := acc_sel_pipe.io.in_flight

  io.write.valid := io.wb_op.fire && io.wb_op.bits.writeback
  io.write.bits.eg := io.wb_op.bits.base_eg
  io.write.bits.data := out.asUInt
  io.write.bits.mask := ~(0.U(dLen.W))
}