// Copyright lowRISC contributors.
// Copyright 2018 ETH Zurich and University of Bologna, see also CREDITS.md.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

/**
 * Execution stage
 *
 * Execution block: Hosts ALU and MUL/DIV unit
 */
module ibex_ex_block #(
    parameter ibex_pkg::rv32m_e RV32M           = ibex_pkg::RV32MFast,
    parameter ibex_pkg::rv32b_e RV32B           = ibex_pkg::RV32BNone,
    parameter bit               BranchTargetALU = 0
) (
    input  logic                  clk_i,
    input  logic                  rst_ni,

    // ALU
    input  ibex_pkg::alu_op_e     alu_operator_i,
    input  logic [31:0]           alu_operand_a_i,
    input  logic [31:0]           alu_operand_b_i,
    input  logic                  alu_instr_first_cycle_i,

  // Branch Target ALU
  // All of these signals are unusued when BranchTargetALU == 0
  // input  logic [31:0]           bt_a_operand_i,
  // input  logic [31:0]           bt_b_operand_i,

    // Multiplier/Divider
    input  ibex_pkg::md_op_e      multdiv_operator_i,
    input  logic                  mult_en_i,             // dynamic enable signal, for FSM control
    input  logic                  div_en_i,              // dynamic enable signal, for FSM control
    input  logic                  mult_sel_i,            // static decoder output, for data muxes
    input  logic                  div_sel_i,             // static decoder output, for data muxes
    input  logic  [1:0]           multdiv_signed_mode_i,
    input  logic [31:0]           multdiv_operand_a_i,
    input  logic [31:0]           multdiv_operand_b_i,
    input  logic                  multdiv_ready_id_i,
    input  logic                  data_ind_timing_i,

    // intermediate val reg
    output logic [1:0]            imd_val_we_o,
    output logic [33:0]           imd_val_d_o[2],
    input  logic [33:0]           imd_val_q_i[2],

  // Outputs
  output logic [31:0]           alu_adder_result_ex_o, // to LSU
  output logic [31:0]           result_ex_o,
  // output logic [31:0]           branch_target_o,       // to IF
  // output logic                  branch_decision_o,     // to ID

    output logic                  ex_valid_o             // EX has valid output
);

  import ibex_pkg::*;

  logic [31:0] alu_result, multdiv_result;

  logic [32:0] multdiv_alu_operand_b, multdiv_alu_operand_a;
  logic        multdiv_valid;
  logic        multdiv_sel;
  logic [31:0] alu_imd_val_q[2];
  logic [31:0] alu_imd_val_d[2];
  logic [ 1:0] alu_imd_val_we;
  logic [33:0] multdiv_imd_val_d[2];
  logic [ 1:0] multdiv_imd_val_we;

  /*
    The multdiv_i output is never selected if RV32M=RV32MNone
    At synthesis time, all the combinational and sequential logic
    from the multdiv_i module are eliminated
  */
  if (RV32M != RV32MNone) begin : gen_multdiv_m
    assign multdiv_sel = mult_sel_i | div_sel_i;
  end else begin : gen_multdiv_no_m
    assign multdiv_sel = 1'b0;
  end

  // Intermediate Value Register Mux
  assign imd_val_d_o[0] = multdiv_sel ? multdiv_imd_val_d[0] : {2'b0, alu_imd_val_d[0]};
  assign imd_val_d_o[1] = multdiv_sel ? multdiv_imd_val_d[1] : {2'b0, alu_imd_val_d[1]};
  assign imd_val_we_o   = multdiv_sel ? multdiv_imd_val_we : alu_imd_val_we;

  assign alu_imd_val_q = '{imd_val_q_i[0][31:0], imd_val_q_i[1][31:0]};

  assign result_ex_o  = multdiv_sel ? multdiv_result : alu_result;

  /////////
  // ALU //
  /////////

  ibex_alu #(
    .RV32B(RV32B)
  ) alu_i (
    .operator_i         (alu_operator_i),
    .operand_a_i        (alu_operand_a_i),
    .operand_b_i        (alu_operand_b_i),
    .instr_first_cycle_i(alu_instr_first_cycle_i),
    .imd_val_q_i        (alu_imd_val_q),
    .imd_val_we_o       (alu_imd_val_we),
    .imd_val_d_o        (alu_imd_val_d),
    .multdiv_operand_a_i(multdiv_alu_operand_a),
    .multdiv_operand_b_i(multdiv_alu_operand_b),
    .multdiv_sel_i      (multdiv_sel),
    .adder_result_o     (alu_adder_result_ex_o),
    .result_o           (alu_result)
  );

  // Multiplier's ALU
  /*
Previously mult/div units was using the main alu for it’s to take addition results and is equal results. Now a separate adder/substracter is implemented in ibex_ex_block to offload these tasks from main alu
Inputs: alu_operand_a_i alu_operand_b_i
Outputs: mul_adder_result_o, mul_alu_is_equal_result
*/

  logic [31:0] mul_adder_result_o;
  logic [33:0] mul_adder_result_ext_o;
  logic [31:0] mul_adder_result;
  logic        mul_alu_is_equal_result;

  assign mul_adder_result_ext_o  = $unsigned(alu_operand_a_i) + $unsigned(alu_operand_b_i);  //this is the seperate adder for multiplier. now it is not sharing main alu.

  assign mul_adder_result        = mul_adder_result_ext_o[32:1];

  assign mul_adder_result_o      = mul_adder_result;

  assign mul_alu_is_equal_result = alu_operand_a_i - alu_operand_b_i;  //this is the seperate subtracter for multiplier. now it is not sharing main alu.

  ////////////////
  // Multiplier //
  ////////////////

  if (RV32M == RV32MSlow) begin : gen_multdiv_slow
    ibex_multdiv_slow multdiv_i (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .mult_en_i         (mult_en_i),
      .div_en_i          (div_en_i),
      .mult_sel_i        (mult_sel_i),
      .div_sel_i         (div_sel_i),
      .operator_i        (multdiv_operator_i),
      .signed_mode_i     (multdiv_signed_mode_i),
      .op_a_i            (multdiv_operand_a_i),
      .op_b_i            (multdiv_operand_b_i),
      .alu_adder_ext_i   (mul_adder_result_ext_o),  //previously this was provided by main alu. now calculated by the multipliers seperate alu in ibex_ex_block.
      .alu_adder_i       (mul_adder_result_o),
      .equal_to_zero_i   (mul_alu_is_equal_result),
      .data_ind_timing_i (data_ind_timing_i),
      .valid_o           (multdiv_valid),
      .alu_operand_a_o   (multdiv_alu_operand_a),
      .alu_operand_b_o   (multdiv_alu_operand_b),
      .imd_val_q_i       (imd_val_q_i),
      .imd_val_d_o       (multdiv_imd_val_d),
      .imd_val_we_o      (multdiv_imd_val_we),
      .multdiv_ready_id_i(multdiv_ready_id_i),
      .multdiv_result_o  (multdiv_result)
    );
  end else if (RV32M == RV32MFast || RV32M == RV32MSingleCycle) begin : gen_multdiv_fast
    ibex_multdiv_fast #(
      .RV32M(RV32M)
    ) multdiv_i (
      .clk_i             (clk_i),
      .rst_ni            (rst_ni),
      .mult_en_i         (mult_en_i),
      .div_en_i          (div_en_i),
      .mult_sel_i        (mult_sel_i),
      .div_sel_i         (div_sel_i),
      .operator_i        (multdiv_operator_i),
      .signed_mode_i     (multdiv_signed_mode_i),
      .op_a_i            (multdiv_operand_a_i),
      .op_b_i            (multdiv_operand_b_i),
      .alu_operand_a_o   (multdiv_alu_operand_a),
      .alu_operand_b_o   (multdiv_alu_operand_b),
      .alu_adder_ext_i   (mul_adder_result_ext_o),
      .alu_adder_i       (mul_adder_result_o),
      .equal_to_zero_i   (mul_alu_is_equal_result),
      .data_ind_timing_i (data_ind_timing_i),
      .imd_val_q_i       (imd_val_q_i),
      .imd_val_d_o       (multdiv_imd_val_d),
      .imd_val_we_o      (multdiv_imd_val_we),
      .multdiv_ready_id_i(multdiv_ready_id_i),
      .valid_o           (multdiv_valid),
      .multdiv_result_o  (multdiv_result)
    );
  end

  // Multiplier/divider may require multiple cycles. The ALU output is valid in the same cycle
  // unless the intermediate result register is being written (which indicates this isn't the
  // final cycle of ALU operation).
  assign ex_valid_o = multdiv_sel ? multdiv_valid : ~(|alu_imd_val_we);

endmodule
