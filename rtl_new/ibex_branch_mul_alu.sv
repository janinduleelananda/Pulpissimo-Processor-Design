

/*
 * Execution stage

This module is implemented as alu for the branch decision handling. It offloads the main alu s tasks of branch decision taking and branch target calculating. 
in ibex core it’s implemented as branch_mul_alu_i. 
inputs: 
clk_i,  rst_ni
ALU signal from ID stage(these are used to decide the operation and to take operands)
	alu_operator_ex
	alu_operand_a_ex
	alu_operand_b_ex
outputs:
	b_mul_alu_adder_result_o (this is used to get branch target)
	alu_cmp_result (this is used to take branch decisions)

*/
 module ibex_branch_mul_alu #(
    parameter ibex_pkg::rv32b_e RV32B = ibex_pkg::RV32BNone
  ) (
    input  logic                  clk_i,
    input  logic                  rst_ni,

    // ALU signal from ID stage
    input  ibex_pkg::alu_op_e     b_mul_alu_operator_i,
    input  logic [31:0]           b_mul_alu_operand_a_i,
    input  logic [31:0]           b_mul_alu_operand_b_i,

    // Outputs
    output logic [31:0]           b_mul_alu_adder_result_o,  //previously it was handled by main alu. 
    output logic                  comparison_result_o
  );
    // Define internal signals
    logic [31:0] alu_adder_result;

    // ALU addition operation
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        alu_adder_result <= 32'b0; // Reset the result on reset
      end else begin
        // Perform the addition operation
        alu_adder_result <= b_mul_alu_operand_a_i + b_mul_alu_operand_b_i; //now this is mainly used to calulate branch target. this value is assigned to branch_target_ex in ibex_core to proceed with taken branches.
      end
    end

    // Assign the ALU result to the output port
    b_mul_alu_adder_result_o = alu_adder_result;

    ////////////////
    // Comparison //
    ////////////////

    logic is_equal;
    logic is_greater_equal;  // handles both signed and unsigned forms
  
    assign is_equal = (b_mul_alu_operand_a_i == b_mul_alu_operand_b_i);
  
    // Is greater equal
    assign is_greater_equal = (b_mul_alu_operand_a_i >= b_mul_alu_operand_b_i);
  
    // generate comparison result
    logic cmp_result;
  
    always_comb begin
      unique case (b_mul_alu_operator_i)
        ALU_EQ:             cmp_result =  is_equal;
        ALU_NE:             cmp_result = ~is_equal;
        ALU_GE,   ALU_GEU,
        ALU_MAX,  ALU_MAXU: cmp_result = is_greater_equal; // RV32B only
        ALU_LT,   ALU_LTU,
        ALU_MIN,  ALU_MINU, //RV32B only
        ALU_SLT,  ALU_SLTU: cmp_result = ~is_greater_equal;
  
        default: cmp_result = is_equal;
      endcase
    end
  
    assign comparison_result_o = cmp_result;

  endmodule
