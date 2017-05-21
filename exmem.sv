module exmem

(

	output [63:0] target,
	output [63:0] ALUresult,
	output [63:0] valB,
	output [4:0] rd,
	output [6:0] opcode,
	output [2:0] funct3,
	output [6:0] funct7,
	output [3:0] num_bytes,
	output is_branch,
	
	output [31:0] instruction,
	output [63:0] pc,
	output noop,

	input hit,
	input clk,
	input [63:0] next_target,
	input [63:0] next_ALUresult,
	input [63:0] next_valB,
	input [4:0] next_rd,
	input [6:0] next_opcode,
	input [2:0] next_funct3,
	input [6:0] next_funct7,
	input [3:0] next_num_bytes,
	input next_is_branch,

	input [31:0] next_instruction,
	input [63:0] next_pc,
	input next_noop

);


	always_ff @ (posedge clk) begin

		if (hit == 1) begin

			target <= next_target;
			ALUresult <= next_ALUresult;
			valB <= next_valB;
			rd <= next_rd;
			opcode <= next_opcode;
			funct3 <= next_funct3;
			funct7 <= next_funct7;
			num_bytes <= next_num_bytes;
			is_branch <= next_is_branch;

			instruction <= next_instruction;

			pc <= next_pc;
			noop <= next_noop;
		end

	end


endmodule
