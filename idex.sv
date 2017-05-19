module idex

(
	output [63:0] pcplus4,
	output [63:0] valA,
	output [63:0] valB,
	output [4:0] rd,
	output [6:0] opcode,
	output [2:0] funct3,
	output [6:0] funct7,
	output [63:0] imm,
	output [3:0] num_bytes,

	output [31:0] instruction,
	output [63:0] pc,
	output noop,

	input hit,
	input clk,
	input stall,
	input [63:0] next_pcplus4,
	input [63:0] next_valA,
	input [63:0] next_valB,
	input [4:0] next_rd,
	input [6:0] next_opcode,
	input [2:0] next_funct3,
	input [6:0] next_funct7,
	input [63:0] next_imm,
	input [3:0] next_num_bytes,

	input [31:0] next_instruction,
	input [63:0] next_pc,
	input next_noop

);


	always_ff @ (posedge clk) begin

		if (hit == 1) begin

			if (stall == 1) begin
				noop <= 1;
			end else begin

				pcplus4 <= next_pcplus4;
				valA <= next_valA;
				valB <= next_valB;
				rd <= next_rd;
				opcode <= next_opcode;
				funct3 <= next_funct3;
				funct7 <= next_funct7;
				imm <= next_imm;
				num_bytes <= next_num_bytes;

				instruction <= next_instruction;

				pc <= next_pc;

				noop <= next_noop;

			end

		end

	end


endmodule
