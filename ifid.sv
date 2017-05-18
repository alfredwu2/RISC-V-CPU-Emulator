module ifid

(
	
	output [63:0] pcplus4,
	output [31:0] instruction,
	output [63:0] pc,
	output noop,

	input hit,
	input clk,
	input stall,
	input stall_ecall,

	input [63:0] next_pcplus4,
	input [31:0] next_instruction,
	input [63:0] next_pc,
	input next_noop

);


	always_ff @ (posedge clk) begin

		if (hit == 1) begin

			if (stall == 1) begin

			/*
			end else if (stall_ecall == 1) begin

				noop <= 1;
			*/
			
			end else begin

				pcplus4 <= next_pcplus4;
				instruction <= next_instruction;
				pc <= next_pc;
				noop <= next_noop;

			end
		end

	end










endmodule
