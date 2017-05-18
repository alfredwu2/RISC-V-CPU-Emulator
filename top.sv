`include "Sysbus.defs"

module top



#(
	BUS_DATA_WIDTH = 64,
	BUS_TAG_WIDTH = 13
)
(
	input  clk,
	reset,

	// 64-bit addresses of the program entry point and initial stack pointer
	input  [63:0] entry,
	input  [63:0] stackptr,
	input  [63:0] satp,
	  
	// interface to connect to the bus
	output bus_reqcyc,
	output bus_respack,
	output [BUS_DATA_WIDTH-1:0] bus_req,
	output [BUS_TAG_WIDTH-1:0] bus_reqtag,
	input  bus_respcyc,
	input  bus_reqack,
	input  [BUS_DATA_WIDTH-1:0] bus_resp,
	input  [BUS_TAG_WIDTH-1:0] bus_resptag
);

	logic [63:0] next_pc;
	logic [63:0] next_pcplus4;

	logic [63:0] pc;
	logic [63:0] pcplus4;


	logic [31:0] instruction;

	// register file
	logic [63:0] valA;
	logic [63:0] valB;
	
	logic [63:0] data;
	logic [4:0] rd;


	logic [63:0] target;
	logic [63:0] ALUresult;

	logic hit;

	logic is_branch;

	logic rd_en;
	logic wr_en;

	logic [6:0] opcode;
	logic [2:0] funct3;
	logic [6:0] funct7;

	logic [63:0] imm;

	logic [3:0] num_bytes;

	logic noop;


	// IFID
	logic [63:0] pcplus4_ifid;
	logic [63:0] pc_ifid;
	logic [31:0] instruction_ifid;
	logic noop_ifid;


	logic stall;


	// IDEX
	// TODO
	logic [63:0] pc_idex;
	logic [31:0] instruction_idex;
	logic [63:0] pcplus4_idex;
	logic [63:0] valA_idex;
	logic [63:0] valB_idex;
	logic [4:0] rd_idex;
	logic [6:0] opcode_idex;
	logic [2:0] funct3_idex;
	logic [6:0] funct7_idex;
	logic [63:0] imm_idex;
	logic [3:0] num_bytes_idex;
	logic noop_idex;

	// EXMEM
	logic [63:0] pc_exmem;
	logic [31:0] instruction_exmem;
	logic [63:0] target_exmem;
	logic [63:0] ALUresult_exmem;
	logic [63:0] valB_exmem;
	logic [4:0] rd_exmem;
	logic [6:0] opcode_exmem;
	logic [2:0] funct3_exmem;
	logic [6:0] funct7_exmem;
	logic [3:0] num_bytes_exmem;
	logic is_branch_exmem;
	logic noop_exmem;


	// MEMWB
	logic [63:0] pc_memwb;
	logic [31:0] instruction_memwb;
	logic [63:0] ALUresult_memwb;
	logic [63:0] valB_memwb;
	logic [63:0] data_memwb;
	logic [4:0] rd_memwb;
	logic [6:0] opcode_memwb;
	logic [2:0] funct3_memwb;
	logic [6:0] funct7_memwb;
	logic noop_memwb;


	int count = 0;


	enum { R_TYPE = 3'b000, I_TYPE = 3'b001, S_TYPE = 3'b010, SB_TYPE = 3'b011, U_TYPE = 3'b100, UJ_TYPE = 3'b101 } instruction_type;

	logic bus_respcyc_i;
	logic bus_reqack_i;
	logic [BUS_DATA_WIDTH-1:0] bus_resp_i;
	logic [BUS_TAG_WIDTH-1:0] bus_resptag_i;

	logic bus_reqcyc_i;
	logic bus_respack_i;
	logic [BUS_DATA_WIDTH-1:0] bus_req_i;
	logic [BUS_TAG_WIDTH-1:0] bus_reqtag_i;


	logic bus_respcyc_d;
	logic bus_reqack_d;
	logic [BUS_DATA_WIDTH-1:0] bus_resp_d;
	logic [BUS_TAG_WIDTH-1:0] bus_resptag_d;

	logic bus_reqcyc_d;
	logic bus_respack_d;
	logic [BUS_DATA_WIDTH-1:0] bus_req_d;
	logic [BUS_TAG_WIDTH-1:0] bus_reqtag_d;

	logic [63:0] registers [32];
	logic [63:0] next_registers [32];

	logic [127:0] temp_128;	// for multiplication, division
	logic [63:0] temp_64;
	logic [31:0] temp_32a;
	logic [31:0] temp_32b;

	// FLAGS
	logic disable_fetch_branch;
	logic stall_ecall;

	logic [31:0] instruction_done;
	logic noop_done;

	logic debugger = 0;

	logic ecalled;

	//


	logic [3:0] set;

	// 2 bit saturating counter
	enum {
		ST = 2'b00,
		WT = 2'b01,
		WNT = 2'b10,
		SNT = 2'b11
		} counter [32], next_counter [32];

	// branch target buffer
	logic [63:0] btb [32];
	logic btb_valid [32];

	// initialize stack pointer
	initial begin
		next_registers[2] = stackptr;
	end

	always @ (posedge clk) begin
		if (reset) begin

			if (debugger == 0) begin
				pc <= entry;
				registers[2] <= stackptr;
			end else begin
				//pc <= 0;

				pc <= entry;
				registers[2] <= stackptr;
			end
		end else begin

			registers = next_registers;
			
		if (hit == 1) begin

				instruction_done <= instruction_memwb;
				noop_done <= noop_memwb;

				pc <= next_pc;

				

				count <= count + 1;


				if (count == 100000) begin
					//$finish;
				end
	
				if (bus_resptag == 13'b0100000000000) begin
					//$display("invalidation");
				end

				if (instruction == 32'h00000073) begin
					set = set + 1;
				end


			end

	
		end

	end

// START

	always_comb begin

		int rs1 = instruction_ifid[19:15];
		int rs2 = instruction_ifid[24:20];

		int dep_idex = opcode_idex != 7'b1100011 && opcode_idex != 7'b0100011 && noop_idex == 0 && (rs1 == rd_idex && rs1 != 0 || rs2 == rd_idex && rs2 != 0);
		int dep_exmem = opcode_exmem != 7'b1100011 && opcode_exmem != 7'b0100011 && noop_exmem == 0 && (rs1 == rd_exmem && rs1 != 0 || rs2 == rd_exmem && rs2 != 0);
		int dep_memwb = opcode_memwb != 7'b1100011 && opcode_memwb != 7'b0100011 && noop_memwb == 0 && (rs1 == rd_memwb && rs1 != 0 || rs2 == rd_memwb && rs2 != 0);

		if ( noop_ifid == 0 && (dep_idex || dep_exmem || dep_memwb) ) begin
			stall = 1;
		end else begin
			stall = 0;
			valA = registers[rs1];
			valB = registers[rs2];
		end

		//


		// if the just-fetched instruction is a branch, disable PC fetching
		// until the branch is resolved
		if (hit == 1 && (instruction[6:0] == 7'b1101111 || instruction[6:0] == 7'b1100111 || instruction[6:0] == 7'b1100011) ) begin
			disable_fetch_branch = 1;
			//noop = 1;
		end

		if (noop_exmem == 0 && (opcode_exmem == 7'b1101111 || opcode_exmem == 7'b1100111 || opcode_exmem == 7'b1100011)) begin
			disable_fetch_branch = 0;
		end
		
		if (noop_memwb == 0 && (opcode_memwb == 7'b1101111 || opcode_memwb == 7'b1100111 || opcode_memwb == 7'b1100011)) begin
			noop = 0;
		end

		
		// ecall
		/*
                if (instruction_ifid[6:0] == 7'b1110011) begin
                        stall_ecall = 1;
                end
		*/

		if (hit == 1 && (instruction[6:0] == 7'b1110011) && noop == 0) begin
			stall_ecall = 1;
		end

                /*if ( (instruction_ifid[6:0] != 7'b1110011 || noop_ifid == 1 ) && ( opcode_idex != 7'b1110011 || noop_idex == 1) && ( opcode_exmem != 7'b1110011 || noop_exmem == 1) && ( opcode_memwb != 7'b1110011 || noop_memwb == 1) ) begin
                        stall_ecall = 0;
                end*/

		if (noop_memwb == 0 && opcode_memwb == 7'b1110011) begin
			stall_ecall = 0;
		end

		if (noop_done == 0 && instruction_done[6:0] == 7'b1110011) begin
			noop = 0;
		end
		

		//
		//if (stall == 1 || disable_fetch_branch == 1 || stall_ecall == 1) begin
		if (stall == 1 || stall_ecall == 1) begin
			next_pc = pc;
		end else if (is_branch_exmem == 1 && noop_exmem == 0 || opcode_exmem == 7'b1100011 && noop_exmem == 0) begin
			next_pc = target_exmem;
		end else begin
			// branch target prediction
			if (noop == 0 && instruction[6:0] == 7'b1100011 && btb_valid[instruction[11:7]] == 1 && (counter[instruction[11:7]] == ST || counter[instruction[11:7]] == WT)) begin
				next_pc = btb[instruction[11:7]];
			end else begin
				next_pc = pc + 4;
			end

		end


	end



	cache cache_0 (hit, instruction, data, bus_reqcyc, bus_respack, bus_req, bus_reqtag, pc, ALUresult_exmem, num_bytes_exmem, pc[5:0], pc[12:6], ALUresult_exmem[5:0], ALUresult_exmem[12:6], valB_exmem, clk, reset, rd_en, wr_en, bus_respcyc, bus_reqack, bus_resp, bus_resptag, debugger);

	/*
	always_comb begin

		// if the just-fetched instruction is a branch, disable PC fetching
		// until the branch is resolved
		if (instruction[6:0] == 7'b1100011) begin
			disable_fetch_branch = 1;
			noop = 1;
		end else begin
			noop = 0;
		end

	end
	*/	

	ifid ifid_0 (pcplus4_ifid, instruction_ifid, pc_ifid, noop_ifid, hit, clk, stall, stall_ecall, pc + 4, instruction, pc, noop);

/*
	// check for instruction dependency
	always_comb begin

		int rs1 = instruction_ifid[19:15];
		int rs2 = instruction_ifid[24:20];


		if ( (noop_idex == 0 && (rs1 == rd_idex && rs1 != 0 || rs2 == rd_idex && rs2 != 0)) || (noop_exmem == 0 && (rs1 == rd_exmem && rs1 != 0 || rs2 == rd_exmem && rs2 != 0)) || (noop_memwb == 0 && (rs1 == rd_memwb && rs1 != 0 || rs2 == rd_memwb && rs1 != 0)) ) begin
			stall = 1;
		end else begin
			stall = 0;
			valA = registers[rs1];
			valB = registers[rs2];
		end

	end
*/


	// decoderw


	always_comb begin

		rd = instruction_ifid[11:7];
		opcode = instruction_ifid[6:0];
		funct3 = instruction_ifid[14:12];
		funct7 = instruction_ifid[31:25];

		// jumping or branching
		if ((opcode == 7'b1101111 || opcode == 7'b1100111 || opcode == 7'b1100011) && noop_ifid == 0) begin
			noop = 1;
		end

		if ((opcode == 7'b1110011 && noop_ifid == 0)) begin
			noop = 1;
		end

		if (opcode == 7'b0110011 || opcode == 7'b0111011) begin

			instruction_type = R_TYPE;

		end else if (opcode == 7'b1100111 || opcode == 7'b0000011 || opcode == 7'b0010011 || opcode == 7'b0011011 || opcode == 7'b1110011) begin

			instruction_type = I_TYPE;

		end else if (opcode == 7'b0100011) begin

			instruction_type = S_TYPE;

		end else if (opcode == 7'b1100011) begin

			instruction_type = SB_TYPE;

		end else if (opcode == 7'b0110111 || opcode == 7'b0010111) begin

			instruction_type = U_TYPE;

		end else if (opcode == 7'b1101111) begin

			instruction_type = UJ_TYPE;

		end
	
		case (instruction_type)

			R_TYPE:
				begin
					imm = 0;
				end
			I_TYPE:
				begin
					imm = { {53{instruction_ifid[31]}}, instruction_ifid[30:25], instruction_ifid[24:21], instruction_ifid[20]};
				end
			S_TYPE:
				begin
					imm = { {53{instruction_ifid[31]}}, instruction_ifid[30:25], instruction_ifid[11:8], instruction_ifid[7]};
				end
			SB_TYPE:
				begin
					imm = { {52{instruction_ifid[31]}}, instruction_ifid[7], instruction_ifid[30:25], instruction_ifid[11:8], 1'b0};
				end
			U_TYPE:
				begin
					imm = { {33{instruction_ifid[31]}}, instruction_ifid[30:20], instruction_ifid[19:12], 12'b000000000000};
				end
			UJ_TYPE:
				begin
					imm = { {44{instruction_ifid[31]}}, instruction_ifid[19:12], instruction_ifid[20], instruction_ifid[30:25], instruction_ifid[24:21], 1'b0};
				end

		endcase

		// calculate number of bytes to write on store instruction
		if (opcode == 7'b0100011) begin

			case (funct3)

				3'b000:
					begin
						num_bytes = 1;
					end		
				3'b001:
					begin
						num_bytes = 2;
					end	
				3'b010:
					begin
						num_bytes = 4;
					end
				3'b011:
					begin
						num_bytes = 8;
					end

			endcase


		end

	end



	idex idex_0 (pcplus4_idex, valA_idex, valB_idex, rd_idex, opcode_idex, funct3_idex, funct7_idex, imm_idex, num_bytes_idex, instruction_idex, pc_idex, noop_idex, hit, clk, stall, pcplus4_ifid, valA, valB, rd, opcode, funct3, funct7, imm, num_bytes, instruction_ifid, pc_ifid, noop_ifid );

	// isBranch calculator
	always_comb begin

		if (opcode_idex == 7'b1101111 || opcode_idex == 7'b1100111) begin	// unconditional jump

			is_branch = 1;

		end else if (opcode_idex == 7'b1100011) begin
			
			case (funct3_idex)

				3'b000:
					begin

						// BEQ
						if ($signed(valA_idex) == $signed(valB_idex)) begin
							is_branch = 1;
						end else begin
							is_branch = 0;
						end
					end

				3'b001:
					begin
						// BNE
						if ($signed(valA_idex) != $signed(valB_idex)) begin
							is_branch = 1;
						end else begin
							is_branch = 0;
						end
					end

				3'b100:
					begin
						// BLT
						if ($signed(valA_idex) < $signed(valB_idex)) begin
							is_branch = 1;
						end else begin
							is_branch = 0;
						end
					end

				3'b101:
					begin
						// BGE
						if ($signed(valA_idex) >= $signed(valB_idex)) begin
							is_branch = 1;
						end else begin
							is_branch = 0;
						end
					end

				3'b110:
					begin
						// BLTU
						if ($unsigned(valA_idex) < $unsigned(valB_idex)) begin
							is_branch = 1;
						end else begin
							is_branch = 0;
						end
					end

				3'b111:
					begin
						// BGEU
						if ($unsigned(valA_idex) >= $unsigned(valB_idex)) begin
							is_branch = 1;
						end else begin
							is_branch = 0;
						end
					end

			endcase

			// updating the 2 bit saturating counter

			if (is_branch == 1) begin

				case (counter[instruction_idex[11:7]])
		
					ST:
						begin
							next_counter[instruction_idex[11:7]] = ST;
						end
					
					WT:
						begin
							next_counter[instruction_idex[11:7]] = ST;
						end
						
					WNT:
						begin
							next_counter[instruction_idex[11:7]] = ST;
						end

					SNT:
						begin
							next_counter[instruction_idex[11:7]] = WNT;
						end

				endcase

			end else if (is_branch == 0) begin

				case (counter[instruction_idex[11:7]])
		
					ST:
						begin
							next_counter[instruction_idex[11:7]] = WT;
						end
					
					WT:
						begin
							next_counter[instruction_idex[11:7]] = SNT;
						end
						
					WNT:
						begin
							next_counter[instruction_idex[11:7]] = SNT;
						end

					SNT:
						begin
							next_counter[instruction_idex[11:7]] = SNT;
						end

				endcase

			end


		end else begin
			is_branch = 0;
		end

	end


	// target calculator
	always_comb begin

		if (opcode_idex == 7'b1101111) begin	// JAL

			target = pc_idex + imm_idex; 

		end else if (opcode_idex == 7'b1100111) begin	// JALR

			target = valA_idex + imm_idex;
			target[0] = 1'b0;

		end else if (opcode_idex == 7'b1100011) begin	// branch

			if (is_branch == 1) begin
				target = pc_idex + imm_idex;
				btb[instruction_idex[11:7]] = target;
				btb_valid[instruction_idex[11:7]] = 1;
			end else begin
				target = pc_idex + 4;
			end

		end
		

	end

	// ALU
	always_comb begin


		if (opcode_idex == 7'b0110111) begin	// LUI

			ALUresult = imm_idex;

		end else if (opcode_idex == 7'b0010111) begin	// AUIPC

			ALUresult = pc_idex + imm_idex;

		end else if (opcode_idex == 7'b1101111) begin	// JAL

			ALUresult = pc_idex + 4;

		end else if (opcode_idex == 7'b1100111)  begin	// JALR

			ALUresult = pc_idex + 4;

		end else if (opcode_idex == 7'b0000011 || opcode_idex == 7'b0100011) begin // load or store, calculates byte address in memory

			ALUresult = valA_idex + imm_idex;

		end else if (opcode_idex == 7'b0010011) begin

			case (funct3_idex)

				3'b000:
					begin
						// ADDI
						ALUresult = valA_idex + imm_idex;
					end

				3'b010:
					begin
						// SLTI
						if ($signed(valA_idex) < $signed(imm_idex)) begin
							ALUresult = 1;
						end else begin
							ALUresult = 0;
						end
					end

				3'b011:
					begin
						// SLTIU
						if ($unsigned(valA_idex) < $unsigned(imm_idex)) begin
							ALUresult = 1;
						end else begin
							ALUresult = 0;
						end
					end

				3'b100:
					begin
						// XORI
						ALUresult = valA_idex ^ imm_idex;
					end
				
				3'b110:
					begin
						// ORI
						ALUresult = valA_idex | imm_idex;
					end

				3'b111:
					begin
						// ANDI
						ALUresult = valA_idex & imm_idex;
					end
	
				3'b001:
					begin
						// SLLI
						int shamt = imm_idex[5:0];
						ALUresult = valA_idex << shamt;
					end

				3'b101:
					begin
						int shamt = imm_idex[5:0];
						if (imm_idex[11:6] == 6'b000000) begin
							// SRLI
							ALUresult = valA_idex >> shamt;
						end else if (imm_idex[11:6] == 6'b010000) begin
							// SRAI

							ALUresult = valA_idex >> shamt;

							for (int i = 63; i > 63 - shamt; i--) begin
								ALUresult[i] = valA_idex[63];
							end


						end
					end


			endcase

		end else if (opcode_idex == 7'b0110011) begin // register instruction
			
			case (funct3_idex)

				3'b000:
					begin
						if (funct7_idex == 7'b0000000) begin
							// ADD
							ALUresult = valA_idex + valB_idex;
						end else if (funct7_idex == 7'b0100000) begin
							// SUB
							ALUresult = valA_idex - valB_idex;
						end else if (funct7_idex == 7'b0000001) begin
							// MUL
							temp_128 = $signed(valA_idex) * $signed(valB_idex);
							ALUresult = temp_128[63:0];
						end
					end

				3'b001:
					begin
						if (funct7_idex == 7'b0000000) begin
							// SLL
							int shamt = valB_idex[5:0];
							ALUresult = valA_idex << shamt;
						end else if (funct7_idex == 7'b0000001) begin
							// MULH
							temp_128 = $signed(valA_idex) * $signed(valB_idex);
							ALUresult = temp_128[127:64];
						end
					end
				
				3'b010:
					begin
						if (funct7_idex == 7'b0000000) begin				
							// SLT
							if ($signed(valA_idex) < $signed(valB_idex)) begin
								ALUresult = 1;
							end else begin
								ALUresult = 0;
							end 
						end else if (funct7_idex == 7'b0000001) begin
							// MULHSU
							temp_128 = $signed(valA_idex) * $unsigned(valB_idex);
							ALUresult = temp_128[127:64];
						end
					end
				
				3'b011:
					begin
						if (funct7_idex == 7'b0000000) begin
							// SLTU
							if ($unsigned(valA_idex) < $unsigned(valB_idex)) begin
								ALUresult = 1;
							end else begin
								ALUresult = 0;
							end
						end else if (funct7_idex == 7'b0000001) begin
							// MULHU
							temp_128 = $unsigned(valA_idex) * $unsigned(valB_idex);
							ALUresult = temp_128[127:64];
						end
					end

				3'b100:
					begin
						if (funct7_idex == 7'b0000000) begin
							// XOR
							ALUresult = valA_idex ^ valB_idex;
						end else if (funct7_idex == 7'b0000001) begin
							// DIV
							ALUresult = $signed(valA_idex) / $signed(valB_idex);
						end
					end

				3'b101:
					begin
						int shamt = valB_idex[5:0];
						assert (shamt >= 0);

						if (funct7_idex == 7'b0000000) begin
							// SRL
							ALUresult = valA_idex >> shamt;
						end else if (funct7_idex == 7'b0100000) begin
							// SRA
							ALUresult = valA_idex >> shamt;
							for (int i = 63; i > 63 - shamt; i--) begin
								ALUresult[i] = valA_idex[63];
							end

						end else if (funct7_idex == 7'b0000001) begin
							// DIVU
							ALUresult = $unsigned(valA_idex) / $unsigned(valB_idex);
						end
					end

				3'b110:
					begin
						if (funct7_idex == 7'b0000000) begin
							// OR
							ALUresult = valA_idex | valB_idex;
						end else if (funct7_idex == 7'b0000001) begin
							// REM
							ALUresult = $signed(valA_idex) % $signed(valB_idex);
						end
					end
				
				3'b111:
					begin
						if (funct7_idex == 7'b0000000) begin
							// AND
							ALUresult = valA_idex & valB_idex;
						end else if (funct7_idex == 7'b0000001) begin
							// REMU
							ALUresult = $unsigned(valA_idex) % $unsigned(valB_idex);
						end
					end

			endcase
	
		end else if (opcode_idex == 7'b0011011) begin

			case (funct3_idex)

				3'b000:
					begin
						// ADDIW
						ALUresult = valA_idex + imm_idex;
						ALUresult = { {32{ALUresult[31]}}, ALUresult[31:0] };
					end

				3'b001:
					begin
						// SLLIW
						int shamt = imm_idex[4:0];
						ALUresult = valA_idex << shamt;
						ALUresult = { {32{ALUresult[31]}}, ALUresult[31:0] };
					end

				3'b101:
					begin
						int shamt = imm_idex[4:0];	

						assert (shamt >= 0);

						if (imm_idex[11:5] == 7'b0000000) begin
							// SRLIW
							ALUresult = valA_idex >> shamt;
							for (int i = 31; i > 31 - shamt; i--) begin
								ALUresult[i] = 0;
							end
							ALUresult = { {32{ALUresult[31]}}, ALUresult[31:0] };


						end else if (imm_idex[11:5] == 7'b0100000) begin
							// SRAIW
							//ALUresult = valA_idex >>> shamt;


							ALUresult = valA_idex >> shamt;

							for (int i = 31; i > 31 - shamt; i--) begin
								ALUresult[i] = valA_idex[31];
							end

							ALUresult = { {32{ALUresult[31]}}, ALUresult[31:0] };

						end
					end

			endcase

		end else if (opcode_idex == 7'b0111011) begin

			case (funct3_idex)

				3'b000:
					begin
						if (funct7_idex == 7'b0000000) begin
							// ADDW
							ALUresult = valA_idex + valB_idex;
							ALUresult = { {32{ALUresult[31]}}, ALUresult[31:0] };
						end else if (funct7_idex == 7'b0100000) begin
							// SUBW
							ALUresult = valA_idex - valB_idex;
							ALUresult = { {32{ALUresult[31]}}, ALUresult[31:0] };
						end else if (funct7_idex == 7'b0000001) begin
							// MULW

							temp_32a = valA_idex[31:0];
							temp_32b = valB_idex[31:0];

							temp_64 = $signed(temp_32a) * $signed(temp_32b);

							ALUresult = { {32{temp_64[31]}}, temp_64[31:0] };

						end
					end

				3'b001:
					begin
						// SLLW
						int shamt = valB_idex[4:0];
						assert (shamt >= 0);
						ALUresult = valA_idex << shamt;
						ALUresult = { {32{ALUresult[31]}}, ALUresult[31:0] };	
					end

				3'b101:
					begin
						int shamt = valB_idex[4:0];
						if (funct7_idex == 7'b0000000) begin
							// SRLW
							ALUresult = valA_idex >> shamt;
							for (int i = 31; i > 31 - shamt; i--) begin
								ALUresult[i] = 1'b0;
							end

						end else if (funct7_idex == 7'b0100000) begin
							// SRAW
							ALUresult = valA_idex >> shamt;
							for (int i = 31; i > 31 - shamt; i--) begin
								ALUresult[i] = valA_idex[31];
							end
						end
						ALUresult = { {32{ALUresult[31]}}, ALUresult[31:0] };	
					end

				3'b100:
					begin
						// DIVW

						temp_32a = valA_idex[31:0];
						temp_32b = valB_idex[31:0];
						temp_32a = $signed(temp_32a) / $signed(temp_32b);

						ALUresult = { {32{temp_32a[31]}}, temp_32a };

					end

				3'b101:
					begin
						// DIVUW
						temp_32a = valA_idex[31:0];
						temp_32b = valB_idex[31:0];
						temp_32a = $unsigned(temp_32a) / $unsigned(temp_32b);

						ALUresult = { {32{temp_32a[31]}}, temp_32a };
					end
		
				3'b110:
					begin
						//REMW
						temp_32a = valA_idex[31:0];
						temp_32b = valB_idex[31:0];
						temp_32a = $signed(temp_32a) % $signed(temp_32b);

						ALUresult = { {32{temp_32a[31]}}, temp_32a };
					end

				3'b111:
					begin
						//REMUW
						temp_32a = valA_idex[31:0];
						temp_32b = valB_idex[31:0];
						temp_32a = $unsigned(temp_32a) % $unsigned(temp_32b);

						ALUresult = { {32{temp_32a[31]}}, temp_32a };
					end
			endcase


		end

	end



	
	exmem exmem_0 ( target_exmem, ALUresult_exmem, valB_exmem, rd_exmem, opcode_exmem, funct3_exmem, funct7_exmem, num_bytes_exmem, is_branch_exmem, instruction_exmem, pc_exmem, noop_exmem, hit, clk, target, ALUresult, valB_idex, rd_idex, opcode_idex, funct3_idex, funct7_idex, num_bytes_idex, is_branch, instruction_idex, pc_idex, noop_idex );


	always_comb begin

		// is load?
		if (opcode_exmem == 7'b0000011 && noop_exmem == 0) begin
			rd_en = 1;
		end else begin
			rd_en = 0;
		end

		// is store?
		if (opcode_exmem == 7'b0100011 && noop_exmem == 0) begin
			wr_en = 1;
		end else begin
			wr_en = 0;
		end

	end

	memwb memwb_0 ( ALUresult_memwb, valB_memwb, data_memwb, rd_memwb, opcode_memwb, funct3_memwb, funct7_memwb, instruction_memwb, pc_memwb, noop_memwb, hit, clk, ALUresult_exmem, valB_exmem, data, rd_exmem, opcode_exmem, funct3_exmem, funct7_exmem, instruction_exmem, pc_exmem, noop_exmem );


	// call do_pending_write
	always_ff @ (posedge clk) begin

		if (opcode_memwb == 7'b0100011 && noop_memwb == 0 && hit == 1) begin
			case (funct3_memwb)

				3'b000:
					begin
						// SB
						do_pending_write(ALUresult_memwb, valB_memwb[7:0], 1);
					end

				3'b001:
					begin
						// SH
						do_pending_write(ALUresult_memwb, valB_memwb[15:0], 2);
					end

				3'b010:
					begin
						// SW
						do_pending_write(ALUresult_memwb, valB_memwb[31:0], 4);
					end

				3'b011:
					begin
						// SD
						do_pending_write(ALUresult_memwb, valB_memwb, 8);
					end

			endcase

		end

	end	

	// call ecall
	always_ff @ (posedge clk) begin

		//if (opcode_memwb == 7'b1110011 && noop_memwb == 0) begin
		if (opcode_memwb == 7'b1110011 && noop_memwb == 0 && hit == 1) begin
			if (instruction_memwb[20] == 0) begin
				do_ecall(registers[17], registers[10], registers[11], registers[12], registers[13], registers[14], registers[15], registers[16], next_registers[10]);
			end else begin
				$display("EBREAK");
			end
		end
	
	end


	// write back to register file
	always_comb begin

		if (opcode_memwb != 7'b1100011 && opcode_memwb != 7'b0100011 && opcode_memwb != 7'b1110011 && noop_memwb == 0) begin

			if (opcode_memwb == 7'b0000011) begin
				
				case (funct3_memwb)

					3'b000:
						begin
							// LB
							next_registers[rd_memwb] = { {56{data_memwb[7]}}, data_memwb[7:0] };
						end

					3'b001:
						begin
							// LH
							next_registers[rd_memwb] = { {48{data_memwb[15]}}, data_memwb[15:0] };
						end

					3'b010:
						begin
							// LW
							next_registers[rd_memwb] = { {32{data_memwb[31]}}, data_memwb[31:0] };
						end

					3'b100:
						begin
							// LBU
							next_registers[rd_memwb] = { {56{1'b0}}, data_memwb[7:0] };
						end

					3'b101:
						begin
							// LHU
							next_registers[rd_memwb] = { {48{1'b0}}, data_memwb[15:0] };
						end					

					3'b110:
						begin
							// LWU
							next_registers[rd_memwb] = { {32{1'b0}}, data_memwb[31:0] };
						end

					3'b011:
						begin
							// LD
							next_registers[rd_memwb] = data_memwb;
						end


				endcase

			end else begin
				if (rd_memwb == 0) begin
					next_registers[rd_memwb] = 0;
				end else begin
					next_registers[rd_memwb] = ALUresult_memwb;
				end
			end

		end

	end
	


endmodule


