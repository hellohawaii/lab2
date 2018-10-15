`timescale 10ns / 1ns

//main control module

module control_unit(
	input  clk,
	input  resetn,
	input  [5:0] behavior,
	//input  [31:0] Result,//ALU result
	output [1:0] reg_dst,//signal for mux(where to write)
	output mem_read,//enable signal
	output [3:0] reg_write_value,//signal for mux(what to write reg_file)
	output [2:0] ALUop,//signal for ALU control
	output mem_write,//enable signal
	output [1:0] B_src,//signal for mux(what to compute)
	output reg_write,//enable signal
	//output [3:0] data_sram_wen,//signal for 8or16-bit write
	output [2:0] mem_write_value,//signal for mux(what to write to mem)
	output bne,
    output beq,
	output j,
	output jal,
	output R_type,
	output regimm,
	output blez,
	output bgtz,
	output sw,
	output sb,
	output sh,
	output swl,
	output swr
);
	//signal recording the decoded instruction
	wire addi , addiu, lw  , nop;
	wire lui  , slti, sltiu;
	wire andi , lb  , lbu  , lh , lhu, lwl , lwr,
	     ori;

	//decoding
	assign addi  =(behavior==6'b001000)?1:0;
	assign addiu =(behavior==6'b001001)?1:0;
	assign lw    =(behavior==6'b100011)?1:0;
	assign sw    =(behavior==6'b101011)?1:0;
	assign bne   =(behavior==6'b000101)?1:0;
	assign R_type=(behavior==6'b000000)?1:0;
	assign beq   =(behavior==6'b000100)?1:0;
	assign j     =(behavior==6'b000010)?1:0;
	assign jal   =(behavior==6'b000011)?1:0;
	assign lui   =(behavior==6'b001111)?1:0;
	assign slti  =(behavior==6'b001010)?1:0;
	assign sltiu =(behavior==6'b001011)?1:0;
	assign andi  =(behavior==6'b001100)?1:0;
	assign regimm=(behavior==6'b000001)?1:0;
	assign blez  =(behavior==6'b000110)?1:0;
	assign bgtz  =(behavior==6'b000111)?1:0;
	assign lb    =(behavior==6'b100000)?1:0;
	assign lbu   =(behavior==6'b100100)?1:0;
	assign lh    =(behavior==6'b100001)?1:0;
	assign lhu   =(behavior==6'b100101)?1:0;
	assign lwl   =(behavior==6'b100010)?1:0;
	assign lwr   =(behavior==6'b100110)?1:0;
	assign ori   =(behavior==6'b001101)?1:0;
	assign sb    =(behavior==6'b101000)?1:0;
	assign sh    =(behavior==6'b101001)?1:0;
	assign swl   =(behavior==6'b101010)?1:0;
	assign swr   =(behavior==6'b101110)?1:0;
	assign xori  =(behavior==6'b001110)?1:0;

	wire Jump_Class, Load_Class, Store_Class;
	assign Jump_Class =(bne || beq || j   || jal || regimm || blez|| bgtz)?1:0;
	assign Load_Class =(lw  || lb  || lbu || lh  || lhu    || lwl || lwr  || lui)?1:0;
	assign Store_Class=(sw  || sb  || sh  || swl || swr                  )?1:0;
	assign mem_read=Load_Class;
	assign mem_write=Store_Class;
	assign reg_write=jal | ~(Jump_Class |  Store_Class);

	//Control Signal(used for make a choice)
	assign reg_dst=(R_type)? 2'b01:
				   (jal   )? 2'b10:
		                     2'b00; //2'b00 is rt
	assign reg_write_value=(lw   )? 4'b0001:
		                   (jal  )? 4'b0010:
				    	   (lui  )? 4'b0011:
						   (sltiu)? 4'b0100:
						   (lb   )? 4'b0101:
						   (lbu  )? 4'b0110:
						   (lh   )? 4'b0111:
						   (lhu  )? 4'b1000:
						   (lwl  )? 4'b1001:
						   (lwr  )? 4'b1010:
		                            4'b0000;     //4'b0000 is result
	assign ALUop=(addiu || lw    || sw    || lb   || lbu   || 
		          lh    || lhu   || lwl   || lwr  || sb    ||
			      sh    || swl   || swr   || addi            )? 3'b000:  //add
				 (bne   || beq   || sltiu                    )? 3'b001:  //sub
				 (R_type                                     )? 3'b011:  //R
				 (slti  || regimm|| blez  || bgtz            )? 3'b010:  //slt
				 (andi                                       )? 3'b100:  //add
				 (ori                                        )? 3'b101:  //or
				 (xori                                       )? 3'b110:  //xor
				                                                3'b010;  //slt
	assign B_src=(addiu || lw    || sw    || slti || sltiu || 
				  lb    || lbu   || lh    || lhu  || lwl   ||
				  lwr   || sb    || sh    || swl  || swr   ||
                  addi				  )? 2'b01:  
				                                                         //sign_extended imm
				 (regimm|| blez  || bgtz                     )? 2'b10:
				 (ori   || andi  || xori                     )? 2'b11:   //zero_extended imm
				                                                2'b00;   //rdata2
	assign mem_write_value=(sb )? 3'b001:
		                   (sh )? 3'b010:
						   (swl)? 3'b011:
						   (swr)? 3'b100:
						          3'b000;
	/*
	wire [3:0] write_strb_sb ;
	wire [3:0] write_strb_sh ;
	wire [3:0] write_strb_swl;
	wire [3:0] write_strb_swr;

	assign write_strb_sb =(Result[1:0]==2'b00)? 4'b0001:
						  (Result[1:0]==2'b01)? 4'b0010:
						  (Result[1:0]==2'b10)? 4'b0100:
						  (Result[1:0]==2'b11)? 4'b1000:
						                        4'b0000;
	assign write_strb_sh =(Result[ 1 ]==0    )? 4'b0011:
						  (Result[ 1 ]==1    )? 4'b1100:
						                        4'b0000;
	assign write_strb_swl=(Result[1:0]==2'b00)? 4'b0001:
						  (Result[1:0]==2'b01)? 4'b0011:
						  (Result[1:0]==2'b10)? 4'b0111:
						  (Result[1:0]==2'b11)? 4'b1111:
			                 			        4'b0000;
	assign write_strb_swr=(Result[1:0]==2'b00)? 4'b1111:
						  (Result[1:0]==2'b01)? 4'b1110:
						  (Result[1:0]==2'b10)? 4'b1100:
						  (Result[1:0]==2'b11)? 4'b1000:
						                        4'b0000;
	assign data_sram_wen=(sw )?        4'b1111:
		                 (sb )? write_strb_sb :
					     (sh )? write_strb_sh :
				   	     (swl)? write_strb_swl:
					     (swr)? write_strb_swr:
					                   4'b0000;
	*/
endmodule
