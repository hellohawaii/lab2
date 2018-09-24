`timescale 10ns / 1ns

module mycpu_top(
	input  resetn,
	input  clk,

	output inst_sram_en,
	output [3:0] inst_sram_wen,
	output [31:0] inst_sram_addr,
	output [31:0] inst_sram_wdata,
	input [31:0] inst_sram_rdata,
	
	output data_sram_en,
	output [3:0] data_sram_wen,
	output [31:0] data_sram_addr,
	output [31:0] data_sram_wdata,
	input [31:0] data_sram_rdata,
	
	output reg [31:0] debug_wb_pc,
	output [3:0]debug_wb_rf_wen,
	output [4:0] debug_wb_rf_wnum,
	output [31:0] debug_wb_rf_wdata,

    output [31:0]	mips_perf_cnt_0//clk_counter
);

    reg [31:0] PC;//在IF结束之前置为正确的值（这样ID阶段有正确的instr）
	              //是IF级的输出，不过不是哪一级的输出
	
	//the following do not need to change when using pipline
	assign inst_sram_wen = 0;//固定值，跟流水的输入输出无关
	assign inst_sram_addr = PC;//与PC有关，在IF结束之前变成正确的值
	assign inst_sram_wdata = 0;//固定值，跟流水的输入输出无关
	assign debug_wb_rf_wen = {4{wen_reg_file}};//TODO
	assign debug_wb_rf_wnum = waddr;//TODO
	assign debug_wb_rf_wdata = wdata;//TODO
		
	wire [31:0] inst;//在ID阶段得到正确的instruction，据他产生很多信号。
	                 //是ID阶段的输出（ID阶段也要使用），可能很多其他信号要用到它。TODO
	assign inst=inst_sram_rdata;
	wire [31:0] data_from_mem;//是MEM阶段的输出（EX阶段，设置了地址等于ALU的结果，从而在MEM阶段一开始就可以得到正确的结果）
	                          //可能WB阶段要用到它
							  //TODO，可能还要收到读使能信号的影响，影响是什么时候输出
    assign data_from_mem=data_sram_rdata;

	//define some simple signals (using extend)
	//用inst产生一些简单信号，这个可能是译码阶段产生的，不知道什么时候用（TODO）
	wire [31:0] sign_extended_imm;
	assign sign_extended_imm={{16{inst[15]}},inst[15:0]};
	wire [25:0] instr_index;
	assign instr_index=inst[25:0];
	wire [27:0] instr_index_sl2;
	assign instr_index_sl2={instr[25:0],2'b00};//稍作更改，更整齐一些
	wire [31:0] imm_sl16;
	assign imm_sl16={inst[15:0],16'b0};
	wire [31:0] zero_extended_imm;
	assign zero_extended_imm={16'b0,inst[15:0]};

    // define the signal related to main control
	//TODO，这个以后再看
    wire [5:0] behavior;                                //done
	wire [31:0] Result;									//done (used in 2 places)
    wire [1:0] reg_dst;//signal for mux(where to write)	//done
    wire branch;//if the behavior is about branch		//done
    wire mem_read;//enable signal						//done
    wire [3:0] reg_write_value;//signal for mux(what to write)		//done	
    wire [2:0] ALUop;//signal for ALU control			//done
	wire mem_write;//enable signal						//done
	wire [1:0] B_src;//signal for mux(what to compute)		//done
    wire reg_write;//enable signal						//done
	wire PC_enable;
	wire bne;                                           //done
	wire beq;											//done
	wire j;												//done
	wire jal;											//done
	wire R_type;
	wire regimm;
	wire blez;
	wire bgtz;
	wire [2:0] mem_write_value;
	wire writing_back;

	//add the control unit into the circuit
	control_unit cpu_control_unit(.clk(clk),.resetn(resetn),.inst_sram_en(inst_sram_en),
	    .behavior(behavior),.Result(Result),
		.reg_dst(reg_dst),.mem_read(mem_read),.reg_write_value(reg_write_value),
		.ALUop(ALUop),.mem_write(mem_write),.B_src(B_src),.reg_write(reg_write),
		.data_sram_wen(data_sram_wen),.mem_write_value(mem_write_value),
		.PC_enable(PC_enable),
		
		//decoding signal
		.bne(bne),.beq(beq),.j(j),.jal(jal),.R_type(R_type),
		.regimm(regimm),.blez(blez),.bgtz(bgtz),.writing_back(writing_back)
	);
    assign behavior=inst[31:26]; //据inst（ID阶段产生）产生的信号，大约也是用在ID，TODO
	assign data_sram_en=mem_read | mem_write;//TODO，大约是在ID阶段产生，后面各个级都要用到它，这个比较复杂
	
	//TODO,不知道新任务中debug_pc怎么赋值，没有这个写回周期了
    wire debug_wb_pc_update;
	assign debug_wb_pc_update = writing_back | jal;
	always@(posedge debug_wb_pc_update)
	begin
	    debug_wb_pc <= PC;
    end
	
	//define the signal related to reg_file
	//TODO，一会儿再看
	wire clk_reg_file;									//
	wire rst_reg_file;									//外界输入，不需要保存
	wire [4:0] waddr;//ID结束的时候产生它的值，从而EX阶段依据它（TODO，以及写使能）来写
	                 //应该是不需要保存
	wire [4:0] raddr1;									//done
	wire [4:0] raddr2;									//done
	wire wen_reg_file;									//done
	wire [31:0] wdata;									//done
	wire [31:0] rdata1;									//done
	wire [31:0] rdata2;									//done (used in 2 places)
	//add the reg_flie into the circuit
	reg_file cpu_reg_file(.clk(clk_reg_file),.resetn(rst_reg_file),.waddr(waddr),.raddr1(raddr1),
		.raddr2(raddr2),.wen(wen_reg_file),.wdata(wdata),.rdata1(rdata1),.rdata2(rdata2));
	assign clk_reg_file=clk;//流水线应该不需要更改它
	assign raddr1=inst[25:21];//ID阶段开始的时候就可以据inst产生，这样ID阶段由于是组合逻辑，可以直接获得正确的读数据
	                          //TODO，与使能信号配合
	assign raddr2=inst[20:16];//ID阶段开始的时候就可以据inst产生，这样ID阶段由于是组合逻辑，可以直接获得正确的读数据
	                          //TODO，与使能信号配合
	assign wen_reg_file=(R_type==1 && inst[5:0]==6'b001011)?(rdata2!=32'b0):
	                    (R_type==1 && inst[5:0]==6'b001010)?(rdata2==32'b0):
	                    reg_write;//movn,movz
						//对于普通的信号，译码完成就可以有效产生；对于movn和movz，可能需要执行阶段开始的时候才能得到正确的值
						//使用的时候，是WB阶段来使用它（在WB阶段之前置为正确值）
	assign rst_reg_file=resetn;

	//define the signal related to ALU
	//TODO，一会儿再看
	wire [31:0] A;										//done
	wire [31:0] B;										//done
	wire [3:0] ALUoperation;							//done
	wire Overflow;
	wire CarryOut;
	wire Zero;											//done (used in pc_decider)
	//add the ALU into the circuit
	alu cpu_alu(.A(A),.B(B),.ALUop(ALUoperation),.Overflow(Overflow),.CarryOut(CarryOut),
		.Zero(Zero),.Result(Result));
	assign data_sram_addr=Result;

	//add the ALU control unit into the circuit
	ALU_control cpu_ALU_control(.func(inst[5:0]),.ALUop(ALUop),
		.ALU_ctr(ALUoperation));
	
	//MUX, what to write to memory
	//这些信号的产生，需要EX阶段读RF以及ALU的结果得到。它们应该在EX阶段产生。
	//被用来在MEM阶段写内存，需要有控制信号和选择信号配合，也需要与addr配合（TODO）
	wire [31:0] write_data_sb;
	wire [31:0] write_data_sh;
	wire [31:0] write_data_swl;
	wire [31:0] write_data_swr;
	assign write_data_sb={4{rdata2[7:0]}};  //unify 4 situations of result
	assign write_data_sh={2{rdata2[15:0]}};  //unify 4 situations of result
	assign write_data_swl=(Result[1:0]==2'b00)?{24'b0,rdata2[31:14]}:
						  (Result[1:0]==2'b01)?{16'b0,rdata2[31:16]}:
						  (Result[1:0]==2'b10)?{8'b0,rdata2[31:8]}:
						  (Result[1:0]==2'b11)?rdata2:
						  32'b0;
	assign write_data_swr=(Result[1:0]==2'b00)?rdata2:
						  (Result[1:0]==2'b01)?{rdata2[23:0],8'b0}:
						  (Result[1:0]==2'b10)?{rdata2[15:0],16'b0}:
						  (Result[1:0]==2'b11)?{rdata2[7:0],24'b0}:
						  32'b0;
	assign data_sram_wdata=(mem_write_value==3'b000)?rdata2:
		              (mem_write_value==3'b001)?write_data_sb:
				      (mem_write_value==3'b010)?write_data_sh:
					  (mem_write_value==3'b011)?write_data_swl:
					  (mem_write_value==3'b100)?write_data_swr:
					  rdata2;

	//MUX, where to write, decide 'waddr'
	//在译码阶段就可以产生出备选地址
	//不知道选择信号怎么产生的。估计也是译码（TODO）
	//在MEM阶段使用这些信号
	wire [4:0] waddr_option00;
	wire [4:0] waddr_option01;
	assign waddr_option00=inst[20:16];
	assign waddr_option01=inst[15:11];
	assign waddr=(reg_dst==2'b00)? waddr_option00: 
		         (reg_dst==2'b01)? waddr_option01:
				 (reg_dst==2'b10)? 5'b11111:
				 5'b00000;
	
	//MUX, what to compute, decide  'B'
	//00选项需要EX阶段产生，其他的选项ID之后就有效了
	//选择信号信号在ID阶段就产生
	wire [31:0] B_option00;
	wire [31:0] B_option01;
	wire [31:0] B_option10;
	wire [31:0] B_option11;
	assign B_option00=rdata2;
	assign B_option01=sign_extended_imm;
	assign B_option10=32'b0;
	assign B_option11=zero_extended_imm;
	assign B=(B_src==2'b00)? B_option00:
		     (B_src==2'b01)? B_option01:
			 (B_src==2'b10)? B_option10:
			 (B_src==2'b11)? B_option11:
			 rdata2;

	//MUX,what to write to reg_flie, decide 'wdata'
	wire [31:0] wdata_option0000;
	wire [31:0] wdata_option0001;
	wire [31:0] wdata_lb;
	wire [31:0] wdata_lbu;
	wire [31:0] wdata_lh;
	wire [31:0] wdata_lhu;
	wire [31:0] wdata_lwl;
	wire [31:0] wdata_lwr;
	assign wdata_option0000=Result;//EX阶段产生
	assign wdata_option0001=data_from_mem;//MEM阶段产生
	assign wdata_lb=(Result[1:0]==2'b00)?{{24{data_from_mem[7]}},data_from_mem[7:0]}:
				    (Result[1:0]==2'b01)?{{24{data_from_mem[15]}},data_from_mem[15:8]}:
					(Result[1:0]==2'b10)?{{24{data_from_mem[23]}},data_from_mem[23:16]}:
					(Result[1:0]==2'b11)?{{24{data_from_mem[31]}},data_from_mem[31:24]}:
					32'b0;
					//依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lbu=(Result[1:0]==2'b00)?{24'b0,data_from_mem[7:0]}:
					 (Result[1:0]==2'b01)?{24'b0,data_from_mem[15:8]}:
					 (Result[1:0]==2'b10)?{24'b0,data_from_mem[23:16]}:
					 (Result[1:0]==2'b11)?{24'b0,data_from_mem[31:24]}:
					 32'b0;
					 //依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lh=(Result[1]==0)?{{16{data_from_mem[15]}},data_from_mem[15:0]}:
		            (Result[1]==1)?{{16{data_from_mem[31]}},data_from_mem[31:16]}:
					32'b0;
					//依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lhu=(Result[1]==0)?{16'b0,data_from_mem[15:0]}:
					 (Result[1]==1)?{16'b0,data_from_mem[31:16]}:
					 32'b0;
					 //依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lwl=(Result[1:0]==2'b00)?{data_from_mem[7:0],rdata2[23:0]}:
					 (Result[1:0]==2'b01)?{data_from_mem[15:0],rdata2[15:0]}:
					 (Result[1:0]==2'b10)?{data_from_mem[23:0],rdata2[7:0]}:
					 (Result[1:0]==2'b11)?data_from_mem[31:0]:
					 32'b0;
					 //依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lwr=(Result[1:0]==2'b00)?data_from_mem[31:0]:
				     (Result[1:0]==2'b01)?{rdata2[31:24],data_from_mem[31:8]}:
					 (Result[1:0]==2'b10)?{rdata2[31:16],data_from_mem[31:16]}:
					 (Result[1:0]==2'b11)?{rdata2[31:8],data_from_mem[31:24]}:
					 32'b0;
					 //依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata=(reg_write_value==4'b0000 &&(( inst[5:0]!=6'b001001 &&
	             inst[5:1]!=5'b00101  && inst[5:0]!=6'b100111 &&
			     inst[5:0]!=6'b101011)&&R_type==1 || R_type==0) )?wdata_option0000:
				                                //unify movn and movz

												//some R_type need handle
												//seperately
		         (reg_write_value==4'b0001)?wdata_option0001:
				 (reg_write_value==4'b0010 ||
			     reg_write_value==4'b0000 && inst[5:0]==6'b001001 && R_type==1
			     )?(pc_next_option00+4):
				                                //pc_next_option00 is defined below
				 (reg_write_value==4'b0011)?imm_sl16:
				 (reg_write_value==4'b0100 || 
				 reg_write_value==4'b0000 && inst[5:0]==6'b101011 && R_type==1
			     )?{31'b0,CarryOut}:
				 (reg_write_value==4'b0101)?wdata_lb:  //lb
				 (reg_write_value==4'b0110)?wdata_lbu: //lbu
				 (reg_write_value==4'b0111)?wdata_lh:  //lh
				 (reg_write_value==4'b1000)?wdata_lhu: //lhu
				 (reg_write_value==4'b1001)?wdata_lwl: //lwl
				 (reg_write_value==4'b1010)?wdata_lwr: //lwr
				 (reg_write_value==4'b0000 && inst[5:1]==5'b00101 && R_type==1)?rdata1:
												//unify movn and movz
				 //note reg_write_value==4'b000 only represent shoult write
				 //result, can not imply it is R_type
				 (reg_write_value==4'b0000 && inst[5:0]==6'b100111 && R_type==1)?~Result:
				 4'b0000;
				 //选择信号是译码阶段产生，供选信号有的是EX，有的是MEM产生

    //MUX,when the option is shift using sa, let a equal to sa instead of rs
	//controled by control unit and function field
	wire [31:0] A_option0;
	wire [31:0] A_option1;
	assign A_option0=rdata1;
	assign A_option1[4:0]=inst[10:6];  //only assign value to part of A_option1
    assign A=(R_type==1 && 
			 (inst[5:0]==6'b000000 || inst[5:0]==6'b000011 ||
			 inst[5:0]==6'b000010)
		     )?A_option1:A_option0;
	//0选项ID阶段产生（TODO），1选项ID阶段产生
	//选择信号ID阶段产生
	//EX阶段要使用它

	//PC
	//TODO，一会儿再看
	always @(posedge clk) begin
		if(resetn==0) begin
			PC<=32'Hbfc00000;
		end
		else if(PC_enable)begin
			PC<=pc_next;
		end
		//do not need PC<=PC
	end

	wire [31:0] pc_next;
	wire [31:0] pc_next_option00;
	wire [31:0] pc_next_option01;
	wire [31:0] pc_next_option10;
	wire [31:0] pc_next_option11;
	wire [1:0] pc_decider;

	assign pc_next_option00=PC+4;  //directly +4

	wire [31:0] offset;
	assign offset={sign_extended_imm[29:0],2'b00};
	assign pc_next_option01=pc_next_option00+offset;  //beq,bne(pc+offset)
	
	assign pc_next_option10={pc_next_option00[31:28],instr_index_sl2[27:0]};

	assign pc_next_option11=rdata1;

	assign pc_decider=(Zero==0 && bne==1)?2'b01:
		              (Zero==1 && beq==1 ||
					  regimm==1 && inst[20:16]==5'b00001 && Result[0]==0 ||//bgez
				      blez==1 && (Result[0]==1 || rdata1==32'b0) ||//blez
					  bgtz==1 && (Result[0]==0 && rdata1!=32'b0) ||//bgtz
					  regimm==1 && inst[20:16]==5'b00000 && Result[0]==1)?2'b01://bltz
					  (j==1 || jal==1)?2'b10:
					  (R_type==1 && inst[5:1]==5'b00100)?2'b11://unify jalr and jr
					  2'b00;

	assign pc_next=(pc_decider==2'b00)?pc_next_option00:
		           (pc_decider==2'b01)?pc_next_option01:
				   (pc_decider==2'b10)?pc_next_option10:
				   (pc_decider==2'b11)?pc_next_option11:
				   0;

	//mips_per_cnt_0   clk_counter
	reg [31:0] clk_counter;
	always @(posedge clk)
	begin
		if(resetn==0)
			clk_counter<=1;
		else
			clk_counter<=clk_counter+32'b1;
	end
	assign mips_perf_cnt_0=clk_counter;
endmodule

