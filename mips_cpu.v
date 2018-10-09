`timescale 10ns / 1ns

module mycpu_top(
	input  resetn,
	input  clk,

	output inst_sram_en,
	output [3:0] inst_sram_wen,
	output [31:0] inst_sram_addr,
	output [31:0] inst_sram_wdata,
	input [31:0] inst_sram_rdata,
	
	output data_sram_en,
	output [3:0] data_sram_wen,
	output [31:0] data_sram_addr,
	output [31:0] data_sram_wdata,
	input [31:0] data_sram_rdata,
	
	output reg [31:0] debug_wb_pc,
	output reg [3:0]debug_wb_rf_wen,
	output reg [4:0] debug_wb_rf_wnum,
	output reg [31:0] debug_wb_rf_wdata
);

    //辅助信号，帮助初始化
    reg just_rst;//为1表示刚刚rst变成1（或还未变成一），之前一个时钟是rst还是还是0。总之，它为1排除了那些rst已经拉高了很久的情况
    always@(posedge clk)
    begin
        if(resetn == 0)
            just_rst<=1;
        else
            just_rst<=0;
    end
	
	reg [31:0] PC;//在IF结束之前置为正确的值（这样ID阶段有正确的instr）
	              //是IF级的输出，不过不是哪一级的输入
	

    //寄存器流水
	reg [31:0] inst_ID_EX,inst_ID_MEM;
	reg [31:0] Result_EX_MEM;
	reg [1:0] reg_dst_ID_EX /*中间变量，过渡用*/,reg_dst_ID_MEM;
	reg mem_read_ID_EX;
	reg mem_write_ID_EX;
	reg [3:0] reg_write_value_ID_EX /*中间变量，过渡用*/,reg_write_value_ID_MEM;
	reg reg_write_ID_EX;
	reg [2:0] mem_write_value_ID_EX;
	reg wen_reg_file_EX_MEM;
	reg [31:0] rdata1_EX_MEM;
	reg [31:0] rdata2_EX_MEM;
	reg bne_ID_EX,beq_ID_EX,j_ID_EX,jal_ID_EX,R_type_ID_EX,R_type_ID_MEM,regimm_ID_EX,blez_ID_EX,bgtz_ID_EX;
	reg CarryOut_EX_MEM;
	reg [31:0] pc_next_option00_EX_MEM;
	reg [1:0] B_src_ID_EX;
	reg [3:0] ALUoperation_ID_EX;
	reg [4:0] raddr1_ID_EX,raddr2_ID_EX;
	//debug的流水寄存器
	reg [31:0] PC_EX,PC_MEM;
	reg [3:0] data_sram_wen_ID_EX;
	reg div_complete_MEM;
	reg just_div;//刚刚开始除法的标志
	reg div_complete_reg;//除法刚刚结束的标志
	//乘除法的寄存器
	reg mul_signed_ID_EX;
	reg MULT_ID_EX /*中间变量，过渡用*/,MULT_ID_MEM;
	reg MULTU_ID_EX /*中间变量，过渡用*/,MULTU_ID_MEM;
	reg div_signed_ID_EX;
	reg doingdiv_ID_EX,doingdiv_ID_MEM;
	reg MTHI_ID_EX,MTHI_ID_MEM;
	reg MTLO_ID_EX,MTLO_ID_MEM;
	reg MFHI_ID_EX;
	reg MFLO_ID_EX;
	reg [31:0] adjust_HI_EX_MEM;
	reg [31:0] adjust_LO_EX_MEM;
	
	
	//CPU简单的输出
	assign inst_sram_wen = 0;
	assign inst_sram_addr =/*(resetn==0)?32'Hbfc00000:*/pc_next;
	assign inst_sram_wdata = 0;
	assign data_sram_wen = data_sram_wen_ID_EX;
	
	always@(posedge clk)
	begin
	    if(doingdiv_ID_EX==0)
		begin
		    just_div<=1'b1;
		end
		else
		begin
		    just_div<=1'b0;
		end
		div_complete_reg<=div_complete;
	end
	
	//debug信号
	always@(posedge clk)
	begin
	    if(wen_reg_file_EX_MEM==1 & waddr!= 5'b00000 & (~doingdiv_ID_EX | (doingdiv_ID_EX & (just_div|div_complete_reg))))//不知道的just_div在连续除法的时候表现如何，TODO
	    begin
	        debug_wb_pc <= PC_MEM;
	        debug_wb_rf_wen <= {4{wen_reg_file_EX_MEM}};
	        debug_wb_rf_wnum <= waddr;
	        debug_wb_rf_wdata <= wdata;
	    end
	    else
	    begin
	        debug_wb_rf_wen <=4'b0;
	    end
	end

	wire [31:0] inst_ID;
	reg [31:0] old_inst;
	reg old_inst_update;
	//assign inst_ID=inst_sram_rdata;
	wire [31:0] data_from_mem;
    //assign data_from_mem=(ID_allowin==1)?data_sram_rdata:old_inst;
	assign data_from_mem=data_sram_rdata;
	assign inst_ID=(ID_allowin_reg==1)?inst_sram_rdata:old_inst;
	reg ID_allowin_reg;
	always@(posedge clk)
	begin
	    if(old_inst_update==1)
		begin
	        old_inst<=inst_sram_rdata;//这个比inst_ID慢一拍
		end
		ID_allowin_reg<=ID_allowin;
	end
	
	


    // define the signal related to main control
    wire [5:0] behavior;
	wire [31:0] Result_EX;
    wire [1:0] reg_dst_ID;
    wire mem_read_ID;
    wire [3:0] reg_write_value_ID;
    wire [2:0] ALUop;
	wire mem_write_ID;
	wire [1:0] B_src_ID;
    wire reg_write_ID;
	wire bne_ID,beq_ID,j_ID,jal_ID,R_type_ID,regimm_ID,blez_ID,bgtz_ID;
	wire [2:0] mem_write_value_ID;
    wire [3:0] data_sram_wen_ID;
	//add the control unit into the circuit
	control_unit cpu_control_unit(.clk(clk),.resetn(resetn),
	    .behavior(behavior),.Result(Result_EX),
		.reg_dst(reg_dst_ID),.mem_read(mem_read_ID),.reg_write_value(reg_write_value_ID),
		.ALUop(ALUop),.mem_write(mem_write_ID),.B_src(B_src_ID),.reg_write(reg_write_ID),
		.data_sram_wen(data_sram_wen_ID),.mem_write_value(mem_write_value_ID),
		//decoding signal
		.bne(bne_ID),.beq(beq_ID),.j(j_ID),.jal(jal_ID),.R_type(R_type_ID),
		.regimm(regimm_ID),.blez(blez_ID),.bgtz(bgtz_ID)
	);
    assign behavior=inst_ID[31:26];
	assign data_sram_en=mem_read_ID_EX| mem_write_ID_EX;
	
	//define the signal related to reg_file
	wire clk_reg_file;
	wire rst_reg_file;
	wire [4:0] waddr;
	wire [4:0] raddr1_ID;
	wire [4:0] raddr2_ID;
	wire wen_reg_file_EX;
	wire [31:0] wdata;
	wire [31:0] rdata1_EX;
	wire [31:0] adjust_rdata1_EX;//经过流水级修正的rdata1
	wire [31:0] rdata2_EX;
	wire [31:0] adjust_rdata2_EX;//经过流水级修正的rdata2
	//add the reg_flie into the circuit
	reg_file cpu_reg_file(.clk(clk_reg_file),.resetn(rst_reg_file),.waddr(waddr),.raddr1(raddr1_ID_EX),
		.raddr2(raddr2_ID_EX),.wen(wen_reg_file_EX_MEM),.wdata(wdata),.rdata1(rdata1_EX),.rdata2(rdata2_EX));
	assign clk_reg_file=clk;
	assign raddr1_ID=inst_ID[25:21];//rs
	assign raddr2_ID=inst_ID[20:16];//rt
	assign wen_reg_file_EX=(R_type_ID_EX==1 && inst_ID_EX[5:0]==6'b001011)?(adjust_rdata2_EX!=32'b0):
	                       (R_type_ID_EX==1 && inst_ID_EX[5:0]==6'b001010)?(adjust_rdata2_EX==32'b0):
						   (doingdiv_ID_EX ==1)?0:
	                       reg_write_ID_EX;//movn,movz
						   //对于普通的信号，译码完成就可以有效产生；对于movn和movz，可能需要EX才能得到正确的值
						   //reg_write是控制单元产生的，但是这个并不是完全的写使能控制信号，可能还受其他的控制
	assign rst_reg_file=resetn;
	assign adjust_rdata1_EX=(raddr1_ID_EX==waddr/*上一条指令的waddr*/ && waddr!=5'b0 && wen_reg_file_EX_MEM==1)?wdata:rdata1_EX;
	assign adjust_rdata2_EX=(raddr2_ID_EX==waddr/*上一条指令的waddr*/ && waddr!=5'b0 && wen_reg_file_EX_MEM==1)?wdata:rdata2_EX;

	//define the signal related to ALU
	wire [31:0] A;
	wire [31:0] B;
	wire [3:0] ALUoperation_ID;
	wire Overflow;
	wire CarryOut_EX;
	wire Zero;
	//add the ALU into the circuit
	alu cpu_alu(.A(A),.B(B),.ALUop(ALUoperation_ID_EX),.Overflow(Overflow),.CarryOut(CarryOut_EX),
		.Zero(Zero),.Result(Result_EX));
	assign data_sram_addr=Result_EX;

	//add the ALU control unit into the circuit
	ALU_control cpu_ALU_control(.func(inst_ID[5:0]),.ALUop(ALUop),//应该是ID阶段得到这个信号
		.ALU_ctr(ALUoperation_ID));
	
	//define the signal related to mul
	wire mul_signed_ID;
	wire [31:0] mul_x,mul_y;
	wire [63:0] mul_result;
	reg [31:0] HI,LO;//除法也用这两个寄存器
	//add the mul unit into the circuit
	mul cpu_mul(.mul_clk(clk),.resetn(resetn),.mul_signed(mul_signed_ID_EX),.x(mul_x),.y(mul_y),.result(mul_result));
	//EX阶段结束前输入，WB开始前得到结果(MEM阶段mul_result就有一部分有效了)

	wire MULT_ID;
	wire MULTU_ID;
	assign MULT_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b011000)?1:0;
	assign MULTU_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b011001)?1:0;
	assign mul_signed_ID=(MULT_ID==1)?1:
	                      (MULTU_ID==1)?0:
					      0;
	assign mul_x=adjust_rdata1_EX;//注意要用调整过后的
	assign mul_y=adjust_rdata2_EX;//注意要用调整过后的
	
	//define the signal related to div
	wire doingdiv_ID;
	//要求doingdiv在除法的EX过程中保持不变，只要下一条指令的译码结果没有进入EX阶段即可。
	//这是由流水线控制的，只需要除法指令还处在EX阶段，设好阻塞的话，doingdiv_ID_EX的值就不会更新成下一条指令的值，就满足条件了
	wire div_signed_ID;
	wire [31:0] div_x,div_y,div_s,div_r;
	wire div_complete;
	//add the div unit into the circuit
	div cpu_div(.div_clk(clk),.resetn(resetn),.div(doingdiv_ID_EX),
	    .div_signed(div_signed_ID_EX),.x(div_x),
	    .y(div_y),.s(div_s),.r(div_r),.complete(div_complete));
    
	wire DIV_ID;
	wire DIVU_ID;
	assign DIV_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b011010)?1:0;
	assign DIVU_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b011011)?1:0;
	assign doingdiv_ID=DIVU_ID | DIV_ID;
	assign div_signed_ID=(DIV_ID==1)?1:
	                     (DIVU_ID==1)?0:
						 0;
	assign div_x=adjust_rdata1_EX;//注意要用调整过后的
	assign div_y=adjust_rdata2_EX;//注意要用调整过后的
	//TODO：其实可以让DIV占用一个MEM周期，这样可能可以快一些，目前暂时不这么做。
    //TODO: complete当然要用于成为流水线阻塞的逻辑

	wire MTHI_ID;
	assign MTHI_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b010001)?1:0;
	wire MTLO_ID;
	assign MTLO_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b010011)?1:0;
	wire MFHI_ID;
	assign MFHI_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b010000)?1:0;
	wire MFLO_ID;
	assign MFLO_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b010010)?1:0;
	always@(posedge clk)
	begin
		{HI,LO}<=(MULT_ID_MEM==1 || MULTU_ID_MEM==1)?mul_result:
		    (doingdiv_ID_MEM ==1)?{div_r,div_s}:
			(MTHI_ID_MEM==1)?{rdata1_EX_MEM,LO}:
			(MTLO_ID_MEM==1)?{HI,rdata1_EX_MEM}:
			{HI,LO};
	end
	
	wire [31:0] adjust_HI_EX;
	assign adjust_HI_EX=(MFHI_ID_EX == 1 && (MULT_ID_MEM==1 || MULTU_ID_MEM==1))?mul_result[63:32]:
	    (MFHI_ID_EX == 1 && (doingdiv_ID_MEM ==1))?div_r:
		(MFHI_ID_EX == 1 && MTHI_ID_MEM==1)?rdata1_EX_MEM:
		HI;
	wire [31:0] adjust_LO_EX;
	assign adjust_LO_EX=(MFLO_ID_EX ==1 && (MULT_ID_MEM==1 || MULTU_ID_MEM==1))?mul_result[31:0]:
	    (MFLO_ID_EX == 1 && (doingdiv_ID_MEM ==1))?div_s:
		(MFLO_ID_EX == 1 && MTLO_ID_MEM==1)?rdata1_EX_MEM:
	    LO;
	
	//MUX, what to write to memory
	//这些信号的产生，需要EX阶段读RF以及ALU的结果得到。它们应该在EX阶段产生。
	//被用来在MEM阶段写内存，需要有控制信号和选择信号配合，也需要与addr配合（TODO）
	wire [31:0] write_data_sb;
	wire [31:0] write_data_sh;
	wire [31:0] write_data_swl;
	wire [31:0] write_data_swr;
	assign write_data_sb={4{adjust_rdata2_EX[7:0]}};  //unify 4 situations of result
	assign write_data_sh={2{adjust_rdata2_EX[15:0]}};  //unify 4 situations of result
	assign write_data_swl=(Result_EX[1:0]==2'b00)?{24'b0,adjust_rdata2_EX[31:14]}:
						  (Result_EX[1:0]==2'b01)?{16'b0,adjust_rdata2_EX[31:16]}:
						  (Result_EX[1:0]==2'b10)?{8'b0,adjust_rdata2_EX[31:8]}:
						  (Result_EX[1:0]==2'b11)?adjust_rdata2_EX:
						  32'b0;
	assign write_data_swr=(Result_EX[1:0]==2'b00)?adjust_rdata2_EX:
						  (Result_EX[1:0]==2'b01)?{adjust_rdata2_EX[23:0],8'b0}:
						  (Result_EX[1:0]==2'b10)?{adjust_rdata2_EX[15:0],16'b0}:
						  (Result_EX[1:0]==2'b11)?{adjust_rdata2_EX[7:0],24'b0}:
						  32'b0;
	assign data_sram_wdata=(mem_write_value_ID_EX==3'b000)?adjust_rdata2_EX:
		              (mem_write_value_ID_EX==3'b001)?write_data_sb:
				      (mem_write_value_ID_EX==3'b010)?write_data_sh:
					  (mem_write_value_ID_EX==3'b011)?write_data_swl:
					  (mem_write_value_ID_EX==3'b100)?write_data_swr:
					  adjust_rdata2_EX;

	//MUX, where to write, decide 'waddr'
	//在译码阶段就可以产生出备选地址
	//不知道选择信号怎么产生的。估计也是译码（TODO）
	//在MEM阶段使用这些信号
	wire [4:0] waddr_option00;
	wire [4:0] waddr_option01;
	assign waddr_option00=inst_ID_MEM[20:16];//TODO,not sure,是不是mem阶段使用的这些，只需要在EX的最后阶段准备好就可以了
	assign waddr_option01=inst_ID_MEM[15:11];//TODO,not sure,是不是mem阶段使用的这些，只需要在EX的最后阶段准备好就可以了
	assign waddr=(reg_dst_ID_MEM==2'b00)? waddr_option00: 
		         (reg_dst_ID_MEM==2'b01)? waddr_option01:
				 (reg_dst_ID_MEM==2'b10)? 5'b11111:
				 5'b00000;
	
	//MUX, what to compute, decide  'B'
	//00选项需要EX阶段产生，其他的选项ID之后就有效了
	//选择信号信号在ID阶段就产生
	wire [31:0] B_option00;
	wire [31:0] B_option01;
	wire [31:0] B_option10;
	wire [31:0] B_option11;
	assign B_option00=adjust_rdata2_EX;
	assign B_option01={{16{inst_ID_EX[15]}},inst_ID_EX[15:0]};
	assign B_option10=32'b0;
	assign B_option11={16'b0,inst_ID_EX[15:0]};
	assign B=(B_src_ID_EX==2'b00)? B_option00:
		     (B_src_ID_EX==2'b01)? B_option01:
			 (B_src_ID_EX==2'b10)? B_option10:
			 (B_src_ID_EX==2'b11)? B_option11:
			 adjust_rdata2_EX;

	//MUX,what to write to reg_flie, decide 'wdata'
	wire [31:0] wdata_option0000;
	wire [31:0] wdata_option0001;
	wire [31:0] wdata_lb;
	wire [31:0] wdata_lbu;
	wire [31:0] wdata_lh;
	wire [31:0] wdata_lhu;
	wire [31:0] wdata_lwl;
	wire [31:0] wdata_lwr;
	assign wdata_option0000=Result_EX_MEM;//EX阶段产生,但是使用要在MEM阶段最后（WB之前有效）
	assign wdata_option0001=data_from_mem;//MEM阶段产生
	assign wdata_lb=(Result_EX_MEM[1:0]==2'b00)?{{24{data_from_mem[7]}},data_from_mem[7:0]}:
				    (Result_EX_MEM[1:0]==2'b01)?{{24{data_from_mem[15]}},data_from_mem[15:8]}:
					(Result_EX_MEM[1:0]==2'b10)?{{24{data_from_mem[23]}},data_from_mem[23:16]}:
					(Result_EX_MEM[1:0]==2'b11)?{{24{data_from_mem[31]}},data_from_mem[31:24]}:
					32'b0;
					//依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lbu=(Result_EX_MEM[1:0]==2'b00)?{24'b0,data_from_mem[7:0]}:
					 (Result_EX_MEM[1:0]==2'b01)?{24'b0,data_from_mem[15:8]}:
					 (Result_EX_MEM[1:0]==2'b10)?{24'b0,data_from_mem[23:16]}:
					 (Result_EX_MEM[1:0]==2'b11)?{24'b0,data_from_mem[31:24]}:
					 32'b0;
					 //依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lh=(Result_EX_MEM[1]==0)?{{16{data_from_mem[15]}},data_from_mem[15:0]}:
		            (Result_EX_MEM[1]==1)?{{16{data_from_mem[31]}},data_from_mem[31:16]}:
					32'b0;
					//依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lhu=(Result_EX_MEM[1]==0)?{16'b0,data_from_mem[15:0]}:
					 (Result_EX_MEM[1]==1)?{16'b0,data_from_mem[31:16]}:
					 32'b0;
					 //依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lwl=(Result_EX_MEM[1:0]==2'b00)?{data_from_mem[7:0],rdata2_EX_MEM[23:0]}:
					 (Result_EX_MEM[1:0]==2'b01)?{data_from_mem[15:0],rdata2_EX_MEM[15:0]}:
					 (Result_EX_MEM[1:0]==2'b10)?{data_from_mem[23:0],rdata2_EX_MEM[7:0]}:
					 (Result_EX_MEM[1:0]==2'b11)?data_from_mem[31:0]:
					 32'b0;
					 //依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata_lwr=(Result_EX_MEM[1:0]==2'b00)?data_from_mem[31:0]:
				     (Result_EX_MEM[1:0]==2'b01)?{rdata2_EX_MEM[31:24],data_from_mem[31:8]}:
					 (Result_EX_MEM[1:0]==2'b10)?{rdata2_EX_MEM[31:16],data_from_mem[31:16]}:
					 (Result_EX_MEM[1:0]==2'b11)?{rdata2_EX_MEM[31:8],data_from_mem[31:24]}:
					 32'b0;
					 //依据EX阶段产生的RESULT，以及MEM阶段的结果产生
	assign wdata=(reg_write_value_ID_MEM==4'b0000 &&(( inst_ID_MEM[5:0]!=6'b001001 &&
	             inst_ID_MEM[5:1]!=5'b00101  && inst_ID_MEM[5:0]!=6'b100111 &&
			     inst_ID_MEM[5:0]!=6'b101011 && inst_ID_MEM[5:0]!=6'b010000 &&
				 inst_ID_MEM[5:0]!=6'b010010)&&R_type_ID_MEM==1 || R_type_ID_MEM==0) )?wdata_option0000:
				                                //unify movn and movz

												//some R_type need handle
												//seperately
		         (reg_write_value_ID_MEM===4'b0001)?wdata_option0001:
				 (reg_write_value_ID_MEM===4'b0010/*jal*/ ||
			     reg_write_value_ID_MEM===4'b0000 && inst_ID_MEM[5:0]==6'b001001 && R_type_ID_MEM==1
			     )?(pc_next_option00_EX_MEM/*+4*//*不应该有加4了*/):
				                                //pc_next_option00 is defined below
				 (reg_write_value_ID_MEM===4'b0011)?{inst_ID_MEM[15:0],16'b0}:
				 (reg_write_value_ID_MEM===4'b0100 || 
				 reg_write_value_ID_MEM===4'b0000 && inst_ID_MEM[5:0]==6'b101011 && R_type_ID_MEM==1
			     )?{31'b0,CarryOut_EX_MEM}:
				 (reg_write_value_ID_MEM===4'b0101)?wdata_lb:  //lb
				 (reg_write_value_ID_MEM===4'b0110)?wdata_lbu: //lbu
				 (reg_write_value_ID_MEM===4'b0111)?wdata_lh:  //lh
				 (reg_write_value_ID_MEM===4'b1000)?wdata_lhu: //lhu
				 (reg_write_value_ID_MEM===4'b1001)?wdata_lwl: //lwl
				 (reg_write_value_ID_MEM===4'b1010)?wdata_lwr: //lwr
				 (reg_write_value_ID_MEM===4'b0000 && inst_ID_MEM[5:1]==5'b00101 && R_type_ID_MEM==1)?rdata1_EX_MEM:
												//unify movn and movz
				 //note reg_write_value_ID_MEM===4'b000 only represent shoult write
				 //result, can not imply it is R_type
				 (reg_write_value_ID_MEM===4'b0000 && inst_ID_MEM[5:0]==6'b100111 && R_type_ID_MEM==1)?~Result_EX_MEM:
				 (reg_write_value_ID_MEM==4'b0000 && inst_ID_MEM[5:0]==6'b010000)?adjust_HI_EX_MEM:
				 (reg_write_value_ID_MEM==4'b0000 && inst_ID_MEM[5:0]==6'b010010)?adjust_LO_EX_MEM:
				 4'b0000;
				 //选择信号是译码阶段产生，供选信号有的是EX，有的是MEM产生

    //MUX,when the option is shift using sa, let a equal to sa instead of rs
	//controled by control unit and function field
	wire [31:0] A_option0;
	wire [31:0] A_option1;
	assign A_option0=adjust_rdata1_EX;
	assign A_option1[4:0]=inst_ID_EX[10:6];  //only assign value to part of A_option1
	                                        //这大约是移位指令
    assign A=(R_type_ID_EX==1 && 
			 (inst_ID_EX[5:0]==6'b000000 || inst_ID_EX[5:0]==6'b000011 ||
			 inst_ID_EX[5:0]==6'b000010)
		     )?A_option1:A_option0;

	//PC
	always @(posedge clk) begin
	    if (resetn==0)
	        PC<=32'Hbfc00000;
	    if(1 && ID_allowin)begin//只有流水的情况下更新
			PC<=pc_next;
			//在ID阶段，PC就是当前被处理的指令的PC值
			//PC只是指令的一个普通的参数。跟指令的译码结果没什么本质区别，一同对待
		end
		//do not need PC<=PC
	end

	wire [31:0] pc_next;
	wire [31:0] pc_next_option00_EX;
	wire [31:0] pc_next_option01;
	wire [31:0] pc_next_option10;
	wire [31:0] pc_next_option11;
	wire [1:0] pc_decider;

	//ID阶段结束后，各个选项都有了，但是decider需要等到EX结束时才可以。
	//IF阶段，是上上条指令的EX阶段，依据这个EX的结果，判断选择跳向分支还是继续执行
	//与之匹配的，option00用的永远是上条指令的PC加4
	assign pc_next_option00_EX=PC+4;  //directly +4
	                                  //在每条指令的EX阶段，可以确定下下条指令的地址。这时候PC的值是下条指令的地址，加4得到下下条指令的地址
	assign pc_next_option01=PC+{{{14{inst_ID_EX[15]}},inst_ID_EX[15:0]},2'b00};  //beq,bne(pc+offset)
	                                                                             //在外部已有延迟槽的时候，不需要从加4之后的基础上再加，故作更改
	assign pc_next_option10={PC[31:28],{inst_ID_EX[25:0],2'b00}};//在外部已有延迟槽的时候，不需要从加4之后的基础上再加，故作更改
    assign pc_next_option11=adjust_rdata1_EX;
    assign pc_decider=(Zero==0 && bne_ID_EX==1)?2'b01:
		              (Zero==1 && beq_ID_EX==1 ||
					  regimm_ID_EX==1 && inst_ID_EX[20:16]==5'b00001 && Result_EX[0]==0 ||//bgez
				      blez_ID_EX==1 && (Result_EX[0]==1 || adjust_rdata1_EX==32'b0) ||//blez
					  bgtz_ID_EX==1 && (Result_EX[0]==0 && adjust_rdata1_EX!=32'b0) ||//bgtz
					  regimm_ID_EX==1 && inst_ID_EX[20:16]==5'b00000 && Result_EX[0]==1)?2'b01://bltz
					  (j_ID_EX==1 || jal_ID_EX==1)?2'b10:
					  (R_type_ID_EX==1 && inst_ID_EX[5:1]==5'b00100)?2'b11://unify jalr and jr
					  2'b00;
	assign pc_next=(resetn==0)?32'Hbfc00000:
	               (just_rst==1)?32'Hbfc00004:
	               (pc_decider==2'b00)?pc_next_option00_EX:
		           (pc_decider==2'b01)?pc_next_option01:
				   (pc_decider==2'b10)?pc_next_option10:
				   (pc_decider==2'b11)?pc_next_option11:
				   0;
	
	reg ID_valid;
	reg EX_valid;
	reg MEM_valid;
	//寄存器流水
	assign inst_sram_en=(1)/*IF->ID*/?1:0;
	
	//ID
	wire ID_allowin;
	wire ID_ready2go;
	wire ID2EX_valid;
	assign ID_ready2go=1;
	assign ID_allowin=~ID_valid || ID_ready2go && EX_allowin;
	assign ID2EX_valid=ID_valid && ID_ready2go;
    always @(posedge clk)
	begin
	    if(resetn==1'b0 )
		begin
		    ID_valid<=1'b1;//复位一段时间后，实际上已经充满了第一条指令。所以ID实际是有的
		end
		else if(ID_allowin)
		begin
		    ID_valid<=1'b1;//ID阶段读到的一定是准确的，只要允许它读的话
		end
		if(1 && ID_allowin)
		begin
		    //更新ID阶段的数据；
			//复位后，ID_valid为1，ID是空的，所以ID_allowin是1，这个if为真
			//由于很多_ID的数据我们用的是组合给它赋值的，所以即使这里的if不满足，_ID的数据也会更新
			//这样就会导致原来的_ID丢失
			
			//inst_ID
			//PC
			//控制单元的输出，这当然依赖于inst_ID
			//乘除法的控制信号，也以来于inst_ID
			//综上，控制PC和inst_ID保持不变
			
			//如果后面堵住了，用来取值的PC可能也不要更新，但是我取值用到PC_next,是组合逻辑来更新。
			//我们的PC_next，本质是EX阶段知道下下条指令的PC，而下条指令的PC是固定的，不用猜
			
			//堵了的话，IF阶段那里已经流入了一个PC了（IF阶段就可以得到inst_ID），只需要让这个PC保持住，那么开始流动时的时候IF的inst_ID就是这个PC对应的
			//担心的时，这个PC的指令给到inst_sram，读出来新指令。并且覆盖inst_ID。这个指令当然丢了时没关系的，只需要PC保持住了即可
			
			//同时，有一个PC即将要进入IF阶段（刚算出来的地址,当然，这个地址不一定是正确的PC，正确的PC需要EX阶段执行完）
			//这个即将进入IF的PC，只需要计算它的那些寄存器不丢失就可以了，这依赖于别的。我们的PC，依据是EX阶段的寄存器，而不可能在ID阶段堵住
			//（流线线前面堵住，后面会把指令流干）所以EX阶段的寄存器在堵住的时候不会流干，所以这个PC会保持住，直到继续流动（EX阶段堵，则会持续更新）
			old_inst_update<=1'b1;
		end
		else
		begin
		    old_inst_update<=1'b0;
			//某个时钟沿前，有堵住信号。这个时钟沿后的周期内，old_inst为0，old_inst在这个周期结束的时候不准更新
			//某个时钟沿前，没有堵住，这个时钟沿后的周期T内，old_inst为1，在这个周期结束后，old_inst可以更新成T时刻内inst_sram的输出
		end
	end
	
	//EX
    wire EX_allowin;
	wire EX_ready2go;
	wire EX2MEM_valid;
	assign EX_ready2go=~(doingdiv_ID_EX && ~div_complete);//体现了除法的阻塞作用
	    //这个是表明处于EX阶段的指令有没有做完，这是决定EX是否可以结束的
		//这个，只需要在EX阶段结束之前及时变化即可
		//我这种写法能保证，所有的指令都会及时移走，这就够了。
		//并不用担心doingdiv_ID能否被取进来（只要上一条能结束即可，上一条按这种写法可以结束）
    assign EX_allowin=~EX_valid || EX_ready2go && MEM_allowin || just_rst;//初始化的时候要改一下
	assign EX2MEM_valid=EX_valid && EX_ready2go;
	always@(posedge clk)
	begin
	    if(resetn==0)
		begin
		    EX_valid<=1'b1;//复位一段时间后，实际上已经充满了第一条指令。所以EX实际是有的,TODO
			              //这样保证第一条的寄存器可以传到后面去
		end
		else if(EX_allowin)
		begin
		    EX_valid<=ID2EX_valid;
		end
		if(ID2EX_valid && EX_allowin)
		begin//ID->EX
		    inst_ID_EX<=inst_ID;
			reg_dst_ID_EX <= reg_dst_ID;
			mem_read_ID_EX<=mem_read_ID;
			mem_write_ID_EX<=mem_write_ID;
			reg_write_value_ID_EX <= reg_write_value_ID;
			reg_write_ID_EX<=reg_write_ID;
			mem_write_value_ID_EX<=mem_write_value_ID;
			//waddr_ID_EX<=waddr_ID;
			bne_ID_EX<=bne_ID;
			beq_ID_EX<=beq_ID;
			j_ID_EX<=j_ID;
			jal_ID_EX<=jal_ID;
			R_type_ID_EX<=R_type_ID;
			regimm_ID_EX<=regimm_ID;
			blez_ID_EX<=blez_ID;
			bgtz_ID_EX<=bgtz_ID;
			PC_EX<=PC;
			//A_ID_EX<=A_ID;
			//B_ID_EX<=B_ID;
			B_src_ID_EX<=B_src_ID;
			ALUoperation_ID_EX<=ALUoperation_ID;
			raddr1_ID_EX<=raddr1_ID;
			raddr2_ID_EX<=raddr2_ID;
			data_sram_wen_ID_EX<=data_sram_wen_ID;
			mul_signed_ID_EX<=mul_signed_ID;
			MULT_ID_EX<=MULT_ID;
			MULTU_ID_EX<=MULTU_ID;
			div_signed_ID_EX<=div_signed_ID;
			doingdiv_ID_EX<=doingdiv_ID;
			MTHI_ID_EX<=MTHI_ID;
			MTLO_ID_EX<=MTLO_ID;
			MFHI_ID_EX<=MFHI_ID;
			MFLO_ID_EX<=MFLO_ID;
		end
	end
	
	//MEM
	wire MEM_allowin;
	wire MEM_ready2go;
	//不需要MEM2WB_valid了，因为不关心可不可发送，永远都从流水级中输出
	assign MEM_ready2go=1;
	assign MEM_allowin=~MEM_valid || MEM_ready2go && 1/*out_allow=1*/;
	always@(posedge clk)
	begin
	    if(resetn==0)
		begin
		    MEM_valid<=1'b1;//复位一段时间后，实际上已经充满了第一条指令。所以MEM实际是有的,TODO
		end
		else if(MEM_allowin)
		begin
		    MEM_valid<=EX2MEM_valid;//ID阶段读到的一定是准确的，只要允许它读的话
		end
		if(EX2MEM_valid && MEM_allowin)
		begin
		    inst_ID_MEM<=inst_ID_EX;
			Result_EX_MEM<=Result_EX;
			reg_dst_ID_MEM <= reg_dst_ID_EX;
			reg_write_value_ID_MEM<=reg_write_value_ID_EX;
			//waddr_ID_MEM<=waddr_ID_EX;
			wen_reg_file_EX_MEM<=wen_reg_file_EX;
			rdata1_EX_MEM<=adjust_rdata1_EX;
			rdata2_EX_MEM<=adjust_rdata2_EX;
			R_type_ID_MEM<=R_type_ID_EX;
			CarryOut_EX_MEM<=CarryOut_EX;
		    pc_next_option00_EX_MEM<=pc_next_option00_EX;
		    PC_MEM<=PC_EX;
			MULT_ID_MEM<=MULT_ID_EX;
			MULTU_ID_MEM<=MULTU_ID_EX;
			doingdiv_ID_MEM<=doingdiv_ID_EX;
			MTHI_ID_MEM<=MTHI_ID_EX;
			MTLO_ID_MEM<=MTLO_ID_EX;
			div_complete_MEM<=div_complete;
			adjust_HI_EX_MEM<=adjust_HI_EX;
			adjust_LO_EX_MEM<=adjust_LO_EX;
			//只要左边的不被错误覆盖就可以了
			//右边，有的是EX阶段依据EX阶段的寄存器产生的值，有的是EX阶段从ID阶段收过来的值
			//这两种没有区别。本质都是EX阶段的寄存器传过来而已（只不过不是直接赋值，有一些逻辑）。
		end
	end
endmodule

