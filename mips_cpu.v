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

    //�����źţ�������ʼ��
    reg just_rst;//Ϊ1��ʾ�ո�rst���1����δ���һ����֮ǰһ��ʱ����rst���ǻ���0����֮����Ϊ1�ų�����Щrst�Ѿ������˺ܾõ����
    always@(posedge clk)
    begin
        if(resetn == 0)
            just_rst<=1;
        else
            just_rst<=0;
    end
	
	reg [31:0] PC;//��IF����֮ǰ��Ϊ��ȷ��ֵ������ID�׶�����ȷ��instr��
	              //��IF�������������������һ��������
	

    //�Ĵ�����ˮ
	reg [31:0] inst_ID_EX,inst_ID_MEM;
	reg [31:0] Result_EX_MEM;
	reg [1:0] reg_dst_ID_EX /*�м������������*/,reg_dst_ID_MEM;
	reg mem_read_ID_EX;
	reg mem_write_ID_EX;
	reg [3:0] reg_write_value_ID_EX /*�м������������*/,reg_write_value_ID_MEM;
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
	//debug����ˮ�Ĵ���
	reg [31:0] PC_EX,PC_MEM;
	reg [3:0] data_sram_wen_ID_EX;
	reg div_complete_MEM;
	reg just_div;//�ոտ�ʼ�����ı�־
	reg div_complete_reg;//�����ոս����ı�־
	//�˳����ļĴ���
	reg mul_signed_ID_EX;
	reg MULT_ID_EX /*�м������������*/,MULT_ID_MEM;
	reg MULTU_ID_EX /*�м������������*/,MULTU_ID_MEM;
	reg div_signed_ID_EX;
	reg doingdiv_ID_EX,doingdiv_ID_MEM;
	reg MTHI_ID_EX,MTHI_ID_MEM;
	reg MTLO_ID_EX,MTLO_ID_MEM;
	reg MFHI_ID_EX;
	reg MFLO_ID_EX;
	reg [31:0] adjust_HI_EX_MEM;
	reg [31:0] adjust_LO_EX_MEM;
	
	
	//CPU�򵥵����
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
	
	//debug�ź�
	always@(posedge clk)
	begin
	    if(wen_reg_file_EX_MEM==1 & waddr!= 5'b00000 & (~doingdiv_ID_EX | (doingdiv_ID_EX & (just_div|div_complete_reg))))//��֪����just_div������������ʱ�������Σ�TODO
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
	        old_inst<=inst_sram_rdata;//�����inst_ID��һ��
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
	wire [31:0] adjust_rdata1_EX;//������ˮ��������rdata1
	wire [31:0] rdata2_EX;
	wire [31:0] adjust_rdata2_EX;//������ˮ��������rdata2
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
						   //������ͨ���źţ�������ɾͿ�����Ч����������movn��movz��������ҪEX���ܵõ���ȷ��ֵ
						   //reg_write�ǿ��Ƶ�Ԫ�����ģ����������������ȫ��дʹ�ܿ����źţ����ܻ��������Ŀ���
	assign rst_reg_file=resetn;
	assign adjust_rdata1_EX=(raddr1_ID_EX==waddr/*��һ��ָ���waddr*/ && waddr!=5'b0 && wen_reg_file_EX_MEM==1)?wdata:rdata1_EX;
	assign adjust_rdata2_EX=(raddr2_ID_EX==waddr/*��һ��ָ���waddr*/ && waddr!=5'b0 && wen_reg_file_EX_MEM==1)?wdata:rdata2_EX;

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
	ALU_control cpu_ALU_control(.func(inst_ID[5:0]),.ALUop(ALUop),//Ӧ����ID�׶εõ�����ź�
		.ALU_ctr(ALUoperation_ID));
	
	//define the signal related to mul
	wire mul_signed_ID;
	wire [31:0] mul_x,mul_y;
	wire [63:0] mul_result;
	reg [31:0] HI,LO;//����Ҳ���������Ĵ���
	//add the mul unit into the circuit
	mul cpu_mul(.mul_clk(clk),.resetn(resetn),.mul_signed(mul_signed_ID_EX),.x(mul_x),.y(mul_y),.result(mul_result));
	//EX�׶ν���ǰ���룬WB��ʼǰ�õ����(MEM�׶�mul_result����һ������Ч��)

	wire MULT_ID;
	wire MULTU_ID;
	assign MULT_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b011000)?1:0;
	assign MULTU_ID=(inst_ID[31:26]==6'b000000 && inst_ID[5:0]==6'b011001)?1:0;
	assign mul_signed_ID=(MULT_ID==1)?1:
	                      (MULTU_ID==1)?0:
					      0;
	assign mul_x=adjust_rdata1_EX;//ע��Ҫ�õ��������
	assign mul_y=adjust_rdata2_EX;//ע��Ҫ�õ��������
	
	//define the signal related to div
	wire doingdiv_ID;
	//Ҫ��doingdiv�ڳ�����EX�����б��ֲ��䣬ֻҪ��һ��ָ���������û�н���EX�׶μ��ɡ�
	//��������ˮ�߿��Ƶģ�ֻ��Ҫ����ָ�����EX�׶Σ���������Ļ���doingdiv_ID_EX��ֵ�Ͳ�����³���һ��ָ���ֵ��������������
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
	assign div_x=adjust_rdata1_EX;//ע��Ҫ�õ��������
	assign div_y=adjust_rdata2_EX;//ע��Ҫ�õ��������
	//TODO����ʵ������DIVռ��һ��MEM���ڣ��������ܿ��Կ�һЩ��Ŀǰ��ʱ����ô����
    //TODO: complete��ȻҪ���ڳ�Ϊ��ˮ���������߼�

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
	//��Щ�źŵĲ�������ҪEX�׶ζ�RF�Լ�ALU�Ľ���õ�������Ӧ����EX�׶β�����
	//��������MEM�׶�д�ڴ棬��Ҫ�п����źź�ѡ���ź���ϣ�Ҳ��Ҫ��addr��ϣ�TODO��
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
	//������׶ξͿ��Բ�������ѡ��ַ
	//��֪��ѡ���ź���ô�����ġ�����Ҳ�����루TODO��
	//��MEM�׶�ʹ����Щ�ź�
	wire [4:0] waddr_option00;
	wire [4:0] waddr_option01;
	assign waddr_option00=inst_ID_MEM[20:16];//TODO,not sure,�ǲ���mem�׶�ʹ�õ���Щ��ֻ��Ҫ��EX�����׶�׼���þͿ�����
	assign waddr_option01=inst_ID_MEM[15:11];//TODO,not sure,�ǲ���mem�׶�ʹ�õ���Щ��ֻ��Ҫ��EX�����׶�׼���þͿ�����
	assign waddr=(reg_dst_ID_MEM==2'b00)? waddr_option00: 
		         (reg_dst_ID_MEM==2'b01)? waddr_option01:
				 (reg_dst_ID_MEM==2'b10)? 5'b11111:
				 5'b00000;
	
	//MUX, what to compute, decide  'B'
	//00ѡ����ҪEX�׶β�����������ѡ��ID֮�����Ч��
	//ѡ���ź��ź���ID�׶ξͲ���
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
	assign wdata_option0000=Result_EX_MEM;//EX�׶β���,����ʹ��Ҫ��MEM�׶����WB֮ǰ��Ч��
	assign wdata_option0001=data_from_mem;//MEM�׶β���
	assign wdata_lb=(Result_EX_MEM[1:0]==2'b00)?{{24{data_from_mem[7]}},data_from_mem[7:0]}:
				    (Result_EX_MEM[1:0]==2'b01)?{{24{data_from_mem[15]}},data_from_mem[15:8]}:
					(Result_EX_MEM[1:0]==2'b10)?{{24{data_from_mem[23]}},data_from_mem[23:16]}:
					(Result_EX_MEM[1:0]==2'b11)?{{24{data_from_mem[31]}},data_from_mem[31:24]}:
					32'b0;
					//����EX�׶β�����RESULT���Լ�MEM�׶εĽ������
	assign wdata_lbu=(Result_EX_MEM[1:0]==2'b00)?{24'b0,data_from_mem[7:0]}:
					 (Result_EX_MEM[1:0]==2'b01)?{24'b0,data_from_mem[15:8]}:
					 (Result_EX_MEM[1:0]==2'b10)?{24'b0,data_from_mem[23:16]}:
					 (Result_EX_MEM[1:0]==2'b11)?{24'b0,data_from_mem[31:24]}:
					 32'b0;
					 //����EX�׶β�����RESULT���Լ�MEM�׶εĽ������
	assign wdata_lh=(Result_EX_MEM[1]==0)?{{16{data_from_mem[15]}},data_from_mem[15:0]}:
		            (Result_EX_MEM[1]==1)?{{16{data_from_mem[31]}},data_from_mem[31:16]}:
					32'b0;
					//����EX�׶β�����RESULT���Լ�MEM�׶εĽ������
	assign wdata_lhu=(Result_EX_MEM[1]==0)?{16'b0,data_from_mem[15:0]}:
					 (Result_EX_MEM[1]==1)?{16'b0,data_from_mem[31:16]}:
					 32'b0;
					 //����EX�׶β�����RESULT���Լ�MEM�׶εĽ������
	assign wdata_lwl=(Result_EX_MEM[1:0]==2'b00)?{data_from_mem[7:0],rdata2_EX_MEM[23:0]}:
					 (Result_EX_MEM[1:0]==2'b01)?{data_from_mem[15:0],rdata2_EX_MEM[15:0]}:
					 (Result_EX_MEM[1:0]==2'b10)?{data_from_mem[23:0],rdata2_EX_MEM[7:0]}:
					 (Result_EX_MEM[1:0]==2'b11)?data_from_mem[31:0]:
					 32'b0;
					 //����EX�׶β�����RESULT���Լ�MEM�׶εĽ������
	assign wdata_lwr=(Result_EX_MEM[1:0]==2'b00)?data_from_mem[31:0]:
				     (Result_EX_MEM[1:0]==2'b01)?{rdata2_EX_MEM[31:24],data_from_mem[31:8]}:
					 (Result_EX_MEM[1:0]==2'b10)?{rdata2_EX_MEM[31:16],data_from_mem[31:16]}:
					 (Result_EX_MEM[1:0]==2'b11)?{rdata2_EX_MEM[31:8],data_from_mem[31:24]}:
					 32'b0;
					 //����EX�׶β�����RESULT���Լ�MEM�׶εĽ������
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
			     )?(pc_next_option00_EX_MEM/*+4*//*��Ӧ���м�4��*/):
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
				 //ѡ���ź�������׶β�������ѡ�ź��е���EX���е���MEM����

    //MUX,when the option is shift using sa, let a equal to sa instead of rs
	//controled by control unit and function field
	wire [31:0] A_option0;
	wire [31:0] A_option1;
	assign A_option0=adjust_rdata1_EX;
	assign A_option1[4:0]=inst_ID_EX[10:6];  //only assign value to part of A_option1
	                                        //���Լ����λָ��
    assign A=(R_type_ID_EX==1 && 
			 (inst_ID_EX[5:0]==6'b000000 || inst_ID_EX[5:0]==6'b000011 ||
			 inst_ID_EX[5:0]==6'b000010)
		     )?A_option1:A_option0;

	//PC
	always @(posedge clk) begin
	    if (resetn==0)
	        PC<=32'Hbfc00000;
	    if(1 && ID_allowin)begin//ֻ����ˮ������¸���
			PC<=pc_next;
			//��ID�׶Σ�PC���ǵ�ǰ�������ָ���PCֵ
			//PCֻ��ָ���һ����ͨ�Ĳ�������ָ���������ûʲô��������һͬ�Դ�
		end
		//do not need PC<=PC
	end

	wire [31:0] pc_next;
	wire [31:0] pc_next_option00_EX;
	wire [31:0] pc_next_option01;
	wire [31:0] pc_next_option10;
	wire [31:0] pc_next_option11;
	wire [1:0] pc_decider;

	//ID�׶ν����󣬸���ѡ����ˣ�����decider��Ҫ�ȵ�EX����ʱ�ſ��ԡ�
	//IF�׶Σ���������ָ���EX�׶Σ��������EX�Ľ�����ж�ѡ�������֧���Ǽ���ִ��
	//��֮ƥ��ģ�option00�õ���Զ������ָ���PC��4
	assign pc_next_option00_EX=PC+4;  //directly +4
	                                  //��ÿ��ָ���EX�׶Σ�����ȷ��������ָ��ĵ�ַ����ʱ��PC��ֵ������ָ��ĵ�ַ����4�õ�������ָ��ĵ�ַ
	assign pc_next_option01=PC+{{{14{inst_ID_EX[15]}},inst_ID_EX[15:0]},2'b00};  //beq,bne(pc+offset)
	                                                                             //���ⲿ�����ӳٲ۵�ʱ�򣬲���Ҫ�Ӽ�4֮��Ļ������ټӣ���������
	assign pc_next_option10={PC[31:28],{inst_ID_EX[25:0],2'b00}};//���ⲿ�����ӳٲ۵�ʱ�򣬲���Ҫ�Ӽ�4֮��Ļ������ټӣ���������
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
	//�Ĵ�����ˮ
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
		    ID_valid<=1'b1;//��λһ��ʱ���ʵ�����Ѿ������˵�һ��ָ�����IDʵ�����е�
		end
		else if(ID_allowin)
		begin
		    ID_valid<=1'b1;//ID�׶ζ�����һ����׼ȷ�ģ�ֻҪ���������Ļ�
		end
		if(1 && ID_allowin)
		begin
		    //����ID�׶ε����ݣ�
			//��λ��ID_validΪ1��ID�ǿյģ�����ID_allowin��1�����ifΪ��
			//���ںܶ�_ID�����������õ�����ϸ�����ֵ�ģ����Լ�ʹ�����if�����㣬_ID������Ҳ�����
			//�����ͻᵼ��ԭ����_ID��ʧ
			
			//inst_ID
			//PC
			//���Ƶ�Ԫ��������⵱Ȼ������inst_ID
			//�˳����Ŀ����źţ�Ҳ������inst_ID
			//���ϣ�����PC��inst_ID���ֲ���
			
			//��������ס�ˣ�����ȡֵ��PC����Ҳ��Ҫ���£�������ȡֵ�õ�PC_next,������߼������¡�
			//���ǵ�PC_next��������EX�׶�֪��������ָ���PC��������ָ���PC�ǹ̶��ģ����ò�
			
			//���˵Ļ���IF�׶������Ѿ�������һ��PC�ˣ�IF�׶ξͿ��Եõ�inst_ID����ֻ��Ҫ�����PC����ס����ô��ʼ����ʱ��ʱ��IF��inst_ID�������PC��Ӧ��
			//���ĵ�ʱ�����PC��ָ�����inst_sram����������ָ����Ҹ���inst_ID�����ָ�Ȼ����ʱû��ϵ�ģ�ֻ��ҪPC����ס�˼���
			
			//ͬʱ����һ��PC����Ҫ����IF�׶Σ���������ĵ�ַ,��Ȼ�������ַ��һ������ȷ��PC����ȷ��PC��ҪEX�׶�ִ���꣩
			//�����������IF��PC��ֻ��Ҫ����������Щ�Ĵ�������ʧ�Ϳ����ˣ��������ڱ�ġ����ǵ�PC��������EX�׶εļĴ���������������ID�׶ζ�ס
			//��������ǰ���ס��������ָ�����ɣ�����EX�׶εļĴ����ڶ�ס��ʱ�򲻻����ɣ��������PC�ᱣ��ס��ֱ������������EX�׶ζ£����������£�
			old_inst_update<=1'b1;
		end
		else
		begin
		    old_inst_update<=1'b0;
			//ĳ��ʱ����ǰ���ж�ס�źš����ʱ���غ�������ڣ�old_instΪ0��old_inst��������ڽ�����ʱ��׼����
			//ĳ��ʱ����ǰ��û�ж�ס�����ʱ���غ������T�ڣ�old_instΪ1����������ڽ�����old_inst���Ը��³�Tʱ����inst_sram�����
		end
	end
	
	//EX
    wire EX_allowin;
	wire EX_ready2go;
	wire EX2MEM_valid;
	assign EX_ready2go=~(doingdiv_ID_EX && ~div_complete);//�����˳�������������
	    //����Ǳ�������EX�׶ε�ָ����û�����꣬���Ǿ���EX�Ƿ���Խ�����
		//�����ֻ��Ҫ��EX�׶ν���֮ǰ��ʱ�仯����
		//������д���ܱ�֤�����е�ָ��ἰʱ���ߣ���͹��ˡ�
		//�����õ���doingdiv_ID�ܷ�ȡ������ֻҪ��һ���ܽ������ɣ���һ��������д�����Խ�����
    assign EX_allowin=~EX_valid || EX_ready2go && MEM_allowin || just_rst;//��ʼ����ʱ��Ҫ��һ��
	assign EX2MEM_valid=EX_valid && EX_ready2go;
	always@(posedge clk)
	begin
	    if(resetn==0)
		begin
		    EX_valid<=1'b1;//��λһ��ʱ���ʵ�����Ѿ������˵�һ��ָ�����EXʵ�����е�,TODO
			              //������֤��һ���ļĴ������Դ�������ȥ
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
	//����ҪMEM2WB_valid�ˣ���Ϊ�����Ŀɲ��ɷ��ͣ���Զ������ˮ�������
	assign MEM_ready2go=1;
	assign MEM_allowin=~MEM_valid || MEM_ready2go && 1/*out_allow=1*/;
	always@(posedge clk)
	begin
	    if(resetn==0)
		begin
		    MEM_valid<=1'b1;//��λһ��ʱ���ʵ�����Ѿ������˵�һ��ָ�����MEMʵ�����е�,TODO
		end
		else if(MEM_allowin)
		begin
		    MEM_valid<=EX2MEM_valid;//ID�׶ζ�����һ����׼ȷ�ģ�ֻҪ���������Ļ�
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
			//ֻҪ��ߵĲ������󸲸ǾͿ�����
			//�ұߣ��е���EX�׶�����EX�׶εļĴ���������ֵ���е���EX�׶δ�ID�׶��չ�����ֵ
			//������û�����𡣱��ʶ���EX�׶εļĴ������������ѣ�ֻ��������ֱ�Ӹ�ֵ����һЩ�߼�����
		end
	end
endmodule

