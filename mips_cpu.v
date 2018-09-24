`timescale 10ns / 1ns

module mycpu_top(
	input  resetn,
	input  clk,

	output inst_sram_en,
	output [3:0] inst_sram_wen,
	output [31:0] inst_sram_addr,
	output [31:0] inst_sram_wdata,
	input [31:0] inst_sram_rdata,
	
	output data_sram_en,�题
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

    reg [31:0] PC;//��IF����֮ǰ��Ϊ��ȷ��ֵ������ID�׶�����ȷ��instr��
	              //��IF�������������������һ��������
	
	//the following do not need to change when using pipline
	assign inst_sram_wen = 0;//�̶�ֵ������ˮ����������޹�
	assign inst_sram_addr = PC_next;//��PC�йأ���IF����֮ǰ�����ȷ��ֵ��������һ��������
	assign inst_sram_wdata = 0;//�̶�ֵ������ˮ����������޹�
	assign debug_wb_pc = pc_next��//TODO����֪����ô��ֵ
	assign debug_wb_rf_wen = {4{wen_reg_file_EX_MEM}};//TODO
	assign debug_wb_rf_wnum = waddr_ID_MEM;//TODO
	assign debug_wb_rf_wdata = wdata;//TODO
		
	wire [31:0] inst_ID;//��ID�׶εõ���ȷ��instruction�����������ܶ��źš�
	                 //��ID�׶ε������ID�׶�ҲҪʹ�ã������ܺܶ������ź�Ҫ�õ�����TODO,��ҪʱPC�����ʹ�ò��Ǻ�����
					 //��raddr������
					 //д�ؽ׶���Ҫ����EX�׶�Ҳ��Ҫ�����Ǵ�����ȥ�ɡ�
	assign inst_ID=inst_sram_rdata;
	wire [31:0] data_from_mem;//��MEM�׶ε������EX�׶Σ������˵�ַ����ALU�Ľ�����Ӷ���MEM�׶�һ��ʼ�Ϳ��Եõ���ȷ�Ľ����
	                          //����WB�׶�Ҫ�õ�������Ų���Ҫ���ݣ�ֻҪ��WB֮ǰ׼���þͿ���
    assign data_from_mem=data_sram_rdata;

    // define the signal related to main control
	//TODO������Ժ��ٿ�
	//��һЩ�����ź���Ҫ��EX�׶εĽ��
	//�����Ŀ����ź�Ӧ�����൱һ������Ҫ��󴫵�
    wire [5:0] behavior;
	wire [31:0] Result_EX;									//done (used in 2 places)
	                    //EX�׶ν�������RESULT��������������д��ʱ����ֽ�ʹ��
						//TODO��PC���ﲻ���
    wire [1:0] reg_dst_ID;//signal for mux(where to write)	//done
	                   //����׶β�����MEM�׶�Ҫʹ�ã�����д�Ĵ���
    wire mem_read_ID;//enable signal						//done
    wire [3:0] reg_write_value_ID;//signal for mux(what to write)		//done	
    wire [2:0] ALUop;//signal for ALU control			//done
	                 //ID�׶β�����EX�׶�Ҫ�ã��ոպ�
	wire mem_write_ID;//enable signal						//done
	wire [1:0] B_src;//signal for mux(what to compute)		//done
	                 //ID�׶β�����EX�׶�Ҫ�ã��ոպ�
    wire reg_write_ID;//enable signal						//done
	wire PC_enable;//TODO����֪����ôŪ
	//TODO������������Щ�ƺ��Ǹ�PC�й�ϵ��
	wire bne_ID;                                           //done
	wire beq_ID;											//done
	wire j_ID;												//done
	wire jal_ID;											//done
	wire R_type_ID;
	wire regimm_ID;
	wire blez_ID;
	wire bgtz_ID;
	wire [2:0] mem_write_value_ID;
	wire writing_back;//TODO,��֪�����費��Ҫ

	//add the control unit into the circuit
	control_unit cpu_control_unit(.clk(clk),.resetn(resetn),
	    .behavior(behavior),.Result(Result_EX),
		.reg_dst(reg_dst_ID),.mem_read(mem_read_ID),.reg_write_value(reg_write_value_ID),
		.ALUop(ALUop),.mem_write(mem_write_ID),.B_src(B_src),.reg_write(reg_write_ID),
		.data_sram_wen(data_sram_wen),.mem_write_value(mem_write_value_ID),
		//decoding signal
		.bne(bne_ID),.beq(beq_ID),.j(j_ID),.jal(jal_ID),.R_type(R_type_ID),
		.regimm(regimm_ID),.blez(blez_ID),.bgtz(bgtz_ID)
	);
    assign behavior=inst_ID[31:26]; //��inst��ID�׶β������������źţ���ԼҲ������ID��TODO
	assign data_sram_en=mem_read_ID_EX | mem_write_ID_EX;��//TODO����Լ����ID�׶β����������������Ҫ�õ���������Ƚϸ���
	
	//define the signal related to reg_file
	//TODO��һ����ٿ�
	wire clk_reg_file;
	wire rst_reg_file;									//������룬����Ҫ����
	wire [4:0] waddr_ID;//ID������ʱ���������ֵ���Ӷ�WB�׶����������Լ�дʹ�ܺ�д���ݣ���д
	wire [4:0] raddr1;//ID�׶β�����EX�׶�Ҫ�ã��ոպ�
	wire [4:0] raddr2;//ID�׶β�����EX�׶�Ҫ�ã��ոպ�
	wire wen_reg_file_EX;//EX������ʱ���������ֵ���Ӷ�WB�׶����������Լ�д��ַ��д���ݣ���д
	wire [31:0] wdata;//TODO,�����øģ���������MEM�׶β�������������ֵ��WB�׶���������
	wire [31:0] rdata1_EX;									//done
	wire [31:0] rdata2_EX;									//done (used in 2 places)
	//add the reg_flie into the circuit
	reg_file cpu_reg_file(.clk(clk_reg_file),.resetn(rst_reg_file),.waddr_ID_MEM(waddr),.raddr1(raddr1),
		.raddr2(raddr2),.wen(wen_reg_file_EX_MEM),.wdata(wdata),.rdata1(rdata1_EX),.rdata2(rdata2_EX));
	assign clk_reg_file=clk;//��ˮ��Ӧ�ò���Ҫ������
	assign raddr1=inst_ID[25:21];//ID�׶ο�ʼ��ʱ��Ϳ��Ծ�inst����������ID�׶�����������߼�������ֱ�ӻ����ȷ�Ķ�����
	                          //TODO����ʹ���ź����
	assign raddr2=inst_ID[20:16];//ID�׶ο�ʼ��ʱ��Ϳ��Ծ�inst����������ID�׶�����������߼�������ֱ�ӻ����ȷ�Ķ�����
	                          //TODO����ʹ���ź����
	assign wen_reg_file_EX=(R_type==1 && inst_ID[5:0]==6'b001011)?(rdata2_EX!=32'b0):
	                    (R_type==1 && inst_ID[5:0]==6'b001010)?(rdata2_EX==32'b0):
	                    reg_write_ID_EX;//movn,movz
						//������ͨ���źţ�������ɾͿ�����Ч����������movn��movz��������ҪEX���ܵõ���ȷ��ֵ
						//ʹ�õ�ʱ����WB�׶���ʹ��������WB�׶�֮ǰ��Ϊ��ȷֵ��
						//�������Ҫ��rdata2���д���
						//reg_write�ǿ��Ƶ�Ԫ�����ģ����������������ȫ��дʹ�ܿ����źţ����ܻ��������Ŀ���
	assign rst_reg_file=resetn;

	//define the signal related to ALU
	//TODO��һ����ٿ�
	wire [31:0] A;										//done
	wire [31:0] B;										//done
	wire [3:0] ALUoperation;							//done
	wire Overflow;
	wire CarryOut_EX;
	wire Zero;											//done (used in pc_decider)
	//add the ALU into the circuit
	alu cpu_alu(.A(A),.B(B),.ALUop(ALUoperation),.Overflow(Overflow),.CarryOut(CarryOut_EX),
		.Zero(Zero),.Result(Result_EX));
	assign data_sram_addr=Result_EX;

	//add the ALU control unit into the circuit
	ALU_control cpu_ALU_control(.func(inst_ID[5:0]),.ALUop(ALUop),//Ӧ����ID�׶εõ�����ź�
		.ALU_ctr(ALUoperation));
	
	//MUX, what to write to memory
	//��Щ�źŵĲ�������ҪEX�׶ζ�RF�Լ�ALU�Ľ���õ�������Ӧ����EX�׶β�����
	//��������MEM�׶�д�ڴ棬��Ҫ�п����źź�ѡ���ź���ϣ�Ҳ��Ҫ��addr��ϣ�TODO��
	wire [31:0] write_data_sb;
	wire [31:0] write_data_sh;
	wire [31:0] write_data_swl;
	wire [31:0] write_data_swr;
	assign write_data_sb={4{rdata2_EX_MEM[7:0]}};  //unify 4 situations of result
	assign write_data_sh={2{rdata2_EX_MEM[15:0]}};  //unify 4 situations of result
	assign write_data_swl=(Result_EX_MEM[1:0]==2'b00)?{24'b0,rdata2_EX_MEM[31:14]}:
						  (Result_EX_MEM[1:0]==2'b01)?{16'b0,rdata2_EX_MEM[31:16]}:
						  (Result_EX_MEM[1:0]==2'b10)?{8'b0,rdata2_EX_MEM[31:8]}:
						  (Result_EX_MEM[1:0]==2'b11)?rdata2_EX_MEM:
						  32'b0;
	assign write_data_swr=(Result_EX_MEM[1:0]==2'b00)?rdata2_EX_MEM:
						  (Result_EX_MEM[1:0]==2'b01)?{rdata2_EX_MEM[23:0],8'b0}:
						  (Result_EX_MEM[1:0]==2'b10)?{rdata2_EX_MEM[15:0],16'b0}:
						  (Result_EX_MEM[1:0]==2'b11)?{rdata2_EX_MEM[7:0],24'b0}:
						  32'b0;
	assign data_sram_wdata=(mem_write_value_ID_EX==3'b000)?rdata2_EX:
		              (mem_write_value_ID_EX==3'b001)?write_data_sb:
				      (mem_write_value_ID_EX==3'b010)?write_data_sh:
					  (mem_write_value_ID_EX==3'b011)?write_data_swl:
					  (mem_write_value_ID_EX==3'b100)?write_data_swr:
					  rdata2_EX;

	//MUX, where to write, decide 'waddr'
	//������׶ξͿ��Բ�������ѡ��ַ
	//��֪��ѡ���ź���ô�����ġ�����Ҳ�����루TODO��
	//��MEM�׶�ʹ����Щ�ź�
	wire [4:0] waddr_option00;
	wire [4:0] waddr_option01;
	assign waddr_option00=inst_ID_MEM[20:16];//TODO,not sure,�ǲ���mem�׶�ʹ�õ���Щ��ֻ��Ҫ��EX�����׶�׼���þͿ�����
	assign waddr_option01=inst_ID_MEM[15:11];//TODO,not sure,�ǲ���mem�׶�ʹ�õ���Щ��ֻ��Ҫ��EX�����׶�׼���þͿ�����
	assign waddr_ID=(reg_dst_ID_MEM==2'b00)? waddr_option00: 
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
	assign B_option00=rdata2_EX_MEM;
	assign B_option01={{16{inst_ID_EX[15]}},inst_ID_EX[15:0]};
	assign B_option10=32'b0;
	assign B_option11={16'b0,inst_ID_EX[15:0]};
	assign B=(B_src==2'b00)? B_option00:
		     (B_src==2'b01)? B_option01:
			 (B_src==2'b10)? B_option10:
			 (B_src==2'b11)? B_option11:
			 rdata2_EX;

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
			     inst_ID_MEM[5:0]!=6'b101011)&&R_type_ID_MEM==1 || R_type_ID_MEM==0) )?wdata_option0000:
				                                //unify movn and movz

												//some R_type need handle
												//seperately
		         (reg_write_value_ID_MEM===4'b0001)?wdata_option0001:
				 (reg_write_value_ID_MEM===4'b0010 ||
			     reg_write_value_ID_MEM===4'b0000 && inst_ID_MEM[5:0]==6'b001001 && R_type_ID_MEM==1
			     )?(pc_next_option00_EX_MEM+4):
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
				 4'b0000;
				 //ѡ���ź�������׶β�������ѡ�ź��е���EX���е���MEM����

    //MUX,when the option is shift using sa, let a equal to sa instead of rs
	//controled by control unit and function field
	wire [31:0] A_option0;
	wire [31:0] A_option1;
	assign A_option0=rdata1;
	assign A_option1[4:0]=inst_ID_EX[10:6];  //only assign value to part of A_option1
    assign A=(R_type==1 && 
			 (inst_ID_EX[5:0]==6'b000000 || inst_ID_EX[5:0]==6'b000011 ||
			 inst_ID_EX[5:0]==6'b000010)
		     )?A_option1:A_option0;
	//0ѡ��ID�׶β�����TODO����1ѡ��ID�׶β���
	//ѡ���ź�ID�׶β���
	//EX�׶�Ҫʹ����

	//PC
	//TODO��һ����ٿ�
	always @(posedge clk) begin
		if(resetn==0) begin
			PC<=32'Hbfc00000;
		end
		else if(PC_enable)begin//TODO����ˮ��ʱ�����
			PC<=pc_next;
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
	assign pc_next_option01=pc_next_option00_EX+{{{14{inst_ID_EX[15]}},inst_ID_EX[15:0]},2'b00};  //beq,bne(pc+offset)
	assign pc_next_option10={pc_next_option00_EX[31:28],{inst_ID_EX[25:0],2'b00}};
    assign pc_next_option11=rdata1;//����պ���EX�׶εģ�ǡ�ÿ���������
    assign pc_decider=(Zero==0 && bne_ID_EX==1)?2'b01://Zero����պ���EX�׶εģ�ǡ�ÿ���������
		              (Zero==1 && beq_ID_EX==1 ||
					  regimm_ID_EX==1 && inst_ID_EX[20:16]==5'b00001 && Result[0]==0 ||//bgez
				      blez_ID_EX==1 && (Result[0]==1 || rdata1==32'b0) ||//blez
					  bgtz_ID_EX==1 && (Result[0]==0 && rdata1!=32'b0) ||//bgtz
					  regimm_ID_EX==1 && inst_ID_EX[20:16]==5'b00000 && Result[0]==1)?2'b01://bltz
					  (j_ID_EX==1 || jal_ID_EX==1)?2'b10:
					  (R_type_ID_EX==1 && inst_ID_EX[5:1]==5'b00100)?2'b11://unify jalr and jr
					  2'b00;
	assign pc_next=(pc_decider==2'b00)?pc_next_option00_EX:
		           (pc_decider==2'b01)?pc_next_option01:
				   (pc_decider==2'b10)?pc_next_option10:
				   (pc_decider==2'b11)?pc_next_option11:
				   0;
	
	
	//�Ĵ�����ˮ
	reg inst_ID_EX, inst_ID_MEM;
	reg Result_EX_MEM;
	reg reg_dst_ID_EX /*�м������������*/,reg_dst_ID_MEM;
	reg mem_read_ID_EX;
	reg mem_write_ID_EX;
	reg reg_write_value_ID_EX /*�м������������*/,reg_write_value_ID_MEM;
	reg reg_write_ID_EX;
	reg mem_write_value_ID_EX;
	reg waddr_ID_EX  /*�м������������*/,waddr_ID_MEM;
	reg wen_reg_file_EX_MEM;
	reg rdata1_EX_MEM;
	reg rdata2_EX_MEM;
	reg bne_ID_EX,beq_ID_EX,j_ID_EX,jal_ID_EX,R_type_ID_EX,R_type_ID_MEM,regimm_ID_EX,blez_ID_EX,bgtz_ID_EX;
	reg CarryOut_EX_MEM;
	reg pc_next_option00_EX_MEM��
	assign inst_sram_en=(1)/*IF->ID*/?1:0;
	always@(posedge clk)
	begin
	    if(1)//ID->EX
		begin
		    inst_ID_EX<=inst_ID;
			reg_dst_ID_EX <= reg_dst_ID;
			mem_read_ID_EX<=mem_read_ID;
			mem_write_ID_EX<=mem_write_ID;
			reg_write_value_ID_EX <= reg_write_value_ID;
			reg_write_ID_EX<=reg_write_ID;
			mem_write_value_ID_EX<=mem_write_value_ID;
			waddr_ID_EX<=waddr_ID;
			bne_ID_EX<=bne_ID;
			beq_ID_EX<=beq_ID;
			j_ID_EX<=j_ID;
			jal_ID_EX<=jal_ID;
			R_type_ID_EX<=R_type_ID;
			regimm_ID_EX<=regimm_ID;
			blez_ID_EX<=blez_ID;
			bgtz_ID_EX<=bgtz_ID;
		end
		if(1)//EX->MEM
		begin
		    inst_ID_MEM<=inst_ID_EX;
			Result_EX_MEM<=Result_EX;
			reg_dst_ID_MEM <= reg_dst_ID_EX;
			reg_write_value_ID_MEM<=reg_write_value_ID_EX;
			waddr_ID_MEM<=waddr_ID_EX;
			wen_reg_file_EX_MEM<=wen_reg_file_EX;
			rdata1_EX_MEM<=rdata1_EX;
			rdata2_EX_MEM<=rdata2_EX;
			R_type_ID_MEM<=R_type_ID_EX��
			CarryOut_EX_MEM<=CarryOut_EX;
		pc_next_option00_EX_MEM<=pc_next_option00_EX;
		end
		if(1)
		begin
		end
	end
endmodule

