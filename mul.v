`timescale 10ns / 1ns

module mul
(
    input mul_clk,
	input resetn,
	input mul_signed,
	input [31:0] x,
	input [31:0] y,
	output [63:0] result
);
parameter TOTAL_BITS=64;//部分积的长度
parameter IN_BITS=34;//输入的乘数的位宽
parameter EXTEND_BITS1=32;
parameter EXTEND_BITS2=2;
parameter NUM_RESULTS=17;//部分积的个数

wire [TOTAL_BITS-1:0] longx;
wire [IN_BITS-1:0] longy;
assign longx={{EXTEND_BITS1{mul_signed&x[31]}},x};
assign longy={{EXTEND_BITS2{mul_signed&y[31]}},y};

//输入扩展后的longx(TOTAL_BITS=64)，与longy（IN_BITS=34），得到P和C
wire [TOTAL_BITS-1:0] P [NUM_RESULTS-1:0];
wire [NUM_RESULTS-1:0] neg_flag;	
wire [TOTAL_BITS-1:0] two_X;
wire [TOTAL_BITS-1:0] neg_X;
wire [TOTAL_BITS-1:0] two_neg_X;
assign two_X={longx[TOTAL_BITS-2:0],1'b0};
assign neg_X=~longx;
assign two_neg_X=~two_X;
    
genvar result_num;
generate
    for(result_num=0;result_num<NUM_RESULTS;result_num=result_num+1)
    begin:booth_num
        wire y2,y1,y0;
        wire [TOTAL_BITS-1:0] single_p;
        wire [TOTAL_BITS-1:0] temp_p;
        wire single_neg_flag;//取反的标志位
        if(result_num==0)
        begin
            assign y0=1'b0;
            assign {y2,y1}=longy[1:0];
        end
        else
        begin
            assign {y2,y1,y0}=longy[2*result_num+1:2*result_num-1];
        end
        wire [2:0] sel;
        assign sel={y2,y1,y0};
        assign temp_p=({TOTAL_BITS{sel==3'b000}} & {TOTAL_BITS{1'b0}})
                |({TOTAL_BITS{sel==3'b001}} & longx)
                |({TOTAL_BITS{sel==3'b010}} & longx)
                |({TOTAL_BITS{sel==3'b011}} & two_X)
                |({TOTAL_BITS{sel==3'b100}} & two_neg_X)
                |({TOTAL_BITS{sel==3'b101}} & neg_X)
                |({TOTAL_BITS{sel==3'b110}} & neg_X)
                |({TOTAL_BITS{sel==3'b111}} & {TOTAL_BITS{1'b0}});
        if(result_num==0)
        begin
            assign single_p=temp_p;
        end
        else
        begin
            assign single_p={temp_p[TOTAL_BITS-1-2*result_num:0],{2*result_num{single_neg_flag}}};//允许0个拼接在一起吗
        end
        assign P[result_num]=single_p;
        assign single_neg_flag=(sel==3'b100)|(sel==3'b101)|(sel==3'b110);
        assign neg_flag[result_num]=single_neg_flag;
    end
endgenerate

//受流水线影响，neg_flag的5-15位都要保存下来
reg [NUM_RESULTS-1:0] neg_flag_reg;
always @(posedge mul_clk)
begin
    if(resetn==0)
	begin
        neg_flag_reg<={NUM_RESULTS{1'b0}};
    end
	else
	begin
	    neg_flag_reg<=neg_flag;
	end
end
	
wire [16:0] in [TOTAL_BITS-1:0];
wire [13:0] cin;
wire [TOTAL_BITS-1:0] tempA;//C,没有进行错位处理，应该左移一位才是要用的
wire [TOTAL_BITS-1:0] B;//S

genvar gvr1;
genvar gvr2;
generate
    for(gvr1=0;gvr1<TOTAL_BITS;gvr1=gvr1+1)
	begin:loop1
	    for(gvr2=0;gvr2<NUM_RESULTS;gvr2=gvr2+1)
		begin:loop2
		    assign in[gvr1][gvr2]=P[gvr2][gvr1];
		end
	end
endgenerate
	
assign cin[13:0]=/*neg_flag[13:0]*/{neg_flag_reg[13:5],neg_flag[4:0]};
//17个加数，每个加数TOTAL_BITS=64位的华莱士树
genvar bit_num;
generate
    for(bit_num=0;bit_num<TOTAL_BITS;bit_num = bit_num+1)
    begin:tree_num
        wire [16:0] single_in;
        wire [13:0] single_cin;
        wire [13:0] single_cout;
        wire single_A,single_B;
        one_bit_Wallace u_one_bit_Wallace(.clk(mul_clk),.resetn(resetn),.in(single_in),.cin(single_cin),.cout(single_cout),.A(single_A),.B(single_B));
        assign single_in=in[bit_num];
        assign tempA[bit_num]=single_A;
        assign B[bit_num]=single_B;
        if(bit_num==0)
        begin
            assign single_cin=cin;
        end
        else
        begin
            assign single_cin=tree_num[bit_num-1].single_cout;
        end
    end
endgenerate
	
wire [TOTAL_BITS-1:0] A;
assign A={tempA[TOTAL_BITS-2:0],neg_flag_reg[14]};
	
assign result=A+B+neg_flag_reg[15];
endmodule
