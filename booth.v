`timescale 10ns / 1ns

module booth
//输入扩展后的X(TOTAL_BITS=64)，与Y（IN_BITS=34），得到P和C
(
    input wire [TOTAL_BITS-1:0] X,
	input wire [IN_BITS-1:0] Y,
	output wire [TOTAL_BITS-1:0] P [NUM_RESULTS-1:0],
	output wire [NUM_RESULTS-1:0] neg_flag//取反的标志位
);
parameter TOTAL_BITS=64;//部分积的长度
parameter IN_BITS=34;//输入的乘数的位宽
parameter NUM_RESULTS=17;//部分积的个数

wire [TOTAL_BITS-1:0] two_X;
wire [TOTAL_BITS-1:0] neg_X;
wire [TOTAL_BITS-1:0] two_neg_X;
assign two_X={X[TOTAL_BITS-2:0],1'b0};
assign neg_X=~X;
assign two_neg_X=~two_X;

genvar result_num;
generate
    for(result_num=0;result_num<NUM_RESULTS;result_num=result_num+1)
	begin:booth_num
	    wire y2,y1,y0;
		wire [TOTAL_BITS-1:0] single_p;
		wire [TOTAL_BITS-1:0] temp_p;
		wire single_neg_flag;
		if(result_num==0)
		begin
		    assign y0=0;
			assign {y2,y1}=Y[1:0];
		end
		else
		begin
		    assign {y2,y1,y0}=Y[2*result_num+1:2*result_num-1];
		end
		wire [2:0] sel;
		assign sel={y2,y1,y0};
		assign temp_p=({TOTAL_BITS{sel==3'b000}} & {TOTAL_BITS{1'b0}})| 
		        |({TOTAL_BITS{sel==3'b001}} & X)
				|({TOTAL_BITS{sel==3'b010}} & X)
				|({TOTAL_BITS{sel==3'b011}} & two_X)
				|({TOTAL_BITS{sel==3'b100}} & two_neg_X)
				|({TOTAL_BITS{sel==3'b000}} & neg_X)
				|({TOTAL_BITS{sel==3'b000}} & neg_X)
				|({TOTAL_BITS{sel==3'b000}} & {TOTAL_BITS{1'b0}});
		assign single_p={temp_p[TOTAL_BITS-1-2*result_num:0],{result_num{2'b00}}};//允许0个拼接在一起吗
		assign P[result_num]=single_p;
		assign single_neg_flag=(sel==3'b100)|(sel==3'b101)|(sel==3'b110);
		assign neg_flag[result_num]=single_neg_flag;
	end
endgenerate
endmodule