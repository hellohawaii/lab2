`timescale 10ns / 1ns

module mul
(parameter TOTAL_BITS=64,//部分积的长度
 parameter IN_BITS=34,//输入的乘数的位宽
 parameter EXTEND_BITS1=32
 parameter EXTEND_BITS2=2
 parameter NUM_RESULTS=17//部分积的个数
)
(
    input mul_clk,
	input resetn,
	input mul_signed,
	input [31:0] x,
	input [31:0] y,
	output [63:0] result
);

    wire [TOTAL_BITS-1:0] longx;
	wire [IN_BITS-1:0] longy;
	assign longx={{EXTEND_BITS1{mul_signed&x[31]}},x};
	assign longy={{EXTEND_BITS2{mul_signed&y[31]}},y};

	wire [TOTAL_BITS-1:0] P [NUM_RESULTS-1:0];
	wire [NUM_RESULTS-1:0] neg_flag;	
	booth u_booth(.X(longx),.Y(longy),.P(P),.neg_flag(neg_flag));
	
	wire [16:0] in [TOTAL_BITS-1,0];
	wire [13:0] cin;
	wire [TOTAL_BITS-1:0] tempA;
	wire [TOTAL_BITS-1:0] B;

	genvar gvr1;
	genvar gvr2;
	generate
	    for(gvr1=0;gvr1<=TOTAL_BITS;gvr1=gvr1+1)
		begin:loop1
		    for(gvr2=0;gvr2<NUM_RESULTS;gvr2=gvr2+1)
			begin:loop2
			    assign in[gvr1][gvr2]=P[gvr2][gvr1];
			end
		end
	endgenerate
	
	assign cin[13:0]=neg_flag[13:0];
	Wallace u_Wallace(.mul_clk(clk),.resetn(resetn),.in(in),.cin(cin),.A(tempA),.B(B));
	
	wire [TOTAL_BITS-1:0] A;
	assign A={tempA[TOTAL_BITS-2:0],neg_flag[14]};
	
	assign result=A+B+neg_flag[15];
);
endmodule
