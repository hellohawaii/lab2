`timescale 10ns / 1ns

module Wallace
(//17个加数，每个加数TOTAL_BITS=64位的华莱士树
    input wire clk,
	input wire resetn,
    input wire [16:0] in [TOTAL_BITS-1:0],
	input wire [13:0] cin,
	output wire [TOTAL_BITS-1:0] A,//C,没有进行错位处理，应该左移一位才是要用的
	output wire [TOTAL_BITS-1:0] B//S
);
parameter TOTAL_BITS=64;
genvar bit_num;
generate
    for(bit_num=0;bit_num<TOTAL_BITS;bit_num = bit_num+1)
    begin:tree_num
	    wire [16:0] single_in;
		wire [13:0] single_cin;
		wire [13:0] single_cout;
		wire single_A,single_B;
	    one_bit_Wallace u_one_bit_Wallace(.clk(clk),.resetn(resetn),.in(single_in),.cin(single_cin),.cout(single_cout),.A(single_A),.B(single_B));
		assign single_in=in[bit_num];
	    assign A[bit_num]=single_A;
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
endmodule