`timescale 10ns / 1ns

module one_bit_Wallace(//对17个加数进行处理的华莱士树，这是其中的一位
    input wire clk,
	input wire resetn,
    input wire [16:0] in,
	input wire [13:0] cin,
	output wire [13:0] cout,
	output wire A,
	output wire B
);

//实例化，对three_to_two模块编号，从底层开始向上为第一个编号，从左向右数为第二个编号
wire D11,E11,F11,C11,S11;
wire D12,E12,F12,C12,S12;
wire D13,E13,F13,C13,S13;
wire D14,E14,F14,C14,S14;
wire D15,E15,F15,C15,S15;

wire D21,E21,F21,C21,S21;
wire D22,E22,F22,C22,S22;
wire D23,E23,F23,C23,S23;
wire D24,E24,F24,C24,S24;

wire D31,E31,F31,C31,S31;
wire D32,E32,F32,C32,S32;

wire D41,E41,F41,C41,S41;
wire D42,E42,F42,C42,S42;

wire D51,E51,F51,C51,S51;

wire D61,E61,F61,C61,S61;
reg [3:0] cout_5_8_reg;
reg [3:0] S21_24_reg;
three_to_two three_to_two_11(.D(D11),.E(E11),.F(F11),.C(C11),.S(S11));
three_to_two three_to_two_12(.D(D12),.E(E12),.F(F12),.C(C12),.S(S12));
three_to_two three_to_two_13(.D(D13),.E(E13),.F(F13),.C(C13),.S(S13));
three_to_two three_to_two_14(.D(D14),.E(E14),.F(F14),.C(C14),.S(S14));
three_to_two three_to_two_15(.D(D15),.E(E15),.F(F15),.C(C15),.S(S15));

three_to_two three_to_two_21(.D(D21),.E(E21),.F(F21),.C(C21),.S(S21));
three_to_two three_to_two_22(.D(D22),.E(E22),.F(F22),.C(C22),.S(S22));
three_to_two three_to_two_23(.D(D23),.E(E23),.F(F23),.C(C23),.S(S23));
three_to_two three_to_two_24(.D(D24),.E(E24),.F(F24),.C(C24),.S(S24));

three_to_two three_to_two_31(.D(D31),.E(E31),.F(F31),.C(C31),.S(S31));
three_to_two three_to_two_32(.D(D32),.E(E32),.F(F32),.C(C32),.S(S32));

three_to_two three_to_two_41(.D(D41),.E(E41),.F(F41),.C(C41),.S(S41));
three_to_two three_to_two_42(.D(D42),.E(E42),.F(F42),.C(C42),.S(S42));

three_to_two three_to_two_51(.D(D51),.E(E51),.F(F51),.C(C51),.S(S51));

three_to_two three_to_two_61(.D(D61),.E(E61),.F(F61),.C(C61),.S(S61));

//连线
//用位拼接可能好看一些。
assign {D11,E11,F11,D12,E12,F12,D13,E13,F13,D14,E14,F14,D15,E15,F15}=in[16:2];
/*
assign E11=in[15];
assign F12=in[14];
assign D12=in[13];
assign E12=in[12];
assign F12=in[11];
assign D13=in[10];
assign E13=in[ 9];
assign F13=in[ 8];
assign D14=in[ 7];
assign E14=in[ 6];
assign F14=in[ 5];
assign D15=in[ 4];
assign E15=in[ 3];
assign F15=in[ 2];
*/
assign cout[0:4]={C11,C12,C13,C14,C15};
/*
assign cout[1]=C12;
assign cout[2]=C13;
assign cout[3]=C14;
assign cout[4]=C15;
*/

assign {D21,E21,F21,D22,E22,F22,D23,E23,F23,D24,E24,F24}={S11,S12,S13,S14,S15,in[1:0],cin[0:4]};
/*
assign E21=S12;
assign F21=S13;
assign D22=S14;
assign E22=S15;
assign F22=in[1];
assign D23=in[0];
assign E23=cin[0];
assign F23=cin[1];
assign D24=cin[2];
assign E24=cin[3];
assign F24=cin[4];
*/
assign cout[5:8]=cout_5_8_reg;

always @(posedge clk)
begin
    if(resetn==0)
	begin
        cout_5_8_reg<=4'b0;
	    S21_24_reg<=4'b0;
    end
	else
	begin
	    cout_5_8_reg<={C21,C22,C23,C24};
	    S21_24_reg<={S21,S22,S23,S24};
	end
end

assign {D31,E31,F31,D32,E32,F32}={S21_24_reg,cin[5:6]};
assign cout[9:10]={C31,C32};

assign {D41,E41,F41,D42,E42,F42}={S31,S32,cin[7:10]};
assign cout[11:12]={C41,C42};

assign {D51,E61,F61}={S41,S42,cin[11]};
assign cout[13]=C51;

assign {D61,E61,F61}={S51,cin[12:13]};
assign {A,B}={C61,S61};

endmodule