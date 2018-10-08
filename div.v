`timescale 10ns / 1ns

module div(
    input div_clk,
	input resetn,
	input div,//表示正在进行除法运算
	input div_signed,
	input [31:0] x,
	input [31:0] y,
	output [31:0] s,
	output [31:0] r,
	output reg complete//要与s、r保持一致 
	               //可能可以依据complete来置div，在complete为1后，div取反
				   //div必须在complete的下一个上沿前取0，下一个上沿之后才能取1同时输入x和y。
				   //complete持续的时候，count是34。然后下一个周期count会变成0
);
//count受div控制，complete受count控制

//计数
reg [5:0] count;
always @(posedge div_clk)
begin
    if(div==0 || resetn==0)//div置为1后，紧接着的时钟上沿，count就会变成1
	                       //如果接连两个除法，不知道会发生什么，TODO
	begin
	    count<=0;
	end
	else
	begin
	    count<=count+1;
	end
	//某个时刻，div变成0，count下一个沿变成0，complete下下个沿就一定是0
	//但是count下一个沿时候的值，取决于沿（div变成0的下一个沿）前count的值是不是32
end

//TODO:这些也要放在时序里面，因为实际上x和y不会在运算过程中保持不变。
//不过好像可以存在A和B中
wire [31:0] abs_x;//在div为1的同时（之后一小点延迟）有效，也即count为1之前就有效了
wire [31:0] abs_y;
wire neg_x,neg_y;//为1表示x和y是负数
assign neg_x=x[31];
assign neg_y=y[31];
assign abs_x=(div_signed==1 && neg_x==1)?(~x+1):x;//TODO:感觉后面算余数其实可以用这里的加法器
assign abs_y=(div_signed==1 && neg_y==1)?(~y+1):y;
reg adjust_s;//s需要调整符号
reg adjust_r;

reg [63:0] A;//TODO:初始化可能需要count来帮助进行
reg [31:0] B;
reg [31:0] S;//商
wire [31:0] R;//余数
reg [63:0] movedA;//移位后的A,会随着计算而更新
wire [32:0] zero_B;//B前面补0
wire [32:0] temp_result;
assign zero_B={1'b0,B};
assign temp_result=movedA[63:31]+~zero_B+1;
assign R=movedA[63:32];

always @(posedge div_clk)
begin
//取A的不同位置应该不好写，可以通过把A移位来进行
    if(count==0)//会把div信号同时的那个x和y读进来，无误
	begin
	    //A<={32'b0,abs_x};//这个值A可能不是很需要
		B<=abs_y;//count为1的同时，变成需要的值
		movedA<={32'b0,abs_x};//count为1的同时，变成需要的值
		adjust_s<=div_signed & (neg_x&~neg_y | ~neg_x&neg_y);
		adjust_r<=div_signed & neg_x;
	end
	else if(count<=32)//余数是组合逻辑，所以要保持住movedA
	begin
	    S<={S[30:0],~temp_result[32]};//在count为1的时间内，第一个temp_result产生；count为1的下一个上升沿时，写入S第0位中
		                              //count为32的下一个上升沿时，写入S的第31位
									  //count为32的下一个上升沿后进行更新，也即count为33的阶段中，S与R的计算都已经完成，只需进行正负数的转换了。要注意R是wire类型
									  //之后如果count不变成0，会保持不变
		movedA<=(({64{temp_result[32]==0}})&({temp_result[31:0],movedA[30:0],1'b0}))
		    |(({64{temp_result[32]==1}})&({movedA[62:0],1'b0}));
			//第一种情况下，temp_result高位0被扔掉
			//第二种的情况下，movedA高位必然是0，因为B第一位是0，所以可以扔掉
	end
end

assign s=({32{adjust_s}} & (~S+1)) | ({32{~adjust_s}} & (S));//依据前面S和R的取值，count为33以及之后的阶段中，它的值是有效的
assign r=({32{adjust_r}} & (~R+1)) | ({32{~adjust_r}} & (R));
//关于如何调整：用绝对值，相当于做的是另一道题，只需要探求两个题之间的答案。
//老师的，是商和余数的符号规定，并不等同于修正方法。
//|x|=|y|*s+r
//x>0,y<0: 自然有x=y*(-s)+r
//x<0,y>0: 自然有x=y*(-s)-r
//x<0,y<0: 自然有x=y*s-r
//上面三个式子指明了“另一道题”的结果与我们要求的结果有什么关系。只需要检查是否合乎正负号的规定即可。
//显然是符合的

always @(posedge div_clk)
begin
    if(count==32)//TODO：如果是32，不太科学，complete与s、r同时更新，会出现complete为1但是s和r不正确的瞬间
	begin
	    complete<=1;
	end
	else
	begin
	    complete<=0;
	end
end
endmodule