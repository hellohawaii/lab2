`timescale 10ns / 1ns

module div(
    input div_clk,
	input resetn,
	input div,//��ʾ���ڽ��г�������
	input div_signed,
	input [31:0] x,
	input [31:0] y,
	output [31:0] s,
	output [31:0] r,
	output reg complete//Ҫ��s��r����һ�� 
	               //���ܿ�������complete����div����completeΪ1��divȡ��
				   //div������complete����һ������ǰȡ0����һ������֮�����ȡ1ͬʱ����x��y��
				   //complete������ʱ��count��34��Ȼ����һ������count����0
);
//count��div���ƣ�complete��count����

//����
reg [5:0] count;
always @(posedge div_clk)
begin
    if(div==0 || resetn==0)//div��Ϊ1�󣬽����ŵ�ʱ�����أ�count�ͻ���1
	                       //�������������������֪���ᷢ��ʲô��TODO
	begin
	    count<=0;
	end
	else
	begin
	    count<=count+1;
	end
	//ĳ��ʱ�̣�div���0��count��һ���ر��0��complete���¸��ؾ�һ����0
	//����count��һ����ʱ���ֵ��ȡ�����أ�div���0����һ���أ�ǰcount��ֵ�ǲ���32
end

//TODO:��ЩҲҪ����ʱ�����棬��Ϊʵ����x��y��������������б��ֲ��䡣
//����������Դ���A��B��
wire [31:0] abs_x;//��divΪ1��ͬʱ��֮��һС���ӳ٣���Ч��Ҳ��countΪ1֮ǰ����Ч��
wire [31:0] abs_y;
wire neg_x,neg_y;//Ϊ1��ʾx��y�Ǹ���
assign neg_x=x[31];
assign neg_y=y[31];
assign abs_x=(div_signed==1 && neg_x==1)?(~x+1):x;//TODO:�о�������������ʵ����������ļӷ���
assign abs_y=(div_signed==1 && neg_y==1)?(~y+1):y;
reg adjust_s;//s��Ҫ��������
reg adjust_r;

reg [63:0] A;//TODO:��ʼ��������Ҫcount����������
reg [31:0] B;
reg [31:0] S;//��
wire [31:0] R;//����
reg [63:0] movedA;//��λ���A,�����ż��������
wire [32:0] zero_B;//Bǰ�油0
wire [32:0] temp_result;
assign zero_B={1'b0,B};
assign temp_result=movedA[63:31]+~zero_B+1;
assign R=movedA[63:32];

always @(posedge div_clk)
begin
//ȡA�Ĳ�ͬλ��Ӧ�ò���д������ͨ����A��λ������
    if(count==0)//���div�ź�ͬʱ���Ǹ�x��y������������
	begin
	    //A<={32'b0,abs_x};//���ֵA���ܲ��Ǻ���Ҫ
		B<=abs_y;//countΪ1��ͬʱ�������Ҫ��ֵ
		movedA<={32'b0,abs_x};//countΪ1��ͬʱ�������Ҫ��ֵ
		adjust_s<=div_signed & (neg_x&~neg_y | ~neg_x&neg_y);
		adjust_r<=div_signed & neg_x;
	end
	else if(count<=32)//����������߼�������Ҫ����סmovedA
	begin
	    S<={S[30:0],~temp_result[32]};//��countΪ1��ʱ���ڣ���һ��temp_result������countΪ1����һ��������ʱ��д��S��0λ��
		                              //countΪ32����һ��������ʱ��д��S�ĵ�31λ
									  //countΪ32����һ�������غ���и��£�Ҳ��countΪ33�Ľ׶��У�S��R�ļ��㶼�Ѿ���ɣ�ֻ�������������ת���ˡ�Ҫע��R��wire����
									  //֮�����count�����0���ᱣ�ֲ���
		movedA<=(({64{temp_result[32]==0}})&({temp_result[31:0],movedA[30:0],1'b0}))
		    |(({64{temp_result[32]==1}})&({movedA[62:0],1'b0}));
			//��һ������£�temp_result��λ0���ӵ�
			//�ڶ��ֵ�����£�movedA��λ��Ȼ��0����ΪB��һλ��0�����Կ����ӵ�
	end
end

assign s=({32{adjust_s}} & (~S+1)) | ({32{~adjust_s}} & (S));//����ǰ��S��R��ȡֵ��countΪ33�Լ�֮��Ľ׶��У�����ֵ����Ч��
assign r=({32{adjust_r}} & (~R+1)) | ({32{~adjust_r}} & (R));
//������ε������þ���ֵ���൱����������һ���⣬ֻ��Ҫ̽��������֮��Ĵ𰸡�
//��ʦ�ģ����̺������ķ��Ź涨��������ͬ������������
//|x|=|y|*s+r
//x>0,y<0: ��Ȼ��x=y*(-s)+r
//x<0,y>0: ��Ȼ��x=y*(-s)-r
//x<0,y<0: ��Ȼ��x=y*s-r
//��������ʽ��ָ���ˡ���һ���⡱�Ľ��������Ҫ��Ľ����ʲô��ϵ��ֻ��Ҫ����Ƿ�Ϻ������ŵĹ涨���ɡ�
//��Ȼ�Ƿ��ϵ�

always @(posedge div_clk)
begin
    if(count==32)//TODO�������32����̫��ѧ��complete��s��rͬʱ���£������completeΪ1����s��r����ȷ��˲��
	begin
	    complete<=1;
	end
	else
	begin
	    complete<=0;
	end
end
endmodule