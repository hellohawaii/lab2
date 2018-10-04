`timescale 10ns / 1ns

module three_to_two(
    input D,
	input E,
	input F,
	output C,
	output S
);
    assign C=E&F | D&E | D&F;
	assign S=D&~E&~F | ~D&E&~F |~D&~E&F | D&E&F;
endmodule