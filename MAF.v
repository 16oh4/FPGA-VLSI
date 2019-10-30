`timescale 1ns/1ps
module DATAP #(parameter bits = 32)(
    input clk,
    input [1:0] func,
    input [bits-1:0] A,B,C,
    
    output [bits*2-1:0] result, //needs to be as big as mult result
    output reg [1:0] func_reg,
    output reg [bits*2-1:0] mult_out_reg,
    output reg [bits-1:0] c_reg
);

//Pipeline stage registers
/*
reg [1:0] func_reg;
reg [bits*2-1:0] mult_out_reg;
reg [bits-1:0] c_reg;
*/

//Internal wires
wire [bits-1:0] mux_ps1, mux_ps2;
wire [bits*2-1:0] mult_out;
//wire [bits*2-1:0] result_w;

always@(posedge clk) begin
    func_reg <= func;
    mult_out_reg <= mult_out;
    c_reg <= C;
    //result <= result_w;
end

//BEGIN pipeline stage 1
MUX #(bits) MUX_PS1(
.A(1'b1),
.B(B),
.sel(func[0]),

.out(mux_ps1)
);

ARRAY_MULTIPLIER #(bits) MULT(
.a(A),
.b(mux_ps1),

.p(mult_out)
);
//END pipeline stage 1


//BEGIN pipeline stage 2
wire mux_ps2_sel;
wire [bits*2-1:0] mux_ps2_ext;

//have to sign extend the output of mux for adder
//since result of multiplier is bits*2
assign mux_ps2_ext = { {bits{mux_ps2[bits-1]}}, mux_ps2 };

MUX #(bits) MUX_PS2(
.A(1'b0),
.B(c_reg),
.sel(mux_ps2_sel),

.out(mux_ps2)
);


CLA #(bits*2) ADDER(
.A(mult_out_reg),
.B(mux_ps2_ext),
.Cin(1'b0),

.Sum(result),
.Cout()
);

//Input to mux select to coincide with func
nor(mux_ps2_sel, func_reg[1], func_reg[0]);

//END pipeline stage 2

endmodule

module MUX #(parameter bits = 32)(
    input [bits-1:0] A, B,
    input sel,
    output [bits-1:0] out
);

assign out = sel ? A : B;

endmodule

module ARRAY_MULTIPLIER #(parameter bits = 32) (
    input [bits-1:0] a, b,
    output [bits*2-1:0] p
);

//rows means the partial sums
//for 4-bit adder:
//carries[1][0] is the carry out from the rightmost halfadder in the 1st partial sum

//DECLARING: [index] carries [rows]
//ACCESSING: carries[rows][index]

wire [bits-1:0]     carries [bits-1:1];
wire [bits*2-1:0]   sums [bits-1:1];

assign p[0] = a[0] & b[0]; //always true

genvar i,j;
generate
    for(i=1; i < bits; i=i+1) begin : GEN_PARTIALS
        for(j=0; j < bits; j=j+1) begin: GEN_ADDERS   
             
            //FIRST LEVEL OF PARTIAL SUMS
            if(i==1) begin 
            
                //FIRST ADDER
                if(j==0) begin
                    ADD_HALF INSTF_HA_FIRST (
                        .a(a[i]&b[j]),
                        .b(a[i-1]&b[i]),
                                        
                        .sum(p[i]),
                        .cout(carries[i][j])
                    );            
                end 
                
                //LAST ADDER
                else if(j==(bits-1)) begin
                    ADD_HALF INSTF_HA_LAST (
                        .a(a[i]&b[j]),
                        .b(carries[i][j-1]),
                    
                        .sum(sums[i][j]),
                        .cout(carries[i][j])  
                    );
                end  
                
                //IN BETWEEN
                else begin
                    FULL_ADDER INSTF_FA (
                        .a(a[i]&b[j]),
                        .b(a[i-1]&b[j+1]),
                        .cin(carries[i][j-1]),
                                
                        .sum(sums[i][j]),
                        .cout(carries[i][j])       
                    );
                end
            end //FIRST LEVEL
            
            //LAST LEVEL OF PARTIAL SUMS
            else if(i==(bits-1)) begin
            
                //FIRST ADDER
                if(j==0) begin 
                    ADD_HALF INSTL_HA_FIRST (
                        .a(sums[i-1][j+1]),
                        .b(a[i]&b[j]),
                    
                        .sum(p[i+j]),
                        .cout(carries[i][j]) 
                    );                
                end
                
                //LAST ADDER
                else if(j==(bits-1)) begin
                    FULL_ADDER INSTL_FA_LAST (
                        .a(carries[i-1][j]),
                        .b(a[i]&b[j]),
                        .cin(carries[i][j-1]),
                    
                        .sum(p[i+j]),
                        .cout(p[i+j+1])
                    );
                end
                
                //IN BETWEEN
                else begin
                    FULL_ADDER INSTL_FA (
                        .a(sums[i-1][j+1]),
                        .b(a[i]&b[j]),
                        .cin(carries[i][j-1]),                  
                    
                        .sum(p[i+j]),
                        .cout(carries[i][j])
                    );                
                end
            end //LAST LEVEL
            
            //ALL LEVELS IN BETWEEN
            else begin
                
                //FIRST ADDER
                if(j==0) begin
                    ADD_HALF INSTB_HA_FIRST (
                        .a(sums[i-1][j+1]),
                        .b(a[i]&b[j]),
                        
                        .sum(p[i]),
                        .cout(carries[i][j])
                    );
                end
                
                //LAST ADDER
                else if(j==(bits-1)) begin
                    FULL_ADDER INSTB_FA_LAST (
                        .a(carries[i-1][j]),
                        .b(a[i]&b[j]),
                        .cin(carries[i][j-1]),
                        
                        .sum(sums[i][j]),                    
                        .cout(carries[i][j])
                    );
                end
                
                //IN BETWEEN
                else begin
                    FULL_ADDER INSTB_FA (
                        .a(sums[i-1][j+1]),
                        .b(a[i]&b[j]),
                        .cin(carries[i][j-1]),
                        
                        .sum(sums[i][j]),
                        .cout(carries[i][j])                        
                    );          
                end
            end //IN BETWEEN LEVELS
        end //ADDERS FOR LOOP        
    end //LEVELS FOR LOOP
endgenerate
endmodule

module ADD_HALF(
    output cout,
    output sum,
    input a,
    input b
);
    xor(sum, a, b);
    and(cout, a, b);
endmodule

//Calculates the propagate, generate, and sum bits
module PFA(
    input A, B, Cin,
    output P, G, S
);
    
    assign P = A ^ B;   //xor gate for propagate
    assign G = A & B;   // and gate for generate
    assign S = Cin ^ P; // xor gate for sum

endmodule

module FULL_ADDER(
    input a,b,cin,
    output cout,sum

);

wire half1_cout, half1_sum, half2_cout;

ADD_HALF HALF1(
.cout(half1_cout),
.sum(half1_sum),
.a(a),
.b(b)
);

ADD_HALF HALF2(
.cout(half2_cout),
.sum(sum),
.a(cin),
.b(half1_sum)
);

//for the carry out of both half adders
or(cout, half2_cout, half1_cout);

endmodule

//Calculates the carry using generate, propagate, and carry in bits
module CARRY_GEN(
    input G, P, Cin,
    output Cout    
);

assign Cout = G | P&Cin; //wire out carry calculation

endmodule

//Wrapper for Carry-Lookahead-Logic and Partial Full Adders
module CLA #(parameter bits = 64) ( //parameter to specify number of bits for adder
input [bits-1:0] A, B, //augend and addend
input Cin,  //first carry in

output [bits-1:0] Sum, //sum output
output Cout //carry out bit

//output [bits-1:0]carries //for testbench
);

wire [bits-1:0]P_in, G_in; //wire bus for propagate and generate bits
wire [bits-1:0]carries; //wire bus for carry bits

assign Cout = carries[bits-1]; //propagate last carry out from msb of carries bus

//create partial full adder instance for the first bit to incorporate Cin
PFA PFA0(
.A(A[0]),
.B(B[0]),
.Cin(Cin),

.P(P_in[0]),
.G(G_in[0]),
.S(Sum[0])

);

genvar i;
generate //Instantiate N PFA's for bits
    for(i=1; i<bits; i=i+1) begin : PFAS
        PFA PFA_I(
            .A(A[i]),
            .B(B[i]),
            .Cin(carries[i-1]),
            
            .P(P_in[i]),
            .G(G_in[i]),
            .S(Sum[i])        
        );    
    end
endgenerate

//Instantiate Carry-Lookahead-Logic block
CLL #(.bits(bits)) CLL_INST(
.Cin(Cin),
.P(P_in),
.G(G_in),

.Cout(carries)
);

endmodule

//This module handles the carry outs for each partial full adder
//Acts as a wrapper to individual carry_gen modules
module CLL #(parameter bits = 8) (
    input Cin,
    input [bits-1: 0] P, G, 
    output [bits-1: 0] Cout
);

//Create carry generator block for first bit
CARRY_GEN INST0(
        .G(G[0]),
        .P(P[0]),
        .Cin(Cin),
        
        .Cout(Cout[0])        
);

genvar i;
generate //Create N bits of carry generators
    for(i=1; i < bits; i=i+1) begin : CARRIES
        CARRY_GEN INST_I(
        .G(G[i]),
        .P(P[i]),
        .Cin(Cout[i-1]),
        
        .Cout(Cout[i])        
        );
    
    end
endgenerate
endmodule