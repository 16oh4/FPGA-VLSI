`timescale 1ns / 1ps
//Engineer: Bruno E. Gracia Villalobos
//16oh4.com
//DECEMBER 4, 2019
//MIT LICENSE

module CNTRL #(parameter BITS = 16, ORDER = 4, ROWS = 16)(
input dpu_clk,

output [BITS*2-1:0] y,

//CONTROLLER DIAGNOSTICS
output [3:0] state_test,
output [3:0] next_state_test,
output ctrl_clk_test,
output [31:0] loop_test,
output [31:0] filter_loop_test,

//DPU DIAGNOSTICS
output [$clog2(ROWS)-1:0] dpu_h_rom_addr_test,
output [BITS*(ORDER+1)-1:0] dpu_h_rom_data_test,
output [BITS*(ORDER+1)-1:0] dpu_h_regs_test,

output [BITS-1:0] dpu_x_rom_data_test,
output [BITS-1:0] dpu_x_regs_test,

output [ORDER:0] dpu_enables_test,
output [BITS*2*(ORDER+1)-1:0] dpu_y_test,
output inv_dpu_clk_test
);

localparam ROM_ADDR_SIZE = $clog2(ROWS);
localparam CLK_DIVIDE = 2;

//SETUP CLOCK FOR STATE MACHINE
wire ctrl_clk;
assign ctrl_clk_test = ctrl_clk;

SLOW_CLK #(CLK_DIVIDE) SLOWCLK(
.clk(dpu_clk),
.slow_clk(ctrl_clk)
);

//SETUP DATAPATH
reg [ROM_ADDR_SIZE-1:0] dpu_h_rom_addr;
assign dpu_h_rom_addr_test = dpu_h_rom_addr;

reg dpu_h_rom_rd;

reg [ROM_ADDR_SIZE-1:0] dpu_x_rom_addr;
reg dpu_x_rom_rd;

reg [ORDER:0] dpu_clrs;
reg [ORDER:0] dpu_enables;
assign dpu_enables_test = dpu_enables;

wire [BITS-1:0] dpu_y;

//INVERTER CLOCK WHENEVER X DATA COMES IN
reg inv_dpu_clk;
initial inv_dpu_clk = 1'b0;
assign inv_dpu_clk_test = inv_dpu_clk;

DATAPATH DPU(
.clk(dpu_clk),

.h_rom_addr         (dpu_h_rom_addr),
.h_rom_rd           (dpu_h_rom_rd),

.x_rom_addr         (dpu_x_rom_addr),
.x_rom_rd           (dpu_x_rom_rd),

.clrs               (dpu_clrs),
.enables            (dpu_enables),

.y(y),

//DIAGNOSTICS
.tap_reg_out_test   (dpu_h_regs_test),
.h_out              (dpu_h_rom_data_test),

.x_reg_test         (dpu_x_regs_test),
.x_out              (dpu_x_rom_data_test),

.y_test(dpu_y_test)
);

defparam DPU.BITS = BITS;
defparam DPU.ORDER = ORDER;
defparam DPU.ROWS = ROWS;

//SETUP STATE MACHINE
reg [3:0] state;
reg [3:0] next_state;
assign state_test = state;
assign next_state_test = next_state;

integer loop;
integer filter_loop;

assign loop_test = loop;
assign filter_loop_test = filter_loop;

localparam RST      = 'h0;

//TAP REGISTERING STATES
localparam SETUP    = 'h1;
localparam LOAD     = 'h2;
localparam IDLE     = 'h3;

//FILTERING STATES
localparam F_SETUP  = 'h4;
localparam F_LOAD   = 'h5;
localparam F_IDLE   = 'h6;
localparam F_IDLE2   = 'h7;

localparam LINGO = 'h8;


//INITIALIZE ALL REGISTERS TO 0
initial begin
dpu_h_rom_addr = {ROM_ADDR_SIZE{1'b0}};
dpu_x_rom_addr = {ROM_ADDR_SIZE{1'b0}};

dpu_h_rom_rd = 1'b0;
dpu_x_rom_rd = 1'b0;

dpu_clrs = {(ORDER+1){1'b0}};
dpu_enables = {(ORDER+1){1'b0}};

state = 'h0;
next_state = 'h0;
end



always@(posedge ctrl_clk) begin
    if (state < F_SETUP) state = next_state;
end

/*
always@(posedge dpu_clk) begin
    if(state >= F_SETUP ) state = next_state;
end
*/

always@(negedge dpu_clk) begin
    if((next_state == F_SETUP)) state = next_state;
    else if(state >= F_SETUP) state = next_state;
end

always@(state) begin
    case(state)
    RST: begin //STATE 0 
           
        //RESET AND DISABLE ALL REGISTERS
        dpu_clrs = {(ORDER+1){1'b1}};
        dpu_enables = {(ORDER+1){1'b0}};
        
        //START ALL ROM ADDRESSES TO 0
        dpu_h_rom_addr = {ROM_ADDR_SIZE{1'b0}};
        dpu_x_rom_addr = {ROM_ADDR_SIZE{1'b0}};
        
        //TURN OFF ROMs
        dpu_x_rom_rd = 1'b0;
        dpu_h_rom_rd = 1'b0;
        
        next_state = SETUP;
    end
    SETUP: begin // STATE 1
    
        //DISABLE CLEARS
        dpu_clrs = {(ORDER+1){1'b0}};
        
        //ENABLE THE TAP REGISTER TO WRITE ROM DATA TO
        //dpu_enables[ORDER] = 1'b1;
        dpu_enables[0] = 1'b1;
        
        //PREPARE TO READ IN TAPS
        dpu_h_rom_rd = 1'b1;
        
        //INITIALIZE LOOP VARIABLE
        loop = ORDER;
        
        next_state = LOAD;    
    end
    LOAD: begin //STATE 2

        //INCREMENT ROM ADDRESS TO READ NEXT TAP
        dpu_h_rom_addr = dpu_h_rom_addr + 1'b1;
        
        //SHIFT REGISTER ENABLE BIT FOR NEXT TAP TO WRITE FROM ROM
        //dpu_enables = {dpu_enables[0], dpu_enables[ORDER:1]};
        dpu_enables = {dpu_enables[ORDER-1:0], dpu_enables[ORDER]};
        
        //DECREMENT LOOP
        loop = loop - 1'b1;      
        
        //CONTINUE TO LOOP IN THIS STATE
        next_state = IDLE;
        
    end
    IDLE: begin //STATE 3
        if(loop > 0) begin
            next_state = LOAD;
        end
        //WHEN ALL TAPS HAVE BEEN LOADED
        else if(loop == 0) begin
        
            //DISABLE ALL REGISTERS TO PREVENT OVERWRITE OF TAPS
            dpu_enables = {(ORDER+1){1'b0}};
            
            next_state = F_SETUP;
            
            
        end
    end
        
    F_SETUP: begin //STATE 4
               
        //START ALL ROM ADDRESSES TO 0
        dpu_h_rom_addr = {ROM_ADDR_SIZE{1'b0}};
        dpu_x_rom_addr = {ROM_ADDR_SIZE{1'b0}};
        
        //START READING FROM SIGNAL ROM
        dpu_x_rom_rd = 1'b1;
            
        //TAPS ARE REGISTERED SO STOP READING FILTER ROM
        dpu_h_rom_rd = 1'b0;        
        
        //START LOOP VARIABLE
        filter_loop = ROWS;
        
        //next_state = F_LOAD
        
        next_state = F_IDLE2;
        
    end
    
    F_LOAD: begin //STATE 5
    
        //INCREMENT ADDRESS TO READ
        dpu_x_rom_addr = dpu_x_rom_addr + 1'b1;
        
        //GO TO IDLE AND COME BACK TO RETRIGGER ALWAYS BLOCK
        next_state = F_IDLE;
        
        //INCREMENT LOOP COUNTER
        filter_loop = filter_loop - 1'b1;
        
        next_state = F_IDLE;
    
    end
    
    F_IDLE: begin //STATE 6
        
        if(filter_loop > 0) begin
            next_state = F_LOAD;
        end
        else if(filter_loop == 0) begin
            next_state = RST;
        end
    
    end
    
    F_IDLE2: begin //STATE 7
        next_state = F_LOAD;
    
    end
    
    LINGO: begin
        next_state = LINGO;
    end
    
    default: next_state = LINGO;
    endcase
end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////

module SLOW_CLK #(parameter DIVIDE = 5) (
input clk,
output slow_clk
);

//10ns period = 100MHz = 1E8 clock speed for datapath
//50ns period = 20MHz = 2E7clock speed for controller
reg [31:0] counter;
reg slow_clk_reg;

//ASSUME PARAM IS 32 BITS

initial counter = {32{1'b0}};
initial slow_clk_reg = 1'b0;

assign slow_clk = slow_clk_reg;

always@(posedge clk) begin
    counter = counter + 1'b1;
    
    if(counter == DIVIDE) begin
        slow_clk_reg = ~slow_clk_reg;
        counter = {32{1'b0}};
    end
end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////

module ROM #(parameter BITS = 16, ROWS = 16, FILENAME="filter_memory.mem") (
input [$clog2(ROWS)-1:0] addr,
input rd,
input clk,

output reg [BITS-1:0] out
);

reg [BITS-1:0] rom [0:ROWS-1];

initial begin
    $readmemh(FILENAME, rom);
end

always@(posedge clk) begin
    if(rd) out = rom[addr];
    else out = {BITS{1'bz}};
end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////

module DREGISTER #(parameter BITS = 16) (
input [BITS-1:0] in,
input clr,
input clk,
input enable,

output [BITS-1:0] out,

//DIAGNOSTICS
output clk2
);

//Clear all registers if clr flag is set
wire [BITS-1:0] clr_wire;
assign clr_wire = clr ? 1 : 0;

//"latch" enable
reg en;
initial en = 1'b1;

always@(*) begin
    en <= enable;
end

//Clock enable
assign clk2 = en ? clk : 0;

genvar i;
generate
    for(i = 0; i < BITS; i = i+1) begin
        DFF DFF_INST(
        .d(in[i]),
        .clr(clr),
        .clk(clk2),
        
        .q(out[i])
        );
    end
endgenerate


endmodule

///////////////////////////////////////////////////////////////////////////////////////////

module DFF(
input d,
input clr,
input clk,

output reg q
);

initial q = 1'b0;

always@(posedge clk, posedge clr) begin
    if(clr) q <= 1'b0;
    else begin
        if((d === 1'bx) | (d === 1'bz)) q <= q;
        else q <= d; 
    end
    
end
endmodule

///////////////////////////////////////////////////////////////////////////////////////////

module DATAPATH #(parameter BITS = 16, ORDER = 4, ROWS = 16) (
input clk,

input [$clog2(ROWS)-1:0] h_rom_addr,
input h_rom_rd,

input [$clog2(ROWS)-1:0] x_rom_addr,
input x_rom_rd,

input [ORDER:0] clrs, 
input [ORDER:0] enables,

output [BITS*2-1:0] y,

//DIAGNOSTICS
output [BITS*(ORDER+1)-1:0] h_out,
output [BITS-1:0] x_out,

output [BITS*(ORDER+1)-1:0] tap_reg_out_test,
output [BITS*2*(ORDER+1)-1:0] y_test,
output [ORDER:0] clk2,
output [BITS-1:0] x_reg_test,
output [2*(ORDER+1)-1:0] ctr_out_test
);

localparam NUM_TAPS = ORDER+1;
localparam H_LENGTH = BITS*(NUM_TAPS);

//INITIALIZE FILTER ROM
wire [BITS-1:0] h_rom_data;

ROM FILTER_ROM(
.addr(h_rom_addr),
.rd(h_rom_rd),
.clk(clk),

.out(h_rom_data)
);

defparam FILTER_ROM.FILENAME = "filter_memory.mem";
defparam FILTER_ROM.BITS = BITS;
defparam FILTER_ROM.ROWS = ROWS;

//INITIALIZE SIGNAL ROM
wire [BITS-1:0] x_rom_data;

ROM SIGNAL_ROM(
.addr(x_rom_addr),
.rd(x_rom_rd),
.clk(clk),

.out(x_rom_data)
);

defparam SIGNAL_ROM.FILENAME = "signal_memory.mem";
defparam SIGNAL_ROM.BITS = BITS;
defparam SIGNAL_ROM.ROWS = ROWS;

//FOR FILTER
wire [H_LENGTH-1:0] h_rom_data_array;

assign h_rom_data_array = {NUM_TAPS{h_rom_data}};

//DIAGNOSTICS
assign h_out = h_rom_data_array;
assign x_out = x_rom_data;

wire [BITS-1:0] x_reg;
assign x_reg_test = x_reg;

//HOLDS X DATA FROM X_ROM TO PIPELINE
DREGISTER X_REG(
.in(x_rom_data),
.clr(clrs[0]), //WHEN FILTER TAPS ARE CLEARED, CLEAR X REG TOO
.clk(clk),
.enable(1'b1), //ALWAYS ENABLED BECAUSE SHORTED TO X ROM
//TAP REGISTERS HAVE ENABLE WIRING BECAUSE THEY SHARE THE SAME ROM
.out(x_reg)
);

defparam X_REG.BITS = BITS;

FILTER FIR(
.x(x_reg),
.h(h_rom_data_array),

.clk(clk),
.clrs(clrs),
.enables(enables),

.y(y),

//DIAGNOSTICS
.tap_reg_out_test(tap_reg_out_test),
.y_test(y_test),
.clk2(clk2),
.ctr_out_test(ctr_out_test)
);

defparam FIR.BITS = BITS;
defparam FIR.ORDER = ORDER;

endmodule

///////////////////////////////////////////////////////////////////////////////////////////

module FILTER #(parameter BITS = 16, parameter ORDER = 4) (
input [BITS-1:0] x,
input [BITS*(ORDER+1)-1:0] h,
//TAP REGISTERS
input clk,
input [ORDER:0] clrs,
input [ORDER:0] enables,

output [BITS*2-1:0] y,

//DIAGONSTICS
output [BITS*(ORDER+1)-1:0] tap_reg_out_test,
output [BITS*2*(ORDER+1)-1:0] y_test,
output [ORDER:0] clk2,
output [2*(ORDER+1)-1:0] ctr_out_test
);
//------------------------------------SETUP---------------------------------
//Wire array for setting up TAPS
wire [BITS-1:0] hn [ORDER:0];

//Feed forward output from each tap module
wire [BITS*2-1:0] tap_data [ORDER:0];

//Breaks down expanded h input
genvar c;
generate
    for(c = ORDER; c>=0; c=c-1) begin
    //TODO: REVERSE ORDER SO USER DOES NOT HAVE TO FLIP IMPULSE RESPONSE
        assign hn[c] = h[BITS*(c+1)-1 : (BITS*(c+1)-1) - (BITS-1)];
    end

endgenerate
//------------------------------------SETUP---------------------------------

//--------------------------------DEBUGGING---------------------------------

wire [BITS-1:0] tap_reg_out [ORDER:0];

//EXPANDED VECTOR TO SEE THE REGISTER CONTENTS OF EACH TAP
genvar G;
generate
    for(G = ORDER; G >= 0; G = G-1) begin
        assign tap_reg_out_test[BITS*(G+1)-1 : (BITS*(G+1)-1) - (BITS-1)] 
        = tap_reg_out[G];
    end
endgenerate


assign y = tap_data[0];

//THIS GENERATE CREATES AN EXPANDED VECTOR TO SEE THE OUTPUT OF EACH TAP
genvar t;
generate
    for(t = ORDER; t >= 0; t = t-1) begin
        assign y_test[BITS*2*(t+1)-1 : (BITS*2*(t+1)-1) - (BITS*2-1)] 
        = tap_data[t];
    end
endgenerate

wire [1:0] ctr_out [ORDER:0];

//THIS GENERATE CREATES AN EXPANDED VECTOR TO SEE THE CTR BITS OF EACH TAP
genvar k;
generate
    for(k = ORDER; k >= 0; k = k-1) begin
        assign ctr_out_test[2*(k+1)-1 : (2*(k+1)-1) - (1)] 
        = ctr_out[k];
    end
endgenerate

//--------------------------------DEBUGGING---------------------------------

//--------------------------------LOGIC-------------------------------------
genvar i;
generate
for(i=ORDER; i>=0; i=i-1) begin
    
    //IN BETWEEN
    if((i!=0) & (i!=ORDER)) begin
        TAP_X #(BITS) TAP_X_INST(
            .tap(hn[i]),
            .x(x),
            .prev_stage(tap_data[i+1]),
            
            .clr(clrs[i]),
            .enable(enables[i]),
            .clk(clk),
            
            .out(tap_data[i]),
            
            //DIAGNOSTICS
            .tap_reg_out(tap_reg_out[i]),
            .clk2(clk2[i])
        );
    end
    
    //HIGHEST ORDER
    else if(i==ORDER) begin
        TAP_M #(BITS) TAP_M_INST(
            .tap(hn[i]),
            .x(x),
            
            .clr(clrs[i]),
            .enable(enables[i]),
            .clk(clk),
            
            .out(tap_data[i]),
            
            //DIAGNOSTICS
            .tap_reg_out(tap_reg_out[i]),
            .clk2(clk2[i]),
            .ctr_out(ctr_out[i])
        );
    end
    
    //LOWEST ORDER (OUTPUT)
    else if(i==0) begin
        TAP_0 #(BITS) TAP_0_INST(
            .tap(hn[i]),
            .x(x),
            .prev_stage(tap_data[i+1]),
            
            .clr(clrs[i]),
            .enable(enables[i]),
            .clk(clk),
            
            .out(tap_data[i]),
            
            //DIAGNOSTICS
            .tap_reg_out(tap_reg_out[i]),
            .clk2(clk2[i])
        );
    end
end
endgenerate

//--------------------------------LOGIC-------------------------------------

endmodule

///////////////////////////////////////////////////////////////////////////////////////////

//Processes the highest order of the filter
//Includes a pipeline register, a multiplier, and a mux
module TAP_M #(parameter BITS = 16) (
input [BITS-1:0] tap,
input [BITS-1:0] x,

//For Tap register
input clr,
input enable,
input clk,

output reg [BITS*2-1:0] out,

//DIAGNOSTICS
output [BITS-1:0] tap_reg_out,
output clk2,
output [1:0] ctr_out
);

wire [BITS-1:0] tap_reg;
assign tap_reg_out = tap_reg;

//Setup Register to hault tap until all registers are done
DREGISTER TAP_REGISTER(
.in(tap),
.clr(clr),
.clk(clk),
.enable(enable),

.out(tap_reg),
.clk2(clk2)
);

defparam TAP_REGISTER.BITS = BITS;

//Instantiate Array Multiplier   
wire [BITS*2-1:0]   mult_out;

ARRAY_MULTIPLIER #(BITS) MULT (
.a(tap_reg),
.b(x),

.p(mult_out)
);

//SETUP CTR FOR PIPELINING ANSWER
reg [1:0] ctr;
initial ctr = 0;

//INITIAL OUTPUT IS 0 TO PREVENT ERRORS
initial out = 0;

//WIRE OUT DIAGNOSTICS
assign ctr_out = ctr;

//Clock pipeline register
always@(posedge clk) begin
    ctr = ctr + 1;
    
    if((x[0] === 1'bx) | (x[0] === 1'bz)) out <= {(BITS*2){1'b0}};
    if(ctr > 1'b1) begin
        out <= mult_out;
        ctr = 2'b00;
    end
    
end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////

//Processes stages in between highest order and lowest order of filter
//Includes a pipeline register, a multiplier, an adder, and a mux
module TAP_X #(parameter BITS = 16)(
input [BITS-1:0] tap,
input [BITS-1:0] x,
input [BITS*2-1:0] prev_stage,

//FOR TAP REGISTER
input clr,
input enable,
input clk,

output reg [BITS*2-1:0] out,

//DIAGNOSTICS
output [BITS-1:0] tap_reg_out,
output clk2
);

wire [BITS-1:0] tap_reg;
assign tap_reg_out = tap_reg;

//Setup Register to hold tap from ROM
DREGISTER TAP_REGISTER(
.in(tap),
.clr(clr),
.clk(clk),
.enable(enable),

.out(tap_reg),
.clk2(clk2)
);

defparam TAP_REGISTER.BITS = BITS;

//Instantiate Array Multiplier   
wire [BITS*2-1:0]   mult_out;

ARRAY_MULTIPLIER #(BITS) MULT (
.a(tap_reg),
.b(x),

.p(mult_out)
);

//Instantiate CLA
wire [BITS*2-1:0]   cla_out;

CLA #(BITS*2) ADDER(
.A(mult_out),
.B(prev_stage),
.Cin(1'b0),

.Sum(cla_out),
.Cout()
);

reg [1:0] ctr;
initial ctr = 0;
initial out = 0;

//Clock pipeline register
always@(posedge clk) begin
    ctr = ctr + 1;

    if((x[0] === 1'bx) | (x[0] === 1'bz)) out <= {(BITS*2){1'b0}};
    if(ctr > 1'b1) begin
        out <= cla_out;
        ctr = 2'b00;
    end
end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////

//Processes the lowest order of the filter
//Includes a a multiplier and a mux
module TAP_0 #(parameter BITS = 16) (
input [BITS-1:0] tap,
input [BITS-1:0] x,
input [BITS*2-1:0] prev_stage,

input clr,
input enable,
input clk,

output reg [BITS*2-1:0] out,

//DIAGNOSTICS
output [BITS-1:0] tap_reg_out,
output clk2
);

//Setup Register to hold tap from ROM
wire [BITS-1:0] tap_reg;
assign tap_reg_out = tap_reg;

DREGISTER TAP_REGISTER(
.in(tap),
.clr(clr),
.clk(clk),
.enable(enable),

.out(tap_reg),
.clk2(clk2)
); 

defparam TAP_REGISTER.BITS = BITS;

//Instantiate Array Multiplier    
wire [BITS*2-1:0]   mult_out;

ARRAY_MULTIPLIER #(BITS) MULT (
.a(tap_reg),
.b(x),

.p(mult_out)
);

//Instantiate CLA
wire [BITS*2-1:0]   cla_out;

CLA #(BITS*2) ADDER(
.A(mult_out),
.B(prev_stage),
.Cin(1'b0),

.Sum(cla_out),
.Cout()
);

reg [1:0] ctr;
initial ctr = 0;
initial out = 0;

//Clock pipeline register
always@(posedge clk) begin
    ctr = ctr + 1;
    
    if((x[0] === 1'bx) | (x[0] === 1'bz)) out <= {(BITS*2){1'b0}};
    if(ctr > 1'b1) begin
        out <= cla_out;
        ctr = 2'b00;
    end
end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
////USE OF PREVIOUSLY WRITTEN MODULES (CLA AND ARRAY MULTIPLIER)

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

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

//PARAMETER BITS SPECIFIES THE INPUTS
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