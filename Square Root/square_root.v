// Code your design here
// Code your design here
module main_module(Go, N, answer, over, clock, reset);
  
  
 
	input clock;
	input reset;
  input [7:0] N;
  input Go;
  
  output [3:0] answer;
  output over;

	wire Go;
	wire [7:0] N;
	wire clock ;
 //  t = 0;  
   // clk = 1;
     // #20
      //while(t<2500)
      //begin
       // #2
        //clk = ~clk;
	wire reset;
  wire over;
  wire borrow;
  wire [2:0] func;
  wire [3:0] load;
  wire [3:0] transfer;

	
  

  
  controller c(over,load, transfer, func, borrow, Go, clock,reset);
  //now we call the datapath on transfer, load , func ,and borrow and answer
  
  datapath d(transfer, load, func, N, borrow, answer);

endmodule

module controller(over,load, transfer, func, borrow, Go, clock,reset);
  
  
  	output [3:0] load;
	output [3:0] transfer;	
	input borrow;
	
	input reset;
  input Go;
  
	input clock;
 // wire [2:0] func;
  //wire [3:0] load;
	wire  Go;
	wire [3:0] internal_load;
  	wire [3:0] transfer;
	wire [3:0] load;
  	reg[3:0] holdval;
  wire reset;
  wire clock;
 	
  
  reg[3:0] testval;
	output [2:0] func;
  reg[3:0] goval;
	wire [2:0] func;
	output over;
	wire over;

  and a1(load[0],internal_load[0],(~clock));
  and a2(load[1],internal_load[1],(~clock));
  //and g3(w2[i], w1[i], nx[i]);
  and a3(load[2],internal_load[2],(~clock));
//  state2: next_sshktate = state3;
//			is this correct? state3: next_state = state4;
  and a4(load[3],internal_load[3],(~clock));
  
	parameter SIZE = 4;
  //parameter state = 4'b0090
 // state5 = 4'b0110,  state8 = 4'b1001, state9 = 4'b1010, state6 = 4'b0111, state7 = 4'b1000,
	
  wire [3:0] state;
  wire [3:0] next_state;
  nextstate nxt(state,next_state,Go,borrow);
	
  register_new yo(state,next_state,clock,reset);
 	
	
  func_over funov(state,func,over);
  load_trans loatr(state,internal_load,transfer);

endmodule

// Code your design here
module nextstate(x,y,Go,borrow);
  input [3:0] x;
  output [3:0] y;
  input Go;
  input borrow;
  
  wire [4:0] nx;
  wire w1,w2,w3,w4,w5,w6,w7,w8,w9,w10,w11,w12,w13,w14,w15,w16,w17,w18,w19,w20,w21,w22,w23,w24;
  wire nGo;
  
  not nr1(nx[0],x[0]);
  not nr2(nx[1],x[1]);
  not nr3(nx[2],x[2]);
  not nr4(nx[3],x[3]);
  not nr5(nGo,Go);
  
  and ar1(w1,nx[0],x[1]); // x[1](~x[0])				//1 w1 
  and ar2(w2,x[3],x[1]); // x[3]x[1]					//2 w2 
  and ar3(w3,x[3],x[0]); // x[3]x[0]					//8 w3
  and ar4(w4,nx[1],nx[0]); //(~x[1])(~x[0])
  and ar5(w5,nx[1],x[0]); //(~x[1])x[0]
  and ar6(w6,x[3],x[2]); //x[3]x[2]
  and ar7(w7,nx[3],x[2]); //(~x[3])x[2]
  and ar8(w8,w4,nx[2]); //(~x[2])(~x[1])(~x[0])			//3 w8
  and ar9(w9,w4,nx[3]); //(~x[3])(~x[1])(~x[0])			//4 w9
  and ar10(w10,w5,x[2]); //x[2](~x[1])x[0]				//9 w10
  and ar11(w11,w7,nx[1]); //(~x[3])x[2](~x[1])			//11 w11
  and ar12(w12,x[1],x[0]); //x[1]x[0]
  and ar13(w13,nx[2],w12); //(~x[2])x[1]x[0]			//12 w13
  and ar14(w14,x[2],w1); // x[2]x[1](~x[0])				//13 w14
  and ar15(w15,nx[2],nx[1]); // (~x[2])(~x[1])
  and ar16(w16,nx[3],w15); // (~x[3])(~x[2])(~x[1])
  and ar17(w17,x[3],w15);  // x[3](~x[2])(~x[1])		//16 w17
  and ar18(w18,x[3],w1);  // x[3]x[1](~x[0])			//17 w18
  and ar19(w19,x[2],w12); //x[2]x[1]x[0]				//18 w19
  and ar20(w20,nGo,w4); //(~Go)(~x[1])(~x[0])			//5 w20
  and ar21(w21,nGo,w15); // (~Go)(~x[3])(~x[2])(~x[1])	//6 w21
  and ar22(w22,Go,w5); //Go(~x[1])x[0]					//10 w22
  and ar23(w23,Go,w6); //Gox[3]x[2]						//14 w23
  and ar24(w24,borrow,w7); //borrow(~x[3])x[2]			//15 w24
  or or1(y[0],w1,w2,w8,w9,w20,w21);
  or or2(y[1],w1,w3,w10,w22);
  or or3(y[2],w11,w13,w14,w23,w24);
  or or4(y[3],w17,w18,w19,w23);
  
endmodule
  
  module load_trans(x,load,trans);
input [3:0] x;
output [3:0] load;
output [3:0] trans;

	wire [4:0] nx;
  	wire w1,w2,w3,w4,w5,w6,w7,w8,w9,w10,w11,w12,w13,w14,w15;
  
  not nr1(nx[0],x[0]);
  not nr2(nx[1],x[1]);
  not nr3(nx[2],x[2]);
  not nr4(nx[3],x[3]);

  and ar1(load[0],nx[3],nx[2],x[1],nx[0]);
  and ar2(load[3],nx[3],nx[2],x[1],x[0]);
  and ar3(w1,nx[3],x[2],x[1],nx[0]);
  and ar4(w2,x[3],nx[2],nx[1],nx[0]);
  or or1(load[2],w1,w2);
  and ar5(w3,nx[3],x[2],nx[1]);
  and ar6(w4,x[3],x[0]);
  and ar7(w5,x[3],x[1]);
  or or2(load[1],w3,w4,w5);


  and ar8(trans[0],nx[3],nx[2],x[1],x[0]);
  and ar9(trans[3],nx[3],x[2],x[1],x[0]);
  and ar10(w6,x[3],nx[2],nx[1]);
  and ar12(w12,x[3],nx[2],x[1],nx[0]);
  or or3(trans[2],w6,w12);
  and ar11(w7,nx[3],x[2],x[0]);
  and ar13(w8,x[3],nx[2],x[1]);
  and ar14(w9,x[3],nx[2],nx[1],x[0]);
  or or4(trans[1],w7,w8,w9);

endmodule


module func_over(x,func,over);
input [3:0] x;
output [2:0] func;
output over;

	wire [4:0] nx;
  	wire w1,w2,w3,w4,w5,w6,w7,w8,w9,w10,w11,w12,w13,w14,w15;
  
  not nr1(nx[0],x[0]);
  not nr2(nx[1],x[1]);
  not nr3(nx[2],x[2]);
  not nr4(nx[3],x[3]);


  and ar1(w1,nx[1],x[0]); //(~x[1])x[0]
  and ar2(w2,w1,x[2],nx[3]); //x[2](~x[1])x[0]		//1
  and ar3(w3,nx[1],nx[0]); //(~x[1])(~x[0])
  and ar4(w4,w3,nx[2],x[3]); //(~x[2])(~x[1])(~x[0])		//2
  and ar5(w5,x[1],x[0]); //x[1]x[0]
  and ar6(w6,x[3],w5,nx[2]); //x[3]x[1]x[0]			//3
  and ar7(w7,x[2],w5,nx[3]); //x[2]x[1]x[0]			//4
  and ar8(w8,x[3],w1,nx[2]); //x[3](~x[1])x[0]		//5
  and ar9(w9,nx[0],x[1]); // x[1](~x[0])
  and ar10(w10,x[3],w9,nx[2]);  // x[3]x[1](~x[0])		//6
  and ar11(w11,nx[3],x[2]); //(~x[3])x[2]
  and ar12(w12,w11,nx[0]); //(~x[3])x[2](~x[0])		//7
  and ar13(w13,nx[2],nx[1]); // (~x[2])(~x[1])
  and ar14(w14,x[3],w13);  // x[3](~x[2])(~x[1])	//8
  and ar15(w15,x[3],w9,nx[2]);  // x[3]x[1](~x[0])		//9
  and ar16(over,x[3],x[2],nx[1],nx[0]);

  or or1(func[2],w2,w4,w6);
  or or2(func[1],w7,w8,w10);
  or or3(func[0],w12,w14,w15);

endmodule



//the controller module starts below this

 	

module register_new(state,next_state,clock,reset);
  input [3:0] next_state;
  output [3:0] state;
  input reset,clock;
  reg [3:0] state;
always@(posedge clock)
 	begin
 		if(reset==1'b1)          
 			state = 4'b0001;      
 		else
 			state = next_state;
 	end
endmodule

module status_det(x,y,Flag,func);
  input [8:0] x;
	input [8:0] y;
 //the three wires tha we will need
wire [8:0] w1;
wire [8:0] w2;
  wire[8:0] testwire2;

wire [8:0] w3;
  wire[8:0] testwire1;
  
   input func;
  output Flag;
  
  wire [9:0] borrow_w;
  wire [8:0] nx;
  
  integer i=0;
  assign borrow_w[0] = 1'b0;
  generate
 
    not g1(nx[0], x[0]);
    or g2(w1[0], borrow_w[0], y[0]);
    and g3(w2[0], w1[0], nx[0]);
    and g4(w3[0], y[0], borrow_w[0]);
    or g5(borrow_w[1], w2[0], w3[0]);
    //1;
    not g1(nx[1], x[1]);
    or g2(w1[1], borrow_w[1], y[1]);
    and g3(w2[1], w1[1], nx[1]);
    and g4(w3[1], y[i], borrow_w[1]);
    or g5(borrow_w[2], w2[1], w3[1]);
  //  2;
    not g1(nx[2], x[2]);
    or g2(w1[2], borrow_w[2], y[2]);
    and g3(w2[2], w1[2], nx[2]);
    and g4(w3[2], y[2], borrow_w[2]);
    or g5(borrow_w[3], w2[2], w3[2]);
   // 3;
    not g1(nx[3], x[3]);
    or g2(w1[3], borrow_w[3], y[3]);
    and g3(w2[3], w1[3], nx[3]);
    and g4(w3[3], y[3], borrow_w[3]);
    or g5(borrow_w[4], w2[3], w3[3]);
  //  4;
  not g1(nx[4], x[4]);
      or g2(w1[4], borrow_w[4], y[4]);
      and g3(w2[4], w1[4], nx[4]);
      and g4(w3[4], y[4], borrow_w[4]);
      or g5(borrow_w[5], w2[4], w3[4]);
  //  5;
    not g1(nx[5], x[5]);
      or g2(w1[5], borrow_w[5], y[5]);
      and g3(w2[5], w1[5], nx[5]);
      and g4(w3[5], y[5], borrow_w[5]);
      or g5(borrow_w[6], w2[5], w3[5]);
  //  6;
    not g1(nx[6], x[6]);
      or g2(w1[6], borrow_w[6], y[6]);
      and g3(w2[6], w1[6], nx[6]);
      and g4(w3[6], y[6], borrow_w[6]);
      or g5(borrow_w[7], w2[6], w3[6]);
  //  7;
    not g1(nx[7], x[7]);
      or g2(w1[7], borrow_w[7], y[7]);
      and g3(w2[7], w1[7], nx[7]);
      and g4(w3[7], y[7], borrow_w[7]);
      or g5(borrow_w[8], w2[7], w3[7]);
  //  8;
    not g1(nx[8], x[8]);
      or g2(w1[8], borrow_w[8], y[8]);
      and g3(w2[8], w1[8], nx[8]);
      and g4(w3[8], y[8], borrow_w[8]);
      or g5(borrow_w[9], w2[8], w3[8]);
    
  endgenerate
  and g(Flag,borrow_w[9],func);
endmodule

module adder(x,y,sum,cout,cin);
	output sum,cout;
	input x,y,cin;
	wire w1,w2,w3;
  and ar1(w1,x,cin);
	and ar2(w2,x,y);
	and ar3(w3,y,cin);
	or u5(cout,w1,w2,w3);
	xor x1(sum,x,y,cin);
endmodule

module adder_9(sum,x,y);
	input [8:0] x;
	input [8:0] y;
	output [8:0] sum;
	wire [9:0]c;
	assign c[0] = 0;
	generate
	genvar i;
		for(i=1;i<10;i=i+1) begin
          adder addresult(x[i-1],y[i-1],sum[i-1],c[i],c[i-1]);
		end
	endgenerate
endmodule



// mux 2to1
module mux2to1(a,b,sel,out);
	input a,b,sel;
	output out;
	tri out;
	bufif1 (out,a,sel);
	bufif0 (out,b,sel);
endmodule
 
//mux 4to1
module mux4to1(a,sel,out);
	input [3:0] a;
	input [1:0] sel;
	output out;
	wire mux[2:0];
 
	mux2to1 m1 (a[3],a[2],sel[0],mux[1]),
	        m2 (a[1],a[0],sel[0],mux[0]),
	        m3 (mux[1],mux[0],sel[1],out);
endmodule
 
module tristate_buffer_9(input_x, enable, output_x);
  input [8:0]input_x;
  input enable;
  output [8:0]output_x;
  
  generate
    genvar i;
    for(i=0;i<9;i=i+1)
      begin
        bufif1 b1(output_x[i], input_x[i], enable);
      end
  endgenerate
  
endmodule


//mux 8to1
module mux8to1(a,sel,out);
	input [7:0] a;
	input [2:0] sel;
	output out;
	wire mux[2:0];
 
 	mux4to1 m1 (a[7:4],sel[1:0],mux[1]),
	        m2 (a[3:0],sel[1:0],mux[0]);
	mux2to1 m3 (mux[1],mux[0],sel[2],out);
endmodule

module alu(x,y,z,borrow,func);
  input [8:0] x;
  output borrow;
  input [8:0] y;
  output [8:0] z;
  input [2:0] func;
  //the alu module 
  wire [8:0]z;
  reg[8:0] kkl;
  status_det check(y,x,borrow,(((~func[2])&&func[1])&&(~func[0])));  
  
   wire [8:0] xpy,ymx,xp1,yp1;
  adder_9 add1(xpy,y,x);
  adder_9 add2(xp1,1,x);
  adder_9 add3(yp1,1,y);
  //sub_9 sub1(ymx,x,y);
 
          //0
          mux8to1 m0({1'bx,1'bx,yp1[0],xp1[0],xpy[0],ymx[0],1'b0,x[0]},(func),z[0]);
          //1
          mux8to1 m1({1'bx,1'bx,yp1[1],xp1[1],xpy[1],ymx[1],1'b0,x[1]},(func),z[1]);
          //2
          mux8to1 m2({1'bx,1'bx,yp1[2],xp1[2],xpy[2],ymx[2],1'b0,x[2]},(func),z[2]);
          //3
          mux8to1 m3({1'bx,1'bx,yp1[3],xp1[3],xpy[3],ymx[3],1'b0,x[3]},(func),z[3]);
          //4
          mux8to1 m4({1'bx,1'bx,yp1[4],xp1[4],xpy[4],ymx[4],1'b0,x[4]},(func),z[4]);
          //5
          mux8to1 m5({1'bx,1'bx,yp1[5],xp1[5],xpy[5],ymx[5],1'b0,x[5]},(func),z[5]);
          //6
          mux8to1 m6({1'bx,1'bx,yp1[6],xp1[6],xpy[6],ymx[6],1'b0,x[6]},(func),z[6]);
          //7
          mux8to1 m7({1'bx,1'bx,yp1[7],xp1[7],xpy[7],ymx[7],1'b0,x[7]},(func),z[7]);
          //8
          mux8to1 m8({1'bx,1'bx,yp1[8],xp1[8],xpy[8],ymx[8],1'b0,x[8]},(func),z[8]);

 
endmodule



module datapath(transfer, load, func, inp, Flag, answer);
  
  input [7:0] inp;
  input [3:0] transfer;              
 
  output [3:0] answer;
  
   input [3:0] load;               
  input [2:0] func;
  output Flag;
  
  wire [8:0] bus_x;
wire [8:0] bus_z;
wire [8:0] bus_y;
  
  wire [7:0] reg_Sw;
 
  wire [8:0] reg_sum;
  
   wire [3:0] reg_k;          
  wire [7:0] reg_N;
  
  assign answer = reg_k;
  
 
  tristate_buffer_9 sw1({1'b0,reg_Sw},transfer[0],bus_x);
  register_8 Sw(inp, reg_Sw, 1, load[0]);

 
  tristate_buffer_9 sw2(reg_sum,transfer[1],bus_x);
  register_9 sum(bus_z, reg_sum, 1, load[1]);

 
 tristate_buffer_9 sw3({5'b00000,reg_k},transfer[2],bus_y);
  register_4 k(bus_z[3:0], reg_k, 1, load[2]);

 
 tristate_buffer_9 sw4({1'b0,reg_N},transfer[3],bus_y);
  register_8 N(bus_z[7:0], reg_N, 1, load[3]);
  //this is the call for the alu
  alu alu_1(bus_x, bus_y, bus_z, Flag, func);
endmodule


module register_8(in,out,load,clock);
  input [7:0] in;
  output [7:0] out;
  input load,clock;
  reg [7:0]out;
  always@(posedge clock)
  begin
    if (load==1)
    out=in;
  end
endmodule

module register_4(in,out,load,clock);
  input [3:0] in;
  output [3:0] out;
  input load,clock;
  reg [3:0]out;
  always@(posedge clock)
   begin
    if (load==1)
    out=in;
  end
endmodule

module register_9(in,out,load,clock);
  input [8:0] in;
  output [8:0] out;
  input load,clock;
  reg [8:0]out;
  always@(posedge clock)
  begin
    if (load==1)
    out=in;
  end
endmodule
