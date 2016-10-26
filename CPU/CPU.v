// Code your design here
module CPU(regbankmatrix,IRout,PCout,buffout,Current_State,clk,rst);
  wire[15:0] MemData;
  wire[15:0] MemAddr;
  output [127:0] regbankmatrix;
  wire memrw,d;
  input clk,rst;
  output dout;
  
  wire [2:0]loadsel,transsel;
  wire [2:0]fsel;
  wire isload,ldbf,ldPC,ldMAR,ldMDR,ldTemp,ldIR,rst,ldcond;
  wire T2,TT,TMAR,TPC_A,TPC_M,TMDRX,TMDRI,RMDRX,RMDRI,Treg;
  output [15:0] IRout,PCout,buffout;
 
  output [4:0] Current_State;

  Controller aslkdf(ldcond,loadsel,isload,ldbf,ldPC,ldMAR,ldMDR,ldTemp,ldIR,T2,TT,TMAR,TPC_A,TPC_M,TMDRX,TMDRI,RMDRX,RMDRI,Treg,transsel,fsel,d,IRout,rst,clk,memrw,Current_State);
  memory ajsdf(MemAddr,MemData,1'b1,memrw,1'b1,rst);
  datapath daksdf(regbankmatrix,IRout,PCout,buffout,d,loadsel,transsel,fsel,isload,ldbf,ldPC,ldMAR,ldMDR,ldTemp,ldIR,ldcond,rst,T2,TT,TMAR,TPC_A,TPC_M,TMDRX,TMDRI,RMDRX,RMDRI,Treg,MemData,MemAddr);
  
endmodule

module memory (
address     , // Address Input
data        , // Data bi-directional
cs          , // Chip Select
we          , // Write Enable/Read Enable
oe          ,  // Output Enable
rst 
);          
parameter RAM_DEPTH = 1 << 16;
//--------------Input Ports----------------------- 
  input [15:0] address ;
input                                     cs           ;
input                                     we,rst          ;
input                                     oe           ; 

//--------------Inout Ports----------------------- 
  inout [15:0]  data       ;

//--------------Internal variables---------------- 
  reg [15:0]   data_out ;
  reg [15:0] mem [0:RAM_DEPTH-1];

//--------------Code Starts Here------------------ 

// Tri-State Buffer control 
// output : When we = 0, oe = 1, cs = 1
assign data = (cs && oe && !we) ? data_out : 8'bz; 
  always @ (rst)
  begin
    if (rst==1)
      begin
        mem[0] = 16'b0000000000000000;// li r0,0
        mem[1] = 16'b0000000000000000;//
        mem[2] = 16'b0100000000000000;//addi r0,1
        mem[3] = 16'b0000000000000001;//
        mem[4] = 16'b0000000101000000;//lr r1,r0
        mem[5] = 16'b0000001100000000;//li r3,5
        mem[6] = 16'b0000000000000101;//
        mem[7] = 16'b0100101100000000;//subi r3,1
        mem[8] = 16'b0000000000000001;//
        mem[9] = 16'b0000001001011000;//lr r2,r3
        mem[10] = 16'b0100101000000000;//subi r2,1
        mem[11] = 16'b0000000000000001;//
        mem[12] = 16'b0101001001001000;//mnsr r2,r1
        mem[13] = 16'b1000100000000000;//jnz ... (-4){as PC will be 14 while reading -4}
        mem[14] = 16'b1111111111111100;//
        mem[15] = 16'b1111111111111111;//exit
      end
  end
  

// Memory Write Block 
// Write Operation : When we = 1, cs = 1
always @ (address or data or cs or we)
begin : MEM_WRITE
   if ( cs && we ) begin
       mem[address] = data;
   end
end

// Memory Read Block 
// Read Operation : When we = 0, oe = 1, cs = 1
always @ (address or cs or we or oe)
begin : MEM_READ
    if (cs && !we && oe)  begin
         data_out = mem[address];
    end
end

endmodule // End of Module ram_sp_ar_aw

// And 4 input
module and_4(out,a1,a2,a3,a4);
  output out;
  input a1,a2,a3,a4;
  wire w1,w2;
  and x1(w1,a1,a2);
  and x2(w2,a3,a4);
  and x3(out,w1,w2);
endmodule
 
// Or 4 input
module or_4(out,a1,a2,a3,a4);
  output out;
  input a1,a2,a3,a4;
  wire w1,w2;
  or x1(w1,a1,a2);
  or x2(w2,a3,a4);
  or x3(out,w1,w2);
endmodule
 
// dff latch
module dff_latch(Q,D,CLK);
  output Q;
  input D,CLK;
  wire nr,inp,X,Y,Q_BAR;
  nand U1 (X,D,CLK) ;
  nand U2 (Y,X,CLK) ;
  nand U3 (Q,Q_BAR,X);
  nand U4 (Q_BAR,Q,Y);
endmodule
 
// positive edge triggered dff
module dff(out,in,clk,rst);
  output out;
  input in,clk,rst;
  wire Q1;
  wire inp,nrst,nclk;
  not n1(nrst,rst);
  not n2(nclk,clk);
  and al(inp,in,nrst);
  dff_latch d1(Q1,inp,nclk);
  dff_latch d2(out,Q1,clk);
endmodule
 
// structural code for register_16
module register_16(out,in,load,clk,rst);
  output [15:0] out;
  input load,clk,rst;
  input [15:0] in;
 
  generate
    genvar n;
    for(n=0;n<16;n=n+1) begin : asdfg
          dff d(out[n],in[n],clk,rst);
    end
  endgenerate
endmodule
 
// structural code for register_4
module register_4(out,in,load,clk,rst);
  output [3:0] out;
  input load,clk,rst;
  input [3:0] in;
 
  generate
    genvar n;
    for(n=0;n<4;n=n+1) begin : asdfg
          dff d(out[n],in[n],clk,rst);
    end
  endgenerate
endmodule
 
 
 
// mux 8to1
module mux8to1(a,sel,out);
  input [7:0] a;
  input [2:0] sel;
  output out;
  wire mux[2:0];
 
  mux4to1 m1 (a[7:4],sel[1:0],mux[1]),
  m2 (a[3:0],sel[1:0],mux[0]);
  mux2to1 m3 (mux[1],mux[0],sel[2],out);
endmodule
 
// 16 bit mux 8 to 1
module bit16_mux_8to1(out,x0,x1,x2,x3,x4,x5,x6,x7,sel);
  input [15:0] x0,x1,x2,x3,x4,x5,x6,x7;
  input [2:0]sel;
  output [15:0] out;
  generate
      genvar k;
      for(k=0;k<16;k=k+1)
        begin : wer
          mux8to1 m({x7[k],x6[k],x5[k],x4[k],x3[k],x2[k],x1[k],x0[k]},sel,out[k]);
        end
    endgenerate
endmodule
 
 
module dmux1to2(c,a,e);
    output [1:0]c;
    input a,e;
  wire na;
  not nt(na,a);
  and a1(c[0],e,na);
  and a2(c[1],e,a);
endmodule
 
module dmux2to4(c,a,e);
  output [3:0]c;
  input [1:0]a;
  input e;
  wire [1:0]e_t;
 
  dmux1to2 d1(e_t, a[1], e);
  dmux1to2 d2(c[3:2],a[0], e_t[1]);
  dmux1to2 d3(c[1:0],a[0], e_t[0]);
endmodule
 
 
module dmux3to8(c,a,e);
  output [7:0]c;
  input [2:0]a;
  input e;
  wire [1:0]e_t;
  dmux1to2 d1(e_t, a[2], e);
  dmux2to4 d2(c[7:4],a[1:0], e_t[1]);
  dmux2to4 d3(c[3:0],a[1:0], e_t[0]);
endmodule
 
 
module registerbank(regbankmatrix,out,in,isload,loadsel,transsel,rst);
  output [15:0]out;
  input [15:0]in;
  input isload;
    input rst;
  input [2:0]loadsel,transsel;
  wire [7:0]dec_out;
  output [127:0] regbankmatrix;
  wire [15:0]mux_in0,mux_in1,mux_in2,mux_in3,mux_in4,mux_in5,mux_in6,mux_in7;
  assign regbankmatrix[15:0]=mux_in0;
  assign regbankmatrix[31:16]=mux_in1;
  assign regbankmatrix[47:32]=mux_in2;
  assign regbankmatrix[63:48]=mux_in3;
  assign regbankmatrix[79:64]=mux_in4;
  assign regbankmatrix[95:80]=mux_in5;
  assign regbankmatrix[111:96]=mux_in6;
  assign regbankmatrix[127:112]=mux_in7;
  dmux3to8 decoder(dec_out,loadsel,isload);
  register_16 r0(mux_in0,in,1'b1,dec_out[0],rst);
  register_16 r1(mux_in1,in,1'b1,dec_out[1],rst);
  register_16 r2(mux_in2,in,1'b1,dec_out[2],rst);
  register_16 r3(mux_in3,in,1'b1,dec_out[3],rst);
  register_16 r4(mux_in4,in,1'b1,dec_out[4],rst);
  register_16 r5(mux_in5,in,1'b1,dec_out[5],rst);
  register_16 r6(mux_in6,in,1'b1,dec_out[6],rst);
  register_16 r7(mux_in7,in,1'b1,dec_out[7],rst);
 
  bit16_mux_8to1 muxtlk(out,mux_in0,mux_in1,mux_in2,mux_in3,mux_in4,mux_in5,mux_in6,mux_in7,transsel);
 
endmodule
 
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
 
 
module alu(bus_x,bus_y,bus_z,sel,c,v,s,z);
  input[15:0] bus_y;
  input[15:0] bus_x;
  input [2:0] sel;
  output [15:0] bus_z;
  output c,v,s,z;
  wire v_sum,c_sum,v_sub,c_sub,nz,tempa,tempb;
 
//  000=Z=X f_sel
  //010=Z=X+Y
 // 001=Z=X-Y
  // 011=Z=X
  //100 = complement
  // 101 = or 
  // 110 = and 
  // 111 = zero;
 
  wire [15:0] out_y,out_sum,out_sub,o5,o4,o6,cmp1;
  add_16 ad(out_sum, bus_x, bus_y,c_sum,v_sum);
  sub_16 sb(out_sub, bus_x, bus_y,c_sub,v_sub);
 
  generate 
    genvar k;
    for(k=0;k<16;k=k+1)
    begin
      not n(cmp1[k],bus_x[k]);        //2: cmp
      and a(o6[k],bus_x[k],bus_y[k]);   //4: and
      or org(o5[k],bus_x[k],bus_y[k]);  //5 : or
    end
  endgenerate
 
  add_16 cpm2(o4,cmp1,1,tempa,tempb);
 
      mux8to1 m0 ({1'b0,o6[0],o5[0],o4[0],bus_x[0],out_sum[0],out_sub[0],bus_x[0]},sel[2:0],bus_z[0]);
  mux8to1 m1 ({1'b0,o6[1],o5[1],o4[1],bus_x[1],out_sum[1],out_sub[1],bus_x[1]},sel[2:0],bus_z[1]);
  mux8to1 m2 ({1'b0,o6[2],o5[2],o4[2],bus_x[2],out_sum[2],out_sub[2],bus_x[2]},sel[2:0],bus_z[2]);
  mux8to1 m3 ({1'b0,o6[3],o5[3],o4[3],bus_x[3],out_sum[3],out_sub[3],bus_x[3]},sel[2:0],bus_z[3]);
  mux8to1 m4 ({1'b0,o6[4],o5[4],o4[4],bus_x[4],out_sum[4],out_sub[4],bus_x[4]},sel[2:0],bus_z[4]);
  mux8to1 m5 ({1'b0,o6[5],o5[5],o4[5],bus_x[5],out_sum[5],out_sub[5],bus_x[5]},sel[2:0],bus_z[5]);
  mux8to1 m6 ({1'b0,o6[6],o5[6],o4[6],bus_x[6],out_sum[6],out_sub[6],bus_x[6]},sel[2:0],bus_z[6]);
  mux8to1 m7 ({1'b0,o6[7],o5[7],o4[7],bus_x[7],out_sum[7],out_sub[7],bus_x[7]},sel[2:0],bus_z[7]);
  mux8to1 m8 ({1'b0,o6[8],o5[8],o4[8],bus_x[8],out_sum[8],out_sub[8],bus_x[8]},sel[2:0],bus_z[8]);
  mux8to1 m9 ({1'b0,o6[9],o5[9],o4[9],bus_x[9],out_sum[9],out_sub[9],bus_x[9]},sel[2:0],bus_z[9]);
  mux8to1 m10 ({1'b0,o6[10],o5[10],o4[10],bus_x[10],out_sum[10],out_sub[10],bus_x[10]},sel[2:0],bus_z[10]);
  mux8to1 m11 ({1'b0,o6[11],o5[11],o4[11],bus_x[11],out_sum[11],out_sub[11],bus_x[11]},sel[2:0],bus_z[11]);
  mux8to1 m12 ({1'b0,o6[12],o5[12],o4[12],bus_x[12],out_sum[12],out_sub[12],bus_x[12]},sel[2:0],bus_z[12]);
  mux8to1 m13 ({1'b0,o6[13],o5[13],o4[13],bus_x[13],out_sum[13],out_sub[13],bus_x[13]},sel[2:0],bus_z[13]);
  mux8to1 m14 ({1'b0,o6[14],o5[14],o4[14],bus_x[14],out_sum[14],out_sub[14],bus_x[14]},sel[2:0],bus_z[14]);
  mux8to1 m15 ({1'b0,o6[15],o5[15],o4[15],bus_x[15],out_sum[15],out_sub[15],bus_x[15]},sel[2:0],bus_z[15]);
  mux8to1 m16 ({1'b0,1'b0,1'b0,1'b0,1'b0,c_sum,c_sub,1'b0},sel[2:0],c);
  mux8to1 m17 ({1'b0,1'b0,1'b0,1'b0,1'b0,v_sum,v_sub,1'b0},sel[2:0],v);
 
 
  and a1(s,1,bus_z[15]);
  or o1(nz,bus_z[0],bus_z[1],bus_z[2],bus_z[3],bus_z[4],bus_z[5],bus_z[6],bus_z[7],bus_z[8],bus_z[9],bus_z[10],bus_z[11],bus_z[12],bus_z[13],bus_z[14],bus_z[15]);
  not n1(z,nz);
endmodule
 
//a-b
module sub_16(sub, a, b,c,v);
  input [15:0] a,b;
  output [15:0] sub;
  output c,v;
  wire [15:0] nb;         //stores not of x
  wire [15:0] notb_inc;
  wire tempa,tempb;
 
  //loop to get 1s complement of x
 
      not n0(nb[0],b[0]);
      not n1(nb[1],b[1]);
      not n2(nb[2],b[2]);
      not n3(nb[3],b[3]);
      not n4(nb[4],b[4]);
      not n5(nb[5],b[5]);
      not n6(nb[6],b[6]);
      not n7(nb[7],b[7]);
      not n8(nb[8],b[8]);
  not n9(nb[9],b[9]);
  not n10(nb[10],b[10]);
  not n11(nb[11],b[11]);
  not n12(nb[12],b[12]);
  not n13(nb[13],b[13]);
  not n14(nb[14],b[14]);
  not n15(nb[15],b[15]);
 
 
//adding 1 to 1s complement of x to get 2s complement
  add_16 a1(notb_inc,nb,1,tempa,tempb);
  add_16 a2(sub,notb_inc,a,c,v);
endmodule
 
 
module add_16(sum,a,b,c,v);
  input [15:0] a,b;
  output [15:0] sum;
  output c,v;
  wire [16:0]carry;
  wire sg,nsg;
 
  assign carry[0] = 0;
 
 
      fa f0(sum[0],carry[1],a[0],b[0],carry[0]);
      fa f1(sum[1],carry[2],a[1],b[1],carry[1]);
      fa f2(sum[2],carry[3],a[2],b[2],carry[2]);
      fa f3(sum[3],carry[4],a[3],b[3],carry[3]);
      fa f4(sum[4],carry[5],a[4],b[4],carry[4]);
      fa f5(sum[5],carry[6],a[5],b[5],carry[5]);
      fa f6(sum[6],carry[7],a[6],b[6],carry[6]);
      fa f7(sum[7],carry[8],a[7],b[7],carry[7]);
      fa f8(sum[8],carry[9],a[8],b[8],carry[8]);
      fa f9(sum[9],carry[10],a[9],b[9],carry[9]);
  fa f10(sum[10],carry[11],a[10],b[10],carry[10]);
  fa f11(sum[11],carry[12],a[11],b[11],carry[11]);
  fa f12(sum[12],carry[13],a[12],b[12],carry[12]);
  fa f13(sum[13],carry[14],a[13],b[13],carry[13]);
  fa f14(sum[14],carry[15],a[14],b[14],carry[14]);
  fa f15(sum[15],carry[16],a[15],b[15],carry[15]);
  xor a1(v,carry[15],carry[16]);
  or o1(c,0,carry[16]);
 
 
endmodule
 
module fa(s,co,a,b,ci);
    output s,co;
    input a,b,ci;
    wire n1,n2,n3;
 
    xor u1(s,a,b,ci);
    and u2(n1,a,b);
    and u3(n2,b,ci);
    and u4(n3,a,ci);
    or u5(co,n1,n2,n3);
 
endmodule
 
// Code your design here
module tristate_buffer_16(input_x, enable, output_x);
  input [15:0]input_x;
  input enable;
  output [15:0]output_x;
 
  generate
    genvar i;
    for(i=0;i<16;i=i+1)
      begin
        bufif1 b1(output_x[i], input_x[i], enable);
      end
  endgenerate
 
endmodule
 
 
module datapath(regbankmatrix,IRout,PCout,buffout,d,loadsel,transsel,fsel,isload,ldbf,ldPC,ldMAR,ldMDR,ldTemp,ldIR,ldcond,rst,T2,TT,TMAR,TPC_A,TPC_M,TMDRX,TMDRI,RMDRX,RMDRI,Treg,MemData,MemAddr);
  input [2:0]loadsel,transsel;
  input [2:0]fsel;
  inout [15:0]MemData;
  input isload,ldbf,ldPC,ldMAR,ldMDR,ldTemp,ldIR,ldcond,rst;
  input T2,TT,TMAR,TPC_A,TPC_M,TMDRX,TMDRI,RMDRX,RMDRI,Treg;
  output d;
  output [15:0]IRout,MemAddr,PCout,buffout;
  wire c,v,s,z;
  wire [15:0]regbankout,busX,busZ,buffin,MARout,Tempout,MDRin,MDRout;
  output [127:0] regbankmatrix;
    alu asdfklj(busX,buffout,busZ,fsel,c,v,s,z);
  //regbank
  registerbank regbank(regbankmatrix,regbankout,busZ,isload,loadsel,transsel,rst);
  tristate_buffer_16 mjh(regbankout,Treg,busX);
 //conditions
  wire [3:0]cond;
  register_4 conditions(cond,{v,s,c,z},1,ldcond,rst);
  mux4to1 condition(cond,IRout[13:12],d);
 
  //IR
  register_16 IR(IRout,MemData,1,ldIR,rst);
 
  //MDR
  register_16 MDR(MDRout,MDRin,1,ldMDR,rst);
  tristate_buffer_16 m1(MemData, RMDRX, MDRin);
  tristate_buffer_16 m2(busZ, RMDRI, MDRin);
  tristate_buffer_16 m3(MDRout, TMDRX, MemData);
  tristate_buffer_16 m4(MDRout, TMDRI, busX);
 
  //buff
  register_16 buff(buffout,busX,1,ldbf,rst);
 
  //PC  
  register_16 PC(PCout,busZ,1,ldPC,rst);
  tristate_buffer_16 m5kj(PCout, TPC_A, busX);
  tristate_buffer_16 m5bh(PCout, TPC_M, MemAddr)
;  
  // MAR
  register_16 MAR(MARout,busZ,1,ldMAR,rst);
  tristate_buffer_16 m5(MARout, TMAR, MemAddr);
 
  //Temp
  register_16 Temp(Tempout,busZ,1,ldTemp,rst);
  tristate_buffer_16 m4sdn(Tempout, TT, busX);
 
  //#2
  tristate_buffer_16 m4kn(1, T2, busX);
 
endmodule

module Controller(ldcond,loadsel,isload,ldbf,ldPC,ldMAR,ldMDR,ldTemp,ldIR,T2,TT,TMAR,TPC_A,TPC_M,TMDRX,TMDRI,RMDRX,RMDRI,Treg,transel,fsel,d,IRout,rst,clk,memrw,Current_State);
  output reg [2:0]loadsel,transel;
  output reg [2:0]fsel;
  output ldcond,isload,ldbf,ldPC,ldMAR,ldMDR,ldTemp,ldIR;
  output reg T2,TT,TMAR,TPC_A,TPC_M,TMDRX,TMDRI,RMDRX,RMDRI,Treg,memrw;
  input d,rst,clk;
  input [15:0]IRout;
  output reg [4:0] Current_State;
  reg [4:0]next_state;
  wire [2:0] rdst,ra,rb,rx;
  assign rdst = IRout[10:8];
  assign ra = IRout[10:8];
  assign rb = IRout[5:3];
  assign rx = IRout[2:0];
	// transfer of rdst, rb, rx, ra, bf
	// load of rdst
	// read and write of memory 
	reg intldcond,intisload,intldbf,intldPC,intldMAR,intldMDR,intldTemp,intldIR;
  assign ldcond = (~clk)&&(intldcond);
  assign isload = (~clk)&&(intisload);
  assign ldbf = (~clk)&&(intldbf);
  assign ldPC = (~clk)&&(intldPC);
  assign ldMAR = (~clk)&&(intldMAR);
  assign ldMDR = (~clk)&&(intldMDR);
  assign ldTemp = (~clk)&&(intldTemp);
  assign ldIR = (~clk)&&(intldIR);
  
  
parameter SIZE = 5;
parameter IDLE = 5'b00000, 	state1 = 5'b00001,	state2 = 5'b00010,	state3 = 5'b00011,	state4 = 5'b00100,	state5 = 5'b00101 ,			  state6 = 5'b00110,	state7 = 5'b00111,	state8 = 5'b01000,	state9 = 5'b01001,	state10 = 5'b01010,	state11 = 5'b01011,	state12 = 5'b01100,	state13 = 5'b01101,	state14 = 5'b01110,	state15 = 5'b01111,	state16 = 5'b10000,	state17 = 5'b10001,	state18 = 5'b10010,	state19 = 5'b10011,	state20 = 5'b10100,	state21 = 5'b10101,	state22 = 5'b10110,	state23 = 5'b10111,	state24 = 5'b11000,	state25 = 5'b11001,	state26 = 5'b11010,	state27 = 5'b11011,	state28 = 5'b11100,	state29 = 5'b11101,	state30 = 5'b11110,	state31 = 5'b11111;

  
  //state logic
always@(Current_State)
begin
	case(Current_State)
      IDLE:begin 
		intldcond=0;
		intisload=0;
		intldbf=0;
		intldPC=1;
		intldMAR=0;
		intldMDR=0;
		intldTemp=0;
		intldIR=0;
		T2=0;
		TT=0;
		TMAR=0;
		TPC_A=0;
		TPC_M=0;
		TMDRX=0;
		TMDRI=0;
		RMDRX=0;
		RMDRI=0;
		Treg=0;
		memrw=0;
		fsel=7;
		end
	state1:	begin // IR <- M[PC] 
		intldcond=0;
		intisload=0;
		intldbf=0;
		intldPC=0;
		intldMAR=0;
		intldMDR=0;
		intldTemp=0;
		T2=0;
		TT=0;
		TMAR=0;
		TPC_A=0;
		TMDRX=0;
		TMDRI=0;
		RMDRX=0;
		RMDRI=0;
		Treg=0;
		intldIR=1;
		TPC_M=1;
		memrw=0;
		fsel=0;
		end
	state2:	begin //buffer <- #2
		intldIR=0;
		TPC_M=0;
		intldbf=1;
		T2=1;
		fsel=0;
		end
	state3: begin //PC <- PC + buffer
		intldbf=0;
		T2=0;
		TPC_A=1;
		intldPC=1;
		fsel=2;
		end
	state4: begin //
		TPC_A=0;
		intldPC=0;
      intldcond=0;
		fsel=0;
		end
      state5: begin // MDR <- M[PC]
		TPC_A=0;
		intldPC=0;
		intisload=0;
		TPC_M=1;
		intldMDR=1;
		RMDRX=1;
		memrw=0;
		fsel=0;
		end
	state6:	begin	//buffer <- MDR
		TPC_M=0;
		intldMDR=0;
		RMDRX=0;
		intldbf=1;
		TMDRI=1;
		fsel=0;
		end
	state7:	begin	//PC <- PC + buffer 
		intldbf=0;
		TMDRI=0;
		TPC_A=1;
		intldPC=1;
		fsel=2;
		end
	state8:	begin //buffer <- #2
		TPC_A=0;
		intldPC=0;
		intldbf=1;
		T2=1;
		fsel=0;
		end
	state9:	begin	//ra <- PC + buffer
		intldbf=0;
		T2=0;
		TPC_A=1;
		intisload=1;
		loadsel=ra;
		fsel=2;
		end
	state10:begin	//PC <- ra
		TPC_A=0;
		intldPC=1;
		Treg=1;
		transel=ra;
		fsel=0;
		end
	state11:begin	//rdst <- rb
		TPC_A=0;
		intldPC=0;
		loadsel=rdst;
		intisload=1;
		transel=rb;
		Treg=1;
		fsel=0;
		end
	state12:begin	//rdst <- rdst'
		TPC_A=0;
		intldPC=0;
		intisload=1;
		loadsel=rdst;
		transel=rdst;
		Treg=1;
		intldcond=1;
		fsel=4;
		end
      state13:begin 	//MDR <- M[PC]
		TPC_A=0;
		intldPC=0;
		intldMDR=1;
		TPC_M=1;
		RMDRX=1;
		memrw=0;
		fsel=0;
		end
	state14:begin 		//buffer <- MDR
		intldMDR=0;
		TPC_M=0;
		RMDRX=0;
		intldbf=1;
		TMDRI=1;
		fsel=0;
		end
	state15:begin	// T <- rb + buffer
		intldbf=0;
		TMDRI=0;
		intldTemp=1;
		transel=rb;
		Treg=1;
		fsel=2;
		end
	state16:begin // buffer <- rx
		intldTemp=0;
		intldbf=1;
		transel=rx;
		Treg=1;
		fsel=0;
		end
	state17:begin // MAR <- T + buffer
		intldbf=0;
		Treg=0;
		intldMAR=1;
		TT=1;
		fsel=2;
		end
      state18:begin 	//MDR <- M[MAR]
		intldMAR=0;
		TT=0;
		RMDRX=1;
		TMAR=1;
		memrw=0;
		fsel=0;
		end
	state19:begin	//MAR <- MDR
		RMDRX=0;
		TMAR=0;
		TMDRI=1;
		intldMAR=1;
		fsel=0;
		end		
      state20:begin   //MDR <- M[MAR]
		intldMAR=0;
		TT=0;
		TMDRI=0;
		intldMAR=0;
		intldMDR=1;
		TMAR=1;
		RMDRX=1;
		memrw=0;
		fsel=0;
		end
	state21:begin	//rdst <- MDR
		intldMDR=0;
		RMDRX=0;
		TMAR=0;
		TPC_M=0;
		intisload=1;
		loadsel=rdst;
		TMDRI=1;
		fsel=0;
		end
	state22:begin //MDR <- rdst
		intldMAR=0;
		TT=0;
		TMDRI=0;
		RMDRI=1;
		intldMDR=1;
		transel=rdst;
		Treg=1;
		fsel=0;
		end
      state23:begin //M[MAR] <- MDR
		intldMDR=0;
		Treg=0;
		RMDRI=0;
		TMAR=1;
		TMDRX=1;
		memrw=1;
		fsel=0;
		end
	state24:begin 	//buffer <- MDR
		TPC_M=0;	// from 13???
		intldMDR=0;
		RMDRX=0;
		TMAR=0;
		intldbf=1;
		TMDRI=1;
		fsel=0;
		end
	state25:begin //rdst <- rdst + buffer
		intldbf=0;
		TMDRI=0;
		intisload=1;
		loadsel=rdst;
		transel=rdst;
		Treg=1;
		intldcond=1;
      if (instset[4:2]==000) fsel=2;
      else if (instset[4:2]==001) fsel=1;
      else if (instset[4:2]==100) fsel=6;
      else fsel=5;
		end
	state29:begin //T <- rdst - buffer
		intldbf=0;
		TMDRI=0;
		intldTemp=1;
		intldcond=1;
		transel=rdst;
		Treg=1;
		fsel=1;
		end
	state30:begin //buffer <- #2
		memrw=0;
		intisload=0;
		TMDRI=0;
		TMAR=0;
		TMDRX=0;
		Treg=0;
		intldTemp=0;
		intldcond=0;
		intldbf=1;
		T2=1;
		fsel=0;
		end
	state31:begin //PC <- PC + buffer
		intldbf=0;
		T2=0;
		TPC_A=1;
		intldPC=1;
		fsel=2;
		end
     state26:begin  //buffer <- rb
		TPC_A=0;
		intldPC=0;
       	intldbf=1;
       	transel=rb;
       	Treg=1;
     end
      state27: begin  //rdst <- rdst (op) buffer
        Treg=0;
        intldbf=0;
        intldcond=1;
        loadsel = rdst;
        intisload=1;
        if (instset[4:2]==000) fsel=2;
        else if (instset[4:2]==001) fsel=1;
        else if (instset[4:2]==100) fsel=6;
        else fsel=5;
	end
        state28: begin // T<- rdst - buffer
        Treg=0;
        intldbf=0;
        intldTemp=1;
		intldcond=1;
		transel=rdst;
		Treg=1;
		fsel=1;
        end
     endcase
end
  
  wire [6:0]instset;
  assign instset = {IRout[15:11],IRout[7:6]};
  
  //next state logic
  always@(negedge clk)
begin
  case(Current_State)
	IDLE: 	begin
		
if(instset[6:2]==5'b11111)next_state = IDLE;
      else next_state = state1;
		end
	state1:	begin 
	
      if(instset[6:2]==5'b11111) next_state = IDLE;
	else next_state = state2;
		end
	state2:	begin
      if(instset[6:2]==5'b11111) next_state = IDLE;
		else next_state = state3;
		end
	state3: begin
      if(instset[6:5]==2'b10)next_state = state4;      
      else if(instset[6:2]==5'b11100)next_state = state5;
      else if(instset[6:2]==5'b11110)next_state = state8;
      else if(instset[6:2]==5'b11010)next_state = state10;
      else if(instset==7'b0000001)next_state = state11;
      else if(instset[6:2]==5'b01011)next_state = state12;
      else if(instset[6:5]==2'b01 && instset[1:0]==2'b01) next_state = state26;
		else next_state = state13;
		end
	state4: begin
      if(d==1'b0)next_state = state5;
		else next_state = state30;
		end
	state5: begin
		next_state = state6;
		end
	state6:	begin
		next_state = state7;
		end
	state7:	begin
		next_state = state1;
		end
	state8:	begin
		next_state = state9;
		end
	state9:	begin
		next_state = state5;
		end
	state10:begin
		next_state = state1;
		end
	state11:begin
		next_state = state1;
		end
	state12:begin
		next_state = state1;
		end
	state13:begin
      if(instset==7'b0000000)next_state = state21;
      else if(instset[1:0]==2'b00)next_state = state24;
      else next_state = state14;
		end
	state14:begin
		next_state = state15;
		end
	state15:begin
		next_state = state16;
		end
	state16:begin
		next_state = state17;
		end
	state17:begin
      if(instset[1:0]==2'b11)next_state = state18;
      else if(instset[6:5]==0 && instset[2]==1)next_state = state22;
		else next_state = state20;
		end
	state18:begin
		next_state = state19;
		end
	state19:begin
      if(instset[6:5]==0 && instset[2]==1)next_state = state22;
		else next_state = state20;
		end
	state20:begin
      if(instset[6:5]==2'b00)next_state = state21;
      else if(instset[6:5]==2'b01)next_state = state24;
		end
	state21:begin
		next_state = state30;
		end
	state22:begin
		next_state = state23;
		end
	state23:begin
		next_state = state30;
		end
	state24:begin
      if(instset[4:2]==3'b010)next_state = state29;
	  else next_state = state25;
      end
	state25:begin
		next_state = state30;
		end
	state26:begin
      if(instset[4:2]==3'b010) next_state=state28;
		else next_state = state27;
		end
	state27:begin
		next_state = state1;
		end
	state28:begin
		next_state = state1;
		end
	state29:begin
		next_state = state30;
		end
	state30:begin
		next_state = state31;
		end
	state31:begin
		next_state = state1;
		end
	endcase
end
  
  //state transition
  always@(posedge clk)
    begin
      if(rst==0)
      	Current_State = next_state;
      else
        Current_State = IDLE;
    end
  
endmodule

