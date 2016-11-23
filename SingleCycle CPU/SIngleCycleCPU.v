
// Code your design here
module CPU(muxsel,out1,bus_y,bus_z,d,data_IR,regbankmatrix,PCout,IR,clk,rst);
  output [127:0] regbankmatrix;
  input clk,rst;
  output [31:0] data_IR;
  output [15:0] PCout;
  wire memrw;
  output d;
  wire [2:0]fsel;
  output [2:0]muxsel;
  wire isload,iscond;
  output [6:0]IR;
  output [15:0] out1,bus_z,bus_y;

  controller SC(IR,fsel,muxsel,iscond,isload,memrw,rst,clk,d);
  datapath SCD(out1,bus_y,bus_z,data_IR,regbankmatrix,PCout,IR,d,fsel,muxsel,iscond,clk,isload,memrw,rst);
  
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
  // 011=Z=Y
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
 
  mux8to1 m0 ({1'b0,o6[0],o5[0],o4[0],bus_y[0],out_sum[0],out_sub[0],bus_x[0]},sel[2:0],bus_z[0]);
  mux8to1 m1 ({1'b0,o6[1],o5[1],o4[1],bus_y[1],out_sum[1],out_sub[1],bus_x[1]},sel[2:0],bus_z[1]);
  mux8to1 m2 ({1'b0,o6[2],o5[2],o4[2],bus_y[2],out_sum[2],out_sub[2],bus_x[2]},sel[2:0],bus_z[2]);
  mux8to1 m3 ({1'b0,o6[3],o5[3],o4[3],bus_y[3],out_sum[3],out_sub[3],bus_x[3]},sel[2:0],bus_z[3]);
  mux8to1 m4 ({1'b0,o6[4],o5[4],o4[4],bus_y[4],out_sum[4],out_sub[4],bus_x[4]},sel[2:0],bus_z[4]);
  mux8to1 m5 ({1'b0,o6[5],o5[5],o4[5],bus_y[5],out_sum[5],out_sub[5],bus_x[5]},sel[2:0],bus_z[5]);
  mux8to1 m6 ({1'b0,o6[6],o5[6],o4[6],bus_y[6],out_sum[6],out_sub[6],bus_x[6]},sel[2:0],bus_z[6]);
  mux8to1 m7 ({1'b0,o6[7],o5[7],o4[7],bus_y[7],out_sum[7],out_sub[7],bus_x[7]},sel[2:0],bus_z[7]);
  mux8to1 m8 ({1'b0,o6[8],o5[8],o4[8],bus_y[8],out_sum[8],out_sub[8],bus_x[8]},sel[2:0],bus_z[8]);
  mux8to1 m9 ({1'b0,o6[9],o5[9],o4[9],bus_y[9],out_sum[9],out_sub[9],bus_x[9]},sel[2:0],bus_z[9]);
  mux8to1 m10 ({1'b0,o6[10],o5[10],o4[10],bus_y[10],out_sum[10],out_sub[10],bus_x[10]},sel[2:0],bus_z[10]);
  mux8to1 m11 ({1'b0,o6[11],o5[11],o4[11],bus_y[11],out_sum[11],out_sub[11],bus_x[11]},sel[2:0],bus_z[11]);
  mux8to1 m12 ({1'b0,o6[12],o5[12],o4[12],bus_y[12],out_sum[12],out_sub[12],bus_x[12]},sel[2:0],bus_z[12]);
  mux8to1 m13 ({1'b0,o6[13],o5[13],o4[13],bus_y[13],out_sum[13],out_sub[13],bus_x[13]},sel[2:0],bus_z[13]);
  mux8to1 m14 ({1'b0,o6[14],o5[14],o4[14],bus_y[14],out_sum[14],out_sub[14],bus_x[14]},sel[2:0],bus_z[14]);
  mux8to1 m15 ({1'b0,o6[15],o5[15],o4[15],bus_y[15],out_sum[15],out_sub[15],bus_x[15]},sel[2:0],bus_z[15]);
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
 


module register_16(out,in,load,clk,rst);
  output reg [15:0] out;
  input load,clk,rst;
  input [15:0] in;
always@(posedge clk or posedge rst)
begin
if(rst) begin 
	out = 0;
end
else 
	out =in;
  end
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

module registerbank(regbankmatrix,out1,out2,in,isload,loadsel,transsel1,transsel2,rst);
  output [15:0]out1,out2;
  input [15:0]in;
  input isload;
    input rst;
  input [2:0]loadsel,transsel1,transsel2;
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
 
  bit16_mux_8to1 muxtlk(out1,mux_in0,mux_in1,mux_in2,mux_in3,mux_in4,mux_in5,mux_in6,mux_in7,transsel1);
  bit16_mux_8to1 muxtlks(out2,mux_in0,mux_in1,mux_in2,mux_in3,mux_in4,mux_in5,mux_in6,mux_in7,transsel2);
 
endmodule

// Code your design here

module memory2 (
address     , // Address Input
data_in        , // Data directional
data_out,
cs          , // Chip Select
we          , // Write Enable/Read Enable
rst 
);          
parameter RAM_DEPTH = 1 << 16;
//--------------Input Ports----------------------- 
  input [15:0] address ;
input                                     cs           ;
input                                     we,rst          ;
  input [15:0]  data_in       ;

//--------------Internal variables---------------- 
  output reg [15:0]   data_out ;
  reg [15:0] mem [0:RAM_DEPTH-1];

//--------------Code Starts Here------------------ 

// Tri-State Buffer control 
// output : When we = 0, oe = 1, cs = 1

  

// Memory Write Block 
// Write Operation : When we = 1, cs = 1
  always @ (address or data_in or cs or we)
begin : MEM_WRITE
   if ( cs && we ) begin
       mem[address] = data_in;
   end
end

// Memory Read Block 
// Read Operation : When we = 0, cs = 1
always @ (address or cs)
begin : MEM_READ
    if (cs)  begin
         data_out = mem[address];
    end
end

endmodule // End of Module ram_sp_ar_aw


// Code your design here

module memory1 (
address     , // Address Input
data_out	,
cs          , // Chip Select
rst 
);          
parameter RAM_DEPTH = 1 << 16;
//--------------Input Ports----------------------- 
  input [15:0] address ;
input                                     cs           ;
input                                     rst          ;

//--------------Internal variables---------------- 
  output reg [31:0]   data_out ;
  reg [15:0] mem [0:RAM_DEPTH-1];

//--------------Code Starts Here------------------ 

// Tri-State Buffer control 
// output : When we = 0, oe = 1, cs = 1
always @ (posedge rst)
  begin
    mem[0] = 16'b0000000000000000;// li r0,0
    mem[1] = 16'b0000000000000011;//
    mem[2] = 16'b0000000000000000;// li r0,1
    mem[3] = 16'b0000000000000001;//
    mem[4] = 16'b0100000000000000;//addi r0,r0,2
    mem[5] = 16'b0000000000000010;//
    mem[6] = 16'b0000000101000000;//lr r1,r0
    mem[7] = 16'b0000000000000001;//
    mem[8] = 16'b0000001100000000;//li r3,5
    mem[9] = 16'b0000000000000101;//
    mem[10] = 16'b0000001001011000;//lr r2,r3
    mem[11] = 16'b0000000000000001;//
    mem[12] = 16'b0100101000010000;//subi r2,r2,1
    mem[13] = 16'b0000000000000001;//
    mem[14] = 16'b0101000001000001;//mnsr r0,r1
    mem[15] = 16'b0000000000000001;//
    mem[16] = 16'b1000000000000000;//jz ... (4){as PC will be 14 while reading -4}
    mem[17] = 16'b0000000000000100;//
    mem[18] = 16'b1111111111111111;//exit
    mem[19] = 16'b1111111111111111;//exit
    mem[20] = 16'b0000011100000000;//li r7,7
    mem[21] = 16'b0000000000000111;//
    mem[22] = 16'b0000011001111000;//lr r6,r7
    mem[23] = 16'b0000000000000001;//
  end
  
  
  

// Memory Read Block 
// Read Operation : When we = 0, cs = 1
always @ (address or cs)
begin : MEM_READ
    if (cs)  begin
         data_out = {mem[address],mem[address+1]};
    end
end

endmodule // End of Module ram_sp_ar_aw

module bit16_mux_2to1(in1,in0,sel,out);
  input [15:0] in1,in0;
  input sel;
  output [15:0] out;
  generate
      genvar k;
      for(k=0;k<16;k=k+1)
        begin : wer
          mux2to1 m(in1[k],in0[k],sel,out[k]);
        end
    endgenerate
endmodule


module datapath(out1,bus_y,bus_z,data_IR,regbankmatrix,PCout,IR,d,fsel,muxsel,iscond,clk,isload,memrw,rst);
	output [6:0] IR;
	output d;
	input [2:0]fsel;
	input [2:0]muxsel;
	input iscond,clk,isload,memrw,rst;
assign IR = {data_IR[31:27],data_IR[23:22]};
	
	
	//PC
wire [15:0]PCin;
  output [15:0]PCout;
  wire [15:0]testin,testout;
  register_16 PC(PCout,PCin,1,clk,rst);

	//Memory 1
output [31:0]data_IR;
memory1 m1(PCout,data_IR,1,rst);

  //Register Bank
wire [15:0]out2,in;
  output [15:0]out1;
  output [127:0]regbankmatrix;
  registerbank regbank(regbankmatrix,out1,out2,in,isload,data_IR[26:24],data_IR[21:19],data_IR[18:16],rst);

// mux alu
output [15:0]bus_y;
bit16_mux_2to1 befaltu(out2,data_IR[15:0],muxsel[1],bus_y);

//alu
wire c,v,s,z;
output [15:0]bus_z;
alu alum(out1,bus_y,bus_z,fsel,c,v,s,z);

// Memory 2
wire [15:0]mem_out;
memory2 afalu(bus_z,out2,mem_out,1,memrw,rst);   

//mux memory
bit16_mux_2to1 befaltu2(bus_z,mem_out,muxsel[0],in);

//conditions
  wire [3:0]cond;
  wire tempd;
  register_4 conditions(cond,{v,s,c,z},1,iscond,rst);
  mux4to1 condition(cond,IR[4:3],tempd);
  xor (d,tempd,IR[2]);  

//mux pc
wire [15:0]adderin;
bit16_mux_2to1 befaltu3(2,data_IR[15:0],muxsel[2],adderin);

//fa
wire w1,w2;
add_16 addr(PCin,PCout,adderin,w1,w2);

endmodule



module controller(IR,fsel,muxsel,iscond,isload,memrw,rst,clk,d);
	input [6:0] IR;
	input d,clk,rst;
	output reg [2:0]fsel;
	output reg [2:0]muxsel;
	output iscond,isload;
  output reg memrw;
 reg intisload,intiscond;
  assign isload = (~clk)&&intisload;
  assign iscond = (~clk)&&intiscond;
  wire nd;
  not sa(nd,d);
  always @(IR)
    begin
      
      // isload
      if(IR[6]==0 && IR[6:2]!=1 && IR[6:2]!=10)
        intisload=1;
      else intisload=0;
      
      // iscond
      if(IR[6:5]==2'b01)
        intiscond=1;
      else intiscond=0;
      
     /* if(IR[6:5]==2'b11)
        muxsel[2]=0;
      else if(IR[6:5]==2'b10)
        muxsel[2]=nd;
      else 
        muxsel[2]=1;*/
 
      
      // muxsel[2]
      if(IR[6:5]==2'b11) begin
        muxsel[2]=0;
      end
      else if(IR[6:5]==2'b10) begin
        muxsel[2]=nd;
      end
      else if(IR[6:5]==2'b01 || IR[6:5]==2'b00) begin
        muxsel[2]=1;
      end
      
      // muxsel[1]
      if(IR[1:0]==0 || IR[1:0]==2 )
        muxsel[1]=0;
      else muxsel[1]=1;
      
      // muxsel[0]
      if(IR[1:0]==2)
        muxsel[0]=0;
      else muxsel[0]=1;
      
      // memrw
      if(IR[6:2]==1)
        memrw=1;
      else memrw=0;
      
      // fsel
      if(IR[6:2]==8 || IR[1:0]==2 )
      	fsel=2;
      else if(IR==1)
        fsel=0;
      else if(IR==0)
        fsel=3;
      else if(IR[6:2]==5'b01101)
        fsel=4;
      else if(IR[6:2]==5'b01100)
        fsel=5;
      else if(IR[6:2]==5'b01011)
        fsel=6;
      else if(IR[6:2]==5'b01001 || IR[6:2]==5'b01010)
        fsel=1;
      else 
        fsel=7;
      
  end
endmodule
