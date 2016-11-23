// Code your testbench here
// or browse Examples
module testbench();
  wire [127:0] regbankmatrix;
  reg clk,rst;
  wire [15:0] PCout;
  wire [6:0] IR;
  wire [31:0] data_IR;
  wire [15:0] out,in,bus_z,out1,bus_y;
  wire d;
  wire [2:0] muxsel;
  CPU test1(muxsel,out1,bus_y,bus_z,d,data_IR,regbankmatrix,PCout,IR,clk,rst);
  register_16 asd(out,in,1,clk,rst);
  initial begin
    rst=0;
    #10
   	rst=1;
    #10
    rst=0;
   
  end
  integer t=0;
  initial begin
    #600
    clk=1;
    #100
    while(t<3) begin
     // $display("PCasldf=%d IR=%b %b d=%b z=%d y=%d x=%d muxsel=%b  clk=%b %b",PCout,data_IR[31:16], data_IR[15:0],d,bus_z,bus_y,out1,muxsel,clk,IR);
      #100
      if(PCout<24)begin
        clk=~clk;
        #100
        begin
          $display("PC=%d %b %b %b %d %d %d %b clk=%b %b",PCout,data_IR[31:16], data_IR[15:0],d,bus_z,bus_y,out1,muxsel,clk,IR);
          if(1)
            begin
              #100
              $display("r0=%d",regbankmatrix[15:0]);
              $display("r1=%d",regbankmatrix[31:16]);
              $display("r2=%d",regbankmatrix[47:32]);
              $display("r3=%d",regbankmatrix[63:48]);
              $display("r4=%d",regbankmatrix[79:64]);
              $display("r5=%d",regbankmatrix[95:80]);
              $display("r6=%d",regbankmatrix[111:96]);
              $display("r7=%d",regbankmatrix[127:112]);
              
            end
          
        end
      end
    end
  end

  
endmodule

