module testbench();
  wire [127:0] regbankmatrix;
  reg clk,rst;
  wire [15:0] IRout,PCout,buffout;
  wire [4:0] Current_State;
  CPU test1(regbankmatrix,IRout,PCout,buffout,Current_State,clk,rst);
  
  initial begin
    rst=0;
    clk=0;
    #10
    rst=1;
    #10
    clk=1;
    #10
    rst=0;
  end
  integer t=0;
  initial begin
    #60
    while(1) begin
      #10
      clk=~clk;
      if(clk==1) begin
        if(Current_State==0 && t<2)
          begin
            #10000000000
            $display("r0=%d",regbankmatrix[15:0]);
     		 $display("r1=%d",regbankmatrix[31:16]);
      $display("r2=%d",regbankmatrix[47:32]);
      $display("r3=%d",regbankmatrix[63:48]);
      $display("r4=%d",regbankmatrix[79:64]);
      $display("r5=%d",regbankmatrix[95:80]);
      $display("r6=%d",regbankmatrix[111:96]);
      $display("r7=%d",regbankmatrix[127:112]);
            t=t+1;
          end

       
      end
    end
  end
  
  initial begin
    #100000
    if(Current_State==0) begin
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
  
endmodule
