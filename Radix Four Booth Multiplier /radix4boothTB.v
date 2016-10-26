
// Code your testbench here
// or browse Examples
module testbench;
  wire [17:0]ans;
  reg [7:0] mplier,mcand;
  reg clk, rst, go;
  wire over;
  wire [1:0] state;
  multiplier m(state,ans,over,mplier,mcand,clk,rst,go);
  initial begin
    mplier = 120;
    mcand = 3;
    go=0;
    rst=1;
    #5
    
    rst=0;
    go=1;
    
    #60
    $display("%d * %d = %d",mplier,mcand,ans[16:1]);
    #1
   // go=0;
    #10
    $display("Go=0 %d * %d = %d",mplier,mcand,ans[16:1]);
  end
  integer t;
  initial begin
    clk=0;
    t=0;
    while(t<80) begin
      #1
     	// $display("state = %d,	over = %d",state,over);
      clk=~clk;
      t=t+1;
    end
  end
 
endmodule
