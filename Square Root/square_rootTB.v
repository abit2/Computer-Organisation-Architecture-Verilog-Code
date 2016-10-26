// Code your testbench here
// or browse Examples
// Code your testbench here
// or browse Examples
module test_bench;

	reg Go;
	reg [7:0] N;
	reg clk;
	reg rst;
//WE ARE KEEPIING THE INputs above
	wire [3:0] ans;
	wire over;


	integer t;
	main_module uut (
		.Go(Go), 
		.N(N),
		.answer(ans), 
		.over(over), 
		.clock(clk), 
		.reset(rst)
	);

	initial begin
		
	N= 48;
    rst=1;
    #10
    rst = 0;
    #2
    #10
    Go = 1;
    #400
      $display("%d", ans);
	

	end
	
	initial begin
	 t = 0;  
    clk = 1;
      #20
      while(t<2500)
      begin
        #2
        clk = ~clk;
		  t = t+1;
      end
      
  end
      
endmodule
