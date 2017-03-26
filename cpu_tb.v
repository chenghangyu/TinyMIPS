`timescale 1ns/10ps
module cpu_tb();

reg	clk;
reg	reset;
wire     [7:0]memdata;	
wire 	memread,memwrite;
wire 	 [7:0] writedata,adr;
reg     groundconstant;

cpu c1(clk,reset,memdata,memread,memwrite,adr,writedata,groundconstant);
ram r1(memdata, memwrite, adr, writedata, clk);


initial begin

#0  reset=1;
#110 reset=0;
groundconstant=0;
//  reset=0;


end

initial 
clk=1;
always #50 clk=~clk;

initial begin #3000 $finish; end
// Open a db file for saving simulation data
initial 
begin
$shm_open ("cpu_tb.db");
// Collect all signals (hierarchically) from the module
$shm_probe (cpu_tb,"AS");
$shm_save;
end
endmodule
