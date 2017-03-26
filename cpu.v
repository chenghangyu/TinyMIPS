`timescale 1ns/10ps
module cpu(clk,reset,memdata,memread,memwrite,adr,writedata,groundconstant);
input		clk;
input		reset;
input           [7:0] memdata;	
input           groundconstant;

output		memread    ;	
output		memwrite   ;
output     	[7:0] adr;
output 		[7:0] writedata;

wire		memread;    
wire		memwrite  ;
wire    	[7:0] adr;
wire		[7:0] writedata;
wire 		zero,pcen,branch,memtoreg,regwrite,regdst,pcwrite,alusrca,iord;
wire 		[1:0] aluop,alusrcb,pcsource;
wire		[2:0] alucontrol;
wire    	[3:0] irwrite;
wire 		[31:0] instr;





PCcontroller P1(.pcwrite(pcwrite), .branch(branch), .zero(zero), .pcen(pcen));
alucontrol a1( .alucont(alucontrol), .aluop(aluop), .funct(instr[5:0])) ;
controller c1(.alusrca(alusrca), .alusrcb(alusrcb), .aluop(aluop), .branch(branch), .iord(iord), .irwrite(irwrite), 
                  .memread(memread), .memwrite(memwrite), .memtoreg(memtoreg), 
		  .pcwrite(pcwrite), .pcsource(pcsource), .regwrite(regwrite), .regdst(regdst),
                  .op(instr[31:26]), .clk(clk), .reset(reset)) ;
datapath d1(.adr(adr), .instr(instr[31:0]), .writedata(writedata), .zero(zero), 
                 .alucontrol(alucontrol), .alusrca(alusrca), .alusrcb(alusrcb), .iord(iord), .irwrite(irwrite), 
		.memdata(memdata), .memtoreg(memtoreg), .pcen(pcen), .pcsource(pcsource), .regdst(regdst), 
		.regwrite(regwrite), .clk(clk), .reset(reset), .groudconstant(groundconstant)) ;
endmodule


module PCcontroller(pcwrite,branch,zero,pcen);
	output pcen;
	input pcwrite,branch,zero;
	
	and an1(sel_1,branch,zero);
	or or1(pcen,pcwrite,sel_1);

endmodule

module alucontrol( alucont, aluop, funct ) ;

// this module decodes the 'funct' field from the assembly instruction
// it then determines the type of instruction it is: R, I, J
// and creates a 3-bit control line (alucont) for the ALU itself.

// the ALU will rely onlt on the alucont line and not the funct line.


   input  [1:0] aluop   ;
   input  [5:0] funct   ;
   output [2:0] alucont ;

   reg    [2:0] alucont ;

   always @(*)
      case(aluop)
         2'b00: alucont <= 3'b010;  // add for lb/sb
         2'b01: alucont <= 3'b110;  // sub (for beq)
         default: case(funct)       // R-Type instructions
                     6'b100000: alucont <= 3'b010; // add (for add)
                     6'b100010: alucont <= 3'b110; // subtract (for sub)
                     6'b100100: alucont <= 3'b000; // logical and (for and)
                     6'b100101: alucont <= 3'b001; // logical or (for or)
                     6'b101010: alucont <= 3'b111; // set on less (for slt)
                     default:   alucont <= 3'b101; // should never happen
                  endcase
      endcase

endmodule

module controller(alusrca, alusrcb, aluop, branch, iord, irwrite, 
                  memread, memwrite, memtoreg, 
		  pcwrite, pcsource, regwrite, regdst,
                  op, clk, reset) ;

   // INPUTS (3)
   input  [5:0]   op         ;	// OPCODE
   input          clk, reset ;

   // OUTPUTS (12)
   output         alusrca    ;	// ALUSrcA
   output [1:0]   alusrcb    ;	// ALUSrcB
   output [1:0]   aluop      ;	// ALUOp
   output         branch     ;	// Branch
   output         iord       ;	// IorD
   output [3:0]   irwrite    ;	// irwrite [0] = IRWrite0, irwrite[1] = IRWrite1, etc.
   output         memread    ;	// MemRead
   output         memwrite   ;	// MemWrite
   output         memtoreg   ;	// MemtoReg
   output         pcwrite    ;	// PCWrite
   output [1:0]   pcsource   ;	// PCSrc
   output         regwrite   ;	// RegWrite
   output	  regdst     ;	// RegDst

   // REGISTERS
   reg         alusrca    ;	// ALUSrcA
   reg [1:0]   alusrcb    ;	// ALUSrcB
   reg [1:0]   aluop      ;	// ALUOp
   reg         branch     ;	// Branch
   reg         iord       ;	// IorD
   reg [3:0]   irwrite    ;	// irwrite [0] = IRWrite0, irwrite[1] = IRWrite1, etc.
   reg         memread    ;	// MemRead
   reg         memwrite   ;	// MemWrite
   reg         memtoreg   ;	// MemtoReg
   reg         pcwrite    ;	// PCWrite
   reg [1:0]   pcsource   ;	// PCSrc
   reg         regwrite   ;	// RegWrite
   reg	       regdst     ;
   // TODO


   // STATES (12)
   //
   parameter      FETCH1  =  4'b0001;	// state 0
   parameter      FETCH2  =  4'b0010;	// state 1
   parameter      FETCH3  =  4'b0011;	// state 2 
   parameter      FETCH4  =  4'b0100;	// state 3
   parameter      DECODE  =  4'b0101;   // state 4
   parameter      MEMADR  =  4'b0110;	// state 5
   parameter      LBRD    =  4'b0111;	// state 6
   parameter      LBWR    =  4'b1000;	// state 7
   parameter      SBWR    =  4'b1001;	// state 8
   parameter      RTYPEEX =  4'b1010;	// state 9
   parameter      RTYPEWR =  4'b1011;	// state 10
   parameter      BEQEX   =  4'b1100;	// state 11
   parameter      JEX     =  4'b1101;	// state 12

   // OPCODES  -- Make input decoding simpler
   // 
   parameter      LB      =  6'b100000;	// load byte
   parameter      SB      =  6'b101000; // store byte
   parameter      RTYPE   =  6'b000000; // TODO - see lecture 4b 
   parameter      BEQ     =  6'b000100; // TODO
   parameter      J       =  6'b000010; // TODO

   // STATE REGISTERS
   reg [3:0] state, nextstate;

   // state register
   always @(posedge clk )
     if(reset) state <= FETCH1;
	else state<=nextstate; // TODO


// next state logic
   always @(*)
      // TODO
   begin
	case(state)
	FETCH1: nextstate<=FETCH2;
	FETCH2: nextstate<=FETCH3;
	FETCH3: nextstate<=FETCH4;	
	FETCH4: nextstate<=DECODE;
	DECODE:case(op)
			LB:nextstate<=MEMADR;	
   			SB:nextstate<=MEMADR; 
      			RTYPE:nextstate<=RTYPEEX;          
      			BEQ:nextstate<=BEQEX;   
     			J:nextstate<=JEX;
			default:nextstate<=FETCH1;
		endcase
	MEMADR:
		case(op)
	    	LB:nextstate<=LBRD; 
		SB:nextstate<=SBWR; 
		default:nextstate<=FETCH1;
		endcase
	LBRD:nextstate<=LBWR;	
    	LBWR:nextstate<=FETCH1;	
     	SBWR:nextstate<=FETCH1;	
     	RTYPEEX:nextstate<=RTYPEWR;	
    	RTYPEWR:nextstate<=FETCH1;	
    	BEQEX:nextstate<=FETCH1;
     	JEX:nextstate<=FETCH1;
	default:nextstate<=FETCH1;
	endcase
   end

   // registered outputs
   always @(*)
     
 // TODO
   	begin
			begin
			alusrca<=0;
			alusrcb<=2'b00;	
			aluop<=2'b00;	   
			branch<=0;	     
			iord<=0;	
			irwrite<=4'b0000;	
			memread<=0; 
			memwrite<=0;   
			memtoreg<=0;    
			pcwrite<=0;	 
			pcsource<=2'b00;	 
			regwrite<=0;	
			regdst<=0;
			end

	case(state)
     		FETCH1:	        // state 0
			begin			
			alusrcb<=2'b01;	
			irwrite<=4'b0001;	
			memread<=1;     
			pcwrite<=1;	 
			end

     		FETCH2:  	// state 1
			begin
			alusrcb<=2'b01;	
			irwrite<=4'b0010;	
			memread<=1;   
			pcwrite<=1;	 
			end
			
       		FETCH3:  	// state 2 
			begin
			alusrcb<=2'b01;	
			irwrite<=4'b0100;	
			memread<=1;      
			pcwrite<=1;	 
			end
		
     		FETCH4:  	// state 3
			begin
			alusrcb<=2'b01;	
			irwrite<=4'b1000;	
			memread<=1;   
			pcwrite<=1;	 
			end
		
    		DECODE:     	// state 4
			alusrcb<=2'b11;	

        	MEMADR:    	// state 5
			begin
			alusrca<=1;
			alusrcb<=2'b10;	
			end
				
  		LBRD:     	// state 6
			begin	     
			iord<=1;	
			memread<=1; 
			end

   		LBWR:     	// state 7
			begin
			memtoreg<=1;  	 
			regwrite<=1;	
			end
		
      		SBWR:      	// state 8
			begin	     
			iord<=1;	
			memwrite<=1;   
			end

     		RTYPEEX:  	// state 9
			begin			
			alusrca<=1;	
			aluop<=2'b10;	 
			end

      		RTYPEWR:   	// state 10
			begin	 
			regwrite<=1;	
			regdst<=1;
			end

		BEQEX:     	// state 11
			begin
			alusrca<=1;	
			aluop<=2'b01;	   
			branch<=1;	 
			pcsource<=2'b01;	 
			end

     		JEX:       	// state 12  
			begin   
			pcwrite<=1;	 
			pcsource<=2'b10;	 
			end
		endcase
	end
endmodule

module enableffp(D,enable,Q,clk,reset);
	output [7:0] Q;
	input [7:0] D;
	input enable,clk,reset;
	reg [7:0] Q;
	
always @(posedge clk)
if(reset)
 Q<=0;	
else if(enable) 
	Q<=D;
else Q<=Q;
endmodule





module ffp(D,Q,clk,reset);
	output [7:0] Q;
	input [7:0] D;
	input clk,reset;
	reg [7:0] Q;

always @(posedge clk)
if (reset) Q<=0;
	else
	Q<=D;
endmodule


module alu (result, a, b, alucont) ;

   input  [7:0] a, b ;
   input  [2:0] alucont ;
   output [7:0] result ;

   reg    [7:0] result ;
   wire   [7:0] b2, sum, slt ;

   assign b2 = alucont[2] ? ~b:b ; 
   assign sum = a + b2 + alucont[2] ;
   // slt should be 1 if most significant bit of sum is 1
   assign slt = sum[7] ;
	always@(*)
      case(alucont[1:0])
         2'b00: result <= a & b ;
         2'b01: result <= a | b ;
         2'b10: result <= sum ;
         2'b11: result <= slt ;
      endcase
endmodule

module regfile (rd1, rd2, clk, regwrite, ra1, ra2, wa, wd,reset) ;

// This reg file will only have registers $zero, $s0->$s7
// On a READ, takes in addresses on lines: ra1 and ra2
// Spits out data for those registers on lines: rd1 and rd2

// On a WRITE, regwrite must equal 1 (to signal a write)
// Takes in address of register to write to on line: wa
// Takes in data to write to register on line: wd
// Only performs a WRITE on posedge of clk


   input        clk,reset ;
   input        regwrite ;  // signal to command regfile to 'write' to a reg
   input  [2:0] ra1 ;	    // 3-bit address of $s0 --> $s7 (source reg: RS)
   input  [2:0] ra2 ;	    // 3-bit address of $s0 --> $s7 (source reg: RT)
   input  [2:0] wa  ;  	    // 3-bit address of $s0 --> $s7 (destin reg: RD)

   input  [7:0] wd  ;	    // 8-bit data to write to RD
   output [7:0]  rd1, rd2 ; // 8-bit data to read from RS and RT

   // 2-dimensional register (8x8) -- holds actual registers $s0 through $s7
   reg  [7:0] REGS [7:0];


   // WRITE
   always @(posedge clk)
     if (reset) begin REGS[3] <= 0; REGS[2] <= 0;REGS[1] <= 0;REGS[0] <= 0;end
     else if (regwrite) REGS[wa] <= wd;	

   // READ
   assign rd1 = ra1 ? REGS[ra1] : 0;   // looks up address ra1 in reg array
   assign rd2 = ra2 ? REGS[ra2] : 0;   // looks up address ra2 in reg array
   				       // note: register 0 hardwired to 0
endmodule


module datapath (adr, instr, writedata, zero, 
                 alucontrol, alusrca, alusrcb, iord, irwrite, memdata, 
		 memtoreg, pcen, pcsource, regdst, regwrite,
		 clk, reset,groudconstant ) ;

   // INPUTS (13)
   input  [2:0]  alucontrol ;  // control signal for ALU
   input         alusrca    ;  // control signal for 2:1 mux for ALU's srca input
   input  [1:0]  alusrcb    ;  // control signal for 4:1 mux for ALU's srcb input
   input 	 iord       ;  // control signal for 2:1 mux from Program counter
   input  [3:0]  irwrite    ;  // control signal for the 4 DFF's holding the instruction
   input  [7:0]  memdata    ;  // 8-bit line coming from memory's RD line
   input 	 memtoreg   ;  // control signal for the 2:1 mux for memory's WD line
   input         pcen       ;  // control signal for PC's DFF
   input 	 regdst     ;  // control signal for 2:1 mux for memory's WA line
   input 	 regwrite   ;  // control signal for regfile
   input  [1:0]  pcsource   ;  // control signal for 4:1 mux leading to PC register
   input         clk, reset ;  
   input         groudconstant;
	

   // OUTPUTS (4)
   output [ 7:0] adr        ;  // output coming from 2:1 mux from program counter
   output [31:0] instr      ;  // output coming from all 4 DFF's holding the instruction
   output [ 7:0] writedata  ;  // output leading to WD line on memory
   output        zero       ;  // output coming from zero detect module

   // WIRES (15) -- no additional wires needed
   wire   [2:0]  ra1, ra2, wa ;  // regfile's 3-bit inputs
   wire   [7:0]  rd1, rd2, wd ;  // regfile's 8-bit outputs/inputs
   wire   [7:0]  pc, nextpc   ;	 // program counter's wires
   wire   [7:0]  data         ;  // output of memdata's register
   wire   [7:0]  a, srca, srcb, aluresult, aluout ;  // ALU & its MUX's & FF's wires
   wire   [7:0]  constx4      ;	 // output of shift left by 2 module

   // NOTE: notice, no REGS, this is purely comb. module, no always blocks



   // shift left constant field by 2 (constx4)
   // TODO: hint can easily be done using an assign statement
	assign constx4={instr[5:0],2'b00};


   // register file address fields
   // TODO: ra1 and ra2 can easily be wired using assign statement
 	assign ra1=instr[23:21];
	assign ra2=instr[18:16];
   
   // 4 instruction registers
   // TODO: instance 4 8-bit DFF's to hold the single 32-bit instruction
	enableffp f0(.D(memdata),.enable(irwrite[0]),.Q(instr[31:24]),.clk(clk), .reset(reset));
	enableffp f1(.D(memdata),.enable(irwrite[1]),.Q(instr[23:16]),.clk(clk), .reset(reset));
	enableffp f2(.D(memdata),.enable(irwrite[2]),.Q(instr[15:8]),.clk(clk), .reset(reset));
	enableffp f3(.D(memdata),.enable(irwrite[3]),.Q(instr[7:0]),.clk(clk), .reset(reset));
	enableffp ffppcen(.D(nextpc),.enable(pcen),.Q(pc),.clk(clk), .reset(reset));

   // instance remaining DFFs
   // TODO:
	ffp fdata(.D(memdata),.Q(data),.clk(clk), .reset(reset));
	ffp fa(.D(rd1),.Q(a),.clk(clk), .reset(reset));
	ffp fwritedata(.D(rd2),.Q(writedata),.clk(clk), .reset(reset));
	ffp faluout(.D(aluresult),.Q(aluout),.clk(clk), .reset(reset));
   // instance remaining MUXs
   // TODO:
	assign adr= iord ? aluout : pc;
	assign wa= regdst ? instr[13:11] : instr[18:16];
	assign wd= memtoreg ? data : aluout;
	assign srca= alusrca ? a : pc;
	assign srcb= alusrcb[1] ?  (alusrcb[0] ?  constx4:instr[7:0]):(alusrcb[0] ? {{7{groudconstant}},~{groudconstant}}:writedata);
	assign nextpc= pcsource[1] ? (pcsource[0] ?  {8{groudconstant}}:constx4):(pcsource[0] ?  aluout:aluresult);
   // instance register file
   // TODO:
	regfile reg1(.rd1(rd1),.rd2(rd2),.clk(clk),.regwrite(regwrite),.ra1(instr[23:21]),.ra2(instr[18:16]),.wa(wa),.wd(wd),.reset(reset));

   // instance ALU
   // TODO:
   alu a1(.result(aluresult),.a(srca),.b(srcb),.alucont(alucontrol));
  
   // perform zero detect on output of ALU
   // TODO: could use simple assign statement, or NAND gate
	assign zero=(aluresult==0);
endmodule
