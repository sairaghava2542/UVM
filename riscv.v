module adder(input [31:0] a, b, output [31:0] y);
	assign y = $signed(a) + $signed(b);
endmodule
------------------------------------------------------
module alu(input logic [31:0] SrcA, 
			input logic [31:0] SrcB, 
			input logic [3:0] ALUControl , 
			output logic  [31:0] ALUResult, 
			output logic Zero, Sign);

logic [31:0] Sum;
logic Overflow;

assign Sum = SrcA + (ALUControl[0] ? ~SrcB : SrcB) + ALUControl[0];  // sub using 1's complement
assign Overflow = ~(ALUControl[0] ^ SrcB[31] ^ SrcA[31]) & (SrcA[31] ^ Sum[31]) & (~ALUControl[1]);

assign Zero = ~(|ALUResult);
assign Sign = ALUResult[31];


always_comb
		casex (ALUControl)
				4'b000x: ALUResult = Sum;				// sum or diff
				4'b0010: ALUResult = SrcA & SrcB;	// and
				4'b0011: ALUResult = SrcA | SrcB;	// or
                                4'b0100: ALUResult = SrcA << SrcB[4:0];	// sll, slli
				4'b0101: ALUResult = {{30{1'b0}}, Overflow ^ Sum[31]}; //slt, slti
				4'b0110: ALUResult = SrcA ^ SrcB;   // Xor
                                4'b0111: ALUResult = SrcA >> SrcB[4:0];  // shift logic
				4'b1000: ALUResult = ($unsigned(SrcA) < $unsigned(SrcB)); //sltu, stlui
                                4'b1111: ALUResult = SrcA >>> SrcB[4:0]; //shift arithmetic
				default: ALUResult = 32'bx;
		endcase


endmodule
--------------------------------------------------------------------------------------------
module aludec(input logic opb5,
	input logic [2:0] funct3,
	input logic funct7b5,
	input logic [1:0] ALUOp,
	output logic [3:0] ALUControl);
	
logic RtypeSub;
assign RtypeSub = funct7b5 & opb5; // TRUE for R-type subtract
always_comb
	case(ALUOp)
		2'b00: ALUControl = 4'b0000; // addition
		2'b01: ALUControl = 4'b0001; // subtraction
		default: case(funct3) // R–type or I–type ALU
			3'b000: if (RtypeSub)
				ALUControl = 4'b0001; // sub
			else
				ALUControl = 4'b0000; // add, addi
			3'b001: ALUControl = 4'b0100; // sll, slli
			3'b010: ALUControl = 4'b0101; // slt, slti
			3'b011: ALUControl = 4'b1000; // sltu, sltiu
			3'b100: ALUControl = 4'b0110; // xor, xori
			3'b101: if (~funct7b5)
				ALUControl = 4'b0111;	// srl
			else
				ALUControl = 4'b1111;  // sra
			3'b110: ALUControl = 4'b0011; // or, ori
			3'b111: ALUControl = 4'b0010; // and, andi
			default: ALUControl = 4'bxxxx; // ???
			endcase
	endcase
endmodule
--------------------------------------------------
module c_ID_IEx (input logic clk, reset, clear,
        input logic RegWriteD, MemWriteD, JumpD, BranchD, ALUSrcAD,
		input logic [1:0] ALUSrcBD,
        input logic [1:0] ResultSrcD, 
        input logic [3:0] ALUControlD, 
        input logic [2:0] funct3D,          
        output logic RegWriteE, MemWriteE, JumpE, BranchE,  ALUSrcAE,
		output logic [1:0] ALUSrcBE,
        output logic [1:0] ResultSrcE,
        output logic [3:0] ALUControlE, 
        output logic [2:0] funct3E);

always_ff @( posedge clk, posedge reset ) begin

		if (reset) begin
            funct3E <= 0;
			RegWriteE <= 0;
			MemWriteE <= 0;
			JumpE <= 0;
			BranchE <= 0; 
			ALUSrcAE <= 0;
			ALUSrcBE <= 0;
			ResultSrcE <= 0;
			ALUControlE <= 0;          
		end

		else if (clear) begin
            funct3E <= 0;
			RegWriteE <= 0;
			MemWriteE <= 0;
			JumpE <= 0;
			BranchE <= 0; 
			ALUSrcAE <= 0;
			ALUSrcBE <= 0;
			ResultSrcE <= 0;
			ALUControlE <= 0;    			
		end
		
		else begin
            funct3E <= funct3D;
			RegWriteE <= RegWriteD;
			MemWriteE <= MemWriteD;
			JumpE <= JumpD;
			BranchE <= BranchD; 
			ALUSrcAE <= ALUSrcAD;
			ALUSrcBE <= ALUSrcBD;
			ResultSrcE <= ResultSrcD;
			ALUControlE <= ALUControlD;   
		end
	 end
endmodule
-------------------------------------------------------------
module c_IEx_IM (input logic clk, reset,
                input logic RegWriteE, MemWriteE,
                input logic [1:0] ResultSrcE,  
                output logic RegWriteM, MemWriteM,
                output logic [1:0] ResultSrcM);

    always_ff @( posedge clk, posedge reset ) begin
        if (reset) begin
            RegWriteM <= 0;
            MemWriteM <= 0;
            ResultSrcM <= 0;
        end
        else begin
            RegWriteM <= RegWriteE;
            MemWriteM <= MemWriteE;
            ResultSrcM <= ResultSrcE; 
        end
        
    end

endmodule
-------------------------------------------------------------------
module c_IM_IW (input logic clk, reset, 
                input logic RegWriteM, 
                input logic [1:0] ResultSrcM, 
                output logic RegWriteW, 
                output logic [1:0] ResultSrcW);

    always_ff @( posedge clk, posedge reset ) begin
        if (reset) begin
            RegWriteW <= 0;
            ResultSrcW <= 0;           
        end

        else begin
            RegWriteW <= RegWriteM;
            ResultSrcW <= ResultSrcM; // lol this wasted 1 hour
        end

    end

endmodule
-----------------------------------------------------------------
module controller(input logic clk, reset,
						input logic [6:0] op,
                        input logic [2:0] funct3D,
						input logic funct7b5,
						input logic ZeroE,
						input logic SignE,
						input logic FlushE,
						output logic ResultSrcE0,
						output logic [1:0] ResultSrcW,
						output logic MemWriteM,
						output logic PCJalSrcE, PCSrcE, ALUSrcAE, 
						output logic [1:0] ALUSrcBE,
						output logic RegWriteM, RegWriteW,
						output logic [2:0] ImmSrcD,
						output logic [3:0] ALUControlE);

logic [1:0] ALUOpD;
logic [1:0] ResultSrcD, ResultSrcE, ResultSrcM;
logic [3:0] ALUControlD;
logic BranchD, BranchE, MemWriteD, MemWriteE, JumpD, JumpE;
logic ZeroOp, ALUSrcAD, RegWriteD, RegWriteE;
logic [1:0] ALUSrcBD;
logic SignOp;
logic BranchOp;
logic [2:0]funct3E ;  

// main decoder
maindec md(op, ResultSrcD, MemWriteD, BranchD, ALUSrcAD, ALUSrcBD, RegWriteD, JumpD, ImmSrcD, ALUOpD);

// alu decoder
aludec ad(op[5], funct3D, funct7b5, ALUOpD, ALUControlD);


c_ID_IEx c_pipreg0(clk, reset, FlushE, RegWriteD, MemWriteD, JumpD, BranchD, ALUSrcAD, ALUSrcBD, ResultSrcD, ALUControlD, funct3D, 
										 RegWriteE, MemWriteE, JumpE, BranchE, ALUSrcAE, ALUSrcBE, ResultSrcE, ALUControlE, funct3E);
assign ResultSrcE0 = ResultSrcE[0];

c_IEx_IM c_pipreg1(clk, reset, RegWriteE, MemWriteE, ResultSrcE, RegWriteM,  MemWriteM, ResultSrcM);

c_IM_IW c_pipreg2 (clk, reset, RegWriteM, ResultSrcM, RegWriteW, ResultSrcW);


assign ZeroOp = ZeroE ^ funct3E[0]; // Complements Zero flag for BNE Instruction
assign SignOp = (SignE ^ funct3E[0]) ; //Complements Sign for BGE
assign BranchOp = funct3E[2] ? (SignOp) : (ZeroOp); 
assign PCSrcE = (BranchE & BranchOp) | JumpE;
assign PCJalSrcE = (op == 7'b1100111) ? 1 : 0; // jalr

//mux2 BranchSrc (ZeroOp, SignOp, funct3[2], BranchOp); // fix this later
  
endmodule
--------------------------------------------------------------------------
module ID_IEx  (input logic clk, reset, clear,
                input logic [31:0] RD1D, RD2D, PCD, 
                input logic [4:0] Rs1D, Rs2D, RdD, 
                input logic [31:0] ImmExtD, PCPlus4D,
                output logic [31:0] RD1E, RD2E, PCE, 
                output logic [4:0] Rs1E, Rs2E, RdE, 
                output logic [31:0] ImmExtE, PCPlus4E);

    always_ff @( posedge clk, posedge reset ) begin
        if (reset) begin
            RD1E <= 0;
            RD2E <= 0;
            PCE <= 0;
            Rs1E <= 0;
            Rs2E <= 0;
            RdE <= 0;
            ImmExtE <= 0;
            PCPlus4E <= 0;
        end

        else if (clear) begin
            RD1E <= 0;
            RD2E <= 0;
            PCE <= 0;
            Rs1E <= 0;
            Rs2E <= 0;
            RdE <= 0;
            ImmExtE <= 0;
            PCPlus4E <= 0;
        end
        else begin
            RD1E <= RD1D;
            RD2E <= RD2D;
            PCE <= PCD;
            Rs1E <= Rs1D;
            Rs2E <= Rs2D;
            RdE <= RdD;
            ImmExtE <= ImmExtD;
            PCPlus4E <= PCPlus4D;
        end

    end

endmodule
----------------------------------------------------------------------------------------------
module IEx_IMem(input logic clk, reset,
                input logic PCJalSrcE,
                input logic [31:0] ALUResultE, WriteDataE, 
                input logic [4:0] RdE, 
                input logic [31:0] PCPlus4E,
                output logic [31:0] ALUResultM, WriteDataM,
                output logic [4:0] RdM, 
                output logic [31:0] PCPlus4M,
                output logic        PCJalSrcM);

always_ff @( posedge clk, posedge reset ) begin 
    if (reset) begin
        ALUResultM <= 0;
        WriteDataM <= 0;
        RdM <= 0; 
        PCPlus4M <= 0;
        PCJalSrcM <= 0 ;
    end

    else begin
        ALUResultM <= ALUResultE;
        WriteDataM <= WriteDataE;
        RdM <= RdE; 
        PCPlus4M <= PCPlus4E; 
        PCJalSrcM <= PCJalSrcE;
    end
    
end

endmodule
-----------------------------------------------------------------------------------------
module IF_ID (input logic clk, reset, clear, enable,
            input logic [31:0] InstrF, PCF, PCPlus4F,
            output logic [31:0] InstrD, PCD, PCPlus4D);


always_ff @( posedge clk, posedge reset ) begin
    if (reset) begin // Asynchronous Clear
        InstrD <= 0;
        PCD <= 0;
        PCPlus4D <= 0;
    end

    else if (enable) begin 
		 if (clear) begin // Synchrnous Clear
			  InstrD <= 0;
			  PCD <= 0;
			  PCPlus4D <= 0;	 
		 end
		 
		 else begin	 
			  InstrD <= InstrF;
			  PCD <= PCF;
			  PCPlus4D <= PCPlus4F;
		 end
	 end
end

endmodule
---------------------------------------------------------------------------------------
module IMem_IW (input logic clk, reset,
                input logic [31:0] ALUResultM, ReadDataM,  
                input logic [4:0] RdM, 
                input logic [31:0] PCPlus4M,
                output logic [31:0] ALUResultW, ReadDataW,
                output logic [4:0] RdW, 
                output logic [31:0] PCPlus4W);

always_ff @( posedge clk, posedge reset ) begin 
    if (reset) begin
        ALUResultW <= 0;
        ReadDataW <= 0;
        
        RdW <= 0; 
        PCPlus4W <= 0;
    end

    else begin
        ALUResultW <= ALUResultM;
        ReadDataW <= ReadDataM;
        
        RdW <= RdM; 
        PCPlus4W <= PCPlus4M;        
    end
    
end

endmodule
-----------------------------------------------------------------------
module dmem(input logic clk, we, 
		input logic [31:0] a, wd, 
		output logic [31:0] rd);
		
  logic [31:0] RAM[255:0]; // 64 x 32 bit memory
  assign rd = RAM[a[9:2]];     // read operation
	
	// 6 bit address enough to address the 64 locations in data memory
	
	always_ff @(posedge clk)
      if (we) RAM[a[9:2]] <= wd;
endmodule
----------------------------------------------------------------------
module datapath(input logic clk, reset,
		input logic [1:0] ResultSrcW,
		input logic PCJalSrcE, PCSrcE, ALUSrcAE, 
		input logic [1:0] ALUSrcBE,
		input logic RegWriteW,
		input logic [2:0] ImmSrcD,
		input logic [3:0] ALUControlE,
		output logic ZeroE,
		output logic SignE,
		output logic [31:0] PCF,
		input logic [31:0] InstrF,
		output logic [31:0] InstrD,
		output logic [31:0] ALUResultM, WriteDataM,
		input logic [31:0] ReadDataM,
		input logic [1:0] ForwardAE, ForwardBE,
		output logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E,
        output logic [4:0] RdE, RdM, RdW,
		input logic StallD, StallF, FlushD, FlushE);


	logic [31:0] PCD, PCE, ALUResultE, ALUResultW, ReadDataW;
	logic [31:0] PCNextF, PCPlus4F, PCPlus4D, PCPlus4E, PCPlus4M, PCPlus4W, PCTargetE, BranJumpTargetE;
	logic [31:0] WriteDataE;
	logic [31:0] ImmExtD, ImmExtE;
	logic [31:0] SrcAEfor, SrcAE, SrcBE, RD1D, RD2D, RD1E, RD2E;
	logic [31:0] ResultW;
    logic        PCJalSrcM ;
	
	logic [4:0] RdD; // destination register address

	
	// Fetch Stage
	
    mux2 jal_r(PCTargetE, ALUResultE, PCJalSrcM, BranJumpTargetE);
	mux2 pcmux(PCPlus4F, BranJumpTargetE, PCSrcE, PCNextF);
	flopenr IF(clk, reset, ~StallF, PCNextF, PCF);
	adder pcadd4(PCF, 32'd4, PCPlus4F);
	
		
	// Instruction Fetch - Decode Pipeline Register	
	
    IF_ID pipreg0 (clk, reset, FlushD, ~StallD, InstrF, PCF, PCPlus4F, InstrD, PCD, PCPlus4D);
	assign Rs1D = InstrD[19:15];
	assign Rs2D = InstrD[24:20];		
	regfile rf (clk, RegWriteW, Rs1D, Rs2D, RdW, ResultW, RD1D, RD2D);	
	assign RdD = InstrD[11:7];
	extend ext(InstrD[31:7], ImmSrcD, ImmExtD);
	
	// Decode - Execute Pipeline Register	
	
	ID_IEx pipreg1 (clk, reset, FlushE, RD1D, RD2D, PCD, Rs1D, Rs2D, RdD, ImmExtD, PCPlus4D, RD1E, RD2E, PCE, Rs1E, Rs2E, RdE, ImmExtE, PCPlus4E);
	mux3 forwardMuxA (RD1E, ResultW, ALUResultM, ForwardAE, SrcAEfor);
	mux2 srcamux(SrcAEfor, 32'b0, ALUSrcAE, SrcAE); // for lui
	mux3 forwardMuxB (RD2E, ResultW, ALUResultM, ForwardBE, WriteDataE);
	mux3 srcbmux(WriteDataE, ImmExtE, PCTargetE, ALUSrcBE, SrcBE); 
	adder pcaddbranch(PCE, ImmExtE, PCTargetE); // Next PC for jump and branch instructions
	alu alu(SrcAE, SrcBE, ALUControlE, ALUResultE, ZeroE, SignE);
	
	
		
	// Execute - Memory Access Pipeline Register
    IEx_IMem pipreg2 (clk, reset, PCJalSrcE, ALUResultE, WriteDataE, RdE, PCPlus4E, ALUResultM, WriteDataM, RdM, PCPlus4M, PCJalSrcM);
	
		
	// Memory - Register Write Back Stage
	IMem_IW pipreg3 (clk, reset, ALUResultM, ReadDataM, RdM, PCPlus4M, ALUResultW, ReadDataW, RdW, PCPlus4W);
	mux3 resultmux( ALUResultW, ReadDataW, PCPlus4W, ResultSrcW, ResultW);

endmodule-----------------
-------------------------------------------------------------------------------------
module extend (input logic [31:7] instr,
			input logic [2:0] immsrc, 
			output logic [31:0] immext);


	always_comb
		case(immsrc)
		// I−type
		3'b000: immext = {{20{instr[31]}}, instr[31:20]};
		
		// S−type (stores)
		3'b001: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
		
		// B−type (branches)
		3'b010: immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
		
		// J−type (jal)
		3'b011: immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};  
		
		// U-type
		3'b100: immext = {instr[31:12], 12'b0};
		
		default: immext = 32'bx; // undefined
	endcase
endmodule
--------------------------------------------------------------------------------------
module flopenr (input logic clk, reset, en,
			input logic [31:0] d,
			output logic [31:0] q);
					
	always_ff @(posedge clk, posedge reset)
		
		if (reset) q <= 0;
		else if (en) q <= d;
	
endmodule
--------------------------------------------------------------------------------------
module flopr (	input logic clk, reset, 
			input logic [31:0] d, 
			output logic [31:0] q);

	always_ff @(posedge clk, posedge reset)

		if (reset) q <= 0;
		
		else q <= d;
	
endmodule
------------------------------------------------------------------
module hazardunit(input logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E,
                input logic [4:0] RdE, RdM, RdW,
                input logic RegWriteM, RegWriteW,
				    input logic ResultSrcE0, PCSrcE,
                output logic [1:0] ForwardAE, ForwardBE,
                output logic StallD, StallF, FlushD, FlushE);
					 
// RAW					 
// Whenever source register (Rs1E, Rs2E) in execution stage matchces with the destination register (RdM, RdW)
// of a previous instruction's Memory or WriteBack stage forward the ALUResultM or ResultW
// And also only when RegWrite is asserted

logic lwStall;
    always_comb begin
	    ForwardAE = 2'b00;
		 ForwardBE = 2'b00;
        if ((Rs1E == RdM) & (RegWriteM) & (Rs1E != 0)) // higher priority - most recent
            ForwardAE = 2'b10; // for forwarding ALU Result in Memory Stage
        else if ((Rs1E == RdW) & (RegWriteW) & (Rs1E != 0))
            ForwardAE = 2'b01; // for forwarding WriteBack Stage Result
                    


        if ((Rs2E == RdM) & (RegWriteM) & (Rs2E != 0))
            ForwardBE = 2'b10; // for forwarding ALU Result in Memory Stage

        else if ((Rs2E == RdW) & (RegWriteW) & (Rs2E != 0))
            ForwardBE = 2'b01; // for forwarding WriteBack Stage Result
     
	  end
	  
// For Load Word Dependency result does not appear until end of Data Memory Access Stage
// if Destination register in EXE stage is equal to souce register in decode stage
// stall previous instructions until the the load word is avialbe at the writeback stage
// Introduce One cycle latency for subsequent instructions after load word 
// There is two cycle difference between Memory Access and the immediate next instruction

   assign lwStall = (ResultSrcE0 == 1) & ((RdE == Rs1D) | (RdE == Rs2D));
//   assign FlushE = lwStall;
	assign StallF = lwStall;
	assign StallD = lwStall;
	
// control hazard
// whenever branch has been taken, we flush the following two instructions from decode and execute pip reg
	assign FlushE = lwStall | PCSrcE;
	assign FlushD = PCSrcE;

endmodule
-------------------------------------------------------------------------------------------------------------
module maindec(
	input logic [6:0] op,
	output logic [1:0] ResultSrc,
	output logic MemWrite,
	output logic Branch, ALUSrcA, 
	output logic [1:0] ALUSrcB,
	output logic RegWrite, Jump,
	output logic [2:0] ImmSrc,
	output logic [1:0] ALUOp
	);
	
logic [13:0] controls;
assign {RegWrite, ImmSrc, ALUSrcA, ALUSrcB, MemWrite, ResultSrc, Branch, ALUOp, Jump} = controls;

always_comb
	case(op)
	// RegWrite_ImmSrc_ALUSrcA_ALUSrcB_MemWrite_ResultSrc_Branch_ALUOp_Jump
		7'b0000011: controls = 14'b1_000_0_01_0_01_0_00_0; // lw
		7'b0100011: controls = 14'b0_001_0_01_1_00_0_00_0; // sw
		7'b0110011: controls = 14'b1_xxx_0_00_0_00_0_10_0; // R–type
		7'b1100011: controls = 14'b0_010_0_00_0_00_1_01_0; // B-type
		7'b0010011: controls = 14'b1_000_0_01_0_00_0_10_0; // I–type ALU
		7'b1101111: controls = 14'b1_011_0_00_0_10_0_00_1; // jal
		7'b0010111: controls = 14'b1_100_1_10_0_00_0_00_0; // auipc // PC Target for SrcB
		7'b0110111: controls = 14'b1_100_1_01_0_00_0_00_0; // lui
		7'b1100111: controls = 14'b1_000_0_01_0_10_0_00_1; // jalr
		7'b0000000: controls = 14'b0_000_0_00_0_00_0_00_0; // for default values on reset
		
		default: 	controls = 14'bx_xxx_x_xx_x_xx_x_xx_x; // instruction not implemented
	endcase
endmodule
-------------------------------------------------------------------------------------
module mux2 (input logic [31:0] d0, d1,input logic s,output logic [31:0] y);
	assign y = s ? d1 : d0;
endmodule
----------------------------------------------------------------------------------
module mux3 (input logic [31:0] d0, d1, d2,input logic [1:0] s,output logic [31:0] y);
	assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule
------------------------------------------------------------------------------------------

module mux4 (input logic [31:0] d0, d1, d2, d3, input logic [1:0] s, 
			output logic [31:0] y);
always_comb begin
	case(s)
			2'b00: y = d0;
			2'b01: y = d1;
			2'b10: y = d2;
			2'b11: y = d3;
	endcase
end

endmodule
----------------------------------------------------------------------------------------
module regfile(input logic clk,
					input logic we3,
					input logic [4:0] a1, a2, a3,
					input logic [31:0] wd3,
					output logic [31:0] rd1, rd2);
					
  logic [31:0] register [31:0]; 	// register file
	

	
	// write on falling edge
	// read on rising edge 
	
	// r0 hardwired to 0
	
	
  assign rd1 = (a1 != 0 ) ? register[a1] : 0;
  assign rd2 = (a2 != 0 ) ? register[a2] : 0;
	
	
	always_ff @(negedge clk)
      if (we3) register[a3] <= wd3;

endmodule
----------------------------------------------------------------------------------------
module risc_top (input  logic 	    clk, reset, 
            input  logic [31:0]     InstrF , 
            output logic [31:0]     PCF ,	        		
            output logic [31:0]     WriteDataM, DataAdrM, ReadDataM,
	    output logic 	    MemWriteM);
            
	
// instantiate processor and memories

	riscv_pip rv( clk, reset, PCF, InstrF, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
	dmem dmem(clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
endmodule
----------------------------------------------------------------------------------
module riscv_pip(input logic clk, reset,
	output logic [31:0] PCF,
	input logic [31:0] InstrF,
	output logic MemWriteM,
	output logic [31:0] ALUResultM, WriteDataM,
	input logic [31:0] ReadDataM);
	
logic ALUSrcAE, RegWriteM, RegWriteW, ZeroE, SignE, PCJalSrcE, PCSrcE;
logic [1:0] ALUSrcBE;
logic StallD, StallF, FlushD, FlushE, ResultSrcE0;
logic [1:0] ResultSrcW; 
logic [2:0] ImmSrcD;
logic [3:0] ALUControlE;
logic [31:0] InstrD;
logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E;
logic [4:0] RdE, RdM, RdW;
logic [1:0] ForwardAE, ForwardBE;

controller c(clk, reset, InstrD[6:0], InstrD[14:12], InstrD[30], ZeroE, SignE, FlushE, ResultSrcE0, ResultSrcW, MemWriteM, PCJalSrcE, PCSrcE, ALUSrcAE, ALUSrcBE, RegWriteM, RegWriteW, ImmSrcD, ALUControlE);

hazardunit h (Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW, RegWriteM, RegWriteW, ResultSrcE0, PCSrcE, ForwardAE, ForwardBE, StallD, StallF, FlushD, FlushE);

datapath dp(clk, reset, ResultSrcW, PCJalSrcE, PCSrcE,ALUSrcAE, ALUSrcBE, RegWriteW, ImmSrcD, ALUControlE, ZeroE, SignE, PCF, InstrF, InstrD, ALUResultM, WriteDataM, ReadDataM, ForwardAE, ForwardBE, Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW, StallD, StallF, FlushD, FlushE);

endmodule
------------------------------------------------------------------------------------------------------------------------------
                                                       VERIFICATION
------------------------------------------------------------------------------------------------------------------------------
`timescale 1ns/1ns
`include "uvm_macros.svh"


  module tb_top;
    
   import uvm_pkg::*;
   import risc_pkg::* ;
   
   
   bit clk ;
    
  // RAL HDL paths
  string blk_hdl_path = "tb_top.dut";
  string mem_hdl_path = "dmem.RAM";
  string reg_hdl_path = "rv.dp.rf.register[%0d]";
  
  // Interface Initaition
  intf risc_intf(clk)  ;
   
  // DUT Intiation 
  risc_top  dut (
           .clk(clk),   
           .reset(risc_intf.reset), 
           .InstrF(risc_intf.InstrF),	
           .PCF(risc_intf.PCF),
           .WriteDataM(risc_intf.WriteDataM),
           .DataAdrM(risc_intf.DataAdrM),
           .ReadDataM(risc_intf.ReadDataM),
           .MemWriteM(risc_intf.MemWriteM)
		  );
  
  
  // Clock Generation
  initial begin 
    forever  #5 clk = ~clk;
  end
  
  
  initial begin 
    uvm_config_db #(virtual intf)::set(null,"*","risc_intf",risc_intf);
    uvm_config_db #(string)::set(null,"*", "blk_hdl_path", blk_hdl_path);
    uvm_config_db #(string)::set(null,"*", "mem_hdl_path", mem_hdl_path);
    uvm_config_db #(string)::set(null,"*", "reg_hdl_path", reg_hdl_path);
    run_test();
  end
    
  initial 
  begin
    // Required to dump signals to EPWave
    $dumpfile("dump.vcd");
    $dumpvars(0);
  end
  

endmodule 
----------------------------------------------------------------------------------------------------------
class sequencer extends uvm_sequencer #(seq_item); 
  `uvm_component_utils(sequencer)
  
  function new (string name = "sequencer", uvm_component parent = null);
    super.new (name, parent);
  endfunction
  
endclass
-------------------------------------------------------------------------------------
import risc_pkg::* ;

class seq_item extends uvm_sequence_item ; 
	 
     // Constructor
     function new (string name = "risc_seq_item") ;
         super.new(name) ;
     endfunction		 
	  
     // Inputs  
     rand logic        reset  ;
     rand bit   [31:0] InstrF ; 
	 
     // Outputs 
     logic [31:0] PCF        ;
     logic [31:0] WriteDataM ;
     logic [31:0] DataAdrM   ;
     logic [31:0] ReadDataM  ;
     logic        MemWriteM  ;
  
     // Auxiliary fields 
     rand instr_type inst_type ;
     bit  lwstall ;
     bit  beqflush ;
  
     // Functions Automation (copy, compare and print)
     `uvm_object_utils_begin(seq_item)
     `uvm_field_int (reset      , UVM_DEFAULT)
     `uvm_field_int (InstrF     , UVM_DEFAULT)
     `uvm_field_int (PCF        , UVM_DEFAULT)
     `uvm_field_int (WriteDataM , UVM_DEFAULT)
     `uvm_field_int (DataAdrM   , UVM_DEFAULT)
     `uvm_field_int (ReadDataM  , UVM_DEFAULT)
     `uvm_field_int (MemWriteM  , UVM_DEFAULT) 
     `uvm_field_int (lwstall    , UVM_DEFAULT)
     `uvm_field_int (beqflush   , UVM_DEFAULT)
     `uvm_field_enum(instr_type , inst_type , UVM_DEFAULT) 
     `uvm_object_utils_end
  
  
  // Basic Constraints 
  constraint opcode_range {InstrF[6:0] inside {lw, imm,
                                               auipc, sw, 
                                               arith, lui,
                                               brnch, jalr,
                                               jal}; } ;   // op_code in the range of the supported instructions only 

  //constraint opcode_dist {InstrF[6:0] dist{lw:=1, imm:=3, auipc:=1, sw:=1, arith:=2, lui:=1, brnch:=1, jalr:=1, jal:=1}; };
	 
  constraint funct_range { (InstrF[6:0] == lw) -> (InstrF[14:12] == 3'b010) ; // lw funct3 
                             
      ((InstrF[6:0] == imm) && (InstrF[14:12] == 3'b101)) -> (InstrF[31:25] inside {7'b0000000, 7'b0100000}) ; // srli and srai funct7  
                              
      ((InstrF[6:0] == imm) && (InstrF[14:12] == 3'b001)) -> (InstrF[31:25] == 7'b0000000) ;// slli funct7  
       
      (InstrF[6:0] == imm) -> (InstrF[14:12] != 3'b011) ; // sltiu is not supported 
       
      (InstrF[6:0] == sw) -> (InstrF[14:12] == 3'b010) ; // sw funct3
                             
      (InstrF[6:0] == arith) -> (InstrF[14:12] inside {add, sll, slt, Xor, srl, Or, And}); // funct3 in range of supported R-type instructions
                             
      ((InstrF[6:0] == arith) && (InstrF[14:12] == add)) -> (InstrF[31:25] inside {7'b0000000, 7'b0100000}); // add and sub funct7
                             
      ((InstrF[6:0] == arith) && (InstrF[14:12]==sll || InstrF[14:12]==slt || InstrF[14:12]==Xor || InstrF[14:12]==Or || InstrF[14:12]==And )) -> (InstrF[31:25] == 7'b0000000); 
                             
      ((InstrF[6:0] == arith) && (InstrF[14:12] == sra)) -> (InstrF[31:25] inside {7'b0100000, 7'b0000000}); // sra and srl funct7  
                             
      (InstrF[6:0] == brnch) -> (InstrF[14:12] inside {beq, bne, blt, bge}); // funct3 in range of supported branch instructions 
                             
      (InstrF[6:0] == jalr) -> (InstrF[14:12] == 3'b000); // jalr funct3
					
      ((InstrF[6:0] != sw) && (InstrF[6:0] != brnch)) -> (InstrF[11:7] != 0) ; // rd is not equal to 0                    
                             } ;
  
  constraint reset_dist {reset dist{1:=1 , 0:=100000}; };
  
 //========================= Item Functions ===========================\\
  
  // Reset Function 
      function void Reset();
        InstrF     = 32'b0;
        PCF        = 32'b0;
        DataAdrM   = 32'b0;
        MemWriteM  = 1'b0;
        WriteDataM = 32'b0;
        lwstall    = 1'b0;
        beqflush   = 1'b0;
        inst_type  = RESET;
      endfunction
  
  
 // Get Instruction type function  
      function Get_type ;
        case (InstrF[6:0])  
          
          lw    : inst_type = LW    ;
          sw    : inst_type = SW    ;
          lui   : inst_type = LUI   ;
          auipc : inst_type = AUIPC ;
          jalr  : inst_type = JALR  ;
          jal   : inst_type = JAL   ;
          
          imm : begin 
            case(InstrF[14:12])     
            3'b000 : inst_type = ADDI ;
            3'b001 : inst_type = SLLI ;
            3'b010 : inst_type = SLTI ;
            3'b100 : inst_type = XORI ;
            3'b110 : inst_type = ORI  ; 
            3'b111 : inst_type = ANDI ;    
            3'b101 : begin 
              if (InstrF[30] == 0) inst_type = SRLI ;
              else inst_type = SRAI ;
             end
            endcase 
          end
          
          arith : begin 
            case(InstrF[14:12])     
            3'b001 : inst_type = SLL ;
            3'b010 : inst_type = SLT ;
            3'b100 : inst_type = XOR ;
            3'b110 : inst_type = OR  ; 
            3'b111 : inst_type = AND ;   
            3'b000 : begin 
              if (InstrF[30] == 0) inst_type = ADD ;
                     else inst_type = SUB ;
                     end
            3'b101 : begin 
              if (InstrF[30] == 0) inst_type = SRL ;
                     else inst_type = SRA ;
                     end  
            endcase   
          end
            
          brnch : begin 
            case(InstrF[14:12])     
            3'b000 : inst_type = BEQ ;
            3'b001 : inst_type = BNE ;
            3'b100 : inst_type = BLT ;
            3'b101 : inst_type = BGE  ;     
            endcase 
          end 
        default : inst_type = UNKNOWN ; 
        endcase 

      endfunction
  
  
  // Extend Immediate function 
  function [31:0] Extend ;

      case(InstrF[6:0])
        // I−type
        imm, lw, jalr : Extend = {{20{InstrF[31]}}, InstrF[31:20]};
		
        // S−type (stores)
        sw : Extend = {{20{InstrF[31]}}, InstrF[31:25], InstrF[11:7]};
		
        // B−type (branches)
        brnch : Extend = {{20{InstrF[31]}}, InstrF[7], InstrF[30:25], InstrF[11:8], 1'b0};
		
        // J−type (jal)
        jal : Extend = {{12{InstrF[31]}}, InstrF[19:12], InstrF[20], InstrF[30:21], 1'b0};
		
        // U-type
        lui, auipc : Extend = {InstrF[31:12], 12'b0};
		
        default: Extend = 32'bx; // undefined
	endcase  
  endfunction : Extend
  
endclass 
----------------------------------------------------------------------------------------------------------------
class rand_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(rand_seq)
  
  seq_item rand_item ;
  
  function  new(string name = "rand_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    rand_item = seq_item::type_id::create("rand_item") ;
    start_item(rand_item) ; 
    assert(rand_item.randomize() )           
    else
    `uvm_error(get_type_name(),"randomization failed in rand_sequence")
    rand_item.Get_type ;  
    finish_item(rand_item);	 
  endtask
  
endclass: rand_seq


//--------------------------------------------------------------------------------

class reset_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(reset_seq)
  
  seq_item reset_item ;
  
  function  new(string name = "reset_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    reset_item = seq_item::type_id::create("reset_item") ;
    start_item(reset_item) ; 
    assert(reset_item.randomize() with {
           reset_item.inst_type == RESET ;
           reset_item.reset == 1;})
     else     
       `uvm_error(get_type_name(),"randomization failed in reset_sequence")
       
       finish_item(reset_item);	 
  endtask
  
endclass: reset_seq

//----------------------------------------------------------------------------------


class lw_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(lw_seq)
  
  seq_item lw_item ;
  
  function  new(string name = "lw_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    lw_item = seq_item::type_id::create("lw_item") ;
    start_item(lw_item) ; 
    assert(lw_item.randomize() with {
           lw_item.inst_type == LW ;
           lw_item.InstrF[6:0] == lw ;
           lw_item.InstrF[14:12] == 3'b010;})
     else     
       `uvm_error(get_type_name(),"randomization failed in lw_sequence")
       
     finish_item(lw_item);	 
  endtask
  
endclass: lw_seq

//----------------------------------------------------------------------------------

class imm_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(imm_seq)
  
  seq_item imm_item ;
  
  function  new(string name = "imm_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    imm_item = seq_item::type_id::create("imm_item") ;
    start_item(imm_item) ;  
    assert(imm_item.randomize() with {
           imm_item.InstrF[6:0] == imm ;
           imm_item.InstrF[11:7] != 0 ; // rd not 0
           imm_item.InstrF[14:12] inside {3'b000,3'b001,3'b010,3'b100,3'b101,3'b110,3111}; })  
     else     
      `uvm_error(get_type_name(),"randomization failed in imm_sequence")
       imm_item.Get_type ;
     finish_item(imm_item);	 
  endtask
  
endclass: imm_seq

//----------------------------------------------------------------------------------

class arith_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(arith_seq)
  
  seq_item arith_item ;
  
  function  new(string name = "arith_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    arith_item = seq_item::type_id::create("arith_item") ;
    start_item(arith_item) ;  
    assert(arith_item.randomize() with {
           arith_item.InstrF[6:0] == arith ;
           arith_item.InstrF[11:7] != 0 ; // rd not 0
           arith_item.InstrF[14:12] inside {3'b000,3'b001,3'b010,3'b100,3'b101,3'b110,3111}; })  
     else     
      `uvm_error(get_type_name(),"randomization failed in arith_sequence")
       arith_item.Get_type ;
     finish_item(arith_item);	 
  endtask
  
endclass: arith_seq

//----------------------------------------------------------------------------------

class branch_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(branch_seq)
  
  seq_item branch_item ;
  
  function  new(string name = "branch_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    branch_item = seq_item::type_id::create("branch_item") ;
    start_item(branch_item) ;  
    assert(branch_item.randomize() with {
           branch_item.InstrF[6:0] == brnch ;
           branch_item.InstrF[14:12] inside {beq,bne,blt,bge}; })  
     else     
      `uvm_error(get_type_name(),"randomization failed in branch_sequence")
       branch_item.Get_type ;
     finish_item(branch_item);	 
  endtask
  
endclass: branch_seq

//----------------------------------------------------------------------------------


class addi_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(addi_seq)
  
  seq_item addi_item ;
  
  function  new(string name = "addi_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    addi_item = seq_item::type_id::create("addi_item") ;
    start_item(addi_item) ;  
    assert(addi_item.randomize() with {
           addi_item.inst_type == ADDI ;
           addi_item.InstrF[6:0] == imm ;
           addi_item.InstrF[19:15] != 0 ; // rs1 not 0 
           addi_item.InstrF[11:7] != 0 ; // rd not 0
           addi_item.InstrF[14:12] == 3'b000;})  
     else     
      `uvm_error(get_type_name(),"randomization failed in addi_sequence")
       
     finish_item(addi_item);	 
  endtask
  
endclass: addi_seq

//----------------------------------------------------------------------------------

class slli_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(slli_seq)
  
  seq_item slli_item ;
  
  function  new(string name = "slli_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    slli_item = seq_item::type_id::create("slli_item") ;
    start_item(slli_item) ; 
    assert(slli_item.randomize() with {
           slli_item.inst_type == SLLI ;
           slli_item.InstrF[6:0] == imm ;
           slli_item.InstrF[14:12] == 3'b001 ;
           slli_item.InstrF[31:25] == 7'b0000000;})
    else     
      `uvm_error(get_type_name(),"randomization failed in slli_sequence")
       
    finish_item(slli_item);	 
  endtask
  
endclass: slli_seq

//----------------------------------------------------------------------------------


class slti_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(slti_seq)
  
  seq_item slti_item ;
  
  function  new(string name = "slti_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    slti_item = seq_item::type_id::create("slti_item") ;
    start_item(slti_item) ; 
    assert(slti_item.randomize() with {
           slti_item.inst_type == SLTI ;
           slti_item.InstrF[6:0] == imm ;
           slti_item.InstrF[14:12] == 3'b010;})
     else     
       `uvm_error(get_type_name(),"randomization failed in slti_sequence")
       
     finish_item(slti_item);	 
  endtask
  
endclass: slti_seq

//----------------------------------------------------------------------------------

class xori_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(xori_seq)
  
  seq_item xori_item ;
  
  function  new(string name = "xori_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    xori_item = seq_item::type_id::create("xori_item") ;
    start_item(xori_item) ; 
    assert(xori_item.randomize() with {
           xori_item.inst_type == XORI ;
           xori_item.InstrF[6:0] == imm ;
           xori_item.InstrF[14:12] == 3'b100;})
     else     
       `uvm_error(get_type_name(),"randomization failed in xori_sequence")
       
       finish_item(xori_item);	 
  endtask
  
endclass: xori_seq

//----------------------------------------------------------------------------------


class sw_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(sw_seq)
  
  seq_item sw_item ;
  
  function  new(string name = "sw_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    sw_item = seq_item::type_id::create("sw_item") ;
    start_item(sw_item) ; 
    assert(sw_item.randomize() with {
           sw_item.inst_type == SW ;
           sw_item.InstrF[6:0] == sw ;
           sw_item.InstrF[14:12] == 3'b010;})
     else     
       `uvm_error(get_type_name(),"randomization failed in sw_sequence")
       
       finish_item(sw_item);	 
  endtask
  
endclass: sw_seq

//----------------------------------------------------------------------------------

class jalr_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(jalr_seq)
  
  seq_item jalr_item ;
  
  function  new(string name = "jalr_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    jalr_item = seq_item::type_id::create("jalr_item") ;
    start_item(jalr_item) ; 
    assert(jalr_item.randomize() with {
           jalr_item.inst_type == JALR ;
           jalr_item.InstrF[6:0] == jalr ;
           jalr_item.InstrF[14:12] == 3'b000;})
     else     
       `uvm_error(get_type_name(),"randomization failed in jalr_sequence")
       
       finish_item(jalr_item);	 
  endtask
  
endclass: jalr_seq

//---------------------------------------------------------------------------------

class jal_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(jal_seq)
  
  seq_item jal_item ;
  
  function  new(string name = "jal_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    jal_item = seq_item::type_id::create("jal_item") ;
    start_item(jal_item) ; 
    assert(jal_item.randomize() with {
           jal_item.inst_type == JAL ;
           jal_item.InstrF[6:0] == jal ;})
     else     
       `uvm_error(get_type_name(),"randomization failed in JAL_sequence")
       
       finish_item(jal_item);	 
  endtask 
endclass: jal_seq

//---------------------------------------------------------------------------------


class srli_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(srli_seq)
  
  seq_item srli_item ;
  
  function  new(string name = "srli_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    srli_item = seq_item::type_id::create("srli_item") ;
    start_item(srli_item) ; 
    assert(srli_item.randomize() with {
           srli_item.inst_type == SRLI ;
           srli_item.InstrF[6:0] == imm ;
           srli_item.InstrF[31:25] == 7'b0000000 ;
           srli_item.InstrF[14:12] == 3'b101;})
     else     
       `uvm_error(get_type_name(),"randomization failed in SRLI_sequence")
       
       finish_item(srli_item);	 
  endtask
  
endclass: srli_seq

//---------------------------------------------------------------------------------

class srai_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(srai_seq)
  
  seq_item srai_item ;
  
  function  new(string name = "srai_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    srai_item = seq_item::type_id::create("srai_item") ;
    start_item(srai_item) ; 
    assert(srai_item.randomize() with {
           srai_item.inst_type == SRAI ;
           srai_item.InstrF[6:0] == imm ;
           srai_item.InstrF[31:25] == 7'b0100000 ;
           srai_item.InstrF[14:12] == 3'b101;})
     else     
       `uvm_error(get_type_name(),"randomization failed in SRAI_sequence")
       
       finish_item(srai_item);	 
  endtask
  
endclass: srai_seq

//---------------------------------------------------------------------------------


class ori_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(ori_seq)
  
  seq_item ori_item ;
  
  function  new(string name = "ori_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    ori_item = seq_item::type_id::create("ori_item") ;
    start_item(ori_item) ; 
    assert(ori_item.randomize() with {
           ori_item.inst_type == ORI ;
           ori_item.InstrF[6:0] == imm ;
           ori_item.InstrF[14:12] == 3'b110;})
     else     
       `uvm_error(get_type_name(),"randomization failed in ORI_sequence")
       
       finish_item(ori_item);	 
  endtask
  
endclass: ori_seq

//---------------------------------------------------------------------------------

class andi_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(andi_seq)
  
  seq_item andi_item ;
  
  function  new(string name = "andi_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    andi_item = seq_item::type_id::create("andi_item") ;
    start_item(andi_item) ; 
    assert(andi_item.randomize() with {
           andi_item.inst_type == ANDI ;
           andi_item.InstrF[6:0] == imm ;
           andi_item.InstrF[14:12] == 3'b111;})
     else     
       `uvm_error(get_type_name(),"randomization failed in ANDI_sequence")
       
       finish_item(andi_item);	 
  endtask
  
endclass: andi_seq

//---------------------------------------------------------------------------------

class add_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(add_seq)
  
  seq_item add_item ;
  
  function  new(string name = "add_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    add_item = seq_item::type_id::create("add_item") ;
    start_item(add_item) ; 
    assert(add_item.randomize() with {
           add_item.inst_type == ADD ;
           add_item.InstrF[6:0] == arith ;
           add_item.InstrF[31:25] == 7'b0000000 ;
           add_item.InstrF[14:12] == 3'b000;})
     else     
       `uvm_error(get_type_name(),"randomization failed in ADD_sequence")
       
       finish_item(add_item);	 
  endtask
  
endclass: add_seq

//---------------------------------------------------------------------------------

class sub_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(sub_seq)
  
  seq_item sub_item ;
  
  function  new(string name = "sub_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    sub_item = seq_item::type_id::create("sub_item") ;
    start_item(sub_item) ; 
    assert(sub_item.randomize() with {
           sub_item.inst_type == SUB ;
           sub_item.InstrF[6:0] == arith ;
           sub_item.InstrF[31:25] == 7'b0100000 ;
           sub_item.InstrF[14:12] == 3'b000;})
     else     
       `uvm_error(get_type_name(),"randomization failed in SUB_sequence")
       
       finish_item(sub_item);	 
  endtask
  
endclass: sub_seq

//---------------------------------------------------------------------------------

class sll_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(sll_seq)
  
  seq_item sll_item ;
  
  function  new(string name = "sll_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    sll_item = seq_item::type_id::create("sll_item") ;
    start_item(sll_item) ; 
    assert(sll_item.randomize() with {
           sll_item.inst_type == SLL ;
           sll_item.InstrF[6:0] == arith ;         
           sll_item.InstrF[31:25] == 7'b0000000 ;
      sll_item.InstrF[11:7] != 0 ; 
      sll_item.InstrF[19:15] != 0 ; 
      sll_item.InstrF[24:20] != 0 ; 
           sll_item.InstrF[14:12] == 3'b001;})
     else     
       `uvm_error(get_type_name(),"randomization failed in SLL_sequence")
       
       finish_item(sll_item);	 
  endtask
  
endclass: sll_seq

//---------------------------------------------------------------------------------

class slt_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(slt_seq)
  
  seq_item slt_item ;
  
  function  new(string name = "slt_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    slt_item = seq_item::type_id::create("slt_item") ;
    start_item(slt_item) ; 
    assert(slt_item.randomize() with {
           slt_item.inst_type == SLT ;
           slt_item.InstrF[6:0] == arith ;
           slt_item.InstrF[31:25] == 7'b0000000 ;
           slt_item.InstrF[14:12] == 3'b010;})
     else     
       `uvm_error(get_type_name(),"randomization failed in SLT_sequence")
       
       finish_item(slt_item);	 
  endtask
  
endclass: slt_seq

//---------------------------------------------------------------------------------

class xor_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(xor_seq)
  
  seq_item xor_item ;
  
  function  new(string name = "xor_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    xor_item = seq_item::type_id::create("xor_item") ;
    start_item(xor_item) ; 
    assert(xor_item.randomize() with {
           xor_item.inst_type == XOR ;
           xor_item.InstrF[6:0] == arith ;
           xor_item.InstrF[31:25] == 7'b0000000 ;
           xor_item.InstrF[14:12] == 3'b100;})
     else     
       `uvm_error(get_type_name(),"randomization failed in XOR_sequence")
       
       finish_item(xor_item);	 
  endtask
  
endclass: xor_seq

//---------------------------------------------------------------------------------

class srl_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(srl_seq)
  
  seq_item srl_item ;
  
  function  new(string name = "srl_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    srl_item = seq_item::type_id::create("srl_item") ;
    start_item(srl_item) ; 
    assert(srl_item.randomize() with {
           srl_item.inst_type == SRL ;
           srl_item.InstrF[6:0] == arith ;
           srl_item.InstrF[31:25] == 7'b0000000 ;
           srl_item.InstrF[14:12] == 3'b101;})
     else     
       `uvm_error(get_type_name(),"randomization failed in SRL_sequence")
       
       finish_item(srl_item);	 
  endtask
  
endclass: srl_seq

//---------------------------------------------------------------------------------

class sra_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(sra_seq)
  
  seq_item sra_item ;
  
  function  new(string name = "sra_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    sra_item = seq_item::type_id::create("sra_item") ;
    start_item(sra_item) ; 
    assert(sra_item.randomize() with {
           sra_item.inst_type == SRA ;
           sra_item.InstrF[6:0] == arith ;
           sra_item.InstrF[31:25] == 7'b0100000 ;
           sra_item.InstrF[14:12] == 3'b101;})
     else     
       `uvm_error(get_type_name(),"randomization failed in SRA_sequence")
       
       finish_item(sra_item);	 
  endtask
  
endclass: sra_seq

//---------------------------------------------------------------------------------

class or_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(or_seq)
  
  seq_item or_item ;
  
  function  new(string name = "or_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    or_item = seq_item::type_id::create("or_item") ;
    start_item(or_item) ; 
    assert(or_item.randomize() with {
           or_item.inst_type == OR ;
           or_item.InstrF[6:0] == arith ;
           or_item.InstrF[31:25] == 7'b0000000 ;
           or_item.InstrF[14:12] == 3'b110;})
     else     
       `uvm_error(get_type_name(),"randomization failed in OR_sequence")
       
       finish_item(or_item);	 
  endtask
  
endclass: or_seq

//---------------------------------------------------------------------------------

class and_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(and_seq)
  
  seq_item and_item ;
  
  function  new(string name = "and_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    and_item = seq_item::type_id::create("and_item") ;
    start_item(and_item) ; 
    assert(and_item.randomize() with {
           and_item.inst_type == AND ;
           and_item.InstrF[6:0] == arith ;
           and_item.InstrF[31:25] == 7'b0000000 ;
           and_item.InstrF[14:12] == 3'b111;})
     else     
       `uvm_error(get_type_name(),"randomization failed in AND_sequence")
       
       finish_item(and_item);	 
  endtask
  
endclass: and_seq


//---------------------------------------------------------------------------------

class auipc_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(auipc_seq)
  
  seq_item auipc_item ;
  
  function  new(string name = "and_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    auipc_item = seq_item::type_id::create("auipc_item") ;
    start_item(auipc_item) ; 
    assert(auipc_item.randomize() with {
           auipc_item.inst_type == AUIPC ;
           auipc_item.InstrF[6:0] == auipc ;})
     else     
       `uvm_error(get_type_name(),"randomization failed in AUIPC_sequence")
       
       finish_item(auipc_item);	 
  endtask
  
endclass: auipc_seq


//---------------------------------------------------------------------------------

class lui_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(lui_seq)
  
  seq_item lui_item ;
  
  function  new(string name = "lui_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    lui_item = seq_item::type_id::create("lui_item") ;
    start_item(lui_item) ; 
    assert(lui_item.randomize() with {
           lui_item.inst_type == LUI ;
           lui_item.InstrF[6:0] == lui ;})
     else     
       `uvm_error(get_type_name(),"randomization failed in LUI_sequence")
       
       finish_item(lui_item);	 
  endtask
  
endclass: lui_seq

//---------------------------------------------------------------------------------

class beq_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(beq_seq)
  
  seq_item beq_item ;
  
  function  new(string name = "beq_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    beq_item = seq_item::type_id::create("beq_item") ;
    start_item(beq_item) ; 
    assert(beq_item.randomize() with {
           beq_item.inst_type == BEQ ; 
           beq_item.InstrF[6:0] == brnch ;
      beq_item.InstrF[19:15] == 6 ; 
      beq_item.InstrF[24:20] == 2 ;
           beq_item.InstrF[14:12] == 3'b000;})
     else     
       `uvm_error(get_type_name(),"randomization failed in BEQ_sequence")
       
       finish_item(beq_item);	 
  endtask  
endclass: beq_seq

//---------------------------------------------------------------------------------

class bne_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(bne_seq)
  
  seq_item bne_item ;
  
  function  new(string name = "bne_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    bne_item = seq_item::type_id::create("bne_item") ;
    start_item(bne_item) ; 
    assert(bne_item.randomize() with { 
           bne_item.inst_type == BNE ;
           bne_item.InstrF[6:0] == brnch ;
      bne_item.InstrF[19:15] == 6 ; 
      bne_item.InstrF[24:20] == 2 ;
           bne_item.InstrF[14:12] == 3'b001;})
     else     
       `uvm_error(get_type_name(),"randomization failed in BNE_sequence")
       
       finish_item(bne_item);	 
  endtask  
endclass: bne_seq

//---------------------------------------------------------------------------------

class blt_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(blt_seq)
  
  seq_item blt_item ;
  
  function  new(string name = "blt_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    blt_item = seq_item::type_id::create("blt_item") ;
    start_item(blt_item) ; 
    assert(blt_item.randomize() with {
           blt_item.inst_type == BLT ;
           blt_item.InstrF[6:0] == brnch ;
           blt_item.InstrF[14:12] == 3'b100;})
     else     
       `uvm_error(get_type_name(),"randomization failed in BLT_sequence")
       
       finish_item(blt_item);	 
  endtask  
endclass: blt_seq

//---------------------------------------------------------------------------------

class bge_seq extends uvm_sequence #(seq_item);
  `uvm_object_utils(bge_seq)
  
  seq_item bge_item ;
  
  function  new(string name = "bge_seq");
    super.new(name); 
  endfunction: new
  
  task body() ; 	
    bge_item = seq_item::type_id::create("bge_item") ;
    start_item(bge_item) ; 
    assert(bge_item.randomize() with {
           bge_item.inst_type == BGE ;
           bge_item.InstrF[6:0] == brnch ;
           bge_item.InstrF[14:12] == 3'b101;})
     else     
       `uvm_error(get_type_name(),"randomization failed in BGE_sequence")
       
       finish_item(bge_item);	 
  endtask  
endclass: bge_seq

-------------------------------------------------------------------------------
class scoreboard extends uvm_scoreboard ;
  `uvm_component_utils(scoreboard)

  uvm_analysis_imp #(seq_item, scoreboard) sb_mon_port; // Connect Monitor to scoreboard
  
  // Handle Declaration 
  seq_item sb_item ; // Copy the recieved item from monitor here to avoid override elsewhere 
  seq_item p_item ; // Predicted sequence item 
  seq_item fetch_seq, dec_seq, ex_seq, mem_seq, wr_seq  ; // For Instruction pipelinin
  ral_model ral_model_h ; // ral handle 
  uvm_status_e   status ; // needed in ral read task 
  
  // Signal Declaration 
  logic [31:0] mem_forward , wr_forward ; // store the memory and write back stages for later forwarding
  logic [31:0] srcA , srcB ; // holders for data read from RAL 
  logic [31:0] mem_data , reg_data ; // holds the data read from memory and regfile for the previous instruction
  logic [31:0] branch_result ; // used to check the msb of the branch operations result
  logic [31:0] regfile_comp ; // used to hold the expected reg valure either in lw or other instructions
  logic [31:0] jal_rd ;// holds the expected value for jal rd 
  bit   [1:0]  forward_A , forward_B ; // For Read after write Hazard
  bit          lwstall ; // For Load word hazard 
  bit          beqflush ;  // For branch control hazard 
  
  // Constructor
  function new (string name = "scoreboard", uvm_component parent);
    super.new(name, parent);
    sb_mon_port = new ("sb_mon_port" ,this);
  endfunction : new

  // Build Phase (get ral model and create pipeline stages)
  function void build_phase(uvm_phase phase);
    super.build_phase(phase) ;  
    
    if (!uvm_config_db #(ral_model)::get(null, "", "ral_model_h", ral_model_h ))
    `uvm_error(get_type_name(), "RAL model not found" );  
    
    fetch_seq = seq_item::type_id::create("fetch_seq");
    dec_seq = seq_item::type_id::create("dec_seq");
    ex_seq = seq_item::type_id::create("ex_seq");
    mem_seq = seq_item::type_id::create("mem_seq");
    wr_seq = seq_item::type_id::create("wr_seq");
  endfunction : build_phase
  
  task run_phase(uvm_phase phase);
    super.run_phase(phase);
    ral_model_h.initialize; // reset regfile and memory
  endtask
  
  // Write Function 
  function void write(seq_item t) ;  
    sb_item = seq_item::type_id::create("sb_item"); 
    p_item = seq_item::type_id::create("p_item");
    sb_item.copy(t) ;       // To prevent unwanted change  
    if (sb_item.reset) begin 
      fetch_seq.Reset();
      dec_seq.Reset();
      ex_seq.Reset();
      mem_seq.Reset();
      wr_seq.Reset();
      p_item.Reset() ;
      compare(sb_item,p_item) ;
    end     
    
    else begin
      
      // Instruction pipelining
      wr_seq.copy(mem_seq) ;      
      if(ex_seq.lwstall == 0) 
        mem_seq.copy(ex_seq) ; 
      else 
      mem_seq.lwstall = ex_seq.lwstall ;
      ex_seq.copy(dec_seq) ; 
      dec_seq.copy(fetch_seq) ; 
      fetch_seq.copy(sb_item) ;

      // Forward Hazard 
      if ((ex_seq.InstrF[19:15] == mem_seq.InstrF[11:7]) && (ex_seq.InstrF[19:15] != 0) && (mem_seq.InstrF[6:0] != brnch) && (mem_seq.InstrF[6:0] != sw)) 
      forward_A = 2'b10 ;  // Get Source A from memory stage 
      else if ((ex_seq.InstrF[19:15] == wr_seq.InstrF[11:7]) && (ex_seq.InstrF[19:15] != 0) && (wr_seq.InstrF[6:0] != brnch) && (wr_seq.InstrF[6:0] != sw))
      forward_A = 2'b01 ;  // Get Source A from write back stage 
      else 
      forward_A = 2'b00 ;  
      
      if ((ex_seq.InstrF[24:20] == mem_seq.InstrF[11:7]) && (ex_seq.InstrF[24:20] != 0) && (mem_seq.InstrF[6:0] != brnch) && (mem_seq.InstrF[6:0] != sw)) 
      forward_B = 2'b10 ;   // Get Source B from memory stage
      else if ((ex_seq.InstrF[24:20] == wr_seq.InstrF[11:7]) && (ex_seq.InstrF[24:20] != 0) && (wr_seq.InstrF[6:0] != brnch) && (wr_seq.InstrF[6:0] != sw))
      forward_B = 2'b01 ;   // Get Source B from write back stage 
      else 
      forward_B = 2'b00 ; 
      `uvm_info("FORWARD_CHECK", $sformatf("forward_A: %d, forward_B: %d", forward_A, forward_B), UVM_HIGH)
        
     
      // Predict and Compare 
       fork 
         begin
         predict();  
         compare(sb_item,p_item) ;
         end
       join_none         
    end
    
  endfunction: write 
  
  task predict ();    
    p_item = seq_item::type_id::create("p_item");
    ral_model_h.regs[mem_seq.InstrF[19:15]].read(status, srcA, UVM_BACKDOOR) ; // Read rs1 
    ral_model_h.regs[mem_seq.InstrF[24:20]].read(status, srcB, UVM_BACKDOOR) ; // Read rs2 
    ral_model_h.regs[wr_seq.InstrF[11:7]].read(status, reg_data, UVM_BACKDOOR) ;   // Read reg (rd) for previous instruction    
    ral_model_h.dmem.read(status , fetch_seq.DataAdrM[9:2] , mem_data, UVM_BACKDOOR); // Read memory for current instruction   
    branch_result = srcA - srcB ;
     
    
     // Branch Hazard
    dec_seq.beqflush = (mem_seq.beqflush || ex_seq.beqflush || ex_seq.lwstall) ? 0 : ((mem_seq.inst_type == BEQ) && (!branch_result) || (mem_seq.inst_type == BNE) && (branch_result) || (mem_seq.inst_type == BGE) && (!branch_result[31]) || (mem_seq.inst_type == BLT) && (branch_result[31]) || (mem_seq.inst_type == JAL) || (mem_seq.inst_type == JALR)) ;
    `uvm_info("BEQFLUSH_CHECK", $sformatf("beqflush: %b", dec_seq.beqflush), UVM_HIGH)
    `uvm_info("OPERANDS_VALUES", $sformatf("srcA: %d, srcB: %d", srcA, srcB), UVM_HIGH)

     
    // LW stall Hazard 
    fetch_seq.lwstall = (dec_seq.beqflush || ex_seq.beqflush || dec_seq.lwstall) ? 0 : (ex_seq.InstrF[6:0] == lw) && ((ex_seq.InstrF[11:7] == dec_seq.InstrF[19:15]) || (ex_seq.InstrF[11:7] == dec_seq.InstrF[24:20])) ;
    `uvm_info("LWSTALL_CHECK", $sformatf("lwstall: %b", fetch_seq.lwstall), UVM_HIGH)
 
    
    
    case (mem_seq.inst_type) // Instruction Type 
       
      LW : begin 
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA + mem_seq.Extend ; 
      end
           
      ADDI : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA + mem_seq.Extend ;
      end
          
      SLLI : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA << mem_seq.InstrF[24:20] ;       
      end
          
      SLTI : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : ($signed(srcA) < $signed(mem_seq.Extend)) ;
      end
          
      XORI : begin 
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA ^ mem_seq.Extend ;
      end
          
      SRAI : begin 
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA >>> mem_seq.InstrF[24:20] ;
      end
            
      SRLI : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA >> mem_seq.InstrF[24:20] ;  
      end 
          
      ORI : begin 
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA | mem_seq.Extend ;
      end
          
      ANDI : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA & mem_seq.Extend ;
      end 
          
      AUIPC : begin 
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ; 
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : mem_seq.PCF + {mem_seq.InstrF[31:12] , 12'b0} ; 
      end
      
      SW : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ; 
        p_item.MemWriteM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : 1 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA + mem_seq.Extend ; 
        p_item.WriteDataM = srcB ;
      end
           
      ADD : begin
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA + srcB ; 
      end
          
      SUB : begin 
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA - srcB ; 
      end
             
      SLL : begin   
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA << srcB[4:0] ;   
      end
          
      SLT : begin 
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : ($signed(srcA) < $signed(srcB)) ;   
      end
          
      XOR : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA ^ srcB ; 
      end  
          
      SRL : begin 
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA >> srcB[4:0] ; 
      end  
      
      SRA : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA >>> srcB[4:0] ; 
      end 

      OR : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA | srcB ;    
      end  
        
      AND : begin  
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ;
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA & srcB ;  
      end  
      
      LUI : begin 
        p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : dec_seq.PCF + 4 ; 
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : {mem_seq.InstrF[31:12] , 12'b0} ; 
      end
      
      BEQ : begin 
        p_item.MemWriteM = 0 ; 
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA - srcB ; 
        if (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall)
          p_item.PCF = dec_seq.PCF + 4 ; 
        else 
          p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : p_item.DataAdrM ? (dec_seq.PCF + 4) : (mem_seq.PCF + mem_seq.Extend);
      end
        
      BNE : begin 
        p_item.MemWriteM = 0 ; 
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA - srcB ;
        if (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall)
          p_item.PCF = dec_seq.PCF + 4 ; 
        else
          p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : p_item.DataAdrM ? (mem_seq.PCF + mem_seq.Extend) : (dec_seq.PCF + 4) ;
      end
        
      BLT : begin  
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA - srcB ;  
        if (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall)
          p_item.PCF = dec_seq.PCF + 4 ; 
        else
          p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : (p_item.DataAdrM[31]) ? mem_seq.PCF + mem_seq.Extend : dec_seq.PCF + 4;
      end
        
      BGE : begin  
        p_item.MemWriteM = 0 ;
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA - srcB ;
        if (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall)
          p_item.PCF = dec_seq.PCF + 4 ; 
        else
          p_item.PCF = dec_seq.lwstall ? dec_seq.PCF : (!p_item.DataAdrM[31]) ? mem_seq.PCF + mem_seq.Extend : dec_seq.PCF + 4;  
      end      
        
      JALR : begin 
        p_item.MemWriteM = 0 ; 
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA + mem_seq.Extend ;
        p_item.PCF = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? dec_seq.PCF +4 : dec_seq.lwstall ? dec_seq.PCF : srcA + mem_seq.Extend ;
        jal_rd = mem_seq.PCF + 4 ;
      end
        
      JAL : begin 
        p_item.MemWriteM = 0 ; 
        p_item.DataAdrM = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? 0 : srcA + srcB ;
        p_item.PCF = (ex_seq.beqflush || mem_seq.beqflush || ex_seq.lwstall) ? dec_seq.PCF +4 : dec_seq.lwstall ? dec_seq.PCF : mem_seq.PCF + mem_seq.Extend ;
        jal_rd = mem_seq.PCF + 4 ;
      end
      
    endcase  // Instruction type  
  endtask // predict 
  
    
  
  function void compare (seq_item a_seq , p_seq) ;
    bit some_pass , reg_pass , mem_pass , all_pass ; 
     
    if(mem_seq.InstrF == 0)begin 
      `uvm_info(get_type_name(),"No Output yet ... In progress", UVM_HIGH)
    end
       
    else begin 
      
    if (wr_seq.InstrF[6:0] == lw)  
     regfile_comp = dec_seq.ReadDataM ;  
    else if ((wr_seq.inst_type == JAL) || (wr_seq.inst_type == JALR))
     regfile_comp = ((mem_seq.inst_type == JAL) || (mem_seq.inst_type == JALR)) ? jal_rd - 4 : jal_rd ; 
    else 
     regfile_comp = dec_seq.DataAdrM ;
    
      
      
    mem_pass = (mem_data == fetch_seq.WriteDataM) ;  
    reg_pass = (reg_data == regfile_comp) ;    
    some_pass = (a_seq.DataAdrM   === p_seq.DataAdrM)  &&
                (a_seq.MemWriteM  === p_seq.MemWriteM) &&
                (a_seq.PCF        === p_seq.PCF) ; 
      
      
    // Memory only 
    if ((mem_seq.InstrF[6:0] == sw) && ((wr_seq.InstrF[6:0] == sw) || (wr_seq.InstrF[6:0] == brnch) || wr_seq.beqflush) && !ex_seq.beqflush && !mem_seq.beqflush) begin 
      all_pass = some_pass && mem_pass ;
      `uvm_info("MEMORY_CHECK", $sformatf("Data read from memory: %h, Expected data: %h", mem_data, fetch_seq.WriteDataM), UVM_HIGH) 
    end 
    // Memory and Regfile
    else if ((mem_seq.InstrF[6:0] == sw) && (wr_seq.InstrF[6:0] != sw) && (wr_seq.InstrF[6:0] != brnch) && !ex_seq.beqflush && !mem_seq.beqflush && !ex_seq.lwstall)  begin 
      all_pass = some_pass && mem_pass && reg_pass;
      `uvm_info("MEMORY_REGFILE_CHECK", $sformatf("Data read from memory: %h, Expected data: %h\nData read from regfile: %h, Expected data: %h",mem_data, fetch_seq.WriteDataM, reg_data, regfile_comp),UVM_HIGH)
    end
    // neither memory nor regfile 
    else if (((mem_seq.InstrF[6:0] != sw) && ((wr_seq.InstrF[6:0] == sw) || (wr_seq.InstrF[6:0] == brnch)))  || mem_seq.lwstall || mem_seq.beqflush || wr_seq.beqflush || (ex_seq.beqflush && (wr_seq.InstrF[6:0] == brnch))) begin 
      all_pass = some_pass ; 
    end
    // regfile 
    else begin 
      all_pass = some_pass && reg_pass ;
      `uvm_info("REGFILE_CHECK", $sformatf("Data read from regfile: %h, Expected data: %h", reg_data, regfile_comp), UVM_HIGH)
    end
     
     
    if (all_pass)
      `uvm_info(get_type_name(),"SUCCESSFUL OPERATION ", UVM_HIGH)
    else  
      `uvm_error(get_type_name(),"FAILED OPEARTION ")

     
    //---------------------------------------- Debug -----------------------------------\\   
         // Display Results 
//       $display("Forward_A : %h   ,   Forward_B : %h",forward_A , forward_B) ;
//       $display("Current Instrution Type : %s", mem_seq.inst_type.name) ;
//       $display("Previous Instruction Type : %s", wr_seq.inst_type.name) ;
//       $display("==========================================================================") ;
//       $display("|    POC   | DataAdrM |    PCF   | MemWriteM | RegFile data |  Mem data  |") ; 
//       $display("==========================================================================") ;
//       $display("| Expected | %h | %h |     %h     |   %h   | %h |", p_seq.DataAdrM, p_seq.PCF, p_seq.MemWriteM, regfile_comp, fetch_seq.WriteDataM) ;
//       $display("|  Actual  | %h | %h |     %h     |   %h   | %h |", a_seq.DataAdrM, a_seq.PCF, a_seq.MemWriteM, reg_data, mem_data) ;
//       $display("==========================================================================") ;
//       $display("") ;
//       $display("") ;
//       $display("") ;
    end 
  endfunction 
  

  
endclass : scoreboard
      ------------------------------------------------------------------------------------------------------------------
package risc_pkg;

     localparam [6:0] lw    = 7'b0000011 ,
	              imm   = 7'b0010011 ,
		      auipc = 7'b0010111 ,
		      sw    = 7'b0100011 ,
		      arith = 7'b0110011 ,
		      lui   = 7'b0110111 ,
	              brnch = 7'b1100011 ,
		      jalr  = 7'b1100111 , 
		      jal   = 7'b1101111 ;
 
					  
    localparam [2:0] add = 3'b000 ,
	             sub = 3'b000 ,
                     sll = 3'b001 ,
                     slt = 3'b010 ,
                     Xor = 3'b100 , 
                     srl = 3'b101 , 
	             sra = 3'b101 ,
                     Or  = 3'b110 ,
                     And = 3'b111 ;


     localparam [2:0] beq = 3'b000 , 
                      bne = 3'b001 ,
                      blt = 3'b100 ,
                      bge = 3'b101 ;


     typedef enum logic [5:0] {LW,SW,
                               ADDI,SLLI,SLTI,XORI,SRLI,SRAI,ORI,ANDI,
                               AUIPC,LUI,
                               ADD,SUB,SLL,SLT,XOR,SRL,SRA,OR,AND,
                               BEQ,BNE,BLT,BGE,
                               JALR,JAL,
                               RESET, UNKNOWN} instr_type;


endpackage 
-------------------------------------------------------------------------------------------------------------------
import uvm_pkg::*;
class reg32 extends uvm_reg; 
  `uvm_object_utils(reg32) 

  uvm_reg_field field;  // register is one field


  function new(string name = "reg32");
    super.new(name, 32, UVM_NO_COVERAGE); // constructor (name , size , coverage)
  endfunction


  virtual function void build();
    field = uvm_reg_field::type_id::create("field"); 

    //function void configure( uvm_reg 	parent,
    // int 	unsigned 	size,
    // int 	unsigned 	lsb_pos,
    // string 	access,
    // bit 	volatile,
    // uvm_reg_data_t 	reset,
    // bit 	has_reset,
    // bit 	is_rand,
    // bit 	individually_accessible	)

    field.configure(this, 32, 0 , "RW", 0, 32'b0, 1, 1, 1);

  endfunction
endclass

////////////////////////////////////////////////////////////////////////////

class data_mem extends uvm_mem;
  `uvm_object_utils(data_mem)
  function new(string name = "data_mem");
    super.new(name, // Name of the memory
              256,  // Number of words (1 KB = 256 words of 4 bytes each)
              32,   // Data width (32 bits)
              "RW", // Access rights
              UVM_NO_COVERAGE); // Coverage (optional)
  endfunction
endclass


////////////////////////////////////////////////////////////////////////////


class ral_model extends uvm_reg_block;
  `uvm_object_utils(ral_model)

  reg32 regs[32];
  data_mem dmem;
  uvm_reg_map map;

  string blk_hdl_path;
  string mem_hdl_path;
  string reg_hdl_path;

  function new(string name = "ral_model");
    super.new(name, UVM_NO_COVERAGE);
  endfunction

  virtual function void build();
    


    uvm_config_db #(string)::get(null, "", "blk_hdl_path", blk_hdl_path);
    uvm_config_db #(string)::get(null, "", "mem_hdl_path", mem_hdl_path);
    uvm_config_db #(string)::get(null, "", "reg_hdl_path", reg_hdl_path);

    add_hdl_path(blk_hdl_path); // HDL path for reg block
    map = create_map("map",'h0, 4, UVM_BIG_ENDIAN, 0);

    dmem = data_mem::type_id::create("dmem"); // Create mem object
    dmem.configure(this, mem_hdl_path); // Parent , HDL path
    map.add_mem( dmem, 0, "RW");

    // create map object
    // Map name, Offset, Number of bytes, Endianess, byte addressing) 


    // Create 32 registers
    foreach (regs[i]) begin
      regs[i] = reg32::type_id::create($sformatf("x%0d", i));
      regs[i].configure(this, null,$sformatf(reg_hdl_path,i) );
      regs[i].build();

      // add reg to map
      // reg, offset, access
      if (i == 0)
        map.add_reg(regs[i], i+500, "RO");  // Read-only access for x0
      else
        map.add_reg(regs[i], i+500, "RW");  // Read-write access

    end
  endfunction
  
  task initialize;
    uvm_status_e status;
    for (int i=0; i <256; i++) this.dmem.write(status, i, 0, UVM_BACKDOOR);
    for (int i=0; i <32; i++ ) this.regs[i].write(status, 0, UVM_BACKDOOR);
  endtask
 

endclass
--------------------------------------------------------------------------------------------------------------------
class monitor extends uvm_monitor ;
  `uvm_component_utils(monitor)


  virtual intf monitor_intf;
  uvm_analysis_port #(seq_item) monitor_ap;
  seq_item monitor_item;

  // Constructor 
  function new(string name = "monitor" ,uvm_component parent);
    super.new(name,parent);
  endfunction :new


  // Build Phase 
  function void build_phase(uvm_phase phase);
    if(!uvm_config_db #(virtual intf)::get(this, "*","risc_intf", monitor_intf))
      `uvm_fatal("MONITOR", "Failed to get Interface");
    
    
    monitor_ap  = new("monitor_ap",this);
  endfunction : build_phase


  // Run Phase 
  task  run_phase(uvm_phase phase);
    super.run_phase(phase);
	
    forever begin 
         monitor_item = seq_item::type_id::create("monitor_item");	 
         @(posedge monitor_intf.clk) ; 	
         monitor_item.reset      = monitor_intf.reset      ;
         monitor_item.InstrF     = monitor_intf.InstrF     ;
         monitor_item.inst_type  = monitor_intf.inst_type  ;
         monitor_item.PCF        = monitor_intf.PCF        ;
         monitor_item.WriteDataM = monitor_intf.WriteDataM ;
         monitor_item.DataAdrM   = monitor_intf.DataAdrM   ;
         monitor_item.MemWriteM  = monitor_intf.MemWriteM  ;	
         monitor_item.ReadDataM  = monitor_intf.ReadDataM  ;
      #1 ;
         monitor_ap.write(monitor_item) ;
    end 
    
  endtask : run_phase  


endclass 
------------------------------------------------------------------------------------------------------------  
interface intf (input logic clk) ; 
  
  import risc_pkg::* ;
  
  logic        reset      ; 
  logic [31:0] InstrF     ;
  logic [31:0] PCF        ;
  logic [31:0] WriteDataM ;
  logic [31:0] DataAdrM   ;
  logic [31:0] ReadDataM  ;
  logic        MemWriteM  ; 
  instr_type   inst_type  ;
  

endinterface 
-----------------------------------------------------------------------------------------------------------
class env extends uvm_env;
  `uvm_component_utils(env)
  
  
  agent risc_agent ;
  scoreboard risc_scoreboard ;
  ral_model ral_model_h;
  coverage risc_coverage ;
 
  
  
  // Constructor 
    function new(string name = "env" ,uvm_component parent);
         super.new(name,parent); 
    endfunction : new
              
              
  
  // Build Phase 
    function void build_phase(uvm_phase phase);
     super.build_phase(phase); 
           
      ral_model_h   = ral_model::type_id::create("ral_model_h"); 
      ral_model_h.build();
      ral_model_h.lock_model();
      ral_model_h.reset(); 
      uvm_config_db#(ral_model)::set(null, "*", "ral_model_h", ral_model_h);
      
      risc_agent = agent::type_id::create("risc_agent",this);  
      risc_scoreboard = scoreboard::type_id::create("risc_scoreboard",this);  
      risc_coverage = coverage::type_id::create("risc_coverage",this);     
    endfunction :build_phase    
              
  
  
    
  // Connect Phase 
    function void connect_phase (uvm_phase phase);
      super.connect_phase(phase); 
      risc_agent.risc_monitor.monitor_ap.connect(risc_scoreboard.sb_mon_port) ; // Connect monitor to scoreboard by analysis port              
      risc_agent.risc_monitor.monitor_ap.connect(risc_coverage.cov_mon_port) ; // Connect monitor to Coverage by analysis port        
  endfunction :connect_phase
  
  // Run Phase 
    task run_phase(uvm_phase phase);
      super.run_phase(phase);
    endtask : run_phase
  
endclass : env
------------------------------------------------------------------------------------------------------------------
 class driver extends uvm_driver #(seq_item);
  `uvm_component_utils(driver)

  seq_item driver_item ;
  virtual intf driver_intf ;
  
  
 // Constructor  
  function new(string name = "driver" ,uvm_component parent);
    super.new(name,parent);
  endfunction :new
 
    
 // Build Phase 
  function void build_phase(uvm_phase phase);  
    super.build_phase(phase);
    if(!(uvm_config_db #(virtual intf)::get(this,"*","risc_intf",driver_intf))) 
   `uvm_error(get_type_name(),"failed to get virtual interface inside Driver class")   
      
      
  endfunction :build_phase 
      
      
 // Run Phase 
  task run_phase (uvm_phase phase);   
    super.run_phase(phase);  
         forever begin      
           driver_item = seq_item::type_id::create("driver_item");     
           seq_item_port.get_next_item(driver_item) ;
           drive(driver_item) ;
           seq_item_port.item_done() ;    
         end   
  endtask : run_phase 
            
      
 // drive task 
      task drive (seq_item RISC_item) ;
        @(posedge driver_intf.clk) ;
		 driver_intf.reset  <= RISC_item.reset  ;
	     driver_intf.InstrF <= RISC_item.InstrF ;
         driver_intf.inst_type <= RISC_item.inst_type ;
      endtask : drive
 
  
endclass : driver
-----------------------------------------------------------------------------------------------------------------
class coverage extends uvm_subscriber #(seq_item) ;
  `uvm_component_utils(coverage)
  
  uvm_analysis_imp #(seq_item, coverage) cov_mon_port; // Connect Monitor to scoreboard

  // Signals Declaration
   instr_type instruction ;
   logic [31:0] PC, DataAdr, WriteData, ReadData ;
   logic [4:0] rs1, rs2, rd ;
   logic       MemWrite ;
   logic       reset ;
  
  
  // Handles declaration 
  seq_item cov_item ; // Copy the recieved item from monitor here to avoid override elsewhere 
  
  
  
  // Covergroups 
  covergroup Instruction_cg ; // To Cover All Instructions and crosses among them
  Instructions_cp :       coverpoint instruction { 
                          bins I_Type[] = {LW, ADDI, SLLI, SLTI, XORI, SRLI, SRAI, ORI, ANDI, JALR} ;
                          bins R_Type[] = {ADD, SUB, SLL, SLT, XOR, SRL, SRA, OR, AND} ;
                          bins B_Type[] = {BEQ, BNE, BLT, BGE} ;
                          bins U_Type[] = {LUI, AUIPC} ;
                          bins S_Type[] = {SW} ;
                          bins J_Type[] = {JAL} ;
                          bins  Reset[] = {RESET} ;
                          illegal_bins unknow = {UNKNOWN} ; 
                          } 
 
  Imm_Transitions_cp :    coverpoint instruction {
                          bins I_R[]     = (SLTI, SRLI => SLL, SLT) ;
                          bins I_B[]     = (SLTI => BEQ, BNE, BLT, BGE) ;
                          bins I_SW[]    = (SLTI => SW) ;
                          bins I_SW_SW[] = (SLTI => SW[*2]) ;
                          bins I_LW[]    = (SLTI => LW) ;
                          bins I_LW_LW[] = (SLTI => LW[*2]) ;
                          bins I_J[]     = (ADDI, SLTI => JAL,JALR) ;
                          bins I_J_J[]   = (ADDI, SLTI => JAL[*2]) ;
                          bins I_JR_JR[] = (ADDI, SLTI => JALR[*2]) ;
                          }

  Branch_Transitions_cp : coverpoint instruction {                      
                          bins B_R[]     = (BEQ, BNE, BLT, BGE => ADD, SLT, SRA) ;
                          bins B_B[]     = (BEQ, BNE, BLT, BGE => BEQ, BNE, BLT, BGE) ;
                          bins B_J[]     = (BEQ, BNE, BLT, BGE => JAL,JALR) ;
                          bins B_J_J[]   = (BEQ, BNE, BLT, BGE => JAL) ;
                          bins B_JR_JR[] = (BEQ, BNE, BLT, BGE => JALR[*2]) ;
                          bins B_SW[]    = (BEQ, BNE, BLT, BGE => SW) ;
                          bins B_SW_SW[] = (BEQ, BNE, BLT, BGE => SW[*2]) ;
                          bins B_LW[]    = (BEQ, BNE, BLT, BGE => LW) ;
                          bins B_LW_LW[] = (BEQ, BNE, BLT, BGE => LW[*2]) ;                         
                          }

  Jump_Transitions_cp :   coverpoint instruction {                     
                          bins J_R[]     = (JAL, JALR => SUB, SLT, SRA) ;
                          bins J_J[]     = (JAL, JALR => JAL,JALR) ;
                          bins J_B[]     = (JAL, JALR => BEQ, BNE, BLT, BGE) ;
                          bins J_B_B[]   =  (JAL, JALR => BEQ,BLT[*2]) ;
                          bins J_SW[]    = (JAL, JALR => SW) ;
                          bins J_SW_SW[] = (JAL, JALR => SW[*2]) ;
                          bins J_LW[]    = (JAL, JALR => LW) ;
                          bins J_LW_LW[] = (JAL  => LW[*2]) ;
                          }

  SW_Transitions_cp :    coverpoint instruction {
                         bins SW_B[]   = (SW => BEQ, BNE, BLT, BGE) ;
                         bins SW_J[]   = (SW => JAL, JALR) ;                      
                         bins SW_SW[] = (SW => SW) ;                        
                         bins SW_LW[] = (SW => LW) ;
                         bins SW_I[]  = (SW => SLTI) ;
                         }

  LW_Transitions_cp :    coverpoint instruction {
                         bins LW_B[]  = (LW => BEQ, BNE, BLT, BGE) ;
                         bins LW_J[]  = (LW => JAL, JALR) ;
                         bins LW_LW[] = (LW => LW) ;
                         bins LW_SW[] = (LW => SW) ;
                         }

  endgroup : Instruction_cg 


  
  covergroup Reset_cg ;
    Reset_Transition_cp : coverpoint reset { 
                          bins OFF_ON = (0 => 1) ;
                          bins ON_OFF = (1 => 0) ; 
                          }
  endgroup : Reset_cg

  covergroup Registers_cg ;
    Rs1_cp : coverpoint rs1 {bins Rs1[] = {[0:31]};}
    Rs2_cp : coverpoint rs2 {bins Rs2[] = {[0:31]};}
    Rd_cp  : coverpoint rd  {bins Rd[]  = {[1:31]};} 
  endgroup : Registers_cg

  covergroup Memory_cg ;
    DataAdr_cp        : coverpoint DataAdr[9:2] {bins DataAddress[] = {[8'h00 : 8'hff]};}

    MemWrite_cp       : coverpoint MemWrite {
                        bins write = {1};
                        bins read  = {0};
                        } 
  
    Memory_Address_cp : cross MemWrite_cp, DataAdr_cp {
                        bins write_addresses = binsof(MemWrite_cp.write) && binsof(DataAdr_cp);
                        }
 
  endgroup : Memory_cg
  
  // Constructor 
  function new(string name = "coverage_collector" ,uvm_component parent);
     super.new(name,parent);
     cov_mon_port = new ("cov_mon_port" ,this); 
     Instruction_cg = new() ;
     Reset_cg  = new() ;
     Registers_cg = new() ;
     Memory_cg = new() ;
  endfunction : new
  
  
  
 // Build Phase 
  function void build_phase(uvm_phase phase);  
    super.build_phase(phase);
  endfunction
  
  
  
  function void write (seq_item  t);
     cov_item = seq_item::type_id::create("sb_item"); 
     cov_item.copy(t) ;   // To prevent unwanted change
     Inst_decode(cov_item) ;
    
       
    // Sampling covergroups 
    Instruction_cg.sample() ; 
    Reset_cg.sample() ;
    Registers_cg.sample() ;
    Memory_cg.sample() ;
    
  endfunction : write 

  function void Inst_decode (seq_item cov_item);

    instruction = cov_item.inst_type;
    reset       = cov_item.reset;
    PC          = cov_item.PCF;
    WriteData   = cov_item.WriteDataM;
    DataAdr     = cov_item.DataAdrM;
    ReadData    = cov_item.ReadDataM;
    MemWrite    = cov_item.MemWriteM;

    case (cov_item.InstrF[6:0])
     arith : 
              begin
                rd  = cov_item.InstrF[11:7];
                rs1 = cov_item.InstrF[19:15];
                rs2 = cov_item.InstrF[24:20];
              end

    imm,lw : 
              begin 
                rd    = cov_item.InstrF[11:7];
                rs1   = cov_item.InstrF[19:15];
              end

    sw,brnch : 
             begin
               rs1   = cov_item.InstrF[19:15];
               rs2   = cov_item.InstrF[24:20];
             end

    jal,jalr,lui,auipc:
            begin
              rd    = cov_item.InstrF[11:7];
            end
    endcase
  endfunction
  
  
endclass : coverage
-------------------------------------------------------------------------------------------------------------
`timescale 1ns/1ps

module riscv_pip_tb;

  // Clock and reset signals
  logic clk;
  logic reset;

  // DUT signals
  logic [31:0] PCF;
  logic [31:0] InstrF;
  logic MemWriteM;
  logic [31:0] DataAdrM, WriteDataM , ReadDataM ;

  // Instantiate the DUT
  risc_top DUT (
    .clk(clk),
    .reset(reset),
    .PCF(PCF),
    .InstrF(InstrF),
    .MemWriteM(MemWriteM),
    .DataAdrM(DataAdrM),
    .WriteDataM(WriteDataM),
    .ReadDataM(ReadDataM)
  );

  // Clock generation
  always #5 clk = ~clk;

  // Testbench logic
  initial begin
    // Initialize signals
    clk = 0;
    reset = 1;
    InstrF = 0;
    

    // Reset the DUT
    #15 reset = 0;
    
    // Send ADDI instructions (put 7 in x6)
    InstrF = {12'b000000000111, // immediate = 7
                 5'b00000,    // rs1 = x1 (source register)
                 3'b000,      // funct3 for ADDI
                 5'b00110,    // rd = x6 (destination register)
                 7'b0010011}; // opcode for ADDI
    
      #10; // Wait for a clock cycle
    // Send ADDI instructions (put 3 in x2)
    InstrF = {12'b000000000011, // immediate = 3
                 5'b00000,    // rs1 = x1 (source register)
                 3'b000,      // funct3 for ADDI
                 5'b00010,    // rd = x2 (destination register)
                 7'b0010011}; // opcode for ADDI
    
    
     #10; // Wait for a clock cycle
    // put 5 in x3
    InstrF = {12'b000000000101, // immediate = 5
                 5'b00000,    // rs1 = x1 (source register)
                 3'b000,      // funct3 for ADDI
                 5'b00011,    // rd = x3 (destination register)
                 7'b0010011}; // opcode for ADDI
    
    #10; // Wait for a clock cycle
    
    // Send SW instruction (put 7 in mem_6)
       InstrF = {7'b0000000,    // imm[11:5] (upper 7 bits of immediate = 0)
                 5'b00110,      // rs2 = x2 (source register containing data to store)
                 5'b00110,      // rs1 = x6 (base register for memory address)
                 3'b010,        // funct3 for SW
                 5'b00000,      // imm[4:0] (lower 5 bits of immediate)
                 7'b0100011};   // opcode for SW
                           
                
      #10; // Wait for a clock cycle
    
    // Send LW instruction (read 7 from mem_6 and put it in x4)
       InstrF = {12'b000000000000, // immediate = 0
                 5'b00110,         // rs1 = x6 (base register for memory address)
                 3'b010,           // funct3 for LW
                 5'b00100,         // rd = x4 (destination register to load data)
                 7'b0000011};      // opcode for LW
    
    #10; // Wait for a clock cycle
    
  //  Send ADDI instructions 
       InstrF = {12'b00000000011, // immediate = 3
                 5'b00100,    // rs1 = x4 (source register)
                 3'b000,      // funct3 for ADDI
                 5'b00110,    // rd = x6 (destination register)
                 7'b0010011}; // opcode for ADDI
    
    #10; // Wait for a clock cycle
    
  //  Send ADDI instructions 
       InstrF = {12'b00000000011, // immediate = 3
                 5'b00101,    // rs1 = x5 (source register)
                 3'b000,      // funct3 for ADDI
                 5'b00110,    // rd = x6 (destination register)
                 7'b0010011}; // opcode for ADDI

     
    #10; // Wait for a clock cycle
    
  //  Send ADDI instructions 
       InstrF = {12'b00000000011, // immediate = 3
                 5'b00101,    // rs1 = x5 (source register)
                 3'b000,      // funct3 for ADDI
                 5'b00110,    // rd = x6 (destination register)
                 7'b0010011}; // opcode for ADDI
    
     
    #10; // Wait for a clock cycle
    
  //  Send ADDI instructions 
       InstrF = {12'b00000000011, // immediate = 3
                 5'b00101,    // rs1 = x5 (source register)
                 3'b000,      // funct3 for ADDI
                 5'b00110,    // rd = x6 (destination register)
                 7'b0010011}; // opcode for ADDI
    
    
     #10; // Wait for a clock cycle
    
  //  Send ADDI instructions 
       InstrF = {12'b00000000011, // immediate = 3
                 5'b00101,    // rs1 = x5 (source register)
                 3'b000,      // funct3 for ADDI
                 5'b00110,    // rd = x6 (destination register)
                 7'b0010011}; // opcode for ADDI
    
    #10; // Wait for a clock cycle
    
  //  Send ADDI instructions 
       InstrF = {12'b00000000011, // immediate = 3
                 5'b00101,    // rs1 = x5 (source register)
                 3'b000,      // funct3 for ADDI
                 5'b00110,    // rd = x6 (destination register)
                 7'b0010011}; // opcode for ADDI
    
  $stop ; 
    
  end
  
  initial 
  begin
    // Required to dump signals to EPWave
    $dumpfile("dump.vcd");
    $dumpvars(0);
  end
endmodule
-----------------------------------------------------------------------------------------------------------------
class agent extends uvm_agent ;
  `uvm_component_utils(agent)
  
  
  driver risc_driver ;
  monitor risc_monitor ;
  sequencer risc_sequencer ;
  
  
  // Constructor  
  function new(string name = "agent" ,uvm_component parent);
    super.new(name,parent);
  endfunction :new
  
  
  
  // Build Phase 
  function void build_phase(uvm_phase phase);  
    super.build_phase(phase);
    risc_driver    = driver::type_id::create("risc_driver",this);
    risc_monitor = monitor::type_id::create("risc_monitor",this);
    risc_sequencer = sequencer::type_id::create("risc_sequencer",this); 
  endfunction : build_phase 
  
  
  
 // Connect Phase 
  function void connect_phase (uvm_phase phase);
    super.connect_phase(phase);	  
	 risc_driver.seq_item_port.connect(risc_sequencer.seq_item_export); // connect driver to sequencer
  endfunction :connect_phase
  
  
 // Run Phase 
  task run_phase(uvm_phase phase);
    super.run_phase(phase);
  endtask : run_phase
  
  
endclass : agent 
---------------------------------------------------------------------------------------------------------
                          TEST_CASE NOT UPLODED 7
-------------------------------------------------------------------------------------------------
