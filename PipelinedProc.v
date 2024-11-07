`timescale 1ns / 1ps

//Defining 2:1 muxes
module mux21(input [31:0]zero , input [31:0]one, input select, output [31:0]out);
	assign out = ((select == 0) ? zero : one);
endmodule

module mux21_5bit(input [4:0]zero , input [4:0]one, input [4:0]two, input [1:0]select, output reg [4:0]out);
  always @(*) begin
    case(select)
        2'b00 : out <= zero;
        2'b01 : out <= one;
        2'b10 : out <= two;
        default: out <= zero;
    endcase
  end
endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Defining 3:1 MUX
module mux31(input [31:0]zero , input [31:0]one, input [31:0]two, input [1:0]select, output reg [31:0]out);
	always @(*) begin
		case(select)
			2'b00 : out <= zero;
			2'b01 : out <= one;
			2'b10 : out <= two;
			2'b11 : out <= zero;
		endcase
	end
endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Defining 4:1 MUX
module mux41(input [31:0]zero , input [31:0]one, input [31:0]two, input [31:0]three, input [1:0]select, output reg [31:0]out);
	always @(*) begin
		case(select)
			2'b00 : out <= zero;
			2'b01 : out <= one;
			2'b10 : out <= two;
			2'b11 : out <= three;
		endcase
	end
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Defining sign-extendeing hardware
module sign_extend(input [15:0]in, input signExtend, output reg [31:0]out);
	always @(*) begin
	   if (signExtend == 0) begin
	       out <= 32'h0 + in;
	   end
	   else begin
	       out <= (in[15] ? (32'hffff0000 + in) : (32'h0 + in));
	   end
	end
endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module PipelinedProc(input CLK, Reset_L, 
					input [31:0]startPC, 
					output [31:0]dMemOut, program_counter, branch_target,EX_alu_jal_mux_out, next_counter, IF_ID_Instruction,MEM_ALU_Output, MEM_Datain, MEM_Dataout, ID_Reg_A, EX_Instruction,
					output EX_ALUZero, MEM_MemRead,IF_write, PC_write, bubble, EX_jal, ID_EX_jal, ID_jal,
					output [1:0]addrSel, 
					output [4:0]EX_Rw, EX_MEM_Rw, MEM_Rw
					);

//==================================================================REGISTERS AND WIRE DECLARATION=================================================
	//Stage 1
	reg [31:0]program_counter; //program_counter
	wire [31:0]interim_counter, next_counter; // program_counter + 4
	wire [31:0] IF_instruction;
	 reg [31:0]IF_ID_PC;
     reg [31:0]IF_ID_Instruction;
	
	
	//Stage 2
	wire ID_jal, ID_jr; //new addition
    wire [31:0]interim_jump_address;//new addition
    reg  ID_EX_jal, ID_EX_jr;//new addition
    wire [1:0]ID_RegDst;//new addition
    reg [1:0]ID_EX_RegDst;//new addition
    reg ID_EX_Branch;
	
	wire [31:0] ID_Reg_A, ID_Reg_B, ID_Sign_Extended, jump_address, ID_zero_Extended;
	reg [31:0] ID_EX_Reg_A, ID_EX_Reg_B ;
	reg [1:0]ID_EX_AluOpCtrl_A, ID_EX_AluOpCtrl_B;
	wire [27:0]interim_jump;

	reg [31:0] ID_EX_PC;
(* keep = "true" *)  reg [31:0] ID_EX_Sign_Extended, ID_EX_Instruction;
	
	reg [31:0]ID_EX_shift; wire [31:0]id_shift;
	
	wire IF_write, PC_write, bubble;
    wire [1:0]addrSel;
		
	wire ID_ALUSrc, ID_MemToReg, ID_RegWrite, ID_MemRead, ID_MemWrite, ID_Branch, ID_Jump, ID_SignExtend, ID_UseShmt;
	
	reg   ID_EX_RegWrite, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_SignExtend_control;

(* keep = "true" *) reg ID_EX_MemToReg;
	
	wire [3:0]ID_ALUOp;
	reg  [3:0]ID_EX_ALUOp;
	
	wire ID_DataMemForwardCtrl_EX, ID_DataMemForwardCtrl_MEM;
	reg  ID_EX_DataMemForwardCtrl_EX, ID_EX_DataMemForwardCtrl_MEM;
		
	wire [1:0] ID_AluOpCtrl_A, ID_AluOpCtrl_B;
	
	wire [31:0]ID_Instruction, ID_PC, signExtend_output;
	
	//Stage 3
	wire EX_jal, EX_jr; //new addition
	wire [31:0]EX_alu_jal_mux_out;//new addition
	wire [1:0]EX_RegDst;//new addition
	reg [1:0]EX_MEM_RegDst;//new addition
	wire [4:0] ra;//new addition
	
	wire  EX_ALUSrc,  EX_RegWrite, EX_MemRead, EX_MemWrite, EX_Branch, EX_Jump, EX_SignExtend, EX_UseShmt;
	reg   EX_MEM_RegWrite, EX_MEM_MemRead, EX_MEM_MemWrite;
(* keep = "true" *) reg EX_MEM_MemToReg;
	wire EX_MemToReg;
	
	wire  [3:0]EX_ALUOp, EX_ALUCtrl;
	
	wire [31:0] advanced_shift;
	
	wire [31:0] EX_Reg_A, EX_Reg_B, EX_Sign_Extended, EX_Instruction, EX_PC, EX_zero_Extended, EX_sign_input_B;
	
	wire  [1:0] EX_AluOpCtrl_A, EX_AluOpCtrl_B;
	
	wire  EX_DataMemForwardCtrl_EX, EX_DataMemForwardCtrl_MEM;
	reg   EX_MEM_DataMemForwardCtrl_MEM;
	reg  [4:0]EX_MEM_Rw;
	wire [4:0]EX_Rw;
	wire [31:0] EX_Datain, EX_ALU_A, EX_ALU_B, EX_ALU_Output, interim_branch, branch_target;
	wire EX_ALUZero;
	reg  [31:0]EX_MEM_ALU_Output, EX_MEM_Datain;
	
	
	//Stage 4
	wire  MEM_MemToReg, MEM_RegWrite, MEM_MemRead, MEM_MemWrite, MEM_DataMemForwardCtrl_MEM;
	reg   MEM_WB_RegWrite; 
	reg MEM_WB_MemToReg;
	reg [31:0]MEM_WB_Dataout, MEM_WB_ALU_Output;
	
	wire [31:0]MEM_ALU_Output, MEM_Datain, MEM_Dataout, MEM_RAM_WriteData;
	wire [4:0]MEM_Rw;
	reg  [4:0]MEM_WB_Rw;
	
	//Stage 5
	wire WB_RegWrite, WB_MemToReg;
	wire [31:0]WB_Dataout, WB_ALU_Output, WB_Output;
	wire [4:0]WB_Rw;
	
//==============================================================================LOGIC DESCRIPTION=============================================================	
	//Stage 1 logic - PC AND Instruction Memory
	//pc logic
	always @(negedge CLK, negedge Reset_L) 
						begin//LOGIC FOR PC_write TO BE WRITTEN
	                         if (Reset_L == 0) program_counter <= startPC;
							 else if (!PC_write) program_counter <= program_counter;
                             else program_counter <= next_counter; // update program_counter
						end 
	
	//program_counter adder
	assign interim_counter = program_counter + 4 ; 
    
	//instantiating the instruction memory
    InstructionMemory InstructionMemory(.Data(IF_instruction), .Address(program_counter));
	
	//Input MUX to PC
	mux31 mux31_pc_input (.zero(interim_counter), .one(jump_address),.two(branch_target),.select(addrSel), .out(next_counter));
	
	//IF_ID Interface flip flops
	always@(negedge CLK, negedge Reset_L) begin
							if (!Reset_L) begin 
								IF_ID_Instruction <= 0;
								IF_ID_PC <= 0;
								end
							else if (!IF_write) begin
							IF_ID_Instruction <= IF_ID_Instruction;
							IF_ID_PC <= IF_ID_PC;
							end
							else begin
								IF_ID_Instruction <= IF_instruction;
								IF_ID_PC <= interim_counter;
							end
						 end
						 
	
	//Stage 2 logic
	
	assign ID_Instruction = IF_ID_Instruction;
	assign ID_PC = IF_ID_PC;
	
	//JUMP address calculation	
	assign interim_jump = ID_Instruction[25:0] << 2;
    assign interim_jump_address[31:28] = ID_PC[31:28];
    assign interim_jump_address[27:0] = interim_jump;
    mux21 EX_mux_jump (.zero(interim_jump_address), .one(ID_Reg_A), .select(ID_jr), .out(jump_address));
	
	//Instantiating registerfile
	registerfile registerfile (.Ra(ID_Instruction[25:21]), 
							  .Rb(ID_Instruction[20:16]), 
							  .Rw(WB_Rw), 
							  .Regwr(WB_RegWrite), 
							  .clk(CLK), 
							  .Bw(WB_Output),
							  .Ba(ID_Reg_A), 
							  .Bb(ID_Reg_B), 
							  .reset(Reset_L)); 
							   
	
	//Instantiating Hazard Unit
/*	HazardUnit HazardUnit ( .IF_write(IF_write), 
							.PC_write(PC_write), 
							.bubble(bubble), 
							.addrSel(addrSel), 
							.Jump(ID_Jump), 
							.Branch(ID_Branch), 
							.ALUZero(EX_ALUZero), 
							.memReadEX(EX_MemRead), 
							.currRs(ID_Instruction[25:21]), 
							.currRt(ID_Instruction[20:16]), 
							.prevRt(EX_Instruction[20:16]), 
							.UseShmt(ID_UseShmt), 
							.UseImmed(ID_ALUSrc),
							.Clk(CLK),
							.Rst(Reset_L));*/
	                        HazardUnit HazardUnit ( .IF_write(IF_write), 
                                                    .PC_write(PC_write), 
                                                    .bubble(bubble), 
                                                    .addrSel(addrSel), 
                                                    .Jump(ID_Jump), 
                                                    .Branch(EX_Branch), 
                                                    .ALUZero(EX_ALUZero), 
                                                    .memReadEX(EX_MemRead), 
                                                    .currRs(ID_Instruction[25:21]), 
                                                    .currRt(ID_Instruction[20:16]), 
                                                    .prevRt(EX_Instruction[20:16]), 
                                                    .UseShmt(ID_UseShmt), 
                                                    .UseImmed(ID_ALUSrc),
                                                    .Clk(CLK),
                                                    .Rst(Reset_L),
                                                    .jr(ID_jr),
                                                    .EX_RegWrite(EX_RegWrite),
                                                    .MEM_RegWrite(MEM_RegWrite),
                                                    .prev_prevRt(MEM_Rw), .jal(ID_jal)
                                                    );
							

				
	//Instantiating control unit
	SingleCycleControl SingleCycleControl(     .RegDst(ID_RegDst),
                                            .ALUSrc(ID_ALUSrc), 
                                            .MemToReg(ID_MemToReg), 
                                            .RegWrite(ID_RegWrite), 
                                            .MemRead(ID_MemRead), 
                                            .MemWrite(ID_MemWrite), 
                                            .Branch(ID_Branch), 
                                            .Jump(ID_Jump), 
                                            .SignExtend(ID_SignExtend), 
                                            .ALUOp(ID_ALUOp), 
                                            .Opcode(ID_Instruction[31:26]),
                                            .UseShmt(ID_UseShmt),
                                            .Function(ID_Instruction[5:0]),
                                            .jal(ID_jal),
                                            .jr(ID_jr)                                        
                                            );
											
	//Instantiating Forwarding Unit
	ForwardingUnit ForwardingUnit (.UseShamt(ID_UseShmt),
								  .UseImmed(ID_ALUSrc), 
								  .ID_Rs(ID_Instruction[25:21]), 
								  .ID_Rt(ID_Instruction[20:16]),
								  .EX_Rw(EX_Rw),
								  .MEM_Rw(MEM_Rw),
								  .EX_RegWrite(EX_RegWrite), 
								  .MEM_RegWrite(MEM_RegWrite), 
								  .AluOpCtrl_A(ID_AluOpCtrl_A),
								  .AluOpCtrl_B(ID_AluOpCtrl_B), 
								  .DataMemForwardCtrl_EX(ID_DataMemForwardCtrl_EX), 
								  .DataMemForwardCtrl_MEM(ID_DataMemForwardCtrl_MEM)
								);

	//Instantiating Sign Extend Hardware
	sign_extend sign_extend (.in(ID_Instruction[15:0]), .signExtend(ID_SignExtend), .out(ID_Sign_Extended));
	
	assign id_shift = ID_Reg_B >> (ID_Instruction[10:6]);
	
	//ID_EX Interface flip flops
	always@(negedge CLK , negedge Reset_L) begin
						if(!Reset_L) 
							begin
							//Rs Rt
							ID_EX_Reg_A <= 0;
							ID_EX_Reg_B <= 0;
							
							//SignExtend
							ID_EX_Sign_Extended <= 0;
							ID_EX_shift <= 0;
							
							//Instruction[25:0]
							ID_EX_Instruction <= 0;
							
							//PC+4 Next Instruction
							ID_EX_PC <= 0;
							
							//EX
							ID_EX_RegDst <= 0;
							ID_EX_ALUOp <=0;
							ID_EX_SignExtend_control <= 0;
							ID_EX_jal <= 0;
                            ID_EX_jr <= 0;
                            ID_EX_Branch <= 0;
							
							//MEM
							ID_EX_MemRead <= 0;
							ID_EX_MemWrite <= 0;
							
							//WB
							ID_EX_RegWrite <= 0;
							ID_EX_MemToReg <= 0;
							
							//Forwarding Unit 
							ID_EX_AluOpCtrl_A <= 0;
							ID_EX_AluOpCtrl_B <=0;
							ID_EX_DataMemForwardCtrl_EX <= 0;
							ID_EX_DataMemForwardCtrl_MEM <= 0;
							
							end
							
						else if(bubble) 
							begin
							//Rs Rt
                            ID_EX_Reg_A <= ID_Reg_A;
                            ID_EX_Reg_B <= ID_Reg_B;
                            
                            //SignExtend
                           // ID_EX_Sign_Extended <= ID_Sign_Extended;
                            ID_EX_Sign_Extended <= ID_EX_Sign_Extended;
							
							//Instruction[25:0]
							//ID_EX_Instruction <= ID_Instruction;
							ID_EX_Instruction <= ID_EX_Instruction;
							
							//PC+4 Next Instruction
							ID_EX_PC <= ID_PC;
							
							//EX
							ID_EX_RegDst <= ID_RegDst;
							ID_EX_ALUOp <= ID_ALUOp;
							ID_EX_SignExtend_control <= ID_SignExtend;
							ID_EX_shift <= id_shift;
							ID_EX_jal <= ID_jal;
                            ID_EX_jr <= ID_jr;
                            ID_EX_Branch <= ID_Branch;							
							
							//MEM
							ID_EX_MemRead <= 0;
							ID_EX_MemWrite <= 0;
							
							//WB
							ID_EX_RegWrite <= 0;
							ID_EX_MemToReg <= 0;
							
							//Forwarding Unit 
                            ID_EX_AluOpCtrl_A <= ID_AluOpCtrl_A;
                            ID_EX_AluOpCtrl_B <=ID_AluOpCtrl_B;
                            ID_EX_DataMemForwardCtrl_EX <= ID_DataMemForwardCtrl_EX;
                            ID_EX_DataMemForwardCtrl_MEM <= ID_DataMemForwardCtrl_MEM;
							
							end
							
						else							
							begin
							//Rs Rt
							ID_EX_Reg_A <= ID_Reg_A;
							ID_EX_Reg_B <= ID_Reg_B;
							
							//SignExtend
							ID_EX_Sign_Extended <= ID_Sign_Extended;
							
							
							//Instruction[25:0]
							ID_EX_Instruction <= ID_Instruction;
							
							//PC+4 Next Instruction
							ID_EX_PC <= ID_PC;
							
							//EX
							ID_EX_RegDst <= ID_RegDst;
							ID_EX_ALUOp <=ID_ALUOp;
							ID_EX_SignExtend_control <= ID_SignExtend;
                            ID_EX_shift <= id_shift;
							ID_EX_jal <= ID_jal;
                            ID_EX_jr <= ID_jr;
                            ID_EX_Branch <= ID_Branch;
                            							
							//MEM
							ID_EX_MemRead <= ID_MemRead;
							ID_EX_MemWrite <= ID_MemWrite;
							
							//WB
							ID_EX_RegWrite <= ID_RegWrite;
							ID_EX_MemToReg <= ID_MemToReg;
							
							//Forwarding Unit 
							ID_EX_AluOpCtrl_A <= ID_AluOpCtrl_A;
							ID_EX_AluOpCtrl_B <=ID_AluOpCtrl_B;
							ID_EX_DataMemForwardCtrl_EX <= ID_DataMemForwardCtrl_EX;
							ID_EX_DataMemForwardCtrl_MEM <= ID_DataMemForwardCtrl_MEM;
							
							end
						 end

	
	//Stage 3 logic
	
	//Rs Rt
	assign EX_Reg_A = ID_EX_Reg_A;
	assign EX_Reg_B = ID_EX_Reg_B;
							
	//SignExtend
	assign EX_SignExtend = ID_EX_SignExtend_control;
	assign EX_Sign_Extended = ID_EX_Sign_Extended;

							
	//Instruction[25:0]
	assign EX_Instruction = ID_EX_Instruction;
	assign advanced_shift = ID_EX_shift;
							
	//PC+4 Next Instruction
	assign EX_PC = ID_EX_PC;
							
	//EX
	assign EX_RegDst = ID_EX_RegDst;
	assign EX_ALUOp =ID_EX_ALUOp;
	assign EX_jal = ID_EX_jal;
    assign EX_jr = ID_EX_jr;
    assign EX_Branch = ID_EX_Branch;
    							
	//MEM
	assign EX_MemRead = ID_EX_MemRead;
	assign EX_MemWrite = ID_EX_MemWrite;
							
	//WB
	assign EX_RegWrite = ID_EX_RegWrite;
	assign EX_MemToReg = ID_EX_MemToReg;
							
	//Forwarding Unit 
	assign EX_AluOpCtrl_A = ID_EX_AluOpCtrl_A;
	assign EX_AluOpCtrl_B =ID_EX_AluOpCtrl_B;
	assign EX_DataMemForwardCtrl_EX = ID_EX_DataMemForwardCtrl_EX;
	assign EX_DataMemForwardCtrl_MEM = ID_EX_DataMemForwardCtrl_MEM;
	
	//Instantiating ALU control
	ALUControl ALUControl(.ALUCtrl(EX_ALUCtrl), .ALUop(EX_ALUOp), .FuncCode(EX_Instruction[5:0]));
			
	//ALU Source Muxes
	mux31 EX_mux41_A (.zero(EX_Reg_A) , .one(MEM_ALU_Output), .two(WB_Output), .select(EX_AluOpCtrl_A), .out(EX_ALU_A));//INCOMPLETE
	
	mux41 EX_mux41_B (.zero(EX_Reg_B) , .one(MEM_ALU_Output), .two(WB_Output), .three(EX_Sign_Extended), .select(EX_AluOpCtrl_B), .out(EX_ALU_B));
	
	//Instantiating ALU
	//ALU ALU (.BusA(EX_ALU_A), .BusB(EX_ALU_B), .ALUCtrl(EX_ALUCtrl), .BusW(EX_ALU_Output), .Zero(EX_ALUZero), .shmt(EX_Instruction[10:6]));
	ALU ALU (.BusA(EX_ALU_A), .BusB(EX_ALU_B), .ALUCtrl(EX_ALUCtrl), .BusW(EX_ALU_Output), .Zero(EX_ALUZero), .shmt(EX_Instruction[10:6]), .shift(advanced_shift));
	
	mux21 EX_mux_datain (.zero(EX_Reg_B), .one(WB_Output), .select(EX_DataMemForwardCtrl_EX), .out(EX_Datain));
	
	mux21 EX_mux_aluout (.zero(EX_ALU_Output), .one(EX_PC), .select(EX_jal), .out(EX_alu_jal_mux_out));
	
	//mux21_5bit EX_mux_Rw (.zero(EX_Instruction[20:16]), .one(EX_Instruction[15:11]), .select(EX_RegDst), .out(EX_Rw));
	//assign ra = 31;
	mux21_5bit EX_mux_Rw (.zero(EX_Instruction[20:16]), .one(EX_Instruction[15:11]), .two(5'b11111), .select(EX_RegDst), .out(EX_Rw));
	
	//calculating branch_address
	assign interim_branch = EX_Sign_Extended << 2;
	assign branch_target = interim_branch + EX_PC;
	
	always@(negedge CLK, negedge Reset_L) begin
					if(!Reset_L)
						begin
						//Destination Register
						EX_MEM_Rw <= 0;
						EX_MEM_Datain <= 0;
						
						//DataMemForwardCtrl_MEM
						EX_MEM_DataMemForwardCtrl_MEM <= 0;
						
						//ALU output
						EX_MEM_ALU_Output <= 0;
						
						//MEM
						EX_MEM_MemRead <= 0;
						EX_MEM_MemWrite <= 0;
							
						//WB
						EX_MEM_RegWrite <= 0;
						EX_MEM_MemToReg <= 0;		
						end			
					else
						begin
						//Destination Register
						EX_MEM_Rw <= EX_Rw;
						EX_MEM_Datain <= EX_Datain;
						
						//DataMemForwardCtrl_MEM
						EX_MEM_DataMemForwardCtrl_MEM <= EX_DataMemForwardCtrl_MEM;
						
						//ALU output
						EX_MEM_ALU_Output <= EX_alu_jal_mux_out;
						
						//MEM
						EX_MEM_MemRead <= EX_MemRead;
						EX_MEM_MemWrite <= EX_MemWrite;
							
						//WB
						EX_MEM_RegWrite <= EX_RegWrite;
						EX_MEM_MemToReg <= EX_MemToReg;
						end
					end	
	
	//Stage 4 logic
	assign MEM_MemToReg = EX_MEM_MemToReg;
	assign MEM_RegWrite = EX_MEM_RegWrite;
	assign MEM_MemRead = EX_MEM_MemRead;
	assign MEM_MemWrite = EX_MEM_MemWrite;
	assign MEM_DataMemForwardCtrl_MEM = EX_MEM_DataMemForwardCtrl_MEM;
	assign MEM_ALU_Output = EX_MEM_ALU_Output;
	assign MEM_Datain = EX_MEM_Datain;
	assign MEM_Rw = EX_MEM_Rw;

	//Instantiating random access memory
    DataMemory DataMemory ( .Clock(CLK), //input clock
                    .MemoryRead(MEM_MemRead),  //Read enable pin
                    .MemoryWrite(MEM_MemWrite), //Write enable pin
                    .Address(MEM_ALU_Output[5:0]), // 6 bit address to access 64 bit registers
                    .ReadData(MEM_Dataout), //read out bus
                    .WriteData(MEM_RAM_WriteData)
                    ); //input write data
        
    mux21 mux_datamemory (.zero(MEM_Datain), .one(WB_Output), .select(MEM_DataMemForwardCtrl_MEM), .out(MEM_RAM_WriteData));//datamemory input mux

	always@(negedge CLK, negedge Reset_L) begin
								if(!Reset_L)
									begin
										MEM_WB_Dataout <= 0;
										MEM_WB_ALU_Output <= 0;
										MEM_WB_Rw <= 0;
										MEM_WB_RegWrite <= 0;
										MEM_WB_MemToReg <= 0;
									end
								else 
									begin
										MEM_WB_Dataout <= MEM_Dataout;
										MEM_WB_ALU_Output <= MEM_ALU_Output;
										MEM_WB_Rw <= MEM_Rw;
										MEM_WB_RegWrite <= MEM_RegWrite;
										MEM_WB_MemToReg <= MEM_MemToReg;
									end
								end

	//Stage 5 logic
	assign WB_RegWrite = MEM_WB_RegWrite;
	assign WB_MemToReg = MEM_WB_MemToReg;
	assign WB_Dataout = MEM_WB_Dataout;
	assign WB_ALU_Output = MEM_WB_ALU_Output;
	assign WB_Rw = MEM_WB_Rw;
	
	mux21 mux_writeback (.zero(WB_ALU_Output), .one(WB_Dataout), .select(WB_MemToReg), .out(WB_Output));
	
	assign dMemOut = WB_Dataout;
	
endmodule