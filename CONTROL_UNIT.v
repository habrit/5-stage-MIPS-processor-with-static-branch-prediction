`timescale 1ns / 1ps


//Defining control signals

`define AND 4'b0000
`define OR 4'b0001
`define ADD 4'b0010
`define SLL 4'b0011
`define SRL 4'b0100
`define SUB 4'b0110
`define SLT 4'b0111
`define ADDU 4'b1000
`define SUBU 4'b1001
`define XOR 4'b1010
`define SLTU 4'b1011
`define NOR 4'b1100
`define SRA 4'b1101
`define LUI 4'b1110
`define FUNC 4'b1111

`define RTYPEOPCODE 6'b000000
`define LWOPCODE 6'b100011
`define SWOPCODE 6'b101011
`define BEQOPCODE 6'b000100
`define JOPCODE 6'b000010
`define ORIOPCODE 6'b001101
`define ADDIOPCODE 6'b001000
`define ADDIUOPCODE 6'b001001
`define ANDIOPCODE 6'b001100
`define LUIOPCODE 6'b001111
`define SLTIOPCODE 6'b001010
`define SLTIUOPCODE 6'b001011
`define XORIOPCODE 6'b001110
`define JALOPCODE 6'b000011



module SingleCycleControl(RegDst, ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Jump, SignExtend, ALUOp, Opcode , UseShmt, Function, jal, jr);
   input [5:0] Opcode;
   input [5:0] Function;
   output Jump;
   output Branch;
   output ALUSrc;
   output UseShmt;
   output MemToReg;
   output RegWrite;
   output MemRead;
   output MemWrite;
   output SignExtend;
   output [3:0] ALUOp;
   output [1:0]RegDst;
   output jal;
   output jr;  
      
    reg  ALUSrc, MemToReg, RegWrite, MemRead, MemWrite, Branch, Jump, SignExtend, UseShmt, jal, jr;
    reg  [3:0] ALUOp;
    reg [1:0]RegDst;
    always @ (Opcode,Function) begin
        case(Opcode)
            `RTYPEOPCODE: begin
                RegDst <= 2'b01;
                ALUSrc <= 0;
                MemToReg <=  0;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;                
				SignExtend <=  1'b0;
                ALUOp <=  `FUNC;
                jal <= 0;
                
                //jr instruction
                if(Function == 6'b001000) begin                   
                    jr <= 1;
                    Jump <=  1;
                    RegWrite <=  0;
                end
				else begin
				    jr <= 0;
				    Jump <=  0;
				    RegWrite <=  1;
				end
				
				//UseShmt signal
				if (Function == 6'b000010 || Function == 6'b000011 || Function == 6'b000000) UseShmt <= 1'b1;
				else UseShmt <= 1'b0;
            end
            `LWOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  1;
                RegWrite <=  1;
                MemRead <=  1;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  0;
                SignExtend <=  1'b1;
				UseShmt <= 1'b0;
			    jal <= 0;
                jr <= 0;
                ALUOp <=  `ADD;
			end
			`SWOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  0;
                RegWrite <=  0;
                MemRead <=  0;
                MemWrite <=  1;
                Branch <=  0;
                Jump <=  0;
				jal <= 0;
                jr <= 0;                
                SignExtend <=  1'b1;
				UseShmt <= 1'b1;
                ALUOp <=  `ADD;
			end
			`BEQOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 0;
                MemToReg <=  0;
                RegWrite <=  0;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  1;
                Jump <=  0;
                SignExtend <=  1'b1;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  `SUB;
			end
			`JOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 0;
                MemToReg <=  0;
                RegWrite <=  0;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  1;
                SignExtend <=  1'b1;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  4'bxxxx;
			end
		    `JALOPCODE : begin
                RegDst <= 2'b10;
                ALUSrc <= 0;
                MemToReg <=  0;
                RegWrite <=  1;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  1;
                SignExtend <=  1'b1;
                UseShmt <= 1'b0;
                jal <= 1;
                jr <= 0;
                ALUOp <=  4'bxxxx;
            end
			`ORIOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  0;
                RegWrite <=  1;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  0;
                SignExtend <=  1'b0;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  `OR;
			end
			`ADDIOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  0;
                RegWrite <=  1;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  0;
                SignExtend <=  1'b1;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  `ADD;
			end
			`ADDIUOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  0;
                RegWrite <=  1;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  0;
                SignExtend <=  1'b0;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  `ADD;
			end
			`ANDIOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  0;
                RegWrite <=  1;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  0;
                SignExtend <=  1'b0;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  `AND;
			end
			`LUIOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  0;
                RegWrite <=  1;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  0;
                SignExtend <=  1'b0;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  `LUI;
			end
			`SLTIOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  0;
                RegWrite <=  1;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  0;
                SignExtend <=  1'b1;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  `SLT;
			end
			`SLTIUOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  0;
                RegWrite <=  1;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  0;
                SignExtend <=  1'b1;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  `SLTU;
			end
			`XORIOPCODE : begin
				RegDst <= 2'b00;
                ALUSrc <= 1;
                MemToReg <=  0;
                RegWrite <=  1;
                MemRead <=  0;
                MemWrite <=  0;
                Branch <=  0;
                Jump <=  0;
                SignExtend <=  1'b0;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  `XOR;
			end
            default: begin
                RegDst <=  2'bxx;
                ALUSrc <=  1'bx;
                MemToReg <=  1'bx;
                RegWrite <=  1'bx;
                MemRead <=  1'bx;
                MemWrite <=  1'bx;
                Branch <=  1'bx;
                Jump <=  1'bx;
                SignExtend <=  1'bx;
				UseShmt <= 1'b0;
				jal <= 0;
                jr <= 0;
                ALUOp <=  4'bxxxx;
            end
        endcase
    end
endmodule