`timescale 1ns / 1ps
//Defining module for Hazard detection Unit
//It detects RAW hazard related to Load and also responsible for generating stalls of 1 cycle for RAW(Load---instruction) and 
//2 cycles when a branch instruction is found
//It is also responsible for changing the PC POINTER to the JUMP address

//Defining states for FSM
`define NO_HAZARD  3'b000
`define RAW_HAZARD 3'b001
`define JUMP	   3'b010
`define BRANCH_0   3'b011
`define BRANCH_1   3'b100
`define JAL_0      3'b101
`define JAL_1      3'b110
`define JR         3'b111

module HazardUnit(IF_write, PC_write, bubble, addrSel,jal, EX_RegWrite, MEM_RegWrite, prev_prevRt, jr, Jump, Branch, ALUZero, memReadEX, currRs, currRt, prevRt, UseShmt, UseImmed, Clk, Rst);

	output reg IF_write, PC_write, bubble;
	output reg [1:0]addrSel;
	input Jump, jr, Branch, ALUZero, memReadEX, Clk, Rst, jal;
	input UseImmed, UseShmt;
	input [4:0] currRt, currRs, prevRt, prev_prevRt;
	input EX_RegWrite, MEM_RegWrite;

	reg Load_Hazard, jr_Hazard; //register containing state of load related RAW hazard
	
	reg [2:0] current_state, next_state;
	
	reg [1:0] next_addrsel;
	
	//Load related RAW hazard detection logic
	always @ (*) begin
		if (prevRt != 0) begin
			if ((currRs == prevRt || currRt == prevRt) && memReadEX == 1'b1 && UseImmed == 1'b0 && UseShmt == 1'b0) Load_Hazard <= 1'b1;
			else if (UseShmt == 1'b1 && currRs == prevRt && memReadEX == 1'b1) Load_Hazard <= 1'b1;
			else if (UseImmed == 1'b1 && currRs == prevRt && memReadEX == 1'b1) Load_Hazard <= 1'b1;
			else Load_Hazard <= 1'b0;
		end
		else Load_Hazard <= 1'b0;
	end
	
	
//	jal - jr odd case detection
	always @(*) 
        begin
        if (jr == 1 && (prevRt != 0 || prev_prevRt != 0)) begin
             if (currRs == prev_prevRt /*&& (prev_prevRt != prevRt || EX_RegWrite == 0)*/ && MEM_RegWrite == 1) jr_Hazard<= 1'b1;
             else if (currRs == prevRt && EX_RegWrite == 1) jr_Hazard  <= 1'b1;
             else jr_Hazard <= 1'b0;
            end
        else jr_Hazard <= 1'b0;
        end
		
	//FSM for hazard detection unit
	//change state every clock cycle
	always @(posedge Clk, negedge Rst) begin
		if (!Rst) begin
		current_state <= `NO_HAZARD;
		end
		else begin
		current_state <= next_state;
		end
	end
	
	//logic for state change and output
	always @(*) begin
		case (current_state)
			`NO_HAZARD : begin
			                if (Branch == 1'b1 && ALUZero == 1'b1) next_state <= `BRANCH_0;
							else if (Load_Hazard == 1'b1) next_state <= `RAW_HAZARD;
							else if (jr_Hazard == 1'b1) next_state <= `JR;
						    else if (Jump == 1'b1) next_state <= `JUMP;						    
							//else if (jal == 1'b1) next_state <= `JAL_0;
							else next_state <= `NO_HAZARD;
							
							PC_write <= 1'b1;
							IF_write <= 1'b1;
							bubble <= 1'b0;
							
                            addrSel <= next_addrsel;
                            next_addrsel <= 2'b00;
						 end
						 
			`RAW_HAZARD :begin
							next_state <= `NO_HAZARD;
							
							PC_write <=1'b0;
							IF_write <=1'b0;
							bubble <= 1'b1;
							addrSel <= 2'b00; //pc+4
							next_addrsel <= 2'b00;
						 end
						 
			`JUMP		:begin
							next_state <= `NO_HAZARD;
							
							PC_write <=1'b1;
							IF_write <=1'b0;
							bubble <= 1'b1;
							addrSel <= 2'b01; //JUMP Target
							next_addrsel <= 2'b00;
						 end

			`JR		    :begin
                            if (jr_Hazard == 1'b1) next_state <= `JR;
                            else next_state <= `JUMP;
                                         
                            PC_write <=1'b0;
                            IF_write <=1'b0;
                            bubble <= 1'b1;
                            addrSel <= 2'b00; //JUMP Target
                            next_addrsel <= 2'b00;
                          end			
						 

			`BRANCH_0   :  begin
                            
                            next_state <= `BRANCH_1;
                            PC_write <=1'b1;
                            IF_write <=1'b0;
                            bubble <= 1'b1;
                                                        
                            addrSel <= 2'b10; //pc+4
                            next_addrsel <= 2'b00;    
							                			                
						 end
						 
			`BRANCH_1   :begin
							next_state <= `NO_HAZARD;
							
							PC_write <=1'b0;
							IF_write <=1'b0;
							bubble <= 1'b1;
							
                            addrSel <= 2'b00; //BRANCH Target
                            next_addrsel <= 2'b00;
						 end
						 
						 
		    `JAL_0   :begin
                           next_state <= `JAL_1;
                                         
							PC_write <=1'b0;
                            IF_write <=1'b0;
                            bubble <= 1'b1;
                            addrSel <= 2'b01; //JUMP Target
                            next_addrsel <= 2'b00;
                                      end                                     
            `JAL_1   :begin
							next_state <= `NO_HAZARD;
             
                            PC_write <=1'b0;
                            IF_write <=1'b0;
                            bubble <= 1'b1;
                            addrSel <= 2'b00; //pc+4
                            next_addrsel <= 2'b00;
                                      end
			default   :begin
						next_state <= current_state;
						
						PC_write <= 1'b0;
						IF_write <= 1'b0;
						bubble <= 1'b0;
						addrSel <= 2'b00; //pc+4
						next_addrsel <= 2'b00;
						end
		endcase
	end
	
		
endmodule