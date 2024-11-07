`timescale 1ns / 1ps
//Module definition of Forwarding Unit
//Detects a RAW data hazard

module ForwardingUnit(UseShamt,UseImmed, ID_Rs, ID_Rt, EX_Rw, MEM_Rw, EX_RegWrite, MEM_RegWrite, AluOpCtrl_A, AluOpCtrl_B, 
						DataMemForwardCtrl_EX, DataMemForwardCtrl_MEM);
						
	
	input UseImmed, UseShamt;
	input [4:0] ID_Rs, ID_Rt, EX_Rw, MEM_Rw;
	input EX_RegWrite, MEM_RegWrite;
	output reg [1:0]AluOpCtrl_A, AluOpCtrl_B;
	output reg DataMemForwardCtrl_EX, DataMemForwardCtrl_MEM;
	
	//Logic for AluOpCtrl_A
	always @(*) 
		begin
			if (UseShamt == 0 /*&& (EX_Rw != 0 || MEM_Rw != 0 )*/)
				begin
					if (ID_Rs == MEM_Rw && (MEM_Rw != EX_Rw || EX_RegWrite == 0) && MEM_RegWrite == 1) AluOpCtrl_A <= 2'b10;
					else if (ID_Rs == EX_Rw && EX_RegWrite == 1) AluOpCtrl_A <= 2'b01;
					else AluOpCtrl_A <= 2'b00;
				end
			else if (UseShamt == 1) AluOpCtrl_A <= 2'b11;
			else AluOpCtrl_A <= 2'b00;	
	
		end
	
	//Logic for AluOpCtrl_B	
	always @(*) 
		begin
			if (UseImmed == 0 /*&& (EX_Rw != 0 || MEM_Rw != 0 )*/)
				begin
					if (ID_Rt == MEM_Rw && (MEM_Rw != EX_Rw || EX_RegWrite == 0) && MEM_RegWrite == 1) AluOpCtrl_B <= 2'b10;
					else if (ID_Rt == EX_Rw && EX_RegWrite == 1) AluOpCtrl_B <= 2'b01;
					else AluOpCtrl_B <= 2'b00;
				end
			else if (UseImmed == 1) AluOpCtrl_B <= 2'b11;
			else AluOpCtrl_B <= 2'b00;
		end
	
	//Logic for DataMemForwardCtrl_EX and DataMemForwardCtrl_MEM
	always @(*) 
		begin
			if(MEM_RegWrite == 1 && ID_Rt == MEM_Rw)
				begin
					DataMemForwardCtrl_EX <= 1'b1;
					DataMemForwardCtrl_MEM <= 1'b0;
				end
			else
				begin
					if(EX_RegWrite == 1 && ID_Rt == EX_Rw)	
						begin
							DataMemForwardCtrl_EX <= 1'b0;
							DataMemForwardCtrl_MEM <= 1'b1;
						end
					else
						begin
							DataMemForwardCtrl_EX <= 1'b0;
							DataMemForwardCtrl_MEM <= 1'b0;
						end
				end	
		end
endmodule
	