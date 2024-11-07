`timescale 1ns / 1ps
//Word addressible register file memory module of dimension 32x32

module registerfile(input [4:0]Ra, //Read from register a(ASYNCHRONOUS)
					input [4:0]Rb, //Read from register b(ASYNCHRONOUS)
					input [4:0]Rw, //Write on register w(SYNCHRONOUS)
					input [31:0]Bw,//Write bus
					input clk,     //clock signal
					input Regwr,   //Write enable
					output reg [31:0]Ba,//Output reading bus for register a
					output reg [31:0]Bb,//Output reading bus for register b
					input reset);
	
(* keep = "true" *)	reg [31:0]rf[0:31]; //32x32 packed array for register file

   integer i;
   always@(posedge clk, negedge reset) begin 
                          if (reset == 0) for (i=0; i<32; i=i+1) rf[i] <= 32'h0;
                          else if(Regwr == 1 && Rw != 0)  rf[Rw] <= Bw;         //write operation
                          end 


	always @(*) begin
	               Ba <= rf[Ra];
	               Bb <= rf[Rb];
	            end
	            

    
endmodule