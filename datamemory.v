`timescale 1ns / 1ps

module DataMemory (input Clock, //input clock
				input MemoryRead,  //Read enable pin
				input MemoryWrite, //Write enable pin
				input [5:0]Address, // 6 bit address to access 64 bit registers
				output reg [31:0]ReadData, //read out bus
				input [31:0]WriteData ); //input write data
				
		reg [31:0]datamem[63:0]; // RAM defined
		//reg [31:0]temp;
		
	always @(posedge Clock) begin //defined for memory read operation 
        
             if (MemoryRead == 1)  // chackes if the enable pin is on
                     ReadData <= datamem[Address];
        end
      
     always @(negedge Clock) //defined for memory write operation
         begin
              if(MemoryWrite == 1) //checks if the enable pin is on
                      datamem[Address] <= WriteData;
         end
        		

		 // assign ReadData = temp;
		

endmodule
