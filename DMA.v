module DMA(clk,data_add,done,bus_req,interrupt,mode,mem_write,mem_read,DMA_req);
input clk,interrupt,mode,DMA_req;
output bus_req,done,mem_write,mem_read;
inout [31:0] data_add;
wire write,writeawe,finish,Wait,mux_1,mux_2,mux_3,count_sig,const;
wire [2:0] regread,regwrite;
wire [31:0] adder_out, read_data,counter_out,mux2_out1,mux2_out2,mux2_in,adder_in,mux4_out2,mux4_out1;
reg [31:0] zero;
initial
begin
zero<=0;
end

InOut Inout_DMA(data_add,mux2_in,adder_out, mux_1);
DMA_controller control_1 (clk ,DMA_req, mode , interrupt , Wait , done , bus_req , regwrite , mux_1 , mux_2 , count_sig , write ,writeawe, regread , finish , mux_3,mem_write,mem_read);
MUX_special mux2 (mux2_out1,mux2_in,mux2_out2,mux_2);
MUX mux3(adder_in,counter_out,zero,mux_3);
adder DMA_adder(read_data,adder_in,adder_out);
registers_DMA DMA_regfile(regwrite, mux2_out1, regread, read_data, write);
counter_bytes DMA_count_bytes( finish, Wait,mux2_out2,clk,writeawe);
counter  DMA_counter ( count_sig,  finish, counter_out);




endmodule

module DMA_controller(clk ,DMA_req, mode , interrupt , WAIT , done , bus_req , regwrite , mux_1 , mux_2 , count_sig , write ,writeawe, regread , finish , mux_3,mem_write,mem_read);

input clk,mode,interrupt,finish,DMA_req;

output reg WAIT,done,bus_req,mux_1,mux_2,count_sig,write,writeawe,mux_3,mem_write,mem_read;
output reg [2:0] regwrite,regread;
reg [2:0] counter,counter2,counter3;
initial
begin
counter <=0;
counter2<=0;
counter3<=0;
mem_write<=0;
mem_read<=0;
bus_req<=0;
done<=0;
end
always@(posedge clk)
begin
if(DMA_req==1)
begin
if(interrupt == 0)
begin 

if(mode == 1)
begin 
case(counter)

0:
begin

WAIT <= 0;
done <= 0;
bus_req <= 0;
regwrite <= 3'b000;
mux_1 <= 0;
count_sig <= 0;
write <= 1;
mux_3 <= 1;
mux_2 <= 0;
mem_write<=0;
mem_read<=0;
counter <= counter+1;


end
1:
begin

mux_2 <= 0;
WAIT <= 0;
done <= 0;
bus_req <= 1;
regwrite <= 3'b010;
mux_1 <= 0;
count_sig <= 0;
write <= 1;
mux_3 <= 1;
mem_write<=0;
mem_read<=0;
counter <= counter+1;



end

2:
begin


WAIT <= 0;
done <= 0;
bus_req <= 1;
mux_1 <= 1;
count_sig <= 0;
write <= 0;
mux_3 <= 1;
mem_write<=1;
mem_read<=0;
counter <= counter+1;
regread <= 3'b000;


end

3:
begin


WAIT <=0;
done <=0;
bus_req <= 1;
mux_1 <= 1;
count_sig <= 0;
write <= 0;
mux_3 <= 1;
mem_write<=1;
mem_read<=0;
regread <= 3'b010;
counter <= counter+1;

end
4:
begin
WAIT <= 0;
done <= 1;
bus_req <= 0;
mux_1 <= 1;
count_sig <= 0;
write <= 0;
mux_3 <= 1;
mem_write<=0;
mem_read<=0;
regread <= 3'b010;
counter <= 0;
end

endcase

end

else
begin
case(counter)

0:
begin
writeawe<=0;
WAIT <= 0 ;
bus_req <= 0;
mux_1 <= 0;
mux_2 <= 0;
write <= 1;
regwrite <= 3'b000;
mem_write<=0;
mem_read<=0;
counter <= counter+1;
done<=0;

end

1:
begin
writeawe<=0;
WAIT <= 0 ;
bus_req <= 0;
mux_1 <= 0;
mux_2 <= 0;
write <= 1;
regwrite <= 3'b001;
mem_write<=0;
mem_read<=0;
counter <= counter+1;
done<=0;
end

2:
begin
writeawe<=1;
WAIT <= 0 ;
bus_req <= 1;
mux_1 <= 0;
mux_2 <= 1;
write <= 0;
counter <= counter+1;
mem_write<=0;
mem_read<=0;
done<=0;
end
3:
begin
writeawe<=0;
if(finish == 0)
begin


case(counter2)
0:
begin
WAIT <= 0 ;
bus_req <= 1;
mux_1 <= 1;
mux_3 <= 0;
write <= 0;
count_sig <= 0;
regread <= 3'b001;
mem_write<=0;
mem_read<=1;
counter2 <= counter2+1;
done<=0;
end
1:
begin
WAIT <= 0 ;
bus_req <= 1;
mux_1 <= 0;
mux_2 <= 0;
write <= 1;
count_sig <= 0;
regwrite <= 3'b010;
mem_write<=0;
mem_read<=1;
counter2 <= counter2+1;
done<=0;
end
2:
begin
WAIT <= 0 ;
bus_req <= 1;
mux_1 <= 1;
mux_3 <= 0;
write <= 0;
count_sig <= 0;
regread <= 3'b000;
mem_write<=1;
mem_read<=0;
counter2 <= counter2+1;
done<=0;
end
3:
begin
WAIT <= 0 ;
bus_req <= 1;
mux_1 <= 1;
mux_3 <= 1;
write <= 0;
count_sig <= 0;
mem_write<=1;
mem_read<=0;
regread <= 3'b010;
counter2 <= counter2+1;
done<=0;
end
4:
begin
WAIT <= 0 ;
bus_req <= 1;
mux_1 <= 1;
mux_3 <= 1;
write <= 0;
count_sig <= 1;
mem_write<=0;
mem_read<=0;
regread <= 3'b010;
counter2 <= 0;
done<=0;
end


endcase
end
else
begin
bus_req <= 0;
counter2 <= 0;
counter <= 0;
mem_write<=0;
mem_read<=0;
done<=1;
end



end
endcase
end
end
else if (interrupt == 1)
begin
case(counter3)
0:
begin
WAIT<=1;
done<=0;
bus_req<=1;
mux_1<=1;
mux_3<=1;
count_sig<=0;
write<=0;
regread<=3'b100;
mem_write<=0;
mem_read<=1;
counter3<= counter3+1;
end
1:
begin
WAIT<=1;
done<=0;
bus_req<=1;
mux_1<=0;
mux_2<=0;
count_sig<=0;
write<=1;
regwrite<=3'b101;
mem_write<=0;
mem_read<=1;
counter3<=counter3+1;

end
2:
begin
WAIT<=1;
done<=0;
bus_req<=1;
mux_1<=1;
mux_3<=1;
count_sig<=0;
write<=0;
regread<=3'b111;
mem_write<=1;
mem_read<=0;
counter3<=counter3+1;
end
3:
begin
WAIT<=0;
done<=1;
bus_req<=0;
mux_1<=1;
mux_3<=1;
count_sig<=0;
write<=0;
mem_write<=1;
mem_read<=0;
regread<=3'b101;
counter3<=counter3+1;
end
4:
begin
WAIT<=0;
done<=0;
bus_req<=0;
mux_1<=1;
mux_3<=1;
count_sig<=0;
write<=0;
mem_write<=0;
mem_read<=0;
regread<=3'b101;
counter3<=0;
end
endcase
end

end



else
begin
mux_1<=0;
end
end

endmodule

module InOut(a,b,out, OutNotIn);
input OutNotIn;
input [31:0] out;
inout [31:0] a;
output wire[31:0] b;
assign a = (OutNotIn)? out : 32'bzzzz;  //a is in output mode 
assign b = (OutNotIn)? 32'bzzzz : a; //a is in input mode
endmodule


module MUX(o1,in1,in2,sel);
output reg [31:0] o1;
input [31:0] in1,in2;
input sel;
always@(in1,in2,sel)
begin
if(sel==0)
o1=in1;
else if(sel==1)
o1=in2;
else
o1=1'bx;

end
endmodule
module MUX_special(o1,in1,o2,sel);
output reg [31:0] o1,o2;
input [31:0] in1;
input sel;
always@(in1,sel)
begin
if(sel==0)
o1=in1;
else if(sel==1)
o2=in1;
else
o1=1'bx;

end
endmodule
module adder(in1,in2,out);
input [31:0] in1,in2;
output reg [31:0] out;
always @(in1,in2)
begin
out<=in1+in2;

end
endmodule
module registers_DMA(write_address, write_data, read_address, read_data, write);
input [2:0] write_address, read_address;
input [31:0]  write_data;
input write;
output  [31:0] read_data;
reg [31:0] register [0:7];
initial
begin
register[4]=1023;
register[7]=900;
end

assign read_data = register[read_address];
always @( write_data,write_address)
begin 
#1
if( write==1) begin
 register[write_address]=write_data; 
end

end



endmodule
module counter_bytes( finish, WAIT,num_data,clk,writeawe);
input WAIT,clk,writeawe;
input [31:0] num_data;
output reg finish;
reg [2:0] counter;
reg [31:0] number;
reg off ;
initial
begin
off<=0;
counter<=0;

//$monitor("numdata=%b,number=%d,finish=%b,write=%b,counter=%d,off=%b",num_data,number,finish,writeawe,counter,off);
end
always@ (posedge clk)
begin
if(writeawe==1)
begin 
off<=0;
number<=num_data+1;
end
if (WAIT==0&& off==0)
begin
case(counter)
0:
begin
finish<=0;
counter<=counter+1;
end
1:
begin
finish<=0;
counter<=counter+1;
end
2:
begin
finish<=0;
counter<=counter+1;
end
3:
begin
finish<=0;
counter<=counter+1;
end
4:
begin
number=number-1;
counter<=0;
if(number==0)
begin
finish<=1;
off<=1;
end
else
begin
finish<=0;

end
end


endcase
end
else
begin
counter<=counter;

end

end



endmodule

module counter( count,  finish, o1);
input count, finish;
output reg [31:0] o1;
initial
begin
o1=32'b0;
end
always @(posedge count)
begin
if(finish==1)
o1=0;
else
o1=o1+1;
end 
endmodule 


module processor(DMA_req,DMA_mode, B_wait,data,clk,mem_write,mem_read);
input  B_wait,clk;
output  DMA_req,DMA_mode,mem_write,mem_read;
inout [31:0] data;
wire Wait,memtoreg,alusrc,wrt,regdst,OutnotIn;
wire [1:0] dataio;
wire [31:0] inst,inst_sign,rd1,rd2,adder_out,mux_in,inst_num,mux1,mux2,mux3,out_mux;
initial 
begin
//$monitor("%t ,mode= %b, Data= %b ,mem_write=%b , mem_read=%b", $time , DMA_mode, data,mem_write,mem_read);

end
pc pc1(inst_num,clk,Wait);
InsturctionMemory inst1(inst_num, inst,clk,Wait);
RegisterFile regfile (inst[25:21],inst[20:16], mux1,mux2,wrt,rd1,rd2);
signextend sign1(inst_sign,inst[15:0]);
adder_process adder1(rd1,mux3,adder_out);
MUX_process mux_1(mux1,inst[20:16],inst[15:11],regdst);
MUX_process mux_2(mux2,adder_out,mux_in,memtoreg);
MUX_process mux_3(mux3,rd2,inst_sign,alusrc);
MUX4in MUX_1(out_mux,rd1,rd2,inst_sign,in4,dataio);
InOut_procc inout1(data,mux_in,out_mux, OutnotIn);
control_unit control_1 (Wait, dataio, DMA_req, DMA_mode, regdst, wrt, alusrc, memtoreg,inst[31:26] , B_wait,clk,OutnotIn,mem_write,mem_read);

endmodule



module clockgen(clk);
output reg clk;
initial
begin
clk=0;
end
always 
begin
#31.25
clk=~clk;
end
endmodule




module InsturctionMemory(address, instruction,clk,Wait);
input  [31:0] address;
input clk,Wait;
output reg [31:0] instruction;
reg [31:0] instruction_memory [0:8191];

initial
begin
$readmemb("Instmem.txt",  instruction_memory);
end
always@(posedge clk )
if(Wait == 0)
begin
begin

instruction = instruction_memory[address];
end
end
endmodule 





module RegisterFile(Read1,Read2, WriteReg,WriteData,RegWrite,Data1,Data2);
input [4:0] Read1, Read2, WriteReg;
input [31:0] WriteData;
input RegWrite;
output  [31:0]Data1;
output [31:0] Data2;
reg [31:0] register [0:31];

initial 
begin 
$readmemb("regFile.txt",  register);
$display("%b reg0",register[0] );
end

assign Data1= register[Read1];
assign  Data2= register[Read2];
always @(WriteData,WriteReg)
begin 
#1
//$display("%b WriteData", WriteData);
if(RegWrite==1) begin
 register[WriteReg]=WriteData; end

end

endmodule



module control_unit (Wait, data_io, DMA_req, DMA_mode, rd, write, Alu_src, mem_reg, op_code, B_wait,clk,OutnotIn,mem_write,mem_read);

output reg [1:0]  data_io;
output reg Wait, DMA_req, DMA_mode, rd, write, Alu_src, mem_reg,OutnotIn,mem_write,mem_read;
input [5:0] op_code;
reg [1:0] counter;
input B_wait,clk;

initial
begin
//$monitor("%t, counter= %d, opcode=%b ,DMA_mode=%b  ",$time, counter,op_code,DMA_mode);

Wait <=0;
counter<=0;
mem_write<=0;
mem_read<=0;
Wait<=0;
end
always@(posedge clk,op_code)
begin

casez(op_code)
6'b000000 : //add
begin
DMA_req<= 0;
Wait<= 0;
rd<= 1;
write<= 1;
Alu_src<=0; 
mem_reg<=0; 
data_io<=11;
OutnotIn <=1;
mem_write<=0;
mem_read<=0;
end

6'b000001 : //lw
begin
if (B_wait ==0)
begin
case (counter)
0:
begin
DMA_req<= 0;
rd<= 0;
write<= 0;
Alu_src<= 1;
mem_reg<= 1;
data_io<= 00;
Wait<= 1;
OutnotIn<=1;
counter = counter+1;
mem_write<=0;
mem_read<=1;
end
1:
begin
DMA_req<= 0;
rd<= 0;
write<= 1;
Alu_src<= 1;
mem_reg<= 1;
data_io<= 11;
Wait<= 0;
OutnotIn<=0;
counter <= counter+1;
mem_write<=0;
mem_read<=1;
end
2:
begin
DMA_req<= 0;
rd<= 0;
write<= 1;
Alu_src<= 1;
mem_reg<= 1;
data_io<= 11;
Wait<= 0;
OutnotIn<=0;
counter <= counter+1;
counter<=0;
mem_write<=0;
mem_read<=0;
end
endcase
end
else
begin
Wait= 1;
data_io<= 11;
OutnotIn<=1;
end
end

6'b000010 ://sw
begin
if (B_wait ==0)
begin
case(counter)
0:
begin
DMA_req<= 0; 
rd<= 0;
write<= 0;
Alu_src<= 1;
mem_reg<= 0;
data_io<= 00;
Wait<= 1;
OutnotIn<=1;
mem_write<=1;
mem_read<=0;
counter<=counter+1;
end
1:
begin
DMA_req<= 0; 
rd<= 0;
write<= 0;
Alu_src<= 1;
mem_reg<= 0;
data_io<= 01;
Wait<= 0;
OutnotIn<=1;
mem_write<=1;
mem_read<=0;
counter<=counter+1;
end
2:
begin
DMA_req<= 0; 
rd<= 0;
write<= 0;
Alu_src<= 1;
mem_reg<= 0;
data_io<= 01;
Wait<= 0;
OutnotIn<=1;
mem_write<=0;
mem_read<=0;
counter<=counter+1;
counter<=0;

end
endcase
end
else
begin
Wait<= 1;
data_io<= 11;
OutnotIn<=1;
end
end


6'b000011://addi
begin
DMA_req<= 0;
Wait<= 0;
rd<= 0;
write<= 1;
Alu_src<= 1;
mem_reg<= 0;
data_io<= 11;
OutnotIn<=1;
mem_write<=0;
mem_read<=0;

end


6'b000100://copy
begin
mem_write<=0;
mem_read<=0;
if (B_wait ==0)
begin

case(counter)
0:
begin
DMA_req<= 1;
DMA_mode<= 0;
Wait<= 1;
write<= 0;
mem_reg<= 0;
data_io<= 11;
OutnotIn<=1;
counter<=counter+1;
end
1:
begin
Wait<= 1;
data_io<= 01;
OutnotIn<=1;
counter<=counter+1;

end
2:
begin
Wait<= 1;
data_io<= 00;
OutnotIn<=1;
counter<=counter+1;

end
3:
begin
Wait<= 0;
data_io<= 10;
OutnotIn<=1;
counter<=counter+1;
counter<=0;
end



endcase
end
else
begin
Wait= 1;
data_io<= 11;
OutnotIn<=1;
end
end

6'b000101://send
begin
mem_write<=0;
mem_read<=0;
if (B_wait ==0)
begin
case(counter)
0:
begin
DMA_req<= 1;
DMA_mode<= 1;
Wait<= 1;
write<= 0;
mem_reg<= 0;
data_io<=11;
OutnotIn<=1;
counter<=counter+1;
end
1:
begin
Wait<= 1;
data_io<= 01;
OutnotIn<=1;
counter<=counter+1;
end
2:
begin
Wait<= 0;
data_io<= 00;
OutnotIn<=1;
counter<=counter+1;
counter<=0;

end

endcase
end
else
begin
Wait= 1;
data_io<= 11;
OutnotIn<=1;
end




end
6'bxxxxxx:
begin
data_io<= 11;
OutnotIn<=1;
DMA_req<= 0;
mem_write<=0;
mem_read<=0;
end
endcase

end
endmodule











module pc(out,clk,Wait);
input clk,Wait ; 
output reg [31:0] out;
initial 
begin
out=0;
end

always@(posedge clk)
begin 
if(Wait==0)
begin
out=out+1; 
end
else 
begin
out=out;
end
end
endmodule 




module MUX_process(o1,in1,in2,sel);
output reg [31:0] o1;
input [31:0] in1,in2;
input sel;
always@(in1,in2,sel)
begin
if(sel==0)
o1=in1;
else if(sel==1)
o1=in2;
else
o1=1'bx;

end
endmodule
module MUX4in(o1,in1,in2,in3,in4,sel);
output reg [31:0] o1;
input [31:0] in1,in2,in3,in4;
input wire[1:0] sel;
always@(in1,in2,in3,in4,sel)
begin
if(sel==2'b00)
o1=in1;
else if(sel==2'b01)
o1=in2;
else if (sel==2'b10)
o1=in3;
else 
o1 = in4;
end
endmodule
module adder_process(in1,in2,out);
input [31:0] in1,in2;
output reg [31:0] out;
always @(in1,in2)
begin
out<=in1+in2;

end
endmodule
module signextend (o1,in1);

input [15:0] in1;
output reg [31:0] o1;
wire [15:0] w0 = 16'b0000000000000000;
wire [15:0] w1 = 16'b1111111111111111;

always@(in1)


if(in1[15] == 1'b0)
 o1 = {w0,in1};

else if (in1[15] == 1'b1)
 o1 = {w1,in1};
else
o1=32'bx;


endmodule

module InOut_procc(a,b,out, OutNotIn);
input OutNotIn;
input [31:0] out;
inout [31:0] a;
output wire[31:0] b;
assign a = (OutNotIn)? out : 32'bzzzz;  //a is in output mode 
assign b = (OutNotIn)? 32'bzzzz : a; //a is in input mode
endmodule




module buscontroller(input interrupt ,input dma_req ,input done,input dma_mode,input bus_request, output reg Wait,output reg Dma_mode, output reg dma_interrupt ,output reg chipselect);
initial
begin
Wait<=0;
chipselect<=0;
end
always @(*)
begin
if (bus_request==1)
begin
assign Wait = 1;
end
if ( interrupt ==1)
begin

assign Wait = 1;
 dma_interrupt<=1;   
chipselect <= 1 ;
end
if(bus_request==0)
begin
assign Wait = 0 ;
dma_interrupt <= 0 ;
chipselect <= 0 ;
end
if (done == 1 )
begin 
Wait <= 0 ;
dma_interrupt <= 0 ;
chipselect <= 0 ;

end
 if (dma_req ==1)
begin
Wait <= 0;
chipselect<=1;
dma_interrupt<=0;
Dma_mode <= dma_mode;
end
end
endmodule

module main_memory( input write , input read ,input clk, input [31:0] data_in, output reg [31:0] data_out, output reg OutNotIn );
reg [31:0] address;
reg [31:0] mem[0:1023];
integer i;
reg[31:0] data;
reg counter;
initial
begin 
$monitor("%t, output= %d",$time,mem[1022]);
//$monitor("%t, data_in= %b , counter=%b , address= %b ,read=%b , write=%b, ",$time,data_in,counter,address,read,write);
OutNotIn<=0;
address<=0;
for(i=0;i<1024;i=i+1)
begin
 mem[i]=i;

end
counter<=0;

end

always@( posedge clk,read,write,data_in)
begin 
if(read==1&& write==0)
begin
case(counter)
0:
begin
address<=data_in;

counter<=counter+1;
OutNotIn=0;
end
1:
begin
data_out<=mem[address];
counter<=counter+1;
OutNotIn=1;
end
2:
begin
data_out<=mem[address];
counter<=0;
OutNotIn=1;
end
endcase
end
else if (write==1&&read==0)
begin 
case(counter)
0:
begin
address <=data_in;
#1
counter<=counter+1;
OutNotIn=0;
end
1:
begin
mem[address]<=data_in;
counter<=0;
OutNotIn<=0;
#1
OutNotIn<=1;
end
endcase
end
else 
begin
OutNotIn=1;
data_out<='bz;
end
end
endmodule 


module memory_file( write, read,  data_add,clk);
input read,write,clk;
inout [31:0] data_add;
wire [31:0] data_in,data_out;
wire OutNotIn;


main_memory mem1( write ,  read , clk, data_in, data_out, OutNotIn );


InOut  in_out_mem (data_add,data_in,data_out, OutNotIn);
initial
begin

//$monitor("%t, data_bus= %b, data_in= %b , data_out= %b, read= %b , write=%b ", $time,data_add,  data_in, data_out, read, write);
end
endmodule



module tb_DMA_procc_mem_BC_5ales_Awe();
wire  [31:0] data_add;
wire clk,done,bus_req,mode_1,chipselect,mem_write_1,mem_read_1,mode_2,B_wait,mem_write_3,mem_write_2,mem_read_3,mem_read_2,dma_interrupt;
reg input_interrupt;

DMA DMA_1(clk,data_add,done,bus_req,dma_interrupt,mode_2,mem_write_2,mem_read_2,chipselect);
processor processor_1(DMA_req,mode_1, B_wait,data_add,clk,mem_write_3,mem_read_3);

memory_file memory1( mem_write_1, mem_read_1, data_add,clk);
or(mem_write_1,mem_write_2,mem_write_3);
or(mem_read_1,mem_read_3,mem_read_2);

buscontroller BC_1( input_interrupt , DMA_req , done, mode_1, bus_req, B_wait, mode_2,  dma_interrupt , chipselect);

clockgen clk_1(clk);

initial 
begin
//$monitor("%t ,mode= %b, DMA_inout= %b ,B_wait= %b, bus_req= %b , done= %b ,interrupt= %b,mem_write=%b , mem_read=%b,select=%b", $time , mode_2, data_add,B_wait, bus_req , done,dma_interrupt,mem_write_1,mem_read_1,chipselect);

input_interrupt=0;


end

endmodule 

