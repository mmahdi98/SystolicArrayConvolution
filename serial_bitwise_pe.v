module semimux(out,a,b,sel,selNot);
    output out;
    input a,b,sel,selNot;
    and and1(aAndSelNot,selNot,a);
    and and2(bAndSel,b,sel);
    or or1(out,bAndSel,aAndSelNot);
endmodule
module mux(out,a,b,sel);
    output out;
    input a,b,sel;
    not not1(selNot,sel);
    semimux m1(out,a,b,sel,selNot);
endmodule
module dff(q,clk,reset,d);
    output q;
    input clk,d,reset;
    reg q;
    always @(posedge clk) begin
        q<=d&(~reset);
    end
endmodule
module storage_6(wOut,wIn,clk,reset,ctrl);
    input wIn,clk,reset,ctrl;
    output [5:0] wOut;
    reg [5:0] Regs;
    assign wOut[0] = Regs[0];
    assign wOut[1] = Regs[1];
    assign wOut[2] = Regs[2];
    assign wOut[3] = Regs[3];
    assign wOut[4] = Regs[4];
    assign wOut[5] = Regs[5];
    always @(posedge clk)begin
        if(ctrl) begin
            Regs[0] <= Regs[1];
            Regs[1] <= Regs[2];
            Regs[2] <= Regs[3];
            Regs[3] <= Regs[4];
            Regs[4] <= Regs[5];
            Regs[5] <= wIn;
        end
    end
endmodule
module serialadder(sum,clk,a,b,sign,ctrl);
    output sum;
    input a,b,clk,sign,ctrl;
    mux m1(toReg,c,sign,ctrl);
    reg q,x,y;
    assign {c,sum} = x+y+q;
    always @(posedge clk) begin
        x <= a;
        y <= b;
        q <= toReg;
    end
endmodule
module clkout(clk);
	output clk;
	reg clk;
	initial begin
		clk=1'b0;
		forever #5 clk=~clk;
	end
endmodule
module test_serialadder;
	reg reset,a,b,sign,ctrl;
	serialadder se1(sum,clk,a,b,sign,ctrl);
	clkout y(clk);
	initial	begin 
		reset=1'b1;
		a=1'b1;
		b=1'b0;
		sign=1'b1;
		ctrl=1'b1;
		#5 reset=1'b0;
		#5 ctrl=1'b0;
		#5 a=1'b1;
		b=1'b1;
		#5 a=1'b0;
		b=1'b0;
		//#5 $finish;
	end
	//always @(posedge clk) $display("sum: %b",sum);
endmodule
module shiftreg_8_serial(out,in,clk,ctrl);
    output out;
    input in,clk,ctrl;
    reg [7:0] Regs;
    assign out = Regs[0];
    always @(posedge clk) begin
        if(!ctrl) begin
            Regs[0]<=Regs[1];
            Regs[1]<=Regs[2];
            Regs[2]<=Regs[3];
            Regs[3]<=Regs[4];
            Regs[4]<=Regs[5];
            Regs[5]<=Regs[6];
            Regs[6]<=Regs[7];
            Regs[7]<=in;
        end
    end
endmodule
module shiftreg_15_serial(out,in,clk);
    output out;
    input in,clk,reset;
    reg [14:0] Regs;
    assign out = Regs[0];
    always @(posedge clk) begin
        Regs[0]<=Regs[1];
        Regs[1]<=Regs[2];
        Regs[2]<=Regs[3];
        Regs[3]<=Regs[4];
        Regs[4]<=Regs[5];
        Regs[5]<=Regs[6];
        Regs[6]<=Regs[7];
        Regs[7]<=Regs[8];
        Regs[8]<=Regs[9];
        Regs[9]<=Regs[10];
        Regs[10]<=Regs[11];
        Regs[11]<=Regs[12];
        Regs[12]<=Regs[13];
        Regs[13]<=Regs[14];
        Regs[14]<=in;
    end
endmodule
module multiplier(out,xIn,w,clk,reset);
    output out;
    input xIn,reset,clk;
    input [4:0] w;
    wire [4:1] toFF;
    wire [4:1] FFout;
    mux m5(toFF[4],1'b0,xIn,w[4]);
    dff d5(FFout[4],clk,reset,toFF[4]);
    mux m4(toFF[3],FFout[4],xIn,w[3]);
    dff d4(FFout[3],clk,reset,toFF[3]);
    mux m3(toFF[2],FFout[3],xIn,w[2]);
    dff d3(FFout[2],clk,reset,toFF[2]);
    mux m2(toFF[1],FFout[2],xIn,w[1]);
    dff d2(FFout[1],clk,reset,toFF[1]);
    mux m1(out,FFout[1],xIn,w[0]);
endmodule
module testserialreg;
    reg [7:0] in;
    reg inp;
    clkout y(clk);
    reg reset, ctrl;
    shiftreg_8_serial Reg1(out,inp,clk,ctrl);
    initial begin
        ctrl=1'b0;
        reset=1'b1;
        in=8'b011001x0;
        #10 inp=in[0];reset=1'b0;
        #10 inp=in[1];
        #10 inp=in[2];
        #10 inp=in[3];
        #10 inp=in[4];
        #10 inp=in[5];
        #10 inp=in[6];
        #10 inp=in[7];
        #10 inp=in[1];
        #10 inp=in[2];
        #10 inp=in[3];
        #10 inp=in[4];
        #10 inp=in[5];
        #10 inp=in[6];
        #10 inp=in[7];
        #10 ctrl=1'b1;inp=1'bx;
        #10 inp=1'bx;
        #10;
        // $finish;
    end
    // always @(posedge clk) $display("out:%b",out);
endmodule
module testbench_bit;
    //output xOut,yOut;
    reg inxw,iny;
    reg [2:0] ctrl;
    clkout clock(clk);
    reg [5:0] W;
    reg [7:0] X,X2;
    reg [15:0] Y,Y2;
    initial begin
        //1: 25461   14 13 9 8 6 5 4 2 0 16'b0110001101110101 -14 12 11 5 2 0 16'b0101100000100101
        //2: 45610 15 13 12 9 5 3 1 16'b1011001000101010         -15 13 9 8 5 3 1 16'b1010001100101010
        W=6'b101000;//8 -8
        X=8'b10110101;//181
        X2=8'b11110000;//240
        Y=16'b0101110111001101;//24013
        Y2=16'b1010101010101010;//43690
        ctrl=3'b001;//ctrl[2]:LSB X , ctrl[1]:citculate, ctrl[0]: LoadW
        iny=1'b0;
        inxw=W[0];
        #10 inxw=W[1];
        #10 inxw=W[2];
        #10 inxw=W[3];
        #10 inxw=W[4];
        #10 inxw=W[5];
        #10 $display("inserting first X");ctrl=3'b100;inxw=X[0];iny=Y[0];
        #10 inxw=X[1];iny=Y[1];ctrl[2]=1'b0;//ctrl=000
        #10 inxw=X[2];iny=Y[2];
        #10 inxw=X[3];iny=Y[3];
        #10 inxw=X[4];iny=Y[4];
        #10 inxw=X[5];iny=Y[5];
        #10 inxw=X[6];iny=Y[6];
        #10 inxw=X[7];iny=Y[7];
        #10 ctrl=3'b010;
        iny=Y[8];
        #10 iny=Y[9];
        #10 iny=Y[10];
        #10 iny=Y[11];
        #10 iny=Y[12];
        #10 iny=Y[13];
        #10 iny=Y[14];
        #10 iny=Y[15];
        #10 $display("inserting second X");ctrl=3'b100;inxw=X2[0];iny=Y2[0];
        #10 inxw=X2[1];iny=Y2[1];ctrl[2]=1'b0;
        #10 inxw=X2[2];iny=Y2[2];
        #10 inxw=X2[3];iny=Y2[3];
        #10 inxw=X2[4];iny=Y2[4];
        #10 inxw=X2[5];iny=Y2[5];
        #10 inxw=X2[6];iny=Y2[6];
        #10 inxw=X2[7];iny=Y2[7];
        #10 ctrl=3'b010;
        iny=Y2[8];
        #10 iny=Y2[9];
        #10 iny=Y2[10];
        #10 iny=Y2[11];
        #10 iny=Y2[12];
        #10 iny=Y2[13];
        #10 iny=Y2[14];
        #10 iny=Y2[15];#10 $display("inserting third X");ctrl=3'b100;inxw=X[0];iny=Y[0];
        #10 inxw=X[1];iny=Y[1];ctrl[2]=1'b0;
        #10 inxw=X[2];iny=Y[2];
        #10 inxw=X[3];iny=Y[3];
        #10 inxw=X[4];iny=Y[4];
        #10 inxw=X[5];iny=Y[5];
        #10 inxw=X[6];iny=Y[6];
        #10 inxw=X[7];iny=Y[7];
        #10 ctrl=3'b010;
        iny=Y[8];
        #10 iny=Y[9];
        #10 iny=Y[10];
        #10 iny=Y[11];
        #10 iny=Y[12];
        #10 iny=Y[13];
        #10 iny=Y[14];
        #10 iny=Y[15];
        #10 $display("inserting don't care");ctrl=3'b100;inxw=1'bx;
        #400 $finish;
    end
    always @(posedge clk) $display("X:%b ,Y:%b",xOut,yOut);
    serial_bitwise_pe pe1(xOut,yOut,inxw,iny,ctrl,clk);
endmodule
module serial_bitwise_pe(xOut,yOut,xOrW,yIn,ctrl,clk);
    output yOut,xOut;
    input xOrW,yIn,clk;
    input [2:0] ctrl;
    wire [5:0] WtoMul;
    wire ctrl_not;
    not n1(ctrl_not,ctrl[0]);
    mux m1(IN,xOrW,1'b0,ctrl[1]);
    shiftreg_8_serial Reg1(Reg1toReg2,IN,clk,ctrl[1]);
    shiftreg_8_serial Reg2(xOut,Reg1toReg2,clk,ctrl[1]);
    storage_6 WStorage(WtoMul,IN,clk,1'b0,ctrl[0]);
    multiplier Mul(toComp,IN,WtoMul[4:0],clk,ctrl[0]);
    xor x1(toAdder,toComp,WtoMul[5]);
    serialadder Adder(to15Reg,clk,toAdder,yIn,WtoMul[5],ctrl[2]);
    shiftreg_15_serial Shift15(yOut,to15Reg,clk);
endmodule
module testWStorage;
    reg Reg,ctrl,reset;
    reg [5:0]inp;
    wire [5:0] out;
    clkout y(clk);
    storage_6 WStorage(out,Reg,clk,reset,ctrl);
    initial begin
        inp=6'b100001;
        reset=1'b1;
        ctrl=1'b0;
        Reg=1'bx;
        #10 reset=1'b0;ctrl=0;
        #10 Reg=inp[0];
        #10 ctrl=1'b1;
        #10 Reg=inp[1];
        #10 Reg=inp[2];
        #10 Reg=inp[3];
        #10 Reg=inp[4];
        #10 Reg=inp[5];
        #10 ctrl=1'b0;Reg=1'bx;
        //#20 $finish;
    end
    //always @(posedge clk) $display("out: %b",out);
endmodule
