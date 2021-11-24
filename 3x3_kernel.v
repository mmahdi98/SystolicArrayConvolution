//in the name of God
module Reg8_1(out,in,clk,ctrl);
    input ctrl;
    output [7:0]out;
    input [7:0]in;
    reg [7:0] Regs;
    input clk;
    assign out[0] = Regs[0];
    assign out[1] = Regs[1];
    assign out[2] = Regs[2];
    assign out[3] = Regs[3];
    assign out[4] = Regs[4];
    assign out[5] = Regs[5];
    assign out[6] = Regs[6];
    assign out[7] = Regs[7];
    always @(posedge clk)begin
        if(!ctrl)begin
            Regs[0]<=in[0];
            Regs[1]<=in[1];
            Regs[2]<=in[2];
            Regs[3]<=in[3];
            Regs[4]<=in[4];
            Regs[5]<=in[5];
            Regs[6]<=in[6];
            Regs[7]<=in[7];
        end
    end
endmodule
module Reg8_2(out,in,clk);
    output [7:0]out;
    input [7:0]in;
    reg [7:0] Regs;
    input clk;
    assign out[0] = Regs[0];
    assign out[1] = Regs[1];
    assign out[2] = Regs[2];
    assign out[3] = Regs[3];
    assign out[4] = Regs[4];
    assign out[5] = Regs[5];
    assign out[6] = Regs[6];
    assign out[7] = Regs[7];
    always @(posedge clk)begin
        Regs[0]<=in[0];
        Regs[1]<=in[1];
        Regs[2]<=in[2];
        Regs[3]<=in[3];
        Regs[4]<=in[4];
        Regs[5]<=in[5];
        Regs[6]<=in[6];
        Regs[7]<=in[7];
    end
endmodule
module Multiplier(out,in,s_amt);
    output [15:0] out;
    input [7:0] in;
    input [2:0] s_amt;
    wire [8:0] w;
    wire [10:0] w2;
    wire [3:0] s_amtNot;
    not n1(s_amtNot[0],s_amt[0]);
    not n2(s_amtNot[1],s_amt[1]);
    not n3(s_amtNot[2],s_amt[2]);
    semimux m01(w[0],in[0],1'b0,s_amt[0],s_amtNot[0]);
    semimux m02(w[1],in[1],in[0],s_amt[0],s_amtNot[0]);
    semimux m03(w[2],in[2],in[1],s_amt[0],s_amtNot[0]);
    semimux m04(w[3],in[3],in[2],s_amt[0],s_amtNot[0]);
    semimux m05(w[4],in[4],in[3],s_amt[0],s_amtNot[0]);
    semimux m06(w[5],in[5],in[4],s_amt[0],s_amtNot[0]);
    semimux m07(w[6],in[6],in[5],s_amt[0],s_amtNot[0]);
    semimux m08(w[7],in[7],in[6],s_amt[0],s_amtNot[0]);
    semimux m09(w[8],1'b0,in[7],s_amt[0],s_amtNot[0]);
    semimux m11(w2[0],w[0],1'b0,s_amt[1],s_amtNot[1]);
    semimux m12(w2[1],w[1],1'b0,s_amt[1],s_amtNot[1]);
    semimux m13(w2[2],w[2],w[0],s_amt[1],s_amtNot[1]);
    semimux m14(w2[3],w[3],w[1],s_amt[1],s_amtNot[1]);
    semimux m15(w2[4],w[4],w[2],s_amt[1],s_amtNot[1]);
    semimux m16(w2[5],w[5],w[3],s_amt[1],s_amtNot[1]);
    semimux m17(w2[6],w[6],w[4],s_amt[1],s_amtNot[1]);
    semimux m18(w2[7],w[7],w[5],s_amt[1],s_amtNot[1]);
    semimux m19(w2[8],w[8],w[6],s_amt[1],s_amtNot[1]);
    semimux m110(w2[9],1'b0,w[7],s_amt[1],s_amtNot[1]);
    semimux m111(w2[10],1'b0,w[8],s_amt[1],s_amtNot[1]);
    semimux m21(out[0],w2[0],1'b0,s_amt[2],s_amtNot[2]);
    semimux m22(out[1],w2[1],1'b0,s_amt[2],s_amtNot[2]);
    semimux m23(out[2],w2[2],1'b0,s_amt[2],s_amtNot[2]);
    semimux m24(out[3],w2[3],1'b0,s_amt[2],s_amtNot[2]);
    semimux m25(out[4],w2[4],w2[0],s_amt[2],s_amtNot[2]);
    semimux m26(out[5],w2[5],w2[1],s_amt[2],s_amtNot[2]);
    semimux m27(out[6],w2[6],w2[2],s_amt[2],s_amtNot[2]);
    semimux m28(out[7],w2[7],w2[3],s_amt[2],s_amtNot[2]);
    semimux m29(out[8],w2[8],w2[4],s_amt[2],s_amtNot[2]);
    semimux m210(out[9],w2[9],w2[5],s_amt[2],s_amtNot[2]);
    semimux m211(out[10],w2[10],w2[6],s_amt[2],s_amtNot[2]);
    semimux m212(out[11],1'b0,w2[7],s_amt[2],s_amtNot[2]);
    semimux m213(out[12],1'b0,w2[8],s_amt[2],s_amtNot[2]);
    semimux m214(out[13],1'b0,w2[9],s_amt[2],s_amtNot[2]);
    semimux m215(out[14],1'b0,w2[10],s_amt[2],s_amtNot[2]);
    assign out[15] = 1'b0;
endmodule
module testmul;
    reg [2:0] Shift_amt;
    reg [7:0] in;
    wire [15:0] out;
    Multiplier m1(out,in,Shift_amt);
    initial begin
        Shift_amt=3'b011;
        in=8'b11101011;
    end
endmodule
module Complement(out,in,sign);
    input sign;
    input [15:0] in;
    output [15:0] out;
    xor x1(out[0],in[0],sign);
    xor x2(out[1],in[1],sign);
    xor x3(out[2],in[2],sign);
    xor x4(out[3],in[3],sign);
    xor x5(out[4],in[4],sign);
    xor x6(out[5],in[5],sign);
    xor x7(out[6],in[6],sign);
    xor x8(out[7],in[7],sign);
    xor x9(out[8],in[8],sign);
    xor x10(out[9],in[9],sign);
    xor x11(out[10],in[10],sign);
    xor x12(out[11],in[11],sign);
    xor x13(out[12],in[12],sign);
    xor x14(out[13],in[13],sign);
    xor x15(out[14],in[14],sign);
    xor x16(out[15],in[15],sign);
endmodule
module testcomp;
    reg [15:0] in;
    reg sign;
    wire [15:0] out;
    Complement cmp(out,in,sign);
    initial begin
        // in=16'b0110101111010011;
        // sign = 1'b1;
        // #10 $display ("out: %b",out);
        // sign = 1'b0;
        // #10 $display ("out: %b",out);
        // #10 $finish;
    end
endmodule
module Mux8(out,in1,in2,sel);
    input [7:0] in1,in2;
    input sel;
    output [7:0] out;
    not n1(selNot,sel);
    semimux m1(out[0],in1[0],in2[0],sel,selNot);
    semimux m2(out[1],in1[1],in2[1],sel,selNot);
    semimux m3(out[2],in1[2],in2[2],sel,selNot);
    semimux m4(out[3],in1[3],in2[3],sel,selNot);
    semimux m5(out[4],in1[4],in2[4],sel,selNot);
    semimux m6(out[5],in1[5],in2[5],sel,selNot);
    semimux m7(out[6],in1[6],in2[6],sel,selNot);
    semimux m8(out[7],in1[7],in2[7],sel,selNot);
endmodule
module SerialAdder(sum,clk,a,b,sign,ctrl);
    output [7:0] sum;
    input [7:0] a,b;
    reg [7:0] Reg1,Reg2;
    input clk,sign,ctrl;
    wire [7:0] c;
    reg q;
    assign {c[0],sum[0]}=Reg1[0]+Reg2[0]+q;
    assign {c[1],sum[1]}=Reg1[1]+Reg2[1]+c[0];
    assign {c[2],sum[2]}=Reg1[2]+Reg2[2]+c[1];
    assign {c[3],sum[3]}=Reg1[3]+Reg2[3]+c[2];
    assign {c[4],sum[4]}=Reg1[4]+Reg2[4]+c[3];
    assign {c[5],sum[5]}=Reg1[5]+Reg2[5]+c[4];
    assign {c[6],sum[6]}=Reg1[6]+Reg2[6]+c[5];
    assign {c[7],sum[7]}=Reg1[7]+Reg2[7]+c[6];
    mux m1(toReg,c[7],sign,ctrl);
    always @(posedge clk)begin
        Reg1[0] <= a[0];
        Reg1[1] <= a[1];
        Reg1[2] <= a[2];
        Reg1[3] <= a[3];
        Reg1[4] <= a[4];
        Reg1[5] <= a[5];
        Reg1[6] <= a[6];
        Reg1[7] <= a[7];
        Reg2[0] <= b[0];
        Reg2[1] <= b[1];
        Reg2[2] <= b[2];
        Reg2[3] <= b[3];
        Reg2[4] <= b[4];
        Reg2[5] <= b[5];
        Reg2[6] <= b[6];
        Reg2[7] <= b[7];
        q <= toReg;
    end
endmodule
// module testadder;
//     reg [7:0] in1,in2;
//     reg sign, ctrl;
//     wire [7:0] out;
//     Clock c(clk);
//     SerialAdder Add(out,clk,in1,in2,sign,ctrl);
//     initial begin
//         ctrl=1'b1;
//         sign=1'b1;
//         in1=8'b00101101;
//         in2=8'b11101111;
//         #10 ctrl=1'b0;in1=8'b00000000;in2=8'b00000000;
//         #10 in1=8'bxxxxxxxx;
//         #20 $finish;
//     end
//     always @(posedge clk) $display("out: %b",out);
// endmodule
module mux(out,a,b,sel);
    output out;
    input a,b,sel;
    not not1(selNot,sel);
    semimux m1(out,a,b,sel,selNot);
endmodule
module semimux(out,a,b,sel,selNot);
    output out;
    input a,b,sel,selNot;
    and and1(aAndSelNot,selNot,a);
    and and2(bAndSel,b,sel);
    or or1(out,bAndSel,aAndSelNot);
endmodule
module Reg_4(out,in,clk,ctrl);
    output [3:0] out;
    input [3:0] in;
    input clk,ctrl;
    reg [3:0] Regs;
    assign out[0] = Regs[0];
    assign out[1] = Regs[1];
    assign out[2] = Regs[2];
    assign out[3] = Regs[3];
    always @(posedge clk)begin
        if(ctrl)begin
            Regs[0] <= in[0];
            Regs[1] <= in[1];
            Regs[2] <= in[2];
            Regs[3] <= in[3];
        end
    end
endmodule
module PE_Conv(yOut,xOut,xOrW,yIn,clk,ctrl);
    input [7:0] xOrW,yIn;
    input clk;
    input [2:0] ctrl;//0:StoreW, 1:LSB, 2:Circulate
    output [7:0] yOut,xOut;
    wire [7:0] Reg1toReg2, Reg3toMux,MuxToAdder,AdderToReg4;
    wire [15:0] MulOut,CompOut;
    wire [3:0] Shift_amt;
    Reg8_1 Reg1(Reg1toReg2,xOrW,clk,ctrl[2]);
    Reg8_1 Reg2(xOut,Reg1toReg2,clk,ctrl[2]);
    Reg_4 WStorage (Shift_amt,xOrW[3:0],clk,ctrl[0]);
    Multiplier Mul(MulOut,xOrW,Shift_amt[2:0]);
    Complement Cmp(CompOut,MulOut,Shift_amt[3]);
    Reg8_2 Reg3(Reg3toMux,CompOut[15:8],clk);
    Mux8 M2(MuxToAdder,Reg3toMux,CompOut[7:0],ctrl[1]);
    SerialAdder Adder(AdderToReg4,clk,MuxToAdder,yIn,Shift_amt[3],ctrl[1]);
    Reg8_2 Reg4(yOut,AdderToReg4,clk);
endmodule
module Clock(clk);
	output clk;
	reg clk;
	initial begin
		clk=1'b0;
		forever #5 clk=~clk;
	end
endmodule
module PE_Conv_test;
    reg [7:0] xOrW,yIn;
    reg [2:0] ctrl;
    wire [7:0] yOut,xOut;
    PE_Conv PE(yOut,xOut,xOrW,yIn,clk,ctrl);
    Clock c(clk);
    reg [3:0] W;
    reg [15:0] X1,X2,X3;
    reg [15:0] Y1,Y2,Y3;
    initial begin
        W=4'b1011;
        X1=8'b10110101;
        X2=8'b11110000;
        X3=8'b11001100;
        Y1=16'b0101110111001101;
        Y2=16'b1010101010101010;
        Y3=16'b1100110011001100;
        ctrl=3'b001;
        yIn=1'b0;
        xOrW[3:0]=W;
        xOrW[7:4]=4'b0000;
        //#10 $display("inserting X1");ctrl=3'b010;xOrW=X1;yIn=Y1[7:0];
        #10 ctrl=3'b100;yIn=Y1[15:8];
        //#10 $display("inserting X2");ctrl=3'b010;xOrW=X2;yIn=Y2[7:0];
        #10 ctrl=3'b100;yIn=Y2[15:8];
        //#10 $display("inserting X3");ctrl=3'b010;xOrW=X3;yIn=Y3[7:0];
        #10 ctrl=3'b100;yIn=Y3[15:8];
        //#10 $display("inserting X4");ctrl=3'b010;xOrW=X1;yIn=Y1[7:0];
        #10 ctrl=3'b100;yIn=Y1[15:8];
        //#10 $display("inserting X5");ctrl=3'b010;xOrW=X2;yIn=Y2[7:0];
        #10 ctrl=3'b100;yIn=Y2[15:8];
        //#10 $display("inserting X6");ctrl=3'b010;xOrW=X3;yIn=Y3[7:0];
        #10 ctrl=3'b100;yIn=Y3[15:8];
        //#10 $display("inserting don't care");ctrl=3'b010;xOrW=8'bxxxxxxxx;yIn=8'bxxxxxxxx;
        //#400 $finish;
    end
    //always @(posedge clk) $display("X:%b ,Y:%b",xOut,yOut);
endmodule
module fulladder_8bit(sum,in1,in2,ctrl,clk);
    output [7:0] sum;
    input [7:0] in1,in2;
    input ctrl,clk;
    wire [7:0] c;
    reg q;
    mux m(carry,1'b0,q,ctrl);
    assign {c[0],sum[0]}=in1[0]+in2[0]+carry;
    assign {c[1],sum[1]}=in1[1]+in2[1]+c[0];
    assign {c[2],sum[2]}=in1[2]+in2[2]+c[1];
    assign {c[3],sum[3]}=in1[3]+in2[3]+c[2];
    assign {c[4],sum[4]}=in1[4]+in2[4]+c[3];
    assign {c[5],sum[5]}=in1[5]+in2[5]+c[4];
    assign {c[6],sum[6]}=in1[6]+in2[6]+c[5];
    assign {c[7],sum[7]}=in1[7]+in2[7]+c[6];
    always @(posedge clk) q<=c[7];
endmodule
module Result3(Yaux,Y1,Y2,Y3,res,ctrl,clk);
    input [7:0] Yaux,Y1,Y2,Y3;
    input clk,ctrl;
    output [7:0] res;
    wire [7:0] sum1,sum2;
    fulladder_8bit a1(sum1,Yaux,Y1,ctrl,clk);
    fulladder_8bit a2(sum2,Y2,Y3,ctrl,clk);
    fulladder_8bit a3(res,sum1,sum2,ctrl,clk);
endmodule
module testbench_res3;
    reg [7:0] Yaux,Y1,Y2,Y3;
    reg ctrl;
    reg [15:0] ya,y1,y2,y3;
    Clock c(clk);
    wire [7:0] out;
    Result3 r(Yaux,Y1,Y2,Y3,out,ctrl,clk);
    initial begin
        ctrl=1'b0;
        ya=16'b0000000011110000;
        y1=16'b0000000000001111;
        y2=16'b0100000000000100;
        y3=16'b0000001111100000;
        Yaux=ya[7:0];
        Y1=y1[7:0];
        Y2=y2[7:0];
        Y3=y3[7:0];
        #10 ctrl=1'b1;Yaux=ya[15:8];Y1=y1[15:8];Y2=y2[15:8];Y3=y3[15:8];
        #10 ctrl=1'b0;Y1=8'b00000000;Y2=8'b00000000;Y3=8'b00000000;
        // $finish;
    end
    // always @(posedge clk) $display ("ya: %b, y1: %b, y2: %b, y3: %b, out: %b",Yaux,Y1,Y2,Y3,out);
endmodule
module row3(out,xout,in,clk,ctrl);
    output [7:0] out,xout;
    input [7:0] in;
    input clk;
    input [2:0] ctrl;
    wire [7:0] xpe1to2,xpe2to3,ype1to2,ype2to3;
    PE_Conv pe1(ype1to2,xpe1to2,in,8'b00000000,clk,ctrl);
    PE_Conv pe2(ype2to3,xpe2to3,xpe1to2,ype1to2,clk,ctrl);
    PE_Conv pe3(out,xout,xpe2to3,ype2to3,clk,ctrl);
endmodule
module testrowof3;
    wire [7:0] out,xout;
    reg [2:0] ctrl;
    reg [7:0] in;
    Clock c(clk);
    row3 r1(out,xout,in,clk,ctrl);
    reg [7:0] x1,x2,x3,x4,x5,x6;
    reg [7:0] w1,w2,w3;
    initial begin
        x1=8'b00000001;
        x2=8'b00000011;
        x3=8'b00000111;
        x4=8'b00001111;
        x5=8'b00011111;
        x6=8'b00111111;
        w1=8'b00000001;
        w2=8'b00000010;
        w3=8'b00000011;
        ctrl=3'b001;
        in=w1;
        #20 in=w2;
        #20 in=w3;
        #10 ctrl=3'b010;in=x1;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x2;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x3;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x4;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x5;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x6;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x1;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x2;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x3;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x4;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x5;
        #10 ctrl=3'b100;
        #10 ctrl=3'b010;in=x6;
        #10 ctrl=3'b100;
        $finish;
    end
    always @(posedge clk) $display("in:%d, out:%d, xout:%d",in,out,xout);
endmodule
module kernel3x3(out,xout1,xout2,xout3,in1,in2,in3,Yaux,ctrl,clk);
    input [7:0] in1,in2,in3,Yaux;
    output [7:0] out,xout1,xout2,xout3;
    input clk;
    input [3:0] ctrl;
    wire [7:0] toRes1,toRes2,toRes3;
    row3 r1(toRes1,xout1,in1,clk,ctrl[2:0]);
    row3 r2(toRes2,xout2,in2,clk,ctrl[2:0]);
    row3 r3(toRes3,xout3,in3,clk,ctrl[2:0]);
    Result3 r(Yaux,toRes1,toRes2,toRes3,out,ctrl[3],clk);
endmodule
