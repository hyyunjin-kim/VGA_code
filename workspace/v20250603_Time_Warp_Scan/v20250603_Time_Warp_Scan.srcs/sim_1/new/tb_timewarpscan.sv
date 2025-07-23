`timescale 1ns / 1ps

module tb_timewarpscan();

    reg clk;
    reg rst;
    reg button;
    wire [9:0] scan_y;

    scanLineGen DUT(
        .clk(clk),
        .rst(rst),
        .button(button),
        .scan_y(scan_y)        
    );

    always #1 clk = ~clk;

    initial begin
        clk=0;
        rst = 1; 
        button = 0;
        #10;
        rst = 0;
        #10;
        button = 1;
        #10;
        button = 0;
        #300;
        button = 1;


        # 1000
        $finish;
    end
endmodule
