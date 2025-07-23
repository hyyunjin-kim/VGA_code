`timescale 1ns / 1ps


module OV7670_VGA_Display (
    // global signals
    input  logic       clk,
    input  logic       rst,
    // ov7670 signals
    output logic       ov7670_xclk,
    input  logic       ov7670_pclk,
    input  logic       ov7670_href,
    input  logic       ov7670_v_sync,
    input  logic [7:0] ov7670_data,
    // export signals
    output logic       h_sync,
    output logic       v_sync,
    output logic [3:0] f_red_port,
    output logic [3:0] f_green_port,
    output logic [3:0] f_blue_port,
    // switch
    input logic [3:0] sw_red,
    input logic [3:0] sw_green,
    input logic [3:0] sw_blue,
    input logic       sw_upscale,
    input  logic      sw_gray,
    /////////////////////////////
    input  logic      button
);
    logic        we;
    logic [16:0] wAddr;
    logic [15:0] wData;
    logic [16:0] rAddr;
    logic [15:0] rData;
    logic [ 9:0] x_pixel;
    logic [ 9:0] y_pixel;
    logic        DE;
    logic        w_rclk, rclk;
    logic        d_en;
    logic [3:0]  w_red_port, w_green_port, w_blue_port, gray_red, gray_green, gray_blue;
    logic [3:0]  red_port, green_port, blue_port;
    logic [9:0]  scan_y;
    logic [3:0]  r_freeze, g_freeze, b_freeze;
    logic freeze_we, is_scan_line;

    assign freeze_we    = (y_pixel == scan_y);     // 해당 줄일 때 freeze
    assign is_scan_line = (y_pixel == scan_y);

    pix_clk_gen U_OV7670_Clk_Gen (
        .clk (clk),
        .rst (rst),
        .pclk(ov7670_xclk)
    );
    VGA_Controller U_VGA_Controller (
        .clk(clk),
        .rst(rst),
        .rclk(w_rclk),
        .h_sync(h_sync),
        .v_sync(v_sync),
        .DE(DE),
        .x_pixel(x_pixel),
        .y_pixel(y_pixel)
    );
    OV7670_MemController U_OV7670_MemController (
        .pclk(ov7670_pclk),
        .rst(rst),
        .href(ov7670_href),
        .v_sync(ov7670_v_sync),
        .ov7670_data(ov7670_data),
        .we(we),
        .wAddr(wAddr),
        .wData(wData)
    );
    frame_buffer U_frame_buffer (
        .wclk(ov7670_pclk),
        .we(we),
        .wAddr(wAddr),
        .rclk(rclk),
        .oe(d_en),
        .wData(wData),
        .rAddr(rAddr),
        .rData(rData)
    );
    QVGA_MemController U_QVGA_MemController (
        .clk       (w_rclk),
        .x_pixel   (x_pixel),
        .y_pixel   (y_pixel),
        .DE        (DE),
        .rclk      (rclk),
        .d_en      (d_en),
        .rAddr     (rAddr),
        .rData     (rData),
        .red_port  (w_red_port),
        .green_port(w_green_port),
        .blue_port (w_blue_port),
        .sw        (sw_upscale),
        .sw_red    (sw_red),
        .sw_green  (sw_green),
        .sw_blue   (sw_blue)
    );

    grayscale_controller U_grayscale_controller (
        .red_port       (w_red_port),
        .green_port     (w_green_port),
        .blue_port      (w_blue_port),
        .gray_red_port  (gray_red),
        .gray_green_port(gray_green),
        .gray_blue_port (gray_blue)
    );

    mux_2x1 U_mux_2x1 (
        .sw             (sw_gray),
        .red_port       (w_red_port),
        .green_port     (w_green_port),
        .blue_port      (w_blue_port),
        .gray_red_port  (gray_red),
        .gray_green_port(gray_green),
        .gray_blue_port (gray_blue),
        .o_red_port     (red_port),
        .o_green_port   (green_port),
        .o_blue_port    (blue_port)
    );

    scanLineGen U_scanLineGen(
        .clk(clk),
        .rst(rst),
        .button(button),
        .scan_y(scan_y)
    );


    FreezeBuffer u_freezeBuffer (
        .clk(clk),
        .rst(rst),
        .freeze_we(freeze_we),
        .x_pixel(x_pixel),
        .y_pixel(y_pixel),
        .r_in(red_port),
        .g_in(green_port),
        .b_in(blue_port),
        .r_freeze(r_freeze),
        .g_freeze(g_freeze),
        .b_freeze(b_freeze)
    );

    FreezeMux u_freezeMux (
        .is_scan_line(is_scan_line),
        .cam_r(red_port),
        .cam_g(green_port),
        .cam_b(blue_port),
        .r_freeze(r_freeze),
        .g_freeze(g_freeze),
        .b_freeze(b_freeze),
        .r_out(f_red_port),
        .g_out(f_green_port),
        .b_out(f_blue_port)
    );

endmodule

module mux_2x1 (
    input  logic       sw,
    input  logic [3:0] red_port,
    input  logic [3:0] green_port,
    input  logic [3:0] blue_port,
    input  logic [3:0] gray_red_port,
    input  logic [3:0] gray_green_port,
    input  logic [3:0] gray_blue_port,
    output logic [3:0] o_red_port,
    output logic [3:0] o_green_port,
    output logic [3:0] o_blue_port
);
    assign o_red_port   = sw ? red_port : gray_red_port;
    assign o_green_port = sw ? green_port : gray_green_port;
    assign o_blue_port  = sw ? blue_port : gray_blue_port;

endmodule

module grayscale_controller (
    input  logic [3:0] red_port,
    input  logic [3:0] green_port,
    input  logic [3:0] blue_port,
    output logic [3:0] gray_red_port,
    output logic [3:0] gray_green_port,
    output logic [3:0] gray_blue_port
);
    reg [11:0] gray = red_port * 77 + green_port * 150 + blue_port * 29;

    assign {gray_red_port, gray_green_port, gray_blue_port} = {
        gray[11:8], gray[11:8], gray[11:8]
    };

endmodule

module scanLineGen (
    input  logic       clk,
    input  logic       rst,
    input  logic       button,
    output logic [9:0] scan_y
);
    // FSM 상태
    typedef enum logic [1:0] {
        IDLE,
        SCANNING
    } state_t;

    state_t state;
    logic button_prev;
    wire  button_rising = (button == 1'b1) && (button_prev == 1'b0);

    // 🔵 속도 조절용 카운터
    logic [19:0] delay_counter;
    logic        tick;

    // tick 생성
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            delay_counter <= 0;
            tick <= 0;
        end else begin
            if (delay_counter == 1000000) begin
                delay_counter <= 0;
                tick <= 1;
            end else begin
                delay_counter <= delay_counter + 1;
                tick <= 0;
            end
        end
    end

    // FSM + scan_y 증가
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= IDLE;
            scan_y      <= 0;
            button_prev <= 0;
        end else begin
            button_prev <= button;

            case (state)
                IDLE: begin
                    scan_y <= 0;
                    if (button_rising)
                        state <= SCANNING;
                end
                SCANNING: begin
                    if (tick) begin
                        if (scan_y < 239)
                            scan_y <= scan_y + 1;  // ✅ 여기서 줄이 내려감
                        else
                            state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule

module FreezeBuffer (
    input  logic        clk,
    input  logic        rst,
    input  logic        freeze_we,
    input  logic [9:0]  x_pixel,
    input  logic [9:0]  y_pixel,
    input  logic [3:0]  r_in,
    input  logic [3:0]  g_in,
    input  logic [3:0]  b_in,
    output logic [3:0]  r_freeze,
    output logic [3:0]  g_freeze,
    output logic [3:0]  b_freeze
);

    (* ram_style = "block" *) logic [11:0] freeze_mem [0:19199];


    logic        valid_line [0:119];
    wire [14:0] addr = y_pixel * 160 + x_pixel;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 120; i++) valid_line[i] <= 0;
        end else if (freeze_we && !valid_line[y_pixel]) begin
            freeze_mem[addr] <= {r_in, g_in, b_in};
            if (x_pixel == 159)
                valid_line[y_pixel] <= 1;
        end
    end

    assign {r_freeze, g_freeze, b_freeze} = freeze_mem[addr];

endmodule


module FreezeMux (
    input  logic        is_scan_line,
    input  logic [3:0]  cam_r,
    input  logic [3:0]  cam_g,
    input  logic [3:0]  cam_b,
    input  logic [3:0]  r_freeze,
    input  logic [3:0]  g_freeze,
    input  logic [3:0]  b_freeze,
    output logic [3:0]  r_out,
    output logic [3:0]  g_out,
    output logic [3:0]  b_out
);

    always_comb begin
        if (is_scan_line)
            {r_out, g_out, b_out} = 12'h00F;
        else
            {r_out, g_out, b_out} = {r_freeze, g_freeze, b_freeze};
    end

endmodule