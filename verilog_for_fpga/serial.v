`timescale 1ns / 1ps

parameter UART_SPEED = 115200;
parameter CLK_FREQUENCY = 14745600;

module uart_tx(input clk, input [7:0] tx_byte, input go, output busy, output tx);
    
    parameter baud_div_width = $clog2(CLK_FREQUENCY / UART_SPEED);
    
    // Sending is easier than transmitting because there's no need to oversample.
    // Just generate a tick at the baud rate, and when go is sent load the bit to be transmitted.
    // the +1 is to take the carry output of the adder as the tick signal
    reg [baud_div_width:0] baud_div = 0;
    wire tick = baud_div[baud_div_width];
    always @ (posedge clk)
        if (go)
            baud_div <= 0;
        else
            baud_div <= baud_div[baud_div_width-1:0] + 1;

    reg [3:0] busy_counter = 0;
    assign busy = busy_counter != 0;
    always @ (posedge clk) if (go) busy_counter <= 10; else if (busy_counter != 0) if (tick) busy_counter <= busy_counter - 1;

    reg [9:0] tx_pattern = 10'b1111111111;
    assign tx = tx_pattern[0];
    always @ (posedge clk) begin
        if (go) tx_pattern <= {1'b1, tx_byte, 1'b0}; // load the output shift register
        else if (tick) tx_pattern <= {1'b1, tx_pattern[9:1]}; // shift out one bit at a time every baud period
    end

endmodule

module uart_rx(input clk, input rx, output rx_done, output reg [7:0] rx_byte = 0);

    wire start;
    parameter oversample_width = $clog2(CLK_FREQUENCY / UART_SPEED / 16);
    
    // From a 14.74 clock, divide by 8 to gen 16x oversample for 115200.
    reg [oversample_width:0] clk16 = 0;
    wire tick16 = clk16[oversample_width];
    always @ (posedge clk) clk16 <= clk16[oversample_width-1:0] + 1;

    // From the 16x oversample tick, divide by 16 to generate the bit sample.
    // (This ticker has the ability to start halfway so that after a start is detected, subsequent ticks
    //  line up halfway in the middle of the next bit.)
    reg [4:0] clkbit = 0;
    always @ (posedge clk) if (start) clkbit <= 5'b01000; else if (tick16) clkbit <= clkbit[3:0] + 1;
    wire tickbit = clkbit[4];

    // From the bit clock, use a shift register to generate the byte done signal.
    reg [10:0] byte_shifter = 11'b10000000000;
    always @ (posedge clk) if (start) byte_shifter <= 1; else if (tick16) if (tickbit) if (!byte_shifter[10]) byte_shifter <= {byte_shifter[9:0], 1'b0};
    assign rx_done = byte_shifter[9]; // Done one bit early because RS232 stop byte.

    // A synchronizer avoids metastability on the asynchronously changing rx input signal.
    reg [1:0] rx_sync = 0;
    always @ (posedge clk) rx_sync <= {rx_sync[0], rx};
    wire rx_synced = rx_sync[1];

    // Detect a falling edge on the rx signal to know when to start receiving a byte.
    reg rx_previous = 0;
    always @ (posedge clk) rx_previous <= rx_synced;
    wire falling_edge = rx_previous & ~rx_synced;

    // Start receiving a byte if the last byte is finished being received (one shifted into last place of shift register) and a falling edge occured.
    assign start = byte_shifter[10] & falling_edge;

    // And that's it. When idle, a falling edge will reset the clkbit to half its period so the next tick
    // lines up with the middle of the next bit, and also reset the shift register to go for 10 counts before stopping.
    // Now all that's needed is to accumulate those bits into a byte with another shift register.
    always @ (posedge clk) if (tick16) if (tickbit) rx_byte <= {rx_synced, rx_byte[7:1]};

endmodule
