module lights(
    input clk_fast,
    input clk_slow,

    input rx,
    output tx,

    output [23:0] leds,

    output [5:0] user_io,

    input ble_rxi,
    output ble_txo,

    input esp_rxi,
    output esp_txo,

    input vusb_half,
    output clk_fast_en,


    input cs_n,
    input sclk,
    input mosi,
    input miso

);

    assign user_io = 6'b000000;
    assign user_io = 4'b0000;
    assign ble_txo = 1;
    assign esp_txo = 1;
    wire clk;

    wire nope = 0;

    // Synchronize vusb_half to avoid metastability as it comes from outside and
    // is very likely to be noisy.
    reg [2:0] vusb_half_synch;
    always @ (posedge clk_slow)
        vusb_half_synch <= {vusb_half_synch[1:0], vusb_half};
    wire vusb_half_i = vusb_half_synch[2];

    // Except the clock because then it never works on battery. Sigh. Oh well.
    assign clk = vusb_half_i ? clk_fast : clk_slow;
    //assign clk = vusb_half_i ? clk_fast : nope;
    assign clk_fast_en = vusb_half ? 1'b1 : 1'b0;

    /*
    assign cs_n = 1;
    assign sclk = 0;
    assign mosi = 0;
    */
    // The incoming 14.7 MHz clock is too fast for changing animation patterns or generating a twinkle effect, so divide it into something slower.
    wire tick, tick_fast, tick_slow;
    //assign tick = clk;

    tickmaker #(.TICK_WIDTH(20)) ticker_fast_clk (clk, 1'b1, tick_fast);
    tickmaker #(.TICK_WIDTH(13)) ticker_slow_clk (clk, 1'b1, tick_slow);
    assign tick = vusb_half_i ? tick_fast : tick_slow;

    // LED intensity values to feed into per-LED PWM modules, coming from either animation player, random, or user-defined values.
    reg [24*3-1:0] intensities, intensities_i;
    wire [24*3-1:0] intensities_s, intensities_r;

    pwm p[23:0](
        .clk({24{clk}}),
        .value(intensities),
        .signal(leds)
    );

    // Signals to update the RAM in the animation player from user-defined LED intensitiy values.
    wire [7:0] pat_up_a;
    wire [71:0] pat_up_d;
    reg pat_up_we = 0;

    // The two main pattern generators.
    stored_pattern pat(clk, tick, pat_up_a, pat_up_d, pat_up_we, intensities_s);
    random rnd(clk, tick, intensities_r);

    // From the above tick signal, divide it down further for triggering a switch from pattern type to another.
`define SWITCH_LENGTH 8
    reg [`SWITCH_LENGTH:0] switcher = 0;
    always @ (posedge clk) if (tick) switcher <= switcher + 1;
    wire clk_switch = switcher[`SWITCH_LENGTH];

    //
    // Handle incoming commands from user over UART.
    // r - switch to random
    // s - switch to stored pattern
    // i - user defined intensities
    // anything else - normal switch back and forth
    // (which also gets triggered at startup; some garbage data is fed in ... fixme)
    //

    wire cmd_go;
    wire [7:0] cmd;
    reg [7:0] cmd_latch;

    always @ (*)
        if (cmd_latch == "r")
            intensities <= intensities_r;
        else if (cmd_latch == "s")
            intensities <= intensities_s;
        else if (cmd_latch == "i")
            intensities <= intensities_i;
        else
            case(vusb_half_i)
                0: intensities <= intensities_s;
                1: intensities <= intensities_r;
            endcase

    //
    // Handle setting specific LED intensities, and programming them into pattern RAM if desired.
    // p, byte - shift that byte into 72 bit LED intensity register
    // w, byte - write intensity register into address <byte>
    //

    assign pat_up_a = cmd;
    assign pat_up_d = intensities_i;

    reg [71:0] temp = 0;
    reg [1:0] s;
    always @ (posedge clk) begin
        if (cmd_go)
            case (s)
                0: begin
                    pat_up_we <= 0;
                    if (cmd == "p") begin
                        s <= 1;
                    end else if (cmd == "w") begin
                        s <= 2;
                    end else if (cmd == "u") begin
                        intensities_i <= temp;
                    end else begin
                        cmd_latch <= cmd;
                    end
                end
                1: begin
                    //intensities_i <= {intensities_i[63:0], cmd};
                    temp <= {temp[63:0], cmd};
                    //intensities_i <= {cmd, intensities_i[71:8]};
                    pat_up_we <= 0;
                    s <= 0;
                end
                2: begin
                    pat_up_we <= 1;
                    s <= 0;
                end
                // Should never reach this state so don't take up hardware resources with assigning specific outputs.
                default: begin
                    pat_up_we <= 1'bx;
                    intensities_i <= 72'hxxxxxxxxxxxxxxxxxx;
                    cmd_latch <= 8'hxx;
                    s <= 0;
                end
            endcase
    end

    // Serial receiver
    wire rx_done;
    wire [7:0] rx_byte;

    uart_rx rx_i(clk, rx, rx_done, rx_byte);

    // fixme: the receiver done signal stays high; perhaps making this a pulse is better handled in the module
    reg rx_done_l = 1;
    always @ (posedge clk) rx_done_l <= rx_done;
    assign cmd_go = rx_done & !rx_done_l;

    assign cmd = rx_byte;

    // Serial transmitter
    wire tx_busy;
    uart_tx tx_i(clk, cmd, cmd_go, tx_busy, tx);
    
endmodule

/*
    Generates a random pattern on the LEDs by taking the output of a LFSR.
*/
module random(
    input clk,  // Input clock signal from ossilator
    input tick, // Pulse which signals its time to move to the next step in the pattern

    // The values for the 24 LEDs (each led having a 3 bit value). These should
    // be updated every time tick pulses.
    output [24*3-1:0] intensities
    );

    // One bit for each LED. The decoder below takes whatever random number the
    // LFSR generates and uses it to pick a single LED to "twinkle"
    reg [23:0] twinkle = 0;

    // Random blinking is controlled by LFSR. If LFSR == LED #, initiate twinkle.
    // In order to avoid too many LEDs twinkling at once, use a few more bits so
    // not every number matches an LED number, making it so that most of the time
    // only one (or no) LED blinks this tick cycle.
    wire [15:0] rnd;
    LFSR16_1002D l(clk, tick, rnd);

    // fixme: presumably there's a more compact way to generate a one-hot decoder than this
    always @ (*) begin
        case (rnd[6:0])
            0: twinkle <= 24'b100000000000000000000000;
            1: twinkle <= 24'b010000000000000000000000;
            2: twinkle <= 24'b001000000000000000000000;
            3: twinkle <= 24'b000100000000000000000000;
            4: twinkle <= 24'b000010000000000000000000;
            5: twinkle <= 24'b000001000000000000000000;
            6: twinkle <= 24'b000000100000000000000000;
            7: twinkle <= 24'b000000010000000000000000;
            8: twinkle <= 24'b000000001000000000000000;
            9: twinkle <= 24'b000000000100000000000000;
            10: twinkle <= 24'b000000000010000000000000;
            11: twinkle <= 24'b000000000001000000000000;
            12: twinkle <= 24'b000000000000100000000000;
            13: twinkle <= 24'b000000000000010000000000;
            14: twinkle <= 24'b000000000000001000000000;
            15: twinkle <= 24'b000000000000000100000000;
            16: twinkle <= 24'b000000000000000010000000;
            17: twinkle <= 24'b000000000000000001000000;
            18: twinkle <= 24'b000000000000000000100000;
            19: twinkle <= 24'b000000000000000000010000;
            20: twinkle <= 24'b000000000000000000001000;
            21: twinkle <= 24'b000000000000000000000100;
            22: twinkle <= 24'b000000000000000000000010;
            23: twinkle <= 24'b000000000000000000000001;
            default: twinkle <= 0;
        endcase
    end
    
    // The twinkler module "twinkles" a single LED when told to do so by the go
    // signal pulsing. Below instantiates 24 twinklers, with each go signal
    // hooked up to one of the bits of the above twinkle signal.
    twinkler t[23:0](
        .clk({24{clk}}),
        .go(twinkle),
        .step({24{tick}}),
        .intensity(intensities)
    );

endmodule

module stored_pattern(
    input clk,
    input step,
    input [7:0] pat_up_a,
    input [71:0] pat_up_d,
    input pat_up_we,
    output [71:0] leds
    );
    
    wire [71:0] led_values;
    assign leds = led_values;
    reg [7:0] prog_addr = 0;

    ram pattern_ram (
      .clk(clk),
      .write_en(pat_up_we),
      .addr(pat_up_we ? pat_up_a : prog_addr),
      .din(pat_up_d),
      .dout(led_values)
    );
    
    always @ (posedge clk) if (step) prog_addr <= prog_addr + 1;

endmodule

module ram (din, addr, write_en, clk, dout);

    parameter addr_width = 8;
    parameter data_width = 72;

    input [addr_width-1:0] addr;
    input [data_width-1:0] din;
    input write_en, clk;
    output [data_width-1:0] dout;

    reg [data_width-1:0] dout;
    reg [data_width-1:0] mem [(1<<addr_width)-1:0];
    initial begin
        $readmemh("waterfall.txt", mem);
    end
    
    always @(posedge clk)  begin
        if (write_en)
            mem[(addr)] <= din;
        dout = mem[addr];
    end 
endmodule

module tickmaker(input clk, input tickin, output tickout);

    parameter TICK_WIDTH = 21;
    reg [TICK_WIDTH:0] c = 0;
    always @ (posedge clk) if (tickin) c <= c[TICK_WIDTH-1:0] + 1;
    wire tickout = c[TICK_WIDTH];

endmodule

/*
    "Twinkles" an LED by ramping its value from 0 to max brightness and back
    again. Do this when told to do so by the go signal.
*/
module twinkler(input clk, input go, input step, output reg [2:0] intensity = 0);

    reg direction = 0;
    reg state; initial state = 0;
    always @ (posedge clk) begin
        case (state)
            0: if (go) state <= 1;
            1: begin
                case (direction)
                    0: begin
                        if (step) begin
                            intensity <= intensity + 1;
                            if (intensity == 3'b100) direction <= 1;
                        end
                    end
                    1: begin
                        if (step) begin
                            intensity <= intensity - 1;
                            if (intensity == 3'b001) begin
                                direction <= 0;
                                state <= 0;
                            end
                        end
                    end
                endcase
            end
        endcase
    end
endmodule

module pwm(clk, value, signal);
    input clk;
    input [2:0] value;
    output signal;

    reg [3:0] PWM;
    always @(posedge clk) PWM <= PWM[2:0]+value;

    assign signal = PWM[3];
endmodule

module LFSR16_1002D(
  input clk,
  input tick,
  output reg [15:0] LFSR = 65535
);

    wire feedback = LFSR[15];

    always @(posedge clk)
    begin
      if (tick) begin
          LFSR[0] <= feedback;
          LFSR[1] <= LFSR[0];
          LFSR[2] <= LFSR[1] ^ feedback;
          LFSR[3] <= LFSR[2] ^ feedback;
          LFSR[4] <= LFSR[3];
          LFSR[5] <= LFSR[4] ^ feedback;
          LFSR[6] <= LFSR[5];
          LFSR[7] <= LFSR[6];
          LFSR[8] <= LFSR[7];
          LFSR[9] <= LFSR[8];
          LFSR[10] <= LFSR[9];
          LFSR[11] <= LFSR[10];
          LFSR[12] <= LFSR[11];
          LFSR[13] <= LFSR[12];
          LFSR[14] <= LFSR[13];
          LFSR[15] <= LFSR[14];
      end
    end
endmodule

