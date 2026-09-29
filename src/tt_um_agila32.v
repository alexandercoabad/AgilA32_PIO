// tt_um_agila32.v -- Tiny Tapeout top level
//
// Pin mapping (v3, CS2/RAM B added):
//   ui_in[7:0]  -> memory-mapped input register at address 0xF4 (switches)
//   uo_out[7:0] -> memory-mapped output register at address 0xF0 (LEDs)
//   uio[0] -> QSPI Pmod CS0  (flash -- backs the 0xB4-0xDF program
//              window when a bootloaded program sets FLASH_MODE; see
//              mem.v's header. Never asserted by the boot ROM itself.)
//   uio[1] -> QSPI Pmod SD0/MOSI
//   uio[2] -> QSPI Pmod SD1/MISO (input)
//   uio[3] -> QSPI Pmod SCK
//   uio[4] -> held high (SD2, unused in single-line mode)
//   uio[5] -> held high (SD3, unused in single-line mode)
//   uio[6] -> QSPI Pmod CS1  (PSRAM "RAM A", backs mem.v's 0xE0-0xEF window
//              by default)
//   uio[7] -> QSPI Pmod CS2  (PSRAM "RAM B", backs that SAME 0xE0-0xEF
//              window instead once a program sets PSRAM_BANK (0xF1) --
//              see mem.v's header. Reachable with no board modification:
//              on the stock Pmod, CS2 is already wired to this second
//              PSRAM chip.)
//
// uo_out[7] is muxed three ways by PIN_MUX (0xFA, bits[1:0]) -- see
// mem.v's header: 00 = LED_OUT[7] (default), 01 = the Timer/PWM
// peripheral's PWM waveform, 10 = the core's `halted` status (ported
// from AgilA8's GPIO_DIR[7] halted-status mux; goes high once EBREAK
// parks the core in ST_HALTED -- see rv32i_core.v). uo_out[6:0] always
// shows LED_OUT[6:0].
//
// PIO (v4): two RP2040-compatible PIO state machines sit on the CPU bus at
// 0xFF (PIO_IDX) / 0xFE (PIO_DATA). PIO pins 0-7 = uo_out[0..7] (input side:
// ui_in[0..7]); PIO pins 8-9 = uio[4], uio[5]. A pin is only taken from the
// CPU/LED/QSPI logic once its PIN_OWN bit is set, so reset behaviour and
// every earlier feature are unchanged. See docs/info.md, "PIO".
//
// `ena` is ignored (always active) per TT convention for simple designs.

`default_nettype none

module tt_um_agila32 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    wire [7:0]  mem_addr;
    wire [31:0] mem_wdata;
    wire [1:0]  mem_size;
    wire        mem_we;
    wire        mem_valid;
    wire        mem_ready;
    wire [31:0] mem_rdata;
    wire [7:0]  led_out;

    wire        qspi_cs0, qspi_cs1, qspi_cs2, qspi_sck, qspi_mosi, qspi_miso;
    wire        pwm_out, halted;
    wire [1:0]  pin_mux;

    // PIO block (tap on the CPU bus at 0xFE/0xFF -- see pio.v)
    wire [31:0] mem_rdata_mem;
    wire [31:0] pio_rdata;
    wire        pio_sel;
    wire [9:0]  pio_pin_out, pio_pin_dir, pio_pin_own;

    rv32i_core u_core (
        .clk       (clk),
        .rst_n     (rst_n),
        .mem_addr  (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_size  (mem_size),
        .mem_we    (mem_we),
        .mem_valid (mem_valid),
        .mem_ready (mem_ready),
        .mem_rdata (mem_rdata),
        .halted    (halted)
    );

    mem u_mem (
        .clk      (clk),
        .rst_n    (rst_n),
        .addr     (mem_addr),
        .wdata    (mem_wdata),
        .size     (mem_size),
        .we       (mem_we),
        .valid    (mem_valid),
        .ready    (mem_ready),
        .rdata    (mem_rdata_mem),
        .gpio_in  (ui_in),
        .gpio_out (led_out),
        .qspi_cs0 (qspi_cs0),
        .qspi_cs1 (qspi_cs1),
        .qspi_cs2 (qspi_cs2),
        .qspi_sck (qspi_sck),
        .qspi_mosi(qspi_mosi),
        .qspi_miso(qspi_miso),
        .pwm_out    (pwm_out),
        .pin_mux_out(pin_mux)
    );

    pio #(.N_SM(2), .FIFO_LOG2(2)) u_pio (
        .clk      (clk),
        .rst_n    (rst_n),
        .valid    (mem_valid),
        .we       (mem_we),
        .addr     (mem_addr),
        .wdata    (mem_wdata),
        .rdata    (pio_rdata),
        .sel      (pio_sel),
        .pins_raw ({uio_in[5:4], ui_in}),
        .pin_out  (pio_pin_out),
        .pin_dir  (pio_pin_dir),
        .pin_own  (pio_pin_own)
    );

    // 0xFE / 0xFF belong to PIO; everything else is answered by mem.v.
    assign mem_rdata = pio_sel ? pio_rdata : mem_rdata_mem;

    // PIN_MUX select for uo_out[7]: 2'b01 = PWM, 2'b10 = halted status,
    // anything else (2'b00 default, or reserved 2'b11) falls back to
    // LED_OUT[7].
    wire uo7_mux = (pin_mux == 2'b01) ? pwm_out :
                   (pin_mux == 2'b10) ? halted  :
                                        led_out[7];

    wire [7:0] uo_cpu = {uo7_mux, led_out[6:0]};

    // A pin whose PIN_OWN bit is set is driven by PIO instead of the
    // CPU/LED logic (PIO pins 0-7 = uo_out[0..7]).
    assign uo_out = (uo_cpu & ~pio_pin_own[7:0]) | (pio_pin_out[7:0] & pio_pin_own[7:0]);

    // uio[2] (MISO) is the only bidirectional pin actually used as an
    // input; everything else this project drives is an output.
    assign qspi_miso = uio_in[2];

    // uio[4]/uio[5] (QSPI SD2/SD3, otherwise held high) become PIO pins
    // 8 and 9 once owned: true bidirectional / open-drain capable, with
    // the PIO pindir bit as the real output enable.
    wire uio4_out = pio_pin_own[8] ? pio_pin_out[8] : 1'b1;
    wire uio5_out = pio_pin_own[9] ? pio_pin_out[9] : 1'b1;
    wire uio4_oe  = pio_pin_own[8] ? pio_pin_dir[8] : 1'b1;
    wire uio5_oe  = pio_pin_own[9] ? pio_pin_dir[9] : 1'b1;

    assign uio_out = {qspi_cs2,   // uio[7]
                       qspi_cs1,  // uio[6]
                       uio5_out,  // uio[5] SD3 / PIO pin 9
                       uio4_out,  // uio[4] SD2 / PIO pin 8
                       qspi_sck,  // uio[3]
                       1'b0,      // uio[2] MISO -- input, value here is don't-care (oe=0 below)
                       qspi_mosi, // uio[1]
                       qspi_cs0}; // uio[0]

    assign uio_oe  = {2'b11, uio5_oe, uio4_oe, 1'b1, 1'b0, 2'b11}; // all outputs except uio[2] (MISO, input); uio[4:5] follow PIO when owned

    // Silence unused-signal lint warnings without affecting synthesis
    wire _unused = &{ena, uio_in[7:6], uio_in[3], uio_in[1:0], 1'b0};

endmodule
