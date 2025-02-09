`timescale 1ns/1ps

module uart_rx_fsm_tb;
    // Testbench signals
    reg clk;
    reg reset;
    reg rx_bit_done;
    reg byte_done;
    reg parity_done;
    wire [2:0] out_state;

    // Instantiate the UART RX FSM module
    uart_rx_fsm uut (
        .clk(clk),
        .reset(reset),
        .rx_bit_done(rx_bit_done),
        .byte_done(byte_done),
        .parity_done(parity_done),
        .out_state(out_state)
    );

    // Clock generation (50MHz -> 20ns period)
    always #10 clk = ~clk;

    // Task to simulate 8-bit data reception
    task send_8_bits;
        integer i;
        begin
            for (i = 0; i < 8; i = i + 1) begin
                #20 rx_bit_done <= 1;  // Simulate bit received
                #20 rx_bit_done <= 0;
            end
        end
    endtask

    // Test sequence
    initial begin
        // Initialize signals
        clk = 0;
        reset = 0;
        rx_bit_done = 0;
        byte_done = 0;
        parity_done = 0;

        // Apply reset
        #20 reset <= 1;
        #20 reset <= 0;

        // Test IDLE -> START
        #20 rx_bit_done <= 1;
        #20 rx_bit_done <= 0;

        // Test START -> DATA
        #20 rx_bit_done <= 1;
        #20 rx_bit_done <= 0;

        // Simulate receiving 8 bits (DATA state)
        send_8_bits();

        // Signal that byte is complete
        #20 byte_done <= 1;
        #20 byte_done <= 0;

        // Test PARITY -> STOP
        #20 parity_done <= 1;
        #20 parity_done <= 0;

        // Test STOP -> IDLE
        #20 rx_bit_done <= 1;
        #20 rx_bit_done <= 0;

        // End simulation
        #50 $finish;
    end

    // Monitor FSM output
    initial begin
        $monitor("Time=%0t | reset=%b | rx_bit_done=%b | byte_done=%b | parity_done=%b | out_state=%b",
                 $time, reset, rx_bit_done, byte_done, parity_done, out_state);
    end
endmodule
