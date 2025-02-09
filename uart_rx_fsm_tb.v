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
    always #10 clk = ~clk;  // Clock toggles every 10ns (20ns period)

    // Task to simulate 8-bit data reception
    task send_8_bits;
        integer i;
        begin
            for (i = 0; i < 8; i = i + 1) begin
                @(posedge clk);  // Wait for clock edge
                rx_bit_done <= 1;  
                @(posedge clk);
                rx_bit_done <= 0;
            end
        end
    endtask

    // Test sequence
    initial begin
        // Initialize signals
        clk = 0;
        reset = 1; // Start with reset active
        rx_bit_done = 0;
        byte_done = 0;
        parity_done = 0;

        // Apply reset
        @(posedge clk);
        reset <= 0;

        // Test IDLE -> START
        @(posedge clk);
        rx_bit_done <= 1;
        @(posedge clk);
        rx_bit_done <= 0;

        // Test START -> DATA
        @(posedge clk);
        rx_bit_done <= 1;
        @(posedge clk);
        rx_bit_done <= 0;

        // Simulate receiving 8 bits (DATA state)
        send_8_bits();

        // Signal that byte is complete
        @(posedge clk);
        byte_done <= 1;
        @(posedge clk);
        byte_done <= 0;

        // Test PARITY -> STOP
        @(posedge clk);
        parity_done <= 1;
        @(posedge clk);
        parity_done <= 0;

        // Test STOP -> IDLE
        @(posedge clk);
        rx_bit_done <= 1;
        @(posedge clk);
        rx_bit_done <= 0;

        // End simulation
        #50 $finish;
    end

    // Monitor FSM output
    initial begin
        $monitor("Time=%0t | reset=%b | rx_bit_done=%b | byte_done=%b | parity_done=%b | out_state=%b",
                 $time, reset, rx_bit_done, byte_done, parity_done, out_state);
    end
endmodule
