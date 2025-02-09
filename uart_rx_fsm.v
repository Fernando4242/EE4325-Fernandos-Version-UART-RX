module uart_rx_fsm(
    input clk,
    input reset,
    input rx_bit_done,
    input byte_done,
    input parity_done,
    output reg [2:0] out_state  // This now reflects the next state immediately
);

    // State Encoding using parameter (Verilog-compatible)
    parameter IDLE   = 3'b000;
    parameter START  = 3'b001;
    parameter DATA   = 3'b010;
    parameter PARITY = 3'b100;
    parameter STOP   = 3'b101;

    reg [2:0] current_state, next_state;

    // State register (current state only updates on clk)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= IDLE;  // Reset state to IDLE
        end else begin
            current_state <= next_state;  // Update current state to next state on clk
        end
    end

    // Next state logic based on current state and input signals
    always @(*) begin
        case (current_state)
            IDLE: begin
                if (rx_bit_done) next_state = START;  // Transition to START when rx_bit_done is high
                else next_state = IDLE;  // Stay in IDLE
            end

            START: begin
                if (rx_bit_done) next_state = DATA;  // Transition to DATA when rx_bit_done is high
                else next_state = START;  // Stay in START
            end

            DATA: begin
                if (byte_done) next_state = PARITY;  // Transition to PARITY when byte_done is high
                else next_state = DATA;  // Stay in DATA
            end

            PARITY: begin
                if (parity_done) next_state = STOP;  // Transition to STOP when parity_done is high
                else next_state = PARITY;  // Stay in PARITY
            end

            STOP: begin
                if (rx_bit_done) next_state = IDLE;  // Transition to IDLE when rx_bit_done is high
                else next_state = STOP;  // Stay in STOP
            end

            default: next_state = IDLE;  // Default state is IDLE
        endcase
    end

    // Output logic: directly reflect the next state
    always @(*) begin
        out_state = next_state;  // Immediate reflection of the next state
    end

endmodule
