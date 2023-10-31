module bridge(input clk, INF.bridge_inf inf);

//================================================================
// logic 
//================================================================

logic [63:0] data_reg;
logic [17:0] addr_reg;

//================================================================
// state 
//================================================================

enum logic [1:0] {
    IDLE,
    READ,
    WRITE,
    OUTPUT
} c_state, n_state;

//================================================================
//   FSM
//================================================================

// c_state
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) c_state <= IDLE;
    else c_state <= n_state;
end

// n_state
always_comb begin 
    case(c_state)
        IDLE: begin 
            if(inf.C_in_valid && inf.C_r_wb) n_state = READ;
            else if(inf.C_in_valid && !inf.C_r_wb) n_state = WRITE;
            else n_state = c_state;
        end
        READ: begin 
            if(inf.R_VALID) n_state = OUTPUT;
            else n_state = c_state;
        end
        WRITE: begin 
            if(inf.B_VALID) n_state = OUTPUT;
            else n_state = c_state;
        end
        OUTPUT: begin 
            n_state = IDLE;
        end
    endcase
end

//================================================================
//   design
//================================================================

assign inf.C_out_valid = (c_state == OUTPUT)? 1'b1 : 1'b0;

assign inf.W_DATA = (c_state == WRITE)? data_reg : 64'b0;
assign inf.C_data_r = (c_state == OUTPUT)? data_reg : 64'b0;

// data_reg
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) data_reg <= 64'b0;
    else if(n_state == WRITE) data_reg <= inf.C_data_w;
    else if(n_state == OUTPUT && c_state == READ) data_reg <= inf.R_DATA;
end

// addr_reg
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) addr_reg <= 17'b0;
    else if(n_state == READ || n_state == WRITE) addr_reg <= {1'b1, 5'b0, inf.C_addr, 3'b0};
end

assign inf.AR_ADDR = (c_state == READ)? addr_reg : 17'b0;
assign inf.AW_ADDR = (c_state == WRITE)? addr_reg : 17'b0;

assign inf.R_READY = (c_state == READ)? 1'b1 : 1'b0;
assign inf.B_READY = (c_state == WRITE)? 1'b1 : 1'b0;

// inf.AR_VALID
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.AR_VALID <= 1'b0;
    else if(n_state == READ && c_state != READ) inf.AR_VALID <= 1'b1;
    else if(n_state == READ && inf.AR_READY) inf.AR_VALID <= 1'b0;
end

// inf.AW_VALID
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.AW_VALID <= 1'b0;
    else if(n_state == WRITE && c_state != WRITE) inf.AW_VALID <= 1'b1;
    else if(n_state == WRITE && inf.AW_READY) inf.AW_VALID <= 1'b0;
end

// inf.W_VALID
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.W_VALID <= 1'b0;
    else if(n_state == WRITE && inf.AW_READY) inf.W_VALID <= 1'b1;
    else if(n_state == WRITE && inf.W_READY) inf.W_VALID <= 1'b0;
end

endmodule
