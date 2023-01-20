module EDH #(parameter ID_WIDTH=4, DATA_WIDTH=128, ADDR_WIDTH=32)(
    clk          ,
    rst_n        ,
    in_valid     ,
    op           ,
    pic_no       ,
    se_no        ,
    busy         ,

    awid_m_inf   ,
    awaddr_m_inf ,
    awlen_m_inf  ,
    awsize_m_inf ,
    awburst_m_inf,
    awvalid_m_inf,
    awready_m_inf,

    wdata_m_inf  ,
    wlast_m_inf  ,
    wvalid_m_inf ,
    wready_m_inf ,

    bid_m_inf    ,
    bresp_m_inf  ,
    bvalid_m_inf ,
    bready_m_inf ,

    arid_m_inf   ,
    araddr_m_inf ,
    arlen_m_inf  ,
    arsize_m_inf ,
    arburst_m_inf,
    arvalid_m_inf,
    arready_m_inf, 

    rid_m_inf    ,
    rdata_m_inf  ,
    rresp_m_inf  ,
    rlast_m_inf  ,
    rvalid_m_inf ,
    rready_m_inf 
);

//======================================
//          I/O PORTS
//======================================
input           clk;
input         rst_n;
input      in_valid;
input[3:0]   pic_no;
input[5:0]    se_no;
input[1:0]       op;
output reg     busy;

// axi read addr channel 
// src master
output [ID_WIDTH-1:0]     arid_m_inf;
output [ADDR_WIDTH-1:0] araddr_m_inf;
output [7:0]             arlen_m_inf;
output [2:0]            arsize_m_inf;
output [1:0]           arburst_m_inf;
output reg             arvalid_m_inf;
// src slave
input                  arready_m_inf;
// ------------------------

// axi read data channel 
// slave
input [ID_WIDTH-1:0]       rid_m_inf;
input [DATA_WIDTH-1:0]   rdata_m_inf;
input [1:0]              rresp_m_inf;
input                    rlast_m_inf;
input                   rvalid_m_inf;
// master
output                  rready_m_inf;
// -----------------------------

// axi write addr channel 
// src master
output [ID_WIDTH-1:0]     awid_m_inf;
output [ADDR_WIDTH-1:0] awaddr_m_inf;
output [7:0]             awlen_m_inf;
output [2:0]            awsize_m_inf;
output [1:0]           awburst_m_inf;
output reg             awvalid_m_inf;
// src slave
input                  awready_m_inf;
// -------------------------

// axi write data channel 
// src master
output reg [DATA_WIDTH-1:0]  wdata_m_inf;
output                   wlast_m_inf;
output reg              wvalid_m_inf;
// src slave
input                   wready_m_inf;

// axi write resp channel 
// src slave
input  [ID_WIDTH-1:0]      bid_m_inf;
input  [1:0]             bresp_m_inf;
input                   bvalid_m_inf;
// src master 
output                  bready_m_inf;
// ------------------------

//======================================
//          Declaration
//======================================

////////////// parameter //////////////

parameter IDLE = 3'd0;
parameter INPUT = 3'd1;
parameter READ_SE = 3'd3;
parameter BUBBLE = 3'd2;
parameter EROSION = 3'd6;
parameter DILATION = 3'd7;
parameter HISTOGRAM = 3'd5;
parameter WRITE_BACK = 3'd4;

////////////// regs //////////////

reg[2:0] c_state, n_state;
reg[1:0] which_op;
reg[3:0] which_pic;
reg[5:0] which_se;
reg PIC_loaded;
reg[7:0] handshake_counter, sram_counter;
reg[7:0] sram_addr;
reg sram_read;
reg[DATA_WIDTH-1:0] sram_in;
wire[DATA_WIDTH-1:0] sram_out, erosion_out, dilation_out, histogram_out;
reg[DATA_WIDTH-1:0] wdata_pipe;
reg[63:0] no_write_count;
wire[7:0] pixel_value[15:0], sram_value[15:0];

// histogram
reg count_table_00[255:0];
reg count_table_01[255:0];
reg count_table_02[255:0];
reg count_table_03[255:0];
reg count_table_04[255:0];
reg count_table_05[255:0];
reg count_table_06[255:0];
reg count_table_07[255:0];
reg count_table_08[255:0];
reg count_table_09[255:0];
reg count_table_10[255:0];
reg count_table_11[255:0];
reg count_table_12[255:0];
reg count_table_13[255:0];
reg count_table_14[255:0];
reg count_table_15[255:0];
reg[4:0] count_table_all[255:0];
reg[12:0] cdf_table[255:0];
reg[7:0] cdf_min_index;
wire[7:0] cmp[14:0];
wire[12:0] cdf_min;
wire[20:0] mult[15:0];
wire[12:0] divisor;
reg[DATA_WIDTH-1:0] histogram_pipe;

// erosion & dilation
reg[1791:0] line_buffer;
reg[7:0] SE_table[15:0];
wire[7:0] sub0[15:0];
wire[7:0] sub1[15:0];
wire[7:0] sub2[15:0];
wire[7:0] sub3[15:0];
wire[7:0] sub4[15:0];
wire[7:0] sub5[15:0];
wire[7:0] sub6[15:0];
wire[7:0] sub7[15:0];
wire[7:0] sub8[15:0];
wire[7:0] sub9[15:0];
wire[7:0] sub10[15:0];
wire[7:0] sub11[15:0];
wire[7:0] sub12[15:0];
wire[7:0] sub13[15:0];
wire[7:0] sub14[15:0];
wire[7:0] sub15[15:0];
wire[8:0] add0[15:0];
wire[8:0] add1[15:0];
wire[8:0] add2[15:0];
wire[8:0] add3[15:0];
wire[8:0] add4[15:0];
wire[8:0] add5[15:0];
wire[8:0] add6[15:0];
wire[8:0] add7[15:0];
wire[8:0] add8[15:0];
wire[8:0] add9[15:0];
wire[8:0] add10[15:0];
wire[8:0] add11[15:0];
wire[8:0] add12[15:0];
wire[8:0] add13[15:0];
wire[8:0] add14[15:0];
wire[8:0] add15[15:0];
wire[7:0] fadd0[15:0];
wire[7:0] fadd1[15:0];
wire[7:0] fadd2[15:0];
wire[7:0] fadd3[15:0];
wire[7:0] fadd4[15:0];
wire[7:0] fadd5[15:0];
wire[7:0] fadd6[15:0];
wire[7:0] fadd7[15:0];
wire[7:0] fadd8[15:0];
wire[7:0] fadd9[15:0];
wire[7:0] fadd10[15:0];
wire[7:0] fadd11[15:0];
wire[7:0] fadd12[15:0];
wire[7:0] fadd13[15:0];
wire[7:0] fadd14[15:0];
wire[7:0] fadd15[15:0];
wire[7:0] min0[15:0];
wire[7:0] min1[15:0];
wire[7:0] min2[15:0];
wire[7:0] min3[15:0];
wire[7:0] min4[15:0];
wire[7:0] min5[15:0];
wire[7:0] min6[15:0];
wire[7:0] min7[15:0];
wire[7:0] min8[15:0];
wire[7:0] min9[15:0];
wire[7:0] min10[15:0];
wire[7:0] min11[15:0];
wire[7:0] min12[15:0];
wire[7:0] min13[15:0];
wire[7:0] min14[15:0];
wire[7:0] max0[15:0];
wire[7:0] max1[15:0];
wire[7:0] max2[15:0];
wire[7:0] max3[15:0];
wire[7:0] max4[15:0];
wire[7:0] max5[15:0];
wire[7:0] max6[15:0];
wire[7:0] max7[15:0];
wire[7:0] max8[15:0];
wire[7:0] max9[15:0];
wire[7:0] max10[15:0];
wire[7:0] max11[15:0];
wire[7:0] max12[15:0];
wire[7:0] max13[15:0];
wire[7:0] max14[15:0];

//======================================
//          Design
//======================================

///////////////// FSM //////////////////

// c_state
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) c_state <= 3'd0;
    else c_state <= n_state;
end

// n_state
always@(*) begin 
    case(c_state)
        IDLE: begin
            if(in_valid == 1'b1) n_state = INPUT;
            else n_state = c_state;
        end
        INPUT: begin
            if(!which_op[1]) n_state = READ_SE;
            else n_state = HISTOGRAM;
        end 
        READ_SE: begin 
            if(!rlast_m_inf) n_state = c_state;
            else n_state = BUBBLE;
        end
        BUBBLE: begin 
            if(which_op == 2'd0) n_state = EROSION;
            else if(which_op == 2'd1) n_state = DILATION;
            else n_state = HISTOGRAM;
        end
        EROSION: begin 
            if(PIC_loaded && sram_counter == 8'd0) n_state = WRITE_BACK;
            else n_state = c_state;
        end
        DILATION: begin 
            if(PIC_loaded && sram_counter == 8'd0) n_state = WRITE_BACK;
            else n_state = c_state;
        end
        HISTOGRAM: begin 
            if(PIC_loaded && wlast_m_inf && wready_m_inf && wvalid_m_inf) n_state = IDLE;
            else n_state = c_state;
        end
        WRITE_BACK: begin 
            if(wlast_m_inf && wready_m_inf && wvalid_m_inf) n_state = IDLE;
            else n_state = c_state;
        end
    endcase
end

//////////// INPUT ////////////

// which_op
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) which_op <= 2'd0;
    else if(n_state == INPUT) which_op <= op;
end

// which_pic
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) which_pic <= 4'd0;
    else if(n_state == INPUT) which_pic <= pic_no;
end

// which_se
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) which_se <= 6'd0;
    else if(n_state == INPUT) which_se <= se_no;
end

// busy
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) busy <= 1'b0;
    else begin 
        case(n_state)
            IDLE: busy <= 1'b0;
            INPUT: busy <= 1'b0;
            default: busy <= 1'b1;
        endcase
    end
end

/////////////// FLAG ///////////////

// PIC_loaded
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) PIC_loaded <= 1'b0;
    else begin
        case(n_state)
            EROSION: if(handshake_counter == 8'd255 && rvalid_m_inf) PIC_loaded <= 1'b1;
            DILATION: if(handshake_counter == 8'd255 && rvalid_m_inf) PIC_loaded <= 1'b1;
            HISTOGRAM: if(handshake_counter == 8'd255 && rvalid_m_inf) PIC_loaded <= 1'b1;
            default: PIC_loaded <= 1'b0;
        endcase
    end
end

////////////// COUNTER ///////////////

// handshake_counter
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) handshake_counter <= 8'd0;
    else begin 
        case(n_state)
            EROSION: if(rvalid_m_inf) handshake_counter <= handshake_counter + 8'd1;
            DILATION: if(rvalid_m_inf) handshake_counter <= handshake_counter + 8'd1;
            HISTOGRAM: if(rvalid_m_inf || (wready_m_inf && wvalid_m_inf) || (awready_m_inf && awvalid_m_inf)) handshake_counter <= handshake_counter + 8'd1;
            WRITE_BACK: if((wready_m_inf && wvalid_m_inf) || (awready_m_inf && awvalid_m_inf)) handshake_counter <= handshake_counter + 8'd1;
            default: handshake_counter <= 8'b0;
        endcase
    end
end

// sram_counter
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) sram_counter <= 8'b0;
    else begin 
        case(n_state)
            EROSION: if(PIC_loaded || (handshake_counter > 8'd13 && rvalid_m_inf)) sram_counter <= sram_counter + 8'b1;
            DILATION: if(PIC_loaded || (handshake_counter > 8'd13 && rvalid_m_inf)) sram_counter <= sram_counter + 8'b1;
            default: sram_counter <= 8'b0;
        endcase
    end
end

//////////// SRAM ////////////

RA1SH_OUT SRAM_OUT(.A(sram_addr), .D(sram_in), .CLK(clk), .OEN(1'b0), .CEN(1'b0), .WEN(sram_read), .Q(sram_out));

// sram_addr
always@(*) begin 
    case(n_state)
        EROSION: sram_addr = sram_counter;
        DILATION: sram_addr = sram_counter;
        HISTOGRAM: sram_addr = handshake_counter;
        WRITE_BACK: sram_addr = handshake_counter;
        default: sram_addr = 8'b0;
    endcase
end

// sram_in
always@(*) begin 
    case(n_state)
        EROSION: sram_in = erosion_out;
        DILATION: sram_in = dilation_out;
        HISTOGRAM: sram_in = rdata_m_inf;
        default: sram_in = 128'b0;
    endcase
end

// sram_read
always@(*) begin 
    case(n_state)
        EROSION: sram_read = !((PIC_loaded && sram_counter != 8'd0) || (handshake_counter > 8'd13 && rvalid_m_inf));
        DILATION: sram_read = !((PIC_loaded && sram_counter != 8'd0) || (handshake_counter > 8'd13 && rvalid_m_inf)); 
        HISTOGRAM: sram_read = !rvalid_m_inf;
        default: sram_read = 1'b1;
    endcase
end

assign sram_value[0] = sram_out[7:0];
assign sram_value[1] = sram_out[15:8];
assign sram_value[2] = sram_out[23:16];
assign sram_value[3] = sram_out[31:24];
assign sram_value[4] = sram_out[39:32];
assign sram_value[5] = sram_out[47:40];
assign sram_value[6] = sram_out[55:48];
assign sram_value[7] = sram_out[63:56];
assign sram_value[8] = sram_out[71:64];
assign sram_value[9] = sram_out[79:72];
assign sram_value[10] = sram_out[87:80];
assign sram_value[11] = sram_out[95:88];
assign sram_value[12] = sram_out[103:96];
assign sram_value[13] = sram_out[111:104];
assign sram_value[14] = sram_out[119:112];
assign sram_value[15] = sram_out[127:120];

//////////// AXI /////////////

// arvalid_m_inf
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) arvalid_m_inf <= 1'b0;
    else begin 
        case(n_state)
            READ_SE: begin 
                if(c_state != READ_SE) arvalid_m_inf <= 1'b1;
                else if(arready_m_inf) arvalid_m_inf <= 1'b0;
            end
            EROSION: begin 
                if(c_state != EROSION) arvalid_m_inf <= 1'b1;
                else if(arready_m_inf) arvalid_m_inf <= 1'b0;
            end
            DILATION: begin 
                if(c_state != DILATION) arvalid_m_inf <= 1'b1;
                else if(arready_m_inf) arvalid_m_inf <= 1'b0;
            end
            HISTOGRAM: begin 
                if(c_state != HISTOGRAM) arvalid_m_inf <= 1'b1;
                else if(arready_m_inf) arvalid_m_inf <= 1'b0;
            end
            default: arvalid_m_inf <= 1'b0;
        endcase
    end
end

// awvalid_m_inf
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) awvalid_m_inf <= 1'b0;
    else begin 
        case(n_state)
            HISTOGRAM: begin 
                if(!PIC_loaded && handshake_counter == 8'd255 && rvalid_m_inf) awvalid_m_inf <= 1'b1;
                else if (awready_m_inf) awvalid_m_inf <= 1'b0;
            end
            WRITE_BACK: begin 
                if(c_state != WRITE_BACK) awvalid_m_inf <= 1'b1;
                else if(awready_m_inf) awvalid_m_inf <= 1'b0;
            end
        endcase
    end
end

// wvalid_m_inf
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) wvalid_m_inf <= 1'b1;
    else if(n_state == HISTOGRAM && wvalid_m_inf && wready_m_inf) wvalid_m_inf <= 1'b0;
    else wvalid_m_inf <= 1'b1;
end

// no_write_count
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) no_write_count <= 64'b0;
    else if(!wready_m_inf) no_write_count <= no_write_count + 1;
    else no_write_count <= 64'b0;
end

// wdata_pipe
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) wdata_pipe <= 128'b0;
    else begin 
        case(n_state)
            HISTOGRAM: if((no_write_count == 1 && !wready_m_inf) || (awready_m_inf && !awvalid_m_inf)) wdata_pipe <= histogram_pipe;
            WRITE_BACK: if((no_write_count == 0 && !wready_m_inf) || (awready_m_inf && !awvalid_m_inf)) wdata_pipe <= sram_out;
        endcase
    end
end

// wdata_m_inf
always@(*) begin 
    case(c_state)
        HISTOGRAM: begin 
            if(no_write_count > 1) wdata_m_inf = wdata_pipe;
            else wdata_m_inf = histogram_pipe;
        end
        WRITE_BACK: begin 
            if(no_write_count > 0) wdata_m_inf = wdata_pipe;
            else wdata_m_inf = sram_out;
        end
        default: wdata_m_inf = 128'b0;
    endcase
end

assign arid_m_inf = 4'd0;
assign araddr_m_inf = (c_state == READ_SE)? {16'h0003, 6'b00, which_se, 4'h0} : {16'h0004, which_pic, 12'h0};
assign arlen_m_inf = (c_state == READ_SE)? 8'd0 : 8'd255;
assign arsize_m_inf = 3'b100;
assign arburst_m_inf = 2'b01;

assign rready_m_inf = 1'b1;

assign awid_m_inf = 4'd0;
assign awaddr_m_inf = {16'h0004, which_pic, 12'h0};
assign awlen_m_inf = 8'd255;
assign awsize_m_inf = 3'b100;
assign awburst_m_inf = 2'b01;

assign wlast_m_inf = (handshake_counter == 8'd0 && !awvalid_m_inf)? 1'b1 : 1'b0;

assign bready_m_inf = 1'b1;

assign pixel_value[0] = rdata_m_inf[7:0];
assign pixel_value[1] = rdata_m_inf[15:8];
assign pixel_value[2] = rdata_m_inf[23:16];
assign pixel_value[3] = rdata_m_inf[31:24];
assign pixel_value[4] = rdata_m_inf[39:32];
assign pixel_value[5] = rdata_m_inf[47:40];
assign pixel_value[6] = rdata_m_inf[55:48];
assign pixel_value[7] = rdata_m_inf[63:56];
assign pixel_value[8] = rdata_m_inf[71:64];
assign pixel_value[9] = rdata_m_inf[79:72];
assign pixel_value[10] = rdata_m_inf[87:80];
assign pixel_value[11] = rdata_m_inf[95:88];
assign pixel_value[12] = rdata_m_inf[103:96];
assign pixel_value[13] = rdata_m_inf[111:104];
assign pixel_value[14] = rdata_m_inf[119:112];
assign pixel_value[15] = rdata_m_inf[127:120];

//////////////////////////////// HISTOGRAM ////////////////////////////////////

genvar i;
generate
    for(i = 0; i < 256; i = i + 1) begin: count_loop
        // count_table_00
        always@(*) begin 
            if(pixel_value[0] == i || pixel_value[0] < i) count_table_00[i] = 1'b1;
            else count_table_00[i] = 1'b0;
        end
        // count_table_01
        always@(*) begin 
            if(pixel_value[1] == i || pixel_value[1] < i) count_table_01[i] = 1'b1;
            else count_table_01[i] = 1'b0;
        end
        // count_table_02
        always@(*) begin 
            if(pixel_value[2] == i || pixel_value[2] < i) count_table_02[i] = 1'b1;
            else count_table_02[i] = 1'b0;
        end
        // count_table_03
        always@(*) begin 
            if(pixel_value[3] == i || pixel_value[3] < i) count_table_03[i] = 1'b1;
            else count_table_03[i] = 1'b0;
        end
        // count_table_04
        always@(*) begin 
            if(pixel_value[4] == i || pixel_value[4] < i) count_table_04[i] = 1'b1;
            else count_table_04[i] = 1'b0;
        end
        // count_table_05
        always@(*) begin 
            if(pixel_value[5] == i || pixel_value[5] < i) count_table_05[i] = 1'b1;
            else count_table_05[i] = 1'b0;
        end
        // count_table_06
        always@(*) begin 
            if(pixel_value[6] == i || pixel_value[6] < i) count_table_06[i] = 1'b1;
            else count_table_06[i] = 1'b0;
        end
        // count_table_07
        always@(*) begin 
            if(pixel_value[7] == i || pixel_value[7] < i) count_table_07[i] = 1'b1;
            else count_table_07[i] = 1'b0;
        end
        // count_table_08
        always@(*) begin 
            if(pixel_value[8] == i || pixel_value[8] < i) count_table_08[i] = 1'b1;
            else count_table_08[i] = 1'b0;
        end
        // count_table_09
        always@(*) begin 
            if(pixel_value[9] == i || pixel_value[9] < i) count_table_09[i] = 1'b1;
            else count_table_09[i] = 1'b0;
        end
        // count_table_10
        always@(*) begin
            if(pixel_value[10] == i || pixel_value[10] < i) count_table_10[i] = 1'b1;
            else count_table_10[i] = 1'b0;
        end
        // count_table_11
        always@(*) begin 
            if(pixel_value[11] == i || pixel_value[11] < i) count_table_11[i] = 1'b1;
            else count_table_11[i] = 1'b0;
        end
        // count_table_12
        always@(*) begin 
            if(pixel_value[12] == i || pixel_value[12] < i) count_table_12[i] = 1'b1;
            else count_table_12[i] = 1'b0;
        end
        // count_table_13
        always@(*) begin 
            if(pixel_value[13] == i || pixel_value[13] < i) count_table_13[i] = 1'b1;
            else count_table_13[i] = 1'b0;
        end
        // count_table_14
        always@(*) begin 
            if(pixel_value[14] == i || pixel_value[14] < i) count_table_14[i] = 1'b1;
            else count_table_14[i] = 1'b0;
        end
        // count_table_15
        always@(*) begin 
            if(pixel_value[15] == i || pixel_value[15] < i) count_table_15[i] = 1'b1;
            else count_table_15[i] = 1'b0;
        end
        // count_table_all
        always@(*) begin 
            count_table_all[i] = count_table_00[i] + count_table_01[i] + count_table_02[i] + count_table_03[i]
                                + count_table_04[i] + count_table_05[i] + count_table_06[i] + count_table_07[i]
                                + count_table_08[i] + count_table_09[i] + count_table_10[i] + count_table_11[i]
                                + count_table_12[i] + count_table_13[i] + count_table_14[i] + count_table_15[i];
        end
        // cdf_table
        always@(posedge clk or negedge rst_n) begin 
            if(!rst_n) cdf_table[i] <= 13'b0;
            else if(n_state == HISTOGRAM && !PIC_loaded && rvalid_m_inf) cdf_table[i] <= cdf_table[i] + count_table_all[i];
            else if(n_state == INPUT) cdf_table[i] <= 13'b0;
        end
    end
endgenerate

assign cmp[0]  = (pixel_value[0]  < pixel_value[1] )? pixel_value[0]  : pixel_value[1] ;
assign cmp[1]  = (pixel_value[2]  < pixel_value[3] )? pixel_value[2]  : pixel_value[3] ;
assign cmp[2]  = (pixel_value[4]  < pixel_value[5] )? pixel_value[4]  : pixel_value[5] ;
assign cmp[3]  = (pixel_value[6]  < pixel_value[7] )? pixel_value[6]  : pixel_value[7] ;
assign cmp[4]  = (pixel_value[8]  < pixel_value[9] )? pixel_value[8]  : pixel_value[9] ;
assign cmp[5]  = (pixel_value[10] < pixel_value[11])? pixel_value[10] : pixel_value[11];
assign cmp[6]  = (pixel_value[12] < pixel_value[13])? pixel_value[12] : pixel_value[13];
assign cmp[7]  = (pixel_value[14] < pixel_value[15])? pixel_value[14] : pixel_value[15];
assign cmp[8]  = (cmp[0] < cmp[1])? cmp[0] : cmp[1];
assign cmp[9]  = (cmp[2] < cmp[3])? cmp[2] : cmp[3];
assign cmp[10] = (cmp[4] < cmp[5])? cmp[4] : cmp[5];
assign cmp[11] = (cmp[6] < cmp[7])? cmp[6] : cmp[7];
assign cmp[12] = (cmp[8]  < cmp[9] )? cmp[8]  : cmp[9] ;
assign cmp[13] = (cmp[10] < cmp[11])? cmp[10] : cmp[11];
assign cmp[14] = (cmp[12] < cmp[13])? cmp[12] : cmp[13];

// cdf_min_index
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) cdf_min_index <= 8'd255;
    else if(n_state == HISTOGRAM && !PIC_loaded && cmp[14] < cdf_min_index && rvalid_m_inf) cdf_min_index <= cmp[14];
    else if(n_state == INPUT) cdf_min_index <= 8'd255; 
end

assign cdf_min = cdf_table[cdf_min_index];

assign divisor = 13'd4096 - cdf_min;

assign mult[0] = ((cdf_table[sram_value[0]] - cdf_min) << 8) - (cdf_table[sram_value[0]] - cdf_min);
assign mult[1] = ((cdf_table[sram_value[1]] - cdf_min) << 8) - (cdf_table[sram_value[1]] - cdf_min);
assign mult[2] = ((cdf_table[sram_value[2]] - cdf_min) << 8) - (cdf_table[sram_value[2]] - cdf_min);
assign mult[3] = ((cdf_table[sram_value[3]] - cdf_min) << 8) - (cdf_table[sram_value[3]] - cdf_min);
assign mult[4] = ((cdf_table[sram_value[4]] - cdf_min) << 8) - (cdf_table[sram_value[4]] - cdf_min);
assign mult[5] = ((cdf_table[sram_value[5]] - cdf_min) << 8) - (cdf_table[sram_value[5]] - cdf_min);
assign mult[6] = ((cdf_table[sram_value[6]] - cdf_min) << 8) - (cdf_table[sram_value[6]] - cdf_min);
assign mult[7] = ((cdf_table[sram_value[7]] - cdf_min) << 8) - (cdf_table[sram_value[7]] - cdf_min);
assign mult[8] = ((cdf_table[sram_value[8]] - cdf_min) << 8) - (cdf_table[sram_value[8]] - cdf_min);
assign mult[9] = ((cdf_table[sram_value[9]] - cdf_min) << 8) - (cdf_table[sram_value[9]] - cdf_min);
assign mult[10] = ((cdf_table[sram_value[10]] - cdf_min) << 8) - (cdf_table[sram_value[10]] - cdf_min);
assign mult[11] = ((cdf_table[sram_value[11]] - cdf_min) << 8) - (cdf_table[sram_value[11]] - cdf_min);
assign mult[12] = ((cdf_table[sram_value[12]] - cdf_min) << 8) - (cdf_table[sram_value[12]] - cdf_min);
assign mult[13] = ((cdf_table[sram_value[13]] - cdf_min) << 8) - (cdf_table[sram_value[13]] - cdf_min);
assign mult[14] = ((cdf_table[sram_value[14]] - cdf_min) << 8) - (cdf_table[sram_value[14]] - cdf_min);
assign mult[15] = ((cdf_table[sram_value[15]] - cdf_min) << 8) - (cdf_table[sram_value[15]] - cdf_min);

assign histogram_out[7:0] = mult[0] / divisor;
assign histogram_out[15:8] = mult[1] / divisor;
assign histogram_out[23:16] = mult[2] / divisor;
assign histogram_out[31:24] = mult[3] / divisor;
assign histogram_out[39:32] = mult[4] / divisor;
assign histogram_out[47:40] = mult[5] / divisor;
assign histogram_out[55:48] = mult[6] / divisor;
assign histogram_out[63:56] = mult[7] / divisor;
assign histogram_out[71:64] = mult[8] / divisor;
assign histogram_out[79:72] = mult[9] / divisor;
assign histogram_out[87:80] = mult[10] / divisor;
assign histogram_out[95:88] = mult[11] / divisor;
assign histogram_out[103:96] = mult[12] / divisor;
assign histogram_out[111:104] = mult[13] / divisor;
assign histogram_out[119:112] = mult[14] / divisor;
assign histogram_out[127:120] = mult[15] / divisor;

// histogram pipe
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) histogram_pipe <= 128'b0;
    else if(n_state == HISTOGRAM && PIC_loaded) histogram_pipe <= histogram_out;
end

////////////////////////////////////////////// EROSION & DILATION ///////////////////////////////////////////////////

// line_buffer
always@(posedge clk or negedge rst_n) begin 
    if(!rst_n) line_buffer <= 1792'b0;
    else begin 
        case(n_state)
            EROSION: begin 
                if(!PIC_loaded && rvalid_m_inf) line_buffer <= {rdata_m_inf, line_buffer[1791:128]};
                else if(PIC_loaded) line_buffer <= {128'b0, line_buffer[1791:128]};
            end
            DILATION: begin 
                if(!PIC_loaded && rvalid_m_inf) line_buffer <= {rdata_m_inf, line_buffer[1791:128]};
                else if(PIC_loaded) line_buffer <= {128'b0, line_buffer[1791:128]};
            end
            INPUT: line_buffer <= 1792'b0;
        endcase
    end
end

genvar s;
generate
    for(s = 0; s < 16; s = s + 1) begin :se_loop
        // SE_table
        always@(posedge clk or negedge rst_n) begin 
            if(!rst_n) SE_table[s] <= 8'b0;
            else if(n_state == BUBBLE && !which_op[0]) SE_table[s] <= pixel_value[s];
            else if(n_state == BUBBLE && which_op[0]) SE_table[s] <= pixel_value[15 - s];
        end

        assign fadd0[s] = (add0[s][8])? 8'hff : add0[s][7:0];
        assign fadd1[s] = (add1[s][8])? 8'hff : add1[s][7:0];
        assign fadd2[s] = (add2[s][8])? 8'hff : add2[s][7:0];
        assign fadd3[s] = (add3[s][8])? 8'hff : add3[s][7:0];
        assign fadd4[s] = (add4[s][8])? 8'hff : add4[s][7:0];
        assign fadd5[s] = (add5[s][8])? 8'hff : add5[s][7:0];
        assign fadd6[s] = (add6[s][8])? 8'hff : add6[s][7:0];
        assign fadd7[s] = (add7[s][8])? 8'hff : add7[s][7:0];
        assign fadd8[s] = (add8[s][8])? 8'hff : add8[s][7:0];
        assign fadd9[s] = (add9[s][8])? 8'hff : add9[s][7:0];
        assign fadd10[s] = (add10[s][8])? 8'hff : add10[s][7:0];
        assign fadd11[s] = (add11[s][8])? 8'hff : add11[s][7:0];
        assign fadd12[s] = (add12[s][8])? 8'hff : add12[s][7:0];
        assign fadd13[s] = (add13[s][8])? 8'hff : add13[s][7:0];
        assign fadd14[s] = (add14[s][8])? 8'hff : add14[s][7:0];
        assign fadd15[s] = (add15[s][8])? 8'hff : add15[s][7:0];

        assign min0[s] = (sub0[s] < sub1[s])? sub0[s] : sub1[s];
        assign min1[s] = (sub2[s] < sub3[s])? sub2[s] : sub3[s];
        assign min2[s] = (sub4[s] < sub5[s])? sub4[s] : sub5[s];
        assign min3[s] = (sub6[s] < sub7[s])? sub6[s] : sub7[s];
        assign min4[s] = (sub8[s] < sub9[s])? sub8[s] : sub9[s];
        assign min5[s] = (sub10[s] < sub11[s])? sub10[s] : sub11[s];
        assign min6[s] = (sub12[s] < sub13[s])? sub12[s] : sub13[s];
        assign min7[s] = (sub14[s] < sub15[s])? sub14[s] : sub15[s];
        assign min8[s] = (min0[s] < min1[s])? min0[s] : min1[s];
        assign min9[s] = (min2[s] < min3[s])? min2[s] : min3[s];
        assign min10[s] = (min4[s] < min5[s])? min4[s] : min5[s];
        assign min11[s] = (min6[s] < min7[s])? min6[s] : min7[s];
        assign min12[s] = (min8[s] < min9[s])? min8[s] : min9[s];
        assign min13[s] = (min10[s] < min11[s])? min10[s] : min11[s];
        assign min14[s] = (min12[s] < min13[s])? min12[s] : min13[s];
        
        assign max0[s] = (fadd0[s] > fadd1[s])? fadd0[s] : fadd1[s];
        assign max1[s] = (fadd2[s] > fadd3[s])? fadd2[s] : fadd3[s];
        assign max2[s] = (fadd4[s] > fadd5[s])? fadd4[s] : fadd5[s];
        assign max3[s] = (fadd6[s] > fadd7[s])? fadd6[s] : fadd7[s];
        assign max4[s] = (fadd8[s] > fadd9[s])? fadd8[s] : fadd9[s];
        assign max5[s] = (fadd10[s] > fadd11[s])? fadd10[s] : fadd11[s];
        assign max6[s] = (fadd12[s] > fadd13[s])? fadd12[s] : fadd13[s];
        assign max7[s] = (fadd14[s] > fadd15[s])? fadd14[s] : fadd15[s];
        assign max8[s] = (max0[s] > max1[s])? max0[s] : max1[s];
        assign max9[s] = (max2[s] > max3[s])? max2[s] : max3[s];
        assign max10[s] = (max4[s] > max5[s])? max4[s] : max5[s];
        assign max11[s] = (max6[s] > max7[s])? max6[s] : max7[s];
        assign max12[s] = (max8[s] > max9[s])? max8[s] : max9[s];
        assign max13[s] = (max10[s] > max11[s])? max10[s] : max11[s];
        assign max14[s] = (max12[s] > max13[s])? max12[s] : max13[s];
    end
endgenerate

assign erosion_out[7:0]     = min14[0];
assign erosion_out[15:8]    = min14[1];
assign erosion_out[23:16]   = min14[2];
assign erosion_out[31:24]   = min14[3];
assign erosion_out[39:32]   = min14[4];
assign erosion_out[47:40]   = min14[5];
assign erosion_out[55:48]   = min14[6];
assign erosion_out[63:56]   = min14[7];
assign erosion_out[71:64]   = min14[8];
assign erosion_out[79:72]   = min14[9];
assign erosion_out[87:80]   = min14[10];
assign erosion_out[95:88]   = min14[11];
assign erosion_out[103:96]  = min14[12];
assign erosion_out[111:104] = min14[13];
assign erosion_out[119:112] = min14[14];
assign erosion_out[127:120] = min14[15];

assign dilation_out[7:0]     = max14[0];
assign dilation_out[15:8]    = max14[1];
assign dilation_out[23:16]   = max14[2];
assign dilation_out[31:24]   = max14[3];
assign dilation_out[39:32]   = max14[4];
assign dilation_out[47:40]   = max14[5];
assign dilation_out[55:48]   = max14[6];
assign dilation_out[63:56]   = max14[7];
assign dilation_out[71:64]   = max14[8];
assign dilation_out[79:72]   = max14[9];
assign dilation_out[87:80]   = max14[10];
assign dilation_out[95:88]   = max14[11];
assign dilation_out[103:96]  = max14[12];
assign dilation_out[111:104] = max14[13];
assign dilation_out[119:112] = max14[14];
assign dilation_out[127:120] = max14[15];

//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LEVEL 111111111 ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

assign sub0[0]  = (line_buffer[7:0]     > SE_table[0])? line_buffer[7:0]     - SE_table[0] : 8'b0;
assign sub0[1]  = (line_buffer[15:8]    > SE_table[0])? line_buffer[15:8]    - SE_table[0] : 8'b0;
assign sub0[2]  = (line_buffer[23:16]   > SE_table[0])? line_buffer[23:16]   - SE_table[0] : 8'b0;
assign sub0[3]  = (line_buffer[31:24]   > SE_table[0])? line_buffer[31:24]   - SE_table[0] : 8'b0;
assign sub0[4]  = (line_buffer[39:32]   > SE_table[0])? line_buffer[39:32]   - SE_table[0] : 8'b0;
assign sub0[5]  = (line_buffer[47:40]   > SE_table[0])? line_buffer[47:40]   - SE_table[0] : 8'b0;
assign sub0[6]  = (line_buffer[55:48]   > SE_table[0])? line_buffer[55:48]   - SE_table[0] : 8'b0;
assign sub0[7]  = (line_buffer[63:56]   > SE_table[0])? line_buffer[63:56]   - SE_table[0] : 8'b0;
assign sub0[8]  = (line_buffer[71:64]   > SE_table[0])? line_buffer[71:64]   - SE_table[0] : 8'b0;
assign sub0[9]  = (line_buffer[79:72]   > SE_table[0])? line_buffer[79:72]   - SE_table[0] : 8'b0;
assign sub0[10] = (line_buffer[87:80]   > SE_table[0])? line_buffer[87:80]   - SE_table[0] : 8'b0;
assign sub0[11] = (line_buffer[95:88]   > SE_table[0])? line_buffer[95:88]   - SE_table[0] : 8'b0;
assign sub0[12] = (line_buffer[103:96]  > SE_table[0])? line_buffer[103:96]  - SE_table[0] : 8'b0;
assign sub0[13] = (line_buffer[111:104] > SE_table[0])? line_buffer[111:104] - SE_table[0] : 8'b0;
assign sub0[14] = (line_buffer[119:112] > SE_table[0])? line_buffer[119:112] - SE_table[0] : 8'b0;
assign sub0[15] = (line_buffer[127:120] > SE_table[0])? line_buffer[127:120] - SE_table[0] : 8'b0;
assign add0[0]  = line_buffer[7:0]     + SE_table[0];
assign add0[1]  = line_buffer[15:8]    + SE_table[0];
assign add0[2]  = line_buffer[23:16]   + SE_table[0];
assign add0[3]  = line_buffer[31:24]   + SE_table[0];
assign add0[4]  = line_buffer[39:32]   + SE_table[0];
assign add0[5]  = line_buffer[47:40]   + SE_table[0];
assign add0[6]  = line_buffer[55:48]   + SE_table[0];
assign add0[7]  = line_buffer[63:56]   + SE_table[0];
assign add0[8]  = line_buffer[71:64]   + SE_table[0];
assign add0[9]  = line_buffer[79:72]   + SE_table[0];
assign add0[10] = line_buffer[87:80]   + SE_table[0];
assign add0[11] = line_buffer[95:88]   + SE_table[0];
assign add0[12] = line_buffer[103:96]  + SE_table[0];
assign add0[13] = line_buffer[111:104] + SE_table[0];
assign add0[14] = line_buffer[119:112] + SE_table[0];
assign add0[15] = line_buffer[127:120] + SE_table[0];

assign sub1[0]  = (line_buffer[15:8]    > SE_table[1])? line_buffer[15:8]    - SE_table[1] : 8'b0;
assign sub1[1]  = (line_buffer[23:16]   > SE_table[1])? line_buffer[23:16]   - SE_table[1] : 8'b0;
assign sub1[2]  = (line_buffer[31:24]   > SE_table[1])? line_buffer[31:24]   - SE_table[1] : 8'b0;
assign sub1[3]  = (line_buffer[39:32]   > SE_table[1])? line_buffer[39:32]   - SE_table[1] : 8'b0;
assign sub1[4]  = (line_buffer[47:40]   > SE_table[1])? line_buffer[47:40]   - SE_table[1] : 8'b0;
assign sub1[5]  = (line_buffer[55:48]   > SE_table[1])? line_buffer[55:48]   - SE_table[1] : 8'b0;
assign sub1[6]  = (line_buffer[63:56]   > SE_table[1])? line_buffer[63:56]   - SE_table[1] : 8'b0;
assign sub1[7]  = (line_buffer[71:64]   > SE_table[1])? line_buffer[71:64]   - SE_table[1] : 8'b0;
assign sub1[8]  = (line_buffer[79:72]   > SE_table[1])? line_buffer[79:72]   - SE_table[1] : 8'b0;
assign sub1[9]  = (line_buffer[87:80]   > SE_table[1])? line_buffer[87:80]   - SE_table[1] : 8'b0;
assign sub1[10] = (line_buffer[95:88]   > SE_table[1])? line_buffer[95:88]   - SE_table[1] : 8'b0;
assign sub1[11] = (line_buffer[103:96]  > SE_table[1])? line_buffer[103:96]  - SE_table[1] : 8'b0;
assign sub1[12] = (line_buffer[111:104] > SE_table[1])? line_buffer[111:104] - SE_table[1] : 8'b0;
assign sub1[13] = (line_buffer[119:112] > SE_table[1])? line_buffer[119:112] - SE_table[1] : 8'b0;
assign sub1[14] = (line_buffer[127:120] > SE_table[1])? line_buffer[127:120] - SE_table[1] : 8'b0;
assign sub1[15] = (sram_counter[1:0] != 2'b11 && line_buffer[135:128] > SE_table[1])? line_buffer[135:128] - SE_table[1] : 8'b0;
assign add1[0]  = line_buffer[15:8]    + SE_table[1];
assign add1[1]  = line_buffer[23:16]   + SE_table[1];
assign add1[2]  = line_buffer[31:24]   + SE_table[1];
assign add1[3]  = line_buffer[39:32]   + SE_table[1];
assign add1[4]  = line_buffer[47:40]   + SE_table[1];
assign add1[5]  = line_buffer[55:48]   + SE_table[1];
assign add1[6]  = line_buffer[63:56]   + SE_table[1];
assign add1[7]  = line_buffer[71:64]   + SE_table[1];
assign add1[8]  = line_buffer[79:72]   + SE_table[1];
assign add1[9]  = line_buffer[87:80]   + SE_table[1];
assign add1[10] = line_buffer[95:88]   + SE_table[1];
assign add1[11] = line_buffer[103:96]  + SE_table[1];
assign add1[12] = line_buffer[111:104] + SE_table[1];
assign add1[13] = line_buffer[119:112] + SE_table[1];
assign add1[14] = line_buffer[127:120] + SE_table[1];
assign add1[15] = (sram_counter[1:0] == 2'b11)? SE_table[1] : line_buffer[135:128] + SE_table[1];

assign sub2[0]  = (line_buffer[23:16]   > SE_table[2])? line_buffer[23:16]   - SE_table[2] : 8'b0;
assign sub2[1]  = (line_buffer[31:24]   > SE_table[2])? line_buffer[31:24]   - SE_table[2] : 8'b0;
assign sub2[2]  = (line_buffer[39:32]   > SE_table[2])? line_buffer[39:32]   - SE_table[2] : 8'b0;
assign sub2[3]  = (line_buffer[47:40]   > SE_table[2])? line_buffer[47:40]   - SE_table[2] : 8'b0;
assign sub2[4]  = (line_buffer[55:48]   > SE_table[2])? line_buffer[55:48]   - SE_table[2] : 8'b0;
assign sub2[5]  = (line_buffer[63:56]   > SE_table[2])? line_buffer[63:56]   - SE_table[2] : 8'b0;
assign sub2[6]  = (line_buffer[71:64]   > SE_table[2])? line_buffer[71:64]   - SE_table[2] : 8'b0;
assign sub2[7]  = (line_buffer[79:72]   > SE_table[2])? line_buffer[79:72]   - SE_table[2] : 8'b0;
assign sub2[8]  = (line_buffer[87:80]   > SE_table[2])? line_buffer[87:80]   - SE_table[2] : 8'b0;
assign sub2[9]  = (line_buffer[95:88]   > SE_table[2])? line_buffer[95:88]   - SE_table[2] : 8'b0;
assign sub2[10] = (line_buffer[103:96]  > SE_table[2])? line_buffer[103:96]  - SE_table[2] : 8'b0;
assign sub2[11] = (line_buffer[111:104] > SE_table[2])? line_buffer[111:104] - SE_table[2] : 8'b0;
assign sub2[12] = (line_buffer[119:112] > SE_table[2])? line_buffer[119:112] - SE_table[2] : 8'b0;
assign sub2[13] = (line_buffer[127:120] > SE_table[2])? line_buffer[127:120] - SE_table[2] : 8'b0;
assign sub2[14] = (sram_counter[1:0] != 2'b11 && line_buffer[135:128] > SE_table[2])? line_buffer[135:128] - SE_table[2] : 8'b0;
assign sub2[15] = (sram_counter[1:0] != 2'b11 && line_buffer[143:136] > SE_table[2])? line_buffer[143:136] - SE_table[2] : 8'b0;
assign add2[0]  = line_buffer[23:16]   + SE_table[2];
assign add2[1]  = line_buffer[31:24]   + SE_table[2];
assign add2[2]  = line_buffer[39:32]   + SE_table[2];
assign add2[3]  = line_buffer[47:40]   + SE_table[2];
assign add2[4]  = line_buffer[55:48]   + SE_table[2];
assign add2[5]  = line_buffer[63:56]   + SE_table[2];
assign add2[6]  = line_buffer[71:64]   + SE_table[2];
assign add2[7]  = line_buffer[79:72]   + SE_table[2];
assign add2[8]  = line_buffer[87:80]   + SE_table[2];
assign add2[9]  = line_buffer[95:88]   + SE_table[2];
assign add2[10] = line_buffer[103:96]  + SE_table[2];
assign add2[11] = line_buffer[111:104] + SE_table[2];
assign add2[12] = line_buffer[119:112] + SE_table[2];
assign add2[13] = line_buffer[127:120] + SE_table[2];
assign add2[14] = (sram_counter[1:0] == 2'b11)? SE_table[2] : line_buffer[135:128] + SE_table[2];
assign add2[15] = (sram_counter[1:0] == 2'b11)? SE_table[2] : line_buffer[143:136] + SE_table[2];

assign sub3[0]  = (line_buffer[31:24]   > SE_table[3])? line_buffer[31:24]   - SE_table[3] : 8'b0;
assign sub3[1]  = (line_buffer[39:32]   > SE_table[3])? line_buffer[39:32]   - SE_table[3] : 8'b0;
assign sub3[2]  = (line_buffer[47:40]   > SE_table[3])? line_buffer[47:40]   - SE_table[3] : 8'b0;
assign sub3[3]  = (line_buffer[55:48]   > SE_table[3])? line_buffer[55:48]   - SE_table[3] : 8'b0;
assign sub3[4]  = (line_buffer[63:56]   > SE_table[3])? line_buffer[63:56]   - SE_table[3] : 8'b0;
assign sub3[5]  = (line_buffer[71:64]   > SE_table[3])? line_buffer[71:64]   - SE_table[3] : 8'b0;
assign sub3[6]  = (line_buffer[79:72]   > SE_table[3])? line_buffer[79:72]   - SE_table[3] : 8'b0;
assign sub3[7]  = (line_buffer[87:80]   > SE_table[3])? line_buffer[87:80]   - SE_table[3] : 8'b0;
assign sub3[8]  = (line_buffer[95:88]   > SE_table[3])? line_buffer[95:88]   - SE_table[3] : 8'b0;
assign sub3[9]  = (line_buffer[103:96]  > SE_table[3])? line_buffer[103:96]  - SE_table[3] : 8'b0;
assign sub3[10] = (line_buffer[111:104] > SE_table[3])? line_buffer[111:104] - SE_table[3] : 8'b0;
assign sub3[11] = (line_buffer[119:112] > SE_table[3])? line_buffer[119:112] - SE_table[3] : 8'b0;
assign sub3[12] = (line_buffer[127:120] > SE_table[3])? line_buffer[127:120] - SE_table[3] : 8'b0;
assign sub3[13] = (sram_counter[1:0] != 2'b11 && line_buffer[135:128] > SE_table[3])? line_buffer[135:128] - SE_table[3] : 8'b0;
assign sub3[14] = (sram_counter[1:0] != 2'b11 && line_buffer[143:136] > SE_table[3])? line_buffer[143:136] - SE_table[3] : 8'b0;
assign sub3[15] = (sram_counter[1:0] != 2'b11 && line_buffer[151:144] > SE_table[3])? line_buffer[151:144] - SE_table[3] : 8'b0;
assign add3[0]  = line_buffer[31:24]   + SE_table[3];
assign add3[1]  = line_buffer[39:32]   + SE_table[3];
assign add3[2]  = line_buffer[47:40]   + SE_table[3];
assign add3[3]  = line_buffer[55:48]   + SE_table[3];
assign add3[4]  = line_buffer[63:56]   + SE_table[3];
assign add3[5]  = line_buffer[71:64]   + SE_table[3];
assign add3[6]  = line_buffer[79:72]   + SE_table[3];
assign add3[7]  = line_buffer[87:80]   + SE_table[3];
assign add3[8]  = line_buffer[95:88]   + SE_table[3];
assign add3[9]  = line_buffer[103:96]  + SE_table[3];
assign add3[10] = line_buffer[111:104] + SE_table[3];
assign add3[11] = line_buffer[119:112] + SE_table[3];
assign add3[12] = line_buffer[127:120] + SE_table[3];
assign add3[13] = (sram_counter[1:0] == 2'b11)? SE_table[3] : line_buffer[135:128] + SE_table[3];
assign add3[14] = (sram_counter[1:0] == 2'b11)? SE_table[3] : line_buffer[143:136] + SE_table[3];
assign add3[15] = (sram_counter[1:0] == 2'b11)? SE_table[3] : line_buffer[151:144] + SE_table[3];





//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LEVEL 222222222 ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////



assign sub4[0]  = (line_buffer[519:512] > SE_table[4])? line_buffer[519:512] - SE_table[4] : 8'b0;
assign sub4[1]  = (line_buffer[527:520] > SE_table[4])? line_buffer[527:520] - SE_table[4] : 8'b0;
assign sub4[2]  = (line_buffer[535:528] > SE_table[4])? line_buffer[535:528] - SE_table[4] : 8'b0;
assign sub4[3]  = (line_buffer[543:536] > SE_table[4])? line_buffer[543:536] - SE_table[4] : 8'b0;
assign sub4[4]  = (line_buffer[551:544] > SE_table[4])? line_buffer[551:544] - SE_table[4] : 8'b0;
assign sub4[5]  = (line_buffer[559:552] > SE_table[4])? line_buffer[559:552] - SE_table[4] : 8'b0;
assign sub4[6]  = (line_buffer[567:560] > SE_table[4])? line_buffer[567:560] - SE_table[4] : 8'b0;
assign sub4[7]  = (line_buffer[575:568] > SE_table[4])? line_buffer[575:568] - SE_table[4] : 8'b0;
assign sub4[8]  = (line_buffer[583:576] > SE_table[4])? line_buffer[583:576] - SE_table[4] : 8'b0;
assign sub4[9]  = (line_buffer[591:584] > SE_table[4])? line_buffer[591:584] - SE_table[4] : 8'b0;
assign sub4[10] = (line_buffer[599:592] > SE_table[4])? line_buffer[599:592] - SE_table[4] : 8'b0;
assign sub4[11] = (line_buffer[607:600] > SE_table[4])? line_buffer[607:600] - SE_table[4] : 8'b0;
assign sub4[12] = (line_buffer[615:608] > SE_table[4])? line_buffer[615:608] - SE_table[4] : 8'b0;
assign sub4[13] = (line_buffer[623:616] > SE_table[4])? line_buffer[623:616] - SE_table[4] : 8'b0;
assign sub4[14] = (line_buffer[631:624] > SE_table[4])? line_buffer[631:624] - SE_table[4] : 8'b0;
assign sub4[15] = (line_buffer[639:632] > SE_table[4])? line_buffer[639:632] - SE_table[4] : 8'b0;
assign add4[0]  = line_buffer[519:512] + SE_table[4];
assign add4[1]  = line_buffer[527:520] + SE_table[4];
assign add4[2]  = line_buffer[535:528] + SE_table[4];
assign add4[3]  = line_buffer[543:536] + SE_table[4];
assign add4[4]  = line_buffer[551:544] + SE_table[4];
assign add4[5]  = line_buffer[559:552] + SE_table[4];
assign add4[6]  = line_buffer[567:560] + SE_table[4];
assign add4[7]  = line_buffer[575:568] + SE_table[4];
assign add4[8]  = line_buffer[583:576] + SE_table[4];
assign add4[9]  = line_buffer[591:584] + SE_table[4];
assign add4[10] = line_buffer[599:592] + SE_table[4];
assign add4[11] = line_buffer[607:600] + SE_table[4];
assign add4[12] = line_buffer[615:608] + SE_table[4];
assign add4[13] = line_buffer[623:616] + SE_table[4];
assign add4[14] = line_buffer[631:624] + SE_table[4];
assign add4[15] = line_buffer[639:632] + SE_table[4];

assign sub5[0]  = (line_buffer[527:520] > SE_table[5])? line_buffer[527:520] - SE_table[5] : 8'b0;
assign sub5[1]  = (line_buffer[535:528] > SE_table[5])? line_buffer[535:528] - SE_table[5] : 8'b0;
assign sub5[2]  = (line_buffer[543:536] > SE_table[5])? line_buffer[543:536] - SE_table[5] : 8'b0;
assign sub5[3]  = (line_buffer[551:544] > SE_table[5])? line_buffer[551:544] - SE_table[5] : 8'b0;
assign sub5[4]  = (line_buffer[559:552] > SE_table[5])? line_buffer[559:552] - SE_table[5] : 8'b0;
assign sub5[5]  = (line_buffer[567:560] > SE_table[5])? line_buffer[567:560] - SE_table[5] : 8'b0;
assign sub5[6]  = (line_buffer[575:568] > SE_table[5])? line_buffer[575:568] - SE_table[5] : 8'b0;
assign sub5[7]  = (line_buffer[583:576] > SE_table[5])? line_buffer[583:576] - SE_table[5] : 8'b0;
assign sub5[8]  = (line_buffer[591:584] > SE_table[5])? line_buffer[591:584] - SE_table[5] : 8'b0;
assign sub5[9]  = (line_buffer[599:592] > SE_table[5])? line_buffer[599:592] - SE_table[5] : 8'b0;
assign sub5[10] = (line_buffer[607:600] > SE_table[5])? line_buffer[607:600] - SE_table[5] : 8'b0;
assign sub5[11] = (line_buffer[615:608] > SE_table[5])? line_buffer[615:608] - SE_table[5] : 8'b0;
assign sub5[12] = (line_buffer[623:616] > SE_table[5])? line_buffer[623:616] - SE_table[5] : 8'b0;
assign sub5[13] = (line_buffer[631:624] > SE_table[5])? line_buffer[631:624] - SE_table[5] : 8'b0;
assign sub5[14] = (line_buffer[639:632] > SE_table[5])? line_buffer[639:632] - SE_table[5] : 8'b0;
assign sub5[15] = (sram_counter[1:0] != 2'b11 && line_buffer[647:640] > SE_table[5])? line_buffer[647:640] - SE_table[5] : 8'b0;
assign add5[0]  = line_buffer[527:520] + SE_table[5];
assign add5[1]  = line_buffer[535:528] + SE_table[5];
assign add5[2]  = line_buffer[543:536] + SE_table[5];
assign add5[3]  = line_buffer[551:544] + SE_table[5];
assign add5[4]  = line_buffer[559:552] + SE_table[5];
assign add5[5]  = line_buffer[567:560] + SE_table[5];
assign add5[6]  = line_buffer[575:568] + SE_table[5];
assign add5[7]  = line_buffer[583:576] + SE_table[5];
assign add5[8]  = line_buffer[591:584] + SE_table[5];
assign add5[9]  = line_buffer[599:592] + SE_table[5];
assign add5[10] = line_buffer[607:600] + SE_table[5];
assign add5[11] = line_buffer[615:608] + SE_table[5];
assign add5[12] = line_buffer[623:616] + SE_table[5];
assign add5[13] = line_buffer[631:624] + SE_table[5];
assign add5[14] = line_buffer[639:632] + SE_table[5];
assign add5[15] = (sram_counter[1:0] == 2'b11)? SE_table[5] : line_buffer[647:640] + SE_table[5];

assign sub6[0]  = (line_buffer[535:528] > SE_table[6])? line_buffer[535:528] - SE_table[6] : 8'b0;
assign sub6[1]  = (line_buffer[543:536] > SE_table[6])? line_buffer[543:536] - SE_table[6] : 8'b0;
assign sub6[2]  = (line_buffer[551:544] > SE_table[6])? line_buffer[551:544] - SE_table[6] : 8'b0;
assign sub6[3]  = (line_buffer[559:552] > SE_table[6])? line_buffer[559:552] - SE_table[6] : 8'b0;
assign sub6[4]  = (line_buffer[567:560] > SE_table[6])? line_buffer[567:560] - SE_table[6] : 8'b0;
assign sub6[5]  = (line_buffer[575:568] > SE_table[6])? line_buffer[575:568] - SE_table[6] : 8'b0;
assign sub6[6]  = (line_buffer[583:576] > SE_table[6])? line_buffer[583:576] - SE_table[6] : 8'b0;
assign sub6[7]  = (line_buffer[591:584] > SE_table[6])? line_buffer[591:584] - SE_table[6] : 8'b0;
assign sub6[8]  = (line_buffer[599:592] > SE_table[6])? line_buffer[599:592] - SE_table[6] : 8'b0;
assign sub6[9]  = (line_buffer[607:600] > SE_table[6])? line_buffer[607:600] - SE_table[6] : 8'b0;
assign sub6[10] = (line_buffer[615:608] > SE_table[6])? line_buffer[615:608] - SE_table[6] : 8'b0;
assign sub6[11] = (line_buffer[623:616] > SE_table[6])? line_buffer[623:616] - SE_table[6] : 8'b0;
assign sub6[12] = (line_buffer[631:624] > SE_table[6])? line_buffer[631:624] - SE_table[6] : 8'b0;
assign sub6[13] = (line_buffer[639:632] > SE_table[6])? line_buffer[639:632] - SE_table[6] : 8'b0;
assign sub6[14] = (sram_counter[1:0] != 2'b11 && line_buffer[647:640] > SE_table[6])? line_buffer[647:640] - SE_table[6] : 8'b0;
assign sub6[15] = (sram_counter[1:0] != 2'b11 && line_buffer[655:648] > SE_table[6])? line_buffer[655:648] - SE_table[6] : 8'b0;
assign add6[0]  = line_buffer[535:528] + SE_table[6];
assign add6[1]  = line_buffer[543:536] + SE_table[6];
assign add6[2]  = line_buffer[551:544] + SE_table[6];
assign add6[3]  = line_buffer[559:552] + SE_table[6];
assign add6[4]  = line_buffer[567:560] + SE_table[6];
assign add6[5]  = line_buffer[575:568] + SE_table[6];
assign add6[6]  = line_buffer[583:576] + SE_table[6];
assign add6[7]  = line_buffer[591:584] + SE_table[6];
assign add6[8]  = line_buffer[599:592] + SE_table[6];
assign add6[9]  = line_buffer[607:600] + SE_table[6];
assign add6[10] = line_buffer[615:608] + SE_table[6];
assign add6[11] = line_buffer[623:616] + SE_table[6];
assign add6[12] = line_buffer[631:624] + SE_table[6];
assign add6[13] = line_buffer[639:632] + SE_table[6];
assign add6[14] = (sram_counter[1:0] == 2'b11)? SE_table[6] : line_buffer[647:640] + SE_table[6];
assign add6[15] = (sram_counter[1:0] == 2'b11)? SE_table[6] : line_buffer[655:648] + SE_table[6];

assign sub7[0]  = (line_buffer[543:536] > SE_table[7])? line_buffer[543:536] - SE_table[7] : 8'b0;
assign sub7[1]  = (line_buffer[551:544] > SE_table[7])? line_buffer[551:544] - SE_table[7] : 8'b0;
assign sub7[2]  = (line_buffer[559:552] > SE_table[7])? line_buffer[559:552] - SE_table[7] : 8'b0;
assign sub7[3]  = (line_buffer[567:560] > SE_table[7])? line_buffer[567:560] - SE_table[7] : 8'b0;
assign sub7[4]  = (line_buffer[575:568] > SE_table[7])? line_buffer[575:568] - SE_table[7] : 8'b0;
assign sub7[5]  = (line_buffer[583:576] > SE_table[7])? line_buffer[583:576] - SE_table[7] : 8'b0;
assign sub7[6]  = (line_buffer[591:584] > SE_table[7])? line_buffer[591:584] - SE_table[7] : 8'b0;
assign sub7[7]  = (line_buffer[599:592] > SE_table[7])? line_buffer[599:592] - SE_table[7] : 8'b0;
assign sub7[8]  = (line_buffer[607:600] > SE_table[7])? line_buffer[607:600] - SE_table[7] : 8'b0;
assign sub7[9]  = (line_buffer[615:608] > SE_table[7])? line_buffer[615:608] - SE_table[7] : 8'b0;
assign sub7[10] = (line_buffer[623:616] > SE_table[7])? line_buffer[623:616] - SE_table[7] : 8'b0;
assign sub7[11] = (line_buffer[631:624] > SE_table[7])? line_buffer[631:624] - SE_table[7] : 8'b0;
assign sub7[12] = (line_buffer[639:632] > SE_table[7])? line_buffer[639:632] - SE_table[7] : 8'b0;
assign sub7[13] = (sram_counter[1:0] != 2'b11 && line_buffer[647:640] > SE_table[7])? line_buffer[647:640] - SE_table[7] : 8'b0;
assign sub7[14] = (sram_counter[1:0] != 2'b11 && line_buffer[655:648] > SE_table[7])? line_buffer[655:648] - SE_table[7] : 8'b0;
assign sub7[15] = (sram_counter[1:0] != 2'b11 && line_buffer[663:656] > SE_table[7])? line_buffer[663:656] - SE_table[7] : 8'b0;
assign add7[0]  = line_buffer[543:536] + SE_table[7];
assign add7[1]  = line_buffer[551:544] + SE_table[7];
assign add7[2]  = line_buffer[559:552] + SE_table[7];
assign add7[3]  = line_buffer[567:560] + SE_table[7];
assign add7[4]  = line_buffer[575:568] + SE_table[7];
assign add7[5]  = line_buffer[583:576] + SE_table[7];
assign add7[6]  = line_buffer[591:584] + SE_table[7];
assign add7[7]  = line_buffer[599:592] + SE_table[7];
assign add7[8]  = line_buffer[607:600] + SE_table[7];
assign add7[9]  = line_buffer[615:608] + SE_table[7];
assign add7[10] = line_buffer[623:616] + SE_table[7];
assign add7[11] = line_buffer[631:624] + SE_table[7];
assign add7[12] = line_buffer[639:632] + SE_table[7];
assign add7[13] = (sram_counter[1:0] == 2'b11)? SE_table[7] : line_buffer[647:640] + SE_table[7];
assign add7[14] = (sram_counter[1:0] == 2'b11)? SE_table[7] : line_buffer[655:648] + SE_table[7];
assign add7[15] = (sram_counter[1:0] == 2'b11)? SE_table[7] : line_buffer[663:656] + SE_table[7];




//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LEVEL 333333333 ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////



assign sub8[0]  = (line_buffer[1031:1024] > SE_table[8])? line_buffer[1031:1024] - SE_table[8] : 8'b0;
assign sub8[1]  = (line_buffer[1039:1032] > SE_table[8])? line_buffer[1039:1032] - SE_table[8] : 8'b0;
assign sub8[2]  = (line_buffer[1047:1040] > SE_table[8])? line_buffer[1047:1040] - SE_table[8] : 8'b0;
assign sub8[3]  = (line_buffer[1055:1048] > SE_table[8])? line_buffer[1055:1048] - SE_table[8] : 8'b0;
assign sub8[4]  = (line_buffer[1063:1056] > SE_table[8])? line_buffer[1063:1056] - SE_table[8] : 8'b0;
assign sub8[5]  = (line_buffer[1071:1064] > SE_table[8])? line_buffer[1071:1064] - SE_table[8] : 8'b0;
assign sub8[6]  = (line_buffer[1079:1072] > SE_table[8])? line_buffer[1079:1072] - SE_table[8] : 8'b0;
assign sub8[7]  = (line_buffer[1087:1080] > SE_table[8])? line_buffer[1087:1080] - SE_table[8] : 8'b0;
assign sub8[8]  = (line_buffer[1095:1088] > SE_table[8])? line_buffer[1095:1088] - SE_table[8] : 8'b0;
assign sub8[9]  = (line_buffer[1103:1096] > SE_table[8])? line_buffer[1103:1096] - SE_table[8] : 8'b0;
assign sub8[10] = (line_buffer[1111:1104] > SE_table[8])? line_buffer[1111:1104] - SE_table[8] : 8'b0;
assign sub8[11] = (line_buffer[1119:1112] > SE_table[8])? line_buffer[1119:1112] - SE_table[8] : 8'b0;
assign sub8[12] = (line_buffer[1127:1120] > SE_table[8])? line_buffer[1127:1120] - SE_table[8] : 8'b0;
assign sub8[13] = (line_buffer[1135:1128] > SE_table[8])? line_buffer[1135:1128] - SE_table[8] : 8'b0;
assign sub8[14] = (line_buffer[1143:1136] > SE_table[8])? line_buffer[1143:1136] - SE_table[8] : 8'b0;
assign sub8[15] = (line_buffer[1151:1144] > SE_table[8])? line_buffer[1151:1144] - SE_table[8] : 8'b0;
assign add8[0]  = line_buffer[1031:1024] + SE_table[8];
assign add8[1]  = line_buffer[1039:1032] + SE_table[8];
assign add8[2]  = line_buffer[1047:1040] + SE_table[8];
assign add8[3]  = line_buffer[1055:1048] + SE_table[8];
assign add8[4]  = line_buffer[1063:1056] + SE_table[8];
assign add8[5]  = line_buffer[1071:1064] + SE_table[8];
assign add8[6]  = line_buffer[1079:1072] + SE_table[8];
assign add8[7]  = line_buffer[1087:1080] + SE_table[8];
assign add8[8]  = line_buffer[1095:1088] + SE_table[8];
assign add8[9]  = line_buffer[1103:1096] + SE_table[8];
assign add8[10] = line_buffer[1111:1104] + SE_table[8];
assign add8[11] = line_buffer[1119:1112] + SE_table[8];
assign add8[12] = line_buffer[1127:1120] + SE_table[8];
assign add8[13] = line_buffer[1135:1128] + SE_table[8];
assign add8[14] = line_buffer[1143:1136] + SE_table[8];
assign add8[15] = line_buffer[1151:1144] + SE_table[8];

assign sub9[0]  = (line_buffer[1039:1032] > SE_table[9])? line_buffer[1039:1032] - SE_table[9] : 8'b0;
assign sub9[1]  = (line_buffer[1047:1040] > SE_table[9])? line_buffer[1047:1040] - SE_table[9] : 8'b0;
assign sub9[2]  = (line_buffer[1055:1048] > SE_table[9])? line_buffer[1055:1048] - SE_table[9] : 8'b0;
assign sub9[3]  = (line_buffer[1063:1056] > SE_table[9])? line_buffer[1063:1056] - SE_table[9] : 8'b0;
assign sub9[4]  = (line_buffer[1071:1064] > SE_table[9])? line_buffer[1071:1064] - SE_table[9] : 8'b0;
assign sub9[5]  = (line_buffer[1079:1072] > SE_table[9])? line_buffer[1079:1072] - SE_table[9] : 8'b0;
assign sub9[6]  = (line_buffer[1087:1080] > SE_table[9])? line_buffer[1087:1080] - SE_table[9] : 8'b0;
assign sub9[7]  = (line_buffer[1095:1088] > SE_table[9])? line_buffer[1095:1088] - SE_table[9] : 8'b0;
assign sub9[8]  = (line_buffer[1103:1096] > SE_table[9])? line_buffer[1103:1096] - SE_table[9] : 8'b0;
assign sub9[9]  = (line_buffer[1111:1104] > SE_table[9])? line_buffer[1111:1104] - SE_table[9] : 8'b0;
assign sub9[10] = (line_buffer[1119:1112] > SE_table[9])? line_buffer[1119:1112] - SE_table[9] : 8'b0;
assign sub9[11] = (line_buffer[1127:1120] > SE_table[9])? line_buffer[1127:1120] - SE_table[9] : 8'b0;
assign sub9[12] = (line_buffer[1135:1128] > SE_table[9])? line_buffer[1135:1128] - SE_table[9] : 8'b0;
assign sub9[13] = (line_buffer[1143:1136] > SE_table[9])? line_buffer[1143:1136] - SE_table[9] : 8'b0;
assign sub9[14] = (line_buffer[1151:1144] > SE_table[9])? line_buffer[1151:1144] - SE_table[9] : 8'b0;
assign sub9[15] = (sram_counter[1:0] != 2'b11 && line_buffer[1159:1152] > SE_table[9])? line_buffer[1159:1152] - SE_table[9] : 8'b0;
assign add9[0]  = line_buffer[1039:1032] + SE_table[9];
assign add9[1]  = line_buffer[1047:1040] + SE_table[9];
assign add9[2]  = line_buffer[1055:1048] + SE_table[9];
assign add9[3]  = line_buffer[1063:1056] + SE_table[9];
assign add9[4]  = line_buffer[1071:1064] + SE_table[9];
assign add9[5]  = line_buffer[1079:1072] + SE_table[9];
assign add9[6]  = line_buffer[1087:1080] + SE_table[9];
assign add9[7]  = line_buffer[1095:1088] + SE_table[9];
assign add9[8]  = line_buffer[1103:1096] + SE_table[9];
assign add9[9]  = line_buffer[1111:1104] + SE_table[9];
assign add9[10] = line_buffer[1119:1112] + SE_table[9];
assign add9[11] = line_buffer[1127:1120] + SE_table[9];
assign add9[12] = line_buffer[1135:1128] + SE_table[9];
assign add9[13] = line_buffer[1143:1136] + SE_table[9];
assign add9[14] = line_buffer[1151:1144] + SE_table[9];
assign add9[15] = (sram_counter[1:0] == 2'b11)? SE_table[9] : line_buffer[1159:1152] + SE_table[9];

assign sub10[0]  = (line_buffer[1047:1040] > SE_table[10])? line_buffer[1047:1040] - SE_table[10] : 8'b0;
assign sub10[1]  = (line_buffer[1055:1048] > SE_table[10])? line_buffer[1055:1048] - SE_table[10] : 8'b0;
assign sub10[2]  = (line_buffer[1063:1056] > SE_table[10])? line_buffer[1063:1056] - SE_table[10] : 8'b0;
assign sub10[3]  = (line_buffer[1071:1064] > SE_table[10])? line_buffer[1071:1064] - SE_table[10] : 8'b0;
assign sub10[4]  = (line_buffer[1079:1072] > SE_table[10])? line_buffer[1079:1072] - SE_table[10] : 8'b0;
assign sub10[5]  = (line_buffer[1087:1080] > SE_table[10])? line_buffer[1087:1080] - SE_table[10] : 8'b0;
assign sub10[6]  = (line_buffer[1095:1088] > SE_table[10])? line_buffer[1095:1088] - SE_table[10] : 8'b0;
assign sub10[7]  = (line_buffer[1103:1096] > SE_table[10])? line_buffer[1103:1096] - SE_table[10] : 8'b0;
assign sub10[8]  = (line_buffer[1111:1104] > SE_table[10])? line_buffer[1111:1104] - SE_table[10] : 8'b0;
assign sub10[9]  = (line_buffer[1119:1112] > SE_table[10])? line_buffer[1119:1112] - SE_table[10] : 8'b0;
assign sub10[10] = (line_buffer[1127:1120] > SE_table[10])? line_buffer[1127:1120] - SE_table[10] : 8'b0;
assign sub10[11] = (line_buffer[1135:1128] > SE_table[10])? line_buffer[1135:1128] - SE_table[10] : 8'b0;
assign sub10[12] = (line_buffer[1143:1136] > SE_table[10])? line_buffer[1143:1136] - SE_table[10] : 8'b0;
assign sub10[13] = (line_buffer[1151:1144] > SE_table[10])? line_buffer[1151:1144] - SE_table[10] : 8'b0;
assign sub10[14] = (sram_counter[1:0] != 2'b11 && line_buffer[1159:1152] > SE_table[10])? line_buffer[1159:1152] - SE_table[10] : 8'b0;
assign sub10[15] = (sram_counter[1:0] != 2'b11 && line_buffer[1167:1160] > SE_table[10])? line_buffer[1167:1160] - SE_table[10] : 8'b0;
assign add10[0]  = line_buffer[1047:1040] + SE_table[10];
assign add10[1]  = line_buffer[1055:1048] + SE_table[10];
assign add10[2]  = line_buffer[1063:1056] + SE_table[10];
assign add10[3]  = line_buffer[1071:1064] + SE_table[10];
assign add10[4]  = line_buffer[1079:1072] + SE_table[10];
assign add10[5]  = line_buffer[1087:1080] + SE_table[10];
assign add10[6]  = line_buffer[1095:1088] + SE_table[10];
assign add10[7]  = line_buffer[1103:1096] + SE_table[10];
assign add10[8]  = line_buffer[1111:1104] + SE_table[10];
assign add10[9]  = line_buffer[1119:1112] + SE_table[10];
assign add10[10] = line_buffer[1127:1120] + SE_table[10];
assign add10[11] = line_buffer[1135:1128] + SE_table[10];
assign add10[12] = line_buffer[1143:1136] + SE_table[10];
assign add10[13] = line_buffer[1151:1144] + SE_table[10];
assign add10[14] = (sram_counter[1:0] == 2'b11)? SE_table[10] : line_buffer[1159:1152] + SE_table[10];
assign add10[15] = (sram_counter[1:0] == 2'b11)? SE_table[10] : line_buffer[1167:1160] + SE_table[10];

assign sub11[0]  = (line_buffer[1055:1048] > SE_table[11])? line_buffer[1055:1048] - SE_table[11] : 8'b0;
assign sub11[1]  = (line_buffer[1063:1056] > SE_table[11])? line_buffer[1063:1056] - SE_table[11] : 8'b0;
assign sub11[2]  = (line_buffer[1071:1064] > SE_table[11])? line_buffer[1071:1064] - SE_table[11] : 8'b0;
assign sub11[3]  = (line_buffer[1079:1072] > SE_table[11])? line_buffer[1079:1072] - SE_table[11] : 8'b0;
assign sub11[4]  = (line_buffer[1087:1080] > SE_table[11])? line_buffer[1087:1080] - SE_table[11] : 8'b0;
assign sub11[5]  = (line_buffer[1095:1088] > SE_table[11])? line_buffer[1095:1088] - SE_table[11] : 8'b0;
assign sub11[6]  = (line_buffer[1103:1096] > SE_table[11])? line_buffer[1103:1096] - SE_table[11] : 8'b0;
assign sub11[7]  = (line_buffer[1111:1104] > SE_table[11])? line_buffer[1111:1104] - SE_table[11] : 8'b0;
assign sub11[8]  = (line_buffer[1119:1112] > SE_table[11])? line_buffer[1119:1112] - SE_table[11] : 8'b0;
assign sub11[9]  = (line_buffer[1127:1120] > SE_table[11])? line_buffer[1127:1120] - SE_table[11] : 8'b0;
assign sub11[10] = (line_buffer[1135:1128] > SE_table[11])? line_buffer[1135:1128] - SE_table[11] : 8'b0;
assign sub11[11] = (line_buffer[1143:1136] > SE_table[11])? line_buffer[1143:1136] - SE_table[11] : 8'b0;
assign sub11[12] = (line_buffer[1151:1144] > SE_table[11])? line_buffer[1151:1144] - SE_table[11] : 8'b0;
assign sub11[13] = (sram_counter[1:0] != 2'b11 && line_buffer[1159:1152] > SE_table[11])? line_buffer[1159:1152] - SE_table[11] : 8'b0;
assign sub11[14] = (sram_counter[1:0] != 2'b11 && line_buffer[1167:1160] > SE_table[11])? line_buffer[1167:1160] - SE_table[11] : 8'b0;
assign sub11[15] = (sram_counter[1:0] != 2'b11 && line_buffer[1175:1168] > SE_table[11])? line_buffer[1175:1168] - SE_table[11] : 8'b0;
assign add11[0]  = line_buffer[1055:1048] + SE_table[11];
assign add11[1]  = line_buffer[1063:1056] + SE_table[11];
assign add11[2]  = line_buffer[1071:1064] + SE_table[11];
assign add11[3]  = line_buffer[1079:1072] + SE_table[11];
assign add11[4]  = line_buffer[1087:1080] + SE_table[11];
assign add11[5]  = line_buffer[1095:1088] + SE_table[11];
assign add11[6]  = line_buffer[1103:1096] + SE_table[11];
assign add11[7]  = line_buffer[1111:1104] + SE_table[11];
assign add11[8]  = line_buffer[1119:1112] + SE_table[11];
assign add11[9]  = line_buffer[1127:1120] + SE_table[11];
assign add11[10] = line_buffer[1135:1128] + SE_table[11];
assign add11[11] = line_buffer[1143:1136] + SE_table[11];
assign add11[12] = line_buffer[1151:1144] + SE_table[11];
assign add11[13] = (sram_counter[1:0] == 2'b11)? SE_table[11] : line_buffer[1159:1152] + SE_table[11];
assign add11[14] = (sram_counter[1:0] == 2'b11)? SE_table[11] : line_buffer[1167:1160] + SE_table[11];
assign add11[15] = (sram_counter[1:0] == 2'b11)? SE_table[11] : line_buffer[1175:1168] + SE_table[11];




//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// LEVEL 444444444 ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////



assign sub12[0]  = (line_buffer[1543:1536] > SE_table[12])? line_buffer[1543:1536] - SE_table[12] : 8'b0;
assign sub12[1]  = (line_buffer[1551:1544] > SE_table[12])? line_buffer[1551:1544] - SE_table[12] : 8'b0;
assign sub12[2]  = (line_buffer[1559:1552] > SE_table[12])? line_buffer[1559:1552] - SE_table[12] : 8'b0;
assign sub12[3]  = (line_buffer[1567:1560] > SE_table[12])? line_buffer[1567:1560] - SE_table[12] : 8'b0;
assign sub12[4]  = (line_buffer[1575:1568] > SE_table[12])? line_buffer[1575:1568] - SE_table[12] : 8'b0;
assign sub12[5]  = (line_buffer[1583:1576] > SE_table[12])? line_buffer[1583:1576] - SE_table[12] : 8'b0;
assign sub12[6]  = (line_buffer[1591:1584] > SE_table[12])? line_buffer[1591:1584] - SE_table[12] : 8'b0;
assign sub12[7]  = (line_buffer[1599:1592] > SE_table[12])? line_buffer[1599:1592] - SE_table[12] : 8'b0;
assign sub12[8]  = (line_buffer[1607:1600] > SE_table[12])? line_buffer[1607:1600] - SE_table[12] : 8'b0;
assign sub12[9]  = (line_buffer[1615:1608] > SE_table[12])? line_buffer[1615:1608] - SE_table[12] : 8'b0;
assign sub12[10] = (line_buffer[1623:1616] > SE_table[12])? line_buffer[1623:1616] - SE_table[12] : 8'b0;
assign sub12[11] = (line_buffer[1631:1624] > SE_table[12])? line_buffer[1631:1624] - SE_table[12] : 8'b0;
assign sub12[12] = (line_buffer[1639:1632] > SE_table[12])? line_buffer[1639:1632] - SE_table[12] : 8'b0;
assign sub12[13] = (line_buffer[1647:1640] > SE_table[12])? line_buffer[1647:1640] - SE_table[12] : 8'b0;
assign sub12[14] = (line_buffer[1655:1648] > SE_table[12])? line_buffer[1655:1648] - SE_table[12] : 8'b0;
assign sub12[15] = (line_buffer[1663:1656] > SE_table[12])? line_buffer[1663:1656] - SE_table[12] : 8'b0;
assign add12[0]  = line_buffer[1543:1536] + SE_table[12];
assign add12[1]  = line_buffer[1551:1544] + SE_table[12];
assign add12[2]  = line_buffer[1559:1552] + SE_table[12];
assign add12[3]  = line_buffer[1567:1560] + SE_table[12];
assign add12[4]  = line_buffer[1575:1568] + SE_table[12];
assign add12[5]  = line_buffer[1583:1576] + SE_table[12];
assign add12[6]  = line_buffer[1591:1584] + SE_table[12];
assign add12[7]  = line_buffer[1599:1592] + SE_table[12];
assign add12[8]  = line_buffer[1607:1600] + SE_table[12];
assign add12[9]  = line_buffer[1615:1608] + SE_table[12];
assign add12[10] = line_buffer[1623:1616] + SE_table[12];
assign add12[11] = line_buffer[1631:1624] + SE_table[12];
assign add12[12] = line_buffer[1639:1632] + SE_table[12];
assign add12[13] = line_buffer[1647:1640] + SE_table[12];
assign add12[14] = line_buffer[1655:1648] + SE_table[12];
assign add12[15] = line_buffer[1663:1656] + SE_table[12];

assign sub13[0]  = (line_buffer[1551:1544] > SE_table[13])? line_buffer[1551:1544] - SE_table[13] : 8'b0;
assign sub13[1]  = (line_buffer[1559:1552] > SE_table[13])? line_buffer[1559:1552] - SE_table[13] : 8'b0;
assign sub13[2]  = (line_buffer[1567:1560] > SE_table[13])? line_buffer[1567:1560] - SE_table[13] : 8'b0;
assign sub13[3]  = (line_buffer[1575:1568] > SE_table[13])? line_buffer[1575:1568] - SE_table[13] : 8'b0;
assign sub13[4]  = (line_buffer[1583:1576] > SE_table[13])? line_buffer[1583:1576] - SE_table[13] : 8'b0;
assign sub13[5]  = (line_buffer[1591:1584] > SE_table[13])? line_buffer[1591:1584] - SE_table[13] : 8'b0;
assign sub13[6]  = (line_buffer[1599:1592] > SE_table[13])? line_buffer[1599:1592] - SE_table[13] : 8'b0;
assign sub13[7]  = (line_buffer[1607:1600] > SE_table[13])? line_buffer[1607:1600] - SE_table[13] : 8'b0;
assign sub13[8]  = (line_buffer[1615:1608] > SE_table[13])? line_buffer[1615:1608] - SE_table[13] : 8'b0;
assign sub13[9]  = (line_buffer[1623:1616] > SE_table[13])? line_buffer[1623:1616] - SE_table[13] : 8'b0;
assign sub13[10] = (line_buffer[1631:1624] > SE_table[13])? line_buffer[1631:1624] - SE_table[13] : 8'b0;
assign sub13[11] = (line_buffer[1639:1632] > SE_table[13])? line_buffer[1639:1632] - SE_table[13] : 8'b0;
assign sub13[12] = (line_buffer[1647:1640] > SE_table[13])? line_buffer[1647:1640] - SE_table[13] : 8'b0;
assign sub13[13] = (line_buffer[1655:1648] > SE_table[13])? line_buffer[1655:1648] - SE_table[13] : 8'b0;
assign sub13[14] = (line_buffer[1663:1656] > SE_table[13])? line_buffer[1663:1656] - SE_table[13] : 8'b0;
assign sub13[15] = (sram_counter[1:0] != 2'b11 && line_buffer[1671:1664] > SE_table[13])? line_buffer[1671:1664] - SE_table[13] : 8'b0;
assign add13[0]  = line_buffer[1551:1544] + SE_table[13];
assign add13[1]  = line_buffer[1559:1552] + SE_table[13];
assign add13[2]  = line_buffer[1567:1560] + SE_table[13];
assign add13[3]  = line_buffer[1575:1568] + SE_table[13];
assign add13[4]  = line_buffer[1583:1576] + SE_table[13];
assign add13[5]  = line_buffer[1591:1584] + SE_table[13];
assign add13[6]  = line_buffer[1599:1592] + SE_table[13];
assign add13[7]  = line_buffer[1607:1600] + SE_table[13];
assign add13[8]  = line_buffer[1615:1608] + SE_table[13];
assign add13[9]  = line_buffer[1623:1616] + SE_table[13];
assign add13[10] = line_buffer[1631:1624] + SE_table[13];
assign add13[11] = line_buffer[1639:1632] + SE_table[13];
assign add13[12] = line_buffer[1647:1640] + SE_table[13];
assign add13[13] = line_buffer[1655:1648] + SE_table[13];
assign add13[14] = line_buffer[1663:1656] + SE_table[13];
assign add13[15] = (sram_counter[1:0] == 2'b11)? SE_table[13] : line_buffer[1671:1664] + SE_table[13];

assign sub14[0]  = (line_buffer[1559:1552] > SE_table[14])? line_buffer[1559:1552] - SE_table[14] : 8'b0;
assign sub14[1]  = (line_buffer[1567:1560] > SE_table[14])? line_buffer[1567:1560] - SE_table[14] : 8'b0;
assign sub14[2]  = (line_buffer[1575:1568] > SE_table[14])? line_buffer[1575:1568] - SE_table[14] : 8'b0;
assign sub14[3]  = (line_buffer[1583:1576] > SE_table[14])? line_buffer[1583:1576] - SE_table[14] : 8'b0;
assign sub14[4]  = (line_buffer[1591:1584] > SE_table[14])? line_buffer[1591:1584] - SE_table[14] : 8'b0;
assign sub14[5]  = (line_buffer[1599:1592] > SE_table[14])? line_buffer[1599:1592] - SE_table[14] : 8'b0;
assign sub14[6]  = (line_buffer[1607:1600] > SE_table[14])? line_buffer[1607:1600] - SE_table[14] : 8'b0;
assign sub14[7]  = (line_buffer[1615:1608] > SE_table[14])? line_buffer[1615:1608] - SE_table[14] : 8'b0;
assign sub14[8]  = (line_buffer[1623:1616] > SE_table[14])? line_buffer[1623:1616] - SE_table[14] : 8'b0;
assign sub14[9]  = (line_buffer[1631:1624] > SE_table[14])? line_buffer[1631:1624] - SE_table[14] : 8'b0;
assign sub14[10] = (line_buffer[1639:1632] > SE_table[14])? line_buffer[1639:1632] - SE_table[14] : 8'b0;
assign sub14[11] = (line_buffer[1647:1640] > SE_table[14])? line_buffer[1647:1640] - SE_table[14] : 8'b0;
assign sub14[12] = (line_buffer[1655:1648] > SE_table[14])? line_buffer[1655:1648] - SE_table[14] : 8'b0;
assign sub14[13] = (line_buffer[1663:1656] > SE_table[14])? line_buffer[1663:1656] - SE_table[14] : 8'b0;
assign sub14[14] = (sram_counter[1:0] != 2'b11 && line_buffer[1671:1664] > SE_table[14])? line_buffer[1671:1664] - SE_table[14] : 8'b0;
assign sub14[15] = (sram_counter[1:0] != 2'b11 && line_buffer[1679:1672] > SE_table[14])? line_buffer[1679:1672] - SE_table[14] : 8'b0;
assign add14[0]  = line_buffer[1559:1552] + SE_table[14];
assign add14[1]  = line_buffer[1567:1560] + SE_table[14];
assign add14[2]  = line_buffer[1575:1568] + SE_table[14];
assign add14[3]  = line_buffer[1583:1576] + SE_table[14];
assign add14[4]  = line_buffer[1591:1584] + SE_table[14];
assign add14[5]  = line_buffer[1599:1592] + SE_table[14];
assign add14[6]  = line_buffer[1607:1600] + SE_table[14];
assign add14[7]  = line_buffer[1615:1608] + SE_table[14];
assign add14[8]  = line_buffer[1623:1616] + SE_table[14];
assign add14[9]  = line_buffer[1631:1624] + SE_table[14];
assign add14[10] = line_buffer[1639:1632] + SE_table[14];
assign add14[11] = line_buffer[1647:1640] + SE_table[14];
assign add14[12] = line_buffer[1655:1648] + SE_table[14];
assign add14[13] = line_buffer[1663:1656] + SE_table[14];
assign add14[14] = (sram_counter[1:0] == 2'b11)? SE_table[14] : line_buffer[1671:1664] + SE_table[14];
assign add14[15] = (sram_counter[1:0] == 2'b11)? SE_table[14] : line_buffer[1679:1672] + SE_table[14];

assign sub15[0]  = (line_buffer[1567:1560] > SE_table[15])? line_buffer[1567:1560] - SE_table[15] : 8'b0;
assign sub15[1]  = (line_buffer[1575:1568] > SE_table[15])? line_buffer[1575:1568] - SE_table[15] : 8'b0;
assign sub15[2]  = (line_buffer[1583:1576] > SE_table[15])? line_buffer[1583:1576] - SE_table[15] : 8'b0;
assign sub15[3]  = (line_buffer[1591:1584] > SE_table[15])? line_buffer[1591:1584] - SE_table[15] : 8'b0;
assign sub15[4]  = (line_buffer[1599:1592] > SE_table[15])? line_buffer[1599:1592] - SE_table[15] : 8'b0;
assign sub15[5]  = (line_buffer[1607:1600] > SE_table[15])? line_buffer[1607:1600] - SE_table[15] : 8'b0;
assign sub15[6]  = (line_buffer[1615:1608] > SE_table[15])? line_buffer[1615:1608] - SE_table[15] : 8'b0;
assign sub15[7]  = (line_buffer[1623:1616] > SE_table[15])? line_buffer[1623:1616] - SE_table[15] : 8'b0;
assign sub15[8]  = (line_buffer[1631:1624] > SE_table[15])? line_buffer[1631:1624] - SE_table[15] : 8'b0;
assign sub15[9]  = (line_buffer[1639:1632] > SE_table[15])? line_buffer[1639:1632] - SE_table[15] : 8'b0;
assign sub15[10] = (line_buffer[1647:1640] > SE_table[15])? line_buffer[1647:1640] - SE_table[15] : 8'b0;
assign sub15[11] = (line_buffer[1655:1648] > SE_table[15])? line_buffer[1655:1648] - SE_table[15] : 8'b0;
assign sub15[12] = (line_buffer[1663:1656] > SE_table[15])? line_buffer[1663:1656] - SE_table[15] : 8'b0;
assign sub15[13] = (sram_counter[1:0] != 2'b11 && line_buffer[1671:1664] > SE_table[15])? line_buffer[1671:1664] - SE_table[15] : 8'b0;
assign sub15[14] = (sram_counter[1:0] != 2'b11 && line_buffer[1679:1672] > SE_table[15])? line_buffer[1679:1672] - SE_table[15] : 8'b0;
assign sub15[15] = (sram_counter[1:0] != 2'b11 && line_buffer[1687:1680] > SE_table[15])? line_buffer[1687:1680] - SE_table[15] : 8'b0;
assign add15[0]  = line_buffer[1567:1560] + SE_table[15];
assign add15[1]  = line_buffer[1575:1568] + SE_table[15];
assign add15[2]  = line_buffer[1583:1576] + SE_table[15];
assign add15[3]  = line_buffer[1591:1584] + SE_table[15];
assign add15[4]  = line_buffer[1599:1592] + SE_table[15];
assign add15[5]  = line_buffer[1607:1600] + SE_table[15];
assign add15[6]  = line_buffer[1615:1608] + SE_table[15];
assign add15[7]  = line_buffer[1623:1616] + SE_table[15];
assign add15[8]  = line_buffer[1631:1624] + SE_table[15];
assign add15[9]  = line_buffer[1639:1632] + SE_table[15];
assign add15[10] = line_buffer[1647:1640] + SE_table[15];
assign add15[11] = line_buffer[1655:1648] + SE_table[15];
assign add15[12] = line_buffer[1663:1656] + SE_table[15];
assign add15[13] = (sram_counter[1:0] == 2'b11)? SE_table[15] : line_buffer[1671:1664] + SE_table[15];
assign add15[14] = (sram_counter[1:0] == 2'b11)? SE_table[15] : line_buffer[1679:1672] + SE_table[15];
assign add15[15] = (sram_counter[1:0] == 2'b11)? SE_table[15] : line_buffer[1687:1680] + SE_table[15];

endmodule