module NN(
	// Input signals
	clk,
	rst_n,
	in_valid_u,
	in_valid_w,
	in_valid_v,
	in_valid_x,
	weight_u,
	weight_w,
	weight_v,
	data_x,
	// Output signals
	out_valid,
	out
);

//---------------------------------------------------------------------
//   PARAMETER
//---------------------------------------------------------------------

// IEEE floating point paramenters
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
parameter inst_arch = 0;

parameter IDLE = 2'd0;
parameter INPUT = 2'd1;
parameter CALCULATION = 2'd2;
parameter OUTPUT = 2'd3;

//---------------------------------------------------------------------
//   INPUT AND OUTPUT DECLARATION
//---------------------------------------------------------------------
input  clk, rst_n, in_valid_u, in_valid_w, in_valid_v, in_valid_x;
input [inst_sig_width+inst_exp_width:0] weight_u, weight_w, weight_v;
input [inst_sig_width+inst_exp_width:0] data_x;
output reg	out_valid;
output reg [inst_sig_width+inst_exp_width:0] out;

//---------------------------------------------------------------------
//   WIRE AND REG DECLARATION
//---------------------------------------------------------------------

reg[1:0] c_state, n_state;
reg[31:0] weight_u_array[8:0], weight_w_array[8:0], weight_v_array[8:0], data_x_array[8:0];
reg[3:0] in_counter, out_counter;
reg[4:0] cal_counter;
reg[31:0] dp_array[22:0], add_array[22:0], exp_array[22:0], recip_array[22:0];
reg[31:0] dp_array_0, dp_array_1, dp_array_2, dp_array_3, dp_array_4, 
			dp_array_5, add_array_0, add_array_1, exp_array_0, recip_array_0;
wire[31:0] dp_ans, add_ans, exp_in, exp_ans, recip_ans, final_ans;
reg[31:0] y_1;

///////////////////// FSM ///////////////////////////

always@(posedge clk or negedge rst_n) begin 
	if(!rst_n) c_state <= IDLE;
	else c_state <= n_state;
end

always@(*) begin 
	case(c_state)
		IDLE: begin
			if(in_valid_x == 1'b1) n_state = INPUT;
			else n_state = c_state;
		end
		INPUT: begin
			if(in_valid_x == 1'b0) n_state = CALCULATION;
			else n_state = c_state;
		end
		CALCULATION: begin 
			if(cal_counter == 5'd17) n_state = OUTPUT;
			else n_state = c_state;
		end
		OUTPUT: begin
			if(out_counter == 4'd9) n_state = IDLE;
			else n_state = c_state;
		end
	endcase
end

////////////////////// input //////////////////////////

// in_counter
always@(posedge clk or negedge rst_n) begin 
	if(!rst_n) in_counter <= 4'd0;
	else begin 
		case(n_state)
			INPUT: in_counter <= in_counter + 4'd1;
			default: in_counter <= 4'd0;
		endcase
	end
end

genvar i;
generate
	for(i = 0; i < 9; i = i + 1) begin: input_loop
		// weight_u_array
		always@(posedge clk or negedge rst_n) begin 
			if(!rst_n) weight_u_array[i] <= 32'b0;
			else begin 
				case(n_state)
					INPUT: if(in_counter == i) weight_u_array[i] <= weight_u;
				endcase
			end
		end
		// weight_w_array
		always@(posedge clk or negedge rst_n) begin 
			if(!rst_n) weight_w_array[i] <= 32'b0;
			else begin 
				case(n_state)
					INPUT: if(in_counter == i) weight_w_array[i] <= weight_w;
				endcase
			end
		end
		// weight_v_array
		always@(posedge clk or negedge rst_n) begin 
			if(!rst_n) weight_v_array[i] <= 32'b0;
			else begin 
				case(n_state)
					INPUT: if(in_counter == i) weight_v_array[i] <= weight_v;
				endcase
			end
		end
		// data_x_array
		always@(posedge clk or negedge rst_n) begin 
			if(!rst_n) data_x_array[i] <= 32'b0;
			else begin 
				case(n_state)
					INPUT: if(in_counter == i) data_x_array[i] <= data_x;
				endcase
			end
		end
	end
endgenerate

// cal_counter
always@(posedge clk or negedge rst_n) begin 
	if(!rst_n) cal_counter <= 4'd0;
	else begin 
		case(n_state)
			IDLE: cal_counter <= 4'd0;
			INPUT: if(in_counter > 4'd5) cal_counter <= cal_counter + 4'd1;
			CALCULATION: cal_counter <= cal_counter + 4'd1;
			OUTPUT: cal_counter <= cal_counter + 4'd1;
		endcase
	end
end

////////////////////// calculation ////////////////////////

DW_fp_dp3 DP0(.a(dp_array_0), .b(dp_array_1), .c(dp_array_2), .d(dp_array_3), .e(dp_array_4), .f(dp_array_5), .z(dp_ans), .rnd(3'b000));
assign final_ans = (dp_ans[31])? 32'b0 : dp_ans;

DW_fp_add ADD0(.a(add_array_0), .b(add_array_1), .z(add_ans), .rnd(3'b000));

assign exp_in = {~exp_array_0[31], exp_array_0[30:0]};
DW_fp_exp #(inst_sig_width, inst_exp_width, inst_ieee_compliance, inst_arch) EXP0(.a(exp_in), .z(exp_ans));

DW_fp_recip RECIP0(.a(recip_array_0), .z(recip_ans), .rnd(3'b000));

// y_1
always@(posedge clk or negedge rst_n) begin 
	if(!rst_n) y_1 <= 32'b0;
	else begin 
		case(cal_counter)
			14: y_1 <= final_ans;
		endcase
	end
end

// dp_array_0
always@(*) begin 
	case(cal_counter)
		1: dp_array_0 = weight_u_array[0];
		2: dp_array_0 = weight_u_array[3];
		3: dp_array_0 = weight_u_array[6];
		4: dp_array_0 = weight_u_array[0];
		5: dp_array_0 = weight_u_array[3];
		6: dp_array_0 = weight_u_array[6];
		7: dp_array_0 = weight_w_array[0];
		8: dp_array_0 = weight_w_array[3];
		9: dp_array_0 = weight_w_array[6];
		10: dp_array_0 = weight_u_array[0];
		11: dp_array_0 = weight_u_array[3];
		12: dp_array_0 = weight_u_array[6];
		15: dp_array_0 = weight_w_array[0];
		16: dp_array_0 = weight_w_array[3];
		17: dp_array_0 = weight_w_array[6];
		14: dp_array_0 = weight_v_array[0];
		18: dp_array_0 = weight_v_array[3];
		19: dp_array_0 = weight_v_array[6];
		20: dp_array_0 = weight_v_array[0];
		21: dp_array_0 = weight_v_array[3];
		22: dp_array_0 = weight_v_array[6];
		23: dp_array_0 = weight_v_array[0];
		24: dp_array_0 = weight_v_array[3];
		25: dp_array_0 = weight_v_array[6];
		default: dp_array_0 = 32'b0;
	endcase
end

// dp_array_1
always@(*) begin 
	case(cal_counter)
		1: dp_array_1 = data_x_array[0];
		2: dp_array_1 = data_x_array[0];
		3: dp_array_1 = data_x_array[0];
		4: dp_array_1 = data_x_array[3];
		5: dp_array_1 = data_x_array[3];
		6: dp_array_1 = data_x_array[3];
		7: dp_array_1 = recip_array[4];
		8: dp_array_1 = recip_array[4];
		9: dp_array_1 = recip_array[4];
		10: dp_array_1 = data_x_array[6];
		11: dp_array_1 = data_x_array[6];
		12: dp_array_1 = data_x_array[6];
		15: dp_array_1 = recip_array[12];
		16: dp_array_1 = recip_array[12];
		17: dp_array_1 = recip_array[12];
		14: dp_array_1 = recip_array[4];
		18: dp_array_1 = recip_array[4];
		19: dp_array_1 = recip_array[4];
		20: dp_array_1 = recip_array[12];
		21: dp_array_1 = recip_array[12];
		22: dp_array_1 = recip_array[12];
		23: dp_array_1 = recip_array[20];
		24: dp_array_1 = recip_array[20];
		25: dp_array_1 = recip_array[20];
		default: dp_array_1 = 32'b0;
	endcase
end

// dp_array_2
always@(*) begin 
	case(cal_counter)
		1: dp_array_2 = weight_u_array[1];
		2: dp_array_2 = weight_u_array[4];
		3: dp_array_2 = weight_u_array[7];
		4: dp_array_2 = weight_u_array[1];
		5: dp_array_2 = weight_u_array[4];
		6: dp_array_2 = weight_u_array[7];
		7: dp_array_2 = weight_w_array[1];
		8: dp_array_2 = weight_w_array[4];
		9: dp_array_2 = weight_w_array[7];
		10: dp_array_2 = weight_u_array[1];
		11: dp_array_2 = weight_u_array[4];
		12: dp_array_2 = weight_u_array[7];
		15: dp_array_2 = weight_w_array[1];
		16: dp_array_2 = weight_w_array[4];
		17: dp_array_2 = weight_w_array[7];
		14: dp_array_2 = weight_v_array[1];
		18: dp_array_2 = weight_v_array[4];
		19: dp_array_2 = weight_v_array[7];
		20: dp_array_2 = weight_v_array[1];
		21: dp_array_2 = weight_v_array[4];
		22: dp_array_2 = weight_v_array[7];
		23: dp_array_2 = weight_v_array[1];
		24: dp_array_2 = weight_v_array[4];
		25: dp_array_2 = weight_v_array[7];
		default: dp_array_2 = 32'b0;
	endcase
end

// dp_array_3
always@(*) begin 
	case(cal_counter)
		1: dp_array_3 = data_x_array[1];
		2: dp_array_3 = data_x_array[1];
		3: dp_array_3 = data_x_array[1];
		4: dp_array_3 = data_x_array[4];
		5: dp_array_3 = data_x_array[4];
		6: dp_array_3 = data_x_array[4];
		7: dp_array_3 = recip_array[5];
		8: dp_array_3 = recip_array[5];
		9: dp_array_3 = recip_array[5];
		10: dp_array_3 = data_x_array[7];
		11: dp_array_3 = data_x_array[7];
		12: dp_array_3 = data_x_array[7];
		15: dp_array_3 = recip_array[13];
		16: dp_array_3 = recip_array[13];
		17: dp_array_3 = recip_array[13];
		14: dp_array_3 = recip_array[5];
		18: dp_array_3 = recip_array[5];
		19: dp_array_3 = recip_array[5];
		20: dp_array_3 = recip_array[13];
		21: dp_array_3 = recip_array[13];
		22: dp_array_3 = recip_array[13];
		23: dp_array_3 = recip_array[21];
		24: dp_array_3 = recip_array[21];
		25: dp_array_3 = recip_array[21];
		default: dp_array_3 = 32'b0;
	endcase
end

// dp_array_4
always@(*) begin 
	case(cal_counter)
		1: dp_array_4 = weight_u_array[2];
		2: dp_array_4 = weight_u_array[5];
		3: dp_array_4 = weight_u_array[8];
		4: dp_array_4 = weight_u_array[2];
		5: dp_array_4 = weight_u_array[5];
		6: dp_array_4 = weight_u_array[8];
		7: dp_array_4 = weight_w_array[2];
		8: dp_array_4 = weight_w_array[5];
		9: dp_array_4 = weight_w_array[8];
		10: dp_array_4 = weight_u_array[2];
		11: dp_array_4 = weight_u_array[5];
		12: dp_array_4 = weight_u_array[8];
		15: dp_array_4 = weight_w_array[2];
		16: dp_array_4 = weight_w_array[5];
		17: dp_array_4 = weight_w_array[8];
		14: dp_array_4 = weight_v_array[2];
		18: dp_array_4 = weight_v_array[5];
		19: dp_array_4 = weight_v_array[8];
		20: dp_array_4 = weight_v_array[2];
		21: dp_array_4 = weight_v_array[5];
		22: dp_array_4 = weight_v_array[8];
		23: dp_array_4 = weight_v_array[2];
		24: dp_array_4 = weight_v_array[5];
		25: dp_array_4 = weight_v_array[8];
		default: dp_array_4 = 32'b0;
	endcase
end

// dp_array_5
always@(*) begin 
	case(cal_counter)
		1: dp_array_5 = data_x_array[2];
		2: dp_array_5 = data_x_array[2];
		3: dp_array_5 = data_x_array[2];
		4: dp_array_5 = data_x_array[5];
		5: dp_array_5 = data_x_array[5];
		6: dp_array_5 = data_x_array[5];
		7: dp_array_5 = recip_array[6];
		8: dp_array_5 = recip_array[6];
		9: dp_array_5 = recip_array[6];
		10: dp_array_5 = data_x_array[8];
		11: dp_array_5 = data_x_array[8];
		12: dp_array_5 = data_x_array[8];
		15: dp_array_5 = recip_array[14];
		16: dp_array_5 = recip_array[14];
		17: dp_array_5 = recip_array[14];
		14: dp_array_5 = recip_array[6];
		18: dp_array_5 = recip_array[6];
		19: dp_array_5 = recip_array[6];
		20: dp_array_5 = recip_array[14];
		21: dp_array_5 = recip_array[14];
		22: dp_array_5 = recip_array[14];
		23: dp_array_5 = recip_array[22];
		24: dp_array_5 = recip_array[22];
		25: dp_array_5 = recip_array[22];
		default: dp_array_5 = 32'b0;
	endcase
end

// add_array_0
always@(*) begin 
	case(cal_counter)
		3: add_array_0 = exp_array[2];
		4: add_array_0 = exp_array[3];
		5: add_array_0 = exp_array[4];
		11: add_array_0 = exp_array[9];
		12: add_array_0 = exp_array[10];
		13: add_array_0 = exp_array[11];
		19: add_array_0 = exp_array[17];
		20: add_array_0 = exp_array[18];
		21: add_array_0 = exp_array[19];
		8: add_array_0 = dp_array[4];
		9: add_array_0 = dp_array[5];
		10: add_array_0 = dp_array[6];
		16: add_array_0 = dp_array[10];
		17: add_array_0 = dp_array[11];
		18: add_array_0 = dp_array[12];
		default: add_array_0 = 32'b0;
	endcase
end

// add_array_1
always@(*) begin 
	case(cal_counter)
		8: add_array_1 = dp_array[7];
		9: add_array_1 = dp_array[8];
		10: add_array_1 = dp_array[9];
		16: add_array_1 = dp_array[15];
		17: add_array_1 = dp_array[16];
		18: add_array_1 = dp_array[17];
		default: add_array_1 = 32'h3f800000;
	endcase
end

// exp_array_0
always@(*) begin 
	case(cal_counter)
		2: exp_array_0 = dp_array[1];
		3: exp_array_0 = dp_array[2];
		4: exp_array_0 = dp_array[3];
		9: exp_array_0 = add_array[8];
		10: exp_array_0 = add_array[9];
		11: exp_array_0 = add_array[10];
		17: exp_array_0 = add_array[16];
		18: exp_array_0 = add_array[17];
		19: exp_array_0 = add_array[18];
		default: exp_array_0 = 32'b0;
	endcase
end

// recip_array_0
always@(*) begin 
	case(cal_counter)
		4: recip_array_0 = add_array[3];
		5: recip_array_0 = add_array[4];
		6: recip_array_0 = add_array[5];
		12: recip_array_0 = add_array[11];
		13: recip_array_0 = add_array[12];
		14: recip_array_0 = add_array[13];
		20: recip_array_0 = add_array[19];
		21: recip_array_0 = add_array[20];
		22: recip_array_0 = add_array[21];
		default: recip_array_0 = 32'b0;
	endcase
end

genvar j;
generate
	for(j = 0; j < 23; j = j + 1) begin: calculation_loop
		// dp_array
		always@(posedge clk or negedge rst_n) begin 
			if(!rst_n) dp_array[j] <= 32'b0;
			else if(cal_counter == j) dp_array[j] <= dp_ans;
		end
		// add_array
		always@(posedge clk or negedge rst_n) begin 
			if(!rst_n) add_array[j] <= 32'b0;
			else if(cal_counter == j) add_array[j] <= add_ans;
		end
		// exp_array
		always@(posedge clk or negedge rst_n) begin 
			if(!rst_n) exp_array[j] <= 32'b0;
			else if(cal_counter == j) exp_array[j] <= exp_ans;
		end
		// recip_array
		always@(posedge clk or negedge rst_n) begin 
			if(!rst_n) recip_array[j] <= 32'b0;
			else if(cal_counter == j) recip_array[j] <= recip_ans;
		end
	end
endgenerate

////////////////////// output //////////////////////////

// out_counter
always@(posedge clk or negedge rst_n) begin 
	if(!rst_n) out_counter <= 4'd0;
	else begin 
		case(n_state)
			OUTPUT: out_counter <= out_counter + 4'd1;
			default: out_counter <= 4'd0;
		endcase
	end
end

// out_valid
always@(posedge clk or negedge rst_n) begin 
	if(!rst_n) out_valid <= 1'b0;
	else begin 
		case(n_state)
			OUTPUT: out_valid <= 1'b1;
			default: out_valid <= 1'b0;
		endcase
	end
end

// out
always@(posedge clk or negedge rst_n) begin 
	if(!rst_n) out <= 32'd0;
	else begin 
		case(n_state)
			OUTPUT: begin
				if(out_counter == 4'd0) out <= y_1;
				else out <= final_ans;
			end
			default: out <= 32'd0;
		endcase
	end
end

endmodule
