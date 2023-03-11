`include "synchronizer.v"
`include "syn_XOR.v"
module CDC(
	//Input Port
	clk1,
    clk2,
    clk3,
	rst_n,
	in_valid1,
	in_valid2,
	user1,
	user2,

    //Output Port
    out_valid1,
    out_valid2,
	equal,
	exceed,
	winner
); 
//---------------------------------------------------------------------
//   INPUT AND OUTPUT DECLARATION
//---------------------------------------------------------------------
input 		clk1, clk2, clk3, rst_n;
input 		in_valid1, in_valid2;
input [3:0]	user1, user2;

output reg	out_valid1, out_valid2;
output reg	equal, exceed, winner;
//---------------------------------------------------------------------
//   WIRE AND REG DECLARATION
//---------------------------------------------------------------------
//----clk1----
wire any_valid;
reg[3:0] new_card;

//----clk2----

//----clk3----
reg[2:0] c_state, n_state;

wire input_valid;
wire yes_winner;
reg[3:0] input_count;
reg[2:0] output_count;

reg[5:0] user1_point;
reg[5:0] user2_point;

reg[2:0] table_10; //4
reg[3:0] table_9; //8
reg[3:0] table_8; //12
reg[4:0] table_7; //16
reg[4:0] table_6; //20
reg[4:0] table_5; //24
reg[4:0] table_4; //28
reg[5:0] table_3; //32
reg[5:0] table_2; //36
reg[5:0] table_1; //52

reg[5:0] cards_to_equal;
reg[5:0] cards_to_exceed;

reg[12:0] dividend_equal;
reg[12:0] dividend_exceed;

reg[11:0] divisor;

//---------------------------------------------------------------------
//   PARAMETER
//---------------------------------------------------------------------
//----clk1----

//----clk2----

//----clk3----
parameter IDLE = 3'd0;
parameter INPUT = 3'd1;
parameter OUTPUT1 = 3'd3;
parameter OUTPUT2 = 3'd2;
parameter BUBBLE2 = 3'd6;
parameter BUBBLE = 3'd7;

//---------------------------------------------------------------------
//   DESIGN
//---------------------------------------------------------------------
//============================================
//   clk1 domain
//============================================
/*
// any_valid
always@(posedge clk1 or negedge rst_n) begin
	if(!rst_n) begin
		any_valid <= 1'b0;
	end 
	else begin
		if(in_valid1 || in_valid2) any_valid <= 1'b1;
		else any_valid <= 1'b0;
	end
end*/
assign any_valid = (in_valid1 || in_valid2);

// new_card
always@(posedge clk1 or negedge rst_n) begin 
	if(!rst_n) begin 
		new_card <= 4'b0;
	end
	else begin 
		if(in_valid1) new_card <= user1;
		else if(in_valid2) new_card <= user2;
		else new_card <= 4'b0;
	end
end

//============================================
//   clk2 domain
//============================================
/*always@(posedge clk2 or negedge rst_n) begin
	if(!rst_n) begin
		
	end else begin
		
	end
end*/
//============================================
//   clk3 domain
//============================================

// c_state
always@(posedge clk3 or negedge rst_n) begin
	if(!rst_n) begin
		c_state <= IDLE;
	end else begin
		c_state <= n_state;
	end
end

// n_state
always@(*) begin 
	case(c_state)
		IDLE: begin
			if(input_valid) n_state = INPUT;
			else n_state = c_state;
		end
		INPUT: begin 
			if(input_count == 3 || input_count == 4 || input_count == 8 || input_count == 9) n_state = BUBBLE2;
			else if(input_count == 10) n_state = OUTPUT2;
			else n_state = IDLE;
		end
		OUTPUT1: begin
			if(output_count == 7) n_state = IDLE;
			else n_state = c_state;
		end
		OUTPUT2: begin 
			if(yes_winner && output_count == 1) n_state = c_state;
			else n_state = IDLE;
		end
		BUBBLE2: n_state = BUBBLE;
		BUBBLE: n_state = OUTPUT1;
		default: n_state = c_state;
	endcase
end

// input_count
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) input_count <= 4'b0;
	else begin 
		case(n_state)
			INPUT: input_count <= input_count + 4'b1;
			OUTPUT2: input_count <= 4'b0;
		endcase
	end
end

// output_count
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) output_count <= 3'b0;
	else begin 
		case(n_state)
			OUTPUT1: output_count <= output_count + 3'b1;
			OUTPUT2: output_count <= output_count + 3'b1;
			default: output_count <= 3'b0;
		endcase
	end
end

// user1_point
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) user1_point <= 6'b0;
	else begin 
		case(n_state)
			INPUT: begin 
				if(input_count < 5) begin
					if(new_card == 11 || new_card == 12 || new_card == 13) user1_point <= user1_point + 6'b1;
					else user1_point <= user1_point + new_card;
				end
			end
			IDLE: if(c_state == OUTPUT2) user1_point <= 6'b0;
		endcase
	end
end

// user2_point
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) user2_point <= 6'b0;
	else begin 
		case(n_state)
			INPUT: begin 
				if(input_count > 4) begin
					if(new_card == 11 || new_card == 12 || new_card == 13) user2_point <= user2_point + 6'b1;
					else user2_point <= user2_point + new_card;
				end
			end
			IDLE: if(c_state == OUTPUT2) user2_point <= 6'b0;
		endcase
	end
end


// table_10
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_10 <= 3'd4;
	else begin 
		case(n_state)
			INPUT: if(new_card == 10) table_10 <= table_10 - 3'b1;
			OUTPUT2: if(table_1 == 2) table_10 <= 3'd4;
		endcase
	end
end

// table_9
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_9 <= 4'd8;
	else begin 
		case(n_state)
			INPUT: if(new_card == 10 || new_card == 9) table_9 <= table_9 - 4'b1;
			OUTPUT2: if(table_1 == 2) table_9 <= 4'd8;
		endcase
	end
end

// table_8
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_8 <= 4'd12;
	else begin 
		case(n_state)
			INPUT: if(new_card == 10 || new_card == 9 || new_card == 8) table_8 <= table_8 - 4'b1;
			OUTPUT2: if(table_1 == 2) table_8 <= 4'd12;
		endcase
	end
end

// table_7
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_7 <= 5'd16;
	else begin 
		case(n_state)
			INPUT: if(new_card < 11 && new_card > 6) table_7 <= table_7 - 5'b1;
			OUTPUT2: if(table_1 == 2) table_7 <= 5'd16;
		endcase
	end
end

// table_6
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_6 <= 5'd20;
	else begin 
		case(n_state)
			INPUT: if(new_card < 11 && new_card > 5) table_6 <= table_6 - 5'b1;
			OUTPUT2: if(table_1 == 2) table_6 <= 5'd20;
		endcase
	end
end

// table_5
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_5 <= 5'd24;
	else begin 
		case(n_state)
			INPUT: if(new_card < 11 && new_card > 4) table_5 <= table_5 - 5'b1;
			OUTPUT2: if(table_1 == 2) table_5 <= 5'd24;
		endcase
	end
end

// table_4
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_4 <= 5'd28;
	else begin 
		case(n_state)
			INPUT: if(new_card < 11 && new_card > 3) table_4 <= table_4 - 5'b1;
			OUTPUT2: if(table_1 == 2) table_4 <= 5'd28;
		endcase
	end
end

// table_3
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_3 <= 6'd32;
	else begin 
		case(n_state)
			INPUT: if(new_card != 1 && new_card < 11 && new_card != 2) table_3 <= table_3 - 6'b1;
			OUTPUT2: if(table_1 == 2) table_3 <= 6'd32;
		endcase
	end
end

// table_2
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_2 <= 6'd36;
	else begin 
		case(n_state)
			INPUT: if(new_card != 1 && new_card < 11) table_2 <= table_2 - 6'b1;
			OUTPUT2: if(table_1 == 2) table_2 <= 6'd36;
		endcase
	end
end

// table_1
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) table_1 <= 6'd52;
	else begin 
		case(n_state)
			INPUT: table_1 <= table_1 - 6'b1;
			OUTPUT2: if(table_1 == 2) table_1 <= 6'd52;
		endcase
	end
end

// cards_to_equal
always@(posedge clk3 or negedge rst_n) begin
	if(!rst_n) cards_to_equal <= 6'b0; 
	else if(n_state == BUBBLE2 && input_count < 6) begin 
		case(user1_point)
			20: cards_to_equal <= table_1 - table_2;
			19: cards_to_equal <= table_2 - table_3;
			18: cards_to_equal <= table_3 - table_4;
			17: cards_to_equal <= table_4 - table_5;
			16: cards_to_equal <= table_5 - table_6;
			15: cards_to_equal <= table_6 - table_7;
			14: cards_to_equal <= table_7 - table_8;
			13: cards_to_equal <= table_8 - table_9;
			12: cards_to_equal <= table_9 - table_10;
			11: cards_to_equal <= table_10;
			default: cards_to_equal <= 6'b0;
		endcase
	end
	else if(n_state == BUBBLE2) begin 
		case(user2_point)
			20: cards_to_equal <= table_1 - table_2;
			19: cards_to_equal <= table_2 - table_3;
			18: cards_to_equal <= table_3 - table_4;
			17: cards_to_equal <= table_4 - table_5;
			16: cards_to_equal <= table_5 - table_6;
			15: cards_to_equal <= table_6 - table_7;
			14: cards_to_equal <= table_7 - table_8;
			13: cards_to_equal <= table_8 - table_9;
			12: cards_to_equal <= table_9 - table_10;
			11: cards_to_equal <= table_10;
			default: cards_to_equal <= 6'b0;
		endcase
	end
end

// cards_to_exceed
always@(posedge clk3 or negedge rst_n) begin
	if(!rst_n) cards_to_exceed <= 6'b0;
	else if(n_state == BUBBLE2 && input_count < 6) begin 
		if(user1_point < 12) cards_to_exceed <= 6'b0;
		else begin 
			case(user1_point)
				20: cards_to_exceed <= table_2;
				19: cards_to_exceed <= table_3;
				18: cards_to_exceed <= table_4;
				17: cards_to_exceed <= table_5;
				16: cards_to_exceed <= table_6;
				15: cards_to_exceed <= table_7;
				14: cards_to_exceed <= table_8;
				13: cards_to_exceed <= table_9;
				12: cards_to_exceed <= table_10;
				default: cards_to_exceed <= table_1;
			endcase
		end
	end
	else if(n_state == BUBBLE2) begin 
		if(user2_point < 12) cards_to_exceed <= 6'b0;
		else begin 
			case(user2_point)
				20: cards_to_exceed <= table_2;
				19: cards_to_exceed <= table_3;
				18: cards_to_exceed <= table_4;
				17: cards_to_exceed <= table_5;
				16: cards_to_exceed <= table_6;
				15: cards_to_exceed <= table_7;
				14: cards_to_exceed <= table_8;
				13: cards_to_exceed <= table_9;
				12: cards_to_exceed <= table_10;
				default: cards_to_exceed <= table_1;
			endcase
		end
	end
end

// dividend_equal
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) dividend_equal <= 13'b0;
	else if(n_state == BUBBLE) dividend_equal <= cards_to_equal * 100;
	else if(n_state == OUTPUT1 && dividend_equal > divisor - 1) dividend_equal <= dividend_equal - divisor;
end

// dividend_exceed
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) dividend_exceed <= 13'b0;
	else if(n_state == BUBBLE) dividend_exceed <= cards_to_exceed * 100;
	else if(n_state == OUTPUT1 && dividend_exceed > divisor - 1) dividend_exceed <= dividend_exceed - divisor;
end

// divisor
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) divisor <= 12'b0;
	else if(n_state == BUBBLE) divisor <= table_1 << 6;
	else if(n_state == OUTPUT1) divisor <= divisor >> 1;
end

assign yes_winner = user1_point != user2_point && (user1_point < 22 || user2_point < 22);

// out_valid1
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) out_valid1 <= 1'b0;
	else begin 
		case(n_state)
			OUTPUT1: out_valid1 <= 1'b1;
			default: out_valid1 <= 1'b0;
		endcase
	end
end

// equal
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) equal <= 1'b0;
	else begin 
		case(n_state)
			OUTPUT1: equal <= dividend_equal > divisor - 1;
			default: equal <= 1'b0;
		endcase
	end
end

// exceed
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) exceed <= 1'b0;
	else begin 
		case(n_state)
			OUTPUT1: exceed <= dividend_exceed > divisor - 1;
			default: exceed <= 1'b0;
		endcase
	end
end

// out_valid2
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) out_valid2 <= 1'b0;
	else begin 
		case(n_state)
			OUTPUT2: out_valid2 <= 1'b1;
			default: out_valid2 <= 1'b0;
		endcase
	end
end

// winner
always@(posedge clk3 or negedge rst_n) begin 
	if(!rst_n) winner <= 1'b0;
	else begin 
		case(n_state)
			OUTPUT2: begin 
				if(!yes_winner) winner <= 1'b0;
				else if(output_count == 0) winner <= 1'b1;
				else winner <= (user1_point > 21 || (user2_point < 22 && user2_point > user1_point));
			end
			default: winner <= 1'b0;
		endcase
	end
end

//---------------------------------------------------------------------
//   syn_XOR
//---------------------------------------------------------------------
syn_XOR u_syn_XOR(.IN(any_valid),.OUT(input_valid),.TX_CLK(clk1),.RX_CLK(clk3),.RST_N(rst_n));


endmodule
