module FD(input clk, INF.FD_inf inf);
import usertype::*;

//===========================================================================
// parameter 
//===========================================================================

enum logic [3:0] {
    IDLE,
    ACTION, 
    RESTAURANT, 
    FOOD, 
    DELIVERYMAN, 
    CUSTOMER, 
    D_READ, 
    R_READ, 
    D_WRITE, 
    R_WRITE,
    SAME,
    MESSAGE1, 
    MESSAGE2, 
    MESSAGE3, 
    COMPLETE
} c_state, n_state;

//===========================================================================
// logic 
//===========================================================================

Action action_reg;
Restaurant_id restaurant_reg;
food_ID_servings food_regs;
Delivery_man_id delivery_reg;
Ctm_Info customer_regs;

logic man_busy;
logic no_food;
logic no_customers;

logic [9:0] busy2;
logic restaurant_busy;

logic wrong_cancel;
logic wrong_restaurant_id;
logic wrong_food_id;

logic same_id;
logic same_id_reg;

D_man_Info d_info, old_d_info, d_to_b_encode;
res_info r_info, old_r_info, r_to_b_encode;

logic[63:0] encoder;

Customer_status decoder_ctm1;
Restaurant_id decoder_res1;
Food_id decoder_food1;
servings_of_food decoder_ser1;

Customer_status decoder_ctm2;
Restaurant_id decoder_res2;
Food_id decoder_food2;
servings_of_food decoder_ser2;

limit_of_orders decoder_limit;
servings_of_FOOD decoder_FOOD1;
servings_of_FOOD decoder_FOOD2;
servings_of_FOOD decoder_FOOD3;

//===========================================================================
// FSM
//===========================================================================

// c_state
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) c_state <= IDLE;
    else c_state <= n_state;
end

// n_state
always_comb begin
    case(c_state)
        IDLE: begin 
            if(inf.act_valid) n_state = ACTION;
            else if(inf.res_valid) n_state = RESTAURANT;
            else if(inf.food_valid) n_state = FOOD;
            else if(inf.id_valid) n_state = DELIVERYMAN;
            else if(inf.cus_valid) n_state = CUSTOMER;
            else n_state = c_state;
        end
        ACTION: n_state = IDLE;
        RESTAURANT: n_state = IDLE;
        FOOD: begin 
            if(action_reg == Order) n_state = R_READ;
            else n_state = IDLE;
        end
        DELIVERYMAN: begin 
            if(action_reg == Deliver || action_reg == Cancel) n_state = D_READ;
            else n_state = IDLE;
        end
        CUSTOMER: begin 
            n_state = D_READ;
        end
        D_READ: begin 
            if(inf.C_out_valid) begin 
                case(action_reg)
                    Take: begin 
                        if(man_busy) n_state = MESSAGE1;
                        else if(same_id_reg && no_food) n_state = MESSAGE2;
                        else if(same_id_reg) n_state = SAME;
                        else n_state = R_READ;
                    end
                    Deliver: begin 
                        if(no_customers) n_state = MESSAGE1;
                        else n_state = D_WRITE;
                    end
                    Cancel: begin 
                        if(wrong_cancel) n_state = MESSAGE1;
                        else if(wrong_restaurant_id) n_state = MESSAGE2;
                        else if(wrong_food_id) n_state = MESSAGE3;
                        else n_state = D_WRITE;
                    end
                    default: n_state = c_state;
                endcase
            end
            else n_state = c_state;
        end
        R_READ: begin 
            if(inf.C_out_valid) begin 
                case(action_reg)
                    Take: begin 
                        if(no_food) n_state = MESSAGE2;
                        else n_state = D_WRITE;
                    end
                    Order: begin 
                        if(restaurant_busy) n_state = MESSAGE1;
                        else n_state = R_WRITE;
                    end
                    default: n_state = c_state;
                endcase
            end
            else n_state = c_state;
        end
        D_WRITE: begin 
            if(inf.C_out_valid && action_reg == Take) n_state = R_WRITE;
            else if(inf.C_out_valid) n_state = COMPLETE;
            else n_state = c_state;
        end
        R_WRITE: begin 
            if(inf.C_out_valid) n_state = COMPLETE;
            else n_state = c_state;
        end
        SAME: begin 
            if(inf.C_out_valid) n_state = COMPLETE;
            else n_state = c_state;
        end
        MESSAGE1: n_state = IDLE;
        MESSAGE2: n_state = IDLE;
        MESSAGE3: n_state = IDLE;
        COMPLETE: n_state = IDLE;
        default: n_state = c_state;
    endcase
end

assign man_busy = decoder_ctm2 != None;

assign no_customers = decoder_ctm1 == None;

assign busy2 = food_regs.d_ser_food + decoder_FOOD1 + decoder_FOOD2 + decoder_FOOD3;

assign restaurant_busy = busy2 > decoder_limit;

assign wrong_cancel = decoder_ctm1 == None;

assign wrong_restaurant_id =
    (decoder_res1 != restaurant_reg) &&
    ((decoder_ctm2 == None) || (decoder_ctm2 != None && decoder_res2 != restaurant_reg));

assign wrong_food_id = 
    ((decoder_res1 == restaurant_reg && decoder_food1 != food_regs.d_food_ID) &&
    !(decoder_ctm2 != None && decoder_res2 == restaurant_reg && decoder_food2 == food_regs.d_food_ID)) || 
    (decoder_res1 != restaurant_reg && decoder_food2 != food_regs.d_food_ID);

assign same_id = delivery_reg == inf.D.d_ctm_info[0].res_ID;

// no_food
always_comb begin 
    case(food_regs.d_food_ID)
        FOOD1: begin 
            if(food_regs.d_ser_food > decoder_FOOD1) no_food = 1'b1;
            else no_food = 1'b0;
        end
        FOOD2: begin 
            if(food_regs.d_ser_food > decoder_FOOD2) no_food = 1'b1;
            else no_food = 1'b0;
        end
        FOOD3: begin 
            if(food_regs.d_ser_food > decoder_FOOD3) no_food = 1'b1;
            else no_food = 1'b0;
        end
        default: no_food = 1'b0;
    endcase
end

// same_id_reg
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) same_id_reg <= 1'b0;
    else begin 
        case(n_state)
            CUSTOMER: same_id_reg <= same_id;
        endcase
    end
end

//===========================================================================
// INPUT
//===========================================================================

// action_reg
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) action_reg <= No_action;
    else if(n_state == ACTION) action_reg <= inf.D.d_act[0];
end

// restaurant_reg
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) restaurant_reg <= 8'b0;
    else if(n_state == RESTAURANT) restaurant_reg <= inf.D.d_res_id[0];
    else if(n_state == CUSTOMER) restaurant_reg <= inf.D.d_ctm_info[0].res_ID;
end

// food_regs
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) begin 
        food_regs.d_food_ID <= No_food;
        food_regs.d_ser_food <= 4'b0;
    end
    else if(n_state == FOOD) begin 
        food_regs.d_food_ID <= inf.D.d_food_ID_ser[0].d_food_ID;
        food_regs.d_ser_food <= inf.D.d_food_ID_ser[0].d_ser_food;
    end
    else if(n_state == CUSTOMER) begin 
        food_regs.d_food_ID <= inf.D.d_ctm_info[0].food_ID;
        food_regs.d_ser_food <= inf.D.d_ctm_info[0].ser_food;
    end
end

// delivery_reg
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) delivery_reg <= 8'b0;
    else if(n_state == DELIVERYMAN) delivery_reg <= inf.D.d_id[0];
end

// customer_regs
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) customer_regs.ctm_status <= None;
    else if(n_state == CUSTOMER) begin 
        customer_regs.ctm_status <= inf.D.d_ctm_info[0].ctm_status;
    end
end

//===========================================================================
// bridge
//===========================================================================

// inf.C_addr
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.C_addr <= 8'b0;
    else begin 
        case(n_state)
            D_READ: inf.C_addr <= delivery_reg;
            R_READ: inf.C_addr <= restaurant_reg;
            D_WRITE: inf.C_addr <= delivery_reg;
            R_WRITE: inf.C_addr <= restaurant_reg;
            SAME: inf.C_addr <= restaurant_reg;
            default: inf.C_addr <= 8'b0;
        endcase
    end
end

// inf.C_r_wb
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.C_r_wb <= 1'b0;
    else begin 
        case(n_state) 
            D_READ: inf.C_r_wb <= 1'b1;
            R_READ: inf.C_r_wb <= 1'b1;
            D_WRITE: inf.C_r_wb <= 1'b0;
            R_WRITE: inf.C_r_wb <= 1'b0;
            SAME: inf.C_r_wb <= 1'b0;
            default: inf.C_r_wb <= 1'b0;
        endcase
    end
end

// inf.C_in_valid
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.C_in_valid <= 1'b0;
    else begin 
        case(n_state)
            D_READ: begin 
                if(c_state != D_READ) inf.C_in_valid <= 1'b1;
                else inf.C_in_valid <= 1'b0;
            end
            R_READ: begin 
                if(c_state != R_READ) inf.C_in_valid <= 1'b1;
                else inf.C_in_valid <= 1'b0;
            end
            D_WRITE: begin 
                if(c_state != D_WRITE) inf.C_in_valid <= 1'b1;
                else inf.C_in_valid <= 1'b0;
            end
            R_WRITE: begin 
                if(c_state != R_WRITE) inf.C_in_valid <= 1'b1;
                else inf.C_in_valid <= 1'b0;
            end
            SAME: begin 
                if(c_state != SAME) inf.C_in_valid <= 1'b1;
                else inf.C_in_valid <= 1'b0;
            end
            default: inf.C_in_valid <= 1'b0;
        endcase
    end
end

// inf.C_data_w
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.C_data_w <= 64'b0;
    else if(n_state == D_WRITE || n_state == R_WRITE || n_state == SAME) inf.C_data_w <= encoder;
    else inf.C_data_w <= 64'b0;
end

assign encoder[7:0] = r_to_b_encode.limit_num_orders;
assign encoder[15:8] = r_to_b_encode.ser_FOOD1;
assign encoder[23:16] = r_to_b_encode.ser_FOOD2;
assign encoder[31:24] = r_to_b_encode.ser_FOOD3;
assign encoder[39:32] = {d_to_b_encode.ctm_info1.ctm_status, d_to_b_encode.ctm_info1.res_ID[7:2]};
assign encoder[47:40] = {d_to_b_encode.ctm_info1.res_ID[1:0], d_to_b_encode.ctm_info1.food_ID, d_to_b_encode.ctm_info1.ser_food};
assign encoder[55:48] = {d_to_b_encode.ctm_info2.ctm_status, d_to_b_encode.ctm_info2.res_ID[7:2]};
assign encoder[63:56] = {d_to_b_encode.ctm_info2.res_ID[1:0], d_to_b_encode.ctm_info2.food_ID, d_to_b_encode.ctm_info2.ser_food};

assign d_to_b_encode = (n_state == R_WRITE)? old_d_info : d_info;
assign r_to_b_encode = (n_state == D_WRITE)? old_r_info : r_info;

// d_info
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) begin 
        d_info.ctm_info1.ctm_status <= None;
        d_info.ctm_info1.res_ID <= 8'b0;
        d_info.ctm_info1.food_ID <= No_food;
        d_info.ctm_info1.ser_food <= 4'b0;
        d_info.ctm_info2.ctm_status <= None;
        d_info.ctm_info2.res_ID <= 8'b0;
        d_info.ctm_info2.food_ID <= No_food;
        d_info.ctm_info2.ser_food <= 4'b0;
    end
    else if(c_state == D_READ && inf.C_out_valid) begin 
        case(action_reg)
            Take: begin 
                if(n_state == SAME) begin 
                    if((customer_regs.ctm_status == VIP && decoder_ctm1 != VIP) || decoder_ctm1 == None) begin 
                        d_info.ctm_info1.ctm_status <= customer_regs.ctm_status;
                        d_info.ctm_info1.res_ID     <= restaurant_reg;
                        d_info.ctm_info1.food_ID    <= food_regs.d_food_ID;
                        d_info.ctm_info1.ser_food   <= food_regs.d_ser_food;
                        d_info.ctm_info2.ctm_status <= decoder_ctm1 ;
                        d_info.ctm_info2.res_ID     <= decoder_res1 ;
                        d_info.ctm_info2.food_ID    <= decoder_food1;
                        d_info.ctm_info2.ser_food   <= decoder_ser1 ;
                    end
                    else begin 
                        d_info.ctm_info1.ctm_status <= decoder_ctm1 ;
                        d_info.ctm_info1.res_ID     <= decoder_res1 ;
                        d_info.ctm_info1.food_ID    <= decoder_food1;
                        d_info.ctm_info1.ser_food   <= decoder_ser1 ;
                        d_info.ctm_info2.ctm_status <= customer_regs.ctm_status;
                        d_info.ctm_info2.res_ID     <= restaurant_reg;
                        d_info.ctm_info2.food_ID    <= food_regs.d_food_ID;
                        d_info.ctm_info2.ser_food   <= food_regs.d_ser_food;
                    end
                end
                else begin 
                    d_info.ctm_info1.ctm_status <= decoder_ctm1;
                    d_info.ctm_info1.res_ID <= decoder_res1;
                    d_info.ctm_info1.food_ID <= decoder_food1;
                    d_info.ctm_info1.ser_food <= decoder_ser1;
                    d_info.ctm_info2.ctm_status <= decoder_ctm2;
                    d_info.ctm_info2.res_ID <= decoder_res2;
                    d_info.ctm_info2.food_ID <= decoder_food2;
                    d_info.ctm_info2.ser_food <= decoder_ser2;
                end
            end
            Deliver: begin 
                d_info.ctm_info1.ctm_status <= decoder_ctm2;
                d_info.ctm_info1.res_ID <= decoder_res2;
                d_info.ctm_info1.food_ID <= decoder_food2;
                d_info.ctm_info1.ser_food <= decoder_ser2;
                d_info.ctm_info2.ctm_status <= None;
                d_info.ctm_info2.res_ID <= 8'b0;
                d_info.ctm_info2.food_ID <= No_food;
                d_info.ctm_info2.ser_food <= 4'b0;
            end
            Cancel: begin 
                if(n_state == D_WRITE) begin 
                    if(decoder_res1 == restaurant_reg &&
                        decoder_food1 == food_regs.d_food_ID &&
                        decoder_res2 == restaurant_reg &&
                        decoder_food2 == food_regs.d_food_ID) begin 
                            d_info.ctm_info1.ctm_status <= None;
                            d_info.ctm_info1.res_ID <= 8'b0;
                            d_info.ctm_info1.food_ID <= No_food;
                            d_info.ctm_info1.ser_food <= 4'b0;
                            d_info.ctm_info2.ctm_status <= None;
                            d_info.ctm_info2.res_ID <= 8'b0;
                            d_info.ctm_info2.food_ID <= No_food;
                            d_info.ctm_info2.ser_food <= 4'b0;
                        end
                    else if(decoder_res1 == restaurant_reg &&
                        decoder_food1 == food_regs.d_food_ID) begin 
                            d_info.ctm_info1.ctm_status <= decoder_ctm2;
                            d_info.ctm_info1.res_ID <= decoder_res2;
                            d_info.ctm_info1.food_ID <= decoder_food2;
                            d_info.ctm_info1.ser_food <= decoder_ser2;
                            d_info.ctm_info2.ctm_status <= None;
                            d_info.ctm_info2.res_ID <= 8'b0;
                            d_info.ctm_info2.food_ID <= No_food;
                            d_info.ctm_info2.ser_food <= 4'b0;
                        end
                    else begin 
                        d_info.ctm_info1.ctm_status <= decoder_ctm1;
                        d_info.ctm_info1.res_ID <= decoder_res1;
                        d_info.ctm_info1.food_ID <= decoder_food1;
                        d_info.ctm_info1.ser_food <= decoder_ser1;
                        d_info.ctm_info2.ctm_status <= None;
                        d_info.ctm_info2.res_ID <= 8'b0;
                        d_info.ctm_info2.food_ID <= No_food;
                        d_info.ctm_info2.ser_food <= 4'b0;
                    end
                end
            end
        endcase
    end
    else if(c_state == R_READ && n_state == D_WRITE) begin 
        if((customer_regs.ctm_status == VIP && d_info.ctm_info1.ctm_status != VIP) || d_info.ctm_info1.ctm_status == None) begin 
            d_info.ctm_info1.ctm_status <= customer_regs.ctm_status;
            d_info.ctm_info1.res_ID     <= restaurant_reg;
            d_info.ctm_info1.food_ID    <= food_regs.d_food_ID;
            d_info.ctm_info1.ser_food   <= food_regs.d_ser_food;
            d_info.ctm_info2.ctm_status <= d_info.ctm_info1.ctm_status;
            d_info.ctm_info2.res_ID     <= d_info.ctm_info1.res_ID    ;
            d_info.ctm_info2.food_ID    <= d_info.ctm_info1.food_ID   ;
            d_info.ctm_info2.ser_food   <= d_info.ctm_info1.ser_food  ;
        end
        else begin 
            d_info.ctm_info2.ctm_status <= customer_regs.ctm_status;
            d_info.ctm_info2.res_ID <= restaurant_reg;
            d_info.ctm_info2.food_ID <= food_regs.d_food_ID;
            d_info.ctm_info2.ser_food <= food_regs.d_ser_food;
        end
    end
end

// old_r_info
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) begin 
        old_r_info.limit_num_orders <= 8'b0;
        old_r_info.ser_FOOD1 <= 8'b0;
        old_r_info.ser_FOOD2 <= 8'b0;
        old_r_info.ser_FOOD3 <= 8'b0;
    end
    else if(c_state == D_READ && inf.C_out_valid) begin 
        old_r_info.limit_num_orders <= decoder_limit;
        old_r_info.ser_FOOD1 <= decoder_FOOD1;
        old_r_info.ser_FOOD2 <= decoder_FOOD2;
        old_r_info.ser_FOOD3 <= decoder_FOOD3;
    end
end

// r_info
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) begin 
        r_info.limit_num_orders <= 8'b0;
        r_info.ser_FOOD1 <= 8'b0;
        r_info.ser_FOOD2 <= 8'b0;
        r_info.ser_FOOD3 <= 8'b0;
    end
    else if(c_state == R_READ && inf.C_out_valid) begin 
        case(action_reg)
            Take: begin 
                if(n_state == D_WRITE) begin 
                    case(food_regs.d_food_ID)
                        FOOD1: begin 
                            r_info.limit_num_orders <= decoder_limit;
                            r_info.ser_FOOD1 <= decoder_FOOD1 - food_regs.d_ser_food;
                            r_info.ser_FOOD2 <= decoder_FOOD2;
                            r_info.ser_FOOD3 <= decoder_FOOD3;
                        end
                        FOOD2: begin 
                            r_info.limit_num_orders <= decoder_limit;
                            r_info.ser_FOOD1 <= decoder_FOOD1;
                            r_info.ser_FOOD2 <= decoder_FOOD2 - food_regs.d_ser_food;
                            r_info.ser_FOOD3 <= decoder_FOOD3;
                        end
                        FOOD3: begin 
                            r_info.limit_num_orders <= decoder_limit;
                            r_info.ser_FOOD1 <= decoder_FOOD1;
                            r_info.ser_FOOD2 <= decoder_FOOD2;
                            r_info.ser_FOOD3 <= decoder_FOOD3 - food_regs.d_ser_food;
                        end
                    endcase
                end
            end
            Order: begin 
                if(n_state == R_WRITE) begin 
                    case(food_regs.d_food_ID)
                        FOOD1: begin 
                            r_info.limit_num_orders <= decoder_limit;
                            r_info.ser_FOOD1 <= decoder_FOOD1 + food_regs.d_ser_food;
                            r_info.ser_FOOD2 <= decoder_FOOD2;
                            r_info.ser_FOOD3 <= decoder_FOOD3;
                        end
                        FOOD2: begin 
                            r_info.limit_num_orders <= decoder_limit;
                            r_info.ser_FOOD1 <= decoder_FOOD1;
                            r_info.ser_FOOD2 <= decoder_FOOD2 + food_regs.d_ser_food;
                            r_info.ser_FOOD3 <= decoder_FOOD3;
                        end
                        FOOD3: begin 
                            r_info.limit_num_orders <= decoder_limit;
                            r_info.ser_FOOD1 <= decoder_FOOD1;
                            r_info.ser_FOOD2 <= decoder_FOOD2;
                            r_info.ser_FOOD3 <= decoder_FOOD3 + food_regs.d_ser_food;
                        end
                    endcase
                end
            end
        endcase
    end
    else if(c_state == D_READ && n_state == SAME) begin 
        case(food_regs.d_food_ID)
            FOOD1: begin 
                r_info.limit_num_orders <= decoder_limit;
                r_info.ser_FOOD1 <= decoder_FOOD1 - food_regs.d_ser_food;
                r_info.ser_FOOD2 <= decoder_FOOD2;
                r_info.ser_FOOD3 <= decoder_FOOD3;
            end
            FOOD2: begin 
                r_info.limit_num_orders <= decoder_limit;
                r_info.ser_FOOD1 <= decoder_FOOD1;
                r_info.ser_FOOD2 <= decoder_FOOD2 - food_regs.d_ser_food;
                r_info.ser_FOOD3 <= decoder_FOOD3;
            end
            FOOD3: begin 
                r_info.limit_num_orders <= decoder_limit;
                r_info.ser_FOOD1 <= decoder_FOOD1;
                r_info.ser_FOOD2 <= decoder_FOOD2;
                r_info.ser_FOOD3 <= decoder_FOOD3 - food_regs.d_ser_food;
            end
        endcase
    end
end

// old_d_info
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) begin 
        old_d_info.ctm_info1.ctm_status <= None;
        old_d_info.ctm_info1.res_ID <= 8'b0;
        old_d_info.ctm_info1.food_ID <= No_food;
        old_d_info.ctm_info1.ser_food <= 4'b0;
        old_d_info.ctm_info2.ctm_status <= None;
        old_d_info.ctm_info2.res_ID <= 8'b0;
        old_d_info.ctm_info2.food_ID <= No_food;
        old_d_info.ctm_info2.ser_food <= 4'b0;
    end
    else if(c_state == R_READ && inf.C_out_valid) begin 
        old_d_info.ctm_info1.ctm_status <= decoder_ctm1;
        old_d_info.ctm_info1.res_ID <= decoder_res1;
        old_d_info.ctm_info1.food_ID <= decoder_food1;
        old_d_info.ctm_info1.ser_food <= decoder_ser1;
        old_d_info.ctm_info2.ctm_status <= decoder_ctm2;
        old_d_info.ctm_info2.res_ID <= decoder_res2;
        old_d_info.ctm_info2.food_ID <= decoder_food2;
        old_d_info.ctm_info2.ser_food <= decoder_ser2;
    end
end

//===========================================================================
// decoder
//===========================================================================

// decoder_ctm1
always_comb begin
    case(inf.C_data_r[39:38])
        2'b00: decoder_ctm1 = None;
        2'b01: decoder_ctm1 = Normal;
        2'b11: decoder_ctm1 = VIP;
        default: decoder_ctm1 = None;
    endcase
end

// decoder_res1
always_comb begin 
    decoder_res1 = {inf.C_data_r[37:32], inf.C_data_r[47:46]};
end

// decoder_food1
always_comb begin 
    case(inf.C_data_r[45:44])
        2'd0: decoder_food1 = No_food;
        2'd1: decoder_food1 = FOOD1;
        2'd2: decoder_food1 = FOOD2;
        2'd3: decoder_food1 = FOOD3;
    endcase
end

// decoder_ser1
always_comb begin 
    decoder_ser1 = inf.C_data_r[43:40];
end

// decoder_ctm2
always_comb begin
    case(inf.C_data_r[55:54])
        2'b00: decoder_ctm2 = None;
        2'b01: decoder_ctm2 = Normal;
        2'b11: decoder_ctm2 = VIP;
        default: decoder_ctm2 = None;
    endcase
end

// decoder_res2
always_comb begin 
    decoder_res2 = {inf.C_data_r[53:48], inf.C_data_r[63:62]};
end

// decoder_food2
always_comb begin 
    case(inf.C_data_r[61:60])
        2'd0: decoder_food2 = No_food;
        2'd1: decoder_food2 = FOOD1;
        2'd2: decoder_food2 = FOOD2;
        2'd3: decoder_food2 = FOOD3;
    endcase
end

// decoder_ser2
always_comb begin 
    decoder_ser2 = inf.C_data_r[59:56];
end

assign decoder_limit = inf.C_data_r[7:0];
assign decoder_FOOD1 = inf.C_data_r[15:8];
assign decoder_FOOD2 = inf.C_data_r[23:16];
assign decoder_FOOD3 = inf.C_data_r[31:24];

//===========================================================================
// OUTPUT
//===========================================================================

// inf.err_msg
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.err_msg <= No_Err;
    else begin 
        case(n_state)
            MESSAGE1: begin 
                case(action_reg)
                    Take: inf.err_msg <= D_man_busy;
                    Deliver: inf.err_msg <= No_customers;
                    Order: inf.err_msg <= Res_busy;
                    Cancel: inf.err_msg <= Wrong_cancel;
                endcase
            end
            MESSAGE2: begin 
                if(action_reg == Take) inf.err_msg <= No_Food;
                else inf.err_msg <= Wrong_res_ID;
            end
            MESSAGE3: inf.err_msg <= Wrong_food_ID;
            default: inf.err_msg <= No_Err;
        endcase
    end
end

// inf.complete
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.complete <= 1'b0;
    else if(n_state == COMPLETE) inf.complete <= 1'b1;
    else inf.complete <= 1'b0;
end

// inf.out_valid
always_ff@(posedge clk or negedge inf.rst_n) begin 
    if(!inf.rst_n) inf.out_valid <= 1'b0;
    else begin 
        case(n_state)
            MESSAGE1: inf.out_valid <= 1'b1;
            MESSAGE2: inf.out_valid <= 1'b1;
            MESSAGE3: inf.out_valid <= 1'b1;
            COMPLETE: inf.out_valid <= 1'b1;
            default: inf.out_valid <= 1'b0;
        endcase
    end
end

// inf.out_info
always_ff@(posedge clk or negedge inf.rst_n) begin
    if(!inf.rst_n) inf.out_info <= 64'b0; 
    else if(n_state == COMPLETE) begin 
        case(action_reg)
            Take: inf.out_info <= {d_info, r_info};
            Deliver: inf.out_info <= {d_info, 32'd0};
            Order: inf.out_info <= {32'd0, r_info};
            Cancel: inf.out_info <= {d_info, 32'd0};
            default: inf.out_info <= 64'b0;
        endcase
    end
    else inf.out_info <= 64'b0;
end

endmodule
