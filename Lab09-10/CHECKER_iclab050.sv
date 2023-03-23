module Checker(input clk, INF.CHECKER inf);
import usertype::*;

//declare other cover group

covergroup COVID@(posedge clk iff inf.id_valid);
    coverpoint inf.D.d_id[0]{
        option.auto_bin_max = 256;
    }
endgroup

covergroup TRANS@(posedge clk iff inf.act_valid);
    coverpoint inf.D.d_act[0]{
        option.at_least = 10;
        bins TT = (Take => Take);
        bins TD = (Take => Deliver);
        bins TO = (Take => Order);
        bins TC = (Take => Cancel);

        bins DD = (Deliver => Deliver);
        bins DO = (Deliver => Order);
        bins DC = (Deliver => Cancel);
        bins DT = (Deliver => Take);

        bins OO = (Order => Order);
        bins OC = (Order => Cancel);
        bins OT = (Order => Take);
        bins OD = (Order => Deliver);

        bins CC = (Cancel => Cancel);
        bins CT = (Cancel => Take);
        bins CD = (Cancel => Deliver);
        bins CO = (Cancel => Order);
    }
endgroup

covergroup COM@(negedge clk iff inf.out_valid);
    coverpoint inf.complete{
        option.at_least = 200;
        bins good = {1'b1};
        bins bad = {1'b0};
    }
endgroup

covergroup MSG@(negedge clk iff inf.out_valid);
    coverpoint inf.err_msg{
        option.at_least = 20;
        bins no_food = {No_Food};
        bins man_busy = {D_man_busy};
        bins no_cus = {No_customers};
        bins res_busy = {Res_busy};
        bins wrong_can = {Wrong_cancel};
        bins wrong_res = {Wrong_res_ID};
        bins wrong_food = {Wrong_food_ID};
    }
endgroup

COVID covid = new();
TRANS trans = new();
COM com = new();
MSG msg = new();

//************************************ below assertion is to check your pattern ***************************************** 
//                                          Please finish and hand in it
// This is an example assertion given by TA, please write the required assertions below
//  assert_interval : assert property ( @(posedge clk)  inf.out_valid |=> inf.id_valid == 0 [*2])
//  else
//  begin
//  	$display("Assertion X is violated");
//  	$fatal; 
//  end
wire #(0.5) rst_reg = inf.rst_n;
//write other assertions
//========================================================================================================================================================
// Assertion 1 ( All outputs signals (including FD.sv and bridge.sv) should be zero after reset.)
//========================================================================================================================================================

Action action_reg;
always_ff @(posedge clk or negedge inf.rst_n) begin
    if(!inf.rst_n) action_reg <= No_action;
    else if(inf.act_valid) action_reg <= inf.D.d_act[0];
end

// 1. All output signals (including FD.sv and bridge.sv) should be zero after reset.
Assertion_1: assert property(@(negedge rst_reg) !inf.out_valid && !inf.err_msg && !inf.complete && !inf.out_info && !inf.C_in_valid && !inf.C_out_valid && !inf.C_data_r && !inf.AR_VALID && !inf.AR_ADDR && !inf.R_READY && !inf.AW_VALID && !inf.AW_ADDR && !inf.W_VALID && !inf.W_DATA && !inf.B_READY && !inf.C_addr && !inf.C_data_w && !inf.C_r_wb)
else begin 
    $display("Assertion 1 is violated");
    $fatal;
end

// 2. If action is completed, err_msg should be 4’b0.
Assertion_2: assert property(@(posedge clk) inf.out_valid && inf.complete |-> inf.err_msg == 4'b0)
else begin 
    $display("Assertion 2 is violated");
    $fatal;
end

// 3. If action is not completed, out_info should be 64’b0.
Assertion_3: assert property(@(posedge clk) inf.out_valid && !inf.complete |-> inf.out_info == 64'b0)
else begin 
    $display("Assertion 3 is violated");
    $fatal;
end

// 4. The gap between each input valid is at least 1 cycle and at most 5 cycles.
Assertion_4: assert property(@(posedge clk) inf.act_valid || inf.res_valid || inf.food_valid || inf.id_valid || inf.cus_valid |=> !(inf.act_valid || inf.res_valid || inf.food_valid || inf.id_valid || inf.cus_valid))
else begin 
    $display("Assertion 4 is violated");
    $fatal;
end
Assertion_401: assert property(@(posedge clk) inf.act_valid && inf.D.d_act[0] == Take |=> ##[1:5] inf.id_valid || inf.cus_valid)
else begin 
    $display("Assertion 4 is violated");
    $fatal;
end
Assertion_402: assert property(@(posedge clk) inf.act_valid && inf.D.d_act[0] == Deliver |=> ##[1:5] inf.id_valid)
else begin 
    $display("Assertion 4 is violated");
    $fatal;
end
Assertion_403: assert property(@(posedge clk) inf.act_valid && inf.D.d_act[0] == Order |=> ##[1:5] inf.res_valid || inf.food_valid)
else begin 
    $display("Assertion 4 is violated");
    $fatal;
end
Assertion_404: assert property(@(posedge clk) inf.act_valid && inf.D.d_act[0] == Cancel |=> ##[1:5] inf.res_valid)
else begin 
    $display("Assertion 4 is violated");
    $fatal;
end
Assertion_41: assert property(@(posedge clk) inf.res_valid |=> ##[1:5] inf.food_valid)
else begin 
    $display("Assertion 4 is violated");
    $fatal;
end
Assertion_42: assert property(@(posedge clk) inf.food_valid && action_reg == Cancel |=> ##[1:5] inf.id_valid)
else begin 
    $display("Assertion 4 is violated");
    $fatal;
end
Assertion_43: assert property(@(posedge clk) inf.id_valid && action_reg == Take |=> ##[1:5] inf.cus_valid)
else begin 
    $display("Assertion 4 is violated");
    $fatal;
end

// 5. All input valid signals won’t overlap with each other.
Assertion_50: assert property(@(posedge clk) inf.act_valid |-> !inf.res_valid && !inf.food_valid && !inf.id_valid && !inf.cus_valid)
else begin 
    $display("Assertion 5 is violated");
    $fatal;
end
Assertion_51: assert property(@(posedge clk) inf.res_valid |-> !inf.act_valid && !inf.food_valid && !inf.id_valid && !inf.cus_valid)
else begin 
    $display("Assertion 5 is violated");
    $fatal;
end
Assertion_52: assert property(@(posedge clk) inf.food_valid |-> !inf.res_valid && !inf.act_valid && !inf.id_valid && !inf.cus_valid)
else begin
    $display("Assertion 5 is violated");
    $fatal;
end
Assertion_53: assert property(@(posedge clk) inf.id_valid |-> !inf.res_valid && !inf.food_valid && !inf.act_valid && !inf.cus_valid)
else begin 
    $display("Assertion 5 is violated");
    $fatal;
end
Assertion_54: assert property(@(posedge clk) inf.cus_valid |-> !inf.res_valid && !inf.food_valid && !inf.id_valid && !inf.act_valid)
else begin 
    $display("Assertion 5 is violated");
    $fatal;
end

// 6. Out_valid can only be high for exactly one cycle.
Assertion_6: assert property(@(posedge clk) inf.out_valid |=> !inf.out_valid)
else begin 
    $display("Assertion 6 is violated");
    $fatal;
end

//  7. Next operation will be valid 2-10 cycles after out_valid fall.
Assertion_70: assert property(@(posedge clk) inf.out_valid |-> ##[2:10] inf.act_valid)
else begin 
    $display("Assertion 7 is violated");
    $fatal;
end
Assertion_71: assert property(@(posedge clk) inf.out_valid |=> !inf.act_valid)
else begin 
    $display("Assertion 7 is violated");
    $fatal;
end

// 8. Latency should be less than 1200 cycles for each operation.
Assertion_8: assert property(@(posedge clk) (action_reg == Take && inf.cus_valid) || (action_reg == Deliver && inf.id_valid) || (action_reg == Order && inf.food_valid) || (action_reg == Cancel && inf.id_valid) |=> ##[1:1199] inf.out_valid)
else begin 
    $display("Assertion 8 is violated");
    $fatal;
end

endmodule