`timescale 1ns / 1ps
//===========================================================================
// motor_drive : 有符号控制量 → TB6612 指令（方向 + 占空比）
//   控制量量程 ±4095（|u|>>4 映射到 duty 0~255，4095→255）
//   死区：|u| < deadband 时输出停止（00），消除小信号 PWM 微振
//   SIGN 参数：方向极性修正（+1 / −1），上板标定时按电机实际转向翻转；
//   仿真物理模型下 SIGN=+1 使 Kp 取正值（详见 README 推导）
//===========================================================================
module motor_drive #(
    parameter integer SIGN = 1
)(
    input               clk_50m ,
    input               rst_n   ,
    input  signed [15:0] u       ,   // 控制量（实际使用 ±4095 量级）
    input  signed [15:0] deadband,   // 死区宽度（正数）
    output reg  [1:0]   sel     ,   // 00停 01正转 10反转 11刹车
    output reg  [7:0]   duty        // 0~255
);

reg signed [15:0] u_eff;    // 极性修正后的控制量
reg signed [15:0] u_abs;    // |u_eff|
reg [7:0]         duty_tmp;

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        sel  <= 2'b00;
        duty <= 8'd0;
    end else begin
        // 极性修正（u 实际不会到 -32768，取负安全）
        u_eff = (SIGN >= 0) ? u : -u;
        u_abs = (u_eff < 0) ? -u_eff : u_eff;
        if (u_abs < deadband) begin
            // 死区：停止
            sel  <= 2'b00;
            duty <= 8'd0;
        end else begin
            // |u|>>4 映射到 0~255，饱和处理
            if (u_abs >= 16'sd4080) duty_tmp = 8'd255;
            else                    duty_tmp = u_abs[11:4];
            duty <= duty_tmp;
            sel  <= (u_eff > 0) ? 2'b01 : 2'b10;
        end
    end
end

endmodule
