`timescale 1ns / 1ps
//===========================================================================
// vel_est : 电位器死区保持 + 摆杆速度估计
//   死区读数打满/跳变时保持上一有效角度,出死区首样本只刷基准不做差分;
//   速度=3ms 差分+4 点 FIR 平均,±512 限幅。theta_hold 是控制链唯一角度入口。
//===========================================================================
module vel_est (
    input              clk_50m ,
    input              rst_n   ,
    input              tick    ,   // 3ms 采样节拍与 adc.v 输出更新同周期
    input      [11:0]  theta_in,   // adc.v 输出的 12 位角度
    output reg [11:0]  theta_hold, // 死区保持后的有效角度
    output reg         theta_valid,// theta_hold 是否已含真实样本首个有效样本后置 1 并保持
    output reg signed [15:0] vel  // 速度估计：计数/采样，已限幅
);

// 电位器死区判定阈值：读数接近满量程或零点视为进入死区
localparam [11:0] RAIL_HI = 12'd4090;
localparam [11:0] RAIL_LO = 12'd5;
// 速度限幅：±512 计数/采样≈ ±14400°/s，远超起摆最大速度，纯保护
localparam signed [15:0] VEL_LIMIT = 16'sd512;

reg signed [11:0] theta_prev;   // 上一有效角度
reg               in_deadzone;  // 上一拍处于死区
reg signed [15:0] diff_d1;      // 差分历史4 点 FIR
reg signed [15:0] diff_d2;
reg signed [15:0] diff_d3;
reg signed [31:0] fir_sum;

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        theta_hold  <= 12'd0;
        theta_prev  <= 12'sd0;
        theta_valid <= 1'b0;
        in_deadzone <= 1'b0;
        diff_d1     <= 16'sd0;
        diff_d2     <= 16'sd0;
        diff_d3     <= 16'sd0;
        fir_sum     <= 32'sd0;
        vel         <= 16'sd0;
    end else if (tick) begin
        if (theta_in >= RAIL_HI || theta_in <= RAIL_LO) begin
            // 死区：保持角度与速度不变theta_valid 保持
            in_deadzone <= 1'b1;
            theta_hold  <= theta_hold;
            vel         <= vel;
        end else if (in_deadzone) begin
            // 出死区后首个有效样本：只刷新基准，速度清零
            in_deadzone <= 1'b0;
            theta_hold  <= theta_in;
            theta_prev  <= $signed(theta_in);
            theta_valid <= 1'b1;
            diff_d1     <= 16'sd0;
            diff_d2     <= 16'sd0;
            diff_d3     <= 16'sd0;
            vel         <= 16'sd0;
        end else begin
            // 正常：差分 + 4 点 FIR 平均 + 限幅
            theta_hold  <= theta_in;
            theta_prev  <= $signed(theta_in);
            theta_valid <= 1'b1;
            diff_d1    <= $signed(theta_in) - theta_prev;
            diff_d2    <= diff_d1;
            diff_d3    <= diff_d2;
            fir_sum    <= $signed(theta_in) - theta_prev + diff_d1 + diff_d2 + diff_d3;
            if      (fir_sum >  (4*VEL_LIMIT)) vel <=  VEL_LIMIT;
            else if (fir_sum < -(4*VEL_LIMIT)) vel <= -VEL_LIMIT;
            else                                vel <= fir_sum >>> 2;
        end
    end
end

endmodule
