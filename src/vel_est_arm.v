`timescale 1ns / 1ps
//===========================================================================
// vel_est_arm : 摆臂速度估计,编码器 16 位计数 → 速度
//   vel=16 拍窗口差分 Q8,整数差分低速偏置大所以数净位移;
//   dsum=4 拍差分之和 Q2,不 >>>2 截断以保留低速阻尼。首拍全写当前值防满幅甩臂。
//===========================================================================
module vel_est_arm (
    input              clk_50m ,
    input              rst_n   ,
    input              tick    ,   // 3ms 采样节拍
    input      [15:0]  pos_in  ,   // encoder.v 输出的 16 位位置计数
    output reg signed [15:0] vel,   // 窗口速度 Q8：计数/采样 ×256
    output reg signed [15:0] dsum   // 4 拍差分之和 Q2：4·计数/采样−4..+4
);

// 4 点 FIR 对 1 拍差分滤噪±1 LSB 抖动 → 输出仍为整数，低速偏置仍在，
// 但只用于小增益阻尼项，偏置无碍
reg signed [15:0] d1, d2, d3;
reg signed [31:0] fir_sum;

// 窗口缓冲：16 个历史位置
reg [15:0] pos_hist [0:15];
reg [3:0]  wptr;        // 下一个写入位置pos_hist[wptr] 为最旧值
reg        first;
integer    i;

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        first   <= 1'b1;
        wptr    <= 4'd0;
        vel     <= 16'sd0;
        dsum    <= 16'sd0;
        d1      <= 16'sd0;
        d2      <= 16'sd0;
        d3      <= 16'sd0;
        fir_sum <= 32'sd0;
    end else if (tick) begin
        if (first) begin
            for (i = 0; i < 16; i = i + 1) pos_hist[i] <= pos_in;
            first <= 1'b0;
            vel   <= 16'sd0;
        end else begin
            // 16 拍净位移16 位回绕安全：|真实位移| < 32768 时减法自动取模
            vel <= ($signed(pos_in) - $signed(pos_hist[wptr])) <<< 4;
            // 1 拍差分 FIR4快速阻尼项用；上拍位置在 wptr−1 槽
            d1      <= $signed(pos_in) - $signed(pos_hist[wptr - 4'd1]);
            d2      <= d1;
            d3      <= d2;
            fir_sum <= $signed(pos_in) - $signed(pos_hist[wptr - 4'd1]) + d1 + d2 + d3;
            dsum    <= fir_sum;   // 不 >>>2：保留低速阻尼见文件头说明
            pos_hist[wptr] <= pos_in;
            wptr <= wptr + 4'd1;
        end
    end
end

endmodule
