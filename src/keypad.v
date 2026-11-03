`timescale 1ns / 1ps
//===========================================================================
// keypad.v —— HW-834 4×4 矩阵键盘扫描
//
//   原理：4 行输出（R1~R4，低有效）+ 4 列输入（C1~C4，FPGA 内部上拉）。
//   1kHz 轮扫：每 1ms 只拉低一行、读 4 列，按下的键把所在列拽成 0 →
//   行列交叉定位。一圈 4ms，键码 = 行号×4+列号（S1=0 … S16=15，与丝印一致）。
//
//   消抖：整矩阵连续 DEB_PASSES 圈（默认 5 圈=20ms）不变才确认按下。
//   防重发：同一键按住只发 1 拍 key_valid，矩阵变化（松开）后重新武装。
//   多键同按：只认优先编码的第一个键（矩阵键盘 3 键会"鬼影"，单键使用）。
//===========================================================================
module keypad #(
    parameter [31:0] SCAN_DIV   = 32'd49_999,  // 50MHz/50k = 1kHz 扫描
    parameter [4:0]  DEB_PASSES = 5'd5         // 连续 5 圈(20ms)同键才确认
)(
    input            clk_50m,
    input            rst_n,
    input      [3:0] col,        // C1~C4 列输入(内部上拉)
    output reg [3:0] row,        // R1~R4 行输出(低有效扫描)
    output reg [3:0] key_code,   // 0~15 = S1~S16(保持最近一次确认值)
    output reg       key_valid   // 按下确认 1 拍脉冲
);

reg [31:0] scan_cnt;
reg [1:0]  drv_idx;     // 当前被拉低的行(采样与行号对齐)
reg [15:0] mat;         // 4×4 键矩阵：mat[i*4+j]=1 表示第 i 行第 j 列按下
reg [15:0] mat_prev;    // 上一圈矩阵（消抖比较基准）
reg [4:0]  deb_cnt;
reg        fired;       // 本键已确认过，按住不连发

wire       scan_tick = (scan_cnt == SCAN_DIV);
wire       pass_end  = scan_tick && (drv_idx == 2'd3);    // 一圈结束(第3行采完)
reg        pass_end_d;                                    // 延迟 1 拍：等第 3 行采样写进 mat

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        scan_cnt   <= 32'd0;
        drv_idx    <= 2'd0;
        row        <= 4'b1110;     // 复位即拉低 R1(原 1111 白等一拍,且与 drv_idx=0 对齐)
        mat        <= 16'd0;
        pass_end_d <= 1'b0;
    end else begin
        pass_end_d <= pass_end;
        if (scan_tick) begin
            scan_cnt <= 32'd0;
            mat[drv_idx*4 +: 4] <= ~col;                     // 采样当前被拉低的这一行;取反:按下=1
            row      <= {row[2:0], row[3]};                  // 循环左移 1110→1101→1011→0111→1110
            drv_idx  <= drv_idx + 2'd1;
        end else begin
            scan_cnt <= scan_cnt + 32'd1;
        end
    end
end

// ---- 消抖 + 防重发（一圈一次；pass_end_d 时 mat 已含本轮全部 4 行）----
integer i;
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        mat_prev  <= 16'd0;
        deb_cnt   <= 5'd0;
        fired     <= 1'b0;
        key_code  <= 4'd0;
        key_valid <= 1'b0;
    end else begin
        key_valid <= 1'b0;      // 1 拍脉冲默认清零
        if (pass_end_d) begin
            if (mat != mat_prev) begin
                mat_prev <= mat;
                deb_cnt  <= 5'd0;
                fired    <= 1'b0;                // 矩阵变化（含松开）→ 重新武装
            end else if (mat != 16'd0 && !fired) begin
                if (deb_cnt == DEB_PASSES - 5'd1) begin
                    fired     <= 1'b1;
                    key_valid <= 1'b1;
                    // 优先编码：取最低置位（单键使用，多键只认第一个）
                    begin : prio
                        reg found;
                        found = 1'b0;
                        for (i = 0; i < 16; i = i + 1) begin
                            if (mat[i] && !found) begin
                                found    = 1'b1;
                                key_code <= 4'd15 - i[3:0];   // 键盘膜 180° 反装(丝印[1,2,3,A/4,5,6,B/7,8,9,C/0,R,#,D] 对应电气码 15..0)，翻转后丝印即语义
                            end
                        end
                    end
                end else
                    deb_cnt <= deb_cnt + 5'd1;
            end
        end
    end
end

endmodule
