`timescale 1ns / 1ps
//===========================================================================
// kb_input.v —— 键盘输入缓冲/命令层
//
//   流程：先按 S4/S8 预告方向 → 输数字（角度）→ S16 确定执行；
//   S15 退格；S14 重置（清输入+取消未走完的旋转）；S12 匀速转独立开关。
//
//   键码：S1=0 S2=1 S3=2 S4=3 / S5=4 S6=5 S7=6 S8=7 / S9=8 S10=9 S11=10 S12=11
//         S13=12 S14=13 S15=14 S16=15
//   角度换算：一圈=1100 编码器计数=360° → 计数=度×55/18（90°=275=SW2 一步）
//===========================================================================
module kb_input(
    input            clk_50m,
    input            rst_n,
    input      [3:0] key_code,    // 0~15 = S1~S16
    input            key_valid,   // 按下确认 1 拍
    output reg [9:0] kb_buf,         // 当前输入角度 0~999（3 位上限，第 4 位忽略）
    output reg [1:0] dir,         // 00=未武装 01=顺时针 10=逆时针
    output reg       goto_valid,  // S16 确定：1 拍执行脉冲
    output reg [15:0] goto_cnts,  // 带符号编码器计数（正=CW 负=CCW）
    output reg       spin_toggle, // S12：1 拍翻转脉冲（匀速转 开/关）
    output reg       cancel,      // S14：1 拍（清输入+取消未走完旋转）
    output reg [3:0] last_key     // 最近键码（串口回显）
);

reg [3:0] d;
reg signed [15:0] cnts_mag;

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        kb_buf         <= 10'd0;
        dir         <= 2'd0;
        goto_valid  <= 1'b0;
        goto_cnts   <= 16'sd0;
        spin_toggle <= 1'b0;
        cancel      <= 1'b0;
        last_key    <= 4'd0;
    end else begin
        goto_valid  <= 1'b0;   // 1 拍脉冲默认清零
        spin_toggle <= 1'b0;
        cancel      <= 1'b0;
        if (key_valid) begin
            last_key <= key_code;
            // 数字键译码：S1=1 S2=2 S3=3 S5=4 S6=5 S7=6 S9=7 S10=8 S11=9 S13=0
            case (key_code)
                4'd0 : d = 4'd1;
                4'd1 : d = 4'd2;
                4'd2 : d = 4'd3;
                4'd4 : d = 4'd4;
                4'd5 : d = 4'd5;
                4'd6 : d = 4'd6;
                4'd8 : d = 4'd7;
                4'd9 : d = 4'd8;
                4'd10: d = 4'd9;
                4'd12: d = 4'd0;
                default: d = 4'd15;   // 非数字标记
            endcase
            if (d != 4'd15 && kb_buf < 10'd100)        // 数字：3 位上限（已有 3 位则忽略本次）
                kb_buf <= kb_buf * 10'd10 + {6'd0, d};
            else if (key_code == 4'd14)             // S15 退格：丢最后一位
                kb_buf <= kb_buf / 10'd10;
            else if (key_code == 4'd13) begin       // S14 重置：清输入+方向，并取消未走完的旋转
                kb_buf    <= 10'd0;
                dir    <= 2'd0;
                cancel <= 1'b1;
            end else if (key_code == 4'd3)          // S4 预告顺时针（只记方向，不动臂）
                dir <= 2'd1;
            else if (key_code == 4'd7)              // S8 预告逆时针
                dir <= 2'd2;
            else if (key_code == 4'd15) begin       // S16 确定：按武装方向转 kb_buf 度
                if (dir != 2'd0 && kb_buf != 10'd0) begin
                    cnts_mag  = $signed({7'd0, kb_buf}) * 16'sd55 / 16'sd18;  // 度→计数(≤3052)
                    goto_cnts <= (dir == 2'd1) ? cnts_mag : -cnts_mag;
                    goto_valid<= 1'b1;
                    kb_buf       <= 10'd0;             // 执行后清数字，方向保持（连续同向输不同角度）
                end
            end else if (key_code == 4'd11)         // S12 匀速转 开/关（独立开关，与方向/数字无关）
                spin_toggle <= 1'b1;
        end
    end
end

endmodule
