//=============================================================================
// swing_up.v : 共振起摆模块,classic 回退路径,6ms 拍
//   初始踢 0.5s 把摆杆踢出盲区,三点滑窗找最高点反向泵踢;
//   摆到平衡点 ±CAP_RANGE 连续 2 采样捕获,交棒给 PID。
//   方向反了翻 SWING_INVERT;仅右顶踢异常置 DIR_RIGHT_SWAP=1。
//=============================================================================
module swing_up #(
    parameter [7:0]  SWING_PWM       = 8'd128,    // 踢力度:128=50% 占空比不全功率,怕烧电机;0~255 可调
    parameter [7:0]  INIT_KICK_TICKS = 8'd83,     // 初始踢时长:83×6ms≈0.5s跳出死区;0.3s 太短
    parameter [7:0]  TOP_KICK_TICKS  = 8'd17,     // 顶踢时长:17×6ms≈0.1s
    parameter [7:0]  SAMPLE_DIV      = 8'd7,      // 采样分频:7×6ms=42ms
    parameter signed [12:0] SWING_RANGE = 13'sd100,  // 顶踢门槛:3 采样全出竖直 ±100 才认峰治噪声假峰
    parameter signed [12:0] CAP_RANGE   = 13'sd100,  // 捕获窗:平衡点 ±100 计数≈8.8°
    parameter        DIR_RIGHT_SWAP = 1'b0,       // 1=右顶踢用与左顶踢相反的方向镜像保险
    parameter        SWING_INVERT   = 1'b0        // 全部踢方向整体取反上板定标开关
)(
    input            clk_50m ,
    input            rst_n   ,
    input            en      ,   // 起摆使能top.v:swing_mode
    input            restart ,   // SW1 重按脉冲:状态机整体复位重新起摆
    input  [11:0]    theta_out,  // 摆杆实时角度vel_est 死区保持
    input  [11:0]    theta_ref,  // SW4 校准的竖直角度起摆中心
    output reg [7:0] duty_out ,  // 0~255全功率=255
    output reg [1:0] motor_sel,  // 00 停 / 01 正 / 10 反
    output reg       capture  ,  // 捕获脉冲1 拍:top.v 切 PID 接管
    output reg       active      // 1=起摆中
);

//------------------ 6ms 节拍与 pid.v 同拍:299999/50MHz ------------------
reg [18:0] tick_cnt;
always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)                    tick_cnt <= 19'd0;
    else if(tick_cnt >= 19'd299_999) tick_cnt <= 19'd0;
    else                          tick_cnt <= tick_cnt + 19'd1;
end
wire tick_6ms = (tick_cnt == 19'd299_999);

//------------------ 仰角:直接带符号相减,不做回绕下垂−1940 在 ±2048 内,无折叠必要 ------------------
wire signed [12:0] elev = {1'b0, theta_out} - {1'b0, theta_ref};
wire [12:0] elev_abs = (elev < 0) ? -elev : elev;

//------------------ 踢方向 ------------------
// 共振起摆原理:初始踢顺向dir_init,到最高点反向推dir_pump。SWING_INVERT 整体翻。
wire [1:0] dir_init  = SWING_INVERT ? 2'b10 : 2'b01;
wire [1:0] dir_pump  = SWING_INVERT ? 2'b01 : 2'b10;
wire [1:0] dir_left  = dir_pump;
wire [1:0] dir_right = DIR_RIGHT_SWAP ? dir_init : dir_pump;

//------------------ 状态机 ------------------
localparam [1:0] S_KICK  = 2'd0,   // 踢初始踢/顶踢
                 S_JUDGE = 2'd1,   // 滑窗找最高点
                 S_CAPT  = 2'd2;   // 已捕获等 en 拉低

reg [1:0]  state;
reg [1:0]  kick_dir;      // 当前踢方向
reg [7:0]  kick_cnt;      // 踢拍数
reg        init_kick;     // 1=当前踢是初始踢用 INIT_KICK_TICKS
reg [7:0]  sample_cnt;    // 采样分频
reg [3:0]  samples_seen;  // 有效采样数≥3 才允许捕获/极值判定,治滑窗初值 0 假在窗
reg signed [12:0] e0, e1, e2;   // 三点滑窗0=最新
reg        captured;      // 捕获锁存等 en 拉低
reg        cap_cnt;       // 捕获连续计数

// 本次采样后的滑窗值组合,先移位再判断
wire signed [12:0] n_e0 = elev;
wire signed [12:0] n_e1 = e0;
wire signed [12:0] n_e2 = e1;

wire signed [12:0] r_pos =  SWING_RANGE;
wire signed [12:0] r_neg = -SWING_RANGE;

// 捕获:平衡点 ±CAP_RANGE
wire in_cap = (elev_abs < CAP_RANGE);
// 最高点=三点滑窗的峰中间采样比两边都大;往右摆读数变大,峰即最高点
wire top_left  = (n_e0 < r_neg) && (n_e1 < r_neg) && (n_e2 < r_neg)
              && (n_e1 > n_e0) && (n_e1 > n_e2);   // 左侧最高点竖直下方
wire top_right = (n_e0 > r_pos) && (n_e1 > r_pos) && (n_e2 > r_pos)
              && (n_e1 > n_e0) && (n_e1 > n_e2);   // 右侧最高点过冲侧,镜像保险

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        state        <= S_KICK;
        kick_dir     <= dir_init;
        kick_cnt     <= 8'd0;
        init_kick    <= 1'b1;
        sample_cnt   <= 8'd0;
        samples_seen <= 4'd0;
        e0 <= 13'sd0; e1 <= 13'sd0; e2 <= 13'sd0;
        captured     <= 1'b0;
        cap_cnt      <= 1'b0;
        duty_out     <= 8'd0;
        motor_sel    <= 2'b00;
        capture      <= 1'b0;
        active       <= 1'b0;
    end
    else if(!en || restart) begin
        // SW2 停止/SW1 重按:全部归零,重新初始踢
        state        <= S_KICK;
        kick_dir     <= dir_init;
        kick_cnt     <= 8'd0;
        init_kick    <= 1'b1;
        sample_cnt   <= 8'd0;
        samples_seen <= 4'd0;
        e0 <= 13'sd0; e1 <= 13'sd0; e2 <= 13'sd0;
        captured     <= 1'b0;
        cap_cnt      <= 1'b0;
        duty_out     <= 8'd0;
        motor_sel    <= 2'b00;
        capture      <= 1'b0;
        active       <= 1'b0;
    end
    else begin
        capture <= 1'b0;   // 捕获脉冲只维持 1 拍
        // 采样节拍所有状态共用;S_KICK 期间也采样,踢中进窗照样捕获
        if(tick_6ms && sample_cnt >= SAMPLE_DIV - 8'd1) begin
            sample_cnt <= 8'd0;
            e0 <= elev; e1 <= e0; e2 <= e1;
            if(samples_seen < 4'd15) samples_seen <= samples_seen + 4'd1;
            // 捕获:采样数≥3 后,连续 2 采样进平衡点 ±100 → 交棒 PID
            if(!captured && samples_seen >= 4'd3) begin
                if(in_cap) begin
                    if(cap_cnt) begin
                        capture  <= 1'b1;
                        captured <= 1'b1;
                        active   <= 1'b0;
                        state    <= S_CAPT;
                        cap_cnt  <= 1'b0;
                    end else
                        cap_cnt <= 1'b1;
                end else
                    cap_cnt <= 1'b0;
            end
        end
        else if(tick_6ms) begin
            sample_cnt <= sample_cnt + 8'd1;
        end
        // 状态机主体捕获优先级最高
        if(captured || capture) begin
            duty_out  <= 8'd0;
            motor_sel <= 2'b00;
            state     <= S_CAPT;
        end else case(state)
            S_KICK: begin
                duty_out  <= SWING_PWM;   // 踢力度默认 50% 占空比
                motor_sel <= kick_dir;
                active    <= 1'b1;
                if(tick_6ms) begin
                    // 初始踢用 INIT_KICK_TICKS,顶踢用 TOP_KICK_TICKS
                    if(kick_cnt >= (init_kick ? INIT_KICK_TICKS : TOP_KICK_TICKS) - 8'd1) begin
                        kick_cnt  <= 8'd0;
                        init_kick <= 1'b0;
                        state     <= S_JUDGE;
                    end else
                        kick_cnt <= kick_cnt + 8'd1;
                end
            end

            S_JUDGE: begin
                duty_out  <= 8'd0;
                motor_sel <= 2'b00;
                active    <= 1'b1;
                // 最高点检测滑窗在采样点更新,此处组合值即最新窗口
                if(top_left) begin
                    state     <= S_KICK;
                    kick_dir  <= dir_left;
                    kick_cnt  <= 8'd0;
                    init_kick <= 1'b0;
                end
                else if(top_right) begin
                    state     <= S_KICK;
                    kick_dir  <= dir_right;
                    kick_cnt  <= 8'd0;
                    init_kick <= 1'b0;
                end
            end

            S_CAPT: begin
                duty_out  <= 8'd0;
                motor_sel <= 2'b00;
                active    <= 1'b0;
            end

            default: state <= S_KICK;
        endcase
    end
end

endmodule
