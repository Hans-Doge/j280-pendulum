`timescale 1ns / 1ps
//===========================================================================
// kick_swing : 谐振踢起摆模块,3ms 拍
//   三点滑窗找摆动顶点,每半摆在最高点反向踢一脚,越荡越高;
//   摆杆进中心窗且慢穿窗时交棒给 PID,交棒后大摆过顶发脉冲重锁臂零位。
//   方向反了置 KICK_FLIP=1;ENERGY_MODE=1 时改由 energy_swing 能量律驱动。
//===========================================================================
module kick_swing #(
    parameter signed [15:0] KICK        = 16'sd4080,  // 踢脉冲幅值,满油门
    parameter [9:0]         KICK_TICKS  = 10'd44,     // 标准踢正/反各 44 拍×3ms≈132ms
    parameter [9:0]         WAKE_TICKS  = 10'd67,     // 唤醒踢正/反各 67 拍≈200ms,静止起摆第一脚
    parameter integer       SMP_TICKS   = 3,          // 采样间隔 3 拍≈9ms
    parameter [9:0]         LOCK_TICKS  = 10'd67,     // 踢完锁 67 拍≈201ms 不再触发
    parameter [10:0]        STALL_TICKS = 11'd1000,   // 3s 没检到最高点 → 自动补唤醒踢
    parameter signed [12:0] CENTER_HALF = 13'sd30,    // 中心半宽,计数:进窗即接 PID
    parameter signed [12:0] NEAR_HALF   = 13'sd60,    // 软踢带,计数:极值距竖直≤60 改软踢
    parameter signed [12:0] RELOCK_MIN  = 13'sd300,   // 大摆重锁门槛≈26°,小摆不再重锁
    parameter signed [15:0] KICK_NEAR   = 16'sd816,   // 软踢幅值 25%
    parameter signed [12:0] MICRO_HALF  = 13'sd100,   // 微踢带,计数:贴窗改微踢
    parameter signed [15:0] KICK_MICRO  = 16'sd272,   // 微踢幅值 1/12
    parameter [9:0]         STABLE_TICKS= 10'd300,    // 连续在窗 300 拍≈0.9s → 冻结臂中心
    parameter [11:0]        HANG_HALF  = 12'd40,      // 挂垂带半宽,计数:下垂盲区冻结值落此带
    parameter [9:0]         WAIT_TICKS = 10'd100,     // 挂垂等静止拍数≈300ms 再唤醒踢
    parameter [9:0]         WAKE2_TICKS= 10'd60,      // 第2脚踢长上限钳位
    parameter [11:0]        KICK2_SCALE = 12'd282,    // 第2脚闭合增益:ticks=apex_dist×SCALE/4096
    parameter [12:0]        KICK2_MIN  = 13'd500,     // 第2脚门槛≈44°,更近撤火
    parameter [11:0]        SLOW_THRESH = 12'd4,      // 交棒速度门槛:|Δθ/3ms|≤4 才算到顶停住
    parameter [11:0]        SWING_TIMEOUT= 12'd2000,  // 起摆超时 2000 拍≈6s → S_RETRY 重试
    parameter integer       KICK_FLIP   = 0,          // 1 = 踢向整体取反,上板方向反了就用
    parameter integer       ENERGY_MODE = 0           // 1=判断态由 energy_swing 连续能量律驱动;0=原闭合踢
)(
    input                    clk_50m ,
    input                    rst_n   ,
    input                    tick    ,   // 3ms 控制拍adc 输出
    input                    start   ,   // 运行电平：0=急停交棒后由 top 层保活，不再拉死
    input      [11:0]        theta   ,   // 摆杆角度theta_hold
    input                    theta_valid, // 角度有效盲区/死区外
    input      [11:0]        cal_up  ,   // SW4 标定竖直读数3080 默认
    input                    handoff ,   // 过顶重锁：1=PID 已接管pid_en && !swing_mode
    input  signed [15:0]      vel     ,   // 摆杆速度6ms 拍→ energy_swing
    input  signed [15:0]      arm_vel ,   // 摆臂速度 Q8vel_est_arm→ energy_swing 臂速门控/刹车
    output reg signed [15:0] u_sw        // 送入 ctrl_fsm 的起摆控制量SWING 态生效
    ,
    output                    top4        // 1 = 摆杆已在中心区间runstate 4→ top 层 force_bal 切 PID 接管
    ,
    output reg                calib_p     // 过顶重锁：1 拍锁中心脉冲首次进窗=交棒拍；交棒后=每次摆动极值
);

// ---- runstate 编码 ----
localparam [5:0] S_STOP = 6'd0,
                 S_JUDGE= 6'd1,
                 S_L21  = 6'd21, S_L22 = 6'd22, S_L23 = 6'd23, S_L24 = 6'd24,
                 S_R31  = 6'd31, S_R32 = 6'd32, S_R33 = 6'd33, S_R34 = 6'd34,
                 S_TOP4 = 6'd4,
                 S_WAIT = 6'd5,   // 挂垂等静止300ms再唤醒踢
                 S_RETRY= 6'd6;   // 起摆超时重试:停踢等衰减到挂垂再唤醒踢途中慢穿窗直接交 PID

reg [5:0] state;

assign top4 = (state == S_TOP4);   // 进中心区间即拉高PID 接管信号；掉出中心自动回低、踢摆继续

// ---- 能量法ENERGY_MODE=1 时 S_JUDGE 由它驱动；唤醒踢/交棒/防停摆壳子不动----
wire signed [15:0] en_u;
energy_swing #(
    .U_START(16'sd2048)   // 开头力大一点旧板最优点 1024;新电机+臂刹车下上调
) es_inst(
    .clk_50m    (clk_50m),
    .rst_n      (rst_n),
    .tick       (tick),
    .start      (start && !handoff),
    .handoff    (handoff),
    .theta_hold (theta),
    .theta_valid(theta_valid),
    .vel        (vel),
    .arm_vel    (arm_vel),
    .u_sw       (en_u),
    .dE_mon     ()
);

// ---- 区间边界13bit 带符号，防 cal_up 附近越界回绕----
wire signed [12:0] L = $signed({1'b0, cal_up}) - CENTER_HALF;
wire signed [12:0] R = $signed({1'b0, cal_up}) + CENTER_HALF;
wire signed [12:0] th = $signed({1'b0, theta});
wire               in_left   = (th <= L);
wire               in_right  = (th >= R);
wire               in_center = (th > L) && (th < R);
wire               hang_band = (theta <= HANG_HALF) || (theta >= (12'd4095 - HANG_HALF));   // 挂垂带下垂盲区两端冻结读数
wire [11:0]        dth        = (theta > th_prev) ? (theta - th_prev) : (th_prev - theta);  // |Δθ/拍|
wire               slow_cross = (dth <= SLOW_THRESH);   // 慢穿窗=近顶停住交棒前提

// ---- 三点滑窗 ----
reg [11:0] p1, p2, p3;      // 旧 → 新：p1 最旧、p3 最新
reg [1:0]  win_cnt;         // 已采点数0~3
reg [7:0]  smp_cnt;         // 采样间隔计数SMP_TICKS 拍一点

// 窗口内三点同侧/极值判定拍组合逻辑
wire [12:0] P1 = $signed({1'b0, p1});
wire [12:0] P2 = $signed({1'b0, p2});
wire [12:0] P3 = $signed({1'b0, p3});
wire        all_left  = (P1 <= L) && (P2 <= L) && (P3 <= L);
wire        all_right = (P1 >= R) && (P2 >= R) && (P3 >= R);
wire        mid_max   = (P2 > P1) && (P2 > P3);   // 左侧最高点
wire        mid_min   = (P2 < P1) && (P2 < P3);   // 右侧最高点
wire signed [12:0] cal_s     = $signed({1'b0, cal_up});   // 竖直读数13bit 带符号,与 P2 同宽
wire [12:0]        apex_dist = (P2 > cal_s) ? (P2 - cal_s) : (cal_s - P2);  // 极值点距竖直
wire               near_top  = (apex_dist <= NEAR_HALF);   // 近竖直 → 本脚软踢
wire               near_micro = (apex_dist <= MICRO_HALF); // 贴窗 → 本脚微踢优先于软踢

wire signed [15:0] kick_pos = KICK_FLIP ? -KICK :  KICK;   // 向左pwm 正
wire signed [15:0] kick_neg = KICK_FLIP ?  KICK : -KICK;   // 向右pwm 负
wire               wake_flip = wake && wake_dir;            // 唤醒踢本趟方向取反臂漂移交替相抵
wire signed [15:0] kick_pos_near = KICK_FLIP ? -KICK_NEAR :  KICK_NEAR;  // 软踢幅值
wire signed [15:0] kick_neg_near = KICK_FLIP ?  KICK_NEAR : -KICK_NEAR;
wire signed [15:0] kick_pos_micro = KICK_FLIP ? -KICK_MICRO :  KICK_MICRO;  // 微踢幅值
wire signed [15:0] kick_neg_micro = KICK_FLIP ?  KICK_MICRO : -KICK_MICRO;

reg [9:0]  cnt;          // 延时/锁计数最大 LOCK_TICKS
reg        wake;         // 1 = 本趟 21-24 为唤醒踢加长版
reg        wake_dir;     // 唤醒踢方向交替:每次唤醒踢翻向,整趟臂漂移翻号→两次相抵治新电机自由轮狂转
reg        kick2;        // 第2脚待发：唤醒踢后第一个极值用加长踢WAKE2_TICKS
reg [9:0]  kick2_len;    // 第2脚闭合长度:第一顶点实测距顶换算的踢长取代固定 WAKE2_TICKS
reg [11:0] th_prev;      // 上一 3ms 拍的摆杆读数算 |Δθ/拍|
reg        kick_soft;    // 本趟踢用软幅值极值近竖直置 1，踢完清
reg        kick_micro;   // 本趟踢用微幅值极值贴窗置 1，优先于软踢
reg [9:0]  stable_cnt;   // 交棒后连续在窗计数达 STABLE_TICKS → 冻结臂中心
reg        calib_frozen; // 已稳定冻结：不再过顶重锁pid 死守冻结中心
reg [10:0] stall_cnt;    // 判断态无最高点计时达 STALL_TICKS → 补唤醒踢
reg [11:0] swing_t;      // 起摆超时计时:仅前交棒活跃态累计挂垂等待/交棒后清零
wire [9:0] kick_len = wake ? WAKE_TICKS : kick2_len;   // 所有顶点踢统一闭合长度三层踢档废弃,治满油门44拍近顶过泵=空转一圈
wire [25:0] k2_mul = apex_dist * KICK2_SCALE;                                 // 闭合换算13b×12b 无符号乘,DSP 综合
wire [13:0] k2_raw = k2_mul[25:12];                                           // >>12
wire [9:0]  k2_calc = (k2_raw > {4'd0, WAKE2_TICKS}) ? WAKE2_TICKS : k2_raw[9:0];   // 上限钳位

// ---- 主状态机 ----
always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        state         <= S_STOP;
        u_sw          <= 16'sd0;
        calib_p       <= 1'b0;
        cnt           <= 10'd0;
        kick_soft     <= 1'b0;
        kick_micro    <= 1'b0;
        stable_cnt    <= 10'd0;
        calib_frozen  <= 1'b0;
        wake      <= 1'b0;
        wake_dir  <= 1'b0;    // 方向交替复位
        kick2     <= 1'b0;
        kick2_len <= 10'd0;   // 闭合长度复位
        th_prev   <= 12'd0;
        stall_cnt <= 11'd0;
        swing_t   <= 12'd0;
        smp_cnt   <= 8'd0;
        win_cnt   <= 2'd0;
        p1        <= 12'd0; p2 <= 12'd0; p3 <= 12'd0;
    end else if (tick) begin
        th_prev <= theta;                     // 每拍更新算 |Δθ/拍|
        // 起摆超时计时:仅前交棒活跃态累计S_WAIT/S_RETRY/S_STOP/S_TOP4 或已交棒→清零
        if (!handoff && start && state != S_WAIT && state != S_RETRY && state != S_STOP && state != S_TOP4)
            swing_t <= swing_t + 12'd1;
        else
            swing_t <= 12'd0;
        if (!start) begin                     // 急停任意态
            state         <= S_STOP;
            u_sw          <= 16'sd0;
            calib_p       <= 1'b0;
            kick_soft     <= 1'b0;
            kick_micro    <= 1'b0;
            stable_cnt    <= 10'd0;
            calib_frozen  <= 1'b0;
            kick2         <= 1'b0;
            kick2_len     <= 10'd0;
        end else if (theta_valid && in_center && slow_cross) begin
            // 任意态慢穿窗即交棒带速度冲过窗不硬接
            state <= S_TOP4;
            u_sw  <= 16'sd0;
            // 首次进窗交棒拍发 1 拍锁中心;交棒后连续在窗 STABLE_TICKS 拍发冻结锁
            calib_p <= !handoff;
            if (handoff) begin
                if (!calib_frozen && (stable_cnt >= STABLE_TICKS - 10'd1)) begin
                    calib_frozen <= 1'b1;
                    calib_p      <= 1'b1;   // 冻结拍=最后一次锁中心
                end else
                    stable_cnt <= stable_cnt + 10'd1;
            end else begin
                stable_cnt    <= 10'd0;
                calib_frozen  <= 1'b0;
            end
        end else if (swing_t >= SWING_TIMEOUT - 12'd1) begin
            // 起摆超时:停踢进 S_RETRY 等静止自动重试
            state       <= S_RETRY;
            u_sw        <= 16'sd0;
            cnt         <= 10'd0;
            wake        <= 1'b0;
            kick2       <= 1'b0;
            kick_soft   <= 1'b0;
            kick_micro  <= 1'b0;
            win_cnt     <= 2'd0;
            stall_cnt   <= 11'd0;
            smp_cnt     <= 8'd0;
            p1 <= 12'd0; p2 <= 12'd0; p3 <= 12'd0;
        end else case (state)
            S_STOP: begin
                if (theta_valid && in_center) begin
                    // SW1 时摆杆已在中心手动倒立标定后→ 不踢唤醒踢，直接进 4 交 PID
                    state <= S_TOP4;
                end else if (hang_band) begin
                    // 入口分路:挂垂带盲区冻结读数→ 先等 300ms 静止再唤醒踢
                    state <= S_WAIT;
                    cnt   <= 10'd0;
                    kick2 <= 1'b0;
                end else begin
                    // 入口分路:摆杆在动不在挂垂带→ 不白踢唤醒踢,直进三点泵能接管
                    state     <= S_JUDGE;
                    u_sw      <= 16'sd0;
                    win_cnt   <= 2'd0;
                    stall_cnt <= 11'd0;
                    kick2     <= 1'b0;
                    smp_cnt   <= 8'd0;                       // 清旧采样窗,防残留三点误踢
                    p1 <= 12'd0; p2 <= 12'd0; p3 <= 12'd0;
                end
            end
            S_WAIT: begin
                // 挂垂等静止:动→三点泵能;静止满→唤醒踢
                u_sw <= 16'sd0;
                if (!hang_band) begin
                    state     <= S_JUDGE;
                    win_cnt   <= 2'd0;
                    stall_cnt <= 11'd0;
                    kick2     <= 1'b0;
                    smp_cnt   <= 8'd0;                       // 清旧采样窗,防残留三点误踢
                    p1 <= 12'd0; p2 <= 12'd0; p3 <= 12'd0;
                end else if (cnt >= WAIT_TICKS - 10'd1) begin
                    u_sw       <= kick_pos;
                    state      <= S_L21;
                    wake       <= 1'b1;
                    cnt        <= 10'd0;
                    kick_soft  <= 1'b0;   // 唤醒踢永远全油门
                    kick_micro <= 1'b0;
                end else
                    cnt <= cnt + 10'd1;
            end
            S_RETRY: begin
                // 起摆超时重试:不泵不踢等衰减;挂稳补唤醒踢,慢穿窗直接交 PID
                u_sw <= 16'sd0;
                if (hang_band) begin
                    if (cnt >= WAIT_TICKS - 10'd1) begin
                        u_sw       <= kick_pos;
                        state      <= S_L21;
                        wake       <= 1'b1;
                        cnt        <= 10'd0;
                        kick_soft  <= 1'b0;
                        kick_micro <= 1'b0;
                    end else
                        cnt <= cnt + 10'd1;
                end else
                    cnt <= 10'd0;
            end
            S_JUDGE: begin
                u_sw    <= (ENERGY_MODE && !handoff) ? en_u : 16'sd0;   // 能量模式:判断态由能量律驱动交棒后停
                calib_p <= 1'b0;   // 默认 0，仅当拍检出极值才置 11 拍脉冲
                if (theta_valid && in_center && slow_cross) begin
                    // 进中心区间 → 启摆结束慢穿窗才接,与上方同门
                    state <= S_TOP4;
                end else if (stall_cnt >= STALL_TICKS) begin
                    if (handoff) begin
                        // 交棒后:不补踢,重置计数继续监视过顶照常锁中心
                        stall_cnt <= 11'd0;
                    end else if (hang_band) begin
                        // 停摆补踢也先等静止:挂垂带 → S_WAIT 再唤醒踢与入口统一
                        state <= S_WAIT;
                        cnt   <= 10'd0;
                        kick2 <= 1'b0;
                    end else begin
                        // 3s 没检到任何最高点停摆/幅度太小/老撞盲区→ 补一脚唤醒踢
                        state     <= S_L21;
                        wake      <= 1'b1;
                        cnt       <= 10'd0;
                        kick_soft <= 1'b0;   // 唤醒踢永远全油门
                        kick_micro <= 1'b0;
                    end
                end else begin
                    // 采样：每 SMP_TICKS 拍采一点；盲区theta_valid=0整窗作废
                    if (smp_cnt >= SMP_TICKS - 1) begin
                        smp_cnt <= 8'd0;
                        if (theta_valid) begin
                            // 判定在移位之前：win_cnt 已满 3 时，旧窗P1/P2/P3即完整有效窗
                            if (win_cnt == 2'd3) begin
                                if (all_left  && mid_max) begin      // 左侧最高
                                    if (handoff) begin
                                        // 交棒后仅大摆≥RELOCK_MIN过顶重锁,小摆/冻结后不锁
                                        if (!calib_frozen && (apex_dist >= RELOCK_MIN)) calib_p <= 1'b1;
                                    end else if (!ENERGY_MODE) begin   // 能量模式:顶点不踢能量律已接管
                                        state     <= S_R31; cnt <= 10'd0;   // 起摆期:向右踢
                                        kick_soft  <= 1'b0;   // 顶点踢统一闭合长度
                                        kick_micro <= 1'b0;
                                        kick2_len  <= k2_calc;   // 每脚按实测距顶闭合减速停在窗
                                    end
                                end
                                else if (all_right && mid_min) begin // 右侧最高
                                    if (handoff) begin
                                        if (!calib_frozen && (apex_dist >= RELOCK_MIN)) calib_p <= 1'b1;
                                    end else if (!ENERGY_MODE) begin   // 能量模式:顶点不踢
                                        state     <= S_L21; cnt <= 10'd0;   // 起摆期:向左踢
                                        kick_soft  <= 1'b0;   // 同上
                                        kick_micro <= 1'b0;
                                        kick2_len  <= k2_calc;   // 同上
                                    end
                                end
                            end
                            p1 <= p2; p2 <= p3; p3 <= theta;   // 滑窗前移
                            if (win_cnt < 2'd3) win_cnt <= win_cnt + 2'd1;
                        end else
                            win_cnt <= 2'd0;   // 盲区样本不进窗，且冲掉旧窗
                    end else
                        smp_cnt <= smp_cnt + 8'd1;
                    stall_cnt <= stall_cnt + 11'd1;
                end
            end
            S_L21: begin u_sw <= wake_flip ? kick_neg : (kick_micro ? kick_pos_micro : (kick_soft ? kick_pos_near : kick_pos)); cnt <= 10'd0; state <= S_L22; end   // 向左 pwm+微踢/软踢/全油门;唤醒踢方向交替
            S_L22: begin if (cnt >= kick_len - 10'd1) begin state <= S_L23; cnt <= 10'd0; end else cnt <= cnt + 10'd1; end
            S_L23: begin u_sw <= wake_flip ? kick_pos : (kick_micro ? kick_neg_micro : (kick_soft ? kick_neg_near : kick_neg)); if (cnt >= kick_len - 10'd1) begin state <= S_L24; cnt <= 10'd0; end else cnt <= cnt + 10'd1; end   // 向右 pwm−横杆回位
            S_L24: begin u_sw <= 16'sd0; if (cnt >= LOCK_TICKS - 10'd1) begin state <= S_JUDGE; win_cnt <= 2'd0; stall_cnt <= 11'd0; kick2 <= wake; wake_dir <= wake ? ~wake_dir : wake_dir; wake <= 1'b0; kick_soft <= 1'b0; kick_micro <= 1'b0; end else cnt <= cnt + 10'd1; end   // 唤醒踢结束→第2脚待发;wake_dir 翻向整趟臂漂移相抵
            S_R31: begin u_sw <= kick_micro ? kick_neg_micro : (kick_soft ? kick_neg_near : kick_neg); cnt <= 10'd0; state <= S_R32; end   // 向右 pwm−微踢/软踢/全油门
            S_R32: begin if (cnt >= kick_len - 10'd1) begin state <= S_R33; cnt <= 10'd0; end else cnt <= cnt + 10'd1; end   // kick_len第2脚可走右路
            S_R33: begin u_sw <= kick_micro ? kick_pos_micro : (kick_soft ? kick_pos_near : kick_pos); if (cnt >= kick_len - 10'd1) begin state <= S_R34; cnt <= 10'd0; end else cnt <= cnt + 10'd1; end   // 向左 pwm+横杆回位
            S_R34: begin u_sw <= 16'sd0; if (cnt >= LOCK_TICKS - 10'd1) begin state <= S_JUDGE; win_cnt <= 2'd0; stall_cnt <= 11'd0; kick2 <= 1'b0; kick_soft <= 1'b0; kick_micro <= 1'b0; end else cnt <= cnt + 10'd1; end
            S_TOP4: begin
                u_sw    <= 16'sd0;   // 启摆结束；PID 后续接在这里
                calib_p <= 1'b0;     // 过顶锁是 1 拍脉冲,滞窗保持 0
                // 掉出中心 → 回判断重摆
                if (theta_valid && !in_center) begin
                    state     <= S_JUDGE;
                    win_cnt   <= 2'd0;
                    smp_cnt   <= 8'd0;
                    stall_cnt <= 11'd0;
                    stable_cnt <= 10'd0;   // 出窗清零稳定计数冻结标志不动
                    p1 <= 12'd0; p2 <= 12'd0; p3 <= 12'd0;
                end
            end
            default: state <= S_STOP;
        endcase
    end
end

endmodule
