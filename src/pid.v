//=============================================================================
// pid.v : 双环 PID 控制器,内环 6ms 角度环 + 外环 50ms 位置环
//   内环把摆杆角度误差变成电机占空比,外环把臂位误差变成角度偏置;
//   增益均为 Q8 定点数,256=1.0,|误差|超 CAPTURE_RANGE 断电保护。
//   调参:先 KP_A 立杆,加 KD_A 压下坠,静偏加 KI_A;内环稳后再开外环 KP_P。
//=============================================================================
module pid #(
    //====== 内环摆杆角度环，每拍======
    parameter signed [15:0] KP_A    = 16'sd200, // 角度增益:误差每 1 计数加 KP_A/256 占空比
    parameter [7:0]  BASE_PWM       = 8'd2,    // 基础力度:误差非零时恒定底力,带误差方向
    parameter signed [15:0] KI_A    = 16'sd8,     // 角度环积分增益 Q8,静偏小步 1/2/4 试
    parameter signed [15:0] KD_A    = 16'sd1100,   // 角度环微分增益 Q8,预判下坠
    parameter signed [15:0] D_ANG_LIMIT = 16'sd800, // D项单拍限幅,计数:只削噪声尖峰
    parameter signed [15:0] ARM_DAMP = -16'sd500,  // 臂速度阻尼 Q8,负号是物理定标,-64 起步
    //====== 外环横杆位置环，慢，回中======
    parameter signed [15:0] KP_P    = 16'sd90,   // 位置环比例增益 Q8,0=外环关,开时 32 起
    parameter signed [15:0] KI_P    = 16'sd0,   // 位置环积分增益 Q8,方向未定标前关
    parameter signed [15:0] KD_P    = 16'sd1300,  // 位置环微分增益 Q8,对位置误差 50ms 差分,0=关
    parameter signed [15:0] D_P_LIMIT = 16'sd27, // D项单拍限幅,偏置计数:防速度尖峰劫持偏置
    parameter signed [15:0] POS_STEP      = 16'sd275, // SW2 步进量,计数:90°=1100/4=275,方向反了取负
    parameter signed [15:0] POS_STEP_RATE = 16'sd10,  // SW2 步进斜坡速率,计数/外环拍
    parameter [21:0] TICK_P_MAX     = 22'd2_499_999, // 外环节拍上限:2_500_000 周期=50ms,独立计数不锁相内环
    parameter signed [15:0] THETA_BIAS_MAX = 16'sd300, // 角度偏置限幅,计数
    parameter signed [15:0] POS_DEADBAND  = 16'sd2,  // 位置死区,计数:回中不必精确到 0
    //====== 公共 ======
    parameter [3:0]  SHIFT         = 4'd8,      // 定点移位位数,与 Q8 对应
    parameter signed [15:0] CAPTURE_RANGE = 16'sd600, // 断电阈值:|误差|>600 才断电
    parameter signed [15:0] ERR_DEADBAND = 16'sd6,    // 摆杆角度死区,计数:死区内全零
    parameter signed [15:0] DUTY_DEADZONE = 16'sd4,    // 占空比死区:|输出|≤4 不出方向
    parameter [7:0]  DUTY_MAX       = 8'd255,   // 内环输出幅度限幅,255=100%
    parameter DIR_INVERT  = 1'b0,   // 1 = 电机方向取反,接线/安装方向不对时置 1
    parameter ENC_INVERT  = 1'b1,   // 1=编码器方向取反,臂外冲时注意此线
    parameter OUTER_INVERT = 1'b1,  // 1=摆杆倾倒方向与横杆运动方向匹配,上板验证过
    parameter BIAS_RAMP_EN = 1'b0  // 1=外环偏置 10 拍斜坡,0=一步到位
)(
    input            clk_50m  ,
    input            rst_n    ,
    input            en       ,   // 使能：1 运行，0 停止积分/偏置清零、电机停
    input            calib    ,   // SW4 校准脉冲：锁存当前臂位为零点与 theta_ref 同拍
    input            step     ,   // SW2 步进脉冲平衡中按一次=横杆目标 +90°；top.v 用 swing_key[1]&&!swing_mode&&pid_en 生成
    input            step_dir ,   // step 有效时 1=反向 -90°SW3；0=原行为
    input            spin     ,   // SW4 匀速转圈使能电平：每外环拍 pos_goal +POS_STEP_RATE，与斜坡同速→臂恒速旋转
    input            spin_dir ,   // S4/S8 联动转圈方向1=反向；0=原默认方向
    input            goto_valid , // 键盘 S16 确定 1 拍脉冲目标角度旋转,走既有斜坡
    input signed [15:0] goto_cnts, // 带符号编码器计数正=CW 负=CCW;度×55/18
    input            goto_cancel, // 键盘 S14 重置 1 拍取消未走完旋转,停在当前位置

    //====== 内环摆杆角度输入 ======
    input  [11:0]    theta_out ,  // ADC 实时角度 0~40950~360°，与 adc.v 输出同名
    input  [11:0]    theta_ref ,  // 目标角度 0~4095SW4 一键校准的竖直读数

    //====== 外环横杆位置输入 ======
    input  [15:0]    pos       ,  // 编码器累计位置，与 encoder.v 输出同名

    //====== 输出给 motor_ctrl ======
    output reg [7:0] duty_out     ,  // 占空比 0~255，与 motor_ctrl.v 输入同名
    output reg [1:0] motor_a_sel  ,  // 00:停 01:正转 10:反转，与 motor_ctrl.v 输入同名

    //====== 调试观测接 GAO 或串口，均带符号 ======
    output [15:0]    dbg_err_ang , // 摆杆角度误差
    output [15:0]    dbg_pos_ref , // 外环角度偏置 theta_bias
    output [15:0]    dbg_err_pos , // 横杆离零位的位置误差
    output [15:0]    dbg_pwm       // 合成输出 duty_cmd带符号
);

//------------------ 内环节拍 6ms167Hz ------------------
reg [18:0] tick_cnt;
reg        tick_v;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        tick_cnt <= 19'd0;
        tick_v   <= 1'b0;
    end else if(tick_cnt == 19'd299_999) begin   // 内环 6ms3ms 版卡抖震，废弃
        tick_cnt <= 19'd0;
        tick_v   <= 1'b1;
    end else begin
        tick_cnt <= tick_cnt + 19'd1;
        tick_v   <= 1'b0;
    end
end

//------------------ 外环节拍 50ms独立计数，不锁相内环 ------------------
reg [21:0] tick_cnt_p;
reg        tick_p;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        tick_cnt_p <= 22'd0;
        tick_p     <= 1'b0;
    end else if(tick_cnt_p == TICK_P_MAX) begin
        tick_cnt_p <= 22'd0;
        tick_p     <= 1'b1;
    end else begin
        tick_cnt_p <= tick_cnt_p + 22'd1;
        tick_p     <= 1'b0;
    end
end

//------------------ 零位与步进：SW4 校准锁存臂位；SW2 按一次=目标 +90°斜坡匀速走位 ------------------
reg [15:0] pos_home;                 // 当前目标位每外环拍向 pos_goal 靠拢 RATE 步
reg signed [15:0] pos_goal;          // 步进终点SW2 累加/SW4 校准归零
wire signed [15:0] step_diff = pos_goal - $signed(pos_home);  // 距终点差带方向,16 位回绕与编码器一致

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        pos_home <= 16'd0;
        pos_goal <= 16'sd0;
    end else if(calib) begin
        pos_home <= pos;             // SW4：双归零取消未走完的步进
        pos_goal <= pos;
    end else if(goto_cancel) begin
        pos_goal <= pos_home;        // 键盘 S14：目标回当前位斜坡差清零=立即停
    end else if(goto_valid) begin
        pos_goal <= pos_goal + goto_cnts;   // 键盘 S16：目标 += ±计数16 位回绕与编码器一致,走既有斜坡
    end else if(step) begin
        pos_goal <= pos_goal + (step_dir ? -POS_STEP : POS_STEP);   // SW2：终点再 +90°每次按下累加；SW3step_dir=1=反向 -90°
    end else if(tick_p) begin
        if(spin)
            pos_goal <= pos_goal + (spin_dir ? -POS_STEP_RATE : POS_STEP_RATE);   // S4/S8 联动方向
        if(step_diff != 16'sd0) begin
            // 斜坡：每外环拍匀速靠拢 RATE 步温和转动、到点自动停、新位置稳定
            if(step_diff > POS_STEP_RATE)
                pos_home <= pos_home + POS_STEP_RATE;
            else if(step_diff < -POS_STEP_RATE)
                pos_home <= pos_home - POS_STEP_RATE;
            else
                pos_home <= pos_goal;
        end
    end
end

//------------------ 臂速度估计：6ms 差分 ------------------
reg [15:0] pos_last;
reg signed [15:0] vel_arm;   // 单位：计数/6ms，带方向

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        pos_last <= 16'd0;
        vel_arm  <= 16'sd0;
    end else if(tick_v) begin
        pos_last <= pos;
        vel_arm  <= $signed(pos) - $signed(pos_last);  // 6ms 位移
    end
end

// 乘前显式扩到 32 位:16 位×16 位乘法按操作数位宽16截断,乘积超 ±32767 翻号,
// 大误差救杆时 P/D 出力方向会错乱
wire signed [31:0] vel_arm_32 = $signed({{16{vel_arm[15]}}, vel_arm});

//------------------ 外环：横杆位置环慢，输出角度偏置 theta_bias ------------------
localparam signed [31:0] I_P_LIMIT = {THETA_BIAS_MAX, {SHIFT{1'b0}}}; // 积分限幅 = THETA_BIAS_MAX << SHIFT
localparam signed [31:0] D_P_LIMIT_Q = {D_P_LIMIT, {SHIFT{1'b0}}}; // D项限幅换算到Q8 = 27<<8与 I_P_LIMIT 同式

// 位置误差 = 横杆离零位的偏移；ENC_INVERT=1 时编码器计数方向取反
wire signed [15:0] err_pos_raw = ENC_INVERT ? ($signed(pos_home) - $signed(pos))
                                            : ($signed(pos) - $signed(pos_home));
// 悬臂位置死区：|误差|≤POS_DEADBAND 视为 0回中不必精确到 0
wire signed [15:0] err_pos_next = (err_pos_raw <= POS_DEADBAND && err_pos_raw >= -POS_DEADBAND) ?
                                  16'sd0 : err_pos_raw;
// 原始误差死区前扩到 32 位：D 项专用，死区边界进出不再造合成台阶
wire signed [31:0] err_pos_raw_32 = $signed({{16{err_pos_raw[15]}}, err_pos_raw});
// 乘前显式扩到 32 位同上,防 16×16 截断翻号
wire signed [31:0] err_pos_32 = $signed({{16{err_pos_next[15]}}, err_pos_next});

reg signed [15:0] err_pos;
reg signed [15:0] err_pos_raw_last;   // 原始位置误差上一拍D 项 50ms 差分窗,死区前
reg signed [31:0] i_pos;
reg signed [15:0] theta_bias;

reg signed [31:0] p_pos_new, i_pos_new, d_pos_new, pos_out_p;
reg signed [15:0] bias_goal_tmp;   // 本拍外环目标阻塞临时量，仅块内用

// 偏置斜坡内插:第 cnt 小步位置 = ramp_start + gap×cnt/10×6554>>>16 ≈ /10,误差 0.006%
reg signed [15:0] ramp_start_q;
reg signed [31:0] bias_gap_q;
reg [3:0]         ramp_cnt;

wire signed [31:0] cnt_mul     = $signed({28'd0, ramp_cnt}) * 32'sd6554;   // cnt×6554 ≤ 9×6554
wire signed [31:0] ramp_frac   = (bias_gap_q * cnt_mul) >>> 16;            // gap×cnt/10
wire signed [31:0] bias_interp = $signed({{16{ramp_start_q[15]}}, ramp_start_q}) + ramp_frac;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        err_pos      <= 16'sd0;
        err_pos_raw_last <= 16'sd0;
        i_pos        <= 32'sd0;
        theta_bias   <= 16'sd0;
    end else if(!en) begin
        i_pos        <= 32'sd0;
        theta_bias   <= 16'sd0;
        ramp_cnt     <= 4'd0;      // 停摆即停斜坡
        err_pos_raw_last <= err_pos_raw;   // 失能期间跟随原始误差，防重使能瞬间 D 项吃到陈旧差分猛踢
    end else if(tick_p) begin
        err_pos      <= err_pos_next;
        err_pos_raw_last <= err_pos_raw;   // 原始误差上一拍50ms 差分窗，死区前

        p_pos_new = err_pos_32 * KP_P;
        i_pos_new = i_pos + err_pos_32 * KI_P;
        d_pos_new = (err_pos_raw_32 - $signed({{16{err_pos_raw_last[15]}}, err_pos_raw_last})) * KD_P;   // D 项：对死区前原始误差 1 拍50ms差分
        if(d_pos_new > D_P_LIMIT_Q)       d_pos_new = D_P_LIMIT_Q;     // 单项限幅：速度尖峰不得劫持整个偏置
        else if(d_pos_new < -D_P_LIMIT_Q) d_pos_new = -D_P_LIMIT_Q;
        if(i_pos_new > I_P_LIMIT)       i_pos_new = I_P_LIMIT;
        else if(i_pos_new < -I_P_LIMIT) i_pos_new = -I_P_LIMIT;
        i_pos <= i_pos_new;

        pos_out_p = (p_pos_new + i_pos_new + d_pos_new) >>> SHIFT;
        if(!in_window) begin
            i_pos      <= 32'sd0;
            theta_bias <= 16'sd0;   // 出窗直接归零，不走斜坡安全
            ramp_cnt   <= 4'd0;
        end else begin
            // 横杆往 + 偏 → 偏置让摆杆目标往能让横杆回零的方向斜OUTER_INVERT 决定
            if(pos_out_p > THETA_BIAS_MAX)
                bias_goal_tmp = OUTER_INVERT ? THETA_BIAS_MAX : -THETA_BIAS_MAX;
            else if(pos_out_p < -THETA_BIAS_MAX)
                bias_goal_tmp = OUTER_INVERT ? -THETA_BIAS_MAX : THETA_BIAS_MAX;
            else
                bias_goal_tmp = OUTER_INVERT ? pos_out_p[15:0] : -pos_out_p[15:0];

            if(BIAS_RAMP_EN) begin
                // 10 拍斜坡60ms 步进会被内环 D 放大成"抽风"：
                // 锁起点与总差，本拍即走第一小步≈1/10，余 9 拍由 tick_v 内插补完
                ramp_start_q <= theta_bias;
                bias_gap_q   <= $signed({{16{bias_goal_tmp[15]}}, bias_goal_tmp}) - $signed({{16{theta_bias[15]}}, theta_bias});
                theta_bias   <= theta_bias + ((($signed({{16{bias_goal_tmp[15]}}, bias_goal_tmp}) - $signed({{16{theta_bias[15]}}, theta_bias})) * 32'sd6554) >>> 16);
                ramp_cnt     <= 4'd1;
            end else begin
                theta_bias <= bias_goal_tmp;   // 关斜坡 = 原一步到位行为
            end
        end
    end else if(tick_v) begin
        // 偏置斜坡内插：每 6ms 从旧目标滑向新目标 1/10治"抽风"；ramp_cnt=0 不动作
        if(ramp_cnt != 4'd0) begin
            theta_bias <= bias_interp[15:0];
            ramp_cnt   <= ramp_cnt + 4'd1;
        end
    end
end

//------------------ 内环：摆杆角度环每拍，输出占空比+方向 ------------------
localparam signed [31:0] I_ANG_LIMIT = {DUTY_MAX, {SHIFT{1'b0}}}; // 积分限幅 = DUTY_MAX << SHIFT与输出限幅一致，防积分卷绕
localparam signed [31:0] D_ANG_LIMIT_Q = {D_ANG_LIMIT, {SHIFT{1'b0}}}; // D项限幅换算到Q8 = 800<<8

// 有效角度目标 = SW4 竖直校准值 + 外环偏置摆杆斜一点，横杆慢慢回中
wire signed [16:0] theta_ref_eff = $signed({5'd0, theta_ref}) + $signed({theta_bias[15], theta_bias});

// 角度误差:直接带符号相减,不做回绕。本机械下垂=0 压盲区,读数跨 0/4095 抖动时
// "最短角"回绕会把误差折成负值,起摆时摆杆被反方向推,所以必须直接相减。
wire signed [16:0] err_ang_full = theta_ref_eff - $signed({5'd0, theta_out});
wire signed [15:0] err_ang_next = err_ang_full[15:0];  // 范围 ±4095+400，16 位足够

// 平衡死区：误差落在 ±ERR_DEADBAND 内视为 0不再纠偏
wire signed [15:0] err_ang_use = (err_ang_next <= ERR_DEADBAND && err_ang_next >= -ERR_DEADBAND) ?
                                 16'sd0 : err_ang_next;
// 乘前显式扩到 32 位同上,防 16×16 截断翻号
wire signed [31:0] err_ang_32  = $signed({{16{err_ang_use[15]}}, err_ang_use});
wire signed [31:0] derr_ang_32 = err_ang_32 - err_ang_last2;  // D 项差分2 拍=12ms 窗

// 断电窗口：|误差| > CAPTURE_RANGE≈53° 时电机断电
wire in_window = (err_ang_next <= CAPTURE_RANGE) && (err_ang_next >= -CAPTURE_RANGE);

reg signed [15:0] err_ang, err_ang_last, err_ang_last2;   // err_ang_last2：2 拍前误差D 项 12ms 差分用
reg signed [31:0] i_ang;

reg signed [31:0] p_ang_new, d_ang_new, i_ang_new, ang_sum, ang_out;
reg signed [31:0] duty_cmd, abs_out;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        err_ang      <= 16'sd0;
        err_ang_last <= 16'sd0;
        err_ang_last2 <= 16'sd0;
        i_ang        <= 32'sd0;
        duty_out     <= 8'd0;
        motor_a_sel  <= 2'b00;
    end else if(!en) begin
        i_ang       <= 32'sd0;
        duty_out    <= 8'd0;
        motor_a_sel <= 2'b00;
        // 失能期间持续跟踪角度，防止使能瞬间 D 项吃到冻结期的陈旧误差产生猛踢
        err_ang_last2 <= err_ang_last;
        err_ang_last  <= err_ang;
        err_ang       <= err_ang_use;
    end else if(tick_v) begin
        err_ang_last2 <= err_ang_last;
        err_ang_last  <= err_ang;
        err_ang       <= err_ang_use;

        // 基础力度 + 线性响应——角度斜率 KP_A和力度恒定底力 BASE_PWM两个独立旋钮
        p_ang_new = err_ang_32 * KP_A;
        if(err_ang_use != 16'sd0)
            p_ang_new = p_ang_new + ((err_ang_use > 16'sd0) ? $signed({24'd0, BASE_PWM} << 8)
                                                             : -$signed({24'd0, BASE_PWM} << 8));
        // D 项用 2 拍差分12ms 窗口+>>1 保持与原 1 拍差分相同的每秒阻尼力度传感器量化踢减半
        d_ang_new = (derr_ang_32 * KD_A) >>> 1;
        // D 项限幅：削掉单拍猛踢，持续下落时仍保持满力度
        if(d_ang_new > D_ANG_LIMIT_Q)       d_ang_new = D_ANG_LIMIT_Q;
        else if(d_ang_new < -D_ANG_LIMIT_Q) d_ang_new = -D_ANG_LIMIT_Q;
        // 防止接杆阶段积分饱和、平衡后电机仍被积分顶住不收力；误差进死区时缓慢衰减归零
        if(err_ang_use == 16'sd0)
            i_ang_new = i_ang - (i_ang >>> 8);          // 死区内积分泄漏每拍衰减约1/256
        else if(duty_cmd >= $signed({1'b0, DUTY_MAX}) || duty_cmd <= -$signed({1'b0, DUTY_MAX}))
            i_ang_new = i_ang;                          // 上拍输出已饱和：本拍不累加
        else
            i_ang_new = i_ang + err_ang_32 * KI_A;
        if(i_ang_new > I_ANG_LIMIT)       i_ang_new = I_ANG_LIMIT;
        else if(i_ang_new < -I_ANG_LIMIT) i_ang_new = -I_ANG_LIMIT;

        ang_sum = p_ang_new + d_ang_new + i_ang_new;
        ang_out = ang_sum >>> SHIFT;   // 角度环输出，单位=计数≈占空比
        // 横杆运动方向 = -ang_outOUTER_INVERT=1；臂速度阻尼直接减占空比
        duty_cmd = (OUTER_INVERT ? -ang_out : ang_out) - ((vel_arm_32 * ARM_DAMP) >>> SHIFT);
        abs_out = (duty_cmd >= 32'sd0) ? duty_cmd : -duty_cmd;

        if(!in_window) begin
            i_ang       <= 32'sd0;
            duty_out    <= 8'd0;
            motor_a_sel <= 2'b00;
        end else begin
            if(abs_out >= DUTY_MAX)
                duty_out <= DUTY_MAX;
            else
                duty_out <= abs_out[7:0];

            // 方向：跟 duty_cmd + 死区
            if(duty_cmd > DUTY_DEADZONE)
                motor_a_sel <= DIR_INVERT ? 2'b10 : 2'b01;
            else if(duty_cmd < -DUTY_DEADZONE)
                motor_a_sel <= DIR_INVERT ? 2'b01 : 2'b10;
            else
                motor_a_sel <= 2'b00;
        end
    end
end

//------------------ 调试输出 ------------------
assign dbg_err_ang = err_ang;
assign dbg_pos_ref = theta_bias;
assign dbg_err_pos = err_pos;
assign dbg_pwm     = duty_cmd[15:0];

endmodule
