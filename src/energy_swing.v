`timescale 1ns / 1ps
//===========================================================================
// energy_swing : Åström–Furuta 能量起摆,kick_swing 能量模式的泵核心,3ms 拍
//   缺能时按势能缺口×速度×cosα 注能,能量够 E_GATE 停泵;挂垂静止向挂垂踢摆;
//   臂速超 ARM_GATE 停泵刹臂。挂垂在电位器盲区,流水线每拍用保持值照跑。
//   角度换算 alpha=theta×ANGLE_SCALE_Q16>>16;E_GATE 板标:甩过头调小,卡差一截调大。
//===========================================================================
module energy_swing #(
    parameter signed [15:0] U_MAX     = 16'sd2048,  // 冲刺段/踢摆幅值限幅,新电机最优点 2048=50% duty
    parameter signed [15:0] V_MIN     = 16'sd2,     // 启动阈值,|vel|<2 计数/拍≈静止
    parameter signed [15:0] E_KICK    = 16'sd600,   // 踢摆能量阈值 Q12,dE<−E_KICK 才踢
    parameter signed [15:0] E_GATE    = 16'sd1100,  // 泵门控阈值 Q12;板标:甩过头调小,卡差一截调大
    parameter signed [15:0] E_REF     = 16'sd1205,  // 竖直能量 2·m·g·lp,Q12
    parameter signed [15:0] KE_SCALE  = 16'sd1135,  // KE=vel²·KE_SCALE/4096;6ms 拍 vel×2→KE×4,故 4541/4=1135
    parameter signed [15:0] U_START   = 16'sd2048,  // 起始段低能量限幅,本板开头力大一点
    parameter signed [15:0] E_CAP     = 16'sd600,   // 小油门缺口阈值 Q12,dE_lim<−E_CAP 限 U_START
    parameter signed [15:0] U_MID     = 16'sd4080,  // 中段限幅,每摆注能最大→甩动次数少
    parameter signed [15:0] E_MID     = 16'sd181,   // 中段阈值 Q12
    parameter signed [15:0] K_DAMP    = 16'sd1000,  // 臂速阻尼增益:u_damp=K_DAMP·arm_vel>>8
    parameter signed [15:0] ARM_GATE  = 16'sd1024,  // 臂速门控 Q8,|arm_vel| 低于此才泵/踢
    parameter signed [15:0] ARM_REST  = 16'sd128,   // 防狂转:踢摆须臂近静止<1rad/s,不泵状态臂快就刹
    parameter [9:0]         KICK_COOLDOWN = 10'd100,// 防狂转:踢完强制 100 拍冷却
    parameter integer       K_SHIFT   = 12,         // 连续律增益移位,定稿 12
    parameter integer       UPPER_PUMP= 1,          // 1=全范围泵,0=仅贴底窗口
    parameter signed [31:0] ANGLE_SCALE_Q16 = 32'sd6081, // 角度比例=180×65536/竖直读数,按 SW4 实标改
    parameter signed [15:0] ALPHA_OFF = 16'sd0,     // 挂垂零点偏移,新机械挂垂=0
    parameter integer       SWING_INVERT = 0,       // 1=输出整体取反,上板方向标定开关
    parameter integer       ARM_VEL_INVERT = 1      // 臂速方向修正:旧板编码器交叉接、本板直连,arm_vel 符号相反→刹车成正反馈;1=取反恢复旧板验证过的刹车方向
)(
    input                     clk_50m ,
    input                     rst_n   ,
    input                     tick    ,   // 3ms 控制拍kick_swing 同拍；vel 为 6ms 拍信号，隔拍重复采样
    input                     start   ,   // 1=起摆使能top 层 swing_mode；0=停止 u=0
    input                     handoff ,   // 1=PID 已接管 → 停泵停踢kick_swing 同语义
    input      [11:0]         theta_hold, // vel_est 死区保持角盲区冻结 0
    input                     theta_valid,// 角度有效盲区=0；流水线仍跑，只禁泵
    input  signed [15:0]      vel     ,   // 摆杆速度：计数/拍6ms 拍，盲区保持
    input  signed [15:0]      arm_vel ,   // 摆臂速度 Q8vel_est_arm 输出，3ms 拍
    output reg  signed [15:0] u_sw    ,   // 起摆控制量±4095 域，经 motor_drive 适配
    output      signed [15:0] dE_mon      // 全能量差 dE_limQ12，截 16 位→ 串口遥测板标 E_GATE 用
);

// cos 表,cos i·4° ×2048,Q11,90 项;打包成一维向量,取 e_idx 用 89−idx×16 位选
localparam [1439:0] COS_TAB = {
    16'sd2048  , 16'sd2043  , 16'sd2028  , 16'sd2003  , 16'sd1969  , 16'sd1924  ,
    16'sd1871  , 16'sd1808  , 16'sd1737  , 16'sd1657  , 16'sd1569  , 16'sd1473  ,
    16'sd1370  , 16'sd1261  , 16'sd1145  , 16'sd1024  , 16'sd898   , 16'sd767   ,
    16'sd633   , 16'sd495   , 16'sd356   , 16'sd214   , 16'sd71    , -16'sd70   ,
    -16'sd213  , -16'sd355  , -16'sd494  , -16'sd632  , -16'sd766  , -16'sd897  ,
    -16'sd1023 , -16'sd1144 , -16'sd1260 , -16'sd1369 , -16'sd1472 , -16'sd1568 ,
    -16'sd1656 , -16'sd1736 , -16'sd1807 , -16'sd1870 , -16'sd1923 , -16'sd1968 ,
    -16'sd2002 , -16'sd2027 , -16'sd2042 , -16'sd2047 , -16'sd2042 , -16'sd2027 ,
    -16'sd2002 , -16'sd1968 , -16'sd1923 , -16'sd1870 , -16'sd1807 , -16'sd1736 ,
    -16'sd1656 , -16'sd1568 , -16'sd1472 , -16'sd1369 , -16'sd1260 , -16'sd1144 ,
    -16'sd1023 , -16'sd897  , -16'sd766  , -16'sd632  , -16'sd494  , -16'sd355  ,
    -16'sd213  , -16'sd70   , 16'sd71    , 16'sd214   , 16'sd356   , 16'sd495   ,
    16'sd633   , 16'sd767   , 16'sd898   , 16'sd1024  , 16'sd1145  , 16'sd1261  ,
    16'sd1370  , 16'sd1473  , 16'sd1569  , 16'sd1657  , 16'sd1737  , 16'sd1808  ,
    16'sd1871  , 16'sd1924  , 16'sd1969  , 16'sd2003  , 16'sd2028  , 16'sd2043
};

// ---- 组合：角度换算theta·ANGLE_SCALE_Q16>>16 − ALPHA_OFF----
// 新机械 theta 范围 [0,4095] → alpha_deg ∈ [0, ~380]，折叠要处理 ≥360旧版只折负值
wire signed [31:0] alpha_deg = (({20'd0, theta_hold} * ANGLE_SCALE_Q16) >>> 16) - ALPHA_OFF;
wire        [31:0] alpha_mod = (alpha_deg < 32'sd0)   ? alpha_deg + 32'sd360 :
                               (alpha_deg >= 32'sd360) ? alpha_deg - 32'sd360 : alpha_deg[31:0];

// ---- 流水线寄存器tick 周期 3ms = 15 万拍，7 拍延迟无感；旧版时序修复保留----
reg [6:0]         idx;       // 拍0：查表索引
reg signed [15:0] vel_abs;   // 拍0：|vel|
reg               side;      // 拍0：摆杆侧别alpha_mod<180 = 挂垂正侧
reg signed [15:0] cos_q;     // 拍1：cosα Q11
reg signed [15:0] c1;        // 拍2：2048·1+cosα
reg signed [31:0] p1;        // 拍2：vel²
reg signed [31:0] dE_lim;    // 拍3：全能量差 E_total−E_ref 限幅 ±4096只当泵门控
reg signed [31:0] pd_lim;    // 拍3：势能缺口幅度 −E_ref−PE ∈[−E_REF,0]泵幅用
reg signed [47:0] q1;        // 拍4：dE·vel
reg signed [63:0] u_cont;    // 拍5：连续律输出
reg signed [63:0] u_kick;    // 拍5：踢摆输出
reg signed [31:0] u_brake;   // 拍5：臂刹车 −K_DAMP·arm_vel>>8
reg signed [63:0] u_sel;     // 拍6：泵/刹车/踢摆/静息 四选一纯选择，不夹数学

// 拍3 的组合输入p1/c1 均为拍2寄存器输出，只一级乘法+加法；逐字来自旧版
wire signed [31:0] pe_term = (c1 * $signed({{16{E_REF[15]}}, E_REF})) >>> 12;   // E_ref−PE_Q12
wire signed [31:0] dE_cur  = ((p1 * $signed({{16{KE_SCALE[15]}}, KE_SCALE})) >>> 12) - pe_term;
wire signed [31:0] pd_cur  = -pe_term;
wire signed [31:0] dE_gate = $signed({{16{E_GATE[15]}}, E_GATE}) - $signed({{16{E_REF[15]}}, E_REF});
wire signed [31:0] e_cap_s = $signed({{16{E_CAP[15]}}, E_CAP});
wire signed [31:0] e_mid_s = $signed({{16{E_MID[15]}}, E_MID});
wire signed [31:0] e_kick_s= $signed({{16{E_KICK[15]}}, E_KICK});
// 三段限幅逐字来自旧版：E<0.148J 起始段 U_START；中段 U_MID；冲刺段 U_MAX
wire signed [63:0] u_cap   = (dE_lim < -e_cap_s) ? $signed({{48{U_START[15]}}, U_START}) :
                             (dE_lim < -e_mid_s) ? $signed({{48{U_MID[15]}},   U_MID})   :
                                                   $signed({{48{U_MAX[15]}},   U_MAX});
// 泵 = 软比例连续律 u_cont09730 基线：u_cont = −dE·vel·cosα>>12 直接输出，
// 幅度随缺能/速度软变化上限被 u_cap 夹。注能方向 −signvel·cosα 由公式自动正确。

// 臂速方向修正见 ARM_VEL_INVERT：刹车/门控/踢摆统一用取反后的臂速
// 旧板真编码器 A/B 交叉接、本板 enc1 直接接、SIGN=-1 两板相同→符号反了，
//   刹车成正反馈=摆臂空转原因
wire signed [15:0] arm_vel_i   = ARM_VEL_INVERT ? -arm_vel : arm_vel;
// 拍5 的组合输入：臂速取负与 u_brake 共用同一 DSP 乘法，仅输入加一级取反
wire signed [15:0] arm_vel_neg = -arm_vel_i;
// 拍6 的组合输入：臂速绝对值门控比较用，纯比较无乘法，不进流水线
wire signed [15:0] arm_vel_abs = (arm_vel_i < 0) ? -arm_vel_i : arm_vel_i;

// 踢摆条件vel<V_MIN、缺能；臂速上限由 512→ARM_REST 收紧：
// 旧板踢到 4rad/s 后摩擦会把臂慢回来；新电机无摩擦，踢到阈值就永久空转=狂转
wire kick_cond  = (vel_abs < V_MIN) && (dE_lim < -e_kick_s) && (arm_vel_abs < ARM_REST);
reg  [9:0] kick_cool;                         // 踢摆冷却计数踢后强制休息，防连续单方向踢
wire       kick_ok    = kick_cond && (kick_cool == 10'd0);

// 总限幅纯选择，不进时序关键路径
wire signed [15:0] u_clamp = (u_sel >  u_cap) ?  u_cap[15:0] :
                             (u_sel < -u_cap) ? -u_cap[15:0] : $signed(u_sel[15:0]);

assign dE_mon = dE_lim[15:0];

always @(posedge clk_50m or negedge rst_n) begin
    if (!rst_n) begin
        idx     <= 7'd0;
        vel_abs <= 16'sd0;
        side    <= 1'b0;
        cos_q   <= 16'sd0;
        c1      <= 16'sd0;
        p1      <= 32'sd0;
        dE_lim  <= 32'sd0;
        pd_lim  <= 32'sd0;
        q1      <= 48'sd0;
        u_cont  <= 64'sd0;
        u_kick  <= 64'sd0;
        u_brake <= 32'sd0;
        u_sel   <= 64'sd0;
        u_sw    <= 16'sd0;
        kick_cool <= 10'd0;
    end else if (tick) begin
        // 流水线每拍照跑盲区用 vel_est 保持值；旧版 theta_valid 门控已删——
        // 新机械挂垂/底部都在盲区，流水线冻住会踢不出启动踢、掐断底部泵
        // 拍0：角度换算 + 索引alpha_mod 组合完成，此拍只寄存
        idx     <= alpha_mod[8:2];
        vel_abs <= (vel < 0) ? -vel : vel;
        side    <= (alpha_mod < 32'sd180);   // 正侧0~180°为 1
        // 拍1：cos 查表
        cos_q   <= $signed(COS_TAB[((7'd89 - idx)*16) +: 16]);
        // 拍2：c1 与 vel²各一级乘法/加法
        c1      <= 16'sd2048 + cos_q;
        p1      <= vel * vel;
        // 拍3：dE = vel²·KE_SCALE>>12 − c1·E_REF>>12，限幅 ±4096防溢出，仅保护
        dE_lim  <= (dE_cur >  32'sd4096) ?  32'sd4096 :
                   (dE_cur < -32'sd4096) ? -32'sd4096 : dE_cur;
        pd_lim  <= (pe_term > 32'sd4096) ? -32'sd4096 : pd_cur;
        // 拍4：连续律第一级乘法
        q1      <= $signed({{16{pd_lim[31]}}, pd_lim}) * vel;
        // 拍5：第二级乘法 / 踢摆项方向见旧版：向挂垂推，正侧 u>0
        u_cont  <= (q1 * cos_q) >>> K_SHIFT;
        u_kick  <= (cos_q > 0) ? (side ?  ((U_MAX * cos_q) >>> 11)
                                      : -((U_MAX * cos_q) >>> 11))
                               : 64'sd0;
        // 臂刹车Q8：−K_DAMP·arm_vel>>8，与臂速反向
        u_brake <= (K_DAMP * arm_vel_neg) >>> 8;
        // 拍6：选择层09730 基线：旧版定稿结构 + 盲区适配——盲区禁连续泵只许踢/刹；
        // 防狂转：凡不泵状态臂>ARM_REST 一律刹臂；臂速门 ARM_GATE 全程管泵/踢
        u_sel   <= (dE_lim >= dE_gate) ? ((arm_vel_abs > ARM_REST) ? u_brake : 64'sd0) :
                   (~theta_valid ? (kick_ok ? u_kick : ((arm_vel_abs > ARM_REST) ? u_brake : 64'sd0))
                                 : ((arm_vel_abs < ARM_GATE) ? ((vel_abs >= V_MIN) ? ((cos_q > 16'sd512) ? u_cont
                                                                                                               : (UPPER_PUMP ? u_cont : u_brake)) :
                                                                                          (kick_ok ? u_kick : 64'sd0))
                                                              : u_brake));
        // 踢摆冷却计数：选中踢的那拍装载 KICK_COOLDOWN，其余递减。
        // start 撤/交棒只断 u_sw 输出，冷却照走——再开时不会立刻连踢
        kick_cool <= (kick_ok && (dE_lim < dE_gate) && (arm_vel_abs < ARM_GATE) &&
                      (~theta_valid || (vel_abs < V_MIN)))
                     ? KICK_COOLDOWN
                     : ((kick_cool > 10'd0) ? kick_cool - 10'd1 : 10'd0);
        // 拍7：总限幅输出仅夹取，路径最短；start 撤/交棒 → u=0；SWING_INVERT 方向翻转
        u_sw <= (!start || handoff) ? 16'sd0 : (SWING_INVERT ? -u_clamp : u_clamp);
    end
end

endmodule
