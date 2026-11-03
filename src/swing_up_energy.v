`timescale 1ns / 1ps
//===========================================================================
// swing_up_energy : Åström–Furuta 能量泵起摆,3ms 拍
//   能量法把摆杆从挂垂泵到竖直:缺能时按势能缺口×速度×cosα 注能,
//   能量够了停泵;摆杆静止时向挂垂方向踢摆;臂速超限全力刹车。
//   每拍走 8 级流水线,参数以 top.v 例化处为准。
//===========================================================================
module swing_up_energy #(
    parameter signed [15:0] U_MAX   = 16'sd4080,   // 起摆输出限幅100% 油门
    parameter signed [15:0] V_MIN   = 16'sd2,      // 启动阈值：|vel| < 2 计数/拍 ≈ 0.98 rad/s
    parameter signed [15:0] E_KICK  = 16'sd600,    // 踢摆能量阈值Q12：dE < −600 即 E < ~0.15 J 才踢
    parameter signed [15:0] E_GATE  = 16'sd1205,   // 泵门控阈值 Q12本板 E_est 偏高，top.v 覆盖调大放行到顶
    parameter signed [15:0] U_START = 16'sd1024,   // 起始段限幅
    parameter signed [15:0] E_CAP   = 16'sd600,    // 小油门缺口阈值 Q12,幅度<90° 时限 U_START
    parameter signed [15:0] U_MID   = 16'sd4080,   // 中段限幅
    parameter signed [15:0] E_MID   = 16'sd181,    // 中段阈值 Q12,dE_lim<−E_MID 用 U_MID
    parameter signed [15:0] K_DAMP  = 16'sd1000,   // 臂速阻尼增益,arm_vel Q8:u_damp=K_DAMP·arm_vel>>8
    parameter signed [15:0] ARM_GATE = 16'sd1024,  // 臂速门控 Q8,|arm_vel| 低于此才泵/踢
    parameter signed [15:0] KICK1   = 16'sd0,     // 首踢限幅,实验开关,0=禁用
    parameter signed [15:0] SWING1  = 16'sd0,     // 首摆限幅,实验开关,0=禁用
    parameter integer       K_SHIFT = 13,          // 连续律增益移位,泵能增益
    parameter integer       PULSE_TICKS = 60,      // 脉冲泵,窗口内每半摆只泵 N 拍,0=连续泵
    parameter integer       UPPER_PUMP = 0,        // 1=全范围泵,0=仅下半圈窗口泵
    parameter integer       BANG = 0,               // 满油门泵,0=禁用
    parameter integer       KICK_ALL = 0,           // 全圈踢摆,1=上半圈转折点也踢
    parameter integer       KE_ON = 0,              // 1=恢复含动能原能量律,默认 0=势能缺口泵
    parameter signed [15:0] ALPHA_OFF = 16'sd84    // 挂垂零点偏移,默认值存档,实际值由 top.v 运行时传入
)(
    input                     clk_50m ,
    input                     rst_n   ,
    input                     tick    ,
    input      [11:0]         theta_hold,
    input                     theta_valid,
    input  signed [31:0]      alpha_off,   // 挂垂零点偏移，运行时输入SW4 标定反推；写死会在电位器漂移后能量基准出错
    input  signed [15:0]      vel     ,
    input  signed [15:0]      arm_vel ,   // 摆臂窗速 Q8vel_est_arm，臂速阻尼用
    output reg  signed [15:0] u_sw
);

// cos 表：cosi·4° × 2048，Q11
// 打包成一维向量iverilog 不支持未打包 localparam 数组，取元素用 [idx*16 +: 16]
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

// ---- 组合：角度换算theta·5520>>16 − alpha_off；范围 −84~+261，无除法----
// alpha_deg ∈ [−84, 261]：非负即已在 [0,360，负数只需 +360 折一次
wire signed [31:0] alpha_deg = ({20'd0, theta_hold} * 32'sd5520 >>> 16) - alpha_off;
wire        [31:0] alpha_mod = (alpha_deg < 0) ? alpha_deg + 32'sd360 : alpha_deg[31:0];

// ---- 流水线寄存器tick 周期 3ms = 15 万拍，7 拍延迟无感----
reg [6:0]         idx;       // 拍0：查表索引
reg signed [15:0] vel_abs;   // 拍0：|vel|
reg               side;      // 拍0：摆杆侧别alpha_mod<180 = 挂垂正侧
reg signed [15:0] cos_q;     // 拍1：cosα Q11
reg signed [15:0] c1;        // 拍2：2048·1+cosα
reg signed [31:0] p1;        // 拍2：vel²
reg signed [31:0] dE_lim;    // 拍3：全能量差 E_total−E_ref 限幅 ±4096只当泵门控
reg signed [31:0] pd_lim;    // 拍3：势能缺口幅度 −E_ref−PE ∈[−1205,0]泵幅用
reg signed [47:0] q1;        // 拍4：dE·vel
reg signed [63:0] u_cont;    // 拍5：连续律输出仅下半圈
reg signed [63:0] u_kick;    // 拍5：踢摆输出
reg signed [31:0] u_brake;   // 拍5：臂刹车 −K_DAMP·arm_vel>>8仅上半圈
reg signed [63:0] u_sel;     // 拍6：泵/刹车/踢摆/静息 四选一纯选择，不夹数学
reg [11:0]        pulse_cnt; // 脉冲泵计时PULSE_TICKS>0 时使用
reg               pulse_en;  // 拍6 辅助：本拍是否允许泵寄存器输出，不占 u_sel 路径
reg               kick1_pending; // 首踢是否已武装能量出踢摆区重新武装、爆发结束清除
reg               swing1_on;     // 首摆进行中pending 期 vel≥V_MIN 置位、首次转折清除

// 拍3 组合输入:p1/c1 均为拍2寄存器输出,只一级乘法+加法。
// dE_cur=全能量差只当泵门控,E≥E_ref 停泵;pd_cur=势能缺口,泵幅用,无比例衰减不卡平台。
wire signed [31:0] pe_term = (c1 * 32'sd1205) >>> 12;   // E_ref−PE_Q12
wire signed [31:0] dE_cur  = ((p1 * 32'sd4541) >>> 12) - pe_term;
wire signed [31:0] pd_cur  = -pe_term;
wire signed [31:0] dE_gate = $signed({{16{E_GATE[15]}}, E_GATE}) - 32'sd1205;   // 门控基准：dE_lim ≥ dE_gate ⟺ E_est ≥ E_GATE
wire signed [31:0] e_cap_s = $signed({{16{E_CAP[15]}}, E_CAP});
wire signed [31:0] e_mid_s = $signed({{16{E_MID[15]}}, E_MID});
// 三段限幅：E<0.148J幅度<90° 水平起始段 U_START压开场漂转；0.148~0.25J 中段
// U_MID=全油门每摆注能最大→甩动次数少；E≥0.25J 冲刺段回 U_MAX。E_CAP=E_KICK
// 保证踢摆区 ⊂ cap 区，踢摆永远 ≤1024。
wire signed [63:0] u_cap   = (dE_lim < -e_cap_s) ? $signed({{48{U_START[15]}}, U_START}) :
                             (dE_lim < -e_mid_s) ? $signed({{48{U_MID[15]}},   U_MID})   :
                                                   $signed({{48{U_MAX[15]}},   U_MAX});

// 拍5 的组合输入：臂速取负与 u_brake 共用同一 DSP 乘法，仅输入加一级取反
wire signed [15:0] arm_vel_neg = -arm_vel;

// 拍6 的组合输入：臂速绝对值门控比较用，纯比较无乘法，不进流水线
wire signed [15:0] arm_vel_abs = (arm_vel < 0) ? -arm_vel : arm_vel;

// ---- 首踢/首摆限幅实验开关，KICK1/SWING1=0 时无效----
// 踢摆条件与拍6踢摆分支完全一致vel<V_MIN、缺能、臂<4rad/s——首踢爆发=此条件连续的整段
wire signed [31:0] e_kick_s  = $signed({{16{E_KICK[15]}}, E_KICK});
wire kick_cond  = (vel_abs < V_MIN) && (dE_lim < -e_kick_s) && (arm_vel_abs < 16'sd512);
wire kick1_active  = (KICK1  != 16'sd0) && kick1_pending && kick_cond;
wire swing1_active = (SWING1 != 16'sd0) && swing1_on;
wire signed [63:0] kick1_cap  = $signed({{48{KICK1[15]}},  KICK1});
wire signed [63:0] swing1_cap = $signed({{48{SWING1[15]}}, SWING1});
wire signed [63:0] cap_eff    = kick1_active  ? kick1_cap  :
                                swing1_active ? swing1_cap : u_cap;

// 满油门泵BANG=1：幅度取 U_MAX、方向取 u_cont 符号；|u_cont|≤64 时
// 方向无定义窗口边缘 dE·vel·cos≈0，维持原小输出防抖
wire signed [63:0] u_bang = (u_cont >  64'sd64) ? $signed({{48{U_MAX[15]}}, U_MAX}) :
                            (u_cont < -64'sd64) ? $signed({{48{-U_MAX[15]}}, -U_MAX}) : u_cont;

// 脉冲泵窗口与拍6 泵分支同条件：在窗内计拍，出窗清零——每半摆一发脉冲
wire pump_win = (vel_abs >= V_MIN) && (cos_q > 16'sd512) &&
                (arm_vel_abs < ARM_GATE) && (dE_lim < dE_gate);

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
        pulse_cnt <= 12'd0;
        pulse_en  <= 1'b0;
        kick1_pending <= 1'b1;
        swing1_on     <= 1'b0;
    end else if (tick && theta_valid) begin
        // 拍0:角度换算+索引
        idx     <= alpha_mod[8:2];
        vel_abs <= (vel < 0) ? -vel : vel;
        side    <= (alpha_mod < 32'sd180);   // 正侧,0~180° 为 1
        // 拍1:cos 查表,索引须取 89−idx
        cos_q   <= $signed(COS_TAB[((7'd89 - idx)*16) +: 16]);
        // 拍2:c1 与 vel²;泵幅用势能缺口,含动能的原律会卡幅度平台
        c1      <= 16'sd2048 + cos_q;
        p1      <= vel * vel;
        // 拍3:dE 与势能缺口,限幅 ±4096 防溢出
        dE_lim  <= (dE_cur >  32'sd4096) ?  32'sd4096 :
                   (dE_cur < -32'sd4096) ? -32'sd4096 : dE_cur;
        pd_lim  <= (pe_term > 32'sd4096) ? -32'sd4096 : pd_cur;
        // 拍4:连续律第一级乘法
        q1      <= $signed({{16{pd_lim[31]}}, pd_lim}) * vel;
        // 拍5:第二级乘法/踢摆项。对称踢:转向点向挂垂推,两侧都往挂垂踢;
        // 上半圈输出 0 让摆杆自由冲顶
        u_cont  <= (q1 * cos_q) >>> K_SHIFT;   // 连续律,抽能限半圈的约束在拍6选择层
        // 全圈踢摆:KICK_ALL=1 时上半圈转折点也踢,方向仍向挂垂推
        u_kick  <= (cos_q > 0) ? (side ?  ((U_MAX * cos_q) >>> 11)
                                      : -((U_MAX * cos_q) >>> 11))
                               : (KICK_ALL ? (side ? -((U_MAX * -cos_q) >>> 11)
                                                   :  ((U_MAX * -cos_q) >>> 11))
                                           : 64'sd0);
        // 臂刹车 Q8:−K_DAMP·arm_vel>>8,与臂速反向;臂空转时离心项抽摆杆能量,必须刹住
        u_brake <= (K_DAMP * arm_vel_neg) >>> 8;
        // 拍6 选择层:dE≥0 能量够了停泵;dE<0 缺能→贴底泵能+上半圈泵 UPPER_PUMP;
        // 摆杆静→踢摆;臂速门控全程管泵/踢,超限刹臂
        u_sel   <= (dE_lim >= dE_gate) ? ((arm_vel_abs < ARM_GATE) ? 64'sd0 : u_brake) :
                   (arm_vel_abs < ARM_GATE) ?
                     ((vel_abs >= V_MIN) ? ((cos_q > 16'sd512) ?
                                            ((PULSE_TICKS == 0 || pulse_en) ? (BANG ? u_bang : u_cont) : 64'sd0)
                                                                          : (UPPER_PUMP ? u_cont : u_brake)) :
                                           (((dE_lim < -E_KICK) && (arm_vel_abs < 16'sd512)) ? u_kick : 64'sd0))
                   : u_brake;
        // 首踢武装:能量出踢摆区重新武装;首摆跟踪:动起即开始、首次转折即结束
        if (dE_lim >= -e_kick_s)                  kick1_pending <= 1'b1;
        else if (!kick_cond)                      kick1_pending <= 1'b0;
        if (kick1_pending && (vel_abs >= V_MIN))  swing1_on <= 1'b1;
        else if (vel_abs < V_MIN)                 swing1_on <= 1'b0;
        // 脉冲泵计时：进窗计拍、出窗清零每半摆一发 PULSE_TICKS 拍脉冲
        if (pump_win) pulse_cnt <= (pulse_cnt < 12'hFFF) ? pulse_cnt + 1'b1 : pulse_cnt;
        else          pulse_cnt <= 12'd0;
        pulse_en <= (PULSE_TICKS == 0) ? 1'b1 : (pump_win && (pulse_cnt < PULSE_TICKS[11:0]));
        // 拍7:总限幅输出,符号已按 SIGN=−1 方向定好,不要整体取负
        u_sw <= (u_sel >  cap_eff) ?  cap_eff[15:0] :
                (u_sel < -cap_eff) ? -cap_eff[15:0] : $signed(u_sel[15:0]);
    end
end

endmodule
