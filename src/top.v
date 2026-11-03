//=============================================================================
// top.v : 倒立摆顶层
//   SW1 起摆,能量泵把摆杆泵到竖直附近,顶点判据交棒给 PID 平衡;
//   SW2/SW3 步进,SW4 转圈,4×4 键盘调参,OLED 显示,串口 66Hz 遥测。
//   基本条件:theta 12 位 0~4095=345°,竖直≈1940,theta_ref 固定 1925。
//=============================================================================
module top(

    input			clk_50m 	,	//时钟输入
    input			rst_n   	,	//复位输入

	input	[3:0]	key_sw		,	//SW1、2、3、4、5按键
	output	[3:0]	kb_row		,	//HW-834 矩阵键盘行输出低有效扫描
	input	[3:0]	kb_col		,	//矩阵键盘列输入内部上拉

	output			oled_scl	,	//OLED I2C 时钟V18,H2 针32
	inout			oled_sda	,	//OLED I2C 数据V17,H2 针34;开漏+上拉

	output			uart_tx		,	//串口发送
	input			uart_rx		,	//串口接收

    // ADC WDD35D4 接口
    output			adc_clk		,	// ADC采样时钟输出
    output			adc_oe_n	,	// 输出使能低有效

    output			adc_stby_n	,	// 休眠低有效
    input	[9:0]	adc_data_in	,	// D9~D0并行数据
    
	//TB6612
	input			enc1_a		,	//编码器输出的A相
    input			enc1_b		,	//编码器输出的B相
	input			enc2_a		,	//编码器输出的A相
    input			enc2_b		,	//编码器输出的B相
	
	//MG531
	output			AN1			,	//电机控制信号IN1
    output			AN2			,	//电机控制信号IN2
    output			PWMA		,	//PWM输出
	
    output			BN1			,	//电机控制信号IN1
    output			BN2			,   //电机控制信号IN2
    output			PWMB		    //PWM输出
    
    );
	


    reg pid_en;
    reg swing_mode;              // 1=自动起摆中捕获后清 0，切 PID 接管
    wire [7:0] swing_duty;
    wire [1:0] swing_sel;
    wire       swing_capture;    // 捕获脉冲：连续 2 次采样进 |theta-theta_ref|<SWING_RANGE
    reg        swing_rst;        // SW1 按下时发 1 拍复位脉冲给 swing_up状态机拉回 S_IDLE 重新初始踢

    reg [11:0] theta_ref_reg;    // 目标角度寄存器：上电锁定固定值，不再用 SW4 标定

    always@(posedge clk_50m or negedge rst_n) begin
      if(!rst_n) theta_ref_reg <= 12'd1925;   // 固定目标角 1925竖直≈1940 附近；要调就调这个，泵顶 alpha_off 公式自动跟随
    end

    // SWING_SEL 选起摆方案:1=能量泵现在用;0=classic 共振踢回退。到顶信号同路交棒:PID 接管calib 同拍重锁臂零位
    localparam SWING_SEL = 1'b1;

    // 能量泵/顺滑单踢/臂速/顶点判据共用的 3ms 节拍classic 路径仍 6ms 拍
    reg [17:0] tick3_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)                    tick3_cnt <= 18'd0;
        else if(tick3_cnt >= 18'd149_999) tick3_cnt <= 18'd0;
        else                          tick3_cnt <= tick3_cnt + 18'd1;
    end
    wire tick_3ms = (tick3_cnt == 18'd149_999);

    wire signed [15:0] u_kick;               // 能量泵输出名字沿用原 kick_swing 输出，下游 mux/pid/串口不动
    wire [7:0]         kick_duty;
    wire [1:0]         kick_sel;
    // 以下信号提前显式声明原来在模块内隐式声明，综合器报 EX3638
    wire [11:0] theta_hold;                 // vel_est 死区保持角6ms 拍；SWING_SEL=0 共振踢路径用
    wire        theta_valid;                // 角度有效标志
    wire [11:0] theta_out;                  // adc 原始角度vel_est/pid/OLED/串口共用
    wire [15:0] enc_pos1, enc_pos2;         // 编码器 1/2 累计位置pid 用 enc_pos1，串口发两路
    // 能量泵配套:3ms 拍专用 vel_est;臂速喂泵前取反本板编码器直连。
    // E_GATE>E_REF 在本板永不关、到顶不停注能 → 交棒用顶点判据而非驻留判据。
    wire [11:0] pump_theta_hold;
    wire        pump_theta_valid;
    wire signed [15:0] pump_vel;
    vel_est pump_vel_est(
        .clk_50m(clk_50m), .rst_n(rst_n), .tick(tick_3ms),
        .theta_in(theta_out), .theta_hold(pump_theta_hold),
        .theta_valid(pump_theta_valid), .vel(pump_vel)
    );
    wire signed [15:0] arm_vel_n = -arm_vel;   // 本板编码器直连，喂泵前取反
    // 挂垂零点偏移:alpha_off = theta_ref·5520>>16 − 180,泵顶对齐 PID 目标角并随 theta_ref 自动跟随
    wire signed [31:0] pump_alpha_off = (({20'd0, theta_ref_reg} * 32'sd5520) >>> 16) - 32'sd180;
    swing_up_energy #(
        .UPPER_PUMP (1),
        .K_SHIFT    (12),
        .PULSE_TICKS(0),
        .U_MAX      (16'sd3072),
        .E_MID      (16'sd120),
        .E_GATE     (16'sd1442),
        .U_START    (16'sd1408)
    ) pump_inst(
        .clk_50m   (clk_50m), .rst_n(rst_n), .tick(tick_3ms),
        .theta_hold(pump_theta_hold), .theta_valid(pump_theta_valid),
        .alpha_off (pump_alpha_off),             // 泵顶对齐 PID 目标角由 theta_ref 反推
        .vel       (pump_vel), .arm_vel(arm_vel_n),
        .u_sw      (u_kick)
    );
    //---- 顺滑单踢 ----
    // 挂垂在盲区 theta_valid 恒 0、能量泵不动 → 不依赖 theta_valid:等 150ms 静止 →
    // 单脚 200ms 渐变踢首尾各 3 拍爬升/回落,无反踢 → 静置 200ms 看结果,没动翻向再踢;停摆 3s 自动补踢。
    localparam [9:0]  WK_WAIT  = 10'd50;    // 150ms 等静止
    localparam [9:0]  WK_KICK  = 10'd67;    // 200ms 单脚踢
    localparam [9:0]  WK_LOCK  = 10'd67;    // 200ms 踢完静置
    localparam [10:0] WK_STALL = 11'd1000;  // 3s 停摆 → 补踢
    localparam [15:0] WK_AMP  = 16'd2040;   // 踢幅 50% 满油门要调就调这个
    localparam [3:0]  WK_RAMP = 4'd3;       // 首尾渐变拍数1=无渐变
    reg [2:0]  wk_state;      // 0=空闲,1=等静止,2=单踢,4=静置
    reg [9:0]  wk_cnt;
    reg [10:0] wk_stall;
    reg        wk_dir;        // 每趟翻向治自由轮电机臂漂移
    reg        wk_kicked;     // 本趟起摆已踢过首踢不等 3s 停摆计时
    reg        wk_drive;      // 1=唤醒踢接管电机mux 盖过能量泵
    reg        wk_moved;      // 踢/静置期间摆杆出现过真实运动静置末判定用
    reg signed [15:0] wk_u;
    wire       wk_moving = pump_theta_valid && ((pump_vel > 16'sd3) || (pump_vel < -16'sd3));
    wire signed [15:0] wk_sign = wk_dir ? -16'sd1 : 16'sd1;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            wk_state <= 2'd0; wk_cnt <= 10'd0; wk_stall <= 11'd0;
            wk_dir <= 1'b0; wk_kicked <= 1'b0; wk_drive <= 1'b0;
            wk_moved <= 1'b0; wk_u <= 16'sd0;
        end else if (!swing_mode) begin          // 非起摆：全清挂起
            wk_state <= 2'd0; wk_cnt <= 10'd0; wk_stall <= 11'd0;
            wk_kicked <= 1'b0; wk_drive <= 1'b0; wk_moved <= 1'b0; wk_u <= 16'sd0;
        end else if (tick_3ms) begin
            case (wk_state)
            2'd0: begin                          // 空闲：动→交泵；静→首踢立即等静止、此后停摆 3s 补踢
                wk_u <= 16'sd0; wk_drive <= 1'b0;
                if (wk_moving)                    wk_stall <= 11'd0;
                else if (!wk_kicked || (wk_stall >= WK_STALL)) begin
                    wk_state <= 2'd1; wk_cnt <= 10'd0; wk_stall <= 11'd0;
                end else
                    wk_stall <= wk_stall + 11'd1;
            end
            2'd1: begin                          // 等静止 150ms：期间动→不踢直接交泵
                wk_u <= 16'sd0; wk_drive <= 1'b0;
                if (wk_moving) begin
                    wk_state <= 2'd0; wk_stall <= 11'd0;
                end else if (wk_cnt >= WK_WAIT - 10'd1) begin
                    wk_state <= 2'd2; wk_cnt <= 10'd0; wk_kicked <= 1'b1; wk_moved <= 1'b0;
                end else
                    wk_cnt <= wk_cnt + 10'd1;
            end
            2'd2: begin                          // 单脚踢 200ms：首尾渐变、中间满幅，踢满不早退
                wk_drive <= 1'b1;
                if (wk_cnt < WK_RAMP)
                    wk_u <= wk_sign * $signed((WK_AMP * (wk_cnt + 10'd1)) / WK_RAMP);
                else if (wk_cnt >= WK_KICK - WK_RAMP)
                    wk_u <= wk_sign * $signed((WK_AMP * (WK_KICK - wk_cnt)) / WK_RAMP);
                else
                    wk_u <= wk_sign * $signed(WK_AMP);
                if (wk_moving) wk_moved <= 1'b1;
                if (wk_cnt >= WK_KICK - 10'd1) begin
                    wk_state <= 3'd4; wk_cnt <= 10'd0;
                end else
                    wk_cnt <= wk_cnt + 10'd1;
            end
            default: begin                       // 3'd4 静置 200ms：动过→交泵；没动→翻向再踢
                wk_u <= 16'sd0; wk_drive <= 1'b0;
                if (wk_moving) wk_moved <= 1'b1;
                if (wk_cnt >= WK_LOCK - 10'd1) begin
                    if (wk_moved || wk_moving) begin
                        wk_state <= 2'd0; wk_stall <= 11'd0; wk_moved <= 1'b0;
                    end else begin
                        wk_dir <= ~wk_dir; wk_state <= 2'd2; wk_cnt <= 10'd0; wk_moved <= 1'b0;
                    end
                end else
                    wk_cnt <= wk_cnt + 10'd1;
            end
            endcase
        end
    end
    wire signed [15:0] u_swing_eff = wk_drive ? wk_u : u_kick;
    motor_drive #(.SIGN(-1)) md_kick(              // SIGN=-1板测确定
        .clk_50m(clk_50m), .rst_n(rst_n),
        .u(u_swing_eff), .deadband(16'sd4),
        .sel(kick_sel), .duty(kick_duty)
    );
    //---- 顶点交棒判据能量泵没有到顶信号,自己加----
    // 速度过零=摆动顶点 + 顶点在目标角 ±APEX_WIN 窗内 + 连续 2 顶点进窗 → 1 拍交棒
    localparam [8:0] APEX_WIN = 9'd250;   // 顶点窗 ±250 计数≈21°
    reg signed [15:0] vel_prev;
    reg        apex_pulse;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            vel_prev   <= 16'sd0;
            apex_pulse <= 1'b0;
        end else if (tick_3ms) begin
            vel_prev   <= pump_vel;
            apex_pulse <= ((vel_prev < 0 && pump_vel >= 0) ||
                           (vel_prev > 0 && pump_vel <= 0));
        end
    end
    wire      in_window = (pump_theta_hold > (theta_ref_reg - APEX_WIN)) &&
                          (pump_theta_hold < (theta_ref_reg + APEX_WIN));
    reg [1:0] apex_cnt;
    reg        capture_apex;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            apex_cnt     <= 2'd0;
            capture_apex <= 1'b0;
        end else if (swing_rst || !swing_mode) begin
            apex_cnt     <= 2'd0;
            capture_apex <= 1'b0;
        end else if (tick_3ms) begin
            if (in_window && apex_pulse) begin
                apex_cnt     <= (apex_cnt == 2'd2) ? 2'd2 : apex_cnt + 2'd1;
                capture_apex <= (apex_cnt == 2'd1);   // 连续第 2 个顶点进窗 → 交棒脉冲
            end else begin
                capture_apex <= 1'b0;   // apex_cnt 不清零：大摆只在顶点瞬间进窗，
                                        // 顶点间必出窗，清零就永远数不到 2
            end
        end
    end
    // 到顶=起摆完成：PID 接管与 swing_up 的 capture 同一条路
    wire swing_capture_eff = SWING_SEL ? capture_apex : swing_capture;

    //------------------ 6ms 节拍vel_est/swing_up 共用；与 pid 内部拍同相位 ------------------
    reg [18:0] tick_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n)                    tick_cnt <= 19'd0;
        else if(tick_cnt >= 19'd299_999) tick_cnt <= 19'd0;
        else                          tick_cnt <= tick_cnt + 19'd1;
    end
    wire tick_6ms = (tick_cnt == 19'd299_999);

    //------------------ 摆杆测速能量起摆配套 ------------------
    wire signed [15:0] vel;        // 摆杆速度计数/6ms 拍;死区保持+4 点 FIR,±512 限幅

    // vel_est 原版:死区保持;vel/theta_valid 下游没人用,留着只为 SWING_SEL=0 回退路径用 theta_hold
    vel_est vel_est_inst(
        .clk_50m(clk_50m), .rst_n(rst_n), .tick(tick_6ms),
        .theta_in(theta_out), .theta_hold(theta_hold), .theta_valid(theta_valid), .vel(vel)
    );

    //------------------ 摆臂测速能量法配套，从 j280_integrated 移植 ------------------
    wire signed [15:0] arm_vel;    // 摆臂速度 Q8vel_est_arm 16 拍窗口差分；能量法臂速门控/刹车输入
    vel_est_arm vel_est_arm_inst(
        .clk_50m(clk_50m), .rst_n(rst_n), .tick(tick_3ms),
        .pos_in(enc_pos1), .vel(arm_vel), .dsum()
    );

    wire [7:0] pid_duty;
    wire [1:0] pid_sel;
	wire			[03:0]	swing_key		;
	
    always@(posedge clk_50m or negedge rst_n) begin
      if(!rst_n) begin
        pid_en     <= 1'b0;
        swing_mode <= 1'b0;
        swing_rst  <= 1'b0;
      end
      else if(swing_key[0]) begin
        pid_en     <= 1'b1;      // SW1：随时重起摆。条件不放 !pid_en，不然按住 SW1 时 pid_en 锁死、SW1 失效
        swing_mode <= 1'b1;
        swing_rst  <= 1'b1;      // 1 拍复位脉冲：swing_up 状态机拉回 S_IDLE 重新初始踢
      end
      else if((swing_key[1] || swing_key[2]) && swing_mode) begin
        pid_en     <= 1'b0;      // SW2/SW3：起摆阶段=停止；平衡后=横杆 ±90° 步进见 pid 例化 .step/.step_dir
        swing_mode <= 1'b0;
        swing_rst  <= 1'b0;
      end
      else if(swing_capture_eff) begin   // 能量泵顶点脉冲与 classic capture 同路交棒
        swing_mode <= 1'b0;      // 起摆完成：PID 接管pid_en 保持 1
        swing_rst  <= 1'b0;
      end
      else begin
        swing_rst  <= 1'b0;      // 其余时间复位脉冲保持 0仅维持 1 拍
      end
    end

    //------------------ SW4 匀速转圈:平衡中按 SW4=转圈 开/关 ------------------
    // spin=1 时 pos_goal 每外环拍 +POS_STEP_RATE → 臂恒速旋转rate6→≈33°/s
    reg spin_en;
    always@(posedge clk_50m or negedge rst_n) begin
      if(!rst_n)
        spin_en <= 1'b0;
      else if(swing_key[0])
        spin_en <= 1'b0;                      // SW1 重新起摆：转圈自动关防交棒后突然转起来
      else if(kb_cancel)
        spin_en <= 1'b0;                      // 键盘 S14 重置：同时关转圈"清零"=全停
      else if((swing_key[3] || kb_spin_toggle) && !swing_mode && pid_en)
        spin_en <= ~spin_en;                  // SW4 或键盘 S12：平衡中按=转圈 开/关同一开关
    end

    //------------------ OLED 第 3 行消息:显示保持到下一次按键;按键/键盘共用 oled_msg0=空白…8=数字 ------------------
    reg [3:0] oled_msg;   // 0=空白 1=RIGHT 90 2=LEFT 90 3=TURN AROUND 4=LEFT 5=RIGHT 6=CANCEL 7=YES 8=数字
    reg [9:0] oled_num;   // msg=8 时显示值
    reg [1:0] oled_dir;   // msg=8 时方向:0=无前缀 1=L: 2=R:跟踪 kb_input.dir
    always@(posedge clk_50m or negedge rst_n) begin
      if(!rst_n) begin
        oled_msg <= 4'd0;
        oled_num <= 10'd0;
        oled_dir <= 2'd0;
      end
      else if(swing_key[0]) begin
        oled_msg <= 4'd0;                              // SW1 起摆:清显示
      end
      else if(kb_cancel) begin
        oled_msg <= 4'd6;                              // S14:CANCEL
        oled_num <= 10'd0;
        oled_dir <= 2'd0;
      end
      else if(kb_spin_toggle) begin
        oled_msg <= 4'd3;                              // S12:TURN AROUND
      end
      else if(kb_valid) begin
        case (kb_code)
        4'd3: begin                                    // S4:LEFT武装顺时针
          oled_msg <= 4'd4;
          oled_dir <= 2'd1;
        end
        4'd7: begin                                    // S8:RIGHT武装逆时针
          oled_msg <= 4'd5;
          oled_dir <= 2'd2;
        end
        4'd14: begin                                   // S15 退格:显示退格后的数
          oled_msg <= 4'd8;
          oled_num <= kb_buf / 10'd10;
        end
        4'd15: if(kb_dir != 2'd0 && kb_buf != 10'd0) begin   // S16 确定：此拍 kb_buf 还是旧值，锁住刚输入的角度
          oled_msg <= 4'd8;
          oled_num <= kb_buf;
        end
        4'd0, 4'd1, 4'd2, 4'd4, 4'd5, 4'd6, 4'd8, 4'd9, 4'd10, 4'd12: begin
          // 数字键与 kb_input 同译码,3 位上限:显示新输入值
          oled_msg <= 4'd8;
          if(kb_buf < 10'd100) begin
            case (kb_code)
            4'd0:  oled_num <= kb_buf*10'd10 + 10'd1;
            4'd1:  oled_num <= kb_buf*10'd10 + 10'd2;
            4'd2:  oled_num <= kb_buf*10'd10 + 10'd3;
            4'd4:  oled_num <= kb_buf*10'd10 + 10'd4;
            4'd5:  oled_num <= kb_buf*10'd10 + 10'd5;
            4'd6:  oled_num <= kb_buf*10'd10 + 10'd6;
            4'd8:  oled_num <= kb_buf*10'd10 + 10'd7;
            4'd9:  oled_num <= kb_buf*10'd10 + 10'd8;
            4'd10: oled_num <= kb_buf*10'd10 + 10'd9;
            default: oled_num <= kb_buf*10'd10;         // S13=0
            endcase
          end
        end
        default: oled_msg <= oled_msg;                 // 其余键:保持
        endcase
      end
      else if(swing_key[1] && !swing_mode && pid_en) begin
        oled_msg <= 4'd1;                              // SW2:RIGHT 90
      end
      else if(swing_key[2] && !swing_mode && pid_en) begin
        oled_msg <= 4'd2;                              // SW3:LEFT 90
      end
      else if(swing_key[3] && !swing_mode && pid_en) begin
        oled_msg <= 4'd3;                              // SW4:TURN AROUND
      end
    end



     pid #(
      .POS_STEP_RATE (16'sd6)   // SW2 步进速度：12 太快回头 45°、4 太慢，6 刚好；只影响步进斜坡，不碰平衡
     ) pid_inst (
      .clk_50m     (clk_50m    ),
      .rst_n       (rst_n      ),
      .en          (pid_en && !swing_mode),  // 起摆期间 PID 停积分天然清零
      .calib       (swing_capture_eff),   // 能量泵交棒脉冲与 classic capture 同一条路
      .step        ((swing_key[1] || swing_key[2]) && !swing_mode && pid_en),   // SW2/SW3 平衡中按一次=±90° 斜坡走位
      .step_dir    (swing_key[2] && !swing_mode && pid_en),   // SW3=反向 -90°与 .step 同拍
      .spin        (spin_en),   // SW4=匀速转圈 开/关
      .spin_dir    ((kb_dir == 2'd2)),   // S4/S8 联动：S8逆时针=反向，S4/无方向=默认正向与 goto 同号
      .theta_out   (theta_out  ),   // 接 adc_inst 的输出
      .theta_ref   (theta_ref_reg),   // 目标角度固定 1925，不再用 SW4 标定
      .pos         (enc_pos1   ),   // 接 encoder_1 的 pos
      .duty_out    (pid_duty   ),
      .motor_a_sel (pid_sel    ),
      .dbg_err_ang (           ),   // 4 个调试口可接 GAO 或串口，不接就悬空
      .dbg_pos_ref (           ),
      .dbg_err_pos (           ),
      .dbg_pwm     (           ),
      .goto_valid  (kb_goto_valid && !swing_mode && pid_en),   // 键盘 S16：平衡态才生效与 SW2/SW3 同语义
      .goto_cnts   (kb_goto_cnts),    // 带符号计数正=CW 负=CCW
      .goto_cancel (kb_cancel)        // 键盘 S14：取消未走完旋转
  );

    //================ 按键调参台：矩阵键盘扫描 + 输入缓冲 ================
    // 键盘：先 S4/S8 预告方向 → 输数字角度→ S16 确定走斜坡；S15 退格、S14 重置、S12 匀速转开关
    wire [3:0]  kb_code;
    wire        kb_valid;
    wire [9:0]  kb_buf;
    wire [1:0]  kb_dir;
    wire        kb_goto_valid;
    wire [15:0] kb_goto_cnts;
    wire        kb_cancel;
    wire        kb_spin_toggle;
    wire [3:0]  kb_last;
    keypad keypad_inst(
        .clk_50m   (clk_50m), .rst_n(rst_n),
        .col       (kb_col), .row(kb_row),
        .key_code  (kb_code), .key_valid(kb_valid)
    );
    kb_input kb_input_inst(
        .clk_50m   (clk_50m), .rst_n(rst_n),
        .key_code  (kb_code), .key_valid(kb_valid),
        .kb_buf     (kb_buf), .dir(kb_dir),
        .goto_valid(kb_goto_valid), .goto_cnts(kb_goto_cnts),
        .spin_toggle(kb_spin_toggle), .cancel(kb_cancel),
        .last_key  (kb_last)
    );

     // 回退路径 SWING_SEL=0:classic 共振踢;SWING_SEL=1 时 .en 恒 0 停用
     swing_up #(
      .SWING_PWM(8'd96)   // 50% 甩太快 PID 接不住 → 37.5%128→96；还快再降
     ) swing_up_inst (
      .clk_50m    (clk_50m      ),
      .rst_n      (rst_n        ),
      .en         (swing_mode && !SWING_SEL),   // SWING_SEL=1 时停防与能量泵并跑
      .restart    (swing_rst    ),
      .theta_out  (theta_hold   ),   // 死区保持角滑窗过死区不跳变
      .theta_ref  (theta_ref_reg),
      .duty_out   (swing_duty   ),
      .motor_sel  (swing_sel    ),
      .capture    (swing_capture),
      .active     (             )
  );










     //==================== OLED 显示：第 1 行 My FPGA；第 2 行摆杆实时角度；第 3 行键盘/按键消息 ====================
     oled_ssd1306 oled_inst (
		.clk_50m  (clk_50m  ),
		.rst_n    (rst_n    ),
		.msg_sel  (oled_msg ),
		.msg_dir  (oled_dir ),
		.msg_num  (oled_num ),
		.disp_theta(theta_out),   // 第 2 行 T:xxxxadc 原始 12 位
		.oled_scl (oled_scl ),
		.oled_sda (oled_sda )
	 );

	genvar i;
		generate for(i = 0; i <= 3; i = i + 1) begin : key_filter_gen
	
		key_filter #(
	
			.CNT_MAX ( 20'd1000_000)
		
		) key_filter_inst (
	
			.sys_clk	(clk_50m),	
			.sys_rst_n	(rst_n),	
			.key_in		(key_sw[i]),	
			
			.key_flag	(swing_key[i])
	
		);
	
		end
	endgenerate
	
	
//参数定义
reg		signed	[15:0]	theta_r		;
wire	signed	[15:0]	theta_out_r	;
	
adc adc_inst(

   .clk_50m     (clk_50m	),	
   .rst_n		(rst_n		),
   // 3PA1030硬件接口	
   .adc_clk		(adc_clk	),	// ADC采样时钟输出
   .adc_oe_n	(adc_oe_n	),	// 输出使能低有效
   .adc_stby_n	(adc_stby_n	),	// 休眠低有效
   .adc_data_in	(adc_data_in),	// D9~D0并行数据
   .theta_out   (theta_out	)	//采集的实时角度值输出
   
);   

//保存当前theta_r、encoder_r值,并符号化
always@(posedge clk_50m or negedge rst_n) begin : theta_r_proc
    if(!rst_n)begin
        theta_r <= 0;
    end else
        theta_r <= {1'b0,theta_out};
end

assign theta_out_r = theta_r;	// 已去掉官方 +135 偏移：串口直接显示原始角度与 PID 同一把尺子

// 起摆期电机由起摆模块驱动,捕获后切回 PID;SWING_SEL=1 时起摆期=能量泵经 motor_drive 适配
wire [7:0] swing_duty_eff = SWING_SEL ? kick_duty  : swing_duty;
wire [1:0] swing_sel_eff  = SWING_SEL ? kick_sel   : swing_sel;
wire [7:0] motor_duty = swing_mode ? swing_duty_eff : pid_duty;
wire [1:0] motor_sel  = swing_mode ? swing_sel_eff  : pid_sel;

motor_ctrl motor_ctrl_A(

    .clk_50m		(clk_50m		),	//时钟输入
    .rst_n			(rst_n			),	//复位输入
    // 电机A 控制指令
    .motor_a_sel	(motor_sel		),
    .duty_out		(motor_duty		),	//duty_out
	//output
    .ain1			(AN1			),
    .ain2			(AN2			),

	.pwm			(PWMA			)

    );

motor_ctrl motor_ctrl_B(

    .clk_50m		(clk_50m		),	//时钟输入
    .rst_n			(rst_n			),	//复位输入
    // 本板电机插 B 口，A/B 双口同驱classic 原来只让 B 跟 PID，起摆期 B 全静
    .motor_a_sel	(motor_sel	),
    .duty_out		(motor_duty	),	//duty_out
	//output 
    .ain1			(BN1			),
    .ain2			(BN2			),
	
	.pwm			(PWMB			)

    );

	encoder encoder_1 (
	
		.clk_50m		(clk_50m),
		.rst_n			(rst_n),
				
		.enc_a			(enc1_a), 
		.enc_b			(enc1_b),
		
		.pos			(enc_pos1)	//编码器采集的位置环
	
	);

	encoder encoder_2 (
	
		.clk_50m		(clk_50m),
		.rst_n			(rst_n),
				
		.enc_a			(enc2_a), 
		.enc_b			(enc2_b),
		
		.pos			(enc_pos2)	//编码器采集的位置环
	
	);	
	
	wire	[15:0]	data_out;
	
uart #(

    .CNT_1S	(32'd749_999 )// 15ms/帧 ≈66Hz 遥测SerialPlot 按此帧率
	
) uart_inst (
	
	.clk_50M		(clk_50m		),
	.rst_n			(rst_n			),
			
	.key3			(1'b1			),	// 上电就自动发串口纯发数据，不动控制
		
	.fpga_rx		(uart_rx		),
	.fpga_tx		(uart_tx		),
	
	.data_in		(theta_out_r	),	// ch1=摆杆角度
	.encoder_in		({enc_pos1 ,u_kick}),	// ch2=臂位置 ch3=泵输出 u_kick
		.keypad_in		(arm_vel),	// ch4=臂速
	
	.data_out   	(data_out		)   
	
);

endmodule

