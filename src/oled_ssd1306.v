//=============================================================================
// oled_ssd1306.v : 0.96" 128x64 OLED,SSD1306,I2C 0x3C 驱动
//   第 1 行固定 "My FPGA",第 2 行摆杆实时角度 "T:xxxx",第 3 行按键/键盘消息;
//   字库 33 字符内联,索引=字符×6+列;I2C 50kHz,初始化序列=实协工程验证版。
//=============================================================================
module oled_ssd1306 #(
    parameter [7:0] I2C_ADDR = 8'h78,   // SSD1306 写地址0x3C<<1
    parameter [7:0] QUART    = 8'd250   // 50kHz
)(
    input                clk_50m ,
    input                rst_n   ,
    input  [3:0]         msg_sel ,   // 第 3 行消息选择0~8,见文件头
    input  [1:0]         msg_dir ,   // msg_sel=8 时方向:0=无前缀 1=L: 2=R:
    input  [9:0]         msg_num ,   // msg_sel=8 时数字 0~999
    input  [11:0]        disp_theta, // 第 2 行摆杆实时角度
    output               oled_scl,
    inout                oled_sda
);

//==================== 字符码 ====================
localparam C0=5'd0, C1=5'd1, C2=5'd2, C3=5'd3, C4=5'd4,
           C5=5'd5, C6=5'd6, C7=5'd7, C8=5'd8, C9=5'd9,
           CA=5'd10, CC=5'd11, CD=5'd12, CE=5'd13, CF=5'd14,
           CG=5'd15, CH=5'd16, CI=5'd17, CL=5'd18, CN=5'd19,
           CO=5'd20, CP=5'd21, CR=5'd22, CS=5'd23, CT=5'd24,
           CU=5'd25, CY=5'd26, CSP=5'd27, CCOL=5'd28, CPLUS=5'd29, CMIN=5'd30,
           CM=5'd31, Cy=6'd32;   // v11 新增:'M' 'y';Cy 必须 6 位5'd32 会溢出成 0,y 就显示成 '0'

//==================== 初始化命令表抄实协工程 oled.c,板上验证过 ====================
function [7:0] ini_cmd; input [4:0] s;
begin
    case (s)
    5'd0:  ini_cmd = 8'hAE;
    5'd1:  ini_cmd = 8'h20;
    5'd2:  ini_cmd = 8'hB0;
    5'd3:  ini_cmd = 8'hC8;
    5'd4:  ini_cmd = 8'h00;
    5'd5:  ini_cmd = 8'h10;
    5'd6:  ini_cmd = 8'h40;
    5'd7:  ini_cmd = 8'h81;
    5'd8:  ini_cmd = 8'hA1;
    5'd9:  ini_cmd = 8'hA6;
    5'd10: ini_cmd = 8'hA8;
    5'd11: ini_cmd = 8'hA4;
    5'd12: ini_cmd = 8'hD3;
    5'd13: ini_cmd = 8'hD5;
    5'd14: ini_cmd = 8'hD9;
    5'd15: ini_cmd = 8'hDA;
    5'd16: ini_cmd = 8'hDB;
    5'd17: ini_cmd = 8'h8D;
    default: ini_cmd = 8'hAF;
    endcase
end
endfunction

function [7:0] ini_dat; input [4:0] s;
begin
    case (s)
    5'd1:  ini_dat = 8'h00;
    5'd7:  ini_dat = 8'hFF;
    5'd10: ini_dat = 8'h3F;
    5'd12: ini_dat = 8'h00;
    5'd13: ini_dat = 8'hF0;
    5'd14: ini_dat = 8'h22;
    5'd15: ini_dat = 8'h12;
    5'd16: ini_dat = 8'h20;
    5'd17: ini_dat = 8'h14;
    default: ini_dat = 8'h00;
    endcase
end
endfunction

function ini_two; input [4:0] s;
begin
    ini_two = (s==5'd1 || s==5'd7 || s==5'd10 || s==5'd12 || s==5'd13 ||
               s==5'd14 || s==5'd15 || s==5'd16 || s==5'd17);
end
endfunction

//==================== I2C 位引擎50kHz;空闲=SCL低/SDA释放 ====================
localparam E_IDLE=4'd0,  E_STA0=4'd1, E_STA1=4'd2, E_STA2=4'd3, E_STA3=4'd4,
           E_WR0=4'd5,   E_WR1=4'd6,  E_WR2=4'd7,  E_WR3=4'd8,
           E_AK0=4'd9,   E_AK1=4'd10, E_AK2=4'd11,
           E_STP0=4'd12, E_STP1=4'd13, E_STP2=4'd14;

reg  [3:0] est;
reg  [7:0] cnt;
reg  [3:0] bit_i;
reg  [7:0] tx_sr;
reg        scl_r, sda_r;

wire bus_busy = (est != E_IDLE);

assign oled_scl = scl_r;
assign oled_sda = sda_r ? 1'bz : 1'b0;

reg        op_start, op_write, op_stop;
reg  [7:0] op_data;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        est<=E_IDLE; scl_r<=1'b0; sda_r<=1'b1; cnt<=8'd0;
        bit_i<=4'd0; tx_sr<=8'd0;
    end else begin
        case (est)
        E_IDLE: begin
            scl_r<=1'b0;
            sda_r<=1'b1;
            if(op_start || op_write || op_stop) begin
                cnt<=8'd0;
                if(op_start) begin
                    est<=E_STA0;
                end else if(op_write) begin
                    tx_sr<=op_data;
                    bit_i<=4'd0;
                    est<=E_WR0;
                end else begin
                    est<=E_STP0;
                end
            end
        end
        E_STA0: begin scl_r<=1'b0; sda_r<=1'b1; if(cnt==QUART-1) begin cnt<=8'd0; est<=E_STA1; end else cnt<=cnt+1'b1; end
        E_STA1: begin scl_r<=1'b1; sda_r<=1'b1; if(cnt==QUART-1) begin cnt<=8'd0; est<=E_STA2; end else cnt<=cnt+1'b1; end
        E_STA2: begin scl_r<=1'b1; sda_r<=1'b0; if(cnt==QUART-1) begin cnt<=8'd0; est<=E_STA3; end else cnt<=cnt+1'b1; end
        E_STA3: begin scl_r<=1'b0; sda_r<=1'b0; if(cnt==QUART-1) begin cnt<=8'd0; est<=E_IDLE; end else cnt<=cnt+1'b1; end
        E_WR0 : begin scl_r<=1'b0; sda_r<=tx_sr[7]; if(cnt==QUART-1) begin cnt<=8'd0; est<=E_WR1; end else cnt<=cnt+1'b1; end
        E_WR1 : begin scl_r<=1'b1;                   if(cnt==QUART-1) begin cnt<=8'd0; est<=E_WR2; end else cnt<=cnt+1'b1; end
        E_WR2 : begin scl_r<=1'b1;                   if(cnt==QUART-1) begin cnt<=8'd0; est<=E_WR3; end else cnt<=cnt+1'b1; end
        E_WR3 : begin scl_r<=1'b0;
                     if(cnt==QUART-1) begin
                         cnt<=8'd0;
                         bit_i<=bit_i+1'b1;
                         tx_sr<={tx_sr[6:0],1'b0};
                         if(bit_i==4'd7) begin
                             est<=E_AK0;
                         end else begin
                             est<=E_WR0;
                         end
                     end else cnt<=cnt+1'b1;
                 end
        E_AK0 : begin scl_r<=1'b0; sda_r<=1'b1; if(cnt==QUART-1) begin cnt<=8'd0; est<=E_AK1; end else cnt<=cnt+1'b1; end
        E_AK1 : begin scl_r<=1'b1;               if(cnt==QUART-1) begin cnt<=8'd0; est<=E_AK2; end else cnt<=cnt+1'b1; end
        E_AK2 : begin scl_r<=1'b1;               if(cnt==QUART-1) begin cnt<=8'd0; est<=E_IDLE; end else cnt<=cnt+1'b1; end
        E_STP0: begin scl_r<=1'b0; sda_r<=1'b0; if(cnt==QUART-1) begin cnt<=8'd0; est<=E_STP1; end else cnt<=cnt+1'b1; end
        E_STP1: begin scl_r<=1'b1; sda_r<=1'b0; if(cnt==QUART-1) begin cnt<=8'd0; est<=E_STP2; end else cnt<=cnt+1'b1; end
        E_STP2: begin scl_r<=1'b1; sda_r<=1'b1; if(cnt==QUART-1) begin cnt<=8'd0; est<=E_IDLE; end else cnt<=cnt+1'b1; end
        default: est<=E_IDLE;
        endcase
    end
end

//==================== 像素字节生成纯组合;case 一律一行一条目! ====================
// 数据流字节序号 idx 1..128 → 列 col 0..127;每字符 6 列
wire [6:0] col = idx - 11'd1;
wire [4:0] chp = col / 7'd6;
wire [2:0] ccl = col % 7'd6;

// 第 1 行字符序列:"My FPGA"固定
reg [5:0] l1_ch;
always @(*) begin
    case (chp)
    5'd0: l1_ch = CM;
    5'd1: l1_ch = Cy;
    5'd2: l1_ch = CSP;
    5'd3: l1_ch = CF;
    5'd4: l1_ch = CP;
    5'd5: l1_ch = CG;
    5'd6: l1_ch = CA;
    default: l1_ch = CSP;
    endcase
end

// 第 2 行字符序列:摆杆实时角度 "T:xxxx"
wire [3:0] t3 = (disp_theta/12'd1000) % 4'd10;
wire [3:0] t2 = (disp_theta/12'd100)  % 4'd10;
wire [3:0] t1 = (disp_theta/12'd10)   % 4'd10;
wire [3:0] t0 =  disp_theta % 4'd10;

reg [5:0] l2_ch;
always @(*) begin
    case (chp)
    5'd0: l2_ch = CT;
    5'd1: l2_ch = CCOL;
    5'd2: l2_ch = t3;
    5'd3: l2_ch = t2;
    5'd4: l2_ch = t1;
    5'd5: l2_ch = t0;
    default: l2_ch = CSP;
    endcase
end

// 第 3 行字符序列:按键/键盘消息v13 原第 2 行逻辑搬来
// 数字显示:msg_num 拆三位
wire [3:0] m2 = (msg_num/10'd100) % 4'd10;
wire [3:0] m1 = (msg_num/10'd10)  % 4'd10;
wire [3:0] m0 =  msg_num % 4'd10;

reg [5:0] l3_ch;
always @(*) begin
    case (msg_sel)
    4'd1: case (chp)   // RIGHT 90SW2
          5'd0: l3_ch = CR;
          5'd1: l3_ch = CI;
          5'd2: l3_ch = CG;
          5'd3: l3_ch = CH;
          5'd4: l3_ch = CT;
          5'd5: l3_ch = CSP;
          5'd6: l3_ch = C9;
          5'd7: l3_ch = C0;
          default: l3_ch = CSP;
          endcase
    4'd2: case (chp)   // LEFT 90SW3
          5'd0: l3_ch = CL;
          5'd1: l3_ch = CE;
          5'd2: l3_ch = CF;
          5'd3: l3_ch = CT;
          5'd4: l3_ch = CSP;
          5'd5: l3_ch = C9;
          5'd6: l3_ch = C0;
          default: l3_ch = CSP;
          endcase
    4'd3: case (chp)   // TURN AROUNDSW4/S12
          5'd0: l3_ch = CT;
          5'd1: l3_ch = CU;
          5'd2: l3_ch = CR;
          5'd3: l3_ch = CN;
          5'd4: l3_ch = CSP;
          5'd5: l3_ch = CA;
          5'd6: l3_ch = CR;
          5'd7: l3_ch = CO;
          5'd8: l3_ch = CU;
          5'd9: l3_ch = CN;
          5'd10: l3_ch = CD;
          default: l3_ch = CSP;
          endcase
    4'd4: case (chp)   // LEFTS4
          5'd0: l3_ch = CL;
          5'd1: l3_ch = CE;
          5'd2: l3_ch = CF;
          5'd3: l3_ch = CT;
          default: l3_ch = CSP;
          endcase
    4'd5: case (chp)   // RIGHTS8
          5'd0: l3_ch = CR;
          5'd1: l3_ch = CI;
          5'd2: l3_ch = CG;
          5'd3: l3_ch = CH;
          5'd4: l3_ch = CT;
          default: l3_ch = CSP;
          endcase
    4'd6: case (chp)   // CANCELS14
          5'd0: l3_ch = CC;
          5'd1: l3_ch = CA;
          5'd2: l3_ch = CN;
          5'd3: l3_ch = CC;
          5'd4: l3_ch = CE;
          5'd5: l3_ch = CL;
          default: l3_ch = CSP;
          endcase
    4'd7: case (chp)   // YESS16 确定
          5'd0: l3_ch = CY;
          5'd1: l3_ch = CE;
          5'd2: l3_ch = CS;
          default: l3_ch = CSP;
          endcase
    4'd8: case (chp)   // 数字显示:"L:xxx" / "R:xxx" / "xxx"
          5'd0: l3_ch = (msg_dir==2'd1) ? CL : ((msg_dir==2'd2) ? CR : CSP);
          5'd1: l3_ch = (msg_dir==2'd0) ? m2 : CCOL;
          5'd2: l3_ch = (msg_dir==2'd0) ? m1 : m2;
          5'd3: l3_ch = (msg_dir==2'd0) ? m0 : m1;
          5'd4: l3_ch = (msg_dir==2'd0) ? CSP : m0;
          default: l3_ch = CSP;
          endcase
    default: l3_ch = CSP;   // 0=空白
    endcase
end

wire [5:0] cur_ch = (pg==3'd0) ? l1_ch : ((pg==3'd2) ? l2_ch : ((pg==3'd4) ? l3_ch : CSP));

// 字库:31 字符 x 6 列 = 186 项数据抄实协工程 FONT_6x8;一行一条目!
// 索引必须 = 字符×6+列此前 {ch,ccl} 拼成 ch×8+ccl 会全表错位
reg [7:0] fbyte;
always @(*) begin
    case (cur_ch*3'd6 + ccl)
    // '0'
    8'd0:   fbyte = 8'h3E;
    8'd1:   fbyte = 8'h51;
    8'd2:   fbyte = 8'h49;
    8'd3:   fbyte = 8'h45;
    8'd4:   fbyte = 8'h3E;
    8'd5:   fbyte = 8'h00;
    // '1'
    8'd6:   fbyte = 8'h00;
    8'd7:   fbyte = 8'h42;
    8'd8:   fbyte = 8'h7F;
    8'd9:   fbyte = 8'h40;
    8'd10:  fbyte = 8'h00;
    8'd11:  fbyte = 8'h00;
    // '2'
    8'd12:  fbyte = 8'h42;
    8'd13:  fbyte = 8'h61;
    8'd14:  fbyte = 8'h51;
    8'd15:  fbyte = 8'h49;
    8'd16:  fbyte = 8'h46;
    8'd17:  fbyte = 8'h00;
    // '3'
    8'd18:  fbyte = 8'h21;
    8'd19:  fbyte = 8'h41;
    8'd20:  fbyte = 8'h45;
    8'd21:  fbyte = 8'h4B;
    8'd22:  fbyte = 8'h31;
    8'd23:  fbyte = 8'h00;
    // '4'
    8'd24:  fbyte = 8'h18;
    8'd25:  fbyte = 8'h14;
    8'd26:  fbyte = 8'h12;
    8'd27:  fbyte = 8'h7F;
    8'd28:  fbyte = 8'h10;
    8'd29:  fbyte = 8'h00;
    // '5'
    8'd30:  fbyte = 8'h27;
    8'd31:  fbyte = 8'h45;
    8'd32:  fbyte = 8'h45;
    8'd33:  fbyte = 8'h45;
    8'd34:  fbyte = 8'h39;
    8'd35:  fbyte = 8'h00;
    // '6'
    8'd36:  fbyte = 8'h3C;
    8'd37:  fbyte = 8'h4A;
    8'd38:  fbyte = 8'h49;
    8'd39:  fbyte = 8'h49;
    8'd40:  fbyte = 8'h30;
    8'd41:  fbyte = 8'h00;
    // '7'
    8'd42:  fbyte = 8'h01;
    8'd43:  fbyte = 8'h71;
    8'd44:  fbyte = 8'h09;
    8'd45:  fbyte = 8'h05;
    8'd46:  fbyte = 8'h03;
    8'd47:  fbyte = 8'h00;
    // '8'
    8'd48:  fbyte = 8'h36;
    8'd49:  fbyte = 8'h49;
    8'd50:  fbyte = 8'h49;
    8'd51:  fbyte = 8'h49;
    8'd52:  fbyte = 8'h36;
    8'd53:  fbyte = 8'h00;
    // '9'
    8'd54:  fbyte = 8'h06;
    8'd55:  fbyte = 8'h49;
    8'd56:  fbyte = 8'h49;
    8'd57:  fbyte = 8'h29;
    8'd58:  fbyte = 8'h1E;
    8'd59:  fbyte = 8'h00;
    // 'A'
    8'd60:  fbyte = 8'h7E;
    8'd61:  fbyte = 8'h11;
    8'd62:  fbyte = 8'h11;
    8'd63:  fbyte = 8'h11;
    8'd64:  fbyte = 8'h7E;
    8'd65:  fbyte = 8'h00;
    // 'C'
    8'd66:  fbyte = 8'h3E;
    8'd67:  fbyte = 8'h41;
    8'd68:  fbyte = 8'h41;
    8'd69:  fbyte = 8'h41;
    8'd70:  fbyte = 8'h22;
    8'd71:  fbyte = 8'h00;
    // 'D'
    8'd72:  fbyte = 8'h7F;
    8'd73:  fbyte = 8'h41;
    8'd74:  fbyte = 8'h41;
    8'd75:  fbyte = 8'h22;
    8'd76:  fbyte = 8'h1C;
    8'd77:  fbyte = 8'h00;
    // 'E'
    8'd78:  fbyte = 8'h7F;
    8'd79:  fbyte = 8'h49;
    8'd80:  fbyte = 8'h49;
    8'd81:  fbyte = 8'h49;
    8'd82:  fbyte = 8'h41;
    8'd83:  fbyte = 8'h00;
    // 'F'
    8'd84:  fbyte = 8'h7F;
    8'd85:  fbyte = 8'h09;
    8'd86:  fbyte = 8'h09;
    8'd87:  fbyte = 8'h09;
    8'd88:  fbyte = 8'h01;
    8'd89:  fbyte = 8'h00;
    // 'G'
    8'd90:  fbyte = 8'h3E;
    8'd91:  fbyte = 8'h41;
    8'd92:  fbyte = 8'h49;
    8'd93:  fbyte = 8'h49;
    8'd94:  fbyte = 8'h7A;
    8'd95:  fbyte = 8'h00;
    // 'H'
    8'd96:  fbyte = 8'h7F;
    8'd97:  fbyte = 8'h08;
    8'd98:  fbyte = 8'h08;
    8'd99:  fbyte = 8'h08;
    8'd100: fbyte = 8'h7F;
    8'd101: fbyte = 8'h00;
    // 'I'
    8'd102: fbyte = 8'h00;
    8'd103: fbyte = 8'h41;
    8'd104: fbyte = 8'h7F;
    8'd105: fbyte = 8'h41;
    8'd106: fbyte = 8'h00;
    8'd107: fbyte = 8'h00;
    // 'L'
    8'd108: fbyte = 8'h7F;
    8'd109: fbyte = 8'h40;
    8'd110: fbyte = 8'h40;
    8'd111: fbyte = 8'h40;
    8'd112: fbyte = 8'h40;
    8'd113: fbyte = 8'h00;
    // 'N'
    8'd114: fbyte = 8'h7F;
    8'd115: fbyte = 8'h04;
    8'd116: fbyte = 8'h08;
    8'd117: fbyte = 8'h10;
    8'd118: fbyte = 8'h7F;
    8'd119: fbyte = 8'h00;
    // 'O'
    8'd120: fbyte = 8'h3E;
    8'd121: fbyte = 8'h41;
    8'd122: fbyte = 8'h41;
    8'd123: fbyte = 8'h41;
    8'd124: fbyte = 8'h3E;
    8'd125: fbyte = 8'h00;
    // 'P'
    8'd126: fbyte = 8'h7F;
    8'd127: fbyte = 8'h09;
    8'd128: fbyte = 8'h09;
    8'd129: fbyte = 8'h09;
    8'd130: fbyte = 8'h06;
    8'd131: fbyte = 8'h00;
    // 'R'
    8'd132: fbyte = 8'h7F;
    8'd133: fbyte = 8'h09;
    8'd134: fbyte = 8'h19;
    8'd135: fbyte = 8'h29;
    8'd136: fbyte = 8'h46;
    8'd137: fbyte = 8'h00;
    // 'S'
    8'd138: fbyte = 8'h46;
    8'd139: fbyte = 8'h49;
    8'd140: fbyte = 8'h49;
    8'd141: fbyte = 8'h49;
    8'd142: fbyte = 8'h31;
    8'd143: fbyte = 8'h00;
    // 'T'
    8'd144: fbyte = 8'h01;
    8'd145: fbyte = 8'h01;
    8'd146: fbyte = 8'h7F;
    8'd147: fbyte = 8'h01;
    8'd148: fbyte = 8'h01;
    8'd149: fbyte = 8'h00;
    // 'U'
    8'd150: fbyte = 8'h3F;
    8'd151: fbyte = 8'h40;
    8'd152: fbyte = 8'h40;
    8'd153: fbyte = 8'h40;
    8'd154: fbyte = 8'h3F;
    8'd155: fbyte = 8'h00;
    // 'Y'
    8'd156: fbyte = 8'h07;
    8'd157: fbyte = 8'h08;
    8'd158: fbyte = 8'h70;
    8'd159: fbyte = 8'h08;
    8'd160: fbyte = 8'h07;
    8'd161: fbyte = 8'h00;
    // ' '
    8'd162: fbyte = 8'h00;
    8'd163: fbyte = 8'h00;
    8'd164: fbyte = 8'h00;
    8'd165: fbyte = 8'h00;
    8'd166: fbyte = 8'h00;
    8'd167: fbyte = 8'h00;
    // ':'
    8'd168: fbyte = 8'h00;
    8'd169: fbyte = 8'h36;
    8'd170: fbyte = 8'h36;
    8'd171: fbyte = 8'h00;
    8'd172: fbyte = 8'h00;
    8'd173: fbyte = 8'h00;
    // '+'
    8'd174: fbyte = 8'h08;
    8'd175: fbyte = 8'h08;
    8'd176: fbyte = 8'h3E;
    8'd177: fbyte = 8'h08;
    8'd178: fbyte = 8'h08;
    8'd179: fbyte = 8'h00;
    // '-'
    8'd180: fbyte = 8'h08;
    8'd181: fbyte = 8'h08;
    8'd182: fbyte = 8'h08;
    8'd183: fbyte = 8'h08;
    8'd184: fbyte = 8'h08;
    8'd185: fbyte = 8'h00;
    // 'M'v11 第 1 行 My FPGA 用
    8'd186: fbyte = 8'h7F;
    8'd187: fbyte = 8'h02;
    8'd188: fbyte = 8'h0C;
    8'd189: fbyte = 8'h02;
    8'd190: fbyte = 8'h7F;
    8'd191: fbyte = 8'h00;
    // 'y'
    8'd192: fbyte = 8'h0C;
    8'd193: fbyte = 8'h50;
    8'd194: fbyte = 8'h50;
    8'd195: fbyte = 8'h50;
    8'd196: fbyte = 8'h3C;
    8'd197: fbyte = 8'h00;
    default: fbyte = 8'h00;
    endcase
end

//==================== 线性序列器初始化→8 页循环刷新→永续 ====================
localparam PH_INIT=2'd0, PH_RUN=2'd1;
localparam S_BOOT=4'd0, S_GO=4'd1, S_ADDR=4'd2, S_BYTE=4'd3, S_STOP=4'd4,
           S_NEXT=4'd5;

reg  [3:0]  sst;
reg  [24:0] boot_cnt;
reg  [1:0]  phase;
reg  [4:0]  ini_i;
reg  [2:0]  ps;         // 页子步:0=B0+p 1=列低 2=列高 3=数据流
reg  [2:0]  pg;         // 页 0~7
reg  [10:0] idx;
reg         iss_flag;
reg         nxt_stop;

reg [7:0] xbyte;
reg       more;
always @(*) begin
    case (phase)
    PH_INIT: begin
        xbyte = (idx==11'd0) ? 8'h00 :
                (idx==11'd1) ? ini_cmd(ini_i) : ini_dat(ini_i);
        more  = (idx==11'd0) || ((idx==11'd1) && ini_two(ini_i));
    end
    default: begin
        if(ps==3'd0) begin
            xbyte = (idx==11'd0) ? 8'h00 : (8'hB0 + {5'd0, pg});
            more  = (idx==11'd0);
        end
        else if(ps==3'd1) begin
            xbyte = (idx==11'd0) ? 8'h00 : 8'h00;
            more  = (idx==11'd0);
        end
        else if(ps==3'd2) begin
            xbyte = (idx==11'd0) ? 8'h00 : 8'h10;
            more  = (idx==11'd0);
        end
        else begin
            // 数据流:0x40 + 128 字节;页0=第1行 页2=第2行实时角度 页4=第3行键盘消息 其余清0
            more = (idx < 11'd128);
            xbyte = (idx==11'd0) ? 8'h40 :
                    ((pg==3'd0) || (pg==3'd2) || (pg==3'd4)) ? fbyte : 8'h00;
        end
    end
    endcase
end

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        sst<=S_BOOT;
        boot_cnt<=25'd0;
        phase<=PH_INIT;
        ini_i<=5'd0;
        ps<=3'd0;
        pg<=3'd0;
        idx<=11'd0;
        iss_flag<=1'b0;
        nxt_stop<=1'b0;
        op_start<=1'b0;
        op_write<=1'b0;
        op_stop<=1'b0;
        op_data<=8'd0;
    end else begin
        op_start<=1'b0;
        op_write<=1'b0;
        op_stop<=1'b0;
        case (sst)
        S_BOOT: begin
            if(boot_cnt == 25'd4_999_999) begin
                sst<=S_GO;
            end else begin
                boot_cnt<=boot_cnt+1'b1;
            end
        end
        S_GO: begin
            if(!bus_busy && !iss_flag) begin
                op_start<=1'b1;
                iss_flag<=1'b1;
            end
            else if(bus_busy && iss_flag) begin
                iss_flag<=1'b0;
                sst<=S_ADDR;
            end
        end
        S_ADDR: begin
            if(!bus_busy && !iss_flag) begin
                op_write<=1'b1;
                op_data<=I2C_ADDR;
                iss_flag<=1'b1;
                idx<=11'd0;
            end
            else if(bus_busy && iss_flag) begin
                iss_flag<=1'b0;
                sst<=S_BYTE;
            end
        end
        S_BYTE: begin
            if(!bus_busy && !iss_flag) begin
                op_write<=1'b1;
                op_data<=xbyte;
                idx<=idx+11'd1;
                iss_flag<=1'b1;
                nxt_stop<=!more;
            end
            else if(bus_busy && iss_flag) begin
                iss_flag<=1'b0;
                if(nxt_stop) begin
                    nxt_stop<=1'b0;
                    sst<=S_STOP;
                end
            end
        end
        S_STOP: begin
            if(!bus_busy && !iss_flag) begin
                op_stop<=1'b1;
                iss_flag<=1'b1;
            end
            else if(bus_busy && iss_flag) begin
                iss_flag<=1'b0;
                sst<=S_NEXT;
            end
        end
        S_NEXT: begin
            case (phase)
            PH_INIT: begin
                if(ini_i==5'd18) begin
                    phase<=PH_RUN;
                    ps<=3'd0;
                    pg<=3'd0;
                end else begin
                    ini_i<=ini_i+5'd1;
                end
                sst<=S_GO;
            end
            default: begin
                if(ps==3'd3) begin
                    ps<=3'd0;
                    if(pg==3'd7) begin
                        pg<=3'd0;   // 8 页写完 → 从头循环刷新
                    end else begin
                        pg<=pg+3'd1;
                    end
                end else begin
                    ps<=ps+3'd1;
                end
                sst<=S_GO;
            end
            endcase
        end
        default: sst<=S_BOOT;
        endcase
    end
end

endmodule
