module uart #(

    parameter   CNT_1S = 32'd99_999_999 //写入后延时1S读取
	
) (

	input              		clk_50M		,
	input              		rst_n		,
				
	input					key3		,
			
	input              		fpga_rx		,
	output             		fpga_tx		,
		
	input      		[15:0]	data_in		,
	input			[31:0]	encoder_in	,
	input			[15:0]	keypad_in	,   //键盘回显打包:{dir[1:0],buf[9:0],last_key[3:0]}
	
	output	reg		[15:0]	data_out      
	
);

localparam	UART_BPS       =   21'd9600			,   //比特率
            CLK_FREQ       =   26'd50_000_000	,   //时钟频率
            SEN_CNT_MAX    =   20'd60_000		,   //发送间隔 60_000 周期=1.2ms/字节(12B 帧≈14.4ms)
            SEN_BYTE_CNT   =   4'd11				;	//发送字节数量(11=9 遥测+2 键盘回显字节)

wire			pi_flag		;
wire	[7:0]	pi_data		;
wire			pi_done		;



uart_tx #(

    .UART_BPS    (UART_BPS  ),  //串口波特率
    .CLK_FREQ    (CLK_FREQ  )   //时钟频率
	
) uart_tx_inst (

    .sys_clk    (clk_50M    ),  
    .sys_rst_n  (rst_n      ),  
    .pi_data    (pi_data    ),  
    .pi_flag    (pi_flag    ),  
                
    .tx         (fpga_tx    ),  
    .pi_done    (pi_done    )
	
);

reg [21:0]  send_cnt;
reg	[3:0]	rx_byte_cnt	;
reg	[7:0]	cmdx		;
reg	[7:0]	data0		;
reg	[7:0]	data1		;

reg	u_wr_r	;
reg	u_wr_rr	;

reg	u_wr_pos	;
reg	u_wr_neg	;

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		u_wr_r   <= 1'b0;
		u_wr_rr  <= 1'b0;
		u_wr_neg <= 1'b0;
	end else begin
		u_wr_r   <= key3;
		u_wr_rr  <= u_wr_r;
		u_wr_pos <= u_wr_r & ~u_wr_rr;
		u_wr_neg <= ~u_wr_r & u_wr_rr;
	end
end

reg			u_wr_valid	;
reg			u_rd_pos	;
reg	[31:0]	cnt_delay	;

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		u_wr_valid    <= 1'b0;
	end else begin
		if(u_wr_pos) 
			u_wr_valid = 1'b1;
		if(cnt_delay == CNT_1S)begin 
			u_wr_valid = 1'b0;
		end
	end
end

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		cnt_delay   <= 'd0;
	end else begin
		if(u_wr_valid)begin
			if (cnt_delay == CNT_1S) begin   
				cnt_delay <= 'd0;
			end else begin
				cnt_delay <= cnt_delay + 'd1;
			end
		end
	end
end

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		u_rd_pos    <= 1'b0;
	end else begin
		if (cnt_delay == CNT_1S)
			u_rd_pos <= 1'b1;
		else 
			u_rd_pos <= 1'b0;
	end
end

//------------------------Print message via serial port---------------------------------------//
reg [4:0]   tx_byte_cnt;
wire        send_en;
reg [7:0]   pi_data_r;

assign pi_flag = send_en;
assign pi_data = pi_data_r;

reg send_valid;

always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n==1'b0)
        send_valid <= 1'b0;
    else if(u_rd_pos)		
        send_valid <= 1'b1;
end

always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        send_cnt <= 'd0;
    end else begin
        if(send_valid && send_cnt < SEN_CNT_MAX && tx_byte_cnt <= SEN_BYTE_CNT)	
            send_cnt <= send_cnt + 1'b1;
        else 
            send_cnt <= 'd0;
    end
end
 
assign send_en = (send_cnt == SEN_CNT_MAX - 1) ? 1'b1 : 1'b0;

reg		[31:0]	count_s;


always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        count_s <= 'd0;
	end else if(count_s == CNT_1S - 'd1) begin
		count_s <= 'd0;
	end else begin
		count_s <= count_s + 'd1;
	end
end

always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        tx_byte_cnt <= 5'd0;
    end else begin		
		//if(u_rd_pos || tx_byte_cnt >= 'd6)	//解除发送字节限制		
		if( u_rd_pos || (count_s == CNT_1S - 'd1) )	//解除发送字节限制	
            tx_byte_cnt <= 5'd0; 
        else if(tx_byte_cnt > SEN_BYTE_CNT)
            tx_byte_cnt <= SEN_BYTE_CNT + 1; 
        else if(send_en)
            tx_byte_cnt <= tx_byte_cnt +1'b1; 
    end
end

reg	[15:0]	data_in_reg		;
reg	[31:0]	encoder_in_reg	;
reg	[15:0]	keypad_in_reg	;   //键盘回显锁存

always@(posedge clk_50M or negedge rst_n) begin
	if(rst_n == 1'b0) begin
		data_in_reg <= 'd0;
		encoder_in_reg <= 'H4E20;
	keypad_in_reg <= 'd0;
	end else if(send_en) begin
		data_in_reg <= data_in;
	keypad_in_reg <= keypad_in;
		encoder_in_reg <= encoder_in;		
	end else begin
		data_in_reg <= data_in_reg;	
		encoder_in_reg <= encoder_in_reg;	
	end
end

always@(posedge clk_50M or negedge rst_n) begin
    if(rst_n == 1'b0) begin
        pi_data_r <= 'd0;
	end else if( send_en )begin
		case(tx_byte_cnt)
			0:	begin 
					pi_data_r <= 'HAA; 
				end
			1:	begin 
					pi_data_r <= 'H55; 
				end
			2:	begin 
					pi_data_r <= data_in_reg[15:8]; 
				end
			3:	begin 
					pi_data_r <= data_in_reg[7:0]; 
				end
			4:	begin 
					pi_data_r <= encoder_in_reg[31:24]; 
				end
			5:	begin 
					pi_data_r <= encoder_in_reg[23:16]; 
				end		
			6:	begin 
					pi_data_r <= encoder_in_reg[15:8]; 
				end
			7:	begin 
					pi_data_r <= encoder_in_reg[7:0];
				end
			8:	begin 
					pi_data_r <= keypad_in_reg[15:8];   //键盘回显高字节 {dir[1:0],buf[9:4]}
				end
			9:	begin 
					pi_data_r <= keypad_in_reg[7:0];   //键盘回显低字节 {buf[3:0],last_key[3:0]}
				end
					10:	begin
					pi_data_r <= 'H0D;
				end
			11:	begin
					pi_data_r <= 'H0A;
				end
endcase
			

	end
end

endmodule