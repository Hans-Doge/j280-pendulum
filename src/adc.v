module  adc(

    input      		clk_50m		,
    input      		rst_n		,
    // 3PA1030硬件接口
    output	    	adc_clk		,	// ADC采样时钟输出
    output	    	adc_oe_n	,	// 输出使能低有效
    output	    	adc_stby_n	,	// 休眠低有效
	output			adc_otr		,
    input	[9:0]	adc_data_in	,	// D9~D0并行数据
    output	[11:0]	theta_out

    );

//参数定义
reg				clk_div			;
//将adc扩大到12bit,
wire	[11:0]	theta			;
reg		[11:0]	theta_out_reg	;
wire	[11:0]	theta_12bit		;
reg		[15:0]	theta_sum		;
reg		[17:0]	cnt_3ms			;
reg		[13:0]	cnt_187us		;
reg		[03:0]	cnt_add			;

//信号定义
assign adc_oe_n		= 1'b0;
assign adc_stby_n	= 1'b0;
assign theta_12bit	= {adc_data_in  , 2'b0 };
assign theta_out	= theta_out_reg;

always@(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)
		clk_div <= 1'b0;
    else
		clk_div <= ~clk_div;
end

assign adc_clk = clk_div; // 25MHz输出给3PA1030 CLK引脚

//cnt_3ms;
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        cnt_3ms <= 0;
    end else if(cnt_3ms == 149999)begin
        cnt_3ms <= 0;
    end else
        cnt_3ms = cnt_3ms + 1;
end
//cnt_187.5Us
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        cnt_187us <= 0;
    end else if(cnt_187us == 9374 )begin
        cnt_187us <= 0;
    end else
        cnt_187us <= cnt_187us +1;
end
//theta_sum
always@(posedge clk_50m or negedge rst_n)begin
    if(!rst_n)begin
        theta_sum <= 0;
        cnt_add <= 0;
        theta_out_reg <= 0;
    end else if(cnt_187us == 9373 )begin
        theta_sum <= theta_sum + theta_12bit;
        cnt_add <= cnt_add + 1;
    end else if(cnt_3ms == 149999)begin
		theta_out_reg <= theta_sum >> 4;
		theta_sum <= 0;
		cnt_add <= 0;
    end else begin
		theta_out_reg <= theta_out_reg;
		theta_sum <= theta_sum;
		cnt_add <= cnt_add;
    end
end

endmodule
