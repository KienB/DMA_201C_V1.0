`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/09/21 16:35:48
// Design Name: 
// Module Name: ddc_edid_slave
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ddc_edid_slave(
 input  i_local_clk   
,input  i_rst_n       

,input  i_ir          

,output o_ddc_hpd     
,inout  io_ddc_i2c_sda
,input  i_ddc_i2c_scl 

);




reg  [3:0]  w_vid_mode;
reg  [3:0]  r_vid_mode;
reg  [3:0]  r_vid_mode_dly1;
wire [7:0]  w_rom_addr;
wire [7:0]  w_rom_data;
wire [7:0]  w_i2c_slave_rdaddr;
wire [7:0]  w_i2c_slave_rddata;

wire [7:0]  w_i2c_slave_device_addr;
wire        w_i2c_slave_osda;
wire        w_i2c_slave_isda;

reg  [23:0] r_ms_cnt;
reg         r_ddc_hpd;





assign o_ddc_hpd = r_ddc_hpd;

assign io_ddc_i2c_sda = (w_i2c_slave_osda==1'b0) ? 1'b0 : 1'hz;
assign w_i2c_slave_isda = io_ddc_i2c_sda;

assign w_i2c_slave_device_addr   = {7'h50, 1'b0};

assign w_rom_addr = w_i2c_slave_rdaddr;
assign w_i2c_slave_rddata = w_rom_data;






always @(posedge i_local_clk or negedge i_rst_n) begin
	if(~i_rst_n)
		r_vid_mode <= 0;
	else
		r_vid_mode <= w_vid_mode;
end

always @(posedge i_local_clk or negedge i_rst_n) begin
	if(~i_rst_n)
		r_vid_mode_dly1 <= 0;
	else
		r_vid_mode_dly1 <= r_vid_mode;
end

always @(posedge i_local_clk or negedge i_rst_n) begin
	if(~i_rst_n)
		r_ms_cnt <= 0;
	else if(r_vid_mode != r_vid_mode_dly1)
		r_ms_cnt <= 0;
	else if(&r_ms_cnt)
		r_ms_cnt <= r_ms_cnt;
	else
		r_ms_cnt <= r_ms_cnt+1;
end

always @(posedge i_local_clk or negedge i_rst_n) begin
	if(~i_rst_n)
		r_ddc_hpd <= 0;
	else if(~(&r_ms_cnt))
		r_ddc_hpd <= 0;
	else
		r_ddc_hpd <= 1;
end




ddc_edid_rom u0_ddc_edid_rom(
 .addra ({r_vid_mode,w_rom_addr})
,.clka  (i_local_clk )
,.douta (w_rom_data  )
);


i2c_slave i2c_slave_u0(
 .iSclk       (i_local_clk             )
,.iRstN       (i_rst_n                 )

,.iDeviceAddr (w_i2c_slave_device_addr )
,.oWrEn       ()
,.oWrAddr     ()
,.oWrData     ()

,.iValid      ()
,.oRdAddr     (w_i2c_slave_rdaddr      )
,.iRdData     (w_i2c_slave_rddata      )
,.oRdDe       ()

,.iSda        (w_i2c_slave_isda        )
,.iScl        (i_ddc_i2c_scl           )
,.oSda        (w_i2c_slave_osda        )

,.oState      ()
);



vio_4 u0_vio_4(
 .clk       (i_local_clk )
,.probe_in0 (w_vid_mode  )
);






endmodule


