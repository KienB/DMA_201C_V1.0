// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2019.2 (win64) Build 2708876 Wed Nov  6 21:40:23 MST 2019
// Date        : Wed Sep 11 18:31:34 2024
// Host        : DESKTOP-AR7JAMA running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               d:/FPGA_project/Xilinx/DMA_201C_V1.0/project/project.srcs/sources_1/ip/ila_256/ila_256_stub.v
// Design      : ila_256
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7s50fgga484-2
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
(* X_CORE_INFO = "ila,Vivado 2019.2" *)
module ila_256(clk, probe0)
/* synthesis syn_black_box black_box_pad_pin="clk,probe0[255:0]" */;
  input clk;
  input [255:0]probe0;
endmodule
