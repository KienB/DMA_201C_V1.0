
Q
Command: %s
53*	vivadotcl2 
route_design2default:defaultZ4-113h px� 
�
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2"
Implementation2default:default2
xc7s502default:defaultZ17-347h px� 
�
0Got license for feature '%s' and/or device '%s'
310*common2"
Implementation2default:default2
xc7s502default:defaultZ17-349h px� 
p
,Running DRC as a precondition to command %s
22*	vivadotcl2 
route_design2default:defaultZ4-22h px� 
P
Running DRC with %s threads
24*drc2
22default:defaultZ23-27h px� 
�	
�Clock Placer Checks: Sub-optimal placement for a clock-capable IO pin and PLL pair. 
Resolution: A dedicated routing path between the two can be used if: (a) The clock-capable IO (CCIO) is placed on a CCIO capable site (b) The PLL is placed in the same clock region as the CCIO pin. Both the above conditions must be met at the same time, else it may lead to longer and less predictable clock insertion delays.
 This is normally an ERROR but the CLOCK_DEDICATED_ROUTE constraint is set to FALSE allowing your design to continue. The use of this override is highly discouraged as it may lead to very poor timing results. It is recommended that this error condition be corrected in the design.

	%s (IBUF.O) is locked to %s
	%s (PLLE2_ADV.CLKIN1) is provisionally placed by clockplacer on %s
%s*DRC2\
 "F
i_osc_clk_27M_IBUF_inst	i_osc_clk_27M_IBUF_inst2default:default2default:default2@
 "*
	IOB_X0Y70
	IOB_X0Y702default:default2default:default2n
 "X
 u0_pll_27MHz/inst/plle2_adv_inst	 u0_pll_27MHz/inst/plle2_adv_inst2default:default2default:default2J
 "4
PLLE2_ADV_X1Y0
PLLE2_ADV_X1Y02default:default2default:default2;
 #DRC|Implementation|Placement|Clocks2default:default8ZPLCK-20h px� 
b
DRC finished with %s
79*	vivadotcl2(
0 Errors, 1 Warnings2default:defaultZ4-198h px� 
e
BPlease refer to the DRC report (report_drc) for more information.
80*	vivadotclZ4-199h px� 
V

Starting %s Task
103*constraints2
Routing2default:defaultZ18-103h px� 
}
BMultithreading enabled for route_design using a maximum of %s CPUs17*	routeflow2
22default:defaultZ35-254h px� 
p

Phase %s%s
101*constraints2
1 2default:default2#
Build RT Design2default:defaultZ18-101h px� 
B
-Phase 1 Build RT Design | Checksum: 441caea8
*commonh px� 
�

%s
*constraints2o
[Time (s): cpu = 00:00:16 ; elapsed = 00:00:14 . Memory (MB): peak = 1841.426 ; gain = 0.0002default:defaulth px� 
v

Phase %s%s
101*constraints2
2 2default:default2)
Router Initialization2default:defaultZ18-101h px� 
o

Phase %s%s
101*constraints2
2.1 2default:default2 
Create Timer2default:defaultZ18-101h px� 
A
,Phase 2.1 Create Timer | Checksum: 441caea8
*commonh px� 
�

%s
*constraints2o
[Time (s): cpu = 00:00:16 ; elapsed = 00:00:15 . Memory (MB): peak = 1841.426 ; gain = 0.0002default:defaulth px� 
{

Phase %s%s
101*constraints2
2.2 2default:default2,
Fix Topology Constraints2default:defaultZ18-101h px� 
M
8Phase 2.2 Fix Topology Constraints | Checksum: 441caea8
*commonh px� 
�

%s
*constraints2o
[Time (s): cpu = 00:00:16 ; elapsed = 00:00:15 . Memory (MB): peak = 1845.219 ; gain = 3.7932default:defaulth px� 
t

Phase %s%s
101*constraints2
2.3 2default:default2%
Pre Route Cleanup2default:defaultZ18-101h px� 
F
1Phase 2.3 Pre Route Cleanup | Checksum: 441caea8
*commonh px� 
�

%s
*constraints2o
[Time (s): cpu = 00:00:16 ; elapsed = 00:00:15 . Memory (MB): peak = 1845.219 ; gain = 3.7932default:defaulth px� 
p

Phase %s%s
101*constraints2
2.4 2default:default2!
Update Timing2default:defaultZ18-101h px� 
C
.Phase 2.4 Update Timing | Checksum: 1d98d1cf5
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:19 ; elapsed = 00:00:16 . Memory (MB): peak = 1864.262 ; gain = 22.8362default:defaulth px� 
�
Intermediate Timing Summary %s164*route2L
8| WNS=-4.278 | TNS=-452.873| WHS=-0.221 | THS=-194.671|
2default:defaultZ35-416h px� 
}

Phase %s%s
101*constraints2
2.5 2default:default2.
Update Timing for Bus Skew2default:defaultZ18-101h px� 
r

Phase %s%s
101*constraints2
2.5.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
E
0Phase 2.5.1 Update Timing | Checksum: 142315801
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:20 ; elapsed = 00:00:17 . Memory (MB): peak = 1870.617 ; gain = 29.1912default:defaulth px� 
�
Intermediate Timing Summary %s164*route2K
7| WNS=-4.278 | TNS=-452.806| WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px� 
P
;Phase 2.5 Update Timing for Bus Skew | Checksum: 1cc6840e4
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:20 ; elapsed = 00:00:17 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
H
3Phase 2 Router Initialization | Checksum: fed6b496
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:20 ; elapsed = 00:00:17 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
p

Phase %s%s
101*constraints2
3 2default:default2#
Initial Routing2default:defaultZ18-101h px� 
C
.Phase 3 Initial Routing | Checksum: 177e2ffd6
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:21 ; elapsed = 00:00:18 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
s

Phase %s%s
101*constraints2
4 2default:default2&
Rip-up And Reroute2default:defaultZ18-101h px� 
u

Phase %s%s
101*constraints2
4.1 2default:default2&
Global Iteration 02default:defaultZ18-101h px� 
�
Intermediate Timing Summary %s164*route2K
7| WNS=-4.380 | TNS=-462.550| WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px� 
H
3Phase 4.1 Global Iteration 0 | Checksum: 19c92b945
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:23 ; elapsed = 00:00:19 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
F
1Phase 4 Rip-up And Reroute | Checksum: 19c92b945
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:23 ; elapsed = 00:00:19 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
|

Phase %s%s
101*constraints2
5 2default:default2/
Delay and Skew Optimization2default:defaultZ18-101h px� 
p

Phase %s%s
101*constraints2
5.1 2default:default2!
Delay CleanUp2default:defaultZ18-101h px� 
r

Phase %s%s
101*constraints2
5.1.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
E
0Phase 5.1.1 Update Timing | Checksum: 1b350c731
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:24 ; elapsed = 00:00:20 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
�
Intermediate Timing Summary %s164*route2K
7| WNS=-4.380 | TNS=-462.550| WHS=N/A    | THS=N/A    |
2default:defaultZ35-416h px� 
C
.Phase 5.1 Delay CleanUp | Checksum: 236a020f0
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:24 ; elapsed = 00:00:20 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
z

Phase %s%s
101*constraints2
5.2 2default:default2+
Clock Skew Optimization2default:defaultZ18-101h px� 
M
8Phase 5.2 Clock Skew Optimization | Checksum: 236a020f0
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:24 ; elapsed = 00:00:20 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
O
:Phase 5 Delay and Skew Optimization | Checksum: 236a020f0
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:24 ; elapsed = 00:00:20 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
n

Phase %s%s
101*constraints2
6 2default:default2!
Post Hold Fix2default:defaultZ18-101h px� 
p

Phase %s%s
101*constraints2
6.1 2default:default2!
Hold Fix Iter2default:defaultZ18-101h px� 
r

Phase %s%s
101*constraints2
6.1.1 2default:default2!
Update Timing2default:defaultZ18-101h px� 
E
0Phase 6.1.1 Update Timing | Checksum: 283fb4135
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:25 ; elapsed = 00:00:20 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
�
Intermediate Timing Summary %s164*route2K
7| WNS=-4.380 | TNS=-462.550| WHS=0.043  | THS=0.000  |
2default:defaultZ35-416h px� 
C
.Phase 6.1 Hold Fix Iter | Checksum: 1a67f7177
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:25 ; elapsed = 00:00:20 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
A
,Phase 6 Post Hold Fix | Checksum: 1a67f7177
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:25 ; elapsed = 00:00:20 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
o

Phase %s%s
101*constraints2
7 2default:default2"
Route finalize2default:defaultZ18-101h px� 
B
-Phase 7 Route finalize | Checksum: 23d330dbe
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:25 ; elapsed = 00:00:20 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
v

Phase %s%s
101*constraints2
8 2default:default2)
Verifying routed nets2default:defaultZ18-101h px� 
I
4Phase 8 Verifying routed nets | Checksum: 23d330dbe
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:25 ; elapsed = 00:00:20 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
r

Phase %s%s
101*constraints2
9 2default:default2%
Depositing Routes2default:defaultZ18-101h px� 
E
0Phase 9 Depositing Routes | Checksum: 23b0de6cb
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:25 ; elapsed = 00:00:21 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
t

Phase %s%s
101*constraints2
10 2default:default2&
Post Router Timing2default:defaultZ18-101h px� 
�
Estimated Timing Summary %s
57*route2K
7| WNS=-4.380 | TNS=-462.550| WHS=0.043  | THS=0.000  |
2default:defaultZ35-57h px� 
B
!Router estimated timing not met.
128*routeZ35-328h px� 
G
2Phase 10 Post Router Timing | Checksum: 23b0de6cb
*commonh px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:25 ; elapsed = 00:00:21 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
@
Router Completed Successfully
2*	routeflowZ35-16h px� 
�

%s
*constraints2p
\Time (s): cpu = 00:00:25 ; elapsed = 00:00:21 . Memory (MB): peak = 1884.758 ; gain = 43.3322default:defaulth px� 
Z
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px� 
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
1852default:default2
1072default:default2
12default:default2
02default:defaultZ4-41h px� 
^
%s completed successfully
29*	vivadotcl2 
route_design2default:defaultZ4-42h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
route_design: 2default:default2
00:00:272default:default2
00:00:222default:default2
1884.7582default:default2
43.3322default:defaultZ17-268h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2 
00:00:00.0042default:default2
1884.7582default:default2
0.0002default:defaultZ17-268h px� 
H
&Writing timing data to binary archive.266*timingZ38-480h px� 
D
Writing placer database...
1603*designutilsZ20-1893h px� 
=
Writing XDEF routing.
211*designutilsZ20-211h px� 
J
#Writing XDEF routing logical nets.
209*designutilsZ20-209h px� 
J
#Writing XDEF routing special nets.
210*designutilsZ20-210h px� 
�
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2)
Write XDEF Complete: 2default:default2
00:00:012default:default2 
00:00:00.5562default:default2
1884.7582default:default2
0.0002default:defaultZ17-268h px� 
�
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2i
UE:/FPGA_project/Xilinx/DMA_201C_V1.0/project/project.runs/impl_1/top_ceshi_routed.dcp2default:defaultZ17-1381h px� 
�
%s4*runtcl2�
pExecuting : report_drc -file top_ceshi_drc_routed.rpt -pb top_ceshi_drc_routed.pb -rpx top_ceshi_drc_routed.rpx
2default:defaulth px� 
�
Command: %s
53*	vivadotcl2w
creport_drc -file top_ceshi_drc_routed.rpt -pb top_ceshi_drc_routed.pb -rpx top_ceshi_drc_routed.rpx2default:defaultZ4-113h px� 
>
IP Catalog is up to date.1232*coregenZ19-1839h px� 
P
Running DRC with %s threads
24*drc2
22default:defaultZ23-27h px� 
�
#The results of DRC are in file %s.
168*coretcl2�
YE:/FPGA_project/Xilinx/DMA_201C_V1.0/project/project.runs/impl_1/top_ceshi_drc_routed.rptYE:/FPGA_project/Xilinx/DMA_201C_V1.0/project/project.runs/impl_1/top_ceshi_drc_routed.rpt2default:default8Z2-168h px� 
\
%s completed successfully
29*	vivadotcl2

report_drc2default:defaultZ4-42h px� 
�
%s4*runtcl2�
�Executing : report_methodology -file top_ceshi_methodology_drc_routed.rpt -pb top_ceshi_methodology_drc_routed.pb -rpx top_ceshi_methodology_drc_routed.rpx
2default:defaulth px� 
�
Command: %s
53*	vivadotcl2�
�report_methodology -file top_ceshi_methodology_drc_routed.rpt -pb top_ceshi_methodology_drc_routed.pb -rpx top_ceshi_methodology_drc_routed.rpx2default:defaultZ4-113h px� 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px� 
Y
$Running Methodology with %s threads
74*drc2
22default:defaultZ23-133h px� 
�
2The results of Report Methodology are in file %s.
450*coretcl2�
eE:/FPGA_project/Xilinx/DMA_201C_V1.0/project/project.runs/impl_1/top_ceshi_methodology_drc_routed.rpteE:/FPGA_project/Xilinx/DMA_201C_V1.0/project/project.runs/impl_1/top_ceshi_methodology_drc_routed.rpt2default:default8Z2-1520h px� 
d
%s completed successfully
29*	vivadotcl2&
report_methodology2default:defaultZ4-42h px� 
�
%s4*runtcl2�
�Executing : report_power -file top_ceshi_power_routed.rpt -pb top_ceshi_power_summary_routed.pb -rpx top_ceshi_power_routed.rpx
2default:defaulth px� 
�
Command: %s
53*	vivadotcl2�
sreport_power -file top_ceshi_power_routed.rpt -pb top_ceshi_power_summary_routed.pb -rpx top_ceshi_power_routed.rpx2default:defaultZ4-113h px� 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px� 
K
,Running Vector-less Activity Propagation...
51*powerZ33-51h px� 
P
3
Finished Running Vector-less Activity Propagation
1*powerZ33-1h px� 
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
1972default:default2
1072default:default2
12default:default2
02default:defaultZ4-41h px� 
^
%s completed successfully
29*	vivadotcl2 
report_power2default:defaultZ4-42h px� 
�
%s4*runtcl2s
_Executing : report_route_status -file top_ceshi_route_status.rpt -pb top_ceshi_route_status.pb
2default:defaulth px� 
�
%s4*runtcl2�
�Executing : report_timing_summary -max_paths 10 -file top_ceshi_timing_summary_routed.rpt -pb top_ceshi_timing_summary_routed.pb -rpx top_ceshi_timing_summary_routed.rpx -warn_on_violation 
2default:defaulth px� 
r
UpdateTimingParams:%s.
91*timing29
% Speed grade: -2, Delay Type: min_max2default:defaultZ38-91h px� 
|
CMultithreading enabled for timing update using a maximum of %s CPUs155*timing2
22default:defaultZ38-191h px� 
�
rThe design failed to meet the timing requirements. Please see the %s report for details on the timing violations.
188*timing2"
timing summary2default:defaultZ38-282h px� 
�
}There are set_bus_skew constraint(s) in this design. Please run report_bus_skew to ensure that bus skew requirements are met.223*timingZ38-436h px� 
�
%s4*runtcl2f
RExecuting : report_incremental_reuse -file top_ceshi_incremental_reuse_routed.rpt
2default:defaulth px� 
g
BIncremental flow is disabled. No incremental reuse Info to report.423*	vivadotclZ4-1062h px� 
�
%s4*runtcl2f
RExecuting : report_clock_utilization -file top_ceshi_clock_utilization_routed.rpt
2default:defaulth px� 
�
%s4*runtcl2�
�Executing : report_bus_skew -warn_on_violation -file top_ceshi_bus_skew_routed.rpt -pb top_ceshi_bus_skew_routed.pb -rpx top_ceshi_bus_skew_routed.rpx
2default:defaulth px� 
r
UpdateTimingParams:%s.
91*timing29
% Speed grade: -2, Delay Type: min_max2default:defaultZ38-91h px� 
|
CMultithreading enabled for timing update using a maximum of %s CPUs155*timing2
22default:defaultZ38-191h px� 


End Record