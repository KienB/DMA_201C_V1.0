#-------------------------------------------------------------------------------------------
#-   ____  ____
#-  /   /\/   /
#- /___/  \  /
#- \   \   \/    � Copyright 2019 Xilinx, Inc. All rights reserved.
#-  \   \        This file contains confidential and proprietary information of Xilinx, Inc.
#-  /   /        and is protected under U.S. and international copyright and other
#- /___/   /\    intellectual property laws.
#- \   \  /  \
#-  \___\/\___\
#-
#--------------------------------------------------------------------------------------------
#- Device:              Ultrascale
#- Author:              Tatsukawa, Kruger, Defossez
#- Entity Name:         top_mmcme3
#- Purpose:
#-
#-   This TCL script is used to do basic project setup and clarify the DRP settings.
#-
#-  Ultrascale MMCM is called:          MMCME3
#-      7-Series MMCM is called:        MMCME2
#-      UltrascalePlus MMCM is called:  MMCME4
#-  MMCME3 attributes
#-      CLKINx_PERIOD:      0.968 to 100.000
#-      REF_JITTERx:        0.001 to 0.999
#-      BANDWIDTH:          LOW, HIGH, OPTIMIZED and POSTCRC
#-      COMPENSATION:       AUTO, ZHOLD, EXTERNAL, INTERNAL and BUF_IN
#-      DIVCLK_DIVIDE:      1 to 106
#-      CLKFBOUT_MULT:      2 to 64
#-      CLKFBOUT_PHASE:     -360 to 360
#-      CLKOUTn_DIVIDE:     1 to 128
#-      CLKOUTn_PHASE:      -360 to 360
#-      CLKOUTn_DUTY_CYCLE: 0.01 to 0.99
#-
#-  XAPP888 TCL commands:
#-      xapp888_create_project <> - Basic project setup. Adjust as needed
#-      xapp888_help <>           - Descriptions of added TCL commands
#-
#-  XAPP888 TCL DRP Settings:
#-      xapp888_drp_settings <CLKFBOUT_MULT>  <DIVCLK_DIVIDE> <PHASE> <BANDWIDTH>
#-      Displays & Returns the ordered pairs of DRP addresses & Data
#-
#-      xapp888_drp_clkout <DIVIDE> <Duty Cycle e.g. 0.5> <Phase e.g.11.25> <CLKOUT0 to CLKOUT6>
#-      Displays & Returns the ordered pairs of DRP addresses & Data
#-
#-      xapp888_merge_drp <list>
#-      Returns the ordered DRP addresses/data merging fractional address 07 & 13
#-
#- TCL process list:
#-     - xapp888_create_project     {}
#-     - xapp888_merge_drp          {list}
#-     - xapp888_drp_clkout_frac    {divide phase}
#-     - xapp888_drp_clkout         {divide dutycycle phase clkout}
#-     - xapp888_drp_calc_m         {divide phase}
#-     - xapp888_drp_calc_d         {divide}
#-     - xapp888_dec2bin4ltht       {dec}
#-     - xapp888_cpres              {div bw}
#-     - xapp888_locking            {div}
#-     - xapp888_bin2hex            {bits}
#-     - xapp888_hex2bin            {hex}
#-     - xapp888_dec2hex            {value}
#-     - xapp888_drp_settings       {m d phase bw}
#-     - xapp888_dec2bin            {dec bits}
#-     - xapp888_help               {}
#-
#- Tools:               2019.1 or newer
#- Limitations:         environmental
#-
#- Vendor:              Xilinx Inc.
#- Version:             1.40
#- Filename:            top_mmcme3.tcl
#- Date Created:        15 Jan 2015
#- Date Last Modified:  19 Jun 2019
#--------------------------------------------------------------------------------------------
#- Disclaimer:
#-        This disclaimer is not a license and does not grant any rights to the materials
#-        distributed herewith. Except as otherwise provided in a valid license issued to you
#-        by Xilinx, and to the maximum extent permitted by applicable law: (1) THESE MATERIALS
#-        ARE MADE AVAILABLE "AS IS" AND WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL
#-        WARRANTIES AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED
#-        TO WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR
#-        PURPOSE; and (2) Xilinx shall not be liable (whether in contract or tort, including
#-        negligence, or under any other theory of liability) for any loss or damage of any
#-        kind or nature related to, arising under or in connection with these materials,
#-        including for any direct, or any indirect, special, incidental, or consequential
#-        loss or damage (including loss of data, profits, goodwill, or any type of loss or
#-        damage suffered as a result of any action brought by a third party) even if such
#-        damage or loss was reasonably foreseeable or Xilinx had been advised of the
#-        possibility of the same.
#-
#- CRITICAL APPLICATIONS
#-        Xilinx products are not designed or intended to be fail-safe, or for use in any
#-        application requiring fail-safe performance, such as life-support or safety devices
#-        or systems, Class III medical devices, nuclear facilities, applications related to
#-        the deployment of airbags, or any other applications that could lead to death,
#-        personal injury, or severe property or environmental damage (individually and
#-        collectively, "Critical Applications"). Customer assumes the sole risk and
#-        liability of any use of Xilinx products in Critical Applications, subject only to
#-        applicable laws and regulations governing limitations on product liability.
#-
#- THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE AT ALL TIMES.
#-
#- Contact:    e-mail  hotline@xilinx.com        phone   + 1 800 255 7778
#--------------------------------------------------------------------------------------------
#- Revision History:
#-  Rev: 15-Jan-2015 - Tatsukawa
#-      Added TCL DRP commands
#-  Rev: 23-Feb-2016 -
#-      Adjusted for CDDCEN in fractional calculations
#-  Rev: 17-Mar-2016 -
#-      Added min duty cycle check
#-  Rev: 17-Oct-2017 -
#-      Rounded floating point pmphasecycles for int to return correct value
#-  Rev: 10-Jul-2018 - Kruger
#-      Fixed duty cycles for divide > 64
#--------------------------------------------------------------------------------------------
#
proc xapp888_create_project {} {
    create_project xapp888_ultrascale  -force xapp888_ultrascale -part xcku040-ffva1156-1-c
    add_files -norecurse {mmcme3_drp_func.h mmcme3_drp.v top_mmcme3.v  top_mmcme3.xdc}
    import_files -force -norecurse
    import_files -fileset sim_1 -norecurse {top_mmcme3_tb.v}
    update_compile_order -fileset sim_1
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_merge_drp {list} {
    set count_07 0
    set merge_07 ""
    set count_13 0
    set merge_13 ""
    set drp_merged ""
    for {set i 0} { $i <= [expr [llength $list]/2] } {incr i} {
        if {[string match [lindex $list [expr $i*2]] 07]} {
            incr count_07; set merge_07 "$merge_07 [lindex $list [expr 2*$i + 1] ]"
        } elseif {[string match [lindex $list [expr $i*2]] 13]} {
            incr count_13; set merge_13 "$merge_13 [lindex $list [expr 2*$i + 1] ]"
        } else {
            set drp_merged "$drp_merged [lindex $list [expr $i * 2]] [lindex $list [expr $i * 2 + 1]]"
        }
    }

    if {[llength $merge_07] > 1 } {set drp_07_merged [format %04x [expr 0x[lindex $merge_07 0] | 0x[lindex $merge_07 1]]]} else {set drp_07_merged [lindex $merge_07 0]}
    if {[llength $merge_13] > 1} {set drp_13_merged  [format %04x [expr 0x[lindex $merge_13 0] | 0x[lindex $merge_13 1]]]} else {set drp_13_merged [lindex $merge_13 0]}
    if {$count_07 >2} {
        puts "ERROR: Too many shared addresses for 07. Only the first 2 terms are being marged. $merge_07"
    } elseif {$count_07 > 0} {
        set drp_merged "$drp_merged  07 $drp_07_merged"
    }
    if {$count_13 >2} {
        puts "ERROR: Too many shared addresses for 13. Only the first 2 terms are being merged. $merge_13"
    } elseif {$count_13 > 0} {
        set drp_merged "$drp_merged 13 $drp_13_merged"
    }
    puts "$list has been changed to $drp_merged"
    return $drp_merged
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_drp_clkout_frac {divide phase} {
    set divide_frac [expr fmod($divide, 1)]
    set divide_frac_8ths [scan [expr $divide_frac * 8] %d]
    set divide_int [scan [expr floor($divide)] %d]

    set even_part_high [scan [expr floor($divide_int / 2)] %d]
    set even_part_low $even_part_high

    set odd [expr $divide_int - $even_part_high - $even_part_low]
    set odd_and_frac [scan [expr 8 * $odd + $divide_frac_8ths] %d]

    if {$odd_and_frac <=9} {set lt_frac [expr $even_part_high - 1]} else {set lt_frac $even_part_high}
    if {$odd_and_frac <=8} {set ht_frac [expr $even_part_low - 1]} else {set ht_frac $even_part_low}

    set pmfall [scan [expr $odd * 4 + floor($divide_frac_8ths / 2)] %d]
    set pmrise 0
    set dt [scan [expr floor($phase * $divide / 360)] %d]
    set pmrise [scan [expr floor( 8 * (($phase * $divide /360 ) - $dt)+ 0.5 )] %d]
    set pmfall [scan [expr $pmfall + $pmrise] %d]

    #puts "Fractional Requested phase is: $phase; Given divide=$divide then phase increments in [format %f [expr 45.000/$divide ]  ];  "
    #puts "Fractional - Requested phase is: $phase; Actual Given divide=$divide then phase increments in [format %f [expr 45.000/$divide ]  ];  "
    ##puts "DT will be $phasecycles, PM will be $pmphasecycles"
    ##puts "Phase will be shifted by VCO period * $phasecycles.[expr 1000*$pmphasecycles / 8]"
    ##puts "Phase will be shifted by [format %f [expr $phasecycles * 360.000 / $divide]] + [format %f  [expr $pmphasecycles * 45.000 / $divide]] = [format %f [expr ( $phasecycles * 360.000 / $divide) + ($pmphasecycles * 45.000 / $divide) ] ]"
    ##puts "Requested Phase is: $phase; Actual: [format %f [expr ( $phasecycles * 360.000 / $divide) + ($pmphasecycles * 45.000 / $divide) ] ];  "

    if {$odd_and_frac <=9 && $odd_and_frac >=2 || $divide == 2.125} {set wf_fall 1} else {set wf_fall 0}
    if {$odd_and_frac <=8 && $odd_and_frac >=1} {set wf_rise 1} else {set wf_rise 0}

    set dt [scan [expr $dt + floor($pmrise / 8)] %d]
    set pmrise [scan [expr fmod($pmrise , 8)] %d]
    set pmfall [scan [expr fmod($pmfall , 8)] %d]

    set reg1       "[xapp888_dec2bin $pmrise 3]1[xapp888_dec2bin $ht_frac 6][xapp888_dec2bin $lt_frac 6]"
    set reg2       "0[xapp888_dec2bin $divide_frac_8ths 3]1[expr $wf_rise]0000[xapp888_dec2bin $dt 6]"
    #For olympus only return 4 bits as the leading 00 was causing problems due to CDDC_EN bits shifting everything
    set regshared  "[xapp888_dec2bin $pmfall 3][expr $wf_fall]"

    return "$reg1 $reg2 $regshared "
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_drp_clkout {divide dutycycle phase clkout} {
    set clkout_lower [string tolower $clkout]
    switch -glob -- $clkout_lower {
        clkout0  {
            set daddr_reg1 08
            set daddr_reg2 09
        }
        clkout1  {
            set daddr_reg1 0A
            set daddr_reg2 0B
        }
        clkout2  {
            set daddr_reg1 0C
            set daddr_reg2 0D
        }
        clkout3  {
            set daddr_reg1 0E
            set daddr_reg2 0F
        }
        clkout4  {
            set daddr_reg1 10
            set daddr_reg2 11
        }
        clkout5  {
            set daddr_reg1 06
            set daddr_reg2 07
        }
        clkout6  {
            set daddr_reg1 12
            set daddr_reg2 13
        }
    }

    if {$phase < 0} {set phase [expr 360 + $phase]}

    # Calculate phase for PM and O counter.
    # Round counter phase setting up if => 0.5 to closes possible phase
    set phase_in_cycles [expr $phase / 360.0 * $divide]
    set phasecycles_dec [expr (8 * $phase_in_cycles)]
    set phasecycles_int [expr int($phasecycles_dec)]
    set phasecycles_rem [expr ($phasecycles_dec - $phasecycles_int )]
    if {$phasecycles_rem >= 0.5} {set phasecycles_int [expr ($phasecycles_int + 1)]}
    set phasecycles [expr int($phasecycles_int / 8)]
    set pmphasecycles [expr ($phasecycles_int - $phasecycles * 8)]
    # Duty cycle stuff
    if {$divide < 64} {
        set min_dc [expr 1.0 / $divide]
        set max_dc [expr ($divide - 0.5) / $divide]
    } else {
        set min_dc [expr ($divide - 64.0) / $divide]
        set max_dc [expr (64 + 0.5) / $divide]
    }
    if {$dutycycle < $min_dc} {puts "\n\tWARNING: Min duty cycle violation $dutycycle < $min_dc\n\t         Changing dutycycle to $min_dc\n"; set dutycycle $min_dc}
    if {$dutycycle > $max_dc} {puts "\n\tWARNING: Max duty cycle is $dutycycle > $max_dc\n\t         Changing dutycycle to $max_dc\n"; set dutycycle $max_dc}

    puts "Requested phase is: $phase; Given divide=$divide then phase increments in [format %f [expr 45.000/$divide ]  ];  "
    #puts "DT will be $phasecycles, PM will be $pmphasecycles"
    #puts "Phase will be shifted by VCO period * $phasecycles.[expr 1000*$pmphasecycles / 8]"
    #puts "Phase will be shifted by [format %f [expr $phasecycles * 360.000 / $divide]] + [format %f  [expr $pmphasecycles * 45.000 / $divide]] = [format %f [expr ( $phasecycles * 360.000 / $divide) + ($pmphasecycles * 45.000 / $divide) ] ]"
    #puts "Requested Phase is: $phase; Actual: [format %f [expr ( $phasecycles * 360.000 / $divide) + ($pmphasecycles * 45.000 / $divide) ] ];  "

    #puts "[expr $pmphasecycles * 45 / $divide]"

    set ht [scan [expr int($dutycycle * [expr ($divide ) ])] %d]
    set lt [scan [expr $divide - $ht] %d]
    set even_high [scan [expr $divide / 2] %d]
    set odd [expr $divide - $even_high * 2]

    if {$divide == 1} {
         set drp_reg1 "[xapp888_bin2hex [binary scan [binary format I $pmphasecycles] B32 var;string range $var end-2 end]1000001000001]"
         set drp_reg2 "[xapp888_bin2hex 00000000[expr $odd]1[xapp888_dec2bin $phasecycles 6] ]"
         puts "DADDR_$daddr_reg1: $drp_reg1\t-[string toupper $clkout] Register 1"
         puts "DADDR_$daddr_reg2: $drp_reg2\t-[string toupper $clkout] Register 2"
         return "$daddr_reg1 $drp_reg1 $daddr_reg2 $drp_reg2"
    } elseif {[expr fmod($divide,1)] == 0  }  {
         set drp_reg1 "[xapp888_bin2hex [binary scan [binary format I $pmphasecycles] B32 var;string range $var end-2 end]1[xapp888_dec2bin4ltht $ht][xapp888_dec2bin4ltht $lt]]"
         set drp_reg2 "[xapp888_bin2hex 00000000[expr $odd]0[xapp888_dec2bin $phasecycles 6] ]"
         puts "DADDR_$daddr_reg1: $drp_reg1\t-[string toupper $clkout] Register 1: Requested Phase is: $phase; Actual: [format %f [expr ( $phasecycles * 360.000 / $divide) + ($pmphasecycles * 45.000 / $divide) ] ]"
         puts "DADDR_$daddr_reg2: $drp_reg2\t-[string toupper $clkout] Register 2"
         return "$daddr_reg1 $drp_reg1 $daddr_reg2 $drp_reg2"
    } elseif {[string tolower $clkout] == "clkout0" } {
        set drp_frac_registers [xapp888_drp_clkout_frac $divide $phase ]
        set drp_reg1 [xapp888_bin2hex [lindex $drp_frac_registers 0]]
        set drp_reg2 [xapp888_bin2hex [lindex $drp_frac_registers 1]]
        #       For Olympus, CDDCEN shifts the PM_F/WF_F bits to be 15:12 instead of 13:10
        set drp_regshared [xapp888_bin2hex [lindex $drp_frac_registers 2]000000000000]
        puts "DADDR_$daddr_reg2: $drp_reg2\t-[string toupper $clkout] Register 1: Requested Phase is: $phase; Actual: [format %f [expr ( $phasecycles * 360.000 / $divide) + ($pmphasecycles * 45.000 / $divide) ] ]"
        puts "DADDR_$daddr_reg1: $drp_reg1\t-[string toupper $clkout] Register 2"
        puts "DADDR_07: $drp_regshared\t-[string toupper $clkout] Register Shared with CLKOUT5"
        return "08 $drp_reg1 09 $drp_reg2 07 $drp_regshared"
    }  else {
        puts "\nERROR: Fractional divide setting only supported for CLKOUT0. Output clock set to [string toupper $clkout] \n"
    }
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_drp_calc_m {divide phase} {
    set phasecycles [expr int(($divide*$phase)/360)]
    set pmphase [expr ($phase - ($phasecycles *360)/$divide)]
    set pmphasecycles [scan [expr int(($pmphase *$divide)/ 45)] %d]
    set ht [scan [expr ($divide ) / 2] %d]
    set lt [scan [expr $divide - $ht] %d]
    set odd [expr $lt - $ht]
    set daddr_reg1 14
    set daddr_reg2 15
    set daddr_regshared 13
    if {$divide == 1} {
        set drp_reg1 "[xapp888_bin2hex [binary scan [binary format I $pmphasecycles] B32 var;string range $var end-2 end]1000001000001]"
        set drp_reg2 "[xapp888_bin2hex 00000000[expr $odd]1[xapp888_dec2bin $phasecycles 6] ]"
        puts "DADDR_$daddr_reg1: $drp_reg1\t-CLKFBOUT Register 1- "
        puts "DADDR_$daddr_reg2: $drp_reg2\t-CLKFBOUT Register 2- "
        return "$daddr_reg1 $drp_reg1 $daddr_reg2 $drp_reg2"
    } elseif {$divide > 64 } {
        puts "DADDR_14: ERROR: M must be 2 to 64\t-CLKFBOUT Register 2-"
        puts "DADDR_15: ERROR: M must be 2 to 64\t-CLKFBOUT Register 2-\tNOTE: The calculations are only for the non-fractional settings. CLKFBOUT must use an integer divide value for these DRP settings to work"
        return "14 ERROR 15 ERROR "
    } elseif {[expr fmod($divide,1)] > 0} {
        set drp_frac_registers [xapp888_drp_clkout_frac $divide $phase ]
        set drp_reg1 [xapp888_bin2hex [lindex $drp_frac_registers 0]]
        set drp_reg2 [xapp888_bin2hex [lindex $drp_frac_registers 1]]
#       For Olympus, CDDC_EN shifts the PM_F/WF_F bits to be 15:12 instead of 13:10
        set drp_regshared [xapp888_bin2hex [lindex $drp_frac_registers 2]000000000000]
        puts "DADDR_$daddr_reg1: $drp_reg1\t-CLKFBOUT Register 1- "
        puts "DADDR_$daddr_reg2: $drp_reg2\t-CLKFBOUT Register 2- "
        puts "DADDR_$daddr_regshared: $drp_regshared\t-CLKFBOUT Register Shared with CLKOUT6- "
        return "$daddr_reg1  $drp_reg1 $daddr_reg2  $drp_reg2 13 $drp_regshared"
    } else {
        puts "DADDR_$daddr_reg1: [xapp888_bin2hex [binary scan [binary format I $pmphasecycles] B32 var;string range $var end-2 end]1[xapp888_dec2bin4ltht $ht][xapp888_dec2bin4ltht $lt]]\t-CLKFBOUT Register 1- "
        puts "DADDR_$daddr_reg2: [xapp888_bin2hex 00000000[expr $odd]0[binary scan [binary format I $phasecycles] B32 var;string range $var end-5 end] ]\t-CLKFBOUT Register 2- "
        return "$daddr_reg1  [xapp888_bin2hex [binary scan [binary format I $pmphasecycles] B32 var;string range $var end-2 end]1[xapp888_dec2bin4ltht $ht][xapp888_dec2bin4ltht $lt]] $daddr_reg2  [xapp888_bin2hex 00000000[expr $odd]0[binary scan [binary format I $phasecycles] B32 var;string range $var end-5 end] ]"
    }
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_drp_calc_d {divide} {
    set ht [scan [expr ($divide ) / 2] %d]
    set lt [scan [expr $divide - $ht] %d]
    if {$divide == 1} {
        puts "DADDR_16: [xapp888_bin2hex 0001000001000001]\t-DIVCLK Register $divide-"
        return "16 [xapp888_bin2hex 0001000001000001]"
    } elseif { $divide > 19} {
        puts "DADDR_16: ERROR D must be 1 to 19"
        return "16 ERROR"
    } else {
        puts "DADDR_16: [xapp888_bin2hex 0000[xapp888_dec2bin4ltht $ht][xapp888_dec2bin4ltht $lt] ]\t-DIVCLK Register $divide-"
    }
    return "16 [xapp888_bin2hex 0000[xapp888_dec2bin4ltht $ht][xapp888_dec2bin4ltht $lt] ] "
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_dec2bin4ltht {dec} {
     binary scan [binary format c $dec] B* bin
     string range $bin end-5 end
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_cpres {div bw} {
    #CP_RES_LFHF
    set div [scan $div %d]
    set bw_lower [string tolower $bw]
    if {$bw_lower == "low" } {
        switch -glob -- $div {
            1    {set CP 0010 ; set RES 1111 ; set LFHF 11 }
            2    {set CP 0010 ; set RES 1111 ; set LFHF 11 }
            3    {set CP 0010 ; set RES 1111 ; set LFHF 11 }
            4    {set CP 0010 ; set RES 1111 ; set LFHF 11 }
            5    {set CP 0010 ; set RES 1111 ; set LFHF 11 }
            6    {set CP 0010 ; set RES 1111 ; set LFHF 11 }
            7    {set CP 0010 ; set RES 0111 ; set LFHF 11 }
            8    {set CP 0010 ; set RES 0111 ; set LFHF 11 }
            9    {set CP 0010 ; set RES 0111 ; set LFHF 11 }
            10   {set CP 0010 ; set RES 1101 ; set LFHF 11 }
            11   {set CP 0010 ; set RES 1101 ; set LFHF 11 }
            12   {set CP 0010 ; set RES 1101 ; set LFHF 11 }
            13   {set CP 0010 ; set RES 0011 ; set LFHF 11 }
            14   {set CP 0010 ; set RES 0101 ; set LFHF 11 }
            15   {set CP 0010 ; set RES 0101 ; set LFHF 11 }
            16   {set CP 0010 ; set RES 0101 ; set LFHF 11 }
            17   {set CP 0010 ; set RES 1001 ; set LFHF 11 }
            18   {set CP 0010 ; set RES 1001 ; set LFHF 11 }
            19   {set CP 0010 ; set RES 1110 ; set LFHF 11 }
            20   {set CP 0010 ; set RES 1110 ; set LFHF 11 }
            21   {set CP 0010 ; set RES 1110 ; set LFHF 11 }
            22   {set CP 0010 ; set RES 1110 ; set LFHF 11 }
            23   {set CP 0010 ; set RES 1110 ; set LFHF 11 }
            24   {set CP 0010 ; set RES 1110 ; set LFHF 11 }
            25   {set CP 0010 ; set RES 0001 ; set LFHF 11 }
            26   {set CP 0010 ; set RES 0001 ; set LFHF 11 }
            27   {set CP 0010 ; set RES 0001 ; set LFHF 11 }
            28   {set CP 0010 ; set RES 0001 ; set LFHF 11 }
            29   {set CP 0010 ; set RES 0001 ; set LFHF 11 }
            30   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            31   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            32   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            33   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            34   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            35   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            36   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            37   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            38   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            39   {set CP 0010 ; set RES 0110 ; set LFHF 11 }
            40   {set CP 0010 ; set RES 1010 ; set LFHF 11 }
            41   {set CP 0010 ; set RES 1010 ; set LFHF 11 }
            42   {set CP 0010 ; set RES 1010 ; set LFHF 11 }
            43   {set CP 0010 ; set RES 1010 ; set LFHF 11 }
            44   {set CP 0010 ; set RES 1010 ; set LFHF 11 }
            45   {set CP 0010 ; set RES 1010 ; set LFHF 11 }
            46   {set CP 0010 ; set RES 1010 ; set LFHF 11 }
            47   {set CP 0010 ; set RES 1010 ; set LFHF 11 }
            48   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            49   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            50   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            51   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            52   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            53   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            54   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            55   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            56   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            57   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            58   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            59   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            60   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            61   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            62   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            63   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
            64   {set CP 0010 ; set RES 1100 ; set LFHF 11 }
        }
    } elseif {$bw_lower == "high" || $bw_lower == "optimized" }  {
      # HIGH or OPTIMIZED
        switch -glob -- $div {
            1    {set CP 0010 ; set RES 1111 ; set LFHF 11 }
            2    {set CP 0010 ; set RES 1111 ; set LFHF 11 }
            3    {set CP 0010 ; set RES 1011 ; set LFHF 11 }
            4    {set CP 0011 ; set RES 1111 ; set LFHF 11 }
            5    {set CP 0100 ; set RES 1111 ; set LFHF 11 }
            6    {set CP 0100 ; set RES 1111 ; set LFHF 11 }
            7    {set CP 0101 ; set RES 1111 ; set LFHF 11 }
            8    {set CP 0110 ; set RES 1111 ; set LFHF 11 }
            9    {set CP 0111 ; set RES 1111 ; set LFHF 11 }
            10   {set CP 0111 ; set RES 1111 ; set LFHF 11 }
            11   {set CP 1100 ; set RES 1111 ; set LFHF 11 }
            12   {set CP 1101 ; set RES 1111 ; set LFHF 11 }
            13   {set CP 0001 ; set RES 1111 ; set LFHF 11 }
            14   {set CP 1111 ; set RES 1111 ; set LFHF 11 }
            15   {set CP 1111 ; set RES 1111 ; set LFHF 11 }
            16   {set CP 1110 ; set RES 0111 ; set LFHF 11 }
            17   {set CP 1110 ; set RES 1011 ; set LFHF 11 }
            18   {set CP 1111 ; set RES 0111 ; set LFHF 11 }
            19   {set CP 1111 ; set RES 1011 ; set LFHF 11 }
            20   {set CP 1111 ; set RES 1011 ; set LFHF 11 }
            21   {set CP 1110 ; set RES 1101 ; set LFHF 11 }
            22   {set CP 1111 ; set RES 1101 ; set LFHF 11 }
            23   {set CP 1111 ; set RES 1101 ; set LFHF 11 }
            24   {set CP 1111 ; set RES 0011 ; set LFHF 11 }
            25   {set CP 1111 ; set RES 0011 ; set LFHF 11 }
            26   {set CP 1111 ; set RES 0011 ; set LFHF 11 }
            27   {set CP 1110 ; set RES 0101 ; set LFHF 11 }
            28   {set CP 1110 ; set RES 0101 ; set LFHF 11 }
            29   {set CP 1110 ; set RES 0101 ; set LFHF 11 }
            30   {set CP 1111 ; set RES 0101 ; set LFHF 11 }
            31   {set CP 1111 ; set RES 0101 ; set LFHF 11 }
            32   {set CP 1111 ; set RES 0101 ; set LFHF 11 }
            33   {set CP 1111 ; set RES 1001 ; set LFHF 11 }
            34   {set CP 1111 ; set RES 1001 ; set LFHF 11 }
            35   {set CP 1111 ; set RES 1001 ; set LFHF 11 }
            36   {set CP 1111 ; set RES 1001 ; set LFHF 11 }
            37   {set CP 1111 ; set RES 1001 ; set LFHF 11 }
            38   {set CP 1110 ; set RES 1110 ; set LFHF 11 }
            39   {set CP 1110 ; set RES 1110 ; set LFHF 11 }
            40   {set CP 1110 ; set RES 1110 ; set LFHF 11 }
            41   {set CP 1110 ; set RES 1110 ; set LFHF 11 }
            42   {set CP 1111 ; set RES 1110 ; set LFHF 11 }
            43   {set CP 1111 ; set RES 1110 ; set LFHF 11 }
            44   {set CP 1111 ; set RES 1110 ; set LFHF 11 }
            45   {set CP 1111 ; set RES 1110 ; set LFHF 11 }
            46   {set CP 1111 ; set RES 1110 ; set LFHF 11 }
            47   {set CP 1111 ; set RES 1110 ; set LFHF 11 }
            48   {set CP 1111 ; set RES 1110 ; set LFHF 11 }
            49   {set CP 1110 ; set RES 0001 ; set LFHF 11 }
            50   {set CP 1110 ; set RES 0001 ; set LFHF 11 }
            51   {set CP 1110 ; set RES 0001 ; set LFHF 11 }
            52   {set CP 1110 ; set RES 0001 ; set LFHF 11 }
            53   {set CP 1110 ; set RES 0001 ; set LFHF 11 }
            54   {set CP 1100 ; set RES 0110 ; set LFHF 11 }
            55   {set CP 1100 ; set RES 0110 ; set LFHF 11 }
            56   {set CP 1100 ; set RES 0110 ; set LFHF 11 }
            57   {set CP 1100 ; set RES 0110 ; set LFHF 11 }
            58   {set CP 1100 ; set RES 0110 ; set LFHF 11 }
            59   {set CP 1100 ; set RES 0110 ; set LFHF 11 }
            60   {set CP 1100 ; set RES 0110 ; set LFHF 11 }
            61   {set CP 1100 ; set RES 1010 ; set LFHF 11 }
            62   {set CP 1100 ; set RES 1010 ; set LFHF 11 }
            63   {set CP 1100 ; set RES 1010 ; set LFHF 11 }
            64   {set CP 1100 ; set RES 1010 ; set LFHF 11 }
        }
    } else { # $bw_lower == "postcrc"
      # POSTCRC
        switch -glob -- $div {
            1    {set CP 0011 ; set RES 0111 ; set LFHF 00 }
            2    {set CP 0011 ; set RES 0111 ; set LFHF 00 }
            3    {set CP 0100 ; set RES 0111 ; set LFHF 00 }
            4    {set CP 0110 ; set RES 1111 ; set LFHF 00 }
            5    {set CP 0111 ; set RES 1111 ; set LFHF 00 }
            6    {set CP 1100 ; set RES 1111 ; set LFHF 00 }
            7    {set CP 1101 ; set RES 1111 ; set LFHF 00 }
            8    {set CP 1110 ; set RES 1111 ; set LFHF 00 }
            9    {set CP 1110 ; set RES 0111 ; set LFHF 00 }
            10   {set CP 1111 ; set RES 0111 ; set LFHF 00 }
            11   {set CP 1111 ; set RES 1011 ; set LFHF 00 }
            12   {set CP 1110 ; set RES 1101 ; set LFHF 00 }
            13   {set CP 1111 ; set RES 1101 ; set LFHF 00 }
            14   {set CP 1111 ; set RES 0011 ; set LFHF 00 }
            15   {set CP 1110 ; set RES 0101 ; set LFHF 00 }
            16   {set CP 1111 ; set RES 0101 ; set LFHF 00 }
            17   {set CP 1111 ; set RES 0101 ; set LFHF 00 }
            18   {set CP 1111 ; set RES 1001 ; set LFHF 00 }
            19   {set CP 1111 ; set RES 1001 ; set LFHF 00 }
            20   {set CP 1110 ; set RES 1110 ; set LFHF 00 }
            21   {set CP 1110 ; set RES 1110 ; set LFHF 00 }
            22   {set CP 1111 ; set RES 1110 ; set LFHF 00 }
            23   {set CP 1111 ; set RES 1110 ; set LFHF 00 }
            24   {set CP 1111 ; set RES 1110 ; set LFHF 00 }
            25   {set CP 1110 ; set RES 0001 ; set LFHF 00 }
            26   {set CP 1111 ; set RES 0001 ; set LFHF 00 }
            27   {set CP 1111 ; set RES 0001 ; set LFHF 00 }
            28   {set CP 1111 ; set RES 0001 ; set LFHF 00 }
            29   {set CP 1111 ; set RES 0001 ; set LFHF 00 }
            30   {set CP 1111 ; set RES 0001 ; set LFHF 00 }
            31   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            32   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            33   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            34   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            35   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            36   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            37   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            38   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            39   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            40   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            41   {set CP 1101 ; set RES 1010 ; set LFHF 00 }
            42   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            43   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            44   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            45   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            46   {set CP 0111 ; set RES 1100 ; set LFHF 00 }
            47   {set CP 1110 ; set RES 0110 ; set LFHF 00 }
            48   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            49   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            50   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            51   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            52   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            53   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            54   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            55   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            56   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            57   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            58   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            59   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            60   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            61   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            62   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            63   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
            64   {set CP 0110 ; set RES 0010 ; set LFHF 00 }
        }
    }
    puts "DADDR_4F: [xapp888_bin2hex "[string index $RES 0]00[string range $RES 1 2]00[string index $RES 3][string index $LFHF 0]00[string index $LFHF 1]0000"]\t-Filter Register 1: M set to $div with $bw bandwidth-"
    puts "DADDR_4E: [xapp888_bin2hex "[string index $CP 0]00[string range $CP 1 2]00[string index $CP 3]00000000"]\t-Filter Register 2: M set to $div with $bw bandwidth-"
    return "4F [xapp888_bin2hex "[string index $RES 0]00[string range $RES 1 2]00[string index $RES 3][string index $LFHF 0]00[string index $LFHF 1]0000"] 4E [xapp888_bin2hex "[string index $CP 0]00[string range $CP 1 2]00[string index $CP 3]00000000"]"
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_locking {div} {
        # LockRefDly_LockFBDly_LockCnt_LockSatHigh_UnlockCnt
        set div [scan $div %d]
        switch -glob -- $div {
            1 {set LockRefDly 00110 ; set LockFBDly 00110 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            2 {set LockRefDly 00110 ; set LockFBDly 00110 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            3 {set LockRefDly 01000 ; set LockFBDly 01000 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            4 {set LockRefDly 01011 ; set LockFBDly 01011 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            5 {set LockRefDly 01110 ; set LockFBDly 01110 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            6 {set LockRefDly 10001 ; set LockFBDly 10001 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            7 {set LockRefDly 10011 ; set LockFBDly 10011 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            8 {set LockRefDly 10110 ; set LockFBDly 10110 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            9 {set LockRefDly 11001 ; set LockFBDly 11001 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            10 {set LockRefDly 11100 ; set LockFBDly 11100 ; set LockCnt 1111101000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            11 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 1110000100;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            12 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 1100111001;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            13 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 1011101110;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            14 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 1010111100;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            15 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 1010001010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            16 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 1001110001;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            17 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 1000111111;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            18 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 1000100110;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            19 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 1000001101;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            20 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0111110100;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            21 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0111011011;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            22 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0111000010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            23 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0110101001;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            24 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0110010000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            25 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0110010000;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            26 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0101110111;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            27 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0101011110;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            28 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0101011110;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            29 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0101000101;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            30 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0101000101;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            31 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0100101100;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            32 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0100101100;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            33 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0100101100;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            34 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0100010011;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            35 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0100010011;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            36 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0100010011;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            37 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            38 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            39 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            40 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            41 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            42 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            43 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            44 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            45 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            46 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            47 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            48 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            49 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            50 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            51 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            52 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            53 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            54 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            55 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            56 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            57 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            58 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            59 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            60 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            61 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            62 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            63 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
            64 {set LockRefDly 11111 ; set LockFBDly 11111 ; set LockCnt 0011111010;set LockSatHigh 1111101001 ;set UnlockCnt 0000000001}
         }
#        puts "debug: $LockRefDly\_$LockFBDly\_$LockCnt\_$LockSatHigh\_$UnlockCnt"
        puts "DADDR_27: FFFF\t-Power register leaving all interpolators on - "
        puts "DADDR_18: [xapp888_bin2hex 000000$LockCnt]\t-Lock Register 1: for M set to $div -"
        puts "DADDR_19: [xapp888_bin2hex 0$LockFBDly$UnlockCnt]\t-Lock Register 2: for M set to $div"
        puts "DADDR_1A: [xapp888_bin2hex 0$LockRefDly$LockSatHigh]\t-Lock Register 3: for M set to $div"

        return "27 FFFF 18 [xapp888_bin2hex 000000$LockCnt] 19 [xapp888_bin2hex 0$LockFBDly$UnlockCnt] 1A [xapp888_bin2hex 0$LockRefDly$LockSatHigh]"
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_bin2hex {bits} {
    set abits ""
    for {set i 0} {$i <= [expr 15 - [string length $bits]] } {incr i} {
        append abits 0}
    append abits "$bits"
    set binValue [binary format B16 $abits]
    binary scan $binValue H4 hex
    return $hex
    }
#--------------------------------------------------------------------------------------------
#
proc xapp888_hex2bin {hex} {
    for {set i 0} { $i <= [string length $hex]} { incr i 1} {
        append convert2bin [string map -nocase {
            0 0000 1 0001 2 0010 3 0011 4 0100 5 0101 6 0110 7 0111
            8 1000 9 1001 a 1010 b 1011 c 1100 d 1101 e 1110 f 1111
        } [string index $hex $i ] ]
    }
    return $convert2bin
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_dec2hex {value} {
    # Creates a 16 bit hex number from a signed decimal number
    # Replace all non-decimal characters
    regsub -all {[^0-9\.\-]} $value {} newtemp
    set value [string trim $newtemp]
    if {$value < 65535 && $value >= 0} {
        set tempvalue [format "%#010X" [expr $value]]
        return [string range $tempvalue 6 9]
    } elseif {$value < 0} {
        puts "Unsigned value"
        return "0000"
    } else {
        puts "Violates 16 bit range"
        return "FFFF"
    }
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_drp_settings {m d phase bw} {
    if {$phase < 0} {set phase [expr 360 + $phase]}
    set data_m [xapp888_drp_calc_m $m $phase]
    set data_d [xapp888_drp_calc_d $d]
    set data_cpres [xapp888_cpres $m $bw]
    set data_locking [xapp888_locking $m]
    return "$data_m $data_d $data_cpres $data_locking"
}
proc xapp888_dec2bin {dec bits} {
    return [binary scan [binary format I $dec] B32 var;string range $var end-[expr $bits-1] end]
}
#--------------------------------------------------------------------------------------------
#
proc xapp888_help {} {
    puts "\n\n   -------------------------------------------------------------------------------------\
    \n   NOTES:\n   The following TCL commands are being added to give example calculations\
    \n   for drp programming values for MMCME3 (Ultrascale). This script can be altered\
    \n   to adjust for those differences but exact settings should be reviewed.\
    \n\n   Also please note that this is a subset of the full programming options.\
    \n   Fine phase shifting and dynamic phase shifting is not directly supported by\
    \n   these scripts.\
    \n      xapp888_drp_settings <CLKFBOUT_MULT> <DIVCLK_DIVIDE> <PHASE> <BANDWIDTH>\
    \n           - BANDWIDTH can be set to LOW, HIGH or OPTIMIZED (case insensitive)
    \n           - Displays & Returns the ordered pairs of DRP addresses & Data
    \n      xapp888_drp_clkout <DIVIDE>  <Duty Cycle e.g. 0.5> <Phase e.g.11.25> <CLKOUT0 to CLKOUT6> \
    \n           - Displays & Returns per output the ordered pairs of DRP addresses & Data
    \n      xapp888_help <>
    \n\n   For Example:\
    \n\t   xapp888_drp_settings <m> <d> <phase> <bw>;\
    \n\t   xapp888_drp_clkout <div> <dc> <phase> clkout0;\
    \n\t   xapp888_drp_clkout <div> <dc> <phase> clkout1;\
    \n\t   xapp888_drp_clkout <div> <dc> <phase> clkout2;\
    \n\t   xapp888_drp_clkout <div> <dc> <phase> clkout3;\
    \n\t   xapp888_drp_clkout <div> <dc> <phase> clkout4;\
    \n\t   xapp888_drp_clkout <div> <dc> <phase> clkout5;\
    \n\t   xapp888_drp_clkout <div> <dc> <phase> clkout6;\
    \n\n   To show how to use the xapp888_merg_drp command the following is an arbitrary example"
    puts {          set drp "[xapp888_drp_settings 2.125 2 0 high]"}
    puts {          set drp "$drp [xapp888_drp_clkout 3.75 0.5 90 clkout0]"}
    puts {          for {set i 1} {$i <= 6} {incr i} {set drp "$drp [xapp888_drp_clkout 7 0.5 90 clkout$i]"} }
    puts {          xapp888_merge_drp $drp}
    puts "   -------------------------------------------------------------------------------------"
}
#--------------------------------------------------------------------------------------------
#
xapp888_help
#
#--------------------------------------------------------------------------------------------
#