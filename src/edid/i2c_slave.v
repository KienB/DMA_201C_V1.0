/****************************************************************************
I2C写寄存器的标准流程为：
1. Master发起START
2. Master发送I2C addr（7bit）和w操作0（1bit），等待ACK
3. Slave发送ACK
4. Master发送reg addr（8bit），等待ACK
5. Slave发送ACK
6. Master发送data（8bit），即要写入寄存器中的数据，等待ACK
7. Slave发送ACK
8. 第6步和第7步可以重复多次，即顺序写多个寄存器
9. Master发起STOP

I2C读寄存器的标准流程为：
1. Master发起START
2. Master发送I2C addr（7bit）和w操作0（1bit），等待ACK
3. Slave发送ACK
4. Master发送reg addr（8bit），等待ACK
5. Slave发送ACK
6. Master发起START
7. Master发送I2C addr（7bit）和r操作1（1bit），等待ACK
8. Slave发送ACK
9. Slave发送data（8bit），即寄存器里的值
10.Master如果继续读发送ACK，否则发送NACK
11.第9步和第10步（最后一次ACK Master发送NACK）可以重复多次，即顺序读多个寄存器
12.Master发起STOP
******************************************************************************/
module i2c_slave(
 input			       iSclk
,input  		       iRstN

,input         [7:0]   iDeviceAddr
,output reg 	       oWrEn
,output        [7:0]   oWrAddr
,output        [7:0]   oWrData

,input                 iValid
,output reg    [7:0]   oRdAddr
,input         [7:0]   iRdData
,output reg            oRdDe

,input			       iSda
,input			       iScl
,output			       oSda

,output        [4:0]   oState
);


parameter  	IDLE		= 'd0;
parameter	START		= 'd1;
parameter   DEV_ADDR    = 'd2;
parameter	DEV_ACK     = 'd3;
parameter	REG_ADDR    = 'd4;
parameter	WR_DATA		= 'd5;
parameter	RD_DATA		= 'd6;
parameter   DATA_ACK    = 'd7;
parameter	STOP	    = 'd8;

////////////////////////////////////////////////////////////////
reg  [4:0]  rCState         ;
reg  [4:0]  rNState         ;

reg				rWrEn		;
reg		[7:0]	rStartCnt	;	
reg				rSCL		;
reg				rSDA		;

reg             rWrRdFlag   ;
reg             rDevAddrChek;
reg             rAckFlag    ;

reg		[7:0]	rSlv_addrTemp	;
reg		[7:0]	rReg_addrTemp	;
reg		[7:0]	rWrDataTemp		;
reg		[7:0]	rSlv_addr	;		
reg		[7:0]	rReg_addr	;		
reg		[7:0]	rWrData		;
reg		[7:0]	rWrAddr		;
reg		[7:0]	rRdAddr		;
reg		     	rWrDone		;
reg		     	rRdDe  		;
reg		     	rIsAddr_Data;
reg		     	rRdEn       ;

reg		[7:0]	rWrDataShift ;
reg		[7:0]	rWrDataShiftD;

reg		[7:0]	rRdData		;
reg				rRdValid	;

reg     [7:0]   rBitCnt     ;

reg     [11:0]  rSampleTimeCnt ;
reg             rSampleEn ;
reg             rSampleEnD;

reg             rOverTimeEn   ;
reg     [11:0]  rOverTime     ;//超时
reg             rOverTimeFlag ;//超时标志

reg    rSda_pos    ;
reg    rSda_neg    ;

reg    [5:0]    rScl_PosD;
reg    [5:0]    rScl_NegD;

reg  [2:0]   rSdaD ;

reg    rScl_pos    ;
reg    rScl_neg    ;

reg    rStartFlag  ;
reg    rStopFlag   ;

reg  [2:0]   rScl ;




///////////////////////////////////////////////////////////////////
assign			oSda      = rSDA     ;
assign			oState    = rCState  ;
//assign          oWrEn     = rWrEn && (rCState==DATA_ACK) && rScl_NegD[0]   ;
assign          oWrData   = rWrData  ;
assign          oWrAddr   = rWrAddr  ;
assign          oWrDone   = rWrDone  ;
//assign          oRdDe     = ((rCState==DATA_ACK) && rScl_NegD[4] && rWrRdFlag);

////////////////////////////////////////////////////////////////
always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rSdaD			<= 'd0;
	else 
		rSdaD			<= {rSdaD[1:0],iSda};
end	

always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rSda_pos			<= 'd0;
	else if(rSdaD[2:1]==2'b01)
		rSda_pos			<= 'd1;
	else 
		rSda_pos			<= 'd0;	
end	

always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rSda_neg			<= 'd0;
	else if(rSdaD[2:1]==2'b10)
		rSda_neg			<= 'd1;
	else 
		rSda_neg			<= 'd0;	
end	

always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rScl			<= 'd0;
	else 
		rScl			<= {rScl[1:0],iScl};
end	

always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rScl_pos			<= 'd0;
	else if(rScl[1:0]==2'b01)
		rScl_pos			<= 'd1;
	else 
		rScl_pos			<= 'd0;	
end	

always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rScl_neg			<= 'd0;
	else if(rScl[1:0]==2'b10)
		rScl_neg			<= 'd1;
	else 
		rScl_neg			<= 'd0;	
end

always @(posedge iSclk or negedge iRstN) begin	
	if(iRstN!=1'b1) begin
		rScl_PosD			<= 'd0;
		rScl_NegD			<= 'd0;
	
	end
	else begin
		rScl_PosD			<= {rScl_PosD[4:0], rScl_pos};	
		rScl_NegD			<= {rScl_NegD[4:0], rScl_neg};	
		
	end
end


always@(posedge iSclk or negedge iRstN) begin
	if(!iRstN) begin
		rStartFlag   <= 'd0;
		rStopFlag    <= 'd0;
	end
	else begin
		rStartFlag   <= rSda_neg && rScl[2];
		rStopFlag    <= rSda_pos && rScl[2];
	end
end


always @(posedge iSclk or negedge iRstN)begin
	if(iRstN!=1'b1)
		rSampleTimeCnt  <= 'd0;
	else if(rScl_pos)
		rSampleTimeCnt  <= 'd0;
	else if(rScl[1] == 'd1)
		if(rSampleTimeCnt >= 'd3)
			rSampleTimeCnt  <= rSampleTimeCnt;
		else
			rSampleTimeCnt  <= rSampleTimeCnt+'d1;
	else ;
end
always @(posedge iSclk or negedge iRstN)begin
	if(iRstN!=1'b1)
		rSampleEn  <= 'd0;
	else if(rScl_pos)
		rSampleEn  <= 'd0;
	else if(rScl[1] == 'd1)
		if(rSampleTimeCnt == 'd2)
			rSampleEn  <= 'd1;
		else
			rSampleEn  <= 'd0;
	else ;
end
always @(posedge iSclk or negedge iRstN)begin
	if(iRstN!=1'b1)
		rSampleEnD  <= 'd0;
	else
		rSampleEnD  <= rSampleEn;
end
always @(posedge iSclk or negedge iRstN)begin
	if(iRstN!=1'b1)
		rOverTime  <= 'd0;
	else if(rScl_pos)
		rOverTime  <= 'd0;
	else if(rOverTimeEn)
		rOverTime  <= rOverTime + 'd1;
	else
		rOverTime  <= rOverTime;
end

always @(posedge iSclk or negedge iRstN)begin
	if(iRstN!=1'b1)
		rOverTimeFlag  <= 'd0;
	else if(&rOverTime)
		rOverTimeFlag  <= 'd1;
	else
		rOverTimeFlag  <= 'd0;
end


always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rRdValid			<= 'd0;
	else 
		rRdValid			<= iValid;
end

always @(posedge iSclk or negedge iRstN) begin	
	if (iRstN!=1'b1)
		oRdAddr			<= 'd0;
	else if((rNState==DEV_ACK  || rNState==DATA_ACK) && rScl_NegD[0] && rWrRdFlag)
		oRdAddr			<= rRdAddr;
	else ;
end

always @(posedge iSclk or negedge iRstN) begin	
	if (iRstN!=1'b1)
		oRdDe			<= 'd0;
	else if((rNState==DEV_ACK || rNState==DATA_ACK) && rScl_NegD[0] && rWrRdFlag)
		oRdDe			<= 'd1;
	else
		oRdDe			<= 'd0;
end

always @(posedge iSclk or negedge iRstN) begin	
	if (iRstN!=1'b1)
		oWrEn			<= 'd0;
	else if((rBitCnt == 'd8) && (rNState==WR_DATA) && rSampleEnD)
		oWrEn			<= 'd1;
	else
		oWrEn			<= 'd0;
end
always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rWrDataShift		<= 'd0;
	else if(rSampleEn)
		rWrDataShift		<= {rWrDataShift[6:0],rSdaD[2]};
	else	;
end

always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rWrDataShiftD		<= 'd0;
	else
		rWrDataShiftD		<= rWrDataShift;
end

always @ (posedge iSclk or negedge iRstN)
begin	
	if (iRstN!=1'b1)
		rRdData				<= 'd0;
	else if(oRdDe)
		rRdData				<= iRdData;
	else if(rNState == RD_DATA && rScl_NegD[0])
			rRdData			<= {rRdData[6:0],1'b0};
	else	;
end

always @(posedge iSclk or negedge iRstN) begin
	if (iRstN!=1'b1) begin
		rSDA        <= 1'b1;
	end
	else if(rNState==DEV_ACK) begin
		if(rScl_NegD[0])
			rSDA        <= 1'b0;
		else
			rSDA        <= rSDA;
	end
	else if(rNState==RD_DATA) begin
		if(rScl_NegD[0])
			rSDA          <= rRdData[7];
		else
			rSDA        <= rSDA;
	end
	else if(rNState==DATA_ACK) begin
		if(!rWrRdFlag && rScl_neg)
			rSDA        <= 1'b0;
		else
			rSDA        <= rSDA;
	end
	else
		rSDA        <= 1'b1;
end
always @(posedge iSclk or negedge iRstN) begin
	if (iRstN!=1'b1)
		rCState <= IDLE     ;
	else
		rCState <= rNState  ;
end
always @(*) begin
	if (iRstN!=1'b1)
		rNState = IDLE;
	else case(rCState)
		IDLE: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else begin
				rNState = IDLE;
			end
		end
		START: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else if(rStopFlag) begin
				rNState = IDLE;
			end
			else if(rScl_neg) begin
				rNState = DEV_ADDR;
			end
			else begin
				rNState = START;
			end
		end
		DEV_ADDR: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else if(rStopFlag) begin
				rNState = IDLE;
			end
			else if(rBitCnt == 'd8 && rScl_NegD[0]) begin
				if(rWrDataShift[7:1] == iDeviceAddr[7:1]) begin 
					rNState = DEV_ACK;
				end
				else begin
					rNState = IDLE;
				end
			end
			else begin
				rNState = DEV_ADDR;
			end
		end
		DEV_ACK: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else if(rStopFlag) begin
				rNState = IDLE;
			end
			else if(rWrRdFlag && rScl_neg) begin
				rNState = RD_DATA;
			end
			else if(!rWrRdFlag && rScl_neg) begin
				rNState = REG_ADDR;
			end
			else if(!rDevAddrChek) begin
				rNState = IDLE;
			end
/* 			else if(rOverTimeFlag) begin
				rNState = IDLE;
			end */
			else begin
				rNState = DEV_ACK;
			end
		end
		REG_ADDR: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else if(rStopFlag) begin
				rNState = IDLE;
			end
			else if(rScl_neg && (rBitCnt == 'd8)) begin
				rNState = DATA_ACK;
			end
			else begin
				rNState = REG_ADDR;
			end
		end
		WR_DATA: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else if(rStopFlag) begin
				rNState = IDLE;
			end
			else if((rBitCnt == 'd8) && rScl_neg) begin
				rNState = DATA_ACK;
			end
			else begin
				rNState = WR_DATA;
			end
		end
		RD_DATA: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else if(rStopFlag) begin
				rNState = IDLE;
			end
			else if(!rWrRdFlag && rScl_neg) begin
				rNState = IDLE;
			end
			else if((rBitCnt == 'd8) && rScl_neg) begin
				rNState = DATA_ACK;
			end
			else begin
				rNState = RD_DATA;
			
			end
		end
		DATA_ACK: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else if(rStopFlag) begin
				rNState = IDLE;
			end
			else if(!rWrRdFlag && rScl_neg) begin
				rNState = WR_DATA;
			end
			else if(rWrRdFlag) begin
				if(rSampleEn) begin
					if(!rSdaD[2])
						rNState = RD_DATA;
					else
						rNState = STOP;
				end
				else begin 
					rNState = DATA_ACK;
				end
			end
			else
				rNState = DATA_ACK;
		end
		STOP: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else if(rStopFlag) begin
				rNState = IDLE;
			end
			else begin
				rNState = IDLE;
			end
		end
		default: begin
			if(rStartFlag) begin
				rNState = START;
			end
			else if(rStopFlag) begin
				rNState = IDLE;
			end
			else begin
				rNState =  IDLE;
			end
		end
	endcase
end

always @(posedge iSclk or negedge iRstN) begin
	if (iRstN!=1'b1) begin
		
		rWrRdFlag     <= 'd0;
		rBitCnt       <= 'd0;
		rDevAddrChek  <= 'd0;
		rAckFlag      <= 'd0;
		rWrDone       <= 'd0;
		rWrEn         <= 'd0;
		rIsAddr_Data  <= 'd0;
		rWrAddr       <= 'd0;
		rRdAddr       <= 'd0;
		rWrData       <= 'd0;
		rRdEn         <= 'd0;
		rRdDe         <= 'd0;
	end
	else case(rNState)
		IDLE:begin
		
			rWrRdFlag     <= 'd0;
			rBitCnt       <= 'd0;
			rDevAddrChek  <= 'd0;
			rAckFlag      <= 'd0;
			rWrDone       <= 'd0;
			rWrEn         <= 'd0;
			rIsAddr_Data  <= 'd0;
			rWrAddr       <= 'd0;
			rRdAddr       <= 'd0;
			rWrData       <= 'd0;
			rRdDe         <= 'd0;
		end
		START: begin
			rWrRdFlag     <= 'd0;
			rBitCnt       <= 'd0;
			rDevAddrChek  <= 'd0;
			rAckFlag      <= 'd0;
			rWrDone       <= 'd0;
			rWrEn         <= 'd0;
			rIsAddr_Data  <= 'd0;
			rRdDe         <= 'd0;
		end
		DEV_ADDR: begin
			if(rBitCnt == 'd8 && rScl_neg) begin
				if(rWrDataShift[7:1] == iDeviceAddr[7:1]) begin
					rWrRdFlag     <= rWrDataShift[0];
					rDevAddrChek  <= 'd1;
					rAckFlag      <= 'd0;
					rWrEn         <= 'd0;
					
				end
				else ;
			end
			// else ;
			// if(rBitCnt == 'd8 && rScl_neg) begin
				// rBitCnt       <= 'd0;
			// end
			else if(rSampleEn) begin
				rBitCnt       <= rBitCnt + 'd1;
			end
			else ;
		end
		DEV_ACK: begin
			rBitCnt       <= 'd0;
			if(!rWrRdFlag && rScl_NegD[0]/* &&  !rAckFlag */) begin
				rAckFlag        <= 'd1;
				rIsAddr_Data    <= 'd1 ;//1:reg address, 0:data
			end
			else if(rWrRdFlag && rScl_NegD[0]/* &&  !rAckFlag */) begin
				rIsAddr_Data    <= 'd1 ;//1:reg address, 0:data
				rAckFlag        <= 'd1;
				rRdEn           <= 'd1 ;
			end
			else ;
		end
		REG_ADDR: begin
			if(rBitCnt=='d8 && rSampleEnD) begin
				rIsAddr_Data  <=  'd0;
				if(rIsAddr_Data == 'd1) begin//从ACK跳过来的，说明第一次接收信息，第一字节是寄存器地址
					/* if(!rWrRdFlag)
						rWrAddr <= rWrDataShift;
					else begin
						rRdDe   <= 'd1;
						rRdAddr <= rWrDataShift;
					end */
					rWrAddr <= rWrDataShift;
					rRdAddr <= rWrDataShift;
					rRdDe   <= 'd1;
				end
				else ;
			end
			else if(!rWrRdFlag && rSampleEn/*  && (rBitCnt<'d8) */) begin
				rBitCnt       <=  rBitCnt + 'd1;
			end
		end
		WR_DATA: begin
			if(rStopFlag) begin
				rWrDone  <= 'd1;
			end
			else if(rBitCnt=='d8 && rSampleEnD) begin
				rIsAddr_Data  <=  'd0;
				
				if(rIsAddr_Data == 'd0) begin
					rWrDone <= 'd1;
					rWrEn   <= 'd1;
					rWrData <= rWrDataShift;
					/* if(&rWrAddr)//地址超过255
						rWrAddr <= rWrAddr;
					else
						rWrAddr <= rWrAddr +'d1;//顺序写多个寄存器 */
				end
				else ;
			end
			else if(!rWrRdFlag && rSampleEn/*  && (rBitCnt<'d8) */) begin
//				rSampleEn     <=  'd0;
				rBitCnt       <=  rBitCnt + 'd1;
			end
			else ;
		end
		RD_DATA: begin
			rIsAddr_Data  <=  'd0;
			if(rBitCnt=='d8) begin
				
			end
			else if(rScl_NegD[0]) begin
				rRdEn         <= 'd0;
				rBitCnt       <= rBitCnt + 'd1;
				
			end
			else ;
		end
		DATA_ACK: begin
			rWrEn         <= 'd0;
			if(!rWrRdFlag && rScl_neg) begin
				rAckFlag      <= 'd1;
				rIsAddr_Data  <= 'd0;
				rBitCnt       <= 'd0;
				if(rWrEn)
					if(&rWrAddr)//地址超过255
						rWrAddr <= rWrAddr;
					else
						rWrAddr <= rWrAddr +'d1;//顺序写多个寄存器
				else ;
			end
			else if(rWrRdFlag && rScl_NegD[0]) begin
				rBitCnt       <= 'd0;
				rAckFlag      <= 'd1;
				rRdEn         <= 'd1;
				rIsAddr_Data  <= 'd0;
				if(rIsAddr_Data == 'd0)
					if(&rRdAddr)//地址超过255
						rRdAddr <= rRdAddr;
					else begin
						rRdDe   <= 'd1;
						rRdAddr <= rRdAddr +'d1;
					end
				else ;
			end
			else ;
		end
		STOP: begin
		
			rWrEn         <= 'd0;
			rWrRdFlag     <= 'd0;
			rBitCnt       <= 'd0;
			rDevAddrChek  <= 'd0;
			rAckFlag      <= 'd0;
			rIsAddr_Data  <= 'd0;
			rWrDone       <= 'd0;
			rRdEn         <= 'd0;
			rWrData       <= 'd0;
			rRdDe         <= 'd0;
		end
		default: begin
		
			rWrEn         <= 'd0;
			rWrRdFlag     <= 'd0;
			rBitCnt       <= 'd0;
			rDevAddrChek  <= 'd0;
			rAckFlag      <= 'd0;
			rIsAddr_Data  <= 'd0;
			rWrDone       <= 'd0;
			rWrData       <= 'd0;
			rRdDe         <= 'd0;
		end
	endcase
end


endmodule


