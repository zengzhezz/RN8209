//----------------------start RN8209  地址定义---------------------------------------------------//
#define					ADSYSCON 0x00 
#define        	ADEMUCON 0x01
#define        	ADHFConst     	0x02 
#define        	ADPStart      	0x03 
#define        	ADDStart      	0x04 
#define					ADGPQA        	0x05 
#define        	ADGPQB        	0x06 
#define        	ADPhsA        	0x07 
#define        	ADPhsB        	0x08
#define					ADQPHSCAL				0x09    
#define					ADAPOSA 				0x0a
#define        	ADAPOSB 				0x0b
#define        	ADRPOSA 				0x0c
#define        	ADRPOSB 				0x0d
#define        	ADIARMSOS     	0x0e
#define        	ADIBRMSOS     	0x0f
#define        	ADIBGain      	0x10
#define					ADD2FPL       	0x11
#define        	ADD2FPH       	0x12
#define        	ADDCIAH       	0x13
#define        	ADDCIBH       	0x14
#define         ADDCUH		0x15   
#define         ADDCL   	0x16 
#define         ADEMUCON2	0x17
#define					ADPFCnt    	0x20
#define        	ADDFcnt    	0x21
#define        	ADIARMS       	0x22
#define        	ADIBRMS       	0x23
#define        	ADURMS        	0x24
#define					ADUFreq       	0x25
#define        	ADPowerPA     	0x26
#define        	ADPowerPB     	0x27
#define         ADEnergyP  	0x29
#define         ADEnergyP2 	0x2a
#define         ADEnergyD  	0x2b
#define         ADEnergyD2    	0x2c
#define         ADEMUStatus   	0x2d
#define         ADSPL_IA      	0x30
#define         ADSPL_IB      	0x31
#define         ADSPL_U       	0x32
#define         ADIE  		0x40
#define         ADIF  		0x41
#define         ADRIF    	0x42
#define         ADSysStatus  	0x43
#define         ADRData      	0x44
#define         ADWData      	0x45
#define         ADDeviceID   	0x7f
#define         WriteEn   	0xea
//----------------------end RN8209  地址定义-----------------------------------------------//
//---start 校表参数文件-------（可根据计量芯片更改）
typedef struct 
{		
	u16			SYSCON;
	u16			EMUCON;
	u16			HFConst;
	u16			PStart;	
	u16			QStart;  			//10
	u16			GPQA;    	
	u16			GPQB;    	
	u16			IAGain;  	
	u16			UGain;   	
	u16			IBGain;  	
	u16			PhsA;	   	
	u16			PhsB;    	
	u16			QPhsCal; 			//22
	u16			APOSA;   	
	u16			APOSB;	 	
	u16			RPOSA;   	
	u16			RPOSB;   	
	u16			IARMSOS; 			//32
	u16			IBRMSOS;			//34
	u16			EMUCON2; 	
	float		KUrms;								// 电压系数
	float		KIArms;								// A通道电流系数
	float		KIBrms;								// B通道电流系数
	float		KPrms;								// 功率系数
	u16			RealUI[2];						// 功率显示值，功率大于此值时显示 0.2%
	u32			RealPw;								// 电流显示值，电流大于此值时显示 0.2%
	u32			ChkSum;             	
	u16			RTCDota0;							// RTC校正寄存器
	u8			TemperAdj[2];					// 高低温补偿值
	u8			RTCAdj[4];						// RTC高低温时补偿值
	u8			CurAdj;								// 自热影响补偿值
	u8 			OfsetAdjAcVolt[2]; 		//根据电压调整OFFSET的值
	u16			CorrectionTemper;  		//校表时刻表计的温度
}sDl645FirmParaFile_TypeDef;		//58 Byte
//---end 校表参数文件-------（可根据计量芯片更改） 

//---start 计量值瞬时值中转文件-------
typedef struct
{
  u8		ChkErrCnt;
  u32 	Pw[2];   		    //pa,pb   
  u32 	UI[3];          // Ia=UI[0] Inal U         
  u16 	Frequency;   		//电网频率，单位：                            	
  u32		Pulse;		    	//前台脉冲
  u16     Pstart;
  //---电能脉冲---	
  u32		Pulse_Eg;	    	//脉冲个数
  u32 	PDirect;				//功率方向
  u32 	ChkSum1;				//读出EMU的校表参数校验
  // 校表使用参数
  u16		RatintU;				// 额定电压
	u16		RatingI;				// 额定电流
  u32		TempU;					// 当前校正点电压
  u32		TempI;					// 当前校正点电流
  u32		TempPw;					// 当前校正点功率
} sDl645FrontTmp_TypeDef;
//---end 计量值瞬时值中转文件-------

//---start 计量值瞬时值文件-------
typedef struct
{	
	struct sDl645FrontPubData_TypeDef  
	{
		u16			U;			    		//---电压---NNN.N
		u32	    Ia;			    		//---电流NNNN.NNNN(电流值要求3整3小，整定值要求2整4小，最高位表示方向)---		
		u32	    In;         		//---零线电流
		sDF09		Pw;			    		//---瞬时有功p
		u16			Pf;			    		//---功率因数N.NNN---	最高位表示方向{Pf Pfa Pfb Pfc}	
		u16			Angle;		  		//---相角NNN.N---		
		u16			Frequency;			//---频率NN.NN
		u32			PPwave;					//---一分钟平均功率NN.NNNN
		u8      Chnsel;     		
		u16			Temperature;		//---NNN.N  温度
		u16			ClockBat;				//---NN.NN  电池电压
		u32			tWorkBat;				//---NNNN  时钟工作时间（分钟）
		u8			PDirect;				//---原功率方向
		                    		
	  u16    	CfIn; 					//脉冲输入电平判断
	  u8    	CfTime;					//
	  u8    	Step;       		
	  u16   	FrontStamp; 		
	  u16			tMaxI;					// 最大电流持续时间，0.5s为单位
	  u8			SafeCurFlag;		// 电流门限不为零标志
	} PubData;
	
	struct sDl645FrontPriData_TypeDef  
	{		
		u8			Flag;						//---工作异常标志---
	} PriData;	
	
	struct sDl645FrontPriPara_TypeDef  
	{		
		u32	 PConstE;						//有功常数
		u16	 Crc;
	} PriPara;		
} sDl645Front_TypeDef;	
//---end 计量值瞬时值文件-------

//---start 串口通讯变量文件-------
typedef struct
{
	u16		EFlag;							//通讯状态
	
  u16 	RxLen;							//接收数据长度
  u16  	TxLen;
  u32		TimeOutStamp;				//发送数据长度
  u8 		*pTx;
  
  u8		fBps;								//波特率变更标志
  u8		NewBps;							//新波特率
  u32 	NewBpsStamp;				//新速率时标
//u8 		TxAddr;
  
  u8 		RxBuf[MAX_COMPACK_SIZE];//接收缓存
  u8		TxBuf[MAX_COMPACK_SIZE];//发送缓存
}sComPack_TypeDef;
//---end 通讯变量文件-------
/*****************************************************************************
** Function name:	fnUSART_RN8209_Init(u8 Cfg)
**
** Description:		计量UART初始化（波特率、数据字节格式控制、发送、接收引脚配置），主控芯片初始化时执行
**
** Parameters:		波特率
**
** Returned value:	NONE
**
******************************************************************************/
#define PinMode_RN8209RX(x) 		( (x == GPIO_MODE_OUT) ? (GPIO->PMA &= 0xfbffffff) : (GPIO->PMA |= 0x04000000) )	// P3.2
#define PinMode_RN8209TX(x) 		( (x == GPIO_MODE_OUT) ? (GPIO->PMA &= 0xf7ffffff) : (GPIO->PMA |= 0x08000000) )	// P3.3
void fnUSART_RN8209_Init(u8 Cfg)
{
	u32	ClkDiv;
	ClkDiv = fnDl645MainClock_Get();//获取系统当前时钟频率
	switch(Cfg & 0xE0)
	{
		case USART_BPS_600:		//600
			ClkDiv=ClkDiv/(600*16)-1;//根据系统当前时钟频率与串口通讯速率，计算UART2->BAUD值
			break;
	  case USART_BPS_1200:	//1200
			ClkDiv=ClkDiv/(1200*16)-1;
			break;
		case USART_BPS_2400:	//2400
			ClkDiv=ClkDiv/(2400*16)-1;
			break;
		case USART_BPS_4800:	//RN8209C仅支持波特率：4800
			ClkDiv=ClkDiv/(4800*16)-1;
			break;
		case USART_BPS_9600:	//9600
			ClkDiv=ClkDiv/(9300*16)-1;
			break;
		default:	
			ClkDiv=ClkDiv/(2400*16)-1;			//2400
			break;
	}
	//UART2用于RN8209通讯.
	UART2->BAUD = ClkDiv;
	UART2->CTRL = (3 << 0)	|   /* uart enable 使能*/
                  (3 << 6)	|	/* data bit: 8 数据位*/
                  (2 << 8);	/* even  偶校验*/
	PinMode_RN8209RX(GPIO_MODE_IN);//MCU串口接收引脚配置
	PinMode_RN8209TX(GPIO_MODE_OUT);//MCU串口发送引脚配置
	return;
}

/***************************************************************************** 
** Function name:	USART_ITConfig(u8 ComPort, FunctionalMODE USART_IT, FunctionalState NewState)                                  
**                                                                             
** Description:		端口管理及工作方式控制函数
**                                                                             
** Parameters:		ComPort：串口选择（串口0、1、2、3）；  USART_IT：串口工作方式选择（接收或发送）；
**								NewState：串口状态选择（使能或关闭）                                                     
**                                                                             
** Returned value:	NONE                                                       
**                                                                             
******************************************************************************/

void USART_ITConfig(u8 ComPort, FunctionalMODE USART_IT, FunctionalState NewState)
{
	u8 Compose;	
	Compose=(ComPort<<2)|(USART_IT<<1)|(NewState);
	switch(Compose)
	{

	//----------------RN8209-----------------------------
	case (SCOM_PORT_RN8209<<2)|(USART_IT_RX<<1)|DISABLE:	//RN8209通信口|接收|关闭
		UART2->CTRL &=0xffe7; 	//屏蔽接收中断,屏蔽接收错误中断
		UART2->STA = 0x3d;		//清接收中断标志及接收错误标志
		break;
	case (SCOM_PORT_RN8209<<2)|(USART_IT_RX<<1)|ENABLE:	//RN8209通信口|接收|打开
		UART2->CTRL |=0x18; 	//允许接收中断,允许接收错误中断
		UART2->STA = 0x3d;		//清接收中断标志及接收错误标志
		break;
	case (SCOM_PORT_RN8209<<2)|(USART_IT_TX<<1)|DISABLE:	//RN8209通信口|发送|关闭
		UART2->CTRL &=0xfff9;		//屏蔽发送中断
		UART2->STA = 0x02;		//清发送中断标志
		break;
	case (SCOM_PORT_RN8209<<2)|(USART_IT_TX<<1)|ENABLE:	//RN8209通信口|发送|打开
		UART2->CTRL |=0x06;		//屏蔽发送中断
		UART2->STA = 0x02;		//清发送中断标志

	  UART2->TXD=*(ComPack[SCOM_PORT_RN8209].pTx++);
	  ComPack[SCOM_PORT_RN8209].TxLen--;
		break;
	default: return;
	}
	return;
}


/*****************************************************************************
** Function name:UART2_HANDLER(void)
**
** Description:与RN8209进行底层通讯的串口中断函数，进行RN8209数据的接收及发送
**
** Parameters: NONE
**
** Returned value:	NONE
**
******************************************************************************/
void UART2_HANDLER(void)
{
	u32  status;
	u8	 temp;
	status = UART2->STA;
	/* UART error irq 串口错误处理*/
	if((UART2->CTRL & 0x10) && (status & 0x3c))
	{
		ComPack[SCOM_PORT_RN8209].RxLen=0;
		ComPack[SCOM_PORT_RN8209].EFlag=SCOMPK_EFLAG_IDLE;
		UART2->STA = status;
	}
	/* receive data complete irq 接收中断处理*/
  	if((UART2->CTRL & 0x8) && (status & 0x1))
  	{
    	temp = UART2->RXD;
    	ComPack[SCOM_PORT_RN8209].RxBuf[ComPack[SCOM_PORT_RN8209].RxLen++]=temp; 
  		UART2->STA = 0x1;			// clear receive IF
  	}
  	
  	/* transmit data complete irq 发送中断处理*/
  	if((UART2->CTRL & 0x4) && (status & 0x2))
  	{
		if(ComPack[SCOM_PORT_RN8209].TxLen>0)
		{
			//for(i=0;i<30;i++) __NOP();
			UART2->TXD=*(ComPack[SCOM_PORT_RN8209].pTx++);
			ComPack[SCOM_PORT_RN8209].TxLen--;
			UART2->STA = 0x2;		
		}
	}
  	UART2->STA = status;//清中断标识位
  	return;
}

/*****************************************************************************
** Function name:Rn8209Delay(u16 t)
**
** Description:延时函数
**
** Parameters: t ：延时时间
**
** Returned value:	NONE
**
******************************************************************************/

void Rn8209Delay(u16 t)
{
	u16 i;
	if(Dl645Inactive.PubData.InactiveStamp) return;
	while(t--)
	{
		for (i = 0;i < 400;i++)
		  ;
		WDT->EN = 0xbb;
	}
}
/*****************************************************************************
** Function name:fnRN8209_Write(u8 wReg,u8 *pBuf,u8 ucLen)
**
** Description:写RN8209寄存器
**
** Parameters:wReg 寄存器地址，*pBuf待写入值的存放地址，ucLen：待写入值的长度
**
** Returned value:	操作标识-成功或失败
**
******************************************************************************/

ErrorStatus fnRN8209_Write(u8 wReg,u8 *pBuf,u8 ucLen)
{
	u8 i,j,temp,chksum,Repeat;
	ErrorStatus	err;	
	if( (ucLen == 0) || (ucLen > 4) ) return(ERROR);
	for( Repeat =2; Repeat != 0 ; Repeat--)	
	{
		err = SUCCESS;	
		ComPack[SCOM_PORT_RN8209].pTx=&ComPack[SCOM_PORT_RN8209].TxBuf[0];	
		
		//写数据前，先发送命令字节，命令字节的最高位bit[7]=0:读操作；1：写操作；bit[6:0]为待操作寄存的地址		
		temp =wReg|0x80;//待操作寄存器地址最高位或1，使命令字节为写命令
		*(ComPack[SCOM_PORT_RN8209].pTx++)=temp;
		chksum = temp;		
		for(i = ucLen; i > 0;i-- )
		{		
			*(ComPack[SCOM_PORT_RN8209].pTx++)=pBuf[i-1];	//向RN8209发送数据		
			chksum +=pBuf[i-1];
		}
		chksum = ~chksum;
		*(ComPack[SCOM_PORT_RN8209].pTx++)=chksum;
		ComPack[SCOM_PORT_RN8209].TxLen = ucLen+2;
		ComPack[SCOM_PORT_RN8209].pTx=&ComPack[SCOM_PORT_RN8209].TxBuf[0];
		USART_ITConfig(SCOM_PORT_RN8209 , USART_IT_TX, ENABLE);         //发送使能	
		Rn8209Delay(30);//等待串口发送完成
		
		//RN8209写使能或写保护、读写入WData寄存器检查（是否正确写入）-----------------------	
		ComPack[SCOM_PORT_RN8209].pTx=&ComPack[SCOM_PORT_RN8209].TxBuf[0];	
    if(wReg == 0xEA) 
    {//RN8209写使能或写保护
    	*(ComPack[SCOM_PORT_RN8209].pTx)=0X43;
    }	
    else 
    {//读写入WData寄存器检查
    	*(ComPack[SCOM_PORT_RN8209].pTx)=wReg;
    }	
		ComPack[SCOM_PORT_RN8209].TxLen = 1;	
		ComPack[SCOM_PORT_RN8209].RxLen=0; 
		
		USART_ITConfig(SCOM_PORT_RN8209 , USART_IT_RX, ENABLE);         //接收使能  
		USART_ITConfig(SCOM_PORT_RN8209 , USART_IT_TX, ENABLE);         //发送使能
		memset(&ComPack[SCOM_PORT_RN8209].RxBuf[0] , 0x00 , 10);
		
	  Rn8209Delay(25);//等待通讯完成
	  
	  fnWDT_Restart();
	  j = 0;
	  if(wReg == 0xEA)
	 	{//RN8209写使能或写保护
	 		if(pBuf[0] == 0XE5) 
	 		{//RN8209写使能
	 			temp = ComPack[SCOM_PORT_RN8209].RxBuf[0];
	 			if(!(temp&0x10)) err = ERROR; 
	 		}
	 		else if(pBuf[0] == 0XDC)
	 		{//RN8209写保护
	 			temp = ComPack[SCOM_PORT_RN8209].RxBuf[0];
	 			if(temp&0x10) err = ERROR; 
	 		}
	 	}
	 	else
	 	{	//读写入WData寄存器检查（接收数据ComPack[SCOM_PORT_RN8209].RxBuf[j++]；发送数据pBuf[i-1]）
		  for(i = ucLen; i > 0;i--)
			{
				temp = ComPack[SCOM_PORT_RN8209].RxBuf[j++]; 
				if((wReg == 0)&&(i==2)) temp = 0;//异常处理
				if(temp != pBuf[i-1]) 
				{
					err = ERROR;
					break;
				}
			}
		}
											
	 fnWDT_Restart();
	 if(err == SUCCESS) break;
	 fnScomPk_Init(SCOM_PORT_RN8209);
	}
	USART_ITConfig(SCOM_PORT_RN8209 , USART_IT_TX, DISABLE);         //发送使能关闭
	return(err);
}
/*****************************************************************************  
** Function name:fnRN8209_Read(u8 wReg,u8 *pBuf,u8 ucLen)                      
**                                                                              
** Description:读RN8209寄存器                                                   
**                                                                              
** Parameters:wReg 寄存器地址，*pBuf读出值的存放地址，ucLen：待读值的长度   
**                                                                              
** Returned value:	操作标识-成功或失败                                         
**                                                                              
******************************************************************************/ 

ErrorStatus fnRN8209_Read(u8 wReg,u8 *pBuf,u8 ucLen)
{
	u8 i,temp,Repeat;
	u8 j=0;
	u8 chksum=0;	
	ErrorStatus	err;
	if(ucLen == 0) return(ERROR);

	for( Repeat=2; Repeat != 0 ; Repeat--)	
	{
		err  = SUCCESS;	    
		temp = wReg ;
		chksum=wReg;
		j = 0;	  
		
    ComPack[SCOM_PORT_RN8209].pTx=&ComPack[SCOM_PORT_RN8209].TxBuf[0];	
    
    //读数据前，先发送命令字节，命令字节的最高位bit[7]=0:读操作；1：写操作；bit[6:0]为待操作寄存的地址
    *(ComPack[SCOM_PORT_RN8209].pTx)=temp;	
		ComPack[SCOM_PORT_RN8209].TxLen = 1;	

		USART_ITConfig(SCOM_PORT_RN8209 , USART_IT_RX, ENABLE);         //接收使能  

		USART_ITConfig(SCOM_PORT_RN8209 , USART_IT_TX, ENABLE);         //发送使能
		memset(&ComPack[SCOM_PORT_RN8209].RxBuf[0] , 0x00 , 10);
	    
	  Rn8209Delay(25);//延迟，以等待数据接收完成
	  fnWDT_Restart();	
	    		        			        				    
		for(i = ucLen; i > 0;i--)
		{
			pBuf[i-1] = ComPack[SCOM_PORT_RN8209].RxBuf[j++]; //将接收到的数据保存到数组pBuf[]
			chksum += pBuf[i-1];//计算接收数据的校验和
		}
		chksum = ~chksum;
		temp=ComPack[SCOM_PORT_RN8209].RxBuf[j++];
		
		if(temp!=chksum)  
		{//若校验和错误，清接收数据
			err = ERROR;
			for(i = ucLen; i > 0;i--) pBuf[i-1] = 0;
		}
		if(err == SUCCESS) break;
		fnScomPk_Init(SCOM_PORT_RN8209);	  	  	
	}

	USART_ITConfig(SCOM_PORT_RN8209 , USART_IT_TX, DISABLE);         //发送使能关闭
	ComPack[SCOM_PORT_RN8209].RxLen=0;                               //  

	return(err);
}
/*****************************************************************************
** Function name:	fnDl645_ComInitRN8209(void)
**
** Description:		清计量（校表）参数函数，清RN8209寄存器
**
** Parameters:		NONE
**
** Returned value:	NONE
**
******************************************************************************/
void fnDl645_ComInitRN8209(void)
{
	Dl645FirmPara.HFConst = 0x1000;
	Dl645FirmPara.PStart = 0x0060;
	Dl645FirmPara.QStart  = 0x0120;
	Dl645FirmPara.GPQA = 0x0000;
	Dl645FirmPara.GPQB = 0x0000;
	Dl645FirmPara.PhsA= 0x0000;
	Dl645FirmPara.PhsB= 0x0000;
	Dl645FirmPara.QPhsCal = 0x0000;
	Dl645FirmPara.APOSA = 0x0000;
	Dl645FirmPara.APOSB = 0x0000;
	Dl645FirmPara.RPOSA = 0x0000;
	Dl645FirmPara.RPOSB = 0x0000;
	Dl645FirmPara.IARMSOS = 0x0000;
	Dl645FirmPara.IBRMSOS = 0x0000;
	Dl645FirmPara.IBGain = 0x0000;
	//Dl645FirmPara.KUrms= 0x00000000;
	//Dl645FirmPara.KIArms= 0x00000000;
	//Dl645FirmPara.KIBrms= 0x00000000;
	
	Dl645FirmPara.KPrms= 0x00000000;
	Dl645RN8209DataComm.ucTemp8 =0XE5;
	//if(fnRN8209_Write( 0xEA , Dl645RN8209DataComm.ucTempBuf , 1 ) == ERROR) return ;   //写使能
	fnRN8209_Write( 0xEA , Dl645RN8209DataComm.ucTempBuf , 1 );
	
	//if(fnRN8209_Write( 0x00 , (u8 *)&Dl645FirmPara.SYSCON , 2 ) == ERROR) return ;   //写系统控制寄存器
	//if(fnRN8209_Write( 0x01 , (u8 *)&Dl645FirmPara.EMUCON , 2 ) == ERROR) return ;   //写计量控制寄存器
		
	if(fnRN8209_Write( ADHFConst , (u8 *)&Dl645FirmPara.HFConst , 2 ) == ERROR) return ; 
	if(fnRN8209_Write( ADPStart , (u8 *)&Dl645FirmPara.PStart , 2 ) == ERROR) return ; 	
	if(fnRN8209_Write( ADDStart , (u8 *)&Dl645FirmPara.QStart , 2 ) == ERROR) return ; 
	
	if(fnRN8209_Write( ADGPQA , (u8 *)&Dl645FirmPara.GPQA , 2 ) == ERROR) return;   //写功率增益寄存器
	if(fnRN8209_Write( ADGPQB , (u8 *)&Dl645FirmPara.GPQB , 2 ) == ERROR) return;   //写功率增益寄存器
	if(fnRN8209_Write( ADPhsA , (u8 *)&Dl645FirmPara.PhsA , 1 ) == ERROR) return;   //写相位校正寄存器
	if(fnRN8209_Write( ADPhsB , (u8 *)&Dl645FirmPara.PhsB , 1 ) == ERROR) return;   //写相位校正寄存器
	if(fnRN8209_Write( ADQPHSCAL , (u8 *)&Dl645FirmPara.QPhsCal , 2 ) == ERROR) return;    
	if(fnRN8209_Write( ADAPOSA , (u8 *)&Dl645FirmPara.APOSA , 2 ) == ERROR) return;
	if(fnRN8209_Write( ADAPOSB , (u8 *)&Dl645FirmPara.APOSB , 2 ) == ERROR) return;
	if(fnRN8209_Write( ADRPOSA , (u8 *)&Dl645FirmPara.RPOSA , 2 ) == ERROR) return;
	if(fnRN8209_Write( ADRPOSB , (u8 *)&Dl645FirmPara.RPOSB , 2 ) == ERROR) return;
	if(fnRN8209_Write( ADIARMSOS , (u8 *)&Dl645FirmPara.IARMSOS , 2 ) == ERROR) return;
	if(fnRN8209_Write( ADIBRMSOS , (u8 *)&Dl645FirmPara.IBRMSOS , 2 ) == ERROR) return;
	if(fnRN8209_Write( ADIBGain , (u8 *)&Dl645FirmPara.IBGain , 2 ) == ERROR) return;
	if(fnRN8209_Write( ADEMUCON2 , (u8 *)&Dl645FirmPara.EMUCON2 , 2 ) == ERROR) return;
	fnWDT_Restart();
	Dl645RN8209DataComm.ucTemp8 =0XDC;
	fnRN8209_Write( 0xEA , Dl645RN8209DataComm.ucTempBuf , 1 ) ; 
	fnDl645File_Write(Dl645FileId_FirmPara,Dl645FileItemInfoOffAddr_FirmPara_HFConst,(u8 *)&Dl645FirmPara.HFConst,42);

}

/*****************************************************************************
** Function name:	fnEMU_Init(void)
**
** Description:		计量EMU初始化函数，初始化RN8209寄存器
**
** Parameters:		NONE
**
** Returned value:	NONE
**
******************************************************************************/
void fnEMU_Init(void)
{

	//写计量芯片相关配置
	Dl645FirmPara.EMUCON2 = 0x300;
	
  Dl645RN8209DataComm.ucTemp8 =0XE5;
  //if(fnRN8209_Write( 0xEA , Dl645RN8209DataComm.ucTempBuf , 1 ) == ERROR) return ;   //写使能
  fnRN8209_Write( 0xEA , Dl645RN8209DataComm.ucTempBuf , 1 ); 
	
	//if(fnRN8209_Write( ADSYSCON , (u8 *)&Dl645FirmPara.SYSCON , 2 ) == ERROR) return ;   //写系统控制寄存器
	//if(fnRN8209_Write( ADEMUCON , (u8 *)&Dl645FirmPara.EMUCON , 2 ) == ERROR) return ;   //写计量控制寄存器
	if(fnRN8209_Write( ADHFConst , (u8 *)&Dl645FirmPara.HFConst , 2 ) == ERROR) return ; 
	if(fnRN8209_Write( ADPStart , (u8 *)&Dl645FirmPara.PStart , 2 ) == ERROR) return ; 	
	if(fnRN8209_Write( ADDStart , (u8 *)&Dl645FirmPara.QStart , 2 ) == ERROR) return ; 
	
	if(fnRN8209_Write( ADGPQA , (u8 *)&Dl645FirmPara.GPQA , 2 ) == ERROR) return;   //写功率增益寄存器
	if(fnRN8209_Write( ADGPQB , (u8 *)&Dl645FirmPara.GPQB , 2 ) == ERROR) return;   //写功率增益寄存器
	if(fnRN8209_Write( ADPhsA , (u8 *)&Dl645FirmPara.PhsA , 1 ) == ERROR) return;   //写相位校正寄存器
	if(fnRN8209_Write( ADPhsB , (u8 *)&Dl645FirmPara.PhsB , 1 ) == ERROR) return;   //写相位校正寄存器
	if(fnRN8209_Write( ADQPHSCAL , (u8 *)&Dl645FirmPara.QPhsCal , 2 ) == ERROR) return;    
  if(fnRN8209_Write( ADAPOSA , (u8 *)&Dl645FirmPara.APOSA , 2 ) == ERROR) return;
  if(fnRN8209_Write( ADAPOSB , (u8 *)&Dl645FirmPara.APOSB , 2 ) == ERROR) return;
  if(fnRN8209_Write( ADRPOSA , (u8 *)&Dl645FirmPara.RPOSA , 2 ) == ERROR) return;
  if(fnRN8209_Write( ADRPOSB , (u8 *)&Dl645FirmPara.RPOSB , 2 ) == ERROR) return;
  if(fnRN8209_Write( ADIARMSOS , (u8 *)&Dl645FirmPara.IARMSOS , 2 ) == ERROR) return;
  if(fnRN8209_Write( ADIBRMSOS , (u8 *)&Dl645FirmPara.IBRMSOS , 2 ) == ERROR) return;
  if(fnRN8209_Write( ADIBGain , (u8 *)&Dl645FirmPara.IBGain , 2 ) == ERROR) return;
  if(fnRN8209_Write( ADEMUCON2 , (u8 *)&Dl645FirmPara.EMUCON2 , 2 ) == ERROR) return;
  fnWDT_Restart();
  Dl645RN8209DataComm.ucTemp8 =0XDC;
  fnRN8209_Write( 0xEA , Dl645RN8209DataComm.ucTempBuf , 1 ) ; //关闭写使能 
}
/*****************************************************************************
** Function name:	fnDl645Front_Init(void)
**
** Description:		计量模块初始化函数，将计量用到的RAM数据进行初始化
**
** Parameters:		NONE
**
** Returned value:	NONE
**
******************************************************************************/
void fnDl645Front_Init(void)
{
	u8 	i,j;
	u32 ChkSum;	
	Dl645Front.PriData.Flag |= FRONT_FLAG_RN8209RST;//置RN8209初始化标识位
	
	memset(&Dl645FrontTmp , 0 , sizeof(sDl645FrontTmp_TypeDef) );//清计量用到的RAM数据
	memset(&Dl645Front , 0 , sizeof(sDl645Front_TypeDef) );//清计量用到的RAM数据	
	
	//写入校表寄存器，检查校验是否正确。正确退出，不正确再次写入。最多循环写入次数5次。
	fnDl645File_Read(Dl645FileId_FirmPara , 0 , (u8 *)&Dl645FirmPara , sizeof(sDl645FirmParaFile_TypeDef) );//读E2中保存的校表参数
	Dl645FrontTmp.ChkSum1 = Dl645FirmPara.ChkSum;
	for(i=0;i<5;i++)
	{
		fnEMU_Init();
		SystemDelay(10);
		ChkSum = 0;		
		for(j=0;j<5;j++)
		{
			fnRN8209_Read( ADEMUStatus , (u8 *)&ChkSum , 3 ) ;
			if(!(ChkSum & 0x010000))	break;
			SystemDelay(10);
		}
		fnWDT_Restart();
		ChkSum = ChkSum & 0x0000ffff;
		if(Dl645FrontTmp.ChkSum1 == ChkSum) 
		{
			Dl645Front.PriData.Flag &= ~FRONT_FLAG_RN8209RST;
			break;
		}			
	}
	if(i>=5)
	{//若校验值连续5次错误，则重新计算校验值
		Dl645FirmPara.ChkSum = 0;
		SystemDelay(1);		
		Dl645FirmPara.ChkSum = ChkSum & 0x0000ffff;
		Dl645FrontTmp.ChkSum1 = Dl645FirmPara.ChkSum;
		fnDl645File_Write(Dl645FileId_FirmPara,Dl645FileItemInfoOffAddr_FirmPara_ChkSum,(u8 *)&Dl645FirmPara.ChkSum,4);
	}
	
}
/*****************************************************************************
** Function name:	fnDl645Front_Exec(void)
**
** Description:		从计量芯片读取数据（电压、电流、功率、脉冲数），及电压、电流、频率、功率因素计算
**
** Parameters:		NONE
**
** Returned value:	NONE
**
******************************************************************************/	
	
void fnDl645Front_Exec(void)
{ 
	u8 i;
	u32 TempU,TempI,TempIn;
	u32 TempStatus;
	u16 TempAngle;	
	u8	PFlag;		
	
	if(Dl645Front.PriData.Flag & FRONT_FLAG_RN8209RST)//RN8209复位
	{
		fnDl645Front_Init();
		Dl645Front.PriData.Flag &= ~FRONT_FLAG_RN8209RST;
		return;
	}
	  
  //读计量状态及校验和
	TempStatus = 0;
	fnRN8209_Read( 0x2d , (u8 *)&TempStatus , 3 ) ;
	if(!(TempStatus & 0x010000))
	{//校表数据校验和计算已完成，校验值可用
		if(Dl645FrontTmp.ChkSum1 == (TempStatus&0x0000ffff)) 
		{//校验值正确，清校验错误次数
			Dl645FrontTmp.ChkErrCnt = 0 ;
		}
		else 
		{
			Dl645FrontTmp.ChkErrCnt++; 
			if(Dl645FrontTmp.ChkErrCnt > 3) 
			{//校验错误次数大于3次后，复位RN8209
				Dl645Front.PriData.Flag |= FRONT_FLAG_RN8209RST;
				Dl645FrontTmp.ChkErrCnt = 0 ;
			}
			return;
		}
	}
    //判断功率方向
  if(TempStatus & 0x020000) 
  {
  	Dl645Front.PubData.PDirect =  INVERSION ;
  }  //判断功率方向是反的。
  else 
  {
  	Dl645Front.PubData.PDirect = POSITIVE ;
  }    

  //读电压、电流到缓冲区，读数为负进行处理
	for(i = 0 ; i < 3 ; i++)
  {
		Dl645FrontTmp.UI[i] = 0;//清电压、电流变量
		fnRN8209_Read( 0x22+i , (u8 *)&Dl645FrontTmp.UI[i] , 3 ) ;
		if(Dl645FrontTmp.UI[i]&0x00800000) Dl645FrontTmp.UI[i]=0;
  }
    //读频率
	fnRN8209_Read( 0x25 , (u8 *)&Dl645FrontTmp.Frequency , 2 ) ;
    //读功率到缓冲区，读数为负进行处理
  fnRN8209_Read( 0x26 , (u8 *)&Dl645FrontTmp.Pw[0] , 4 ) ;
	fnRN8209_Read( 0x27 , (u8 *)&Dl645FrontTmp.Pw[1] , 4 ) ;
	if(Dl645FrontTmp.Pw[0]&0x80000000) 
	{
		Dl645FrontTmp.Pw[0]=(~Dl645FrontTmp.Pw[0])+1;PFlag = 1;
	}
	else PFlag = 0;
	//计算功率
	Dl645Front.PubData.Pw = fnDFConver_Hex32ToDF09((s32)((Dl645FrontTmp.Pw[0])*((Dl645FirmPara.KPrms))));
	if(PFlag) Dl645Front.PubData.Pw.S = 1;
	//电流电压计算
	Dl645Front.PubData.U = 0x7fff&(fnDFConver_Bcd16To16((s16)(Dl645FrontTmp.UI[2]/(10*(Dl645FirmPara.KUrms))))); //电压
  TempI = (s32)(Dl645FrontTmp.UI[0]/(Dl645FirmPara.KIArms));   
  TempIn = (s32)(Dl645FrontTmp.UI[1]/(Dl645FirmPara.KIBrms));	
	Dl645Front.PubData.Ia = fnDFConver_Bcd32To32(TempI);	
	Dl645Front.PubData.In = fnDFConver_Bcd32To32(TempIn);
	if(PFlag) Dl645Front.PubData.Ia |= 0x80000000;
	
	//功率有效值计算,功率因数计算  
	//计算功率因数
	TempU &=0x00ffffff; 
	if(TempU&0x00800000) TempU=((~TempU)&0x00ffffff)+1;
	Dl645Front.PubData.Pf = fnHexToBcd_u16((u16)((float)TempU/8388.608));
	Dl645Front.PubData.Angle=	fnHexToBcd_u16(TempAngle*3600/32768); 
	//功率小于0.0030，清零，功率因数置0.999
	if(((Dl645Front.PubData.Pw.Dat2&0x7f)==0)&&(Dl645Front.PubData.Pw.Dat1==0)&&(Dl645Front.PubData.Pw.Dat0<0x30)) 
	{
		Dl645Front.PubData.Pw.Dat0 = 0;
		Dl645Front.PubData.Pw.Dat1 = 0;
		Dl645Front.PubData.Pw.Dat2 = 0;
	}
	//电流小于起动电流，清零
	if((Dl645Front.PubData.Ia&0x7fffffff) < 0x00000150) {Dl645Front.PubData.Ia = 0;Dl645Front.PubData.Pf = 0x0999;}
	if((Dl645Front.PubData.In&0x7fffffff) < 0x00000150) Dl645Front.PubData.In = 0;
	
	//电压频率计算	
	Dl645Front.PubData.Frequency = fnHexToBcd_u16((u16)(((u32)357954500)/((u32)8*Dl645FrontTmp.Frequency)));	
	if((Dl645Front.PubData.Ia&0x7fffffff) > 0x00550000) 
	{	
		if(Dl645Front.PubData.tMaxI < 2405) Dl645Front.PubData.tMaxI++;
	}
	else Dl645Front.PubData.tMaxI = 0;
	
	if(Dl645Front.PubData.Pw.S) Dl645Front.PubData.Pf |=0x8000;
	
	//读电能脉冲，加入脉冲计数器
	Dl645FrontTmp.Pulse = 0;
	fnRN8209_Read( 0x2a , (u8 *)&Dl645FrontTmp.Pulse , 3 ) ;
	if(Dl645FrontTmp.Pulse > 100) Dl645FrontTmp.Pulse = 0;	//容错，脉冲个数过大，清除
	Dl645FrontTmp.Pulse_Eg+=Dl645FrontTmp.Pulse;
	
	#if  DL645SOFT_DEBUG
	Dl645FrontTmp.Pulse_Eg+=1;
	#endif 
	
}