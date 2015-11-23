#pragma NOP
#pragma sfr
/***********************************************************************/
#include "includes.h"
/***********************************************************************/
const u8 _rn8209default[][3] = /*/////RN8209默认参数//////*/  
{   
   #ifdef _IB1DOT5ATO6A   
     {0x00,0x02,0x02},    /*///// 1.5(6)A不使用通道2 通道1采用8倍增益//////*/ 	      
   #else
     {0x00,0x43,0x43},    /*///// 只列了00-04地址默认值 其他默认都为0  第3个字节为校验和//////*/
   #endif
   {0x00,0x01,0x01},	
   {0x2B,0x00,0x2B},
   //{0x40,0x00,0x40},	
   {0x00,0x60,0x60},	
   {0x01,0x20,0x21}
};	
/***********************************************************************/
extern  ST_BN8209_CS  BN8209_CS;
extern  ST_MPARA  MeterPara;

u8 RN8209_SPI_Read(void);
void RN8209_SPI_Write(u8 bdata); 
void RN8209_DisablePulse(void);
void RN8209_EnablePulse(void);
/***********************************************************************/
/*///// 函数名称: void RN8209_SPI_Ini(void) //////*/  
/*///// 函数功能: SPI通信口初始化 //////*/  
/*///// 输入参数: 无 //////*/  
/*///// 输出参数: 无 //////*/  
/***********************************************************************/
void RN8209_SPI_Ini(void)
{
  DIR_SPIRST = OUT_PORT;
  PIN_SPIRST = HIGH_LEVEL;
  DIR_SPICS  = OUT_PORT; 
  PIN_SPICS  = HIGH_LEVEL;
  DIR_SPISDO = OUT_PORT ;
  PIN_SPISDO = HIGH_LEVEL;
  DIR_SPISDI = INPUT;      
  DIR_SPICLK = OUT_PORT;
  PIN_SPICLK = HIGH_LEVEL; 
  
  return; 
}
/***********************************************************************/
/*///// 函数名称: u8 RN8209_SPI_Read(void) //////*/  
/*///// 函数功能: 读取8209一字节数据 //////*/  
/*///// 输入参数: 无 //////*/  
/*///// 输出参数: 读取的1字节数据 //////*/  
/***********************************************************************/
u8 RN8209_SPI_Read(void)
{
	u8 i,rbyte=0;
	
	DIR_SPICLK = OUT_PORT;
	PIN_SPICLK = HIGH_LEVEL;
	
	for(i=0;i<8;i++)
	{		
		Delay1us();		
	  DIR_SPICLK = OUT_PORT;		
		Delay1us();		
		PIN_SPICLK = HIGH_LEVEL;		
		DelayNus(50);		
		rbyte <<= 1;		
		PIN_SPICLK = LOW_LEVEL;			 
		DIR_SPISDI = INPUT; 		
		if (PIN_SPISDI == 1)  
		{
			rbyte |= 1; 
	  } 	
//		PIN_SPICLK  = LOW_LEVEL ;			
		DelayNus(50);
	}	
	DIR_SPICLK = OUT_PORT;	
	PIN_SPICLK = HIGH_LEVEL;	
	DelayNus(1);
	
	return(rbyte);
}
/***********************************************************************/
/*///// 函数名称: void RN8209_SPI_Write(u8 bdata) //////*/  
/*///// 函数功能: 写入1个字节数据到8209中 //////*/  
/*///// 输入参数: bdata-数据 //////*/  
/*///// 输出参数: 无 //////*/  
/***********************************************************************/
void RN8209_SPI_Write(u8 bdata)
{
	u8 i;
	u8 byte;
	
	for(i=0;i<8;i++)
	{
		Delay1us();	
		byte = bdata & 0x80;		
		DIR_SPISDO = OUT_PORT;		
		if (byte==0) 
		{	
		  PIN_SPISDO = LOW_LEVEL;
		}     
		else
		{	
		  PIN_SPISDO = HIGH_LEVEL;
		}
		DIR_SPICLK = OUT_PORT;		
		PIN_SPICLK = HIGH_LEVEL;		
		DelayNus(50);
		PIN_SPICLK = LOW_LEVEL;
		DelayNus(50);
		bdata <<= 1;
	}	  
	DIR_SPISDO = OUT_PORT; 
	PIN_SPISDO = HIGH_LEVEL; 
	DelayNus(2);
	
	return;		 
}
/***********************************************************************/
/*///// 函数名称: u8 RN8209_ReadData(u8 add,u8 *data,u8 *len) //////*/  
/*///// 函数功能: 读取8209寄存器数据 //////*/  
/*///// 输入参数: add-8209寄存器地址 *data-读取后放置的缓冲区 *len-数据长度 //////*/  
/*///// 输出参数: 0-读取失败 1-读取成功 //////*/  
/***********************************************************************/
u8 RN8209_ReadData(u8 add,u8 *data,u8 *len)
{
	u8 ret=1;
	
	DIR_SPICS = OUT_PORT ; 
  PIN_SPICS = LOW_LEVEL;
  DelayNus(2);
  PIN_SPICS = LOW_LEVEL;
	RN8209_SPI_Write(add);
	switch(add)
	{
    case 0x00:
		case 0x01:
		case 0x02:	
		case 0x03:	
		case 0x04:	
		case 0x05:
		case 0x06:				
		case 0x09:	
		case 0x0A:	
		case 0x0B:	
		case 0x0C:	
		case 0x0D:	
		case 0x0E:	
		case 0x0F:	
		case 0x10:	
		case 0x20: 
		case 0x21:
		case 0x25:	
		case 0x45: 	
			*data = RN8209_SPI_Read();
			*(data+1) = RN8209_SPI_Read(); 
			*len = 2;
		break; 				
		case 0x07:	 
		case 0x08:	
		case 0x40:
		case 0x41:
		case 0x42:
		case 0x43:  
			*data = RN8209_SPI_Read();
			*len = 1;
			break; 				 
		case 0x22:
		case 0x23:
		case 0x24:
		case 0x29:	
		case 0x2A:	
		case 0x2B:
		case 0x2C:	
		case 0x2D:
    case 0x7f : 
    	*data = RN8209_SPI_Read();
			*(data+1) = RN8209_SPI_Read(); 
			*(data+2) = RN8209_SPI_Read(); 
			*len = 3;
			break; 
		case 0x26 :	 
		case 0x27 :	 
		case 0x28 :	 
		case 0x44 :	
			*data = RN8209_SPI_Read();
			*(data+1) = RN8209_SPI_Read(); 
			*(data+2) = RN8209_SPI_Read(); 
		  *(data+3) = RN8209_SPI_Read();
			*len = 4;
			break;        
		default :
			ret = 0;	
			break;
	}
	DelayNus(2);
	DIR_SPICS = OUT_PORT ; 
  PIN_SPICS = HIGH_LEVEL;
  
	return(ret);
}
/***********************************************************************/
/*///// 函数名称: void RN8209_WriteData(u8 *ptr) //////*/  
/*///// 函数功能: 写入数据到8209寄存器中//////*/  
/*///// 输入参数: *ptr-指向要写入的数据缓冲区 //////*/  
/*///// 输出参数: 无  //////*/  
/***********************************************************************/
void RN8209_WriteData(u8 *ptr)    
{
	u8 temp[2];
	
	if ((*ptr) > 0x10)      /*// 控制寄存器地址不大于0x10 //*/ 
	{
		return;
	}
	
	DIR_SPICS = OUT_PORT; 
  PIN_SPICS = LOW_LEVEL;
  DelayNus(2);
	temp[0] = 0xea;        /*// 写使能命令 //*/
	temp[1] = 0xe5;
	RN8209_SPI_Write(temp[0]);
	RN8209_SPI_Write(temp[1]);
	
	RN8209_SPI_Write((*ptr)+0x80);	 /*// 写入数据 //*/
	if ( (*ptr != 0x07) && (*ptr != 0x08) )
	{ 
	  RN8209_SPI_Write(*(ptr+1));
	}	
	RN8209_SPI_Write(*(ptr+2));
	
	temp[0] = 0xea;       /*// 写保护命令 //*/
	temp[1] = 0xdc;  
	RN8209_SPI_Write(temp[0]);
	RN8209_SPI_Write(temp[1]);
	DelayNus(2);
	DIR_SPICS = OUT_PORT ; 
  PIN_SPICS = HIGH_LEVEL;
  
  return;
}
/***********************************************************************/
/*///// 函数名称: u8 RN8209_WriteDataToEE(u8 *ptr) //////*/  
/*///// 函数功能: 写入校表数据到EEP中//////*/  
/*///// 输入参数: *ptr-指向要写入的数据缓冲区 //////*/  
/*///// 输出参数: 0-失败 1-成功  //////*/  
/***********************************************************************/
u8 RN8209_WriteDataToEE(u8 *ptr)  
{
	u8 len;
	u16 tmpadr;	
	
	if(*ptr > 0x14)
	{
		return;
	}
	
	if(*ptr > 0x10)
	{
		if (*ptr == 0x11)      /*///// 0x11-功率校正系数为4个字节 //////*/  
		{		  
		  len = 4;
		  tmpadr = EM_ADR_JBCSP; 
		}
		else                  /*/////  0x12-电流校准系数  0x13-零线电流校准系数  0x14电压校准系数//////*/  
		{
			len = 3;
			tmpadr = EM_ADR_JBCSP+5+((u16)((*ptr)-0x12))*4; 
		}
	}
	else 
	{
	  tmpadr = EM_ADR_RN8209JBCS+((u16)(*ptr))*3;
	  len = 2;
	}
 	
 	return( Write24LC64WithBK(tmpadr,len,ptr+1) ); 
}
/***********************************************************************/
/*///// 函数名称: void RN8209_TestJBCS(u16 r8209chksum) //////*/  
/*///// 函数功能: 比对EEP中的校表数据校验和和8209的寄存器校验和  1秒校验1次//////*/  
/*///// 输入参数: r8209chksum-8209中读取的校验和 //////*/  
/*///// 输出参数: 无  //////*/  
/***********************************************************************/
void RN8209_TestJBCS(u16 r8209chksum)    
{  
  if(BN8209_CS.RegDataChkSum != r8209chksum)
  {
  	RN8209_LoadJBCS();
  	
  	#ifdef _NBKZCMD       
	    Print_DebugInf_Uart6((u8 *)("DEBUG INF: BN8209 chksum error \r\n"),ISN,0,ISTRUE);
	    Print_DebugInf_Uart6((u8 *)("DEBUG INF: end\r\n"),ISN,0,ISFALSE);
		#endif 
  }
  
  return;
}
/***********************************************************************/
/*///// 函数名称: u8 RN8209_Test(u8 testnum) //////*/  
/*///// 函数功能: 测试与8209的通信状态//////*/  
/*///// 输入参数: testnum-测试次数 //////*/  
/*///// 输出参数: 0-通信失败 1-通信正常 //////*/  
/*///// 说明: 通信故障时给8209复位 直到通讯正常 //////*/  
/***********************************************************************/
u8 RN8209_Test(u8 testnum)   /*// 测试8209 ////*/
{
   u8 i,tmpbyte;
   u8 temp[2];     
   
   RN8209_SPI_Ini();
   
   i = 0;
   while(i < testnum)
   {   	 
   	 MemCopyNN(0,temp,2);
	   RN8209_ReadData(0x7f,temp,&tmpbyte);	   
	   if ((temp[0] == 0x82) && (temp[1] == 0x09))
	   {
	   	 break;
	   }
	   Delay2ms(1);
	   FeedWatchDog();
	   i++;
	 }	
	  
	 if(i == testnum)
	 {
	 	 DIR_SPIRST = OUT_PORT;
     PIN_SPIRST = LOW_LEVEL;
	 	 
	 	 Set_MeterErr_8209Err();
	 	 AlarmLED_On();
     BackLight_On();
	 	 return(false);
	 }
	 
	 Set_MeterErr_8209OK();
	 return(true);	
} 
/***********************************************************************/
/*///// 函数名称: void RN8209_LoadJBCS(void) //////*/  
/*///// 函数功能: 读取EEP校表参数并写入到8209中//////*/  
/*///// 输入参数: 无 //////*/  
/*///// 输出参数: 无 //////*/  
/***********************************************************************/
void RN8209_LoadJBCS(void)     
{
	 u8 tmpbyte,i,j;
	 u8 temp[3];
	 u8 temptst[2];
	 u16 addr;     
   
   RN8209_SPI_Ini();
   
	 RN8209_DisablePulse();
	 if (RN8209_Test(TPOWERONTSTNUM) == 1) /*// 上电时测试50次 ////*/ 
	 {	 	 
  	 for (i=0;i<=16;i++)  
  	 {
  	 	 addr = EM_ADR_RN8209JBCS+i*3;
  	 	 ReadParameter(addr,2,&temp[1]);
  	 	 temp[0] = i;	 
  	 	 if(i == 1)
  	 	 {
  	 	 	 temp[2] = 0;	
  	 	 	 temp[1] = 0;
  	 	 }	 
  	 	 for (j=0;j<3;j++)
  	 	 {
  		   RN8209_WriteData(&temp[0]);
  		   Delay2ms(1);
  		   RN8209_ReadData(i,temptst,&tmpbyte);	
  		   	   		   
  		   if ((i == 7) || (i == 8))
  		   {
  		   	 temptst[1] = temptst[0];
  		   	 temptst[0] = 0;
  		   }
  		   
  		   if (MemCompareZheng(&temp[1],temptst,2) == ISEQUAL)
  		   {
  		   	 break;
  		   }
  		 }
  	 }
	 }	 	 
	 RN8209_CalChkSum();         /*// 计算EEPROM校表数据累计校验和 ////*/ 
	 
	 RN8209_EnablePulse();
	 
	 return;
}
/***********************************************************************/
/*///// 函数名称: void RN8209_InitDefault(void) //////*/  
/*///// 函数功能: 初始化EEP校表参数为默认参数 //////*/  
/*///// 输入参数: 无 //////*/  
/*///// 输出参数: 无 //////*/  
/***********************************************************************/
void  RN8209_InitDefault(void) /*// 初始化EEP校表参数为默认参数 ////*/ 
{
   u8 tempbuf[3];
   
   Read24LC64(EM_ADR_RN8209JBCS,3,tempbuf,TNOCHK); 
   if( IsEqualSpecialData(tempbuf,0xff,3) == OK)
   {
     RN8209_PInitDefault();
   }
	 
	 return;
}
/***********************************************************************/
/*///// 函数名称: void RN8209_PInitDefault(void) //////*/  
/*///// 函数功能: 初始化EEP校表参数为默认参数 //////*/  
/*///// 输入参数: 无 //////*/  
/*///// 输出参数: 无 //////*/  
/***********************************************************************/
void  RN8209_PInitDefault(void) /*// 初始化EEP校表参数为默认参数 ////*/ 
{
   u8 tempbuf[36];

   MemCopyNN(0,tempbuf,36);
      	    	     
	 Write24LC64(EM_ADR_RN8209JBCS,15,&_rn8209default[0][0],TNOCHK); 
	 Write24LC64(EM_ADR_RN8209JBCS+BACKUP_LOCATION,15,&_rn8209default[0][0],TNOCHK); 
	 FeedWatchDog();
	 Write24LC64(EM_ADR_RN8209JBCS+15,36,tempbuf,TNOCHK);
	 Write24LC64(EM_ADR_RN8209JBCS+15+BACKUP_LOCATION,36,tempbuf,TNOCHK);
	 
	 return;
}
/***********************************************************************/
/*///// 函数名称: void RN8209_CalChkSum(void) //////*/  
/*///// 函数功能: 计算EEP校表数据校验和 存储在ram中 //////*/  
/*///// 输入参数: 无 //////*/  
/*///// 输出参数: 无 //////*/  
/***********************************************************************/
void  RN8209_CalChkSum(void)     
{
	u8 i;
	u8 tmpbuf[2];
	u16 addr,tmpeepchksum;  
  
  tmpeepchksum = 0;
  i = 0;
  while(i <= 16)
  {
  	addr = EM_ADR_RN8209JBCS+i*3;
  	ReadParameter(addr,2,&tmpbuf[0]);  	
  	tmpeepchksum += (((u16)(tmpbuf[0])) << 8)+tmpbuf[1]; 
  	i++;
  }	
  
  BN8209_CS.RegDataChkSum = 0xffff-tmpeepchksum; 
	
	return; 
}
/***********************************************************************/
/*///// 函数名称: void RN8209_DisablePulse(void) //////*/  
/*///// 函数功能: 关闭8029脉冲输出 //////*/  
/*///// 输入参数: 无 //////*/  
/*///// 输出参数: 无 //////*/  
/***********************************************************************/
void  RN8209_DisablePulse(void) 
{
	 u8 temp[3];	 
	 
	 temp[0] = 0x01;
	 temp[1] = 0x00;
	 temp[2] = 0x00;
	 RN8209_WriteData(&temp[0]);
	 
	 return;
}
/***********************************************************************/
/*///// 函数名称: void RN8209_DisablePulse(void) //////*/  
/*///// 函数功能: 使能8029脉冲输出 //////*/  
/*///// 输入参数: 无 //////*/  
/*///// 输出参数: 无 //////*/  
/***********************************************************************/ 
void  RN8209_EnablePulse(void)  
{
	 u8 temp[3];
	 
	 temp[0] = 0x01;
	 temp[1] = 0x00;
	 temp[2] = 0x01;
	 RN8209_WriteData(&temp[0]);
	 
	 return;
}
/***********************************************************************/
/*///// 函数名称: void RN8209_DisablePulse(void) //////*/  
/*///// 函数功能: 计算校表参数并写入到8209寄存器和EEPROM中 //////*/  
/*///// 输入参数: regadr-8209寄存器地址 *ptr-指向要写入的数据缓冲区 //////*/  
/*///// 输出参数: 无 //////*/  
/***********************************************************************/ 
void  RN8209_CalJBCS(u8 regadr,u8 *ptr)   
{
	 u8 tmpbyte,tmpadr;
	 u8 tmpbuf[5];
	 u16 tmpcal,tmpu16;
	 u32 tmp8209P,tmprealP;
	 float tmpfloat;
	 
   switch (regadr)	
   {
     case 0x02:     /*//////HFConst设置/////*/
     	 tmpbuf[1] = *(ptr+1);
	     tmpbuf[2] = *(ptr); 
     break; 
     case 0x03:     /*//////启动功率值设置/////*/
     	 /*//////台子打到0.2%Ib,读出有功功率,取中间两字位传到有功启动寄存器03中/////*/
     	 RN8209_ReadData(RN8209_REG_PA,&tmpbuf[0],&tmpbyte);     	 
     break; 
     case 0x05:     /*//////有功功率1.0校正/////*/
     case 0x07:	    /*//////有功功率0.5L校正/////*/
   	   tmp8209P = ((u32)(BN8209_CS.P[2]) << 16) + ((u32)(BN8209_CS.P[1]) << 8) + BN8209_CS.P[0];
   	   tmp8209P &= 0x7fffff;    /*//////屏蔽功率符号位/////*/
   	   tmprealP = ((u32)(*(ptr+2)) << 16) + ((u32)(*(ptr+1)) << 8) + (*(ptr));   	 
   	   tmp8209P = BCDLongTOHexlong(tmp8209P); /*//////RN8209功率值/////*/
   	   tmprealP = BCDLongTOHexlong(tmprealP); /*//////校表台功率值/////*/
     	 
     	 if(regadr == 0x05)           
     	 {
     	 	 if(tmprealP > tmp8209P)    /*//////误差偏负/////*/
   	     {
   	 	     tmpfloat = ((float)(tmprealP - tmp8209P)) / ((float)(tmprealP));
   	 	     tmpfloat = tmpfloat/(1.0000-tmpfloat);
   	 	     tmpu16 = (u16)(32768*tmpfloat);
   	     }
   	     else                       /*//////误差偏正/////*/
   	     {
   	       tmpfloat = ((float)(tmp8209P - tmprealP)) / ((float)(tmprealP));
   	       tmpfloat = tmpfloat/(1.0000+tmpfloat);
   	 	     tmpu16 = (u16)(32768*(2.0000-tmpfloat));
   	     }
     	 }
     	 else if(regadr == 0x07)      
     	 {
     	 	 if(tmprealP > tmp8209P)    /*//////误差偏负/////*/
   	 		 {
   	 	     tmpfloat = ((float)(tmprealP - tmp8209P)) / ((float)(tmprealP));
   	 	     tmpu16 = (u16)(1654*tmpfloat);
   	     }
   	     else
   	     {
   	       tmpfloat = ((float)(tmp8209P - tmprealP)) / ((float)(tmprealP));
   	 	     tmpu16 = (u16)(256-(u8)(1654*tmpfloat));
   	     }
     	 }
     	 
     	 tmpbuf[1] = (u8)(tmpu16 >> 8);
	     tmpbuf[2] = (u8)(tmpu16);
     break;	
     case 0x12:   /*//////电流有效值校正/////*/
     case 0x13:   /*//////零线电流有效值校正/////*/
     case 0x14:   /*//////电压有效值校正/////*/
     	 if((regadr == 0x12) || (regadr == 0x13))
     	 {
     	 	 if(regadr == 0x12)
     	 	 {
     	 	   tmpadr = RN8209_REG_IA;
     	 	 }
     	 	 else
     	 	 {
     	 	 	 tmpadr = RN8209_REG_IB;
     	 	 }
     	 	 tmprealP = ((u32)(*(ptr+2)) << 16) + ((u32)(*(ptr+1)) << 8) + (*(ptr));   	 
   	     tmprealP = BCDLongTOHexlong(tmprealP); /*//////RN8209功率值/////*/
     	 	 tmpcal = 1000;
     	 }
     	 else
     	 {
     	 	 tmpadr = RN8209_REG_U;
     	 	 tmprealP = (u32)(BcdTOHexInt(((u16)(*(ptr+1)) << 8) + (*(ptr)))); 
     	 	 tmpcal = 10;
     	 }
     	 
     	 RN8209_ReadData(tmpadr,&tmpbuf[0],&tmpbyte);  /*//////读取寄存器的值/////*/
   	   tmp8209P = ((u32)(tmpbuf[0]) << 16) + ((u32)(tmpbuf[1]) << 8) + tmpbuf[2];   	       
       tmprealP = (u32)((tmpcal*tmp8209P)/tmprealP); /*//////结果等于寄存器值除以实际值/////*/ 
       
       tmpbuf[1] = (u8)(tmprealP >> 16);
	     tmpbuf[2] = (u8)(tmprealP >> 8); 
	     tmpbuf[3] = (u8)(tmprealP);
	   break;
	   case 0x0A:     /*//////有功功率offset校正/////*/
	   	 /*//////台体电流打到5%Ib,1.0,若此时表误差不好,需设此值,好的话就不用设置/////*/
	   	 RN8209_ReadData(RN8209_REG_PA,&tmpbuf[0],&tmpbyte); /*//////读取寄存器的值/////*/
	   	 tmpu16 = ((u16)(tmpbuf[2]) << 8) + tmpbuf[3];	   	 
	   	 tmpu16 = (~tmpu16)+1;
	   	 tmpbuf[1] = (u8)(tmpu16 >> 8); 
	     tmpbuf[2] = (u8)(tmpu16);
	   break;
	   case 0x0E:     /*//////电流有效值offset校正/////*/
	   case 0x0f:     /*//////零线电流有效值offset校正/////*/
	   	 if(regadr == 0x0e)
	   	 {
     	 	 tmpadr = RN8209_REG_IA;
     	 }
     	 else if(regadr == 0x0f)
     	 {
     	 	 tmpadr = RN8209_REG_IB;
     	 }	
	   	 RN8209_ReadData(RN8209_REG_IA,&tmpbuf[0],&tmpbyte); /*//////读取寄存器的值/////*/
	   	 
	   	 tmpu16 = ((u16)(tmpbuf[1]) << 8) + tmpbuf[2];
	   	 tmp8209P = ((u32)(tmpu16))*tmpu16;	   	 
	   	 tmp8209P = (~tmp8209P)+1;
	   	 
	   	 tmpbuf[1] = (u8)(tmp8209P >> 16); 
	     tmpbuf[2] = (u8)(tmp8209P >> 8);
	   break;
	   case 0xff:     /*//////校表初始化/////*/
	     RN8209_PInitDefault();
 	 	   RN8209_LoadJBCS();
 	 	 
 	 	   return;      
	   break;
     default:break;
   }                    	                   
	 
	 tmpbuf[0] = regadr;     /*//////写入校表参数到EEP中和RN8209中/////*/
   RN8209_WriteDataToEE(&tmpbuf[0]);
	 RN8209_WriteData(&tmpbuf[0]);         
  
   if(regadr == 0x02)      /*//////计算功率换算系数/////*/
   {         	 
	   tmpu16 = ((u16)(tmpbuf[1]) << 8)+tmpbuf[2];
//	   tmprealP = ((u32)(tmpu16))*1600;     
     tmpfloat = ((float)(tmpu16))*133.3013*(IMPULSE_CONSTANT);
     tmprealP = (u32)(tmpfloat);           
	   UlongTOUcharbuf(tmprealP,&tmpbuf[1]);    		   
	   tmpbuf[0] = 0x11;
     RN8209_WriteDataToEE(&tmpbuf[0]);
   }
   
   RN8209_CalChkSum();   /*///// 更新8209校表数据累计校验和 //////*/
   
	 return;
}
