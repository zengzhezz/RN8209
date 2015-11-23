#ifndef  rn8209h
#define  rn8209h 
/***********************************************************************/
#define  RN8209_REG_IA   0x22 
#define  RN8209_REG_IB   0x23 
#define  RN8209_REG_U    0x24 
#define  RN8209_REG_FREQ 0x25 
#define  RN8209_REG_PA   0x26 
#define  RN8209_REG_STS  0x2D 
/***********************************************************************/
#define  TPOWERONTSTNUM  50 
#define  TNORMALTSTNUM   5 
/***********************************************************************/
void  RN8209_SPI_Ini(void);
void  RN8209_TestJBCS(u16 r8209chksum);
void  RN8209_InitDefault(void);
void  RN8209_PInitDefault(void);
void  RN8209_LoadJBCS(void);
void  RN8209_WriteData(u8 *ptr);
void  RN8209_CalChkSum(void);
void  RN8209_CalJBCS(u8 regadr,u8 *ptr);
u8 RN8209_Test(u8 testnum);
u8 RN8209_WriteDataToEE(u8 *ptr);
u8 RN8209_ReadData(u8 add,u8 *data,u8 *len);
/***********************************************************************/
#endif

