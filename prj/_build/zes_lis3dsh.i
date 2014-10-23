#line 1 "..\\src\\zes_lis3dsh.c"
#line 1 "..\\src\\zes_lis3dsh.h"


 




#line 1 "..\\src\\zes.h"


 
 





#line 1 "..\\lib\\nrf51822\\Include\\nrf.h"




























 





 
#line 1 "..\\lib\\nrf51822\\Include\\nrf51.h"

 








































 





 



 









 

typedef enum {
 
  Reset_IRQn                    = -15,               
  NonMaskableInt_IRQn           = -14,               
  HardFault_IRQn                = -13,               
  SVCall_IRQn                   =  -5,               
  DebugMonitor_IRQn             =  -4,               
  PendSV_IRQn                   =  -2,               
  SysTick_IRQn                  =  -1,               
 
  POWER_CLOCK_IRQn              =   0,               
  RADIO_IRQn                    =   1,               
  UART0_IRQn                    =   2,               
  SPI0_TWI0_IRQn                =   3,               
  SPI1_TWI1_IRQn                =   4,               
  GPIOTE_IRQn                   =   6,               
  ADC_IRQn                      =   7,               
  TIMER0_IRQn                   =   8,               
  TIMER1_IRQn                   =   9,               
  TIMER2_IRQn                   =  10,               
  RTC0_IRQn                     =  11,               
  TEMP_IRQn                     =  12,               
  RNG_IRQn                      =  13,               
  ECB_IRQn                      =  14,               
  CCM_AAR_IRQn                  =  15,               
  WDT_IRQn                      =  16,               
  RTC1_IRQn                     =  17,               
  QDEC_IRQn                     =  18,               
  LPCOMP_IRQn                   =  19,               
  SWI0_IRQn                     =  20,               
  SWI1_IRQn                     =  21,               
  SWI2_IRQn                     =  22,               
  SWI3_IRQn                     =  23,               
  SWI4_IRQn                     =  24,               
  SWI5_IRQn                     =  25                
} IRQn_Type;




 


 
 
 

 




   

#line 1 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"
 







 

























 
























 




 


 

 













#line 110 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"


 







#line 145 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"

#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 







 




  
 







#line 37 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 208 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 272 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 147 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 



#line 292 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cmInstr.h"


#line 684 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cmInstr.h"

   

#line 148 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 271 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cmFunc.h"


#line 307 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cmFunc.h"


#line 634 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cmFunc.h"

 

#line 149 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"








 
#line 174 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"

 






 
#line 190 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
  else {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) & 0xFF) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 










#line 120 "..\\lib\\nrf51822\\Include\\nrf51.h"
#line 1 "..\\lib\\nrf51822\\Include\\system_nrf51.h"




























 







#line 38 "..\\lib\\nrf51822\\Include\\system_nrf51.h"


extern uint32_t SystemCoreClock;     









 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);





#line 121 "..\\lib\\nrf51822\\Include\\nrf51.h"


 
 
 




 


 

  #pragma push
  #pragma anon_unions
#line 148 "..\\lib\\nrf51822\\Include\\nrf51.h"


typedef struct {
  volatile uint32_t  CPU0;                               
  volatile uint32_t  SPIS1;                              
  volatile uint32_t  RADIO;                              
  volatile uint32_t  ECB;                                
  volatile uint32_t  CCM;                                
  volatile uint32_t  AAR;                                
} AMLI_RAMPRI_Type;

typedef struct {
  volatile  uint32_t  EN;                                 
  volatile  uint32_t  DIS;                                
} PPI_TASKS_CHG_Type;

typedef struct {
  volatile uint32_t  EEP;                                
  volatile uint32_t  TEP;                                
} PPI_CH_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[30];
  volatile  uint32_t  TASKS_CONSTLAT;                     
  volatile  uint32_t  TASKS_LOWPWR;                       
  volatile const  uint32_t  RESERVED1[34];
  volatile uint32_t  EVENTS_POFWARN;                     
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile uint32_t  RESETREAS;                          
  volatile const  uint32_t  RESERVED4[9];
  volatile const  uint32_t  RAMSTATUS;                          
  volatile const  uint32_t  RESERVED5[53];
  volatile  uint32_t  SYSTEMOFF;                          
  volatile const  uint32_t  RESERVED6[3];
  volatile uint32_t  POFCON;                             
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  GPREGRET;                          
 
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  RAMON;                              
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RESET;                             
 
  volatile const  uint32_t  RESERVED10[3];
  volatile uint32_t  RAMONB;                             
  volatile const  uint32_t  RESERVED11[8];
  volatile uint32_t  DCDCEN;                             
  volatile const  uint32_t  RESERVED12[291];
  volatile uint32_t  DCDCFORCE;                          
} NRF_POWER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_HFCLKSTART;                   
  volatile  uint32_t  TASKS_HFCLKSTOP;                    
  volatile  uint32_t  TASKS_LFCLKSTART;                   
  volatile  uint32_t  TASKS_LFCLKSTOP;                    
  volatile  uint32_t  TASKS_CAL;                          
  volatile  uint32_t  TASKS_CTSTART;                      
  volatile  uint32_t  TASKS_CTSTOP;                       
  volatile const  uint32_t  RESERVED0[57];
  volatile uint32_t  EVENTS_HFCLKSTARTED;                
  volatile uint32_t  EVENTS_LFCLKSTARTED;                
  volatile const  uint32_t  RESERVED1;
  volatile uint32_t  EVENTS_DONE;                        
  volatile uint32_t  EVENTS_CTTO;                        
  volatile const  uint32_t  RESERVED2[124];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[63];
  volatile const  uint32_t  HFCLKRUN;                           
  volatile const  uint32_t  HFCLKSTAT;                          
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  LFCLKRUN;                           
  volatile const  uint32_t  LFCLKSTAT;                          
  volatile const  uint32_t  LFCLKSRCCOPY;                      
 
  volatile const  uint32_t  RESERVED5[62];
  volatile uint32_t  LFCLKSRC;                           
  volatile const  uint32_t  RESERVED6[7];
  volatile uint32_t  CTIV;                               
  volatile const  uint32_t  RESERVED7[5];
  volatile uint32_t  XTALFREQ;                           
} NRF_CLOCK_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[330];
  volatile uint32_t  PERR0;                              
  volatile uint32_t  RLENR0;                             
  volatile const  uint32_t  RESERVED1[52];
  volatile uint32_t  PROTENSET0;                         
  volatile uint32_t  PROTENSET1;                         
  volatile uint32_t  DISABLEINDEBUG;                     
  volatile uint32_t  PROTBLOCKSIZE;                      
} NRF_MPU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[448];
  volatile uint32_t  REPLACEADDR[8];                     
  volatile const  uint32_t  RESERVED1[24];
  volatile uint32_t  PATCHADDR[8];                       
  volatile const  uint32_t  RESERVED2[24];
  volatile uint32_t  PATCHEN;                            
  volatile uint32_t  PATCHENSET;                         
  volatile uint32_t  PATCHENCLR;                         
} NRF_PU_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[896];
  AMLI_RAMPRI_Type RAMPRI;                           
} NRF_AMLI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_TXEN;                         
  volatile  uint32_t  TASKS_RXEN;                         
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_DISABLE;                      
  volatile  uint32_t  TASKS_RSSISTART;                    
  volatile  uint32_t  TASKS_RSSISTOP;                     
  volatile  uint32_t  TASKS_BCSTART;                      
  volatile  uint32_t  TASKS_BCSTOP;                       
  volatile const  uint32_t  RESERVED0[55];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_ADDRESS;                     
  volatile uint32_t  EVENTS_PAYLOAD;                     
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_DISABLED;                    
  volatile uint32_t  EVENTS_DEVMATCH;                    
  volatile uint32_t  EVENTS_DEVMISS;                     
  volatile uint32_t  EVENTS_RSSIEND;                    
 
  volatile const  uint32_t  RESERVED1[2];
  volatile uint32_t  EVENTS_BCMATCH;                     
  volatile const  uint32_t  RESERVED2[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[61];
  volatile const  uint32_t  CRCSTATUS;                          
  volatile const  uint32_t  CD;                                 
  volatile const  uint32_t  RXMATCH;                            
  volatile const  uint32_t  RXCRC;                              
  volatile const  uint32_t  DAI;                                
  volatile const  uint32_t  RESERVED5[60];
  volatile uint32_t  PACKETPTR;                          
  volatile uint32_t  FREQUENCY;                          
  volatile uint32_t  TXPOWER;                            
  volatile uint32_t  MODE;                               
  volatile uint32_t  PCNF0;                              
  volatile uint32_t  PCNF1;                              
  volatile uint32_t  BASE0;                              
  volatile uint32_t  BASE1;                              
  volatile uint32_t  PREFIX0;                            
  volatile uint32_t  PREFIX1;                            
  volatile uint32_t  TXADDRESS;                          
  volatile uint32_t  RXADDRESSES;                        
  volatile uint32_t  CRCCNF;                             
  volatile uint32_t  CRCPOLY;                            
  volatile uint32_t  CRCINIT;                            
  volatile uint32_t  TEST;                               
  volatile uint32_t  TIFS;                               
  volatile const  uint32_t  RSSISAMPLE;                         
  volatile const  uint32_t  RESERVED6;
  volatile const  uint32_t  STATE;                              
  volatile uint32_t  DATAWHITEIV;                        
  volatile const  uint32_t  RESERVED7[2];
  volatile uint32_t  BCC;                                
  volatile const  uint32_t  RESERVED8[39];
  volatile uint32_t  DAB[8];                             
  volatile uint32_t  DAP[8];                             
  volatile uint32_t  DACNF;                              
  volatile const  uint32_t  RESERVED9[56];
  volatile uint32_t  OVERRIDE0;                          
  volatile uint32_t  OVERRIDE1;                          
  volatile uint32_t  OVERRIDE2;                          
  volatile uint32_t  OVERRIDE3;                          
  volatile uint32_t  OVERRIDE4;                          
  volatile const  uint32_t  RESERVED10[561];
  volatile uint32_t  POWER;                              
} NRF_RADIO_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile  uint32_t  TASKS_STOPRX;                       
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile  uint32_t  TASKS_STOPTX;                       
  volatile const  uint32_t  RESERVED0[3];
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile const  uint32_t  RESERVED1[56];
  volatile uint32_t  EVENTS_CTS;                         
  volatile uint32_t  EVENTS_NCTS;                        
  volatile uint32_t  EVENTS_RXDRDY;                      
  volatile const  uint32_t  RESERVED2[4];
  volatile uint32_t  EVENTS_TXDRDY;                      
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED4[7];
  volatile uint32_t  EVENTS_RXTO;                        
  volatile const  uint32_t  RESERVED5[46];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED6[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED7[93];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED8[31];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED9;
  volatile uint32_t  PSELRTS;                            
  volatile uint32_t  PSELTXD;                            
  volatile uint32_t  PSELCTS;                            
  volatile uint32_t  PSELRXD;                            
  volatile const  uint32_t  RXD;                               

 
  volatile  uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED10;
  volatile uint32_t  BAUDRATE;                           
  volatile const  uint32_t  RESERVED11[17];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED12[675];
  volatile uint32_t  POWER;                              
} NRF_UART_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[66];
  volatile uint32_t  EVENTS_READY;                       
  volatile const  uint32_t  RESERVED1[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[125];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELMISO;                           
  volatile const  uint32_t  RESERVED4;
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED7[681];
  volatile uint32_t  POWER;                              
} NRF_SPI_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTRX;                      
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STARTTX;                      
  volatile const  uint32_t  RESERVED1[2];
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED2;
  volatile  uint32_t  TASKS_SUSPEND;                      
  volatile  uint32_t  TASKS_RESUME;                       
  volatile const  uint32_t  RESERVED3[56];
  volatile uint32_t  EVENTS_STOPPED;                     
  volatile uint32_t  EVENTS_RXDREADY;                    
  volatile const  uint32_t  RESERVED4[4];
  volatile uint32_t  EVENTS_TXDSENT;                     
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED6[4];
  volatile uint32_t  EVENTS_BB;                          
  volatile const  uint32_t  RESERVED7[3];
  volatile uint32_t  EVENTS_SUSPENDED;                   
  volatile const  uint32_t  RESERVED8[45];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED9[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED10[110];
  volatile uint32_t  ERRORSRC;                           
  volatile const  uint32_t  RESERVED11[14];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  PSELSCL;                            
  volatile uint32_t  PSELSDA;                            
  volatile const  uint32_t  RESERVED13[2];
  volatile const  uint32_t  RXD;                                
  volatile uint32_t  TXD;                                
  volatile const  uint32_t  RESERVED14;
  volatile uint32_t  FREQUENCY;                          
  volatile const  uint32_t  RESERVED15[24];
  volatile uint32_t  ADDRESS;                            
  volatile const  uint32_t  RESERVED16[668];
  volatile uint32_t  POWER;                              
} NRF_TWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[9];
  volatile  uint32_t  TASKS_ACQUIRE;                      
  volatile  uint32_t  TASKS_RELEASE;                      
  volatile const  uint32_t  RESERVED1[54];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED2[8];
  volatile uint32_t  EVENTS_ACQUIRED;                    
  volatile const  uint32_t  RESERVED3[53];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED4[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED5[61];
  volatile const  uint32_t  SEMSTAT;                            
  volatile const  uint32_t  RESERVED6[15];
  volatile uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED7[47];
  volatile uint32_t  ENABLE;                             
  volatile const  uint32_t  RESERVED8;
  volatile uint32_t  PSELSCK;                            
  volatile uint32_t  PSELMISO;                           
  volatile uint32_t  PSELMOSI;                           
  volatile uint32_t  PSELCSN;                            
  volatile const  uint32_t  RESERVED9[7];
  volatile uint32_t  RXDPTR;                             
  volatile uint32_t  MAXRX;                              
  volatile const  uint32_t  AMOUNTRX;                           
  volatile const  uint32_t  RESERVED10;
  volatile uint32_t  TXDPTR;                             
  volatile uint32_t  MAXTX;                              
  volatile const  uint32_t  AMOUNTTX;                           
  volatile const  uint32_t  RESERVED11;
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED12;
  volatile uint32_t  DEF;                                
  volatile const  uint32_t  RESERVED13[24];
  volatile uint32_t  ORC;                                
  volatile const  uint32_t  RESERVED14[654];
  volatile uint32_t  POWER;                              
} NRF_SPIS_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_OUT[4];                       
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_IN[4];                       
  volatile const  uint32_t  RESERVED1[27];
  volatile uint32_t  EVENTS_PORT;                        
  volatile const  uint32_t  RESERVED2[97];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[129];
  volatile uint32_t  CONFIG[4];                          
  volatile const  uint32_t  RESERVED4[695];
  volatile uint32_t  POWER;                              
} NRF_GPIOTE_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_END;                         
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  BUSY;                               
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_ADC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_COUNT;                        
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_SHUTDOWN;                     
  volatile const  uint32_t  RESERVED0[11];
  volatile  uint32_t  TASKS_CAPTURE[4];                   
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[44];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED3[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED4[126];
  volatile uint32_t  MODE;                               
  volatile uint32_t  BITMODE;                            
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED6[11];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED7[683];
  volatile uint32_t  POWER;                              
} NRF_TIMER_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_CLEAR;                        
  volatile  uint32_t  TASKS_TRIGOVRFLW;                   
  volatile const  uint32_t  RESERVED0[60];
  volatile uint32_t  EVENTS_TICK;                        
  volatile uint32_t  EVENTS_OVRFLW;                      
  volatile const  uint32_t  RESERVED1[14];
  volatile uint32_t  EVENTS_COMPARE[4];                  
  volatile const  uint32_t  RESERVED2[109];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[13];
  volatile uint32_t  EVTEN;                              
  volatile uint32_t  EVTENSET;                          
 
  volatile uint32_t  EVTENCLR;                          
 
  volatile const  uint32_t  RESERVED4[110];
  volatile const  uint32_t  COUNTER;                            
  volatile uint32_t  PRESCALER;                         
 
  volatile const  uint32_t  RESERVED5[13];
  volatile uint32_t  CC[4];                              
  volatile const  uint32_t  RESERVED6[683];
  volatile uint32_t  POWER;                              
} NRF_RTC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_DATARDY;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[127];
  volatile const  int32_t   TEMP;                               
  volatile const  uint32_t  RESERVED3[700];
  volatile uint32_t  POWER;                              
} NRF_TEMP_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_VALRDY;                      
  volatile const  uint32_t  RESERVED1[63];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[126];
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  VALUE;                              
  volatile const  uint32_t  RESERVED4[700];
  volatile uint32_t  POWER;                              
} NRF_RNG_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_STARTECB;                    

 
  volatile  uint32_t  TASKS_STOPECB;                     
 
  volatile const  uint32_t  RESERVED0[62];
  volatile uint32_t  EVENTS_ENDECB;                      
  volatile uint32_t  EVENTS_ERRORECB;                   
 
  volatile const  uint32_t  RESERVED1[127];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  ECBDATAPTR;                         
  volatile const  uint32_t  RESERVED3[701];
  volatile uint32_t  POWER;                              
} NRF_ECB_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                       
 
  volatile const  uint32_t  RESERVED0;
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  EVENTS_END;                         
  volatile uint32_t  EVENTS_RESOLVED;                    
  volatile uint32_t  EVENTS_NOTRESOLVED;                 
  volatile const  uint32_t  RESERVED2[126];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  STATUS;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  NIRK;                               
  volatile uint32_t  IRKPTR;                             
  volatile const  uint32_t  RESERVED5;
  volatile uint32_t  ADDRPTR;                            
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED6[697];
  volatile uint32_t  POWER;                              
} NRF_AAR_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_KSGEN;                       
 
  volatile  uint32_t  TASKS_CRYPT;                       
 
  volatile  uint32_t  TASKS_STOP;                         
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_ENDKSGEN;                    
  volatile uint32_t  EVENTS_ENDCRYPT;                    
  volatile uint32_t  EVENTS_ERROR;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  MICSTATUS;                          
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  MODE;                               
  volatile uint32_t  CNFPTR;                             
  volatile uint32_t  INPTR;                              
  volatile uint32_t  OUTPTR;                             
  volatile uint32_t  SCRATCHPTR;                        
 
  volatile const  uint32_t  RESERVED5[697];
  volatile uint32_t  POWER;                              
} NRF_CCM_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile const  uint32_t  RESERVED0[63];
  volatile uint32_t  EVENTS_TIMEOUT;                     
  volatile const  uint32_t  RESERVED1[128];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED2[61];
  volatile const  uint32_t  RUNSTATUS;                          
  volatile const  uint32_t  REQSTATUS;                          
  volatile const  uint32_t  RESERVED3[63];
  volatile uint32_t  CRV;                                
  volatile uint32_t  RREN;                               
  volatile uint32_t  CONFIG;                             
  volatile const  uint32_t  RESERVED4[60];
  volatile  uint32_t  RR[8];                              
  volatile const  uint32_t  RESERVED5[631];
  volatile uint32_t  POWER;                              
} NRF_WDT_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_READCLRACC;                  
 
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_SAMPLERDY;                   
  volatile uint32_t  EVENTS_REPORTRDY;                  
 
  volatile uint32_t  EVENTS_ACCOF;                       
  volatile const  uint32_t  RESERVED1[61];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[125];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  LEDPOL;                             
  volatile uint32_t  SAMPLEPER;                          
  volatile const  int32_t   SAMPLE;                             
  volatile uint32_t  REPORTPER;                          
  volatile const  int32_t   ACC;                                
  volatile const  int32_t   ACCREAD;                           
 
  volatile uint32_t  PSELLED;                            
  volatile uint32_t  PSELA;                              
  volatile uint32_t  PSELB;                              
  volatile uint32_t  DBFEN;                              
  volatile const  uint32_t  RESERVED4[5];
  volatile uint32_t  LEDPRE;                             
  volatile const  uint32_t  ACCDBL;                             
  volatile const  uint32_t  ACCDBLREAD;                        
 
  volatile const  uint32_t  RESERVED5[684];
  volatile uint32_t  POWER;                              
} NRF_QDEC_Type;


 
 
 




 

typedef struct {                                     
  volatile  uint32_t  TASKS_START;                        
  volatile  uint32_t  TASKS_STOP;                         
  volatile  uint32_t  TASKS_SAMPLE;                       
  volatile const  uint32_t  RESERVED0[61];
  volatile uint32_t  EVENTS_READY;                       
  volatile uint32_t  EVENTS_DOWN;                        
  volatile uint32_t  EVENTS_UP;                          
  volatile uint32_t  EVENTS_CROSS;                       
  volatile const  uint32_t  RESERVED1[60];
  volatile uint32_t  SHORTS;                             
  volatile const  uint32_t  RESERVED2[64];
  volatile uint32_t  INTENSET;                           
  volatile uint32_t  INTENCLR;                           
  volatile const  uint32_t  RESERVED3[61];
  volatile const  uint32_t  RESULT;                             
  volatile const  uint32_t  RESERVED4[63];
  volatile uint32_t  ENABLE;                             
  volatile uint32_t  PSEL;                               
  volatile uint32_t  REFSEL;                             
  volatile uint32_t  EXTREFSEL;                          
  volatile const  uint32_t  RESERVED5[4];
  volatile uint32_t  ANADETECT;                          
  volatile const  uint32_t  RESERVED6[694];
  volatile uint32_t  POWER;                              
} NRF_LPCOMP_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  UNUSED;                             
} NRF_SWI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[256];
  volatile const  uint32_t  READY;                              
  volatile const  uint32_t  RESERVED1[64];
  volatile uint32_t  CONFIG;                             
  volatile uint32_t  ERASEPAGE;                          
  volatile uint32_t  ERASEALL;                           
  volatile uint32_t  ERASEPROTECTEDPAGE;                 
  volatile uint32_t  ERASEUICR;                          
} NRF_NVMC_Type;


 
 
 




 

typedef struct {                                     
  PPI_TASKS_CHG_Type TASKS_CHG[4];                   
  volatile const  uint32_t  RESERVED0[312];
  volatile uint32_t  CHEN;                               
  volatile uint32_t  CHENSET;                            
  volatile uint32_t  CHENCLR;                            
  volatile const  uint32_t  RESERVED1;
  PPI_CH_Type CH[16];                                
  volatile const  uint32_t  RESERVED2[156];
  volatile uint32_t  CHG[4];                             
} NRF_PPI_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[4];
  volatile const  uint32_t  CODEPAGESIZE;                       
  volatile const  uint32_t  CODESIZE;                           
  volatile const  uint32_t  RESERVED1[4];
  volatile const  uint32_t  CLENR0;                             
  volatile const  uint32_t  PPFC;                               
  volatile const  uint32_t  RESERVED2;
  volatile const  uint32_t  NUMRAMBLOCK;                        
  
  union {
    volatile const  uint32_t  SIZERAMBLOCK[4];                 

 
    volatile const  uint32_t  SIZERAMBLOCKS;                    
  };
  volatile const  uint32_t  RESERVED3[5];
  volatile const  uint32_t  CONFIGID;                           
  volatile const  uint32_t  DEVICEID[2];                        
  volatile const  uint32_t  RESERVED4[6];
  volatile const  uint32_t  ER[4];                              
  volatile const  uint32_t  IR[4];                              
  volatile const  uint32_t  DEVICEADDRTYPE;                     
  volatile const  uint32_t  DEVICEADDR[2];                      
  volatile const  uint32_t  OVERRIDEEN;                         
  volatile const  uint32_t  NRF_1MBIT[5];                      
 
  volatile const  uint32_t  RESERVED5[10];
  volatile const  uint32_t  BLE_1MBIT[5];                      
 
} NRF_FICR_Type;


 
 
 




 

typedef struct {                                     
  volatile uint32_t  CLENR0;                             
  volatile uint32_t  RBPCONF;                            
  volatile uint32_t  XTALFREQ;                           
  volatile const  uint32_t  RESERVED0;
  volatile const  uint32_t  FWID;                               
  volatile uint32_t  BOOTLOADERADDR;                     
} NRF_UICR_Type;


 
 
 




 

typedef struct {                                     
  volatile const  uint32_t  RESERVED0[321];
  volatile uint32_t  OUT;                                
  volatile uint32_t  OUTSET;                             
  volatile uint32_t  OUTCLR;                             
  volatile const  uint32_t  IN;                                 
  volatile uint32_t  DIR;                                
  volatile uint32_t  DIRSET;                             
  volatile uint32_t  DIRCLR;                             
  volatile const  uint32_t  RESERVED1[120];
  volatile uint32_t  PIN_CNF[32];                        
} NRF_GPIO_Type;


 

  #pragma pop
#line 1138 "..\\lib\\nrf51822\\Include\\nrf51.h"




 
 
 

#line 1179 "..\\lib\\nrf51822\\Include\\nrf51.h"


 
 
 

#line 1218 "..\\lib\\nrf51822\\Include\\nrf51.h"


   
   
   








#line 38 "..\\lib\\nrf51822\\Include\\nrf.h"
#line 1 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"




























 



 

#line 1 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"
 







 

























 






#line 151 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"



#line 697 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\4.1.1\\CMSIS\\Include\\core_cm0.h"





#line 36 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 
 

 






 






 






 
 

 






 






 






 
 

 



 
 

 





 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 





 
 

 






 
#line 184 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 192 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 201 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 






 
 

 



 
 

 






 
 

 
 

 
#line 243 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 255 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 267 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 279 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 291 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 303 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 315 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 327 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 
#line 342 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 354 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 366 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 378 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 390 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 402 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 414 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 426 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 
#line 441 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 453 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 465 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 477 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 489 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 501 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 513 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 525 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 
#line 540 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 552 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 564 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 576 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 588 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 600 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 612 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 624 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 
#line 639 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 651 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 663 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 675 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 687 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 699 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 711 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 723 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 
#line 738 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 750 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 762 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 774 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 786 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 798 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 810 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
#line 822 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 
 

 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 





 
 

 






 
 

 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 





 
 

 





 
 

 





 






 
 

 






 
 

 






 
 

 



 
 

 






 
 

 
 

 






 






 
 

 






 






 
 

 






 
 

 
 

 





 
 

 



 



 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 
#line 2683 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 






 





 






 
 

 
 

 






 






 






 






 






 
 

 






 






 






 






 






 
 

 





 






 



 






 
 

 






 
 

 
 

 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 





 
 

 
#line 2950 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 
#line 2965 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 
 

 





 
 

 
 

 





 
 

 






 
 

 





 
 

 






 
 

 
 

 






 
 

 






 
 

 



 



 



 



 



 



 



 
 

 





 





 





 





 
 

 




 
 

 
#line 3739 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 





 
 

 



 
 

 





 





 





 





 
 

 





 
 

 





 





 





 





 
 

 





 
 

 





 






 
 

 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 
 

 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 





 






 
 

 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 







 
 

 
 

 





 





 
 

 






 






 






 
 

 






 






 






 
 

 





 
 

 





 
 

 
#line 4863 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 
#line 4885 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 





 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 






 






 






 






 






 
 

 






 






 






 






 






 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 



 
 

 
#line 5170 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 
#line 5181 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 



 



 



 
 

 





 





 



 



 



 
 

 



 



 



 



 
 

 



 



 



 



 
 

 



 
 

 





 





 





 





 





 





 





 





 
 

 





 
#line 5336 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 





 





 
 

 



 
 

 



 
 

 
#line 5395 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 



 
 

 



 



 



 



 



 



 



 



 





 





 





 





 





 





 





 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 



 
 

 






 
 

 
 

 





 
 

 






 
 

 






 
 

 





 
 

 



 
 

 






 
 

 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 





 





 





 





 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 



 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 





 
 

 



 
 

 



 
 

 
#line 5914 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 





 





 





 
 

 






 
 

 
 

 





 
 

 






 






 
 

 






 






 
 

 
#line 6002 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 






 






 
 

 





 
 

 



 
 

 



 
 

 



 
 

 



 
 

 





 





 





 
 

 



 
 

 



 
 

 






 
 

 
 

 






 
 

 






 
 

 






 
 

 
 

 





 





 





 





 





 





 





 





 
 

 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 
#line 6270 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 



 
 

 






 
 

 
 

 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 
 

 





 
 

 



 
 

 



 
 

 






 
 

 



 
 

 






 
 

 
 

 





 





 
 

 






 






 






 






 






 






 
 

 






 






 






 






 






 






 
 

 






 






 






 






 
 

 





 
 

 



 
 

 



 
 

 
#line 6647 "..\\lib\\nrf51822\\Include\\nrf51_bitfields.h"

 
 

 





 





 
 

 






 
 

 
 

 





 





 
 

 





 
 

 




 
 

 
 

 






 
 

 






 
 

 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 





 





 





 





 





 





 
 

 





 





 
 

 




 
 

 






 
#line 39 "..\\lib\\nrf51822\\Include\\nrf.h"
#line 1 "..\\lib\\nrf51822\\Include\\nrf51_deprecated.h"




























 



 




 

 
 

 

 
 
 

 
 
 
 




 
 
 
 




 




 




 





 
 
 

 




 




 






 
 




 


 




 




 




 
 
#line 136 "..\\lib\\nrf51822\\Include\\nrf51_deprecated.h"
 
#line 169 "..\\lib\\nrf51822\\Include\\nrf51_deprecated.h"
 




 
#line 431 "..\\lib\\nrf51822\\Include\\nrf51_deprecated.h"



 



#line 40 "..\\lib\\nrf51822\\Include\\nrf.h"




#line 1 "..\\lib\\nrf51822\\Include\\compiler_abstraction.h"




























 



 


    



    



    

  
#line 90 "..\\lib\\nrf51822\\Include\\compiler_abstraction.h"

 

#line 45 "..\\lib\\nrf51822\\Include\\nrf.h"





#line 11 "..\\src\\zes.h"

#line 13 "..\\src\\zes.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"



#line 14 "..\\src\\zes.h"

#line 1 "..\\inc\\board.h"


 




#line 1 "..\\inc\\sam_pin.h"


 




#line 24 "..\\inc\\sam_pin.h"


#line 38 "..\\inc\\sam_pin.h"

#line 12 "..\\inc\\board.h"
#line 19 "..\\inc\\board.h"

#line 16 "..\\src\\zes.h"
#line 1 "..\\src\\zes_res.h"


 




#line 9 "..\\src\\zes_res.h"

 


 
enum res_value_t
{
	res_Ok = 0, 
	res_Err = -1, 
	res_Error = res_Err, 

	res_ESystem = -16384, 
	res_ERange, 
	res_EArgument, 
	res_EOverflow, 
	res_ETimeout, 
	res_EFormat, 
	res_EBusy, 
	res_ECancel, 
	res_EFail, 
	res_EAbort, 
	res_ERefuse, 
	res_EReset, 
	res_EExist, 
	res_EIo, 
	res_EData, 
	res_EMemory, 
	res_EPermission, 
	res_EUnknown, 
};

typedef int16_t res_t; 



 



 
int resLog(res_t c, const char *m);






















































































 
#line 17 "..\\src\\zes.h"
#line 1 "..\\src\\zes_def.h"


 
 



#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"


  
  typedef unsigned int size_t;








extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 185 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 201 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 224 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 239 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 262 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 494 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\string.h"



 

#line 9 "..\\src\\zes_def.h"



#line 18 "..\\src\\zes.h"

#line 9 "..\\src\\zes_lis3dsh.h"
#line 1 "..\\src\\zes_sns.h"


 




#line 9 "..\\src\\zes_sns.h"

#line 1 "..\\src\\zes_bus.h"


 




#line 9 "..\\src\\zes_bus.h"
#line 10 "..\\src\\zes_bus.h"

#line 12 "..\\src\\zes_bus.h"

enum zes_bus_value_t
{
	
	zes_bus_Twi = 0,
	zes_bus_Spi,

	
	zes_bus_Tx = 1<<0, 
	zes_bus_Rx = 1<<1, 
	zes_bus_Stop = 1<<2, 
	zes_bus_Asynch = 1<<3, 
};












 



typedef struct zes_bus_t zes_bus_t;

 
typedef res_t (*zes_bus_init_t)(zes_bus_t *s);

 
typedef res_t (*zes_bus_trx_t)(zes_bus_t *s, uint8_t l, const uint8_t *din, uint8_t *dout, uint8_t f);

 
typedef void (*zes_bus_handler_t)(zes_bus_t *s);

 
struct zes_bus_t
{
	void *device; uint8_t type; res_t code; uint8_t speed; uint16_t timeout; uint8_t channel; uintptr_t context; zes_bus_init_t init; zes_bus_trx_t trx; zes_bus_handler_t handler;
};

#line 11 "..\\src\\zes_sns.h"

 
enum zes_sns_value_t
{
	
	zes_sns_Data = 1<<0,
};








 


typedef struct zes_sns_t zes_sns_t;

 
typedef void (*zes_sns_handler_t)(zes_sns_t *s);

 
struct zes_sns_t
{
	volatile uint8_t state; volatile res_t code; zes_bus_t *bus; zes_sns_handler_t handler; uint32_t last; uint8_t address;
};

 
res_t zes_sns_setup(zes_sns_t *s, zes_bus_t *b, zes_sns_handler_t h);

 
res_t zes_sns_read(zes_sns_t *s, uint8_t a, uint8_t l, uint8_t *b);

 
res_t zes_sns_write(zes_sns_t *s, uint8_t a, uint8_t l, const uint8_t *b);

#line 10 "..\\src\\zes_lis3dsh.h"
#line 1 "..\\inc\\lis3dsh.h"




 




#line 11 "..\\inc\\lis3dsh.h"

 






#line 27 "..\\inc\\lis3dsh.h"

#line 38 "..\\inc\\lis3dsh.h"







#line 51 "..\\inc\\lis3dsh.h"




#line 74 "..\\inc\\lis3dsh.h"

#line 95 "..\\inc\\lis3dsh.h"

 
enum Lis3dshInf1Value
{
	lis3dsh_INF1_Val = 0x21
};

 
enum Lis3dshInf2Value
{
	lis3dsh_INF2_Val = 0x00
};

 
enum Lis3dshIdValue
{
	lis3dsh_ID_Val = 0x3f
};

 
enum Lis3dshCtl3Value
{
	lis3dsh_STRT_Pos = 0, 
	lis3dsh_STRT_Len = 1,
	lis3dsh_STRT_Off = 0,
	lis3dsh_STRT_On,

	lis3dsh_VFLT_Pos = 2, 
	lis3dsh_VFLT_Len = 1,
	lis3dsh_VFLT_Off = 0,
	lis3dsh_VFLT_On,

	lis3dsh_INT1EN_Pos = 3, 
	lis3dsh_INT1EN_Len = 1,
	lis3dsh_INT1EN_Off = 0,
	lis3dsh_INT1EN_On,

	lis3dsh_INT2EN_Pos = 4, 
	lis3dsh_INT2EN_Len = 1,
	lis3dsh_INT2EN_Off = 0,
	lis3dsh_INT2EN_On,

	lis3dsh_IEL_Pos = 5, 
	lis3dsh_IEL_Len = 1,
	lis3dsh_IEL_On = 0,
	lis3dsh_IEL_Off,

	lis3dsh_IEA_Pos = 6, 
	lis3dsh_IEA_Len = 1,
	lis3dsh_IEA_Low = 0,
	lis3dsh_IEA_High,

	lis3dsh_DRDE_Pos = 7, 
	lis3dsh_DRDE_Len = 1,
	lis3dsh_DRDE_Off = 0,
	lis3dsh_DRDE_On,
};

 
enum Lis3dshCtl4Value
{
	lis3dsh_XEN_Pos = 0, 
	lis3dsh_XEN_Len = 1,
	lis3dsh_XEN_Off = 0,
	lis3dsh_XEN_On,

	lis3dsh_YEN_Pos = 1, 
	lis3dsh_YEN_Len = 1,
	lis3dsh_YEN_Off = 0,
	lis3dsh_YEN_On,

	lis3dsh_ZEN_Pos = 2, 
	lis3dsh_ZEN_Len = 1,
	lis3dsh_ZEN_Off = 0,
	lis3dsh_ZEN_On,

	lis3dsh_BDU_Pos = 3, 
	lis3dsh_BDU_Len = 1,
	lis3dsh_BDU_Off = 0,
	lis3dsh_BDU_On,

	lis3dsh_ODR_Pos = 4, 
	lis3dsh_ODR_Len = 5,
	lis3dsh_ODR_Off = 0, 
	lis3dsh_ODR_3_125Hz,
	lis3dsh_ODR_6_25Hz,
	lis3dsh_ODR_12_5Hz,
	lis3dsh_ODR_25Hz,
	lis3dsh_ODR_50Hz,
	lis3dsh_ODR_100Hz,
	lis3dsh_ODR_400Hz,
	lis3dsh_ODR_800Hz,
	lis3dsh_ODR_1600Hz,
};

 
enum Lis3dshCtl5Value
{
	lis3dsh_SIM_Pos = 0, 
	lis3dsh_SIM_Len = 1,
	lis3dsh_SIM_4Wire = 0,
	lis3dsh_SIM_3Wire,

	lis3dsh_ST_Pos = 1, 
	lis3dsh_ST_Len = 2,
	lis3dsh_ST_Off = 0, 
	lis3dsh_ST_Positive, 
	lis3dsh_ST_Negative, 

	lis3dsh_FS_Pos = 3, 
	lis3dsh_FS_Len = 3,
	lis3dsh_FS_2 = 0, 
	lis3dsh_FS_4,
	lis3dsh_FS_6,
	lis3dsh_FS_8,
	lis3dsh_FS_16,

	lis3dsh_BW_Pos = 6, 
	lis3dsh_BW_Len = 2,
	lis3dsh_BW_800Hz = 0,
	lis3dsh_BW_400Hz,
	lis3dsh_BW_200Hz,
	lis3dsh_BW_50Hz,
};

 
enum Lis3dshCtl6Value
{
	lis3dsh_P2BOOT_Pos = 0, 
	lis3dsh_P2BOOT_Len = 1,
	lis3dsh_P2BOOT_Off = 0,
	lis3dsh_P2BOOT_On,

	lis3dsh_P1OVR_Pos = 1, 
	lis3dsh_P1OVR_Len = 1,
	lis3dsh_P1OVR_Off = 0,
	lis3dsh_P1OVR_On,

	lis3dsh_P1WTM_Pos = 2, 
	lis3dsh_P1WTM_Len = 1,
	lis3dsh_P1WTM_Off = 0,
	lis3dsh_P1WTM_On,

	lis3dsh_P1EMP_Pos = 3, 
	lis3dsh_P1EMP_Len = 1,
	lis3dsh_P1EMP_Off = 0,
	lis3dsh_P1EMP_On,

	lis3dsh_ADDI_Pos = 4, 
	lis3dsh_ADDI_Len = 1,
	lis3dsh_ADDI_Off = 0,
	lis3dsh_ADDI_On,

	lis3dsh_WTME_Pos = 5, 
	lis3dsh_WTME_Len = 1,
	lis3dsh_WTME_Off = 0,
	lis3dsh_WTME_On,

	lis3dsh_FEN_Pos = 6, 
	lis3dsh_FEN_Len = 1,
	lis3dsh_FEN_Off = 0,
	lis3dsh_FEN_On,

	lis3dsh_BOOT_Pos = 7, 
	lis3dsh_BOOT_Len = 1,
	lis3dsh_BOOT_Off = 0,
	lis3dsh_BOOT_On,
};

 















 
 








 
 











































 
 
enum Lis3dshIcfgValue
{
	lis3dsh_IEN_Pos = 0, 
	lis3dsh_IEN_Len = 1,
	lis3dsh_IEN_Off = 0,
	lis3dsh_IEN_On,

	lis3dsh_LIR_Pos = 1, 
	lis3dsh_LIR_Len = 1,
	lis3dsh_LIR_On = 0, 
	lis3dsh_LIR_Off,






	lis3dsh_ZIEN_Pos = 5, 
	lis3dsh_ZIEN_Len = 1,
	lis3dsh_ZIEN_Off = 0,
	lis3dsh_ZIEN_On,

	lis3dsh_YIEN_Pos = 6, 
	lis3dsh_YIEN_Len = 1,
	lis3dsh_YIEN_Off = 0,
	lis3dsh_YIEN_On,

	lis3dsh_XIEN_Pos = 7, 
	lis3dsh_XIEN_Len = 1,
	lis3dsh_XIEN_Off = 0,
	lis3dsh_XIEN_On,

};

 
enum Lis3dshIsrcValue
{
	lis3dsh_IA_Pos = 0, 
	lis3dsh_IA_Len = 1,
	lis3dsh_IA_Off = 0,
	lis3dsh_IA_On,

	lis3dsh_MROI_Pos = 1, 
	lis3dsh_MROI_Len = 1,
	lis3dsh_MROI_Off = 0,
	lis3dsh_MROI_On,

	lis3dsh_ZNTH_Pos = 2, 
	lis3dsh_ZNTH_Len = 1,
	lis3dsh_ZNTH_Off = 0,
	lis3dsh_ZNTH_On,

	lis3dsh_YNTH_Pos = 3, 
	lis3dsh_YNTH_Len = 1,
	lis3dsh_YNTH_Off = 0,
	lis3dsh_YNTH_On,

	lis3dsh_XNTH_Pos = 4, 
	lis3dsh_XNTH_Len = 1,
	lis3dsh_XNTH_Off = 0,
	lis3dsh_XNTH_On,

	lis3dsh_ZPTH_Pos = 5, 
	lis3dsh_ZPTH_Len = 1,
	lis3dsh_ZPTH_Off = 0,
	lis3dsh_ZPTH_On,

	lis3dsh_YPTH_Pos = 6, 
	lis3dsh_YPTH_Len = 1,
	lis3dsh_YPTH_Off = 0,
	lis3dsh_YPTH_On,

	lis3dsh_XPTH_Pos = 7, 
	lis3dsh_XPTH_Len = 1,
	lis3dsh_XPTH_Off = 0,
	lis3dsh_XPTH_On,
};

 




#line 11 "..\\src\\zes_lis3dsh.h"

 
typedef struct
{
	float x_G; 
	float y_G; 
	float z_G; 
} zes_lis3dsh_d;

 
typedef struct
{
	volatile uint8_t state; volatile res_t code; zes_bus_t *bus; zes_sns_handler_t handler; uint32_t last; uint8_t address;
	zes_lis3dsh_d data;
} zes_lis3dsh_t;

 
res_t zes_lis3dsh_setup(zes_lis3dsh_t *s, zes_bus_t *b);

 
res_t zes_lis3dsh_init(zes_lis3dsh_t *s);

 
res_t zes_lis3dsh_data(zes_lis3dsh_t *s, zes_lis3dsh_d *d);

 
void zes_lis3dsh_handler(zes_lis3dsh_t *s);

#line 2 "..\\src\\zes_lis3dsh.c"

#line 1 "..\\lib\\nrf51822\\Include\\nrf_delay.h"



#line 5 "..\\lib\\nrf51822\\Include\\nrf_delay.h"

 

static __asm void __inline nrf_delay_us(uint32_t volatile number_of_us)
{
loop
        SUBS    R0, R0, #1
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        NOP
        BNE    loop
        BX     LR
}
#line 71 "..\\lib\\nrf51822\\Include\\nrf_delay.h"

void nrf_delay_ms(uint32_t volatile number_of_ms);

#line 4 "..\\src\\zes_lis3dsh.c"
#line 1 "..\\lib\\nrf51822\\Include\\spi_master.h"










 











 




#line 29 "..\\lib\\nrf51822\\Include\\spi_master.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
 
 
 




 
 



 







 




  
 










  


 








#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


  
  typedef unsigned int size_t;










    



    typedef unsigned short wchar_t;  
#line 86 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { __int64 quot, rem; } lldiv_t;
    


#line 107 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   



 

   




 
#line 126 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) __int64 atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) __int64 strtoll(const char * __restrict  ,
                               char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned __int64 strtoull(const char * __restrict  ,
                                         char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 429 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 517 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 546 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) __int64 llabs(__int64  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(__int64  , __int64  );
   











 
#line 627 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 885 "C:\\Keil_v5\\ARM\\ARMCC\\bin\\..\\include\\stdlib.h"


 

#line 30 "..\\lib\\nrf51822\\Include\\spi_master.h"





 
#line 49 "..\\lib\\nrf51822\\Include\\spi_master.h"

 
typedef struct
{
    uint32_t SPI_Freq;           
    uint32_t SPI_Pin_SCK;        
    uint32_t SPI_Pin_MISO;       
    uint32_t SPI_Pin_MOSI;       
    uint32_t SPI_Pin_SS;         
    uint32_t SPI_PriorityIRQ;    
    uint8_t SPI_CONFIG_ORDER;    
    uint8_t SPI_CONFIG_CPOL;     
    uint8_t SPI_CONFIG_CPHA;     
    uint8_t SPI_DisableAllIRQ;   
} spi_master_config_t;

 
typedef enum spi_master_evt_type_t
{
    SPI_MASTER_EVT_TRANSFER_STARTED = 0,     
    SPI_MASTER_EVT_TRANSFER_COMPLETED,       
    SPI_MASTER_EVT_TYPE_MAX                  
} spi_master_evt_type_t;

 
typedef struct
{
    spi_master_evt_type_t evt_type;  
    uint16_t data_count;             
} spi_master_evt_t;

 
typedef enum
{
    SPI_MASTER_STATE_DISABLED,   
    SPI_MASTER_STATE_BUSY,       
    SPI_MASTER_STATE_IDLE        
} spi_master_state_t;

 
typedef enum
{








    SPI_MASTER_HW_ENABLED_COUNT  
} spi_master_hw_instance_t;




 
typedef void (*spi_master_event_handler_t)(spi_master_evt_t spi_master_evt);

















 
uint32_t spi_master_open(const spi_master_hw_instance_t    spi_master_hw_instance,
                         spi_master_config_t const * const p_spi_master_config);








 
void spi_master_close(const spi_master_hw_instance_t spi_master_hw_instance);


















 
uint32_t spi_master_send_recv(const spi_master_hw_instance_t spi_master_hw_instance,
                              uint8_t * const p_tx_buf, const uint16_t tx_buf_len,
                              uint8_t * const p_rx_buf, const uint16_t rx_buf_len);










 
void spi_master_evt_handler_reg(const spi_master_hw_instance_t spi_master_hw_instance,
                                spi_master_event_handler_t     event_handler);











 
spi_master_state_t spi_master_get_state(const spi_master_hw_instance_t spi_master_hw_instance);


 
#line 5 "..\\src\\zes_lis3dsh.c"
#line 1 "..\\lib\\nrf51822\\Include\\sd_common\\app_util_platform.h"










 








 




#line 26 "..\\lib\\nrf51822\\Include\\sd_common\\app_util_platform.h"
#line 27 "..\\lib\\nrf51822\\Include\\sd_common\\app_util_platform.h"
#line 28 "..\\lib\\nrf51822\\Include\\sd_common\\app_util_platform.h"
#line 1 "..\\lib\\nrf51822\\Include\\app_common\\app_error.h"










 
 







 




#line 26 "..\\lib\\nrf51822\\Include\\app_common\\app_error.h"
#line 27 "..\\lib\\nrf51822\\Include\\app_common\\app_error.h"
#line 1 "..\\lib\\nrf51822\\Include\\s110\\nrf_error.h"







 
 




 

 




 




 

#line 46 "..\\lib\\nrf51822\\Include\\s110\\nrf_error.h"





 
#line 28 "..\\lib\\nrf51822\\Include\\app_common\\app_error.h"






 
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);




 









     
#line 60 "..\\lib\\nrf51822\\Include\\app_common\\app_error.h"
    



     
#line 74 "..\\lib\\nrf51822\\Include\\app_common\\app_error.h"



 
#line 29 "..\\lib\\nrf51822\\Include\\sd_common\\app_util_platform.h"

 
typedef enum
{
    APP_IRQ_PRIORITY_HIGH = 1,
    APP_IRQ_PRIORITY_LOW  = 3
} app_irq_priority_t;



 

 








 
#line 67 "..\\lib\\nrf51822\\Include\\sd_common\\app_util_platform.h"
    





 
#line 86 "..\\lib\\nrf51822\\Include\\sd_common\\app_util_platform.h"
        






 
static __inline uint8_t current_int_priority_get(void)
{
    uint32_t isr_vector_num = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->ICSR & (0x1FFUL << 0));
    if (isr_vector_num > 0)
    {
        int32_t irq_type = ((int32_t)isr_vector_num - 16);
        return (NVIC_GetPriority((IRQn_Type)irq_type) & 0xFF);
    }
    else
    {
        return 4;
    }
}



 
#line 6 "..\\src\\zes_lis3dsh.c"


res_t
zes_lis3dsh_setup(zes_lis3dsh_t *s, zes_bus_t *b)
{
	memset(s, 0, sizeof(*s));

	s->bus = b;
	s->address = 0x3c;

	return res_Ok;
}

res_t
zes_lis3dsh_init(zes_lis3dsh_t *s)
{
	uint8_t b;
	
	
		
	if (zes_sns_read((zes_sns_t *)s, 0x0f, 1, &b) != res_Ok)
		return -1; 
	
	zes_sns_read((zes_sns_t *)s, 0x20, 1, &b);
	b = b | (lis3dsh_BDU_On << lis3dsh_BDU_Pos) | (lis3dsh_ODR_1600Hz << lis3dsh_ODR_Pos);
	if (zes_sns_write((zes_sns_t *)s, 0x20, 1, &b) != res_Ok)
		return -2; 
	
	b = 0x10;
	if (zes_sns_write((zes_sns_t *)s, 0x25, 1, &b) != res_Ok)
		return -3; 
	
	
	
	



	if (b != lis3dsh_ID_Val)
		return -4; 
	
	return res_Ok; 
}

res_t
zes_lis3dsh_data(zes_lis3dsh_t *s, zes_lis3dsh_d *d)
{
	uint8_t buf[6];

	if (zes_sns_read((zes_sns_t *)s, 0x28, 6, buf) != res_Ok)
		return -3;

	s->data.x_G = (float)(*(int16_t *)&buf[0]);
	s->data.y_G = (float)(*(int16_t *)&buf[2]);
	s->data.z_G = (float)(*(int16_t *)&buf[4]);

	
	if (d)
		*d = s->data;
	

	return res_Ok;
}

void
zes_lis3dsh_handler(zes_lis3dsh_t *s)
{

}
