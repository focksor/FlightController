#include "RM.h"

unsigned short RM_CH_Length[9];
unsigned char CH_Def;

void RM_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = __RM_Pin;			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	CH_Def=0;
}//void RM_Init(void);

unsigned char RC_Com(unsigned char CH_Def,unsigned short Exp_Len){
	if((RM_CH_Length[CH_Def]-Exp_Len)>50 || (RM_CH_Length[CH_Def]-Exp_Len)<-50)
		return 0;
	else
		return 1;
}//bool RC_Com(unsigned char CH_Def,unsigned short Exp_Len);
