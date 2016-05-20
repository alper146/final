#include "stm32l4xx_hal.h"
#include "usb_device.h"
#include "rapidtest.h"
#include "string.h"
void getsensordata(void);
#define L_ARRAY 128
#define L_ARRAY_1 129
uint16_t cyclecounter=0;
uint16_t i=0;
uint8_t *p;
char buf2[7];
void nop(void){
unsigned int i=0;
	while(i<20){
	__nop();
		i++;
	}
} 
//a2=si
void firstclock(void){

GPIOB->BSRR=(1<<6);
nop();
			
GPIOB->BSRR=(1<<7);
nop();
			
GPIOB->BSRR=(1<<22);
nop();
	
GPIOB->BSRR=(1<<23);
	nop();
	 }
	uint8_t c=0;
volatile uint16_t value;

int main(void)
{
init(); 
HAL_Delay(500);
uint8_t message[]="started";
CDC_Transmit_FS(message,sizeof(message));	
firstclock();
  while (1)
  {
//	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==1){	
			getsensordata();
																							}
//	}
}
uint8_t data11[128];
uint16_t data12[128];
uint16_t data21[128];
uint8_t data22[128];

void getsensordata(void){
// PB6 --> CLOCK
// PB7 --> SI
// uint8_t uyari[]=" ilkdata: ";uint8_t uyari2[]=" ikincidata: ";
	cyclecounter=0;
while(cyclecounter<L_ARRAY){	
	GPIOB->BSRR=(1<<6);
			nop();
			nop();
	GPIOB->BSRR=(1<<22);
			nop();
			nop();
	cyclecounter++;
	if(c==0){
			value=getadc1();
			data12[(cyclecounter)]=(value);
					}	
		else if(c==1){
			value=getadc2();
			data21[(cyclecounter)]=value;
						}	
													} // end of while loop

	// one more clock is required												
	GPIOB->BSRR=(1<<6);
			nop();					
			nop();
	GPIOB->BSRR=(1<<22);
			nop();
			nop();
	// new conversion
	GPIOB->BSRR=(1<<7);
			nop();
			nop();
	GPIOB->BSRR=(1<<6);
		  nop();
			nop();
	GPIOB->BSRR=(1<<23);
			nop();	
			nop();
	GPIOB->BSRR=(1<<22);
			nop();
			nop();
	i=0;
	if(c==1){
			c=0;
			while(i<128){
					i++;
					sprintf(buf2,"\n\r%d",data21[i]);
					p=buf2;
					CDC_Transmit_FS(p,sizeof(buf2));
									} //end of while
					 } //end of if
	else if(c==0){
			c=1;
			while(i<128){
					i++;
					sprintf(buf2,"\n\r%d",data12[i]);
					p=buf2;
					CDC_Transmit_FS(p,sizeof(buf2));
		//		HAL_Delay(1);
										}//end of while
								} // end of else-if							
}



#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line){}

#endif
