

//////////////Bismillahir Rahmanir Rahim/////////

/////////////////Work 19 November2019/////////////////////

/////////////////////UART//////////////////////////
#define F_CPU 8000000UL
///////////////////////////////////////////////////

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

//#include "lcd.h"
#define RUN 1
#define STOP 0
char udat = 0;
unsigned char uchar[3];
#define END_OF_COMMAND_CHAR	13
#define MAX_DATA_INDEX 36
#define IGNORED 27
char DataContent[MAX_DATA_INDEX];
unsigned char LoopState = RUN;
unsigned char DataIndex = 0;
unsigned char RecByteCount = 0;
char StopString[MAX_DATA_INDEX];
void ResetSensorDelayCounter(unsigned char *chsid);
unsigned char clear_buffer_count = 0;
unsigned char data_in[35];
uint8_t data_count=0;
uint8_t spH=0,spL=0,AC_DLY=0,ACLOW=0,EXTD=0,FANL=0,FJAM_T=0,FANRESET=0,FANRESETCNTR=0;
uint16_t BUZZER=0,BUZZERCNTR=0,BUZZER2=0,BUZZERCNTR2=0,RESET_CNTR1=0;
uint16_t FJMCNTR1=0,FJMCNTR2=0;

int test_sensor2=0,temperature1=0,humidity1=0,RPMWORKS=0, ReconnectVolt = 50;
uint16_t FHUM;
uint8_t FIRE_ALARM=0;
uint8_t fjamcntr1=0,fjamcntr2=0;
uint8_t extcntr1=0,extcntr2=0;
uint8_t SP_DIFF=0;

#include "INIT_R_E_T.h"
#include "rtc_eeprom.h"
#include "uart_func.h"
#include "datalogger.h"
#include "DHT11_2.h"

///////////////////////////////////////////
//////////////////////////////Time func
void time_func()
{

ss++;
if(ss>=60)
	{
	mm++;
	ss=0;
	if(mm>=60)
			{
			hh++;
			mm=0;
			}
	}
}
void time_func_fan()
{

	ssf++;
	if(ssf>=60)
	{
		mmf++;
		ssf=0;
		if(mmf>=60)
		{
			hhf++;
			mmf=0;
		}
	}
}

void time_func_ac()
{

	ssa++;
	if(ssa>=60)
	{
		mma++;
		ssa=0;
		if(mma>=60)
		{
			hha++;
			mma=0;
		}
	}
}

/////////////////////////////Time func end



ISR(INT7_vect)
{
	Pulse++;
}

void init_timer()
{
	    TIMSK |= (1 << TOIE0);
	    TCCR0 |= (1 << CS01) | (1 << CS00);
}

ISR (TIMER0_OVF_vect) // timer0 overflow interrupt
{
	
	T2TickCount++;
	
	if(T2TickCount<500){PORTF |=(1<<PINF1);}else{PORTF &=(~(1<<PINF1));}
	
	if(data_count>32)
	{
		data_count=0;
		sprintf(data_in,"");
	}
	
	if(data_count>0)
	{
		DATATIMECOUNT++;
		if(DATATIMECOUNT > 100)
		{
			DATATIMECOUNT=0;
			data_count=0;
			sprintf(data_in,"");
		}
	}
		
	if(BUZZER2==1)
	{
		BUZZERCNTR2++;
		if(BUZZERCNTR2 > 3000)
		{
			BUZZERCNTR2=0;
		}
		PORTA |=(1<<PINA1);
		if(BUZZERCNTR2 > 100)
		{
			PORTA &=(~(1<<PINA1));
		}
	}
	
	
	if(T2TickCount > 976)
	{
		if(ONBIT==1){ON_CNTR++;}
		
		if((FANMODE==1)|(FANREST==1))
		{time_func_fan();Speed1=Pulse;Pulse=0;FFLT++;if(FFLT>60){FFLT=60;FC=1;}}
		if((ACMODE==1)|(ACREST==1)|(ACDELAY==1)){if(EXDBIT==1){EXD++;}else{if(ACDELAY==0){time_func_ac();}else{ssaa++;}}	}
		time_func();
		T2TickCount = 0;
		if(FANFAULTY==1){blink_FF++;if(blink_FF>4){blink_FF=0;}}
		//if(ACFAULTY==1){blink_AC++;if(blink_AC>10){blink_AC=0;}}
		
		//if(FJAM==1){FJAMCNTR++;if(FJAMCNTR>60){FJAMCNTR=65;}}
		
		RESET_CNTR1++; if(RESET_CNTR1==120){lcdInit();RESET_CNTR1=0;}
			
		if(FANRESET==1){FANRESETCNTR++;if(FANRESETCNTR>90){PORTD &=(~(1<<PIND3));FANRESET=0;FANRESETCNTR=0;}else{PORTD |=(1<<PIND3);}}
		//HuT++; if(HuT>3){HuT=0;humidity_func();}
			
			FHUM++;if(FHUM>350){FHUM=0;}
	}
}

void disp_time_fan()
	{
disp_int(hhf);lcdPrintData(":",1);disp_int(mmf);lcdPrintData(":",1);disp_int(ssf);
	}
void disp_time_ac()
	{
		disp_int(hha);lcdPrintData(":",1);disp_int(mma);lcdPrintData(":",1);disp_int(ssa);
	}

void external_interrupt_on()
{

	//DDRD &= ~(1<<PD2);
	DDRE &=(~(1<<PINE7));
	PORTE |=(PORTE|0b10000000);
	//MCUCR |=(0<<ISC00)|(1<<ISC01);
	//GICR |=(1<<INT0);
	EICRB |= (1 << ISC71)|(0 << ISC70);
	EIMSK |= (1 << INT7);

}


void external_interrupt_off()
{

	EICRB &=~(1<<ISC70);
	EICRB &=~(1<<ISC71);

	EIMSK &=~(1<<INT7);

}


void special1()
{

	char cgram[]=
	{
		0x0A, 0x0A, 0x1F, 0x1F, 0x0E, 0x06, 0x0C, 0x18,
		0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F, //Char0
		0x0E, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F, //Char1
		0x0E, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, //Char2
		0x0E, 0x11, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, //Char3
		0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, //Char4
		0x0A, 0x0A, 0x1F, 0x1F, 0x0E, 0x06, 0x0C, 0x18, //Char5
	};
		
	lcdControlWrite(0x40+0);     //Generating 'b' at starting address of CG-RAM
	int i=0;
	
	while(i!=56)                     //Generate 'b' By sending 'b' PAttern
	{
		lcdDataWrite(cgram[i]);
		i++;
		wdt_reset();
		wdt_enable(WDTO_2S);
	}
}

void ButtonCheck_Secondary()
{
	uint16_t KEY0=0,KEY1=0,KEY2=0,KEY3=0,ADD_VALUE=0;
	KEY0=(PINC & 0x01);
	KEY1=(PINC & 0x02);
	KEY2=(PINC & 0x08);
	KEY3=(PINC & 0x20);
	
	ADD_VALUE=KEY0+KEY1+KEY2+KEY3;
	//lcdGotoXY(0,1);
	//disp_int(ADD_VALUE);
	//disp_delay();
	//disp_delay();

	if(ADD_VALUE==3)
	{
		//save_record_no(0);eeprom_write_byte((uint8_t*)(152),0);
		//_delay_ms(1000);
		//lcdClear();lcdGotoXY(0,0);
		//lcdPrintData("Data Erase Success",18);
		//disp_delay();
		//disp_delay();
		
		FANLED_OUT_HIGH();_delay_ms(500);FANLED_OUT_LOW();
		wdt_reset();
		wdt_enable(WDTO_2S);
		FANF_OUT_HIGH();_delay_ms(500);FANF_OUT_LOW();
		
		wdt_reset();
		wdt_enable(WDTO_2S);
		SMOKE_OUT_HIGH();_delay_ms(500);SMOKE_OUT_LOW();
		
		//FANLED_OUT_HIGH();_delay_ms(500);FANLED_OUT_LOW();
		
		wdt_reset();
		wdt_enable(WDTO_2S);
		HIGHT_OUT_HIGH();_delay_ms(500);HIGHT_OUT_LOW();
		wdt_reset();
		wdt_enable(WDTO_2S);
		
		FJAM_OUT_HIGH();_delay_ms(500);FJAM_OUT_LOW();
		wdt_reset();
		wdt_enable(WDTO_2S);
		ALLF_OUT_HIGH();_delay_ms(500);ALLF_OUT_LOW();
		wdt_reset();
		wdt_enable(WDTO_2S);
		//ALLF_OUT_HIGH();_delay_ms(500);ALLF_OUT_LOW();
		//wdt_reset();
		//wdt_enable(WDTO_2S);
		
		PORTA |=(1<<PINA3);
		_delay_ms(100);
		PORTA &=(~(1<<PINA3));
		wdt_reset();
		wdt_enable(WDTO_2S);
		
		
		
		//FAN_DIR_OUT();FANF_OUT_LOW();
		//FANF_DIR_OUT();FANF_OUT_LOW();
		//DISP_DIR_OUT();DISP_OUT_HIGH();
		//FJAM_DIR_OUT();FJAM_OUT_LOW();
		//FANLED_DIR_OUT();FANLED_OUT_LOW();
		//CONTF_DIR_OUT();CONTF_OUT_HIGH();
		//SMOKE_DIR_OUT();SMOKE_OUT_LOW();
	//	HIGHT_DIR_OUT();HIGHT_OUT_LOW();
	//	ALLF_DIR_OUT();ALLF_OUT_LOW();
		
	}
}


void init_devices(void)
{

//unsigned char tmr2ctrl = 0;
DDRE &=(~(1 << PINE2))|(~(1 << PINE4));
DDRF &=(~(1 << PINF4))|(~(1 << PINF3));
DDRF |=(0<<PINF4)|(0<<PINF3);

PORTE |=(1 << PINE2)|(1 << PINE4);

DDRD |=(1<<PIND3);

DDRF &=(~(1 << PINF5))|(~(1 << PINF7));

DDRE|=(1<<PINE6)|(1<<PINE5);
DDRF|=(1<<PINF6)|(1<<PINF1);

DDRB|=(1<<PINB0)|(1<<PINB2)|(1<<PINB4)|(1<<PINB6)|(1<<PINB7);
DDRC &=(~(1 << PINC0))|(~(1 << PINC1))|(~(1 << PINC3))|(~(1 << PINC5));
PORTC|=(1 << PINC0)|(1 << PINC1)|(1 << PINC3)|(1 << PINC5);
PORTE |=((1<<PINE5));
DDRA |=(1<<PINA3);
//PORTA &=(~(1<<PINA1));
PORTA |=(0<<PINA3);

//DDRA=0xFF;

DDRG |=(1<<PING1);

DDRD |=(1<<PIND3);
PORTD &=(~(1<<PIND3));

PORTG |=(1<<PING1);

 

FAN_DIR_OUT();FANF_OUT_LOW();  
FANF_DIR_OUT();FANF_OUT_LOW();
DISP_DIR_OUT();DISP_OUT_HIGH();
FJAM_DIR_OUT();FJAM_OUT_LOW();
FANLED_DIR_OUT();FANLED_OUT_LOW();
CONTF_DIR_OUT();CONTF_OUT_HIGH();
SMOKE_DIR_OUT();SMOKE_OUT_LOW();
HIGHT_DIR_OUT();HIGHT_OUT_LOW();
ALLF_DIR_OUT();ALLF_OUT_LOW();


_delay_ms(1000);

lcdInit();
adc_init();
lcdClear();

lcdGotoXY(0,0);   
lcdPrintData("Bismillahir         ",20);
lcdGotoXY(0,1);  
lcdPrintData("Rahmanir Rahim      ",20);
lcdGotoXY(0,2);   
lcdPrintData("Banglalink FCU_V5.07",20);
lcdGotoXY(0,3);   
lcdPrintData("HS ENGINEERING LTD. ",20);


//lcdPrintData("  ZASS TELECOM LTD. ",20);

disp_delay();
lcdClear();

//////////////////////////////////////////////////////////////////////////
// Address EEprom (aE)
// EEPROM addresses loaded to Multidimensional Array
aE[0][0]=1;
aE[1][0]=2;
aE[2][0]=3;

aE[3][0]=5;
aE[3][1]=6;
aE[3][2]=7;
aE[4][0]=8;
aE[4][1]=9;
aE[4][2]=10;
aE[5][0]=51;

aE[6][0]=11;
aE[7][0]=12;
aE[8][0]=13;

aE[9][0]=14;
aE[9][1]=15;
aE[9][2]=16;
aE[10][0]=17;
aE[10][1]=18;
aE[10][2]=19;

aE[11][0]=20;
aE[12][0]=21;
aE[13][0]=22;


aE[14][0]=23;
aE[15][0]=24;
aE[16][0]=25;
aE[17][0]=26;
aE[18][0]=27;
aE[22][0]=28;
aE[20][0]=29;

init_timer();

external_interrupt_on();

USART_Init(51);

sei();
special1();

PORTA |=(0<<PINA3);

}

void take_all_setting()
{
	
	ROOMHIGH=eeprom_read_byte((uint8_t*)aE[0][0]);
	SP_DIFF=eeprom_read_byte((uint8_t*)aE[1][0]);
	FANL=eeprom_read_byte((uint8_t*)aE[2][0]);
	FANH=eeprom_read_byte((uint8_t *)aE[3][0]);
	FANM=eeprom_read_byte((uint8_t *)aE[3][1]);
	FANS=eeprom_read_byte((uint8_t *)aE[3][2]);
	FANHR=eeprom_read_byte((uint8_t *)aE[4][0]);
	FANMR=eeprom_read_byte((uint8_t *)aE[4][1]);
	FANSR=eeprom_read_byte((uint8_t *)aE[4][2]);

	TEX=eeprom_read_byte((uint8_t*)aE[5][0]);

	LOWV_SET=eeprom_read_word(300);
	
	RECONNECT_VOLT_SET = eeprom_read_word(220);
	
	INTERV=eeprom_read_byte((uint8_t *)aE[7][0]);
	RPMWORKS=eeprom_read_byte((uint8_t *)aE[11][0]);
}


void chk_format()
{
	if(FANL>ROOMHIGH){ROOMHIGH=FANL+2;eeprom_write_byte((uint8_t *)aE[0][0],ROOMHIGH);}
		
	//////if(SP_DIFF>50){SP_DIFF=ROOMHIGH+5;eeprom_write_byte((uint8_t *)aE[1][0],SP_DIFF);}
	if(SP_DIFF>50){SP_DIFF=ROOMHIGH;eeprom_write_byte((uint8_t *)aE[1][0],SP_DIFF);}

	if(ROOMHIGH>100){ROOMHIGH=30;eeprom_write_byte((uint8_t *)aE[0][0],ROOMHIGH);}
	if(FANL>100){FANL=25;eeprom_write_byte((uint8_t *)aE[1][0],FANL);}

		
	if((FANH>100)|(FANM>100)|(FANS>100)){FANH=2;eeprom_write_byte((uint8_t *)aE[2][0],FANH);
	FANM=0;eeprom_write_byte((uint8_t *)aE[2][1],FANM);
	FANS=15;eeprom_write_byte((uint8_t *)aE[2][2],FANS);}
		
	if((FANHR>100)|(FANMR>100)|(FANSR>100)){FANHR=0;eeprom_write_byte((uint8_t *)aE[3][0],FANHR);
	FANMR=15;eeprom_write_byte((uint8_t *)aE[3][1],FANMR);
	FANSR=15;eeprom_write_byte((uint8_t *)aE[3][2],FANSR);
	}
	
	if(TEX>100){TEX=42;eeprom_write_byte((uint8_t *)aE[4][0],TEX);}
	if(INTERV>55){INTERV=15;eeprom_write_byte((uint8_t *)aE[6][0],INTERV);}
	if(RPMWORKS>2){RPMWORKS=1;eeprom_write_byte((uint8_t *)aE[10][0],RPMWORKS);}
}


void init_internaleeprom()
{
	eeprom_write_byte((uint8_t *)aE[0][0],30);
	eeprom_write_byte((uint8_t *)aE[1][0],35);
	eeprom_write_byte((uint8_t *)aE[2][0],28);
	
	eeprom_write_byte((uint8_t *)aE[3][0],2);eeprom_write_byte((uint8_t *)aE[3][1],0); eeprom_write_byte((uint8_t *)aE[3][2],00);
	eeprom_write_byte((uint8_t *)aE[4][0],00);eeprom_write_byte((uint8_t *)aE[4][1],15); eeprom_write_byte((uint8_t *)aE[4][2],00);
	
	eeprom_write_byte((uint8_t *)aE[5][0],42);
	eeprom_write_byte((uint8_t *)aE[7][0],15);
	
	
	eeprom_write_byte((uint8_t *)aE[11][0],1);
	
	
	eeprom_write_word(300,480);
	eeprom_write_word(220,490);
	
}

void init_internaleeprom2()
{
	eeprom_write_byte((uint8_t *)aE[0][0],30);
	eeprom_write_byte((uint8_t *)aE[1][0],35);
	eeprom_write_byte((uint8_t *)aE[2][0],28);
	
	eeprom_write_byte((uint8_t *)aE[3][0],0);eeprom_write_byte((uint8_t *)aE[3][1],2); eeprom_write_byte((uint8_t *)aE[3][2],00);
	eeprom_write_byte((uint8_t *)aE[4][0],00);eeprom_write_byte((uint8_t *)aE[4][1],1); eeprom_write_byte((uint8_t *)aE[4][2],00);
	
	eeprom_write_byte((uint8_t *)aE[5][0],42);
	eeprom_write_byte((uint8_t *)aE[7][0],2);
	
	
	eeprom_write_byte((uint8_t *)aE[11][0],1);
	
	
	eeprom_write_word(300,400);
	eeprom_write_word(220,420);
	
}

/////////////////////main function start//////////////////////
//////////////////////////////////////////////////////////////

void common_alarm()
{
	if((FJAM==1)|(FLVD==1)|(EXTR_HIGH==1)|(FIRE_ALARM==1)|(FANFAULTY==1))
	{
		ALLF_OUT_HIGH();
	}
	else
	{
		ALLF_OUT_LOW();
	}
}

void fjam_alarm_check()
{		
	if((PINF & 0x08)>0)
	{
		fjamcntr1++;
		if(fjamcntr1>50)
		{
			FJAM=1;fjamcntr2=0;fjamcntr1=120;
			////FJAM_OUT_HIGH();
			FJAM_OUT_LOW();
		}
	}
	else{
		fjamcntr2++;
		if(fjamcntr2>20)
		{
			FJAM=0;fjamcntr1=0;fjamcntr2=120;
			////FJAM_OUT_LOW();
			FJAM_OUT_HIGH();
		}
	}
}

void operation_alarm_fan()
{
	if(RPMWORKS==1){if((FANMODE==1)&&(FANREST==0)&&(FC==1)){if(RPM<100){FANF_OUT_HIGH();FANFAULTY=1;}else{FANF_OUT_LOW();FANFAULTY=0;}}}
		else{FANFAULTY=0;}	
}

////////////////////Operation Teletalk Logic//////
void check_reset_fan()
{
	if(((mmf%5)==0)&&(mmf!=0)&&(ssf==0)&&(RPMWORKS==1)){
		if(RPM<100){FANRESET=1;}
		;
	}
}

//////////////////////////////////////////////////////////////////////////
// MOD
// Tune / Adjust PWM upon temperature Feedback
void Pwm_Step()
{
	
	uint16_t SPEED2, TEMP_DIFF=0;
	uint16_t pod_factor = 0;
	
	TEMP_DIFF=TROOM-ROOMHIGH;
	
	SPEED2=150+(TEMP_DIFF*100/(SP_DIFF-ROOMHIGH));
	//pod_factor
	////SPEED2=150+((TROOM * 10) / 5);
	
	/*
	if(TROOM>=(ROOMHIGH+SP_DIFF)){SPEED2=255;}
	else if(TROOM>(ROOMHIGH+4)){SPEED2=225;}
		else if(TROOM>(ROOMHIGH+3)){SPEED2=200;}
			else if(TROOM>(ROOMHIGH+2)){SPEED2=175;}
				else if(TROOM>(ROOMHIGH+1)){SPEED2=150;}
					else {SPEED2=150;}
	*/
	//lcdGotoXY(0,3);disp_int(SPEED2);
	
	if(SPEED2<70){SPEED2=150;}
	if(SPEED2>255){SPEED2=255;}
	
	OCR2=SPEED2;

	if((mmf>=2)|(hh>0)){operation_alarm_fan();}
	
	if(((mmf>=2)|(hh>0))&&(SPEED2>230)){fjam_alarm_check();}
	
}


int FAN_timeover()
{
	if(ssf>=FANS)
	{
		if(mmf>=FANM)
		{
			if(hhf>=FANH)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return 0;
		}
	}else
	{
		return 0;
	}
}

int FAN_timeover_rest()
{

	if(ssf>FANSR)
	{
		if(mmf>=FANMR)
		{
			if(hhf>=FANHR)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			return 0;
		}
	}else
	{
		return 0;
	}
}


void fan_on()
{
	pwm_init();
	mmf=0;ssf=0;hhf=0;
	FANMODE=1;
	FFLT=0;
	FANLED_OUT_HIGH();
}


void fan_off()
{
	FAN_OUT_LOW();
	FANLED_OUT_LOW();
	mmf=0;ssf=0;hhf=0;
	FANREST=1;
	pwm_disruct();
	FAN_OUT_LOW();
	FC=0;
	Speed1=0;fjamcntr1=0;fjamcntr2=0;
	FJAMCNTR=0;
}

void fan_off_all()
{
	FANMODE=0;FC=0;
	FJAMCNTR=0;
	FAN_OUT_LOW();
	fan_off();FANREST=0;
	fjamcntr1=0;fjamcntr2=0;
	mmf=0;ssf=0;hhf=0;
	ONBIT=0;ON_CNTR=0;
}

void Customized_fan_run()
{
	
	if(TROOM<=FANL){
		fan_off();mmf=0;ssf=0;hhf=0;
		FANREST=0;ACDELAY=0;FC=0;FANMODE=0;
		FANFAULTY=0;
	}
	else
	{
		if(FAN_timeover()==1){fan_off();}
		else{
			Pwm_Step();
		}
	}
}


void fan_func_Robi()
{
	if(TROOM>=TOUT)
	{
		F_OF_CNTR=0;
		COMPOWER=0;
		Customized_fan_run();
	}
	else if(TROOM<TOUT)
	{
		F_OF_CNTR++;
		if(F_OF_CNTR>150)
		{
			FANMODE=0;FC=0;
			ACMODE=0;FJAMCNTR=0;fjamcntr1=0;fjamcntr2=0;
			FAN_OUT_LOW();
			fan_off();FANREST=0;
			mmf=0;ssf=0;hhf=0;
			F_OF_CNTR=0;
			if(F_OF_CNTR>200){F_OF_CNTR=106;}
		}
	}/////////////////Troom<Tout Fan off mode
}



void Operation_Logic_Check_init()
{	
	if(TROOM>ROOMHIGH){
		if((TROOM>TOUT)&&(FLVD==0))
		{
			//ACDELAY=0;ssaa=0;
			ONBIT=1;
			ACDELAY=0;ssaa=0;mma=0;ssa=0;hha=0;
			if(ON_CNTR>15)
			{
				OF_CNTR=0;
				ONBIT=0;
				fan_on();
				if(ON_CNTR>250)
				{
					ON_CNTR=206;
				}
			}
		}//////////Troom>Tout
		else if(TROOM<TOUT)
		{
			OF_CNTR++;
			ONBIT=0;
			if(OF_CNTR>20)
			{
				ON_CNTR=0;
				ACMODE=0;
				FANMODE=0;FC=0;
				if(OF_CNTR>250)
				{
					OF_CNTR=206;
				}
			}	
		}else{
		ONBIT=0;ACDELAY=0;ssaa=0;
		ACDELAY=0;ssaa=0;mma=0;ssa=0;hha=0;
		ONBIT=0;ON_CNTR=0;ACFAULTY=0;FANFAULTY=0;EXD=0;EXDBIT=0;EXTR_HIGH=0;
		FJAM_OUT_LOW();FANF_OUT_LOW();
	}
	
		}
	
}

void IVS_Robi_PARAM_Auto()
{
	if(FANMODE==0)
	{
		Operation_Logic_Check_init();
		F_OF_CNTR=0;
	}
	else if(FANMODE==1)
	{
		if((FLVD==1)){fan_off_all();FANMODE=0;FANREST=0;FC=0;ON_CNTR=0;mmf=0;ssf=0;hhf=0;}
		else{
			if(FANREST==0){check_reset_fan();fan_func_Robi();}
				else{if(FAN_timeover_rest()){FANMODE=0;FANREST=0;FC=0;ON_CNTR=0;mmf=0;ssf=0;hhf=0;FJAMCNTR=0;fjamcntr1=0;fjamcntr2=0;}}
		}
	}
	
}


////////////////////Operation Ends////////////////

void temp_show(){

	lcdPrintData("RomT:",5);disp_int(TROOM);lcdDataWrite(223);lcdPrintData("C ",2);
	lcdPrintData(" TouT:",6);disp_int(TOUT);lcdDataWrite(223);lcdPrintData("C",1);
}

void low_line(){
	uint16_t BTV=0;
	
	BTV=BTVOLT*10;

	//lcdPrintData("DC:",3);
	if(BTV<LOWV_SET)
	{
		lcdDataWrite(1);
	}
	else if(BTV < (LOWV_SET+20))
	{
		lcdDataWrite(2)
		;
	}
	else if(BTV < (LOWV_SET+40))
	{
		lcdDataWrite(3);
	}
	else
	{
		lcdDataWrite(4);
	}
		
	if(BTV<LOWV_SET){
		blink_FLV++;if(blink_FLV>5){blink_FLV=0;}
		if(blink_FLV<3){lcdPrintData("    ",4);lcdPrintData(" Volt ",6);}else{disp_float2(BTV);lcdPrintData(" Volt ",6);}
		lcdPrintData(" Low volt  ",11);
	}
	else
	{
		disp_float2(BTV);lcdPrintData(" Volt ",6);
		
		if(RPMWORKS==1){lcdPrintData(" RPM:",5);
		if(RPM>10){if(RPM>100){if(RPM>1000){;}else{lcdPrintData("0",1);}}else{lcdPrintData("00",2);}}else{lcdPrintData("00",2);}
		disp_int(RPM);
		}
		else{lcdPrintData("         ",9);}	
	}
}

void display_Main_auto()
{
	
	lcdGotoXY(0,0);
	lcdPrintData("HS ENGINEERING LTD. ",20);
	//lcdPrintData("  ZASS TELECOM LTD. ",20);
	lcdGotoXY(0,1);
	temp_show();


	lcdGotoXY(0,2);
	lcdPrintData("FAN :",5);
	if(FANFAULTY==0){
	if(FANMODE==0)
			{
				if(ONBIT==1){lcdPrintData("Delay  ",7);}else{lcdPrintData("Off    ",7);}
			}
	else if(FANMODE==1)
			{
				if(FANREST==1){lcdPrintData("REST   ",7);}else{lcdPrintData("On     ",7);}
			}
	}else{if(blink_FF<3){lcdPrintData("FAULT  ",7);}else{
		if(FANMODE==0){lcdPrintData("Off    ",7);}
		else if(FANMODE==1){if(FANREST==1){lcdPrintData("REST   ",7);}else{lcdPrintData("On     ",7);}}
		}
	}	
		
	if(ONBIT==0){disp_time_fan();}else{disp_int(ON_CNTR);lcdPrintData("Sec   ",6);}

	lcdGotoXY(0,3);
	low_line();
	//disp_int(BUZZER2);disp_int(BUZZERCNTR);disp_int(fjamcntr1);disp_int(fjamcntr2);
	
}

void take_all_input()
	{
wdt_reset();
wdt_enable(WDTO_2S);
ADCSRA = ADCSRA | (1<<ADSC); 
_delay_ms(6);
start_convar();
ADCSRA = ADCSRA | (1<<ADSC);
//_delay_ms(6);
//start_convar1();
ADCSRA = ADCSRA | (1<<ADSC);
_delay_ms(6);
start_convar2();

DS18X20_start_meas();
_delay_ms(2);
DS18X20_read_meas_single();
DS18X20_start_meas2();
_delay_ms(2);
DS18X20_read_meas_single2();
	}

//////////////////////////////////////////////////////////////////////////
// MOD
// LOW VOLTAGE DETECTION CHECK. SET FLVD FLAG TO 1
void low_voltage_check()
{
	if(BTVOLT*10<LOWV_SET)
	{
		lvcntr1++;
		if(lvcntr1>100)
		{
			lvcntr1=105;FLVD=1;lvcntr2=0;
			if(BUZZER2==0){BUZZER2=1;}
		}
	}
	else
	{
		lvcntr2++;
		if(lvcntr2 > 900)
		{
			lvcntr2=905;
			FLVD=0;
			lvcntr1=0;
			if(BUZZER2==1)
			{
				BUZZER2=0;
			}
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
// MOD
// Read Menu Data or Parameter Data From EEPROM based on menu number indexing
void increment_MainItem()
{
	if(level==0)
	{
		SFTM_ITM++;
		if(SFTM_ITM>13)
		{
			SFTM_ITM=1;
		}
		
	    if(SFTM_ITM==10)
		{
			DATE_TIME_Conversion();
			VALUE2=HOURin;
			VALUE3=MINin;
			VALUE4=SECin;
		}
		else if(SFTM_ITM==11)
		{
			DATE_TIME_Conversion();
			VALUE2=DATEin;
			VALUE3=MONTHin;
			VALUE4=YEARin;
		}
		else
		{
			if((SFTM_ITM<=3)|(SFTM_ITM==6)|(SFTM_ITM==7)|(SFTM_ITM==8)|(SFTM_ITM==12)|(SFTM_ITM==13))
			{
				if(SFTM_ITM==7)
				{
					VALUE5=eeprom_read_word(300); //Read only for menu item 7
				}
				else if (SFTM_ITM==13)
				{
					VALUE5=eeprom_read_word(220);
				}
				//else if(SFTM_ITM==23){eeprom_read_byte((uint8_t *)aE[19][0]);}else
				{
					VALUE2=eeprom_read_byte((uint8_t *)aE[SFTM_ITM-1][0]);
				}
			}
			else
			{
				VALUE2=eeprom_read_byte((uint8_t *)aE[SFTM_ITM-1][0]);
				VALUE3=eeprom_read_byte((uint8_t *)aE[SFTM_ITM-1][1]);
				VALUE4=eeprom_read_byte((uint8_t *)aE[SFTM_ITM-1][2]);
			}
		}
	}
	else if(level==1)
	{
		VALUE2++;
		if((SFTM_ITM==12))
		{
			if(VALUE2>2)
			{
				VALUE2=1;
			}
		}
		
		if(VALUE2>1000)
		{
			VALUE2=0;
		}
		VALUE5++;
	}        
}

void decrement_MainItem()
{
if(level==0)
   {
   SFTM_ITM--;
   if(SFTM_ITM<1){SFTM_ITM=13;}
	   if(SFTM_ITM==10){DATE_TIME_Conversion();VALUE2=HOURin;VALUE3=MINin;VALUE4=SECin;}
		   else if(SFTM_ITM==11){DATE_TIME_Conversion();VALUE2=DATEin;VALUE3=MONTHin;VALUE4=YEARin;}
			   else{
   	if((SFTM_ITM<=3)|(SFTM_ITM==6)|(SFTM_ITM==7)|(SFTM_ITM==8)|(SFTM_ITM==12)|(SFTM_ITM==13))
		{
	   		if(SFTM_ITM==7)
			{
				VALUE5=eeprom_read_word(300);
			}
			
			if(SFTM_ITM==13)
			{
				VALUE5=eeprom_read_word(220);
			}
		//else if(SFTM_ITM==23){eeprom_read_byte((uint8_t *)aE[19][0]);}else
	   		{
		   	VALUE2=eeprom_read_byte((uint8_t *)aE[SFTM_ITM-1][0]);
	   		}

		}
   	else{
	   	VALUE2=eeprom_read_byte((uint8_t *)aE[SFTM_ITM-1][0]);
	   	VALUE3=eeprom_read_byte((uint8_t *)aE[SFTM_ITM-1][1]);
	   	VALUE4=eeprom_read_byte((uint8_t *)aE[SFTM_ITM-1][2]);
   	}
		}	   
   }
else if(level==1)
      {
	  VALUE2--;if(VALUE2>1000){VALUE2=0;}
		  if((SFTM_ITM==12)){if(VALUE2<1){VALUE2=2;}
				else{VALUE2=1;}}
	  VALUE5--;	  
      }
	}


void ButtonCheck()
{
  uint16_t KEY0=0,KEY1=0,KEY2=0,KEY3=0,ADD_VALUE=0;

	KEY0=(PINC & 0x01);
	KEY1=(PINC & 0x02);
	KEY2=(PINC & 0x08);
	KEY3=(PINC & 0x20);
ADD_VALUE=KEY0+KEY1+KEY2+KEY3;

if(ADD_VALUE==40)
{
	save_record_no(0);
	eeprom_write_byte((uint8_t*)(152),0);
	//_delay_ms(1000);
	lcdClear();
	lcdGotoXY(0,0);
	lcdPrintData("Data Erased",10);
	disp_delay();
	disp_delay();
}
	
	
LeisureBIT++;if(LeisureBIT>500){BUTTON=0;POK=2;SFTM_ITM=0;LeisureBIT=550;PORTG &= (~(1<<PING1));}else{PORTG|=(1<<PING1);}
	if(LeisureBIT>300){SEC_DISP=0;}
		
   if(ADD_VALUE<43)
      {	 
    KEY0=(PINC & 0x01);
    KEY1=(PINC & 0x02);
    KEY2=(PINC & 0x08);
    KEY3=(PINC & 0x20);
ADD_VALUE=KEY0+KEY1+KEY2+KEY3;


    
	   if(ADD_VALUE<43)
         {
			 //BUZZER=1;
			 PORTA |=(1<<PINA3);
			 _delay_ms(100);
			 PORTA &=(~(1<<PINA3));
			 
            if(ADD_VALUE==42)
            {
LeisureBIT=0;
///////////bounceprevent//////////////////
while(KEY0!=1)
{
KEY0=(PINC & 0x01);
wdt_reset();					/////////Watchdog Reset
wdt_enable(WDTO_2S);
}


if(BUTTON==0)
{ 
MAINMENU=1;
BUTTON=1;
level=0;
SFTM_ITM=1;
} 

if(POK==0){
	if((MAINMENU==1)&&(PASS==0)&&(BUTTON==2)){
		PASS=1;
	}
	else if((MAINMENU==1)&&(PASS==1)&&(BUTTON==3)){
		if(PVALUE==135){POK=1;}else{BUTTON=0;MAINMENU=0;POK=2;}
	}
	BUTTON++;
}
else if(POK==1){
 if(MAINMENU==1)
	{
if((BUTTON==1)&(level==0))
	{
	if(SFTM_ITM==1){VALUE2=eeprom_read_byte((uint8_t *)aE[SFTM_ITM-1][0]);}
	}
else if((BUTTON==2)&(level==0))
	{
		level=1;sub_level2=1;
	}
else if((BUTTON==3)&(level==1))
	{
	if((SFTM_ITM<=3)|(SFTM_ITM==6)|(SFTM_ITM==7)|(SFTM_ITM==8)|(SFTM_ITM==12)|(SFTM_ITM==13))
	{
		level=2;
	}
	else
	{
		if(sub_level2==1)
		{
			level=1;
			BUTTON=2;
			sub_level2=2;
			tmpsave1=VALUE2;
			VALUE2=VALUE3;
		}
		else if(sub_level2==2)
		{
			level=1;
			BUTTON=2;
			sub_level2=3;
			tmpsave2=VALUE2;
			VALUE2=VALUE4;
		}
		else
		{
			level=2;
			sub_level2=0;
		}
	}
	}

BUTTON++;
}

	}
/////////////////////////////////////////
            }
         else if(ADD_VALUE==41)
            {
LeisureBIT=0;
///////////bounceprevent//////////////////
if(BUTTON<3){
while(KEY1!=2)
{
KEY1=(PINC & 0x02);
wdt_reset();					/////////Watchdog Reset
wdt_enable(WDTO_2S);
}
}
if(BUTTON==0){SEC_DISP=1;}
if(BUTTON>=1){
 if(POK==1){if(MAINMENU==1){if(SFTM_ITM==9){if(level==1){level=2;}else{increment_MainItem();}}else{increment_MainItem();}}}
	 else if(PASS==1){PVALUE++;}
}////////////Button>=1
   			}////////////ADD_VALUE=208 End
         else if(ADD_VALUE==35)
            {
LeisureBIT=0;
		///////////bounceprevent//////////////////
		if(BUTTON < 3)
		{
			while(KEY2!=8)
			{
				KEY2=(PINC & 0x08);
				wdt_reset();					/////////Watchdog Reset
				wdt_enable(WDTO_2S);
			}
		}
/////////////////////////////////////////
if(BUTTON==0){SEC_DISP=0;}
if(BUTTON>=1){
if(POK==1){if(MAINMENU==1){if(SFTM_ITM==9){if(level==1){level=0;sub_level=0;BUTTON=1;}else{decrement_MainItem();}}else{decrement_MainItem();}}}
	  else if(PASS==1){PVALUE--;}
	  }//////////Button>=1
            }

          else if(ADD_VALUE==11)
            {
LeisureBIT=0;
///////////bounceprevent//////////////////

while(KEY3!=32)
{
KEY3=(PINC & 0x20);
wdt_reset();					/////////Watchdog Reset
wdt_enable(WDTO_2S);
}

/////////////////////////////////////////

if(BUTTON>0){
	
	POK=2;
	if(level==0){BUTTON=0;}
	else if(level==1)
		{
		level=0;sub_level=0;BUTTON=1;
		}
}
 
	
	        }

      }
   }

}///////////End button check

void menu_date_func()
{
if(level==0)
	{
	disp_int(VALUE2);lcdPrintData("/",1);disp_int(VALUE3);lcdPrintData("/20",3);disp_int(VALUE4);
	}
else if(level==1)
	{
		if(sub_level2==1)
			{
				if(VALUE2>31){VALUE2=31;}
			if(blink<5)
				{
				disp_int(VALUE2);lcdPrintData("/",1);disp_int(VALUE3);lcdPrintData("/20",3);disp_int(VALUE4);
				}
			else
				{
				lcdPrintData("  /",3);disp_int(VALUE3);lcdPrintData("/20",3);disp_int(VALUE4);
				}
			}
			else if(sub_level2==2)
			{
				if(VALUE2>12){VALUE2=12;}
				if(blink<5)
				{
					disp_int(tmpsave1);
					lcdPrintData("/",1);
					disp_int(VALUE2);
					lcdPrintData("/20",3);
					disp_int(VALUE4);
				}
				else
				{
					disp_int(tmpsave1);
					lcdPrintData("/  /20",6);
					disp_int(VALUE4);
				}
			}
		else if(sub_level2==3)
			{
				if(VALUE2>59){VALUE2=59;}
			if(blink<5)
				{
				disp_int(tmpsave1);lcdPrintData("/",1);disp_int(tmpsave2);lcdPrintData("/20",3);disp_int(VALUE2);
				}
			else
				{
				disp_int(tmpsave1);lcdPrintData("/",1);disp_int(tmpsave2);lcdPrintData("/    ",5);
				}
			}
	}
	lcdPrintData("    ",4);
}

void menu_time_func()
{
	if(level==0)
	{
		disp_int(VALUE2);lcdPrintData(":",1);disp_int(VALUE3);lcdPrintData(":",1);disp_int(VALUE4);
	}
	else if(level==1)
	{
		if(sub_level2==1)
		{
			if(VALUE2>23){VALUE2=23;}
			if(blink<5)
			{
				disp_int(VALUE2);lcdPrintData(":",1);disp_int(VALUE3);lcdPrintData(":",1);disp_int(VALUE4);
			}
			else
			{
				lcdPrintData("  :",3);disp_int(VALUE3);lcdPrintData(":",1);disp_int(VALUE4);
			}
		}
		else if(sub_level2==2)
		{
			if(VALUE2>59){VALUE2=59;}
			if(blink<5)
			{
				disp_int(tmpsave1);lcdPrintData(":",1);disp_int(VALUE2);lcdPrintData(":",1);disp_int(VALUE4);
			}
			else
			{
				disp_int(tmpsave1);lcdPrintData(":  :",4);disp_int(VALUE4);
			}
		}
		else if(sub_level2==3)
		{
			if(VALUE2>59){VALUE2=59;}
			if(blink<5)
			{
				disp_int(tmpsave1);lcdPrintData(":",1);disp_int(tmpsave2);lcdPrintData(":",1);disp_int(VALUE2);
			}
			else
			{
				disp_int(tmpsave1);lcdPrintData(":",1);disp_int(tmpsave2);lcdPrintData(":  ",3);
			}
		}
	}
	lcdPrintData("      ",6);
}

void LCD_MENU2(unsigned int position)
   {

lcdGotoXY(0,0);lcdPrintData( "Menu Item Selection  ",20);

//lcdGotoXY(0,1);
lcdGotoXY(0,1);
   switch(position)
      {
	  case 1:
	  lcdPrintData("Fan Start Temp>=",16);
      break;
	  case 2:
	  lcdPrintData("Max Speed Temp>=",16);
	  break;
	  case 3:
	  lcdPrintData("Fan Stop Temp <",15);
	  break;   
	  case 4:
	  lcdPrintData("Fan RunTime:",12);
	  break;
	  case 5:
	  lcdPrintData("FanRestTime:",12);
	  break;
	  case 6:
	  lcdPrintData("Extreme Temperature>",20);
	  break;
	  case 7:
	  lcdPrintData("Fan Disconnect volt<",20);
	  break;
	  case 8:
	  lcdPrintData("Data Interval=",14);
	  break;
	  case 9:
	  lcdPrintData("Download Data       ",20);
	  break;
	  case 10:
	  lcdPrintData("Time: ",6);
	  break;
	  case 11:
	  lcdPrintData("Date: ",6);
	  break;
	  case 12:
	  lcdPrintData("RPM Off: ",9);
	  break;
	  case 13:
	  lcdPrintData("Fan Reconnect volt<",20);
	  break;
      }

	if(level > 0)
	{
		blink++;
		if(blink > 10)
		{
			blink=0;
		}
	}
	else
	{
		blink=0;
	}
	
if(SFTM_ITM==9){
	lcdGotoXY(0,2);
	if(level==0){lcdPrintData("                    ",20);}
		else{
			if(level==1){lcdPrintData("Press UP=Yes Down=No",20);}
		}
}
else if(SFTM_ITM==11){menu_date_func();}
else if(SFTM_ITM==10){menu_time_func();}
else if(SFTM_ITM==8){
	if(level==0){disp_int(VALUE2);}
	else if(level=1){if(blink<5){lcdPrintData("  ",2);}else{disp_int(VALUE2);}}
	lcdPrintData("Min ",4);
}
else if(SFTM_ITM==6){
		lcdGotoXY(8,2);	
		if(level==0){disp_int(VALUE2);}
		else if(level=1){if(blink<5){lcdPrintData("  ",2);}else{disp_int(VALUE2);}}
		lcdDataWrite(223);lcdPrintData("C",1);
	}
else if(SFTM_ITM==2){
	//lcdGotoXY(8,2);
	if(level==0){disp_int(VALUE2);}
	else if(level=1){if(blink<5){lcdPrintData("  ",2);}else{disp_int(VALUE2);}}
	lcdDataWrite(223);
	lcdPrintData("C ",2);
}
else if(SFTM_ITM==7)
	{
		lcdGotoXY(8,2);
		if(level==0)
		{
			disp_float2(VALUE5);
		}
		else if(level==1)
		{
			if(blink < 5)
			{
				lcdPrintData("  . ",4);
			}
			else
			{
				disp_float2(VALUE5);
			}
		}
		lcdPrintData("Volt",4);	
	}

else if(SFTM_ITM==13)
{
	lcdGotoXY(8,2);
	if(level == 0)
	{
		disp_float2(VALUE5);
	}
	else if(level == 1)
	{
		if(blink < 5)
		{
			lcdPrintData("  . ",4);
		}
		else
		{
			disp_float2(VALUE5);
		}
	}
	lcdPrintData("Volt",4);
}

else if((SFTM_ITM==1)|(SFTM_ITM==3))
		{
		if(level==0){itoa(VALUE2,ch,10);lcdPrintData(ch,strlen(ch));}
	  	else if(level=1){if(blink<5){lcdPrintData("  ",2);}else{disp_int(VALUE2);}}
		lcdDataWrite(223);lcdPrintData("C ",2);
		}
else if(SFTM_ITM==12)
{
	if(level==0){if(VALUE2==2){lcdPrintData("[OFF]....  ",11);}else{lcdPrintData("[ON].....  ",11);}}
		else if(level==1){
			if(blink<5){lcdPrintData("[   ]      ",11);}
			else {if(VALUE2==2){lcdPrintData("[OFF]....  ",11);}else{lcdPrintData("[ON].....  ",11);}}
		}
}
else if((SFTM_ITM==4)|(SFTM_ITM==5))
		{
		if(level==0)
				{
				disp_int(VALUE2);lcdPrintData(":",1);disp_int(VALUE3);lcdPrintData(":",1);disp_int(VALUE4);
				}
		else if(level==1)
			{
			if(sub_level2==1)
			{
			if(blink<5)
				{
				disp_int(VALUE2);lcdPrintData(":",1);disp_int(VALUE3);lcdPrintData(":",1);disp_int(VALUE4);
				}
				else
				{
				lcdPrintData("  :",3);disp_int(VALUE3);lcdPrintData(":",1);disp_int(VALUE4);
				}
			}
		else if(sub_level2==2)
			{
			if(blink<5)
				{
				disp_int(tmpsave1);lcdPrintData(":",1);disp_int(VALUE2);lcdPrintData(":",1);disp_int(VALUE4);
				}
				else
				{
				disp_int(tmpsave1);lcdPrintData(":  :",4);disp_int(VALUE4);
				}	
			}
		 else if(sub_level2==3)
			{
			if(blink<5)
				{
					disp_int(tmpsave1);lcdPrintData(":",1);disp_int(tmpsave2);lcdPrintData(":",1);disp_int(VALUE2);
				}
				else
				{
					disp_int(tmpsave1);lcdPrintData(":",1);disp_int(tmpsave2);lcdPrintData(":  ",3);
				}
			}
		 }
		}

//////////////////////////////////////////////////////////////////////////
// MOD
// Flickering Issues Solved Here
if((SFTM_ITM==6)|(SFTM_ITM==7)|(SFTM_ITM==9)|(SFTM_ITM==13))
{
	//lcdGotoXY(0,2);
	//	lcdPrintData("                    ",20);
}
else{
	lcdGotoXY(0,2);
	lcdPrintData("                    ",20);
}

lcdGotoXY(0,3);
lcdPrintData("                    ",20);

   }



void updating_menu()
   {
   
 if(MAINMENU==1)
	{
lcdClear();   //clear the LCD
lcdGotoXY(0,0);
lcdPrintData( "Updating Menu.......",20);
//disp_delay(); 
if(SFTM_ITM==9){
	lcdPrintData("Show Data On PC     ",20);//disp_delay();
	SHOW_ROLING_EEPROM_EX_DATA();
	}else{ 
if((SFTM_ITM<=3)|(SFTM_ITM==6)|(SFTM_ITM==7)|(SFTM_ITM==8)|(SFTM_ITM==12)|(SFTM_ITM==13))
{
	if(SFTM_ITM==7)
	{
		eeprom_write_word(300,VALUE5);
	}
	else if (SFTM_ITM==13)
	{
		eeprom_write_word(220,VALUE5);
	}
	else{
	eeprom_write_byte((uint8_t *)aE[SFTM_ITM-1][0],VALUE2);
	}
}
else
{
	if(SFTM_ITM==10){convert_func_Time(tmpsave1,tmpsave2,VALUE2);update_time();}
		else if(SFTM_ITM==11){convert_func_Date(tmpsave1,tmpsave2,VALUE2);update_Date();}
	else{
		eeprom_write_byte((uint8_t *)aE[SFTM_ITM-1][0],tmpsave1);
		eeprom_write_byte((uint8_t *)aE[SFTM_ITM-1][1],tmpsave2);
		eeprom_write_byte((uint8_t *)aE[SFTM_ITM-1][2],VALUE2);
	}		
}
   
   
if((SFTM_ITM<9)|(SFTM_ITM==12)){
VALUE2=eeprom_read_byte(aE[SFTM_ITM-1][0]);
VALUE3=eeprom_read_byte(aE[SFTM_ITM-1][1]);
VALUE4=eeprom_read_byte(aE[SFTM_ITM-1][2]);
}else{
	VALUE4=VALUE2;
	VALUE2=tmpsave1;VALUE3=tmpsave2;
}

}
sub_level2=0;
BUTTON=2;tmpsave1=0;tmpsave2=0;

	}///////////Update Main menu

else	{
lcdGotoXY(0,0);
lcdPrintData( "Updating Menu ERROR",19);
}

lcdClear();   //clear the LCD
   }

void menu_disp()
{
lcdClear();   //clear the LCD
SFTM_ITM=1;
POK=0;
while(POK==0)
{
	wdt_reset();					/////////Watchdog Reset
	wdt_enable(WDTO_2S);
	ButtonCheck();
	lcdGotoXY(0,0);
	if(PASS==0){
	lcdPrintData( "Provide the Password ",20);
	}
	else{lcdPrintData( "Press Ok to Enter    ",20);}
	lcdGotoXY(0,1);
	lcdPrintData( "Password:[",10);
	if(PASS==1){if(blink<5){lcdPrintData( "   ",3);}else{disp_int(PVALUE);}}else{disp_int(PVALUE);}
	lcdPrintData( "]",1);
	blink++;
	if(blink>10){blink=0;}
	
}
if(POK==1){BUTTON=1;blink=0;VALUE2=eeprom_read_byte((uint8_t *)aE[0][0]);}else{BUTTON=0;}
	

while(BUTTON>=1)
      {

wdt_reset();					/////////Watchdog Reset
wdt_enable(WDTO_2S);

ButtonCheck();
	
 if(level==0)
      {
	  LCD_MENU2(SFTM_ITM);
      }
else if(level==1)
      {
     LCD_MENU2(SFTM_ITM);
      }
else if(level==2)
      {
	 updating_menu();
     level=0;
      }
else if(level==3)
      {
      /////////Out from the menu
      BUTTON=0;
      SFTM_ITM=0;
      level=0;
	  
	  MAINMENU=0;
      }

      }
lcdClear();   //clear the LCD
lcdGotoXY(0,0);
lcdPrintData( "***Menu END***",14);
disp_delay();
wdt_reset();					/////////Watchdog Reset
wdt_enable(WDTO_2S);
SFTM_ITM=0;BUTTON=0;POK=0;PASS=0;PVALUE=130;
lcdClear();   //clear the LCD

take_all_setting();

  }/////////menu disp end

//////////////////////////////////////////////////////////////
///////////////////////////////////button Menu End //////////////


void fire_alarm_check()
{
	
	if((PINF & 0x10)>0)
	{
		humcntr1++;
		if(humcntr1>100)
		{
			FIRE_ALARM=1;humcntr2=0;humcntr1=120;
			SMOKE_OUT_HIGH();
		}
	}
	else{
		humcntr2++;
		if(humcntr2>100)
		{
			FIRE_ALARM=0;humcntr1=0;humcntr2=120;
			SMOKE_OUT_LOW();
		}
	}
}


void secondary_display()
{
	
	DATE_TIME_Conversion();
	lcdGotoXY(0,0);
	lcdPrintData("Date:",5);
	//disp_int(BUZZERCNTR);
//	disp_int(BUZZER);
	
	
	//lcdGotoXY(8,0);
	disp_int(DATEin);
	lcdPrintData("/",1);disp_int(MONTHin);lcdPrintData("/20",3);disp_int(YEARin);lcdPrintData("     ",5);
	lcdGotoXY(0,1);
	lcdPrintData("Time:",5);
	//disp_int(BUZZERCNTR2);disp_int(BUZZER2);
	//lcdGotoXY(6,1);
	disp_int(HOURin);lcdPrintData(":",1);disp_int(MINin);lcdPrintData(":",1);disp_int(SECin);lcdPrintData("       ",7);
	lcdGotoXY(0,2);lcdPrintData("Save Interval:",14);disp_int(INTERV);  lcdPrintData("Min ",4);
	
	//lcdPrintData("  ",3);disp_int(RPMWORKS);
	
	lcdGotoXY(0,3);lcdPrintData("Saved Data:",11);
	
	if(SAVED_DATA>10){if(SAVED_DATA>100){lcdPrintData("0",1);}else{lcdPrintData("00",2);}}else{lcdPrintData("00",2);}
		disp_int(SAVED_DATA);lcdPrintData(" ",1);
		if(eeprom_read_byte((uint8_t*)(152))==1){
			lcdPrintData("Full",4);
		}else{lcdPrintData("    ",4);}
}

void fire_display()
{
	lcdGotoXY(0,0);lcdPrintData("                    ",20);
	lcdGotoXY(0,1);lcdPrintData("!!!!! CAUTON !!!!!  ",20);
	lcdGotoXY(0,2);lcdPrintData("Fire Alarm All OFF  ",20);
	lcdGotoXY(0,3);lcdPrintData("                    ",20);
}

void high_temp_alarm()
{

	if(TROOM>TEX)
	{
		extcntr1++;
		if(extcntr1>100)
		{
			extcntr1=120;
			extcntr2=0;
			EXTR_HIGH=1;
			HIGHT_OUT_HIGH();
		}
	}
	else{
		extcntr2++;
		if(extcntr2>100)
		{
			extcntr2=120;
			extcntr1=0;
			EXTR_HIGH=0;
			HIGHT_OUT_LOW();
		}
		
	}
}

void main()
{

uint8_t t=0,nsensor1=0,nsensor2=0;


init_devices();



nsensor1 = search_sensors();
nsensor2 = search_sensors2();

lcdGotoXY(0,0);
lcdPrintData("Sensor Check...     ",20);
ButtonCheck_Secondary();

PORTA |=(1<<PINA3);
_delay_ms(100);
PORTA &=(~(1<<PINA3));
wdt_reset();
wdt_enable(WDTO_2S);



for (t=0;t<50;t++)
   {
ADCSRA = ADCSRA | (1<<ADSC); 
start_convar();
_delay_ms(10);
ADCSRA = ADCSRA | (1<<ADSC); 
//start_convar1();
//_delay_ms(10);
//ADCSRA = ADCSRA | (1<<ADSC);
start_convar2();
_delay_ms(10);

DS18X20_start_meas();
//_delay_ms(10);
DS18X20_read_meas_single();

DS18X20_start_meas2();
//_delay_ms(10);
DS18X20_read_meas_single2();
wdt_reset();
wdt_enable(WDTO_2S);


   }



_delay_ms(700);

if((eeprom_read_word(aE[20][0]))!=1)
	{
//init_internaleeprom2();
init_internaleeprom();
eeprom_write_word(aE[20][0],1);
convert_func_Time(13,45,50);
update_time();
convert_func_Date(15,03,20);
update_Date();
save_record_no(0);
save_record_no_full(0);
	}


take_all_setting();


lcdClear();

USART_TransmitString("Bismnillahir Rahmanir Rahim \r\n");
USART_TransmitString("FCU BanglalinkV1.01 \r\n");
HuT=3;
PORTA |=(0<<PINA1);
lcdGotoXY(0,3);
//low_line();
disp_int(BUZZER2);disp_int(BUZZERCNTR);disp_int(fjamcntr1);disp_int(fjamcntr2);

while(1)
   {
wdt_reset();
wdt_enable(WDTO_2S);

//RS485_Check();
Save_EEPROM();
RPM=Speed1*20.48387;

//USART_TransmitString_Second("Test 2");

/*
lcdGotoXY(0,0);disp_int(DV_ID);lcdPrintData(" ",1);disp_int(data_count);lcdPrintData(" ",1);disp_int(sizeof(data_in));
lcdGotoXY(10,1);disp_int(t);

lcdGotoXY(0,2);lcdPrintData(data_in,sizeof(data_in));
lcdGotoXY(0,3);disp_int(TTT);

*/

chk_format();

	take_all_input(); 
	if(TROOM < 5)
	{
		TROOM=99;
	}
	if(TOUT < 5)
	{
		TOUT=0;
	}
	
	fire_alarm_check();
	if(FIRE_ALARM==0)
	{
		if(SEC_DISP==0)
		{
			display_Main_auto();
			}
			else
			{
				secondary_display();
			}
		}
		else
		{
			fire_display();
			fan_off_all();
		}

		IVS_Robi_PARAM_Auto();
		low_voltage_check();
		ButtonCheck();
		high_temp_alarm();
		common_alarm();

		if(BUTTON>0)
		{
			menu_disp();
		}
	}

}////////main End 
