

#include "onewire.h"
#include "ds18x20.h"
//#include "delay.h"
//#include "lcdlib.h"

#include "pwm.h"
#include "lcd2.h"
//#include "delay.h"
#include "lcdconf.h"

uint16_t tot_overflow,ck=0;


uint16_t TROOM=0,TOUT=0;
unsigned char ch[10];
uint8_t aE[25][3];
uint8_t address=0,Eval=9;
uint8_t EEP_CHK_BIT=1;
//uint8_tTEMPIN=10;TEMPOU=12;
/////////////////////For Menu Function////////
uint8_t SFTM_ITM=0,BUTTON=0,level=0,sub_level=0,MODE=0;
uint8_t VALUE2=0,blink=0,VALUE3=0,VALUE4=0,FANREST=0,ACREST=0,ACDELAY=0;
uint16_t tmp_Menu1=0,tmp_Menu2=0,tmp_Menu3=0,MAINMENU=0,LeisureBIT=0,sub_level2=0,Temp_VALUE2=0,ssaa=0;
uint8_t POK=0,PVALUE=130,PASS=0;
unsigned int TEMP_AVG2=0,TEMP_AVG1=0;
int HuT=0,HUM_DH=0;
//////////////////////////////////////////////
///////////////////For EEPROM   ////////////////
////////////////////////////////////////////////

///////////////////Teletalk Operation Variable//////
uint8_t ACMODE=0,FANMODE=0,COMPOWER=0,EXTR_HIGH=0,TAVGCN2=0,TAVGCN1=0;
uint16_t comPo_cntr1=0,comPo_cntr2=0;
uint16_t EXT_CNTR1=0,EXT_CNTR2=0;
uint16_t ACFAIL=0,ON_CNTR=0,OF_CNTR=0,F_OF_CNTR=0;
uint16_t AC_OF_CNTR1=0,AC_OF_CNTR2=0,ACFAULTY=0,VALUE5=0,FANFAULTY=0;
uint16_t PWM=0,RPM=0,ACDL=0;
////////////////////////////////////////////////////

float  floatvalB=0,floatvalAVG=0,BTVOLT=0;
uint16_t  FF_VAL=0,FF_AVG=0,FF_temp=0,ADAVGCNTR3,ADAVGCNTR2,PDBCHKCNTR2=0,ADAVGCNTR=0,PDB_temp=0,ADAVGCNTR4=0,PDB_AVG=0;
uint16_t  PDB=0,PDB_val_temp=0,BATTVAL=0,FF=0,cntF1=0,cntF2=0;
uint8_t sub_btn=0,cnTT1=0,cnTT2=0;
uint8_t cnttime,rnfst=0,hh=0,ss=0,mm=0,tmpsave1=0,tmpsave2=0;

uint8_t hhf=0,ssf=0,mmf=0,hha=0,ssa=0,mma=0;
///////////////////For Time/////////////////////
////////////////////////////////////////////////
/////////////////////////////////////For DS18B20/////////////////
#define MAXSENSORS 5
#define OW_GET_IN()   ( OW_IN & (1<<OW_PIN))
#define OW_OUT_LOW()  ( OW_OUT &= (~(1 << OW_PIN)) )
#define OW_OUT_HIGH() ( OW_OUT |= (1 << OW_PIN) )
#define OW_DIR_IN()   ( OW_DDR &= (~(1 << OW_PIN )) )
#define OW_DIR_OUT()  ( OW_DDR |= (1 << OW_PIN) )


#define OW2_GET_IN()   ( OW2_IN & (1<<OW2_PIN))
#define OW2_OUT_LOW()  ( OW2_OUT &= (~(1 << OW2_PIN)) )
#define OW2_OUT_HIGH() ( OW2_OUT |= (1 << OW2_PIN) )
#define OW2_DIR_IN()   ( OW2_DDR &= (~(1 << OW2_PIN )) )
#define OW2_DIR_OUT()  ( OW2_DDR |= (1 << OW2_PIN) )


uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
/////////////////////////////////////////////////////////////////
////////////////////////////////////////////////
uint16_t lvcntr2=0;
uint8_t SECDISP=0,SECDISPCNT=0,FC=0,FFLT=0,EXDBIT=0,blink_FF=0,blink_AC=0,lvcntr1=0,FLVD=0,blink_FLV=0,ONBIT=0,FJAM=0,blinkH=0;
uint8_t HUMIDITY=0,humcntr1=0,humcntr2=0;
uint16_t Speed=0,Pulse=0,Speed1=0,EXD=0,FJAMCNTR=0;;
uint16_t LOWV_SET=0;
uint16_t RECONNECT_VOLT_SET = 0;
uint8_t HUM_SET=0;
uint8_t ROOMHIGH=0,ACHIGH=0,TEX=0,ACON=0,ACST=0;
uint8_t FANH=0,FANM=0,FANS=0;
uint8_t FANHR=0,FANMR=0,FANSR=0;
uint8_t ACH=0,ACM=0,ACS=0;
uint8_t ACHRE=0,ACMRE=0,ACSRE=0,MinAC=0;
uint8_t TTT=0,DV_ID=0;
uint16_t TEST_CNTR_1=0,DATASTART=0,DATATIMECOUNT=0;

unsigned char Did[2];

uint8_t MAN=0,FANMAN=0,ACMAN=0,SEC_DISP=0,INTERV=0;
uint16_t SAVED_DATA=0;


float take_value();

unsigned int T2TickCount=0;

#define UNIC_IN   PINC
#define UNIC_OUT  PORTC
#define UNIC_DDR  DDRC



#define UNIA_IN   PINA
#define UNIA_OUT  PORTA
#define UNIA_DDR  DDRA


#define UNID_IN   PIND
#define UNID_OUT  PORTD
#define UNID_DDR  DDRD

#define UNIB_IN   PINB
#define UNIB_OUT  PORTB
#define UNIB_DDR  DDRB

#define UNIE_IN   PINE
#define UNIE_OUT  PORTE
#define UNIE_DDR  DDRE

#define UNIF_IN   PINF
#define UNIF_OUT  PORTF
#define UNIF_DDR  DDRF

#define FJAM_PIN PE6 /////////////Filter Jam
#define FANF_PIN PB5 /////////////Fan Fault  ok
#define FANLED_PIN PB6 //////////////////FAN ON LED  ok

#define SMOKE_PIN PB2 //////////////////FAN ON LED  ok
#define HIGHT_PIN PB0 //////////////////FAN ON LED  ok
#define FANLED_PIN PB6 //////////////////FAN ON LED  ok
#define CONTF_PIN PF2 //////////////////FAN ON LED  ok
#define ALLF_PIN PB4 //////////////////FAN ON LED  ok

#define FAN_PIN PB7 //////////////////FAN

#define DISP_PIN PG1 //////////////////DISP

////////////////////Operation//////////////////


#define FANLED_DIR_OUT()  ( UNIB_DDR |= (1 << FANLED_PIN) )
#define FANLED_OUT_HIGH() ( UNIB_OUT |= (1 << FANLED_PIN) )
#define FANLED_OUT_LOW()  ( UNIB_OUT &= (~(1 << FANLED_PIN)) )

#define FJAM_DIR_OUT()  ( UNIE_DDR |= (1 << FJAM_PIN) )
#define FJAM_OUT_HIGH() ( UNIE_OUT |= (1 << FJAM_PIN) )
#define FJAM_OUT_LOW()  ( UNIE_OUT &= (~(1 << FJAM_PIN)) )

#define FANF_DIR_OUT()  ( UNIB_DDR |= (1 << FANF_PIN) )
#define FANF_OUT_HIGH() ( UNIB_OUT |= (1 << FANF_PIN) )
#define FANF_OUT_LOW()  ( UNIB_OUT &= (~(1 << FANF_PIN)) )

#define FAN_DIR_OUT()  ( UNIB_DDR |= (1 << FAN_PIN) )
#define FAN_OUT_HIGH() ( UNIB_OUT |= (1 << FAN_PIN) )
#define FAN_OUT_LOW()  ( UNIB_OUT &= (~(1 << FAN_PIN)) )

#define DISP_DIR_OUT()  ( UNIA_DDR |= (1 << DISP_PIN) )
#define DISP_OUT_HIGH() ( UNIA_OUT |= (1 << DISP_PIN) )
#define DISP_OUT_LOW()  ( UNIA_OUT &= (~(1 << DISP_PIN)) )


#define ALLF_DIR_OUT()  ( UNIB_DDR |= (1 << ALLF_PIN) )
#define ALLF_OUT_HIGH() ( UNIB_OUT |= (1 << ALLF_PIN) )
#define ALLF_OUT_LOW()  ( UNIB_OUT &= (~(1 << ALLF_PIN)) )

#define SMOKE_DIR_OUT()  ( UNIB_DDR |= (1 << SMOKE_PIN) )
#define SMOKE_OUT_HIGH() ( UNIB_OUT |= (1 << SMOKE_PIN) )
#define SMOKE_OUT_LOW()  ( UNIB_OUT &= (~(1 << SMOKE_PIN)) )


#define HIGHT_DIR_OUT()  ( UNIB_DDR |= (1 << HIGHT_PIN) )
#define HIGHT_OUT_HIGH() ( UNIB_OUT |= (1 << HIGHT_PIN) )
#define HIGHT_OUT_LOW()  ( UNIB_OUT &= (~(1 << HIGHT_PIN)) )


#define CONTF_DIR_OUT()  ( UNIF_DDR |= (1 << CONTF_PIN) )
#define CONTF_OUT_HIGH() ( UNIF_OUT |= (1 << CONTF_PIN) )
#define CONTF_OUT_LOW()  ( UNIF_OUT &= (~(1 << CONTF_PIN)) )



uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	//	lcd_gotoXY(0,1);
	//	lcd_write("Bus scanning ...\n");
	//	/
	//	lcdGotoXY(0,1);
	//	lcdPrintData("Bus scanning ...    ",20);
	
	nSensors = 0;
	
	for( diff = OW_SEARCH_FIRST;
	diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; )
	{
		DS18X20_find_sensor( &diff );
		
		if( diff == OW_PRESENCE_ERR ) {
			
			//	lcdGotoXY(0,1);
			//		lcdPrintData("No sensor found 1   ",20);
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			
			//	lcdGotoXY(0,1);
			//	lcdPrintData("Bus error\n",20);
			break;
		}
		
		for (i=0;i<OW_ROMCODE_SIZE;i++)
		gSensorIDs[nSensors][i]=id[i];
		
		nSensors++;
	}
	
	//	disp_delay();
	return nSensors;
}


uint8_t search_sensors2(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	//	lcdGotoXY(0,1);
	//	lcdPrintData("Bus scanning ...    ",20);
	
	
	nSensors = 0;
	
	for( diff = OW_SEARCH_FIRST;
	diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; )
	{
		DS18X20_find_sensor2( &diff );
		
		if( diff == OW_PRESENCE_ERR ) {
			
			//	lcdGotoXY(0,1);
			//	lcdPrintData("No sensor found 2   ",20);
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			
			//	lcdGotoXY(0,1);
			//	lcdPrintData("Bus error\n",20);
			break;
		}
		
		for (i=0;i<OW_ROMCODE_SIZE;i++)
		gSensorIDs[nSensors][i]=id[i];
		
		nSensors++;
	}
	
	//	disp_delay();
	return nSensors;
}


void disp_delay()
{
	//////////////////////////////////////////
	wdt_reset();
	wdt_enable(WDTO_2S);
	/////////////////////////////////////////
	_delay_ms(1000);//_delay_ms(1000);
	//////////////////////////////////////////
	wdt_reset();
	wdt_enable(WDTO_2S);
	/////////////////////////////////////////
	_delay_ms(1000);//_delay_ms(1000);
	///////////////////////////////////////////
	wdt_reset();
	wdt_enable(WDTO_2S);
	/////////////////////////////////////////
	_delay_ms(1000);//_delay_ms(1000);
	//////////////////////////////////////////
	wdt_reset();
	wdt_enable(WDTO_2S);
	/////////////////////////////////////////
	_delay_ms(1000);//_delay_ms(1000);
	//printf(lcd_putc, "\f");
	lcdClear();
}


////////////////////////////////////////////ADC convarsion Start///
float take_value()
{
	int adc_value=0;
	adc_value = ADC;
	return (adc_value * 0.05865);
}
int take_value2()
{
	int adc_value=0;
	adc_value = ADC;
	return (adc_value);
}

void adc_init()
{
	ADMUX = (1<<REFS0) |(0<<REFS1) | (0<<ADLAR);
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |=  (1 << ADEN)|(1<<ADFR);
}
void start_convar()
{
	
	ADMUX=0x45;
	_delay_ms(10);
	floatvalB=take_value();
	//BATTVAL=take_value();
	if(ADAVGCNTR<10)
	{
		ADAVGCNTR++;
		floatvalAVG+=floatvalB;
	}
	else if(ADAVGCNTR==10)
	{
		ADAVGCNTR=0;
		BTVOLT=(floatvalAVG/10);
		floatvalAVG=0;
	}
}
/*
void start_convar1()
{
	
	ADMUX=0x43;
	_delay_ms(10);
	FF_temp=take_value2();
	
	if(ADAVGCNTR3<10)
	{
		ADAVGCNTR3++;
		FF_AVG+=FF_temp;
	}
	else if(ADAVGCNTR3==10)
	{
		ADAVGCNTR3=0;
		FF=(FF_AVG/10)*0.136476426;
		FF_AVG=0;
	}
}
*/

void start_convar2()
{
	
	ADMUX=0x47;
	_delay_ms(10);
	PDB_temp=take_value2();
	
	if(ADAVGCNTR4<10)
	{
		ADAVGCNTR4++;
		PDB_AVG+=PDB_temp;
	}
	else if(ADAVGCNTR4>=10)
	{
		ADAVGCNTR4=0;
		PDB=(PDB_AVG/10);
		PDB_AVG=0;
	}
}




/////////////////////////////////////////ADC convarsion end///////
/////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
/////////////////////////Disp Properties///////////

void disp_int(unsigned int dispval)
{
	if(dispval<10){lcdPrintData( "0",1);}
	itoa(dispval,ch,10);lcdPrintData(ch,strlen(ch));
}

void disp_float2(unsigned int adval)
{
	uint16_t b,nucnt=0;
	//float c;
	char x[10];

	//cnt=0;
	b=adval;
	if(b==0)
	{
		lcdPrintData("0",1);
	}

	itoa(b,x,10);

	if(adval<10)
	{
		lcdPrintData("0",1);
		lcdPrintData(x,strlen(x));
	}
	else
	{
		for(int cont=(strlen(x)-1);cont>=0;cont--)
		{
			lcdDataWrite(x[nucnt]);
			nucnt++;
			if(nucnt==2){
				lcdDataWrite(46);
			}
		}
	}
	
}



//////////////////////////////////////one wire start/////////////
/////////////////////////////////////////////////////////////////



uint8_t ow_input_pin_state()
{
	return OW_GET_IN();
}


uint8_t ow_reset(void)
{

	uint8_t err;
	uint8_t sreg;
	
	OW_OUT_LOW();
	OW_DIR_OUT();
	
	_delay_us(480);
	
	sreg=SREG;
	cli();
	
	OW_DIR_IN();
	
	_delay_us(66);
	err = OW_GET_IN();

	
	SREG=sreg;

	_delay_us((480-66));
	if( OW_GET_IN() == 0 )		// short circuit
	//TOUT=0;//tempo=0;
	err = 1;
	
	return err;
}


uint8_t ow_bit_io( uint8_t b )
{
	uint8_t sreg;
	
	sreg=SREG;
	cli();
	
	OW_DIR_OUT();
	
	_delay_us(1);
	if ( b ) OW_DIR_IN();
	
	
	_delay_us(((15-(1+OW_CONF_DELAYOFFSET)))); //NEED to up
	
	if( OW_GET_IN() == 0 ) b = 0;
	
	_delay_us((60-15));
	OW_DIR_IN();
	
	SREG=sreg;
	
	return b;
}


uint8_t ow_byte_wr( uint8_t b )
{
	uint8_t i = 8, j;
	
	do {
		j = ow_bit_io( b & 1 );
		b >>= 1;
		if( j ) b |= 0x80;
	} while( --i );
	
	return b;
}


uint8_t ow_byte_rd( void )
{
	return ow_byte_wr( 0xFF );
}


uint8_t ow_rom_search( uint8_t diff)
{
	uint8_t i, j, next_diff;
	uint8_t b;
	
	if( ow_reset() ) return OW_PRESENCE_ERR;	// error, no device found
	
	ow_byte_wr( OW_SEARCH_ROM );			// ROM search command
	next_diff = OW_LAST_DEVICE;			// unchanged on last device
	
	i = OW_ROMCODE_SIZE * 8;					// 8 bytes
	
	do {
		j = 8;					// 8 bits
		do {
			b = ow_bit_io( 1 );			// read bit
			if( ow_bit_io( 1 ) ) {			// read complement bit
				if( b )					// 11
				return OW_DATA_ERR;			// data error
			}
			else {
				if( !b ) {				// 00 = 2 devices
					if( diff > i || ( diff != i) ) {
						b = 1;				// now 1
						next_diff = i;			// next pass 0
					}
				}
			}
			ow_bit_io( b );     			// write bit
			i--;
			
		} while( --j );
		
		// next byte
		
	} while( i );
	
	return next_diff;				// to continue search
}



void ow_command( uint8_t command)
{


	ow_reset();

	ow_byte_wr( OW_SKIP_ROM );			// to all devices
	
	
	ow_byte_wr( command );
}
////////////////////////////////////////////////////////////////////
///////////////////////////////////////////One wire end//////////

////////////////////////////////////////////////////////////////
/////////////////////////////////////////////ds18x210start//////
////////////////////////////////////////////////////////////////



void DS18X20_find_sensor(uint8_t *diff)
{
	for (;;) {
		*diff = ow_rom_search( *diff );
		if ( *diff==OW_PRESENCE_ERR || *diff==OW_DATA_ERR ||
		*diff == OW_LAST_DEVICE )
		return;

		
		
	}
}





uint8_t DS18X20_start_meas()
{
	//if(ow_reset()==1){TOUT=0;}
	//	else{
	ow_reset();
	if( ow_input_pin_state() ) {
		ow_command(OW_SKIP_ROM);
		ow_command( DS18X20_CONVERT_T );
		return DS18X20_OK;
		//	}
		
	}
}





uint8_t DS18X20_read_meas_single()
{
	uint8_t i,tempo=0,Temp1=0;
	uint16_t meas;
	uint8_t sp[2]={0,0};
	
	//	if(ow_reset()==1){TOUT=0;}
	//	else{

	tempo=TOUT;
	ow_command(OW_SKIP_ROM);
	ow_command(DS18X20_READ);
	for ( i=0 ; i< 2; i++ ){ sp[i]=ow_byte_rd();}
	//ow_reset();
	if(ow_reset()){cnTT1++;if(cnTT1>20){Temp1=0;cnTT1=25;}}else{
	meas = sp[0];
	meas |= ((uint16_t)sp[1])<<8;
	Temp1=meas * 0.0625;
	cnTT1=0;
}

if(TAVGCN1<10){TEMP_AVG1+=Temp1;TAVGCN1++;}
else{TAVGCN1=0;TOUT=TEMP_AVG1/10;TEMP_AVG1=0;}
if(TOUT>150){TOUT=tempo;}
if(TOUT<3){TOUT=0;tempo=0;}
return DS18X20_OK;
//}
//	}
}




///////////////////////////////////////////////////////////////
///////////////////////////////ds18x20end/////////////////////
//////////////////////////////////////////////////////////////


uint8_t ow_input_pin_state2()
{
	return OW2_GET_IN();
}



uint8_t ow_reset2(void)
{

	uint8_t err;
	uint8_t sreg;
	
	OW2_OUT_LOW();
	OW2_DIR_OUT();
	
	_delay_us(480);
	
	sreg=SREG;
	cli();
	
	OW2_DIR_IN();
	
	_delay_us(66);
	err = OW2_GET_IN();

	
	SREG=sreg;

	_delay_us((480-66));
	if( OW2_GET_IN() == 0 )		// short circuit
	err = 1;
	
	return err;
}


uint8_t ow_bit_io2( uint8_t b )
{
	uint8_t sreg;
	
	sreg=SREG;
	cli();
	
	OW2_DIR_OUT();
	
	_delay_us(1);
	if ( b ) OW2_DIR_IN();
	
	
	_delay_us(((15-(1+OW_CONF_DELAYOFFSET)))); //NEED to up
	
	if( OW2_GET_IN() == 0 ) b = 0;
	
	_delay_us((60-15));
	OW2_DIR_IN();
	
	SREG=sreg;
	
	return b;
}


uint8_t ow_byte_wr2( uint8_t b )
{
	uint8_t i = 8, j;
	
	do {
		j = ow_bit_io2( b & 1 );
		b >>= 1;
		if( j ) b |= 0x80;
	} while( --i );
	
	return b;
}


uint8_t ow_byte_rd2( void )
{
	return ow_byte_wr2( 0xFF );
}


uint8_t ow_rom_search2( uint8_t diff)
{
	uint8_t i, j, next_diff;
	uint8_t b;
	
	if( ow_reset2() ) return OW_PRESENCE_ERR;	// error, no device found
	
	ow_byte_wr2( OW_SEARCH_ROM );			// ROM search command
	next_diff = OW_LAST_DEVICE;			// unchanged on last device
	
	i = OW_ROMCODE_SIZE * 8;					// 8 bytes
	
	do {
		j = 8;					// 8 bits
		do {
			b = ow_bit_io2( 1 );			// read bit
			if( ow_bit_io2( 1 ) ) {			// read complement bit
				if( b )					// 11
				return OW_DATA_ERR;			// data error
			}
			else {
				if( !b ) {				// 00 = 2 devices
					if( diff > i || ( diff != i) ) {
						b = 1;				// now 1
						next_diff = i;			// next pass 0
					}
				}
			}
			ow_bit_io( b );     			// write bit
			i--;
			
		} while( --j );
		
		// next byte
		
	} while( i );
	
	return next_diff;				// to continue search
}



void ow_command2( uint8_t command)
{


	ow_reset2();

	ow_byte_wr2( OW_SKIP_ROM );			// to all devices
	
	
	ow_byte_wr2( command );
}
////////////////////////////////////////////////////////////////////
///////////////////////////////////////////One wire end//////////

////////////////////////////////////////////////////////////////
/////////////////////////////////////////////ds18x210start//////
////////////////////////////////////////////////////////////////


void DS18X20_find_sensor2(uint8_t *diff)
{
	for (;;) {
		*diff = ow_rom_search2( *diff );
		if ( *diff==OW_PRESENCE_ERR || *diff==OW_DATA_ERR ||
		*diff == OW_LAST_DEVICE ) return;
		
	}
}





uint8_t DS18X20_start_meas2()
{
	ow_reset2();
	if( ow_input_pin_state2() ) {
		ow_command2(OW_SKIP_ROM);
		ow_command2( DS18X20_CONVERT_T );
		return DS18X20_OK;
	}
	

}





uint8_t DS18X20_read_meas_single2()
{
	uint8_t i,tempo=0,Temp2=0;
	uint8_t sp[2]={0,0};
	uint16_t meas;
	tempo=TROOM;
	ow_command2(OW_SKIP_ROM);
	ow_command2(DS18X20_READ);
	for ( i=0 ; i< 2; i++ ){ sp[i]=ow_byte_rd2();}
	//ow_reset2();
	if(ow_reset2()){cnTT2++;if(cnTT2>20){Temp2=0;cnTT2=25;}}else{
	meas = sp[0];
	meas |= ((uint16_t)sp[1])<<8;
	Temp2=meas * 0.0625;
	cnTT2=0;
		}

if(TAVGCN2<10){TEMP_AVG2+=Temp2;TAVGCN2++;}
else{TAVGCN2=0;TROOM=TEMP_AVG2/10;TEMP_AVG2=0;}
if(TROOM>150){TROOM=tempo;}
if(TROOM<3){TROOM=0;tempo=0;}

return DS18X20_OK;
}






///////////////////////////////////////////////////////////////
///////////////////////////////ds18x20end/////////////////////
//////////////////////////////////////////////////////////////


/////////////////////////Function ENDS/////////////////
