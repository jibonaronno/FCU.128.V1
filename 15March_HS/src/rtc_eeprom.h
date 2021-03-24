
///////////////////For Time/////////////////////
uint8_t SECin=0,MINin=0,HOURin=0;
uint8_t MONTHin=0,DATEin=0,YEARin=0;
////////////////////////////////////////////////

unsigned char rtc_register[7];
unsigned char time[10];       //xx:xx:xx;
unsigned char date[12];         //xx/xx/xxxx;
unsigned char day;

#define  START            0x08
#define  REPEAT_START     0x10
#define  MT_SLA_ACK       0x18
#define  MT_SLA_NACK      0x20
#define  MT_DATA_ACK      0x28
#define  MT_DATA_NACK     0x30
#define  MR_SLA_ACK       0x40
#define  MR_SLA_NACK      0x48
#define  MR_DATA_ACK      0x50
#define  MR_DATA_NACK     0x58
#define  ARB_LOST         0x38

#define  ENABLE_I2C       TWCR= 0x44
#define  DISABLE_I2C      TWCR= 0x00

#define  ERROR_CODE         0x7e

#define  DS1307_W         0xd0
#define  DS1307_R         0xd1

#define      SECONDS         rtc_register[0]
#define      MINUTES         rtc_register[1]
#define      HOURS           rtc_register[2]
#define      DAY             rtc_register[3]
#define      DATE            rtc_register[4]
#define      MONTH           rtc_register[5]
#define      YEAR            rtc_register[6]


#define FALSE 0
#define TRUE 1


void twi_init(void)
{
	TWCR= 0x00; //disable twi
	TWBR= 0x12; //set bit rate
	TWSR= 0x01;
	TWAR= 0x00; //set slave address
	TWCR= 0x44; //enable twi
	//wdt_enable(WDTO_2S);
	wdt_reset();
}


////////////////////////////////////EEPROM//////////////////////
void EEOpen()
{
	//Set up TWI Module
	TWBR = 5;
	TWSR &= (~((1<<TWPS1)|(1<<TWPS0)));
	//wdt_disable();
	wdt_reset();
}

uint8_t EEWriteByte(uint16_t address,uint8_t data)
{
	do
	{
		//Put Start Condition on TWI Bus
		TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

		//Poll Till Done
		while(!(TWCR & (1<<TWINT)));

		//Check status
		if((TWSR & 0xF8) != 0x08)
			return FALSE;

		//Now write SLA+W
		//EEPROM @ 00h
		TWDR=0b10100000;	

		//Initiate Transfer
		TWCR=(1<<TWINT)|(1<<TWEN);

		//Poll Till Done
		while(!(TWCR & (1<<TWINT)));
	
	}while((TWSR & 0xF8) != 0x18);

	//Now write ADDRH
	TWDR=(address>>8);

	//Initiate Transfer
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x28)
		return FALSE;

	//Now write ADDRL
	TWDR=(address);
	

	//Initiate Transfer
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x28)
		return FALSE;

	TWDR=(data);


	
	//Initiate Transfer
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x28)
		return FALSE;

	//Put Stop Condition on bus
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	
	//Wait for STOP to finish
	while(TWCR & (1<<TWSTO));

	//Wait untill Writing is complete
	_delay_ms(12);

	//Return TRUE

	//lcdPrintData(" OK2",4);
	return TRUE;

}



uint8_t EEReadByte(uint16_t address)
{
	uint8_t data;

	//Initiate a Dummy Write Sequence to start Random Read
	do
	{
		//Put Start Condition on TWI Bus
		TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

		//Poll Till Done
		while(!(TWCR & (1<<TWINT)));

		//Check status
		if((TWSR & 0xF8) != 0x08)
			return FALSE;

		//Now write SLA+W
		//EEPROM @ 00h
		TWDR=0b10100000;	

		//Initiate Transfer
		TWCR=(1<<TWINT)|(1<<TWEN);

		//Poll Till Done
		while(!(TWCR & (1<<TWINT)));
	
	}while((TWSR & 0xF8) != 0x18);
	


	//Now write ADDRH
	TWDR=(address>>8);

	//Initiate Transfer
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x28)
		return FALSE;

	//Now write ADDRL


	TWDR=(address);


	//Initiate Transfer
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x28)
		return FALSE;

	//*************************DUMMY WRITE SEQUENCE END **********************	
	//Put Start Condition on TWI Bus
	TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x10)
		return FALSE;

	//Now write SLA+R
	//EEPROM @ 00h
	TWDR=0b10100001;	
	
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Poll Till Done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x40)
		return FALSE;

	//Now enable Reception of data by clearing TWINT
	TWCR=(1<<TWINT)|(1<<TWEN);

	//Wait till done
	while(!(TWCR & (1<<TWINT)));

	//Check status
	if((TWSR & 0xF8) != 0x58)
		return FALSE;

	//Read the data
	data=TWDR;

	
	//Put Stop Condition on bus
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	
	//Wait for STOP to finish
	while(TWCR & (1<<TWSTO));

	//Return TRUE
//	lcdPrintData(" OK2",4);
	return data;
}

////////////////////////////////////EEPROM ends/////////////////


///////////////////////////Disp Int End///////////////////////////////////

/////////////////////I2C Starts//////////////////////////////////////////

unsigned char i2c_start(void)
{
	
	TWCR = ((1<<TWINT)|(1<<TWSTA)|(1<<TWEN));       //Send START condition
	
	while (!(TWCR & (1<<TWINT)));      //Wait for TWINT flag set. This indicates that the
	//START condition has been transmitted
	if ((TWSR & 0xF8) == START)         //Check value of TWI Status Register
	return(0);
	else                            //if problem, transmit the code to PC  to know what's the problem
	{
		return(1);
	}

}


unsigned char i2c_repeatStart(void)
{
	
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);  //Send START condition
	while (!(TWCR & (1<<TWINT)));          //Wait for TWINT flag set. This indicates that the
	//START condition has been transmitted
	if ((TWSR & 0xF8) == REPEAT_START)       //Check value of TWI Status Register
	return(0);
	else
	return(1);
}

unsigned char i2c_sendAddress(unsigned char address)
{
	unsigned char STATUS;
	
	if((address & 0x01) == 0)
	STATUS = MT_SLA_ACK;
	else
	STATUS = MR_SLA_ACK;
	
	TWDR = address;
	TWCR = (1<<TWINT)|(1<<TWEN);//Load SLA_W into TWDR Register. Clear TWINT bit
	//in TWCR to start transmission of address
	while (!(TWCR & (1<<TWINT)));//Wait for TWINT flag set. This indicates that the
	//SLA+W has been transmitted, and
	//ACK/NACK has been received.
	if ((TWSR & 0xF8) == STATUS)//Check value of TWI Status Register
	return(0);
	else
	return(1);
}


unsigned char i2c_sendData(unsigned char data)
{
	TWDR = data;
	TWCR = (1<<TWINT) |(1<<TWEN);      //Load SLA_W into TWDR Register. Clear TWINT bit
	//in TWCR to start transmission of data
	while (!(TWCR & (1<<TWINT)));      //Wait for TWINT flag set. This indicates that the
	//data has been transmitted, and
	//ACK/NACK has been received.
	if ((TWSR & 0xF8) != MT_DATA_ACK)   //Check value of TWI Status Register
	return(1);
	else
	return(0);
}


unsigned char i2c_receiveData_ACK(void)
{
	unsigned char data;
	
	TWCR = (1<<TWEA)|(1<<TWINT)|(1<<TWEN);
	
	
	while (!(TWCR & (1<<TWINT)));            //Wait for TWINT flag set. This indicates that the
	//data has been received
	if ((TWSR & 0xF8) != MR_DATA_ACK)    //Check value of TWI Status Register
	return(ERROR_CODE);
	
	data = TWDR;
	return(data);
}


unsigned char i2c_receiveData_NACK(void)
{
	unsigned char data;
	
	TWCR = (1<<TWINT)|(1<<TWEN);
	
	while (!(TWCR & (1<<TWINT)));            //Wait for TWINT flag set. This indicates that the
	//data has been received
	if ((TWSR & 0xF8) != MR_DATA_NACK)   //Check value of TWI Status Register
	return(ERROR_CODE);
	
	data = TWDR;

	return(data);
}

void i2c_stop(void)
{
	TWCR =  (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);     //Transmit STOP condition
}

/////////////////////////////////////////I2c Ends/////////////////////


//////////////////////////////////////RTC STarts///////////////////
void RTC_updateRegisters(void)
{
	
	SECONDS = ((time[6] & 0x07) << 4) | (time[7] & 0x0f);
	MINUTES = ((time[3] & 0x07) << 4) | (time[4] & 0x0f);
	HOURS = ((time[0] & 0x03) << 4) | (time[1] & 0x0f);
	DAY = date[10];
	DATE = ((date[0] & 0x03) << 4) | (date[1] & 0x0f);
	MONTH = ((date[3] & 0x01) << 4) | (date[4] & 0x0f);
	YEAR = ((date[8] & 0x0f) << 4) | (date[9] & 0x0f);


}

void RTC_setStartAddress(void)
{
   unsigned char errorStatus;
   
   errorStatus = i2c_start();
   if(errorStatus == 1)
   {
   // lcdGotoXY(0,0);
  //  lcdPrintData("RTC start1 failed...",20);
       i2c_stop();
    return;
   }
   
   errorStatus = i2c_sendAddress(DS1307_W);
   
   if(errorStatus == 1)
   {
   // lcdGotoXY(0,0);
   // lcdPrintData("RTCsendAdress1failed",20);
    i2c_stop();
    return;
   }
   
   errorStatus = i2c_sendData(0x00);
   if(errorStatus == 1)
   {
    ///lcdGotoXY(0,0);
    ///lcdPrintData("EEPROMwrite-2 failed",20);
    i2c_stop();
    return;
   }

   i2c_stop();
}

void RTC_read(void)
{

  unsigned char errorStatus, i, data;
 

  errorStatus = i2c_start();
  if(errorStatus == 1)
   {
    // lcdGotoXY(0,0);
    // lcdPrintData("RTC start1 failed...",20);
       i2c_stop();
    return;
   }
   
   errorStatus = i2c_sendAddress(DS1307_W);
   
   if(errorStatus == 1)
   {
    // lcdGotoXY(0,0);
  //  lcdPrintData("RTCsendAdress1failed.",20);
    i2c_stop();
    return;
   }
 
   errorStatus = i2c_sendData(0x00);
   
   if(errorStatus == 1)
   {
    // lcdGotoXY(0,0);
    //lcdPrintData("RTC write-1 failed..",20);


    i2c_stop();
    return;
   }

    errorStatus = i2c_repeatStart();
   if(errorStatus == 1)
   {
    // lcdGotoXY(0,0);
   // lcdPrintData("RTCrepeatstartfailed",20);
       i2c_stop();
    return;
   }
   
    errorStatus = i2c_sendAddress(DS1307_R);
   
   if(errorStatus == 1)
   {
   // lcdGotoXY(0,0);
  // lcdPrintData("RTCsendAdress2failed",20);
    i2c_stop();
    return;
   }
 
    for(i=0;i<7;i++)
   {
      if(i == 6)      //no Acknowledge after receiving the last byte
           data = i2c_receiveData_NACK();
     else
          data = i2c_receiveData_ACK();
       
        if(data == ERROR_CODE)
        {
         lcdGotoXY(0,0);
        // lcdPrintData("RTCreceive failed",20);
         i2c_stop();
            return;
        }
    
  else    {
        // lcdGotoXY(0,2);
        // lcdPrintData("RTC receive OK......",20);
         }
   
     rtc_register[i] = data;
   }
   
   i2c_stop();
}    

//******************************************************************
//Function to read
//******************************************************************
void RTC_getTime(void)
{
	RTC_read();
	//unsigned int ta=0,tb=0;
	//time[9] = 0x00;	  //NIL
	//time[8] = ' ';
	time[7] = (SECONDS & 0x0f) | 0x30;		//seconds(1's)
	time[6] = ((SECONDS & 0x70) >> 4) | 0x30;		//seconds(10's)

	time[5] = ':';
	
	time[4] = (MINUTES & 0x0f) | 0x30;
	time[3] = ((MINUTES & 0x70) >> 4) | 0x30;
	time[2] = ':';
	
	time[1] = (HOURS & 0x0f) | 0x30;
	time[0] = ((HOURS & 0x30) >> 4) | 0x30;
	//   ta=time[7]-48;
	// tb=time[6]-48;
	//tc=tb*10+ta;

}





void RTC_getDate(void)
{
	RTC_read();
	date[11] = 0x00;  //NIL
	date[11] = ' ';
	date[9] = (YEAR & 0x0f) | 0x30;
	date[8] = ((YEAR & 0xf0) >> 4) | 0x30;
	date[7] = '0';
	date[6] = '2';
	date[5] = '/';
	date[4] = (MONTH & 0x0f) | 0x30;
	date[3] = ((MONTH & 0x10) >> 4) | 0x30;
	date[2] = '/';
	date[1] = (DATE & 0x0f) | 0x30;
	date[0] = ((DATE & 0x30) >> 4) | 0x30;
}



void DATE_TIME_Conversion()
{
	//ta=time[7]-48;tb=time[6]-48;tc=tb*10+ta;
	twi_init();
	
	RTC_getTime();
	SECin=(time[7]-48)+(time[6]-48)*10;
	MINin=(time[4]-48)+(time[3]-48)*10;
	HOURin=(time[1]-48)+(time[0]-48)*10;
	
	RTC_getDate();
	DATEin=(date[1]-48)+(date[0]-48)*10;
	MONTHin=(date[4]-48)+(date[3]-48)*10;
	YEARin=(date[9]-48)+(date[8]-48)*10;
	
}

unsigned char RTC_writeTime(void)
{
	unsigned char errorStatus, i;
	
	errorStatus = i2c_start();
	if(errorStatus == 1)
	{
		//lcdGotoXY(0,0);
		//lcdPrintData("RTC start1 failed...",20);
		i2c_stop();
		return(1);
	}
	
	errorStatus = i2c_sendAddress(DS1307_W);
	
	if(errorStatus == 1)
	{
		//lcdGotoXY(0,0);
		//lcdPrintData("RTCsendAdress1failed.",20);
		i2c_stop();
		return(1);
	}
	
	errorStatus = i2c_sendData(0x00);
	if(errorStatus == 1)
	{
		//lcdGotoXY(0,0);
		//lcdPrintData("RTC write-1 failed..",20);
		i2c_stop();
		return(1);
	}

	for(i=0;i<3;i++)
	{
		errorStatus = i2c_sendData(rtc_register[i]);
		if(errorStatus == 1)
		{
			//lcdGotoXY(0,0);
			//lcdPrintData("RTCwrite timefailed",20);
			i2c_stop();
			return(1);
		}
	}
	
	i2c_stop();
	return(0);
}
void RTC_updateTime(void)
{
	unsigned char  data;
	
	data =1;                             //receive hours
	time[0]= data;
	
	data = 6;
	time[1]= data;
	
	time[2]= ':';
	
	data = 5;                       //receive minutes
	
	time[3]= data;
	
	data = 6;
	
	time[4]= data;
	
	time[5]= ':';
	
	data = 5;                       //receive seconds
	
	time[6]= data;
	
	data =8;
	
	time[7]= data;
	
	
	RTC_updateRegisters();
	data = RTC_writeTime();
	
	if(data == 0)
	{
		lcdClear();
		lcdGotoXY(0,0);
		//lcdPrintData("TimeUpdtedsucessfuly",20);
	}

	else
	{
		lcdClear();
		lcdGotoXY(0,0);
		//lcdPrintData("Data ERROR          ",20);
	}
	return;
}






unsigned char RTC_writeDate(void)
{
	unsigned char errorStatus, i;
	
	errorStatus = i2c_start();
	if(errorStatus == 1)
	{
		lcdGotoXY(0,0);
		//lcdPrintData("RTC start1 failed...",20);
		i2c_stop();
		return(1);
	}
	
	errorStatus = i2c_sendAddress(DS1307_W);
	
	if(errorStatus == 1)
	{
		lcdGotoXY(0,0);
		//lcdPrintData("RTCsendAdress1failed.",20);
		i2c_stop();
		return(1);
	}
	
	errorStatus = i2c_sendData(0x03);
	if(errorStatus == 1)
	{
		lcdGotoXY(0,0);
		//lcdPrintData("RTC write-1 failed..",20);
		i2c_stop();
		return(1);
	}

	for(i=3;i<7;i++)
	{
		errorStatus = i2c_sendData(rtc_register[i]);
		if(errorStatus == 1)
		{
			lcdGotoXY(0,0);
			//lcdPrintData("RTC write datefailed",20);
			i2c_stop();
			return(1);
		}
	}
	
	i2c_stop();
	return(0);
}


void RTC_updateDate(void)
{
	unsigned char  data;
	
	
	data = 1;
	date[0]= data;
	
	data = 5;
	date[1]= data;
	
	date[2] = '/';
	
	
	data = 0;                       //receive month
	date[3]= data;
	data = 2;
	date[4] = data;
	
	date[5] = '/';                    //receive special char (/,-, etc.)
	
	date[6] = '2';                           //year is 20xx
	date[7] = '0';
	
	data = 1;          //receive seconds
	date[8]= data;
	
	data =3;
	date[9]= data;
	
	RTC_updateRegisters();
	data = RTC_writeDate();
	
	if(data == 0)
	{
		lcdGotoXY(0,0);
		//lcdPrintData("Date Updated sucessfully..",20);
	}
	return;

}

void update_time()
{
	unsigned int data;
	
	RTC_updateRegisters();
	data = RTC_writeTime();
	
	if(data == 0)
	{
		lcdClear();
		lcdGotoXY(0,0);
		//lcdPrintData("TimeUpdtedsucessfuly",20);
	}

	else
	{
		lcdClear();
		lcdGotoXY(0,0);
		//lcdPrintData("Data ERROR          ",20);
	}

}
void convert_func_Time(unsigned int x,unsigned int  y,unsigned int z)
{


	time[0]=(x/10)+48;
	time[1]=(x-((x/10)*10))+48;

	time[2]= ':';

	time[3]=(y/10)+48;
	time[4]=(y-((y/10)*10))+48;

	time[5]= ':';

	time[6]=(z/10)+48;
	time[7]=(z-((z/10)*10))+48;

	//time[9] = 0x00;	  //NIL
	//time[8] = ' ';
	
}

void update_Date()
{
	unsigned int data;
	
	RTC_updateRegisters();
	data = RTC_writeDate();
	
	if(data == 0)
	{
		lcdClear();
		lcdGotoXY(0,0);
		//lcdPrintData("DateUpdtedsucessfuly",20);
	}

	else
	{
		lcdClear();
		lcdGotoXY(0,0);
		//lcdPrintData("Data ERROR          ",20);
	}

}
void convert_func_Date(unsigned int x,unsigned int  y,unsigned int z)
{

	date[0]=(x/10)+48;
	date[1]=(x-((x/10)*10))+48;

	date[2]= '/';

	date[3]=(y/10)+48;
	date[4]=(y-((y/10)*10))+48;

	date[5]= '/';

	date[6] = '2';                           //year is 20xx
	date[7] = '0';
	
	//date[8] = '/';                           //year is 20xx
	
	date[8]=(z/10)+48;
	date[9]=(z-((z/10)*10))+48;

	//time[9] = 0x00;	  //NIL
	//time[8] = ' ';
	
}



/////////For RTC///////////////////////////////////



