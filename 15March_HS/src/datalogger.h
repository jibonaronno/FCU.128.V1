uint32_t LAST_ADDRESS=140000;
uint16_t DATANO=0,VOLT_SAVE=0;
uint8_t VOLT[4];


void convert_volttoint(uint32_t test_card)
{
	VOLT[0] = test_card & 0x000000ff;          //lower 8 bits
	VOLT[1] = (test_card & 0x0000ff00) >> 8;
}

uint32_t convertinttovolt(uint8_t arr[])
{
	VOLT_SAVE=0;
	for (int i =1; i >=0 ; --i)
	{
		VOLT_SAVE= (VOLT_SAVE << 8) | arr[i];
	}
	return 	VOLT_SAVE;
}

void save_record_no(uint16_t record)
{
	uint8_t rec1=0,rec2=0;
	rec1 = record & 0x000000ff;          //lower 8 bits
	rec2= (record & 0x0000ff00) >> 8;
	eeprom_write_byte((uint8_t*)505,rec1);
	eeprom_write_byte((uint8_t*)506,rec2);
}

void save_record_no_full(uint16_t record)
{
	uint8_t rec1=0,rec2=0;
	rec1 = record & 0x000000ff;          //lower 8 bits
	rec2= (record & 0x0000ff00) >> 8;
	eeprom_write_byte((uint8_t*)507,rec1);
	eeprom_write_byte((uint8_t*)508,rec2);
}

uint16_t no_of_saved_data()
{
	uint8_t rec[2];
	uint32_t recc=0;
	
	rec[0]=eeprom_read_byte((uint8_t*)505);
	rec[1]=eeprom_read_byte((uint8_t*)506);
	
	for (int i =1; i >=0 ; --i)
	{
		recc= (recc << 8) | rec[i];
	}

	return recc;
}

uint16_t no_of_saved_data_full()
{
	uint8_t rec[2];
	uint32_t recc=0;

	rec[0]=eeprom_read_byte((uint8_t*)507);
	rec[1]=eeprom_read_byte((uint8_t*)508);
	
	for (int i =1; i >=0 ; --i)
	{
		recc= (recc << 8) | rec[i];
	}
	return recc;
}


void EXT_EEPROM_PAGE_WRITE()
{
	uint16_t testRec=0;
	uint32_t StartAddress=0;
	int PAGESIZE=12;
	testRec=no_of_saved_data();
	StartAddress=testRec*12;
	
	convert_volttoint(BTVOLT*10);

	if((StartAddress+PAGESIZE)>LAST_ADDRESS)
	{
		lcdClear();
		lcdGotoXY(0,3);
		lcdPrintData("EEPROM FULL        ",20);
		eeprom_write_byte((uint8_t*)152,1);
		save_record_no(0);
		save_record_no_full(testRec);
		testRec=0;
		StartAddress=testRec*12;
		//disp_delay();
	}
	
	
	_delay_ms(10);
	EEWriteByte(StartAddress,DATEin);
	_delay_ms(10);
	
	EEWriteByte(StartAddress+1,MONTHin);
	_delay_ms(10);
	
	EEWriteByte(StartAddress+2,YEARin);
	_delay_ms(10);


	EEWriteByte(StartAddress+3,HOURin);
	_delay_ms(10);

	EEWriteByte(StartAddress+4,MINin);
	_delay_ms(10);

	EEWriteByte(StartAddress+5,TROOM);
	_delay_ms(10);

	EEWriteByte(StartAddress+6,TOUT);
	_delay_ms(10);

	EEWriteByte(StartAddress+7,FANMODE);
	_delay_ms(10);
	
	EEWriteByte(StartAddress+8,FANFAULTY);
	_delay_ms(10);
	
	EEWriteByte(StartAddress+9,FJAM);
	_delay_ms(10);
	
	EEWriteByte(StartAddress+10,VOLT[0]);
	_delay_ms(10);
	
	EEWriteByte(StartAddress+11,VOLT[1]);
	_delay_ms(10);


	testRec=testRec+1;
	save_record_no(testRec);
}


void Save_EEPROM()
{

	DATE_TIME_Conversion();

	if((MINin%INTERV)==0)
	{
		if((SECin==0) & (EEP_CHK_BIT==1))
		{
			
			EEOpen();
			EXT_EEPROM_PAGE_WRITE();
			EEP_CHK_BIT=2;
			twi_init();
			SAVED_DATA=no_of_saved_data();
		}
	}
	if(SECin==1){EEP_CHK_BIT=1;}
}

void SHOW_SAVED_DATA_PC(uint32_t ST_AD)
{
	
	int tempData=0,PRINT=0;
	uint32_t START_ADD_EXEE=0,CARD_ID_PC=0;
	
	DATANO++;
	USART_TransmitString("->");
	if(DATANO>10){if(DATANO>100){if(DATANO>=1000){;}else{USART_TransmitString("0");}}else{USART_TransmitString("00");}}else{USART_TransmitString("00");}
	disp_int_onPC(DATANO);USART_TransmitString(":-");
	USART_TransmitString("Date=");
	EEOpen();
	
	
	for(START_ADD_EXEE=ST_AD;START_ADD_EXEE<ST_AD+12;START_ADD_EXEE+=1)
	{
		PRINT++;
		tempData=EEReadByte(START_ADD_EXEE);
		
		if(PRINT<6){
			disp_int_onPC(tempData);
		}
		else if(PRINT==6){USART_TransmitString("Troom=");disp_int_onPC(tempData);}
			else if(PRINT==7){USART_TransmitString(" Tout=");disp_int_onPC(tempData);}
				else if(PRINT==8){USART_TransmitString(" FANMODE=");disp_int_onPC(tempData);}
					else if(PRINT==9){USART_TransmitString(" FAN_ALARM=");disp_int_onPC(tempData);}
						else if(PRINT==10){USART_TransmitString(" Filter_Jam=");disp_int_onPC(tempData);}
							else if(PRINT==11){VOLT[0]=tempData;USART_TransmitString(" BAT_VOLTAGE=");}
								else if(PRINT==12){VOLT[1]=tempData;convertinttovolt(VOLT);disp_float_onPC(VOLT_SAVE);USART_TransmitString("V");}
		
		if(PRINT==1){USART_transmit('-');}
		else if(PRINT==2){USART_transmit('-');}//USARTWriteChar('1');USARTWriteChar('3');
		else if(PRINT==3){USART_TransmitString(" Time=");}//USARTWriteChar('1');USARTWriteChar('3');
		else if(PRINT==4){USART_transmit(':');}
		else if(PRINT==5){USART_transmit(' ');}
		wdt_reset();
		wdt_enable(WDTO_2S);
	}
	twi_init();
	enter();
	_delay_ms(100);
	wdt_reset();
	wdt_enable(WDTO_2S);

}

void SHOW_ROLING_EEPROM_EX_DATA()
{
	uint16_t testRec=0,testRec2=0;
	long StartAddress=0;
	testRec=no_of_saved_data();
	
	PORTE &=(~(1 << PINE5));
	
	enter();
	USART_TransmitString("Data Sending Starts");
	DATANO=0;
	if(testRec==0){USART_TransmitString("NO SAVED EEPROM DATA");}
	else{
		
		if(eeprom_read_byte((uint8_t*)(152))==1)
		{
			testRec=no_of_saved_data_full();
			enter();
			testRec2=no_of_saved_data();
			
			while(testRec2!=0)
			{
				wdt_reset();
				wdt_enable(WDTO_2S);
				StartAddress=(testRec2-1)*12;
				SHOW_SAVED_DATA_PC(StartAddress);
				testRec2--;
			}
			
			testRec2=no_of_saved_data();
			
			while(testRec!=testRec2)
			{	wdt_reset();
				wdt_enable(WDTO_2S);
				StartAddress=(testRec-1)*12;
				SHOW_SAVED_DATA_PC(StartAddress);
				testRec--;
				//testRec2++;
			}
		}////////////////////Data Full
		else
		{
			//lcdClear();
			enter();
			testRec2=0;
			while(testRec!=0)
			{	wdt_reset();
				wdt_enable(WDTO_2S);
				StartAddress=(testRec-1)*12;
				
				SHOW_SAVED_DATA_PC(StartAddress);
				testRec--;
				//testRec2++;
			}
		}////////////////////Data Not Full
		
	}/////////////////testRec=0;
	
	PORTE |=((1<<PINE5));
	
}

