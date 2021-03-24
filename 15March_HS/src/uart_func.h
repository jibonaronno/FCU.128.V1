
char USART_Receive(void)
{
	while(!(UCSR0A & (1<<RXC0)))
	;
	return UDR0;
}

void USART_transmit(unsigned char data)
{
	while(!(UCSR0A & (1<<UDRE0)))
	;

	UDR0 = data;
}

void USART_TransmitString(unsigned char *dat)
{
	unsigned char size = strlen(dat);

	unsigned char i = 0;

//	sbi(DDRD, PD2);

//	PORTD |= (1<<PD2); /* Set the MAX485 Chip to output mode */

//	delay_us(1000);

	for(i=0;i<size;i++)
	{
		USART_transmit(dat[i]);
	}

//	delay_us(2000);

//	cbi(PORTD, PD2);
}

void USART_Init(unsigned int baud)
{
	UBRR0H = (unsigned char)(baud>>8);

	UBRR0L = (unsigned char)baud;

	UCSR0B = (1<<TXEN0);

	//UCSR0C = (1<<USBS0)|(3<<UCSZ00)|(1<<URSEL);
	//UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);// Set the USART data size to 8b
	//UCSR0B = (1<<RXEN0)|(1<<TXEN0);  // Enable the receiver and transmitter
	DDRE |= (1<<1);                  // Set the Tx line (PE0) to be an output.
	
	
}

void USART_Init_Second()
{
	UCSR1B |= (1 << RXEN1) | (1 << TXEN1);
	UCSR1C |= (0 << UMSEL1) | (1 << UCSZ10) | (1 << UCSZ11);
	UBRR1H = (unsigned char)(51>>8);
	UBRR1L = (unsigned char)51;
}

void USART_transmit_Second(unsigned char data)
{
	while ((UCSR1A & (1 << UDRE1)) == 0) {};
	UDR1 = data;
}

void USART_TransmitString_Second(unsigned char *dat)
{
	unsigned char size = strlen(dat);

	unsigned char i = 0;

	for(i=0;i<size;i++)
	{
		USART_TransmitString_Second(dat[i]);
	}
}

void ClearCommandBuffer()
{
	clear_buffer_count = 1;
}

void disp_int_onPC(unsigned int adval)
{
	uint16_t b,cnt;
	float c;
	char x[10];
	for(int inf=0;inf<=5;inf++)
	{
		x[inf]='0';
	}
	cnt=0;
	b=adval;
	if(b==0)
	{
		USART_transmit('0');
	}

	while(b!=0)
	{
		c=(b%10);
		b=b/10;
		x[cnt]=c+48;
		cnt++;
	}
	if(adval< 10)
	{
		USART_transmit('0');
		for(int cont=(cnt-1);cont>=0;cont--)
		{
			USART_transmit(x[cont]);
		}
	}
	else
	{
		for(int cont=(cnt-1);cont>=0;cont--)
		{
			USART_transmit(x[cont]);
		}
	}
}

void enter()
{
	USART_transmit(13);
	USART_transmit(10);
}

void disp_float_onPC(unsigned int adval)
{
	uint16_t b,nucnt=0;
	//float c;
	char x[10];

	//cnt=0;
	b=adval;
	if(b==0)
	{
		USART_transmit("0");
	}

	itoa(b,x,10);

	if(adval<10)
	{
		USART_transmit("0");
		for(int cont=(strlen(x)-1);cont>=0;cont--)
		{
			USART_transmit(x[cont]);
		}
	}
	else
	{
		for(int cont=(strlen(x)-1);cont>=0;cont--)
		{
			USART_transmit(x[nucnt]);
			nucnt++;
			if(nucnt==2){
				USART_transmit(46);
			}
		}
	}
	
}