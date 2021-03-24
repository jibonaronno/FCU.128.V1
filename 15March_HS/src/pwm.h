void pwm_init()
	{
	
    DDRB |= (1 << PINB7);
	
	OCR2 = 255;
    // set PWM for 50% duty cycle


    TCCR2 |= (1 << COM21);
    // set none-inverting mode

    TCCR2 |= (1 << WGM21) | (1 << WGM20);
    // set fast PWM Mode

    TCCR2 |= (1 << CS21);
    // set prescaler to 8 and starts PWM

	}
	
/*	
void pwm_init()
{
DDRB |= (1 << PINB7);
	
 // FOC2 WGM20 COM21 COM20 WGM21 CS22 CS21 CS20
 TCCR2 = 0b01101010; // Fast PWM 0xFF

 TCCR0 = 0x7; 
 //TIFR |= 0x1; //TCNT0=100; 
}
	
*/
void pwm_disruct()
	{
	
	OCR2 = 0;
    DDRB &= (~(1 << PINB7));

	}




