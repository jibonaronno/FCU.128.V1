#define DHT_DHT11 1
#define DHT_FLOAT 0
#define DHT_TIMEOUT 200

#define DHT_DDR DDRF
#define DHT_PORT PORTF
#define DHT_PIN PINF
#define DHT_INPUTPIN PF3

int8_t temperature = 0;
int8_t humidity = 0;

//-----------------delays---------------------------------------------------------
#define LOOP_CYCLES 8 				//Number of cycles that the loop takes

#define fcpu_delay_us(num) delay_int(num/(LOOP_CYCLES*(1/(F_CPU/1000000.0))))
#define fcpu_delay_ms(num) delay_int(num/(LOOP_CYCLES*(1/(F_CPU/1000.0))))


void delay_int(unsigned long delay)
{
	while(delay--) asm volatile("nop");
};



int8_t dht_getdata(int8_t *temperature, int8_t *humidity) {
	
	uint8_t bits[5];
	uint8_t i,j = 0;

	memset(bits, 0, sizeof(bits));

	//reset port
	DHT_DDR |= (1<<DHT_INPUTPIN); //output
	DHT_PORT |= (1<<DHT_INPUTPIN); //high
	fcpu_delay_ms(100);

	//send request
	DHT_PORT &= ~(1<<DHT_INPUTPIN); //low
	fcpu_delay_ms(18);
	
	DHT_PORT |= (1<<DHT_INPUTPIN); //high
	DHT_DDR &= ~(1<<DHT_INPUTPIN); //input
	fcpu_delay_us(40);

	//check start condition 1
	if((DHT_PIN & (1<<DHT_INPUTPIN))) {
		return -1;
	}
	fcpu_delay_us(80);
	//check start condition 2
	if(!(DHT_PIN & (1<<DHT_INPUTPIN))) {
		return -1;
	}
	fcpu_delay_us(80);

	//read the data
	uint16_t timeoutcounter = 0;
	for (j=0; j<5; j++) { //read 5 byte
		uint8_t result=0;
		for(i=0; i<8; i++) {//read every bit
			timeoutcounter = 0;
			while(!(DHT_PIN & (1<<DHT_INPUTPIN))) { //wait for an high input (non blocking)
				timeoutcounter++;
				if(timeoutcounter > DHT_TIMEOUT) {
					return -1; //timeout
				}
			}
			fcpu_delay_us(30);
			if(DHT_PIN & (1<<DHT_INPUTPIN)) //if input is high after 30 us, get result
			result |= (1<<(7-i));
			timeoutcounter = 0;
			while(DHT_PIN & (1<<DHT_INPUTPIN)) { //wait until input get low (non blocking)
				timeoutcounter++;
				if(timeoutcounter > DHT_TIMEOUT) {
					return -1; //timeout
				}
			}
		}
		bits[j] = result;
	}

	//reset port
	DHT_DDR |= (1<<DHT_INPUTPIN); //output
	DHT_PORT |= (1<<DHT_INPUTPIN); //low
	fcpu_delay_ms(100);

	//check checksum
	if ((uint8_t)(bits[0] + bits[1] + bits[2] + bits[3]) == bits[4]) {
		//return temperature and humidity
		
		*temperature = bits[2];
		*humidity = bits[0];
		
		return 0;
	}

	return -1;
}


int8_t dht_gettemperaturehumidity(int8_t *temperature, int8_t *humidity) {

	return dht_getdata(temperature, humidity);
}