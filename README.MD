### FCU V1 Landing Page

"LCD_MENU2"

"(SFTM_ITM==13)"

"void increment_MainItem()"

"void Pwm_Step()"

"LCD_MENU2" : 
				case 13:
	  			lcdPrintData("Fan Reconnect volt<",20);
	  			break;

"FLVD" = Low Voltage Flag

		lvcntr2++;
		if(lvcntr2 > 301)
		{
			lvcntr2=905;
			FLVD=0;
			lvcntr1=0;

