#include "usart_send.h"

volatile unsigned int buffercounter = 0;

char usart_rx_buffer[BUFFER_SIZE];
char *rx_buffer_pointer_in	= &usart_rx_buffer[0];
char *rx_buffer_pointer_out	= &usart_rx_buffer[0];

//----------------------------------------------------------------------------
//Init serielle Schnittstelle
void usart_init(unsigned long baudrate)
{
	//Serielle Schnittstelle 1
	//Enable TXEN im Register UCR TX-Data Enable
	UCR =(1 << TXEN );
	// 0 = Parity Mode Disabled
	// 1 = Parity Mode Enabled, Even Parity
	// 2 = Parity Mode Enabled, Odd Parity
	//UCSRC = 0x06 + ((parity+1)<<4);
	//UCSRC |= (1<<USBS);
	//Teiler wird gesetzt
	// Set baud rate
	UCSRA = 0;
	// Enable receiver and transmitter
	UBRR=(F_CPU / (baudrate * 16L) - 1);
	usart_status.usart_disable = 0;
}

//----------------------------------------------------------------------------
//Routine f�r die Serielle Ausgabe eines Zeichens (Schnittstelle0)
void usart_write_char(char c)
{
        if(!usart_status.usart_disable)
        {
            //Warten solange bis Zeichen gesendet wurde
            while(!(USR & (1<<UDRE)));
            //Ausgabe des Zeichens
            UDR = c;
        }
        return;
}

//------------------------------------------------------------------------------
void usart_write_P (const char *Buffer,...)
{
	va_list ap;
	va_start (ap, Buffer);

	int format_flag;
	char str_buffer[10];
	char str_null_buffer[10];
	char move = 0;
	char Base = 0;
	int tmp = 0;
	char by;
	char *ptr;

	//Ausgabe der Zeichen
    for(;;)
	{
		by = pgm_read_byte(Buffer++);
		if(by==0) break; // end of format string

		if (by == '%')
		{
            by = pgm_read_byte(Buffer++);
			if (isdigit(by)>0)
				{

 				str_null_buffer[0] = by;
				str_null_buffer[1] = '\0';
				move = atoi(str_null_buffer);
                by = pgm_read_byte(Buffer++);
				}

			switch (by)
				{
                case 's':
                    ptr = va_arg(ap,char *);
                    while(*ptr) { usart_write_char(*ptr++); }
                    break;
				case 'b':
					Base = 2;
					goto ConversionLoop;
				case 'c':
					//Int to char
					format_flag = va_arg(ap,int);
					usart_write_char (format_flag++);
					break;
				case 'i':
					Base = 10;
					goto ConversionLoop;
				case 'o':
					Base = 8;
					goto ConversionLoop;
				case 'x':
					Base = 16;
					//****************************
					ConversionLoop:
					//****************************
					itoa(va_arg(ap,int),str_buffer,Base);
					int b=0;
					while (str_buffer[b++] != 0){};
					b--;
					if (b<move)
						{
						move -=b;
						for (tmp = 0;tmp<move;tmp++)
							{
							str_null_buffer[tmp] = '0';
							}
						//tmp ++;
						str_null_buffer[tmp] = '\0';
						strcat(str_null_buffer,str_buffer);
						strcpy(str_buffer,str_null_buffer);
						}
					usart_write_str (str_buffer);
					move =0;
					break;
				}

			}
		else
		{
			usart_write_char ( by );
		}
	}
	va_end(ap);
}

//----------------------------------------------------------------------------
//Ausgabe eines Strings
void usart_write_str(char *str)
{

	while (*str)
	{
		usart_write_char(*str++);
	}
}
