/*
 * Domotica.c
 *
 * Created: 9/3/2020 19:30:10
 * Author : Dell
 */ 



#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

//Declaration of our functions
void USART_init(void);
unsigned char USART_receive(void);
void USART_send( unsigned char data);

unsigned int tiempoPresentar,tiempoPresentar1;

#define LED PORTA		/* connected LED on PORT pin */
#define LED1 PORTC		/* connected LED on PORT pin */
#define LED2 PORTK		/* connected LED on PORT pin */

// Definimos el valor máximo que puede tomar el PWM.
#define PWM_MAX 255
// Led conectado al PIN PB2 del micro donde está la salida PWM.
#define LEDPIN 2


int main(void){
	
	// PWM de 8 bits, fase correcta.
	// 8 bits 256 niveles de brillo.
	TCCR0A |= (1<<WGM00);

	// Limpiamos los bits OC0A/OC0B en el comparador.
	// Setea OC0A/OC0B en la parte inferior (Modo no invertido)
	TCCR0A |= (1<<COM0A1);

	// Seteamos el prescaler a 64.
	// 4 MHz / 64*256 = Frecuencia de PWM = ~235 Hz
	TCCR0B = (1<<CS00)|(1<<CS01);

	// -------------------------
	
	uint8_t brillo1 = 0;
	uint8_t brilloz = 20;
	uint8_t brillo2=  63;
	uint8_t brillo3 = 126;
	uint8_t brillo4 = 189;
	uint8_t brillo5 = 255;
	
	
	
	void Wait()
	{
		uint8_t i;
		for(i=0;i<10;i++)
		{
			_delay_loop_2(0);
			_delay_loop_2(0);
			_delay_loop_2(0);
		}
		
	}
	
	
	
	char Data_in;
	DDRA = 0xff;		/* make PORT as output port */
	DDRB = 0xff;		/* make PORT as output port */
	DDRC = 0xff;		/* make PORT as output port */
	DDRD = 0xff;		/* make PORT as output port */
	DDRK = 0xff;
	PORTB = 0x00; // Ponemos todas las salidas a 0.
	LED = 0;
	LED1 = 0;
	LED2 = 1;
	
	USART_init();        //Call the USART initialization code

	while(1){        //Infinite loop
		
		Data_in = USART_receive();	/* receive data from Bluetooth device*/
		
		//LEDS SALA
		if(Data_in =='0')
		{
			LED |= (1<<PA0);	/* Turn ON LED */
			LED |= (1<<PA1);	/* Turn ON LED */
		}
		else
		
		if(Data_in =='1')
		{
			LED &= ~(1<<PA0);	/* Turn OFF LED */
			LED &= ~(1<<PA1);	/* Turn OFF LED */
		}
		
		
		
		//LEDS DORMITORIO
		if(Data_in =='2')
		{
			OCR0A = brillo5;
			LED |= (1<<PA3);	/* Turn ON LED */
		}
		else
		
		if(Data_in =='3')
		{
			OCR0A = brillo1;
			LED &= ~(1<<PA3);	/* Turn OFF LED */
		}
		
		
		//LEDS BA�O
		if(Data_in =='4')
		{
			LED |= (1<<PA4);	/* Turn ON LED */
		}
		else
		
		if(Data_in =='5')
		{
			LED &= ~(1<<PA4);	/* Turn OFF LED */
		}
		
		//LEDS GARAJE
		if(Data_in =='6')
		{
			LED |= (1<<PA5);	/* Turn ON LED */
		}
		else
		
		if(Data_in =='7')
		{
			LED &= ~(1<<PA5);	/* Turn OFF LED */
		}
		
		//LEDS COCINA
		if(Data_in =='8')
		{
			LED |= (1<<PA6);	/* Turn ON LED */
		}
		else
		
		if(Data_in =='9')
		{
			LED &= ~(1<<PA6);	/* Turn OFF LED */
		}
		
		
		
		
		//SERVO MOTOR
		
		if(Data_in =='A'||Data_in =='a')
		{
			
			TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
			TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)

			ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).

			DDRB=(1<<PB5);   //PWM Pins as Out
			
			OCR1A=120;   //0 degree
			Wait();
			ICR1=0;
			
			}else{
			
			if(Data_in =='B'||Data_in =='b')
			{
				
				ICR1=4999;
				OCR1A=450;  //90 degree
				Wait();
				
				ICR1=0;
				
			}
		}
		
		
				
		// SENSOR LUZ
		if(Data_in =='C'||Data_in =='c')
		{
			LED |= (1<<PA7);	/* Turn ON LED */
		}
		else
		
		if(Data_in =='D'||Data_in =='d')
		{
			LED &= ~(1<<PA7);	/* Turn OFF LED */
		}
		
		
		//ALARMA
		if(Data_in =='E'||Data_in =='e')
		{
			LED1 |= (1<<PC7);	/* Turn ON LED */
		}
		else
		
		if(Data_in =='F'||Data_in =='f')
		{
			LED1 &= ~(1<<PC7);	/* Turn OFF LED */
		}
		
		
		
		//LUZ DESC
		if(Data_in =='G')
		{
		
		// Cargamos el valor de brillo en el comparador.
		OCR0A = brilloz;
		}
		
		if(Data_in =='H')
		{
			// Cargamos el valor de brillo en el comparador.
			OCR0A = brillo2;
			
		}
		
		
		if(Data_in =='I')
		{
		LED &= ~(1<<PA1);	/* Turn OFF LED */

			// Cargamos el valor de brillo en el comparador.
			OCR0A = brillo3;
		}
		
		
		if(Data_in =='J')
		{

			// Cargamos el valor de brillo en el comparador.
			OCR0A = brillo4;
		}
		
		if(Data_in =='K')
		{

			// Cargamos el valor de brillo en el comparador.
			OCR0A = brillo5;
		}
		
		//Next
		if(Data_in =='L')
		{
				
				tiempoPresentar++;
				if (tiempoPresentar==1)
				{
					LED2 &= ~(1<<PK0);	/* Turn OFF LED */
				}
				if (tiempoPresentar==2)
				{
					LED2 |= (1<<PK0);	/* Turn ON LED */
							tiempoPresentar=0;

					
				}
						
		}
		
				
			
	}

	return 0;
}

void USART_init(void){
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	tiempoPresentar=0;
	tiempoPresentar1=0;
	
}

unsigned char USART_receive(void){

	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;

}

void USART_send( unsigned char data){

	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;

}















/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


