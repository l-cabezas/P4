#include "MKL46Z4.h"
#include "lcd.h"

// LED (RG)
// LED_GREEN = PTD5 (pin 98)
// LED_RED = PTE29 (pin 26)

// SWICHES
// RIGHT (SW1) = PTC3 (pin 73)
// LEFT (SW2) = PTC12 (pin 88)

// Enable IRCLK (Internal Reference Clock)
// see Chapter 24 in MCU doc

//unsigned int hits=0; //numero de aciertos
//unsigned int misses=0; //numero de fallos
unsigned int not_pressed=1; //variable global que se pone a 0 cuando se pulsa un boton y sirve apra esperar y que se quede bloqueado mientras no se pulsa

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

void delay(void)
{
  volatile int i;

  for (i = 0; i < 1000000; i++);
}

void delayfor(int n)
{
  volatile int i;
  for (i = 0; i < n; i++);
}

// RIGHT_SWITCH (SW1) = PTC3
void sw1_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3);
}

// LEFT_SWITCH (SW2) = PTC12
void sw2_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 12);
}

int sw1_check()
{
  return( !(GPIOC->PDIR & (1 << 3)) );
}

int sw2_check()
{
  return( !(GPIOC->PDIR & (1 << 12)) );
}

// RIGHT_SWITCH (SW1) = PTC3
// LEFT_SWITCH (SW2) = PTC12
void sws_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);
  
  PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); // IRQ on falling edge
  PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); // IRQ on falling edge
  
  // IRQ#31: Pin detect for PORTS C & D
  NVIC_SetPriority(31, 0); // Max priority for IRQ#31
  NVIC_EnableIRQ(31);      // Enable IRQ#31
}

// LED_GREEN = PTD5
void led_green_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOD->PSOR = (1 << 5);
}


// LED_RED = PTE29
void led_red_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR = (1 << 29);
}


// LED_RED = PTE29
// LED_GREEN = PTD5
void leds_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOE->PDDR |= (1 << 29);
  // both LEDS off after init
  GPIOD->PSOR = (1 << 5);
  GPIOE->PSOR = (1 << 29);
}


void PORTDIntHandler(void)
{
  PORTC->ISFR = 0xFFFFFFFF; // Clear IRQ
  //if((sw1_check() && (GPIOE->PDIR & (1 << 29)))||(sw2_check() && (GPIOD->PDIR & (1 << 5)))){
	//hits++;
    //}else{
    	//misses++;
    //} 
    not_pressed=0; 
}

int main(void)
{
  irclk_ini(); // Enable internal ref clk to use by LCD

  lcd_ini();
  
  leds_ini();
  sw1_ini();
  sw2_ini();
  sws_ini();
  
  //encender led verde por X tiempo cuandos e pulse el boton de los puntos y 3x cuandos epulse el de las rayas
  //cuando se introduzca un caracter se muestra por el lcd:
  //se introduce una S: tres puntos
  //seintroduce una O:tres rayas
  //si se introduce otro caracter o un caracter se escrie mals e borra el lcd
  //cuando se complete SOS se enciende el el rojo y SOS parpadea. no se aceptan mas entradas (no se vuelve a enceder el led verde)
 
  
  // 'Random' sequence :-)
  volatile unsigned int sequence = 0x1C7,
  index = 0;
  int vabien = 1;

  while (index < 9) {
    if (sequence & (1 << index)) { //deberia entrar un punto
    	while(not_pressed);
    	GPIOD->PCOR =(1<<5);
    	if(sw2_check()){
    		vabien=0;//el sw2 es el de las rayas, si se pulsa cuando hace falta un punto ya no va bien
    		delayfor(3000000);
    	}else{delayfor(1000000);}
    	GPIOD->PSOR =(1<<5);

    } else { //deberia entrar una raya
    	while(not_pressed);
    	GPIOD->PCOR =(1<<5);
    	if(sw1_check()){//el sw1 es el de los puntos
    		vabien=0;
    		delayfor(1000000);
    	}else{delayfor(3000000);}
    	GPIOD->PSOR =(1<<5);

    }
    
    if((vabien) && (index==2)){//mostrar S
    	lcd_display_dec(5);
    };
    if((vabien) && (index==5)){//mostrar SO
    	lcd_display_dec(50);
    };
    if((vabien) && (index==8)){//mostrar SOS
    	lcd_display_dec(505);
    };
    //si index es 2 o 5 o 8 se ha acabado de hacer una letra, imprimirla en el lcd o borrar
    not_pressed = 1;
    index++;
    
    if(!vabien){lcd_display_dec(0); index=0;vabien=1;};

  }
	//PORTC->PCR[3] &= ~(1<<24);
  	//PORTC->PCR[12]&= ~(1<<24);
  	

  GPIOE->PCOR =(1<<29);
  LCD->AR =LCD_AR_BLINK(1);

  while (1) {
  }

  return 0;
}
