/*
 * ADC.c
 *
 *  Created on: 04/11/2019
 *      Author: ravenelco
 */
#include "lib/include.h"

#define NVIC_ST_CTRL_R      (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile uint32_t *)0xE000E018))
void delay_ms(uint8_t milis);

extern void Configura_Reg_ADC(void)
{
    /*
    Habilitar el modulo 0  del ADC con  tres canales analogicos puerto D  a una velocidad de conversion de 1msps
    dandole la mayor prioridad al secuenciador 1, segunda al secuenciador 2,
     y tercera al secuenciador 3 con evento de interrupción por GPIO
    */
     //Pag 396 para inicializar el modulo de reloj del adc RCGCADC
    SYSCTL->RCGCADC |= (1<<0); 
    //Pag 382 (RGCGPIO) Puertos base habilitación del reloj(D para UART2 y ADC,E para ADC, F para leds )
    //                     F     E      D       C      B     A
    SYSCTL->RCGCGPIO |= (0<<5)|(0<<4)|(1<<3)|(0<<2)|(0<<1)|(0<<0);
    //Pag 760 (GPIODIR) Habilta los pines como I/O un cero para entrada y un uno para salida
    
    GPIOD->DIR |= (0<<0)| (0<<2)|(0<<3); //PD0 , PD1 y PD3

      //(GPIOAFSEL) pag.770 Enable alternate función para que el modulo analógico tenga control de esos pines
    GPIOD->AFSEL |=  (1<<0) | (1<<2)|(1<<3);

     //(GPIODEN) pag.781 desabilita el modo digital
    GPIOD->DEN = (0<<0) | (0<<2)|(0<<3); 

   //Pag 660 GPIOPCTL *VA NEGADA? registro combinado con el GPIOAFSEL y la tabla pag 1351
    GPIOD->PCTL = GPIOD->PCTL &~(0xFFFF00F0);
    
  //(GPIOAMSEL) pag.687 habilitar analogico
    GPIOD->AMSEL |= (1<<0) | (1<<2)|(1<<3);
   
    //Pag 891 El registro (ADCPC) establece la velocidad de conversión por segundo
    ADC0->PC = 0x7;//1Msps
   
    //Pag 841 Este registro (ADCSSPRI) configura la prioridad de los secuenciadores
    ADC0->SSPRI = 0x0000; //secuenciador 0, menor prioridad porque no se me fue asignado    
    
    //Pag 821  (ADCACTSS) Este registro controla la activación de los secuenciadores
    ADC0->ACTSS  =   (0<<3) | (0<<2) | (0<<1) | (0<<0);
    
    //Pag 833 Este registro (ADCEMUX) selecciona el evento que activa la conversión (trigger)
    ADC0->EMUX  = 0x0; //EVENTO POR PROCESADOR
    //Pag 867 Este registro (ADCSSMUX1) define las entradas analógicas con el canal y secuenciador seleccionado
    ADC0->SSMUX1 = 0x0457; //SEÑAL, 4, 5 Y 7
    //pag 868 Este registro (ADCSSCTL2), configura el bit de control de muestreo y la interrupción
    ADC0->SSCTL1 = 0X0644;  //OJO, investigar esste registro
      
 
 
    /* Enable ADC Interrupt */
    ADC0->IM |= (1<<1); /* Unmask ADC0 sequence 1 interrupt pag 825*/
     
    //NVIC_PRI4_R = (NVIC_PRI4_R & 0xFFFFFF00) | 0x00000020;
    //NVIC_EN0_R = 0x00010000;
    //Pag 821 (ADCACTSS) Este registro controla la activación de los secuenciadores
    //ADC0->ACTSS |= (0<<3) | (0<<2) | (1<<1) | (0<<0);
    ADC0->PSSI |= (1<<1);


        ////////////////ADC1//////////////////////////////////////////////////////////////////////////////////////////
    SYSCTL->RCGCADC |= (1<<0); 
    //Pag 382 (RGCGPIO) Puertos base habilitación del reloj(D para UART2 y ADC,E para ADC, F para leds )
    //                     F     E      D       C      B     A
    SYSCTL->RCGCGPIO |= (0<<5)|(1<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0);
    //Pag 760 (GPIODIR) Habilta los pines como I/O un cero para entrada y un uno para salida
    GPIOE->DIR |= (0<<0) | (0<<1)|(0<<5); //PE0 y PE5

    //(GPIOAFSEL) pag.770 Enable alternate función para que el modulo analógico tenga control de esos pines
    GPIOE->AFSEL =   (1<<0) | (1<<1)| (1<<5);

     //(GPIODEN) pag.781 desabilita el modo digital
    GPIOE->DEN = (0<<0) | (0<<1)|(0<<5);

   //Pag 660 GPIOPCTL *VA NEGADA? registro combinado con el GPIOAFSEL y la tabla pag 1351
    GPIOE->PCTL = GPIOE->PCTL &~(0xFFFF00F0);;
    
  //(GPIOAMSEL) pag.687 habilitar analogico
    GPIOE->AMSEL |= (1<<0) | (1<<1)| (1<<5);
   
    //Pag 891 El registro (ADCPC) establece la velocidad de conversión por segundo
    ADC1->PC = 0x7; //1Msps
   
    //Pag 841 Este registro (ADCSSPRI) configura la prioridad de los secuenciadores
    //ADC1->SSPRI = 0x0231; //secuenciador 0, menor prioridad porque no se me fue asignado
    
    //Pag 821  (ADCACTSS) Este registro controla la activación de los secuenciadores
    ADC1->ACTSS  =   (0<<3) | (0<<2) | (0<<1) | (0<<0);
    
    //Pag 833 Este registro (ADCEMUX) selecciona el evento que activa la conversión (trigger)
    ADC1->EMUX  = 0x0; //EVENTO POR PROCESADOR

//      
    ADC1->SSMUX2 = 0x0023;//SEÑAL 2 Y 3
    ADC1->SSCTL2 = 0X0064; ; //OJO, investigar esste registro

    ADC1->SSMUX3 = 0x0008; //SEÑAL EN 8 
    ADC1->SSCTL3 = 0X0006;; //OJO, investigar esste registro  

    /* Enable ADC Interrupt */
   
    ADC1->IM |= (1<<2); /* Unmask ADC0 sequence 2 interrupt pag 825*/
    ADC1->IM |= (1<<3); /* Unmask ADC0 sequence 2 interrupt pag 825*/
   
    //NVIC_PRI4_R = (NVIC_PRI4_R & 0xFFFFFF00) | 0x00000020;
    //NVIC_EN0_R = 0x00010000;
    //Pag 821 (ADCACTSS) Este registro controla la activación de los secuenciadores
    
    ADC1->ACTSS |= (1<<3) | (1<<2) | (1<<0) | (0<<0);
    ADC1->PSSI |=  (1<<3) | (1<<2);

}


/*
extern void ADC0_InSeq2(uint16_t *Result){

    //ADC Processor Sample Sequence Initiate (ADCPSSI)
       ADC0->PSSI = 0x00000004;
       while((ADC0->RIS&0x04)==0){}; // espera al convertidor
       Result[1] = ADC0->SSFIFO2&0xFFF; //  Leer  el resultado almacenado en la pila2
       Result[0] = ADC0->SSFIFO2&0xFFF;
       printChar('A');
       ADC0->ISC = 0x0004;  //Conversion finalizada
}

*/

extern void Leer_ADC(uint16_t data[])
{
     // selector 1
    ADC0->PSSI |= (1<<0);
    delay_ms(1);

    while ((ADC0->RIS & 0x02) == 0);
    delay_ms(1);
    data[0] = ADC0->SSFIFO1 & 0xfff;
    data[1] = ADC0->SSFIFO1 & 0xfff;
    data[2] = ADC0->SSFIFO1 & 0xfff;
   

    ADC0->ISC |= (1<<0);
    delay_ms(1);

    // selector 2
    ADC1->PSSI |= (1<<1);
    delay_ms(1);

    while ((ADC1->RIS & 0x04) == 0);
    delay_ms(1);
    data[3] = ADC1->SSFIFO2 & 0xfff;
    data[4] = ADC1->SSFIFO2 & 0xfff;
    ADC1->ISC |= (1<<1);

    //selector 3
ADC1 ->PSSI |= (1<<2);
    while ((ADC1->RIS & 0x08) == 0);
    delay_ms(1);
    data[5] = ADC1->SSFIFO3 & 0xfff;
    ADC1->ISC |= (1<<1);
}

void delay_ms(uint8_t milis)
{
    uint8_t indice;
    for(indice=0;indice<milis;indice++)
    {
        NVIC_ST_RELOAD_R=1999;
        NVIC_ST_CTRL_R=5;
        while ((NVIC_ST_CTRL_R&0X10000)){}
    }
}

