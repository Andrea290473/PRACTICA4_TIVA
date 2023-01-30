#include "lib/include.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>


#define FUNDELAY
void funcion(uint16_t num);





int main(void)
{
    //char arreglo[32];
    //uint16_t Result[2];
    //float valor;
    //float valor1; 

    //char data_str[6] = " ";
    uint16_t adc_data[6] = {0};
    uint8_t i = 0;


    Configurar_PLL(_50MHZ);  //Confiuracion de velocidad de reloj
    Configura_Reg_ADC();
    Configurar_UART0();
    //printString("3");
    while(1)
    {
        
        /*ADC0_InSeq2(Result); //llamada a la conversion por procesador
        valor=(float)(((Result[0]))*3.3)/4096;
        valor1=(float)(((Result[1]))*3.3)/4096;
        */
       ///////////////////// leer ADC
        Leer_ADC(adc_data);
        funcion(adc_data[0]);
        ///////////////////// enviar datos
        for (i = 0; i < 6; i++)
        {
            //sprintf(data_str, "adc%u:%04u, ", i, adc_data[i]);
            //printString(data_str);
        }

        //sprintf(data_str, "min:0, max:4095\r\n");
        //printString(data_str);

        ///////////////////// retraso
        //delay_ms(10);

    }

}

void funcion(u_int16_t num){
char Cadena[5];
int i = 0;
while(num > 0){
    Cadena[i] = (char)(num % 10);
    num = (int) (num/10);
    i++;
}
char Numero[5] = {0,0,0,0,0};
for(int j =  0; j == i;j++){
    Numero[j] = Cadena[i - j];
}
printString(Numero);
}