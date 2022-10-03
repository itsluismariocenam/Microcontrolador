/*
 * File:   SAPCG.c
 * Author: Miguel, Ilse, Alexis
 * 
 * Programa que emplea un microcontrolador PIC18F4553 como esclavo para ejecutar 
 * los comandos enviados por comunicaci?n serial mediante un programa de LabVIEW
 * y realizar los movimientos necesarios para hacer funcionar el Sistema Autom?-
 * tico de Pesaje de Cilindros de Gas del CENAM.
 * 
 * Created on marzo - junio de 2022
 */

#include <xc.h>
#include <pic18f4553.h>
#include <stdint.h>
#include <stdbool.h>

// User constants and global variable declarations
#define _XTAL_FREQ          20000000UL
#define Sensor_Freno_Abajo  PORTAbits.RA5 // RV6
#define Sensor_Freno_Arriba PORTAbits.RA4 // RV5
#define Sensor_P2           PORTAbits.RA0 // RV1 : Posici?n 2
#define Sensor_P3_Arriba    PORTAbits.RA1 // RV2 : Posici?n 3 Arriba
#define Sensor_P3_Abajo     PORTAbits.RA2 // RV4 : Posici?n 3 Abajo
#define Sensor_Altura       PORTAbits.RA3 // 
#define Sensor_Altura2      PORTBbits.RB0 // A6

//Macros para el control de motores
//Pines del Puerto D asociados al control del husillo(Moviemiento del plato)
#define HUSILLO_ENABLE      PORTDbits.RD3
#define HUSILLO_DIRECCION   PORTDbits.RD2
#define HUSILLO_RELOJ       PORTDbits.RD1
#define HUSILLO_RESOLUCION  PORTDbits.RD0

//Pines del Puerto D asociados al control del freno(Cremallera con piñon)
#define FRENO_ENABLE        PORTDbits.RD7
#define FRENO_DIRECCION     PORTDbits.RD6
#define FRENO_RELOJ         PORTDbits.RD5
#define FRENO_RESOLUCION    PORTDbits.RD4

//Comandos de control de los motores
#define HORARIO     1
#define ANTIHORARIO 0
#define ABAJO       1
#define ARRIBA      0

// Declaraci?n de variables globales
int Status_Freno_Ab = 0;
int Status_Freno_Ar = 0;

int Status_Freno = 0; // 0 OFF Abajo. 1 ON Arriba
int Status_P2 = 0;
int Status_P3_Ar = 0;
int Status_P3_Ab = 0;
int Status_Al = 0;
int Status_Al2 = 0;
int Error = 0;

int comando_ejecutado = 0; // Variable que guarda el valor entero del comando serial recibido y ejecutado por el PIC
int comando_recibido = 0;
int comando_enviado = 0;
char comando[6];            //Variable que almacena las instrucciones del puerto Serial

// Definition and declaration of functions 
    // PIC18F4553 Ajustes de los Configuration Bits y de los Puertos
void init_config() {
    // 'C' source line config statements
    // CONFIG1L
    #pragma config PLLDIV = 5       // PLL Prescaler Selection bits (Divide by 5 (20 MHz oscillator input))
    #pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
    #pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

    // CONFIG1H
    #pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
    #pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
    #pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

    // CONFIG2L
    #pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
    #pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
    #pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting)
    #pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

    // CONFIG2H
    #pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
    #pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

    // CONFIG3H
    #pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
    #pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
    #pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
    #pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

    // CONFIG4L
    #pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
    #pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
    #pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
    #pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

    // CONFIG5L
    #pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
    #pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
    #pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
    #pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

    // CONFIG5H
    #pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
    #pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

    // CONFIG6L
    #pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
    #pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
    #pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
    #pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

    // CONFIG6H
    #pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
    #pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
    #pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

    // CONFIG7L
    #pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
    #pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
    #pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
    #pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

    // CONFIG7H
    #pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

    // Configuraci?n de Puertos
    ADCON0 = 0;
    ADCON1 = 0b00001111;         // PORTA pins set as Digital I/O. 
    ADCON2 = 0b00001111;         // PORTB as DIGITAL I/O
    
    TRISA = 0xFF;   // Configure PORTA as inputs
    TRISB = 0x00000001;      // Configure PORTB as outputs
    TRISC = 0; // Configure PORTC as outputs
    TRISD = 0; // Configure PORTD as outputs
    
    PORTA=0; // Inicializa en 0 todos los puertos
    PORTB=0;
    PORTC=0;
    PORTD=0;
    
//---------------------------------CONFIGURACION SERIAL GENERAL---------------------------------
//  Descripcion de los pines en la pagina 237 del Datasheet -> EUSART
//  Documentacion de referencia -> https://deepbluembedded.com/uart-pic-microcontroller-tutorial/
    
    TXSTAbits.BRGH = 1;     //Habilitar velocidades altas para el BRG(Baud Rate Generator)
    BAUDCONbits.BRG16 = 0;  //Establecer la resolucion del generador a 8 bits
    SPBRG = 0b10000001;     //Genera un baud rate de 9600
    
    TXSTAbits.SYNC = 0;     //Modo de comunicacion asincrono
    RCSTAbits.SPEN = 1;     //Habilita los pines RC6 y RC7 de puerto C para comunicacion Serial
    
    TRISCbits.RC6 = 1;      //Configuracion del pin RC6
    TRISCbits.RC7 = 1;      //Configuracion del pin RC6
    
    TXSTAbits.TXEN = 1;     //Habilitar la transmision de datos
    RCSTAbits.CREN = 1;     //Recepcion continua de datos
    
}

void read_FAb(){
    Status_Freno_Ab = Sensor_Freno_Abajo; 
}
void read_FAr(){
    Status_Freno_Ar = Sensor_Freno_Arriba; 
}
void read_P2(){
    Status_P2 = Sensor_P2; 
}
void read_P3_Ar(){
    Status_P3_Ar = Sensor_P3_Arriba; 
}
void read_P3_Ab(){
    Status_P3_Ab = Sensor_P3_Abajo; 
}
void read_Al(){
    Status_Al = Sensor_Altura; 
}

void read_Al2(){
    Status_Al2 = Sensor_Altura2; 
}

int get_FAb(Status_Freno_Ab){
    return Status_Freno_Ab; 
}
int get_FAr(Status_Freno_Ar){
    return Status_Freno_Ar; 
}
int get_P2(Status_P2){
    return Status_P2; 
}
int get_P3_Ar(Status_P3_Ar){
    return Status_P3_Ar; 
}
int get_P3_Ab(Status_P3_Ab){
    return Status_P3_Ab; 
}
int get_Al(Status_Al){
    return Status_Al; 
}

int get_A2(Status_Al2){
    return Status_Al2; 
}

int get_Status_Freno (){
    if ((get_FAb(Status_Freno_Ab)==0) &&(get_FAr(Status_Freno_Ar)==1))//
    {
        Status_Freno=1; //Freno On
    }
    else if((get_FAb(Status_Freno_Ab)==1) &&(get_FAr(Status_Freno_Ar)==0))
    {
        Status_Freno=0; //Freno off
    }
    return Status_Freno;
}

//Funcion para transmitir los datos por el puerto serial
//Manda un caracter de 8 bits(1 byte) cada vez que se llama
void transmitir(char data){
    //Esperar hasta que el registro para transmisión de datos este vacio
    while(!TRMT);
    TXREG = data; //Transmitir el caracter pasado a la funcion
}

//Funcion para recibir datos a traves del puerto Serial
//Recibe un caracter de 8 bits(1 byte) cada vez que se llama
char recibir(){
    // Esperar hasta que el registro de la bandera de comunicacion serial
    // se active en 1 (señal de que recibió un dato)
    while(!RCIF); 
    RCIF = 0; // Reiniciar la bandera a 0
    return RCREG; // Regresar el valor recibido en el registro de datos de
                  // comunicación serial
}

// Funcion que manda llamar la funcion recibir() para leer caracter por caracter 
// el comando guardado en el registro de comunicación serial RCREG desde que se detecta
// el caracter de inicio STX (0x02) hasta que se detecta el de final ETX (0x03)
void leer_comando(){
    char pos = 0;
    if(recibir() == 0x02){
        do{
           comando[pos] = recibir();
           pos++;
        } while(recibir() != 0x03);
    }
}

//Funcion para mover el freno en incrementos de un pulso cada vez que se llama
void mover_freno(char direccion){
    FRENO_ENABLE = 1;
    FRENO_DIRECCION = direccion;  
    /*Generador de señal de reloj para el freno
    * __delay_ms(x)... donde x corresponde al tiempo de un semiciclo
    * x = 4.16/RPM --> Full x = 2.08/RPM --> Half
    */ 
    FRENO_RELOJ = 0;
    __delay_ms(0.83);
    FRENO_RELOJ = 1;
    __delay_ms(0.83);
    FRENO_ENABLE = 0;
}

//Funcion para mover el husillo en incrementos de un pulso cada vez que se llama
void mover_husillo(char direccion){
    HUSILLO_ENABLE = 1;
    HUSILLO_DIRECCION = direccion;
    /*Generador de señal de reloj para el husillo
    * __delay_ms(x)... donde x corresponde al tiempo de un semiciclo
    * x = 20.83/RPM --> Full x = 10.42/RPM --> Half
    */ 
    HUSILLO_RELOJ = 1;
    __delay_ms(5.21);
    HUSILLO_RELOJ = 0;
    __delay_ms(5.21);
    //HUSILLO_ENABLE = 0;
}

// Funci?n que mueve el motor del freno para liberarlo, se ejecuta hasta que se desactive el freno
void libera_freno(){
    do{
        read_FAr(); // Lee valores de los sensores Freno Arriba y Freno Abajo
        read_FAb();
            if (get_Status_Freno(Status_Freno)==1){ // Si el freno est? activado, desact?valo
                    mover_freno(ABAJO);
           }
        } while(get_Status_Freno(Status_Freno)==1); // Repetir hasta que el freno se desactive
}

// Funci?n que mueve el motor del freno para activarlo
void activa_freno(){
    do{
        read_FAr(); // Lee valores de los sensores Freno Arriba y Freno Abajo
        read_FAb();
            if (get_Status_Freno(Status_Freno)==0){ // Si el freno est? desactivado, act?valo     
                mover_freno(ARRIBA);   
            }
        } while(get_Status_Freno(Status_Freno)==0); // Repetir hasta que el freno se active
}

// Funci?n que revisa valores de los sensores y lleva el plato 1 a home 
void barrido_inicial(void) {
    int Plato1_Home = 0; // Variable local para indicar que plato 1 est? en home y se puede pasar a lectura de valores de LabView, cuando est? en HIGH.
    do {
        read_Al();
        if (get_Al(Status_Al) == 0){ // Si la placa est? arriba o a la mitad
            read_P2();      // Lee valores de los sensores P2, P3 Arriba y P3 Abajo
            read_P3_Ar();
            read_P3_Ab();
            // Si alg?n sensor de los anteriores est? encendido entonces la placa est? arriba
            if ((get_P2(Status_P2)==1)||(get_P3_Ar(Status_P3_Ar)==1)||(get_P3_Ab(Status_P3_Ab)==1)) { 
                libera_freno(); // Gira motor para desactivar freno
                read_P2();      // Lee valores de los sensores P2, P3 Arriba y P3 Abajo
                read_P3_Ar();
                read_P3_Ab();
                if ((get_P3_Ab(Status_P3_Ab)==1)&&(get_P3_Ar(Status_P3_Ar)==0)&&(get_P2(Status_P2)==0)) {
                    // Plato 1 est? en home, ir a leer valores de LabView
                    Plato1_Home = 1;
                } else if ((get_P3_Ab(Status_P3_Ab)==1)&&(get_P3_Ar(Status_P3_Ar)==1)&&(get_P2(Status_P2)==0)) {
                    //Plato 2 en HOME
                    //Girar horario hasta que el plato 1 llegue a home
                    do{
                       mover_husillo(HORARIO);
                    }while(PORTA != 0b00100100);  
                    Plato1_Home = 1;
                } else if ((get_P3_Ab(Status_P3_Ab)==0)&&(get_P3_Ar(Status_P3_Ar)==1)&&(get_P2(Status_P2)==1)) {
                    //Plato3 en HOME
                    //Girar horario hasta que el plato 1 llegue a home
                    do{
                       mover_husillo(HORARIO);  
                    }while(PORTA != 0b00100100);  
                    Plato1_Home = 1;
                } else if ((get_P3_Ab(Status_P3_Ab)==1)&&(get_P3_Ar(Status_P3_Ar)==0)&&(get_P2(Status_P2)==1)) {
                    //Plato4 en HOME
                    //Girar horario hasta que el plato 1 llegue a home
                    do{
                       mover_husillo(HORARIO);
                    }while(PORTA != 0b00100100);  
                    Plato1_Home = 1;
                } else {
                    // Error revisar sensores
                }
            // Si ninguno de los sensores P2, P3 Arriba y P3 Abajo est? encendido, entonces la placa est? en medio. Subirla.
            } else {
                activa_freno(); // Gira motor para activar freno
                mover_husillo(ANTIHORARIO); // Girar motor del husillo antihorario para subir la placa
            }
        } else if(get_Al(Status_Al) == 1){ // Si la placa est? abajo poner freno y subirla
            activa_freno(); // Gira motor para activar freno
            mover_husillo(ANTIHORARIO); //Girar motor del husillo antihorario para subir la placa
        }
    } while (Plato1_Home == 0); // Mientras plato 1 no haya llegado a home repetir (hasta que plato 1 est? en Home)
    // Plato 1 est? en HOME, ir a revisar ID recibido de LabView
    activa_freno();
    return;
} // COMPLEMENTAR CON AVANCES

// Funci?n que recibe el comando serial de ID de LabVIEW al PIC
int ID_receive() {
    // Recibe el comando de identificaci?n serial de LabView al PIC
    leer_comando();
    if(comando[3] == 0x49 && comando[4] == 0x44){
        return 1;
    }
    /*
        0x02 0x30 0x32 0x00 0x49 0x44 0x03   ID
        2    48   50    0   73   68    3
        STX   0    2   NULL  I    D    ETX 
    */
    // Si el comando recibido es el anterior entonces regresa 1
        // return 1;
    // Si no recibe comando alguno o el comando recibido no es el anterior entonces regresa 0
    return 0;
} 

// Funci?n que env?a el comando serial de ID del PIC a LabVIEW
void ID_send() {
    // Env?a el comando de identificaci?n serial del PIC a LabView
    /*
        0x02 0x30 0x32 0x00 0x49 0x31 0x03   ID
        2    48   50    0   73   49    3
        STX   0    2   NULL  I    1    ETX
    */
    char ID[] = {0x02,0x30,0x32,0x00,0x49,0x31,0x03};
    
    for(char i = 0;i <= 6;i++){
        transmitir(ID[i]);
    }
    
}

// Funci?n que a partir de la lectura de los sensores de posici?n P2, P3 Arriba y 
// P3 Abajo regresa un entero que indica el n?mero de plato presente en la 
// posici?n de HOME cuando el plato est? arriba
int revisa_posicion(){
    // Funci?n que realiza un barrido de los sensores y regresa un n?m. entero con el # de plato presente en HOME
    read_P2();      // Lee valores de los sensores P2, P3 Arriba y P3 Abajo
    read_P3_Ar();
    read_P3_Ab();
    if ((get_P3_Ab(Status_P3_Ab)==1)&&(get_P3_Ar(Status_P3_Ar)==0)&&(get_P2(Status_P2)==0)) {
        // Plato 1 est? en HOME
        return 1;
    } else if ((get_P3_Ab(Status_P3_Ab)==1)&&(get_P3_Ar(Status_P3_Ar)==1)&&(get_P2(Status_P2)==0)) {
        // Plato 2 en HOME
        return 2;
        // Ir a leer valores de LabView
    } else if ((get_P3_Ab(Status_P3_Ab)==0)&&(get_P3_Ar(Status_P3_Ar)==1)&&(get_P2(Status_P2)==1)) {
        // Plato 3 en HOME
        return 3;
        // Ir a leer valores de LabView
    } else if ((get_P3_Ab(Status_P3_Ab)==1)&&(get_P3_Ar(Status_P3_Ar)==0)&&(get_P2(Status_P2)==1)) {
        // Plato 4 en HOME
        return 4;
        // Ir a leer valores de LabView
    } else {
        // Error revisar sensores
        return 5;
    }
}

// Funci?n que recibe el # de plato presente en HOME y el plato que se desea
// posicionar en HOME. Regresa 0 si el plato deseado no ha llegado a HOME y
// regresa 1 si el plato deseado ya lleg? a HOME
int envia_posicion(int plato_HOME, int plato_deseado){ 
    // Si el plato presente en HOME coincide con el # de plato deseado en HOME no actives motores y regresa 1.
    if (plato_HOME == plato_deseado){
        return 1; // El plato deseado ya se encuentra en HOME
    } else {
        switch(plato_deseado){
            case 1: // Llevar plato 1 a HOME
                if (plato_HOME == 2) {
                    // Gira 90? CW hasta que sensores lo detecten en HOME
                    mover_husillo(HORARIO);
                } else if (plato_HOME == 3) {
                    // Gira 180? hasta que sensores lo detecten en HOME
                    mover_husillo(HORARIO);
                } else if (plato_HOME == 4) {
                    // Gira 90? CCW hasta que sensores lo detecten en HOME
                    mover_husillo(ANTIHORARIO);
                }
            break;
            case 2: // Llevar plato 2 a HOME
                if (plato_HOME == 1) {
                    // Gira 90? CCW hasta que sensores lo detecten en HOME
                    mover_husillo(ANTIHORARIO);
                } else if (plato_HOME == 3) {
                    // Gira 90? CW hasta que sensores lo detecten en HOME
                    mover_husillo(HORARIO);
                } else if (plato_HOME == 4) {
                    // Gira 180? hasta que sensores lo detecten en HOME
                    mover_husillo(HORARIO);
                }
            break;
            case 3: // Llevar plato 3 a HOME
                if (plato_HOME == 1) {
                    // Gira 180? hasta que sensores lo detecten en HOME
                    mover_husillo(HORARIO);
                } else if (plato_HOME == 2) {
                    // Gira 90? CCW hasta que sensores lo detecten en HOME
                    mover_husillo(ANTIHORARIO);
                } else if (plato_HOME == 4) {
                    // Gira 90? CW hasta que sensores lo detecten en HOME
                    mover_husillo(HORARIO);
                }
            break;
            case 4: // Llevar plato 4 a HOME
                if (plato_HOME == 1) {
                    // Gira 90? CW hasta que sensores lo detecten en HOME
                    mover_husillo(HORARIO);
                } else if (plato_HOME == 2) {
                    // Gira 180? hasta que sensores lo detecten en HOME
                    mover_husillo(HORARIO);
                } else if (plato_HOME == 3) {
                    // Gira 90? CCW hasta que sensores lo detecten en HOME
                    mover_husillo(ANTIHORARIO);
                }
            break;
        }
        return 0; // Regresa 0 si el plato deseado a?n no llega a HOME
    }
} // EDITAR, AGREGAR LOOPS

// Funcion que recibe el comando serial de LabVIEW al PIC y regresa un entero
// dependiendo de su valor
char serial_receive(){
     
    leer_comando();
    switch(comando[3]){
            //Mover el plato a una posicion especificada
            //0x50 = 'P'
            case 0x50:
                switch (comando[4]){
                    //Posicion 1
                    case 0x31:
                        return 1;
                        
                    //Posicion 2
                    case 0x32:
                        return 2;

                    //Posicion 3
                    case 0x33:
                        return 3;

                    //Posicion 4
                    case 0x34:
                        return 4;
                }
                
            //Mover el plato a Home
            //0x48 = 'H'
            case 0x48:
                return 5;

            //Ascenso del plato
            //0x41 = 'A'
            case 0x41:
                return 6;

            //Descenso del plato
            //0x44 = 'D'
            case 0x44:
                return 7;

            //Modo manual
            //0x4D = 'M'
            case 0x4D:
                switch(comando[4]){
                    //Manual detener
                    case 0x30:
                        return 8;

                    //Manual rotar
                    case 0x31:
                        return 9;

                    //Manual subir freno
                    case 0x32:
                        return 10;

                    //Manual bajar freno
                    case 0x33:
                        return 11;
                    
                    //Manual ascenso
                    case 0x34:
                        return 12;
                    
                    //Manual descenso
                    case 0x35:
                        return 13;    
                }

            default:
                return 0;
        }
} 

// Funcion que recibe el # de comando serial recibido de LabVIEW al PIC y ejecuta 
// las acciones necesarias para mover los actuadores seg?n lo requerido
char ejecucion_comandos(char cmd_recibido){
    // Recibe comandos seriales de LabView para actuadores
    int plato = 0;
    int pos_lista = 0;
    switch(cmd_recibido){
        // Comandos Modo Autom?tico
        
        case 1: // Comando para enviar a posici?n 1
            /*
            0x02 0x30 0x32 0x00 0x50 0x31 0x03   posicion 1
            2    48   50    0   80   49    3
            STX   0    2   NULL  P    1    ETX
            */
            libera_freno(); // Asegurar que el freno est? liberado (abajo)
            do {
                plato = revisa_posicion(); // Detecta el # de plato en HOME 
                pos_lista = envia_posicion(plato,1);// Rotar plato 1 a HOME desde la posici?n detectada 
            } while(pos_lista != 1); // Repetir hasta que pos_lista sea igual a 1, pues el plato deseado ha llegado a HOME
            activa_freno(); // Asegurar que el freno est? puesto (arriba))
            return 1; // Listo
        break;
        case 2: // Comando para enviar a posici?n 2
            /*
            0x02 0x30 0x32 0x00 0x50 0x32 0x03   posicion 2
            2    48   50    0   80   50    3
            STX   0    2   NULL  P    2    ETX
            */
            libera_freno(); // Asegurar que el freno est? liberado (abajo)
            do {
                plato = revisa_posicion(); // Detecta el # de plato en HOME 
                pos_lista = envia_posicion(plato,2);// Rotar plato 2 a HOME desde la posici?n detectada 
            } while(pos_lista != 1); // Repetir hasta que pos_lista sea igual a 1, pues el plato deseado ha llegado a HOME
            activa_freno(); // Asegurar que el freno est? puesto (arriba))
            return 2; // Listo
        break;
        case 3: // Comando para enviar a posici?n 3
            /*
            0x02 0x30 0x32 0x00 0x50 0x33 0x03   posicion 3
            2    48   50    0   80   51    3
            STX   0    2   NULL  P    3    ETX
            */
            libera_freno(); // Asegurar que el freno est? liberado (abajo)
            do {
                plato = revisa_posicion(); // Detecta el # de plato en HOME 
                pos_lista = envia_posicion(plato,3);// Rotar plato 3 a HOME desde la posici?n detectada 
            } while(pos_lista != 1); // Repetir hasta que pos_lista sea igual a 1, pues el plato deseado ha llegado a HOME
            activa_freno(); // Asegurar que el freno est? puesto (arriba))
            return 3; // Listo
        break;
        case 4: // Comando para enviar a posici?n 4
            /*
            0x02 0x30 0x32 0x00 0x50 0x34 0x03   posicion 4
            2    48   50    0   80   52    3
            STX   0    2   NULL  P    4    ETX
            */
            libera_freno(); // Asegurar que el freno est? liberado (abajo)
            do {
                plato = revisa_posicion(); // Detecta el # de plato en HOME 
                pos_lista = envia_posicion(plato,4);// Rotar plato 4 a HOME desde la posici?n detectada 
            } while(pos_lista != 1); // Repetir hasta que pos_lista sea igual a 1, pues el plato deseado ha llegado a HOME
            activa_freno(); // Asegurar que el freno est? puesto (arriba))
            return 4; // Listo
        break;
        case 5: // Comando para enviar a home
            /*
            0x02 0x30 0x32 0x00 0x48 0x31 0x03   home
            2    48   50    0   72   49    3
            STX   0    2   NULL  H    1    ETX
            */
            libera_freno(); // Asegurar que el freno est? liberado (abajo)
            do {
                plato = revisa_posicion(); // Detecta el # de plato en HOME 
                pos_lista = envia_posicion(plato,1);// Rotar plato 1 a HOME desde la posici?n detectada 
            } while(pos_lista != 1); // Repetir hasta que pos_lista sea igual a 1, pues el plato deseado ha llegado a HOME
            activa_freno(); // Asegurar que el freno est? puesto (arriba))
            return 5; // Listo
        break;
        case 6: // Comando para ascenso de plato
            /*
            0x02 0x30 0x32 0x00 0x41 0x31 0x03   ascenso
            2    48   50    0   64   49    3
            STX   0    2   NULL  A    1    ETX
            */
            activa_freno(); // Asegurar que el freno est? puesto (arriba))
            do{
               mover_husillo(ANTIHORARIO); //Girar motor del husillo antihorario para subir la placa 
            // Girar el motor para subir placa mientras que cualquiera de los sensores de posici?n no detecte algo (0) y el de altura detecte la placa abajo (1)
            } while( (get_Al(Status_Al) == 1)||(get_P3_Ab(Status_P3_Ab)==1)||(get_P3_Ar(Status_P3_Ar)==1)||(get_P2(Status_P2)==1) ); 
            
            return 6;
        break;
        case 7: // Comando para descenso de plato
            /*
            0x02 0x30 0x32 0x00 0x44 0x31 0x03   descenso
            2    48   50    0   68   49    3
            STX   0    2   NULL  D    1    ETX
            */
            activa_freno(); // Asegurar que el freno est? puesto (arriba))
            do {
               mover_husillo(HORARIO); //Girar motor del husillo horario para bajar la placa 
            // Girar el motor para bajar placa mientras que cualquiera de los sensores de posici?n detecte algo (1) o el de altura no detecte nada (0)
            } while( (get_Al(Status_Al) == 0)||(get_P3_Ab(Status_P3_Ab)==1)||(get_P3_Ar(Status_P3_Ar)==1)||(get_P2(Status_P2)==1) ); 
            return 7;
        break;
        
        // Comandos Modo Manual
        
        case 8: // Comando detener
            // Deshabilita el motor del husillo y del freno
            // S?lo se realiza una vez
            /*
            0x02 0x30 0x32 0x00 0x4D 0x30 0x03   Manual Detener
            2    48   50    0   77   48    3
            STX   0    2   NULL  M    0    ETX
            */
            return 8;
        break;
        case 9: // Comando rotar
            // Rotar indefinidamente hasta que reciba un detener
            // Interrupci?n Necesaria por el comando detener
            /*
            0x02 0x30 0x32 0x00 0x4D 0x31 0x03   Manual Rotar
            2    48   50    0   77   49    3
            STX   0    2   NULL  M    1    ETX
            */
            return 9;
        break;
        case 10: // Comando subir freno
            // Hasta que condiciones de sensado se cumplan
            /*
            0x02 0x30 0x32 0x00 0x4D 0x32 0x03   Manual subir freno
            2    48   50    0   77   50    3
            STX   0    2   NULL  M    2    ETX
            */
            activa_freno();
            return 10;
        break;
        case 11: // Comando bajar freno
            /*
            0x02 0x30 0x32 0x00 0x4D 0x33 0x03   Manual bajar freno
            2    48   50    0   77   51    3
            STX   0    2   NULL  M    3    ETX
            */
            libera_freno();
            return 11;
        break;
        case 12: // Comando ascenso
            // Hasta cumplir condiciones de sensado
            /*
            0x02 0x30 0x32 0x00 0x4D 0x34 0x03   Manual ascenso
            2    48   50    0   77   52    3
            STX   0    2   NULL  M    4    ETX
            */
            activa_freno(); // Asegurar que el freno est? arriba
            // Corroborar hasta que llegue al tope
            return 12;
        break;
        case 13: // Comando descenso
            // Hasta cumplir cond sensado
            /*
            0x02 0x30 0x32 0x00 0x4D 0x35 0x03   Manual descenso
            2    48   50    0   77   53    3
            STX   0    2   NULL  M    5    ETX
            */
            activa_freno(); // Asegurar que el freno est? arriba
            // Corroborar hasta que llegue a la balanza
            return 13;
        break;
        default:
            return 0; // Error
        break;
    }    
} // EDITAR, AGREGAR LOOPS

// Funcion que recibe el # de comando ejecutado y env?a el comando serial 
// correspondiente del PIC a LabVIEW para su confirmaci?n
void serial_send(char cmd_enviado){
    // Env?a respuestas en comandos seriales: OK o ERROR
    char respuesta[] = {0x02,0x30,0x32,0x00,0x00,0x00,0x03};
    switch(cmd_enviado){
        // Respuestas Modo Autom?tico
            // Falta X1,X2,X3 
        
        case 1: // Comando para avisar que se envi? a posici?n 1 (OK)
            /*
            0x02 0x30 0x32 0x00 0x50 0x31 0x03   posicion 1 Ok
            2    48   50    0   80   49    3
            STX   0    2   NULL  P    1    ETX
            */
            respuesta[4] = 0x50;
            respuesta[5] = 0x31;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 2: // Comando para avisar que se envi? a posici?n 2 (OK)
            /*
            0x02 0x30 0x32 0x00 0x50 0x32 0x03   posicion 2 Ok
            2    48   50    0   80   50    3
            STX   0    2   NULL  P    2    ETX
            */
            respuesta[4] = 0x50;
            respuesta[5] = 0x32;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 3: // Comando para avisar que se envi? a posici?n 3 (OK)
            /*
            0x02 0x30 0x32 0x00 0x50 0x33 0x03   posicion 3 Ok
            2    48   50    0   80   51    3
            STX   0    2   NULL  P    3    ETX
            */
            respuesta[4] = 0x50;
            respuesta[5] = 0x33;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 4: // Comando para avisar que se envi? a posici?n 4 (OK)
            /*
            0x02 0x30 0x32 0x00 0x50 0x34 0x03   posicion 4 Ok
            2    48   50    0   80   52    3
            STX   0    2   NULL  P    4    ETX
            */
            respuesta[4] = 0x50;
            respuesta[5] = 0x34;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 5: // Comando para avisar que se envia a Home (OK)
            /*
            0x02 0x30 0x32 0x00 0x48 0x30 0x03   home Ok
            2    48   50    0   72   48    3
            STX   0    2   NULL  H    0    ETX
            */
            respuesta[4] = 0x48;
            respuesta[5] = 0x30;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 6: // Comando para confirmar ascenso de plato (OK)
            /*
            0x02 0x30 0x32 0x00 0x41 0x30 0x03   ascenso Ok
            2    48   50    0   64   48    3
            STX   0    2   NULL  A    0    ETX
            */
            respuesta[4] = 0x41;
            respuesta[5] = 0x30;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 7: // Comando para confirmar descenso de plato (OK)
            /*
            0x02 0x30 0x32 0x00 0x44 0x31 0x03   descenso Ok
            2    48   50    0   68   48    3
            STX   0    2   NULL  D    0    ETX
            */
            respuesta[4] = 0x44;
            respuesta[5] = 0x30;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 8: // Comando para avisar que NO se envi? a Home (ERROR)
            /*
            0x02 0x30 0x32 0x00 0x48 0x65 0x03   home Err
            2    48   50    0   72  101    3
            STX   0    2   NULL  H    e    ETX
            */
            respuesta[4] = 0x48;
            respuesta[5] = 0x65;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        
        // Respuestas Modo Manual
        
        case 9: // Comando para confirmar subir freno (OK)
            /*
            0x02 0x30 0x32 0x00 0x46 0x31 0x03   subir freno ok
            2    48   50    0   70   49    3
            STX   0    2   NULL  F    1    ETX
            */
            respuesta[4] = 0x46;
            respuesta[5] = 0x31;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 10: // Comando para confirmar bajar freno (OK)
            /*
            0x02 0x30 0x32 0x00 0x46 0x32 0x03   bajar freno ok
            2    48   50    0   70   49    3
            STX   0    2   NULL  F    2    ETX
            */
            respuesta[4] = 0x46;
            respuesta[5] = 0x32;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 11: // Comando para confirmar ascenso (OK)
            /*
            0x02 0x30 0x32 0x00 0x41 0x30 0x03   ascenso Ok
            2    48   50    0   64   48    3
            STX   0    2   NULL  A    0    ETX
            */
            respuesta[4] = 0x41;
            respuesta[5] = 0x30;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
        case 12: // Comando para confirmar descenso (OK)
            /*
            0x02 0x30 0x32 0x00 0x41 0x30 0x03  descenso Ok
            2    48   50   0    64   48   3
            STX  0    2    NULL A    0    ETX
             */
            respuesta[4] = 0x44;
            respuesta[5] = 0x30;
            for(char i = 0;i <= 6;i++){
                transmitir(respuesta[i]);
            }
        break;
    }     
} 

// Inicio del programa principal
void main(void) {
    init_config(); // Ajuste de "Configuration Bits" del PIC y de sus puertos digitales I/O
    //barrido_inicial(); // Se realiza un barrido inicial del sistema para posicionar el plato 1 de la balanza en Home
    //activa_freno();
    do{
        mover_husillo(HORARIO);
        }while(1);
    
    do { // Ejecutar hasta que se verifique la recepci?n del comando de comunicaci?n serial emitido por LabView  
        ID_receive(); // Cuando se reciba el comando se regresar? un 1     
    } while (ID_receive()==0); // Repetir mientras que ID_receive sea 0
    ID_send(); // Enviar comando de regreso para validar comunicaci?n exitosa
    
    do {
        
        char comando_recibido = serial_receive(); 
        char comando_ejecutado = ejecucion_comandos(comando_recibido); // Recibe un comando serial, ejec?talo y guarda su valor de retorno en la variable "comando_ejecutado"
        serial_send(comando_ejecutado); // Env?a a LabVIEW o RealTerm el comando serial que representa el ejecutado por el PIC en la l?nea anterior 
        
    } while(1); // Repetir siempre o a menos que se active alguna interrupcion
    
    return;
}