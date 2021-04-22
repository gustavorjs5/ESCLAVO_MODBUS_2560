
//Incluye la librería del protocolo Modbus
#include <ModbusRtu.h>
#include <EEPROM.h>

int dd;
Modbus slave(ID,2,39);


#define OUTDIG1  49 
#define OUTDIG2  48
#define OUTDIG3 47
#define OUTDIG4 46
#define OUTDIG5 45
#define OUTDIG6 44
#define OUTDIG7 43
#define OUTDIG8 42
#define OUTDIG9 19
#define OUTDIG10 18
#define OUTDIG11 38
#define OUTDIG12 41 
#define OUTDIG13 40
#define OUTDIG14 37
#define OUTDIG15 36
#define OUTDIG16 35


#define EE_DIRECCION_MF_RTU 0
#define EE_RATE_BAUDIOS_RS485 2

        
#define ON    0
#define OFF   1
#define IN1  8
#define IN2  7
#define IN3  6
#define IN4  3
#define IN5  2
#define IN6  5
#define IN7  4
#define IN8  A0

#define RELE_1 10 
#define RELE_2 11
#define RELE_3 12
#define RELE_4 32

#define ID   5
#define JUMPER1  52   
#define JUMPER2  51 
#define JUMPER3  50
#define MD0  9 
#define MD1  53
#define LED_AUX A13
#define SDA 20
#define LLAVE_PROGRAMACION 21 //SDL
#define RS485TxEnablePin 39


int8_t state = 0;
unsigned long tempus;
bool bG_PrimeraEntrada;  
unsigned char cG_Recibir[10];
unsigned char DIRECCION_MF_RTU;
unsigned char RATE_BAUDIOS_RS485;
word Baudios;
uint16_t au16data[37]; //La tabla de registros que se desea compartir por la red

//CANALES DE FRECUENCIA

  const byte CANAL14_430MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X14, 0X40}; 
  const byte CANAL15_431MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X15, 0X40};  
  const byte CANAL16_432MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X16, 0X40}; 
  const byte CANAL17_433MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X17, 0X40}; 
  const byte CANAL18_434MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X18, 0X40}; 
  const byte CANAL19_435MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X19, 0X40}; 
  const byte CANAL20_436MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X1A, 0X40}; 
  const byte CANAL21_437MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X1B, 0X40}; 
  const size_t LONGITUD_TRAMA_MODULO = sizeof(CANAL14_430MHZ) / sizeof(CANAL14_430MHZ[0]);

/*********************************************************
  Configuración del programa
*********************************************************/
void setup() 
{

 

  pinMode(MD0, OUTPUT);// MDO
  pinMode(MD1, OUTPUT);// MD1
  pinMode(IN1, INPUT_PULLUP); //ENTRADA IN1
  pinMode(IN2, INPUT_PULLUP); //ENTRADA IN2
  pinMode(IN3, INPUT_PULLUP); //ENTRADA IN1
  pinMode(IN4, INPUT_PULLUP); //ENTRADA IN2
  pinMode(IN5, INPUT_PULLUP); //ENTRADA IN1
  pinMode(IN6, INPUT_PULLUP); //ENTRADA IN2
  pinMode(IN7, INPUT_PULLUP); //ENTRADA IN1
  pinMode(IN8, INPUT_PULLUP); //ENTRADA IN2

  pinMode(SDA, INPUT); //SDA
  pinMode(LLAVE_PROGRAMACION, INPUT); //SDL

//SALIDAS MEDIANTE RELES
  pinMode(RELE_1, OUTPUT);//SALIDA 1
  pinMode(RELE_2, OUTPUT);//SALIDA 2
  pinMode(RELE_3, OUTPUT);//SALIDA 3
  pinMode(RELE_4, OUTPUT);//SALIDA 4

//SALIDAS DIGITALES
  pinMode(OUTDIG1, OUTPUT);//SALIDA 1
  pinMode(OUTDIG2, OUTPUT);//SALIDA 2
  pinMode(OUTDIG3, OUTPUT);//SALIDA 3
  pinMode(OUTDIG4, OUTPUT);//SALIDA 4
  pinMode(OUTDIG5, OUTPUT);//SALIDA 5
  pinMode(OUTDIG6, OUTPUT);//SALIDA 6
  pinMode(OUTDIG7, OUTPUT);//SALIDA 7
  pinMode(OUTDIG8, OUTPUT);//SALIDA 8
  pinMode(OUTDIG9, OUTPUT);//SALIDA 9
  pinMode(OUTDIG10, OUTPUT);//SALIDA 10
  pinMode(OUTDIG11, OUTPUT);//SALIDA 11
  pinMode(OUTDIG12, OUTPUT);//SALIDA 12
  pinMode(OUTDIG13, OUTPUT);//SALIDA 13
  pinMode(OUTDIG14, OUTPUT);//SALIDA 14
  pinMode(OUTDIG15, OUTPUT);//SALIDA 15
  pinMode(OUTDIG16, OUTPUT);//SALIDA 16
  pinMode(RS485TxEnablePin, OUTPUT);//pinrs485
  pinMode(LED_AUX, OUTPUT);//LED AUXILIAR
  pinMode(JUMPER1, INPUT_PULLUP );  //PIN CAMBIO DE FRECUENCIA
  pinMode(JUMPER2, INPUT_PULLUP );  //PIN CAMBIO DE FRECUENCIA
  pinMode(JUMPER3, INPUT_PULLUP );  //PIN CAMBIO DE FRECUENCIA
  //Serial3.begin(9600);
  Serial2.begin(9600);
  Serial.begin(9600);
  digitalWrite(10, LOW );
  digitalWrite(11, LOW );
  digitalWrite(12, LOW );
  digitalWrite ( MD0, HIGH);
  digitalWrite ( MD1, HIGH);
  delay(1000);
  CONFIG_MODULO();
  delay(1000);
  digitalWrite ( MD0, LOW);
  digitalWrite ( MD1, LOW);
  delay(500);

  tempus = millis() + 100; //Guarda el tiempo actual + 100ms
  digitalWrite(LED_AUX, HIGH ); //Prende el led del pin 13 (el de la placa)       
  
  bG_PrimeraEntrada = true; //PRIMERA ENTRADA PARA CONFIGURAR LA ID DEL EQUIPO MF RTU
  InicializarVariables();
  //slave.begin(9600);
  
  if (digitalRead (SDA) == LOW )
    {
       Serial.println ("PUERTO RS485");
      slave.setPORT(3);     
      slave.begin(Baudios); //Abre la comunicación como esclavo
     
    }else { 
    slave.setPORT(2);      
      slave.begin(9600); //Abre la comunicación como esclavo 
       Serial.println ("PUERTO RF");
}

}
/*********************************************************
  Inicio del programa
*********************************************************/
void loop()
{

 if(digitalRead(LLAVE_PROGRAMACION) == OFF)
  {
   if(bG_PrimeraEntrada == true)
   
    {  
      bG_PrimeraEntrada = false;
      InicializarVariables();
      slave.setID(DIRECCION_MF_RTU);
    }

  io_poll();
  
   //Serial.println ("CONSULTA");
   state = slave.poll( au16data, 37); //Parámetros: Tabla de registros para el intercambio de info, Tamaño de la tabla de registros
  
  //Devuelve 0 si no hay pedido de datos
  //Devuelve 1 al 4 si hubo error de comunicación
  //Devuelve mas de 4 si se procesó correctamente el pedido

  if (state > 4) 
  
  { //Si es mayor a 4 = el pedido fué correcto
    tempus = millis() + 50; //Tiempo actual + 50ms
    digitalWrite(LED_AUX, HIGH);//Prende el led
  }
  if (millis() > tempus) digitalWrite(LED_AUX, LOW );//Apaga el led 50ms después
 
  
  }
  else 
  {

 Programacion();
 bG_PrimeraEntrada = true;
 digitalWrite(LED_AUX, LOW);
 }  

}

/*********************************************************
  Enlaza la tabla de registros con los pines
*********************************************************/
void io_poll() 
{
 
  
  bitWrite( au16data[0], 0, digitalRead( IN1 )); //Lee el pin A2 de Arduino y lo guarda en el bit 0 de la variable au16data[0]
  bitWrite( au16data[0], 1, digitalRead( IN2 )); //Lee el pin A3 de Arduino y lo guarda en el bit 1 de la variable au16data[0]
  bitWrite( au16data[0], 2, digitalRead( IN3 )); //Lee el pin A2 de Arduino y lo guarda en el bit 0 de la variable au16data[0]
  bitWrite( au16data[0], 3, digitalRead( IN4 )); //Lee el pin A3 de Arduino y lo guarda en el bit 1 de la variable au16data[0]
  bitWrite( au16data[0], 4, digitalRead( IN5 )); //Lee el pin A2 de Arduino y lo guarda en el bit 0 de la variable au16data[0]
  bitWrite( au16data[0], 5, digitalRead( IN6 )); //Lee el pin A3 de Arduino y lo guarda en el bit 1 de la variable au16data[0]
  bitWrite( au16data[0], 6, digitalRead( IN7 )); //Lee el pin A2 de Arduino y lo guarda en el bit 0 de la variable au16data[0]
  bitWrite( au16data[0], 7, digitalRead( IN8 )); //Lee el pin A3 de Arduino y lo guarda en el bit 1 de la variable au16data[0]
  
 

  //SALIDAS RELES


  if ( au16data[1]== 1)
       { 
        digitalWrite( RELE_1, HIGH);
       }
       else{ 
       digitalWrite( RELE_1, LOW);
       }
if ( au16data[2]==1)
       { 
        digitalWrite( RELE_2, HIGH);
       }      
        else {
        digitalWrite(RELE_2, LOW);
       }
if ( au16data[3]== 1)
       { 
        digitalWrite(RELE_3, HIGH);
       }       
        else {
        digitalWrite(RELE_3, LOW);
       }
if ( au16data[4]== 1)
       { 
        digitalWrite(RELE_4, HIGH);
       }
       else {        
        digitalWrite(RELE_4, LOW);
       }
//SALIDAS DIGITALES

if ( au16data[5]== 1)
       { 
        digitalWrite( OUTDIG1, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG1, LOW);
       }
if ( au16data[6]== 1)
       { 
        digitalWrite( OUTDIG2, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG2, LOW);
       }
if ( au16data[7]== 1)
       { 
        digitalWrite( OUTDIG3, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG3, LOW);
       }
if ( au16data[8]== 1)
       { 
        digitalWrite( OUTDIG4, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG4, LOW);
       }
if ( au16data[9]== 1)
       { 
        digitalWrite( OUTDIG5, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG5, LOW);
       }
if ( au16data[10]== 1)
       { 
        digitalWrite( OUTDIG6, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG6, LOW);
       }
if ( au16data[11]== 1)
       { 
        digitalWrite( OUTDIG7, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG7, LOW);
       }
 if ( au16data[12]== 1)
       { 
        digitalWrite( OUTDIG8, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG8, LOW);
       }   

if ( au16data[13]== 1)
       { 
        digitalWrite( OUTDIG9, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG9, LOW);
       }
if ( au16data[14]== 1)
       { 
        digitalWrite( OUTDIG10, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG10, LOW);
       }
if ( au16data[15]== 1)
       { 
        digitalWrite( OUTDIG11, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG11, LOW);
       }
if ( au16data[16]== 1)
       { 
        digitalWrite( OUTDIG12, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG12, LOW);
       }
if ( au16data[17]== 1)
       { 
        digitalWrite( OUTDIG13, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG13, LOW);
       }
if ( au16data[18]== 1)
       { 
        digitalWrite( OUTDIG14, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG14, LOW);
       }
if ( au16data[19]== 1)
       { 
        digitalWrite( OUTDIG15, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG15, LOW);
       }
 if ( au16data[20]== 1)
       { 
        digitalWrite( OUTDIG16, HIGH);
       }
       else{ 
       digitalWrite( OUTDIG16, LOW);
       }            
 

//  // ENTRADAS ANALOGICAS (ADC)
  
  au16data[21]  = analogRead(A1); //El valor analógico leido en el pin A1 se guarda en au16data[4]. (siendo 0=0v y 1023=5v)
  au16data[22]  = analogRead(A2); //El valor analógico leido en el pin A0 se guarda en au16data[5]. (siendo 0=0v y 1023=5v)
  au16data[23]  = analogRead(A3);  //El valor analógico leido en el pin A7 se guarda en au16data[10]. (siendo 0=0v y 1023=5v)
  au16data[24]  = analogRead(A4);  //El valor analógico leido en el pin A6se guarda en au16data[11]. (siendo 0=0v y 1023=5v)
  au16data[25]  = analogRead(A5); //El valor analógico leido en el pin A1 se guarda en au16data[4]. (siendo 0=0v y 1023=5v)
  au16data[26]  = analogRead(A6); //El valor analógico leido en el pin A0 se guarda en au16data[5]. (siendo 0=0v y 1023=5v)
  au16data[27]  = analogRead(A7);  //El valor analógico leido en el pin A7 se guarda en au16data[10]. (siendo 0=0v y 1023=5v)
  au16data[28]  = analogRead(A8);  //El valor analógico leido en el pin A6se guarda en au16data[11]. (siendo 0=0v y 1023=5v)
  au16data[29]  = analogRead(A9); //El valor analógico leido en el pin A1 se guarda en au16data[4]. (siendo 0=0v y 1023=5v)
  au16data[30]  = analogRead(A10); //El valor analógico leido en el pin A0 se guarda en au16data[5]. (siendo 0=0v y 1023=5v)
  au16data[31]  = analogRead(A11);  //El valor analógico leido en el pin A7 se guarda en au16data[10]. (siendo 0=0v y 1023=5v)
  au16data[32]  = analogRead(A12);  //El valor analógico leido en el pin A6se guarda en au16data[11]. (siendo 0=0v y 1023=5v)
   
  //Diagnóstico de la comunicación (para debug)
  
  au16data[33] = slave.getInCnt();  //Devuelve cuantos mensajes se recibieron
  au16data[34] = slave.getOutCnt(); //Devuelve cuantos mensajes se transmitieron
  au16data[35] = slave.getErrCnt(); //Devuelve cuantos errores hubieron
  au16data[36] = slave.getID(); //Devuelve cuantos errores hubieron
 
}



void CONFIG_MODULO()
{
   
  unsigned char ValorJumpers;
  ValorJumpers = 0;
  ValorJumpers = digitalRead(JUMPER1)*4+digitalRead(JUMPER2)*2+digitalRead(JUMPER3);
 
  switch(ValorJumpers)
  {
    case 0:
    {
     Serial2.write(CANAL14_430MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 1:
    {
      Serial2.write(CANAL15_431MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 2:
    {
     Serial2.write(CANAL16_432MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 3:
    {
     Serial2.write(CANAL17_433MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 4:
    {
      Serial2.write(CANAL21_437MHZ, LONGITUD_TRAMA_MODULO);
     break; 
    }
    case 5:
    {
      Serial2.write(CANAL19_435MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 6:
    {
      Serial2.write(CANAL20_436MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 7:
    {
     Serial2.write(CANAL18_434MHZ, LONGITUD_TRAMA_MODULO);
     break;
    } 
  }
    
  return;  
}


void InicializarVariables(void)
{
  static byte VELOCIDAD;
 r_eeprom(&DIRECCION_MF_RTU, EE_DIRECCION_MF_RTU, 1);
 //r_eeprom(&RATE_BAUDIOS_RS485,EE_RATE_BAUDIOS_RS485, 1);  
 VELOCIDAD = EEPROM.read(EE_RATE_BAUDIOS_RS485) ;
Serial.println (VELOCIDAD, DEC);
       switch (VELOCIDAD)
       { 
        case 1:
        {
          Baudios = 1200;
          Serial.println ("VELOCIDAD RATE BAUDIOS 1200");
         
        break;  
        }
        case 2:
        {

          Baudios = 2400;
          Serial.println ("VELOCIDAD RATE BAUDIOS 2400");
        break;  
        }
        case 3:
        {
          Baudios = 9600;
          Serial.println ("VELOCIDAD RATE BAUDIOS 9600");
         
        break;  
        }
        case 4:
        {
          Baudios = 19200;
          Serial.println ("VELOCIDAD RATE BAUDIOS 19200");
         
        break;  
        }
        case 5:
        {
          Baudios = 38400;
          Serial.println ("VELOCIDAD RATE BAUDIOS 38400");
         
        break; 
       }
       case 6:
        {
          Baudios = 57600;
          Serial.println ("VELOCIDAD RATE BAUDIOS 57600");
         
        break; 
        }
//        case 7:
//        {
//          Baudios = 115200;
//          Serial.println ("VELOCIDAD RATE BAUDIOS 115200");
//         
//        break; 
//        }
       }
 return;
 
}


void w_eeprom(unsigned char DirEE, unsigned char *pDato, unsigned char cL_CantidadDeBytes)
{
  static unsigned char i;
  for(i=0; i<cL_CantidadDeBytes;i++)
  {
    EEPROM.update(DirEE+i, *(pDato+i));
  }
  return;
}
void r_eeprom(unsigned char *pDato, unsigned char DirEE, unsigned char cL_CantidadDeBytes)
{
  static unsigned char i;
  for(i=0; i<cL_CantidadDeBytes;i++)
  {
    *(pDato+i)=EEPROM.read(DirEE+i);
  }
  return;
}

void Programacion(void)
{
  static unsigned char i, cL_CantidadDeCaracteres;
  while(digitalRead(LLAVE_PROGRAMACION) == ON)
  {  
     //Serial.println ("PROGRAMANDO");
    digitalWrite(LED_AUX, HIGH);

    if (Serial2.available()>0)
    {  
      cL_CantidadDeCaracteres = Serial2.readBytes(cG_Recibir,64); //devuelve cantidad de caracteres
      cL_CantidadDeCaracteres = cL_CantidadDeCaracteres - 1; //le restamos uno para sacar el primer caracter de reconocimiento 
       switch (cG_Recibir[0] )
       { 
       
        //configuramos direccion MF RTU
        case 'A':
        {

          if (cG_Recibir[1]> 0 && cG_Recibir[1]<255) 
         {
         DIRECCION_MF_RTU = cG_Recibir[1]; 
         w_eeprom(EE_DIRECCION_MF_RTU,&DIRECCION_MF_RTU , 1);
         Serial2.println ("OK");
         }
         else { Serial2.println ("Direccion Fuera de Rango");
         }

        break;  
        }

        case 'B':
        {
          RATE_BAUDIOS_RS485 = cG_Recibir[1]; 
        // w_eeprom(EE_RATE_BAUDIOS_RS485,&RATE_BAUDIOS_RS485,1);

         EEPROM.write(EE_RATE_BAUDIOS_RS485,RATE_BAUDIOS_RS485);
           switch (RATE_BAUDIOS_RS485)
       { 
        case 1:
        {
           RATE_BAUDIOS_RS485= 1;
          Serial2.println ("Baud Rate 1200 OK!");
         
        break;  
        }
        case 2:
        {

          RATE_BAUDIOS_RS485= 2;
          Serial2.println ("Baud Rate 2400 OK!");
        break;  
        }
        case 3:
        {
          RATE_BAUDIOS_RS485= 3;
          Serial2.println ("Baud Rate 9600 OK!");
         
        break;  
        }
        case 4:
        {
          RATE_BAUDIOS_RS485= 4;
          Serial2.println ("Baud Rate 19200 OK!");
         
        break;  
        }
         case 5:
        {
          RATE_BAUDIOS_RS485= 5;
          Serial2.println ("Baud Rate 38400 OK!");
         
        break;  
        }
         case 6:
        {
          RATE_BAUDIOS_RS485= 6;
          Serial2.println ("Baud Rate 57600 OK!");
         
        break;  
        }
//         case 7:
//        {
//          RATE_BAUDIOS_RS485= 7;
//          Serial2.println ("Baud Rate 115200 OK!");
//         
//        break;  
//        }
       }
        break;  
        }
      }   
    } 


 if (Serial.available()>0)
    {  
      cL_CantidadDeCaracteres = Serial.readBytes(cG_Recibir,64); //devuelve cantidad de caracteres
      cL_CantidadDeCaracteres = cL_CantidadDeCaracteres - 1; //le restamos uno para sacar el primer caracter de reconocimiento 
       switch (cG_Recibir[0] )
       { 
       
        //configuramos direccion MF RTU
        case 'A':
        {

         if (cG_Recibir[1]> 0 && cG_Recibir[1]<255) 
         {
         DIRECCION_MF_RTU = cG_Recibir[1]; 
         //w_eeprom(EE_DIRECCION_MF_RTU,&DIRECCION_MF_RTU , 1);
         Serial.println ("OK");
         }
         else { Serial.println ("Direccion Fuera de Rango");
         }

//          RATE_BAUDIOS_RS485 = cG_Recibir[2]; 
//         w_eeprom(EE_RATE_BAUDIOS_RS485,&RATE_BAUDIOS_RS485,1);
//         Serial.println ("VELOCIDAD CAMBIADA");
        break;  
        }

        case 'B':
        {
          RATE_BAUDIOS_RS485 = cG_Recibir[1]; 
          EEPROM.write(EE_RATE_BAUDIOS_RS485,RATE_BAUDIOS_RS485);
        // w_eeprom(EE_RATE_BAUDIOS_RS485,&RATE_BAUDIOS_RS485,1);

         switch (RATE_BAUDIOS_RS485)
       { 
        case 1:
        {
           RATE_BAUDIOS_RS485= 1;
          Serial.println ("Baud Rate 1200 OK!");
         
        break;  
        }
        case 2:
        {

          RATE_BAUDIOS_RS485= 2;
          Serial.println ("Baud Rate 2400 OK!");
        break;  
        }
        case 3:
        {
          RATE_BAUDIOS_RS485= 3;
          Serial.println ("Baud Rate 9600 OK!");
         
        break;  
        }
        case 4:
        {
          RATE_BAUDIOS_RS485= 4;
          Serial.println ("Baud Rate 19200 OK!");
         
        break;  
        }
        case 5:
        {
          RATE_BAUDIOS_RS485= 5;
          Serial.println ("Baud Rate 38400 OK!");
         
        break;  
        }
         case 6:
        {
          RATE_BAUDIOS_RS485= 6;
          Serial.println ("Baud Rate 57600 OK!");
         
        break;  
        }
//         case 7:
//        {
//          RATE_BAUDIOS_RS485= 7;
//          Serial.println ("Baud Rate 115200 OK!");
//         
//        break;  
//        }
       }
        break;  
        }
      }   
    }

      
  }
 return;
}
