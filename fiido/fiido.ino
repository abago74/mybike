#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

// notas 
//<= 10º MIN_POWER
//6 pulsos x segundo MIN_POWER
//> 30ª MAX_POWER
//24 pulsos x segundo MAX_POWER

//si el ángulo de inclinación es mayor de 10ª ponemos la máxima potencia 25km/h.
//si el ángulo de inclinación es mayor de 30ª ponemos la máxima potencia 25km/h.
//si pedaleas 24 pulsos por segundo ponemos la máxima potencia 25km/h.
//si pedaleas 12 pulsos por minuto asistencia a 15km/h

//V_2.0.19

// ********************** Pins
// > Inputs
const int GEAR_PLATE_IN = 2; // Pin de entrada de sensor de pedal por interrupcion. pull_down 0V
const int MODE_IN = 3; // Pin de entrada de cambio de modo. pull_down 0V
const int BRAKE_IN = 6; //Pin del sensor hall de freno. pull_up 5V
// > Outputs
const int POWER_OUT = 11; // Pin de potencia de salida.
const int STATUS_LED_OUT = 13; // Pin de visualización de selección de modo

// ********************** Def Consts
const boolean LOGGER = true;

const int SERIAL_PORT = 9600; // Configuración de baudios del puerto serie.

const int GEAR_PLATE_PULSES = 12; // Pulsos proporcionados por el plato en cada vuelta.

const int MIN_PLATE_TIME_SLOW = 2000; // Tiempo que se tarda en dar una vuelta de plato
const int MAX_PLATE_TIME_FAST = 600; // Tiempo que se tarda en dar una vuelta de plato

const int POWER_MAX_VALUE = 4800; // Máxima potencia del PWM. 4800 -> 3.58v
const int POWER_MIN_ASSISTENCE_VALUE = 2000; // Mínima potencia del PWM. 1470 -> 1.108v
const int POWER_MIN_VALUE = 1; // Mínima potencia del PWM. 1470 -> 1.108v

const int POWER_STEPTS[] = {1, 2, 5, 10, 25, 50}; // Incremento de potencia de la salida PWM por cada paso.
const int POWER_STEPTS_DEFAULT_VALUE = 2; 

const int MAX_TIME_BETWEEN_GEAR_PLATE_PULSES = 1000; // Tiempo mínimo en ms que se tiene que estar pedaleando para empezar a recibir potencia.

const int DEBOUNCE_TIMETHRESHOLD_GEAR_PLATE = 50; // Tiempo de espera entre pulsos de interrupcion 49ms entre pulsación. 100 Pedaladas(GEAR_PLATE_PULSES*100=1200) por minuto (20 pulsos por segundo). 50ms de debounce para detectar el máximo de 20 pulsos por segundo.
const int DEBOUNCE_TIMETHRESHOLD_CHANGE_MODE = 600; // Tiempo de espera entre pulsos de interrupcion 600ms entre pulsación

const int EEPROM_INIT_ADDRESS=0; // Posición de memoria que almacena los datos de modo.
const int MPU_ADDRESS = 0x68;  // Dirección de memória del acelerómetro - Puede ser 0x68 o 0x69 segín el dispositivo.

const int MAX_POWER_ANGLE = 30;

// ********************** Control de rebote de pulsos 
unsigned long debounceLastGearPlatePulseTime; // Almacena el millis() en el que se tomo medida del pulso anterior del sensor de pedalada para controlar el debounce.
unsigned long debounceChangeModeTime = 0; // Almacena el millis() en el que se tomo medida del último pulso de cambio de modo para controlar el debounce.

// ********************** Variables de control de pedalada.
unsigned long gearPlateLastPulseTime; // Temporizador que almacena el millis() del último pulso de plato para saber si se está pedaleando.
unsigned int gearPlatecompleteCicleCounter; // Contador de pulsos por ciclo de pedalada para controlar las revoluciones por minuto del plato.
boolean completeGearPlateCicle = false; // Pasa el testigo al loop para operar con los datos detectados por las interrupciones.
unsigned long pulseInit; // Almacena los millis del primer pulso de la pedalada.
unsigned long pulseEnd; // Almacena los millis del último pulso de la pedalada.

// ********************** Variables de control de potencia
int powerCurrentValue; // Potencia actual
int maxPowerValue; // Máxima potencia calculada según pedalada y la inclinación.

// ********************** Variables de control de cambio de modo. + EEPROM
int powerModeChangeTrigger; // Trigger de solicitud de cambio de modo . La variable es cambiada de estado por el método changeModeInt lanzado por la interrupción para detectar el tipo. Valores 0/1/2.
// Estructura para almacenar los valores de los modos de control.
struct EStorage{
  int powerMode;
  int powerBrakeMode;
};
EStorage eStorage; // Instancia Variable de control de modos.
boolean eStorageUpdate = false; // Flag solicitud de almacenamiendo de modos en la eeprom.

// ********************** Variables de control de Acelerómetro MPU6050 
boolean mpuenabled=true;
MPU6050 mpu6050(MPU_ADDRESS); // Crea una instancia del acelerómetro.
int16_t ax, ay, az; // Variables de control de Acelerómetro xyz
int16_t gx, gy, gz; // Variables de control de Giroscopio xyz

String outputMessage="";

// main
void loop() {
  changeModeFuncion(); // Controla powerModeChangeTrigger para cambiar los modos y almacenar los nuevos valores en la eeprom

  if(mpuenabled){
    mpu6050.getAcceleration(&ax, &ay, &az); // Lee los datos del acelerómetro.
    //mpu6050.getRotation(&gx, &gy, &gz);  // Lee los datos del giroscopio
    //mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Lee los valores de acelerómetro y giroscopio
  } 
  
  if((digitalRead(BRAKE_IN) == LOW || isCrashDrop()) && powerCurrentValue>POWER_MIN_VALUE){ // Resetea el acelerador si se toca el freno o se está en estado caida y la potencia de acelerador no es la mínima.
      reset(true); // Resetea valores cortando la potencia en modo seguridad
      updatePower();  // Actualizamos potencia de salida.
      
      outputMessage = " > BRAKE: \t maxPowerValue: \t";
      outputMessage += maxPowerValue;
      Serial.println(outputMessage);
      
      blinkLed(STATUS_LED_OUT, 15, 20); // Muestra el testigo de frenado. 300ms
  }else{
     if(millis()-gearPlateLastPulseTime < MAX_TIME_BETWEEN_GEAR_PLATE_PULSES){ // Si en los ultimos n segundos se ha detectado un pulso de pedaleo
        //Serial.println("PEDALEANDO");
        // V En cada ciclo de disco ejecutamos las operaciones necesarias. 
        // V Como calcular la potencia dependiendo de la potencia de pedalada. A esto le añadiremos el control de inclinación cuando esté implementado.
        if (completeGearPlateCicle){ 
            calculateMaxPower(pulseEnd-pulseInit, getAngle());
        }
     }else{ // El sensor no detecta pulso de pedaleo. // TODO decrementa progresivamente la pedalada.    
      //Serial.println("PARADO");
      reset(false); // Resetea valores cortando la potencia en modo progresivo
     }  
  }
  updatePower(); // actualizamos la potencia de salida.
  showStatus(); // Muestra el led de estado y guarda valores en la eeprom
}

float getAngle(){
   //Calcular los angulos de inclinacion
   float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
   return accel_ang_y;
}

boolean isCrashDrop(){ // Si El acelerómetro detecta que la bicicleta ha caido al suelo.
  float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);
  //if(accel_ang_x>60 || accel_ang_x<-60){ // ELIMINAR DESPUES DE PRUEBAS
  //    outputMessage = "* CAIDA ";
  //    outputMessage += accel_ang_x;
  //    Serial.println(outputMessage);
  //}
  return (accel_ang_x>50 || accel_ang_x<-50);
}

void calculateMaxPower(int timeCicle, float angle){
  int maxPowerValueTmp = 0;
  int pedalPowerValueTmp = 0;
  
  int currentAngle = (int) angle; // Valor entero del ángulo de inclinación
  
  if(currentAngle>MAX_POWER_ANGLE){ // Si el ángulo es mayor de 30º
    maxPowerValueTmp = POWER_MAX_VALUE; // Setea el máximo valor de potencia
  }else{ // Si el ángulo es menor de 30º calcula la potencia dependiendo del rango del ángulo.
    float min_assistence_range = ((1.0*POWER_MAX_VALUE)/POWER_MIN_ASSISTENCE_VALUE)/MAX_POWER_ANGLE; // Calculamos la fracción del ángulo     
    maxPowerValueTmp = min_assistence_range*currentAngle*POWER_MIN_ASSISTENCE_VALUE; // Calculamos el valor máximo de potencia
    pedalPowerValueTmp = ((POWER_MAX_VALUE-POWER_MIN_ASSISTENCE_VALUE)/(MIN_PLATE_TIME_SLOW-MAX_PLATE_TIME_FAST))*(POWER_MIN_ASSISTENCE_VALUE-timeCicle)+POWER_MIN_ASSISTENCE_VALUE;    
  }

  //maxPowerValue = pedalPowerValueTmp>maxPowerValueTmp?pedalPowerValueTmp:maxPowerValueTmp;
  maxPowerValue = (maxPowerValue + (pedalPowerValueTmp>maxPowerValueTmp?pedalPowerValueTmp:maxPowerValueTmp))/2; // Calculamos la media de la potencia anterior y la potencia actual. Proceso para controlar cambios bruscos en las medidas.
  
  if(maxPowerValue<POWER_MIN_ASSISTENCE_VALUE){ // Si máximo valor de potencia es menor que el mímimo, seteamos el valor mínimo de referencia,
    maxPowerValue = POWER_MIN_ASSISTENCE_VALUE;
  }else if(maxPowerValue>POWER_MAX_VALUE){
    maxPowerValue = POWER_MAX_VALUE;
  }
  
  outputMessage = F("\tInclinacion en eje Y:");
  outputMessage += angle;
  outputMessage += "\tPULSO :";
  outputMessage += timeCicle;
  outputMessage += "\tmaxPowerValue :";
  outputMessage += maxPowerValue;
  outputMessage += "\tpowerCurrentValue :";
  outputMessage += powerCurrentValue;
  Serial.println(outputMessage);
  
  completeGearPlateCicle=false; // Reseteamos ciclo de plato.
}

void updatePower(){
      if(powerCurrentValue < maxPowerValue){ // incrementa progresivamente la potencia hasta la máxima.
        powerCurrentValue = powerCurrentValue + POWER_STEPTS[eStorage.powerMode];
        if(powerCurrentValue > maxPowerValue){ // Si se supera el valor máximo de escala de potencia, este se regula.
          powerCurrentValue = maxPowerValue;
        }
      }else if(powerCurrentValue > maxPowerValue){ // decrementa progresivamente la potencia.
        powerCurrentValue = powerCurrentValue - POWER_STEPTS[eStorage.powerBrakeMode]; // Desacelera en el step más bajo.
        if(powerCurrentValue < maxPowerValue){// Si se rebasa el nivel mínimo de escala de potencia, este se regula.
          powerCurrentValue = maxPowerValue;
        }
      }
      //if(powerCurrentValue < maxPowerValue && powerCurrentValue > POWER_MIN_VALUE){
      //  Serial.print("PWR-> ");
      //  Serial.println(powerCurrentValue);
      //}
      analogWrite(POWER_OUT, powerCurrentValue/20); // Actualiza la salida con la potencia de acelerador.
}



// interruption methods
void gearPlatePulseInt(){ // Método que incrementa el contador de pedal en caso de pulso por el pin de interrupcion.
 
  if (millis() > debounceLastGearPlatePulseTime + DEBOUNCE_TIMETHRESHOLD_GEAR_PLATE){ // Debounce Si ha pasado el tiempo de control de rebote.
      gearPlateLastPulseTime = millis(); // Almacena el flag de control de último pulso para evitar rebotes.
      
      if(gearPlatecompleteCicleCounter > 0){ // Iteramos los pulsos del plato para detectar el inicio y el fin vuelta.
        gearPlatecompleteCicleCounter--;
      }else{
        gearPlatecompleteCicleCounter=GEAR_PLATE_PULSES;
      }

      if(gearPlatecompleteCicleCounter==0 && pulseInit>0){ // Si el pulso es final y tenemos timer de inicio
          completeGearPlateCicle = true; // Activamos el flag de ciclo completo.
          pulseEnd=millis(); // Seteamos el timer de fin
      }else if(gearPlatecompleteCicleCounter==GEAR_PLATE_PULSES){ // Si el pulso es inicial
        pulseInit=millis(); // Seteamos el timer de inicio
      }

      //outputMessage = gearPlatecompleteCicleCounter;
      //outputMessage += " - ";
      //outputMessage += pulseInit;
      //outputMessage += " | ";
      //outputMessage += pulseEnd;
      //Serial.println(outputMessage);
            
    debounceLastGearPlatePulseTime = millis(); //Debounce Guardamos el tiempo de la operación para bloquear el rebote.
  }  
}

void changeModeInt(){
  if (millis() > debounceChangeModeTime + DEBOUNCE_TIMETHRESHOLD_CHANGE_MODE){ //Debounce
    if(digitalRead(BRAKE_IN) == LOW){
      powerModeChangeTrigger = 2; // cambia el modo de desaceleración
    }else{
      powerModeChangeTrigger = 1; // cambia el modo de aceleración
    }
  }
  debounceChangeModeTime = millis();//Debounce
}

void changeModeFuncion(){
  if(powerModeChangeTrigger == 1){
    eStorageUpdate=true;
    powerModeChangeTrigger = 0;
    if(eStorage.powerMode < (sizeof(POWER_STEPTS) / sizeof(int)-1)){
      eStorage.powerMode++;
    }else{
      eStorage.powerMode = 0;
    }

    outputMessage = "ChangeModeFuncion: (";
    outputMessage += eStorage.powerMode;
    outputMessage += ") - pasos: ";
    outputMessage += POWER_STEPTS[eStorage.powerMode];
    Serial.println(outputMessage);
      
    blinkLed(STATUS_LED_OUT, eStorage.powerMode+1, (600/(eStorage.powerMode+1)));
    powerCurrentValue=POWER_MIN_VALUE; 
  }else if(powerModeChangeTrigger == 2){
    eStorageUpdate=true; //habilita el flag de actualización de eeprom.
    powerModeChangeTrigger = 0;
    if(eStorage.powerBrakeMode < (sizeof(POWER_STEPTS)/sizeof(int)-1)){
      eStorage.powerBrakeMode++;
    }else{
      eStorage.powerBrakeMode = 0;
    }
    
    outputMessage = "ChangeBrakeModeFuncion: (";
    outputMessage += eStorage.powerBrakeMode;
    outputMessage += ") - pasos: ";
    outputMessage += POWER_STEPTS[eStorage.powerBrakeMode];
    Serial.println(outputMessage);
    
    blinkLed(STATUS_LED_OUT,eStorage.powerBrakeMode+1,(600/(eStorage.powerBrakeMode+1)));
    powerCurrentValue = POWER_MIN_VALUE; 
  }  
}

void showStatus(){ // Muestra el led de estado y guarda valores en la eeprom
  if(powerCurrentValue > ((maxPowerValue-POWER_MIN_VALUE)*0.8) && powerCurrentValue > POWER_MIN_VALUE){ //Enciende status de potencia cuando el acelerador está superando el 80X% 
    digitalWrite(STATUS_LED_OUT, HIGH);
    if(eStorageUpdate==true){
      updateEepromPowerModeStatus(); // Guardamos los datos de modos en la eeprom.
    }
  }else{
    digitalWrite(STATUS_LED_OUT, LOW);
  }  
}

void updateEepromPowerModeStatus(){  
    eStorageUpdate=false; // Anulamos el guardado de valores 

    outputMessage = "Update eeprom: [powerMode: ";
    outputMessage += eStorage.powerMode;
    outputMessage += "| powerBrakeMode: ";
    outputMessage += eStorage.powerBrakeMode;
    outputMessage += "]";
    Serial.println(outputMessage);    
    // TODO - HABILITAR PARA GRABAR EN ENTORNO DE PRODUCCIÓN 
    //EEPROM.put(EEPROM_INIT_ADDRESS, eStorage); // Actualizan los datos de modos entrega de potencia en la eeprom
}

void reset(boolean brake){
      gearPlateLastPulseTime=0; // Reiniciamos el tiempo de control de rebote.
      gearPlatecompleteCicleCounter=0; // Reiniciamos la posición del contador de vueltas de plato.
      pulseEnd=0; // Reiniciamos el timer de inicio de pedalada.
      pulseInit=0;// Reiniciamos el timer de fin de pedalada.  
      maxPowerValue=POWER_MIN_VALUE;
      if(brake){
        powerCurrentValue=POWER_MIN_VALUE;
      }   
}
// init
void setup() {

    if(LOGGER==true){
      Serial.begin(SERIAL_PORT); //Inicializa el puesto serie para sacar datos por consola.
    }


    outputMessage = " | Fiido D1|D2 asistence project | \n";
    outputMessage += "***********************************\n";
    Serial.println(outputMessage);
    
    if(mpuenabled){
        // MPU6050
        #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
            Wire.begin();
        #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
            Fastwire::setup(400, true);
        #endif
        
        mpu6050.initialize(); // Inicializa el acelerómetro.
        Serial.println(mpu6050.testConnection() ? F(" > MPU iniciado correctamente.") : F(" * ERROR: Error al iniciar MPU."));
        
    }else{
      // Inicializamos valores de inclinación sin acelerómetro.
      ax=0; ay=0; az=1;
    }

    // Inicializa valores de modos.
    eStorage.powerMode=POWER_STEPTS_DEFAULT_VALUE;
    eStorage.powerBrakeMode=POWER_STEPTS_DEFAULT_VALUE;
    EEPROM.get(EEPROM_INIT_ADDRESS, eStorage); // Captura los valores desde la eeprom

    outputMessage = " > powerMode: ";
    outputMessage += eStorage.powerMode;
    outputMessage += "\t > powerBrakeMode: ";
    outputMessage += eStorage.powerBrakeMode;
    Serial.println(outputMessage);

    //Inicializamos los pines
    pinMode(LED_BUILTIN,OUTPUT); // Salida led modo/estado. //STATUS_LED_OUT
    pinMode(POWER_OUT,OUTPUT); // Salida PWM acelerador variable.
    
    pinMode(BRAKE_IN, INPUT); // Entrada de control de freno.
    pinMode(GEAR_PLATE_IN, INPUT); // Entrada Interrupción  - control de pedalaleo.
    pinMode(MODE_IN, INPUT); // Entrada Interrupción - control nodo de operación.
        
    // Inicialización de variables
    debounceLastGearPlatePulseTime = 0; // Inicializamos el timer de control de rebote
    gearPlateLastPulseTime = 0; // Inicializamos el timer de último pulso de pedaleo.
    gearPlatecompleteCicleCounter = GEAR_PLATE_PULSES; // Inicializamos el contador de pulsos para controlar el ciclo de pedalada. cada n pulsos se calculará el tiempo que se ha tardado en dar una vuelta al disco.

    initFlashLeds(); // Ejecuta los leds de inicio de script. y muestra los modos.
   
    // Definición de interrupciones 
    attachInterrupt(digitalPinToInterrupt(GEAR_PLATE_IN), gearPlatePulseInt, RISING); // Definición de la interrupción para la entrada de pin de pedal.
    attachInterrupt(digitalPinToInterrupt(MODE_IN), changeModeInt, RISING); // Definición de la interrupción para la entrada de pin de cambio de modo.

    outputMessage = "\n";
    outputMessage += " > Init Done!!!\n";
    outputMessage += "***********************************";
    Serial.println(outputMessage);
}

// init methods
void initFlashLeds(){
    Serial.print(" .");
    blinkLed(STATUS_LED_OUT, eStorage.powerMode+1, 60); // Muestra el modo de aceleración.
    runDelay(1000);
    blinkLed(STATUS_LED_OUT, eStorage.powerBrakeMode+1, 60); // Muestra el modo de desaceleración.
    runDelay(1500);
    digitalWrite(STATUS_LED_OUT, HIGH); // Muestra el led de fin de inicialización.
    runDelay(500);
    digitalWrite(STATUS_LED_OUT, LOW);
    Serial.println(".");
}

void runDelay(int dlay){ // Ejecuta un delay y sacamos puntos por la consola.
  int repeat = dlay/100;
  for(int i=0; i <= repeat; i++){
    delay(100);
    Serial.print(".");
  }
}

void blinkLed(int ledPin, int repeats, int time){ // Ejecuta un parpadeo en el led durante x repeticiones y con una frecuencia de x_ms.
  for (int i = 0; i < repeats; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(time);
    digitalWrite(ledPin, LOW);
    delay(time);
  }
}

void checkI2cDevices(){
  byte count = 0; 
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0){
      outputMessage = "Found address: ";
      outputMessage += (i, DEC);
      outputMessage += " (0x";
      outputMessage += (i, HEX);
      outputMessage += ")";
      Serial.println(outputMessage);
        
      count++;
      delay (1);
    }
  }
  
  outputMessage = "Done.\n";
  outputMessage += "Found ";
  outputMessage += (count, DEC);
  outputMessage += " device(s).";
  Serial.println(outputMessage);
}
