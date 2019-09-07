const String VERSION = "V_2.4.10";
const boolean plotteron=false;
const boolean traceOn=!plotteron;

#include <EEPROM.h>

#define OLED_ADDR 0x3C // Dirección de memória del display
#define MPU_ADDR  0x68 // Dirección de memória del acelerómetro - Puede ser 0x68 o 0x69 segín el dispositivo.
#define DAC_ADDR  0x60 // Dirección DAC

#include <Adafruit_SSD1306.h> // I2C DISPLAY
#include <Adafruit_GFX.h>

#include <Adafruit_MCP4725.h> // I2C DAC MCP4725

#include <MPU6050.h> // I2C MPU6050

#include <I2Cdev.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

// notas
//<= 10º MIN_ASSISTENCE_POWER
//6 pulsos x segundo MIN_ASSISTENCE_POWER
//> 30ª MAX_POWER
//24 pulsos x segundo MAX_POWER

//si el ángulo de inclinación es mayor de 30ª ponemos la máxima potencia 25km/h.
//si pedaleas 24 pulsos por segundo ponemos la máxima potencia 25km/h.
//si pedaleas 12 pulsos por minuto asistencia a 15km/h

// ********************** DISPLAY
#define SSD1306_128_64
#define OLED_RESET 4
Adafruit_SSD1306 oled1306(OLED_RESET); // Creamos instancia de la pantalla

// ********************** DAC
Adafruit_MCP4725 dac4725;

// ********************** GIROSCOPIO
MPU6050 mpu6050(MPU_ADDR); // Crea una instancia del acelerómetro.
const int DEVOUNCE_MPU_TIME_THRESHOLD_CONST=100; // 2000 default

// ------------------------- CONSTANTS ----------------------------------

const byte ZERO = 0; // Pin de entrada de sensor de pedal por interrupcion. pull_down 0V
const long LZERO = 0.0; // Pin de entrada de sensor de pedal por interrupcion. pull_down 0V

// ********************** PINS
const byte GEAR_PLATE_IN = 2; // Pin de entrada de sensor de pedal por interrupcion. pull_down 0V
const byte MODE_IN = 3; // Pin de entrada de cambio de modo. pull_down 0V
const byte BRAKE_IN = 6; //Pin del sensor hall de freno. pull_up 5V
const byte THROTTLE_IN = A0; // pin acelerador

const byte POWER_OUT = 11; // Pin de potencia de salida.
const byte STATUS_LED_OUT = 13; // Pin de visualización de selección de modo

// ********************** DEFAULT
const int SERIAL_PORT = 19200; // Configuración de baudios del puerto serie.
const byte SERIAL_BUFFER_SIZE = 32; // Buffer de lectura del puerto serie.
const byte EEPROM_INIT_ADDRESS = 0; // Posición de memoria que almacena los datos de modo.

// ********************** DEVOUNCE
const byte DEBOUNCE_TIMETHRESHOLD_GEAR_PLATE = 60; // Tiempo de espera entre pulsos de interrupcion 49ms entre pulsación. 100 Pedaladas(GEAR_PLATE_PULSES*100=1200) por minuto (20 pulsos por segundo). 50ms de debounce para detectar el máximo de 20 pulsos por segundo.
const int DEBOUNCE_TIMETHRESHOLD_CHANGE_MODE = 1200; // Tiempo de espera entre pulsos de interrupcion 1200ms entre pulsación

// ********************** POWER

const int POWER_INPUT_MULTIPLIER = 10; // Máxima potencia del A0 732 - PWM. 4800 -> 3.58v
const int DEFAULT_POWER_MAX_VALUE = 4800; // Máxima potencia del PWM. 4800 -> 3.58v
const int DEFAULT_POWER_MIN_ASSISTENCE_VALUE = 2000; // Mínima potencia de asistencia ?.???v
const int DEFAULT_POWER_MIN_VALUE = 1470; // Mínima potencia del PWM. 1470 -> 1.108v

//const int DEFAULT_POWER_MAX_VALUE = 732*POWER_INPUT_MULTIPLIER; // Máxima potencia del A0 732 - PWM. 4800 -> 3.58v
//const int DEFAULT_POWER_MIN_VALUE = 226*POWER_INPUT_MULTIPLIER; // Mínima potencia del A0 226 - PWM. 1470 -> 1.108v
//const int DEFAULT_POWER_MIN_ASSISTENCE_VALUE = DEFAULT_POWER_MIN_VALUE + ((DEFAULT_POWER_MAX_VALUE-DEFAULT_POWER_MIN_VALUE)/2); // Mínima potencia de asistencia ?.???v


const int POWER_STEPTS[] = {2, 4, 8, 16, 32, 64, 128}; // Incremento de potencia de la salida POWER por cada paso.
const byte POWER_STEPTS_DEFAULT_POSITION = 3; // El rango (3)=16 es el valor por defecto
const byte UNDEFINED_CHANGE_MODE = 0, POWER_CHANGE_MODE = 1, POWERBRAKE_CHANGE_MODE = 2;

// ********************** PAS PLATE
const byte GEAR_PLATE_PULSES = 10; // Pulsos proporcionados por el plato en cada vuelta. 10+ en el caso de la fiido
const int MAX_PLATE_TIME_FAST = 600; // Tiempo que se tarda en dar una vuelta de plato
const int MIN_PLATE_TIME_SLOW = 2000; // Tiempo que se tarda en dar una vuelta de plato
const int MAX_TIME_BETWEEN_GEAR_PLATE_PULSES = 300; // Tiempo mínimo en ms que se tiene que estar pedaleando para empezar a recibir potencia.

// ********************** MPU
const byte DEFAULT_MAX_POWER_ANGLE = 30;
const byte X=0, Y=1, Z=2; // Utilizadas por el método getAxisAngle
const byte CRASH_ANGLE=50; // Anagulo de control de caida.
const byte DEFAULT_POWER_BRAKE_DIVIDER = 20; // baja la potencia por frenada en pasos de 20%
const int CUT_POWER_ANGLE = -10; // inclinación negativa que bloquea la entrega de potencia

// ********************** DISPLAY
const byte START_LINE = 0;
const byte LINE_0 = 0, LINE_1 = 10, LINE_2 = 20;

// Estructura para almacenar los valores de los modos de control.
struct EStorage {
  boolean mpuenabled = false;
  byte powerAngleAxis=X; // Eje que controla el ángulo de potencia
  boolean powerAngleAxisInverter=true; // Inversor del eje de potencia

  int XAccelOffset=ZERO;
  int YAccelOffset=ZERO;
  int ZAccelOffset=ZERO;
  int XGyroOffset=ZERO;
  int YGyroOffset=ZERO;
  int ZGyroOffset=ZERO;

  byte powerMode = POWER_STEPTS_DEFAULT_POSITION;
  byte powerBrakeMode = POWER_STEPTS_DEFAULT_POSITION;
  int powerMaxValue = DEFAULT_POWER_MAX_VALUE;
  int powerMinAssistenceValue = DEFAULT_POWER_MIN_ASSISTENCE_VALUE;
  int powerMinValue = DEFAULT_POWER_MIN_VALUE;

  byte maxPowerAngle = DEFAULT_MAX_POWER_ANGLE;
  byte powerBrakeDivider = DEFAULT_POWER_BRAKE_DIVIDER;
  
};

// ------------------------- VARS -------------------------------

// ********************** Serial Input
char receivedChars[SERIAL_BUFFER_SIZE]; // an array to store the received data ut to SERIAL_BUFFER_SIZE
boolean newSerialDataFlag = false; // Flag que controla cuando entran datos por el puerto seria que dispara la interpretación de comandos AT

// **********************  DEVOUNCE
unsigned long debounceLastGearPlatePulseTime; // Almacena el millis() en el que se tomo medida del pulso anterior del sensor de pedalada para controlar el debounce.
unsigned long debounceChangeModeTime; // Almacena el millis() en el que se tomo medida del último pulso de cambio de modo para controlar el debounce.

// ********************** PAS PLATE
byte gearPlatecompleteCicleCounter; // Contador de pulsos por ciclo de pedalada para controlar las revoluciones por minuto del plato.
unsigned long gearPlateLastPulseTime; // Tiempo tomado en el último pulso.
unsigned int gearPlateLastCompleteCicleValue; // Tiempo medio del ciclo completo.
unsigned int gearPlateCurrentPulseValue; // Último pulso leido.
unsigned int gearPlateCompleteCicleValue; // Contenedor del valor tomado por una vuelta de disco.


// ********************** POWER
unsigned int maxPowerValue; // Máxima potencia calculada según pedalada y la inclinación.
unsigned int currentPowerValue; // Potencia actual
float currentThrottleValue; // Lectura de tensión de acelerador.
unsigned int cruisePower;

// ********************** MODE
int powerModeChangeTrigger; // Trigger de solicitud de cambio de modo . La variable es cambiada de estado por el método changeModeInt lanzado por la interrupción para detectar el tipo. Valores 0/1/2.

// ********************** MPU6050
float currentAngle; //Variable que almacena el ángulo de potencia.
long devounceMpuTimeThreshold; // Tiempo entre las medidas de ángulo
byte crashAngleAxis; // Eje que controla el ángulo de control de caida

// ********************** DEFAULT

EStorage eStorage; // Instancia de datos de configuración eeprom
String messageBuilder; // Variable utilizada para crear mensajes dinámicos.

boolean flaginit=false;
// init
void setup() {

  // Lee configuración desde la eeprom.
  EEPROM.get(EEPROM_INIT_ADDRESS, eStorage); // Captura los valores desde la eeprom

  Serial.begin(SERIAL_PORT); //Inicializa el puesto serie

  oled1306.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR); // inicializamos pantalla en dirección i2c
  showHomeScreen();

  if (eStorage.mpuenabled) {
    // MPU6050
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu6050.initialize(); // Inicializa el acelerómetro.
    //serialTraceLn(mpu6050.testConnection() ? F(" > MPU iniciado correctamente.") : F(" * ERROR: Error al iniciar MPU."));
    oled1306.setCursor(START_LINE, LINE_0);
    oled1306.print(F(" + MPU_"));
    oled1306.print(mpu6050.testConnection() ? F("OK") : F("KO"));
    devounceMpuTimeThreshold=millis();
  } else {
    oled1306.print(F(" - MPU"));
  }

  dac4725.begin(DAC_ADDR); // Inicializa el DAC
  oled1306.print(F(" > +DAC"));

  //Inicializamos los pines
  pinMode(LED_BUILTIN, OUTPUT); // Salida led modo/estado. //STATUS_LED_OUT
  pinMode(POWER_OUT, OUTPUT); // Salida PWM acelerador variable.

  pinMode(BRAKE_IN, INPUT_PULLUP); // Entrada de control de freno.
  pinMode(GEAR_PLATE_IN, INPUT_PULLUP); // Entrada Interrupción  - control de pedalaleo.
  pinMode(MODE_IN, INPUT_PULLUP); // Entrada Interrupción - control nodo de operación.

  pinMode(THROTTLE_IN, INPUT); // Entrada lectura potencia acelerador.

  // Inicialización de variables
  debounceLastGearPlatePulseTime = ZERO; // Inicializamos el timer de control de rebote.
  debounceChangeModeTime = ZERO; // Inicializamos el timer de control de rebote.
  gearPlatecompleteCicleCounter = GEAR_PLATE_PULSES; // Inicializamos el contador de pulsos para controlar el ciclo de pedalada. cada n pulsos se calculará el tiempo que se ha tardado en dar una vuelta al disco.
  maxPowerValue=DEFAULT_POWER_MIN_VALUE;
  currentPowerValue=DEFAULT_POWER_MIN_VALUE;
  powerModeChangeTrigger = ZERO;
  currentAngle = LZERO;
  devounceMpuTimeThreshold = ZERO; // Tiempo entre las medidas de ángulo
  cruisePower=4000;
  crashAngleAxis=(eStorage.powerAngleAxis == X)?Y:X; // Eje que controla el ángulo de control de caida

  //initFlashLeds(); // Ejecuta los leds de inicio de script. y muestra los modos.

  // Definición de interrupciones
  attachInterrupt(digitalPinToInterrupt(GEAR_PLATE_IN), gearPlatePulseInt, CHANGE); // Definición de la interrupción para la entrada de pin de pedal.
  attachInterrupt(digitalPinToInterrupt(MODE_IN), changeModeInt, FALLING); // Definición de la interrupción para la entrada de pin de cambio de modo.
  oled1306.print(F(" |"));
  oled1306.display();

  showEepromDataScreen();

  // Carga el inicializador de configuración manual.
  //if (digitalRead(BRAKE_IN) == LOW) {
  //  flaginit=true;
  //}
}
int cont=0;
// Main Program
void loop() {
  
  //if(flaginit){ // inicializamos dejando pulsado el pin de freno al iniciar. Después contamos los pulsos para detectar que comando mandar.
  //  initThrotleMinMax();
  //  flaginit=false;
  //}else{  
    //serialTraceLn(F("1 loop"));
    changeModeListener(); // Controla powerModeChangeTrigger para cambiar los modos
    currentThrottleValue=analogRead(THROTTLE_IN); // Lee el valor analógico del acelerador. *5/1023 para obtener voltios.
    //Serial.println(currentThrottleValue*5/1023);
    serialAtCommandListener(); // Lee comandos AT por puerto serie
    ATCommandsManager(); // Interpreta comandos AT
  
    if (eStorage.mpuenabled && (millis() - devounceMpuTimeThreshold > DEVOUNCE_MPU_TIME_THRESHOLD_CONST)) { // leemos el valor de inclinación (Y) cada 2 segundos para ayudar a calcular la potencia.
      currentAngle=getAxisAngle(eStorage.powerAngleAxis);
      if(eStorage.powerAngleAxisInverter)
        currentAngle=-currentAngle;
      devounceMpuTimeThreshold = millis();
    }
    
    boolean iscrashdropflag = isCrashDrop();
    if ((digitalRead(BRAKE_IN) == LOW || iscrashdropflag) && currentPowerValue > eStorage.powerMinValue) { // Resetea el acelerador si se toca el freno o se está en estado caida y la potencia de acelerador no es la mínima.
      // Resetea valores cortando la potencia en modo seguridad
      //gearPlatePulseEnd = 0; // Reiniciamos el tiempo de control de rebote.
      //gearPlatecompleteCicleCounter = GEAR_PLATE_PULSES; // Reiniciamos la posición del contador de vueltas de plato.
      //gearPlatePulseInit = 0; // Reiniciamos el timer de fin de pedalada.
      maxPowerValue = eStorage.powerMinValue;
      if(!iscrashdropflag){
        if(currentPowerValue > eStorage.powerMinValue){     
          currentPowerValue = makeDiscount(currentPowerValue,eStorage.powerBrakeDivider); // Decrementa la potencia un x% - si se deja pulsado el freno corta toda la potencia en unos 400ms
          if(currentPowerValue < eStorage.powerMinValue){currentPowerValue = eStorage.powerMinValue;} 
        }
      }else{
        currentPowerValue = eStorage.powerMinValue;
      }
  
      messageBuilder = F(" BRAKE!!! ");
      messageBuilder = messageBuilder + currentPowerValue;
      showAlertScreen(messageBuilder);
      
      blinkLed(STATUS_LED_OUT, 10, 10); // Muestra el testigo de frenado. 100ms
    } else {
      if(millis()-gearPlateLastPulseTime<MAX_TIME_BETWEEN_GEAR_PLATE_PULSES){
        /*
        serialTrace("PEDALEANDO - ");
        serialTrace(gearPlateLastCompleteCicleValue);
        serialTrace(" >>> pwr: ");
        serialTraceLn(currentPowerValue);
        */
        showPedalIcon(WHITE);
        oled1306.setCursor(110, LINE_2);
        oled1306.print(gearPlatecompleteCicleCounter);
        oled1306.display();   
        maxPowerValue = (maxPowerValue + calculateMaxPower())/2;
        
      }else{
        //serialTrace("."); 
        showPedalIcon(BLACK);   
        maxPowerValue = eStorage.powerMinValue;
      }  
    }
    updatePower(); // actualizamos la potencia de salida.
    
    if(plotteron){
        //plotter/
        Serial.print((((int) currentAngle)*100)+DEFAULT_POWER_MAX_VALUE+1000);
        Serial.print("\t");
          
        //plotter
        Serial.print(maxPowerValue);
        Serial.print("\t");
        
        //plotter
        Serial.print(currentPowerValue);
        Serial.print("\t");
        
        Serial.println(currentThrottleValue);
    }
  //}
}
// Interruption Methods ***********************************************************************************
void gearPlatePulseInt() { // Método que incrementa el contador de pedal en caso de pulso por el pin de interrupcion.
  //serialTraceLn(F("2 gearPlatePulseInt"));

  if (millis() > gearPlateLastPulseTime + DEBOUNCE_TIMETHRESHOLD_GEAR_PLATE) { // Debounce Si ha pasado el tiempo de control de rebote.

    gearPlateCurrentPulseValue = millis()-gearPlateLastPulseTime;
    if(gearPlateCurrentPulseValue>MAX_TIME_BETWEEN_GEAR_PLATE_PULSES){ // Si llevas más de 300ms sin pedalear
      gearPlateCurrentPulseValue = MAX_TIME_BETWEEN_GEAR_PLATE_PULSES;
      gearPlatecompleteCicleCounter = GEAR_PLATE_PULSES; //Reiniciamos pulsos de plato
    }
    //serialTrace(gearPlatecompleteCicleCounter);
    //serialTrace(" --->>> pedal : ");
    //serialTraceLn(gearPlateCurrentPulseValue);
    
    if (gearPlatecompleteCicleCounter > 1) { // Iteramos los pulsos del plato para detectar el inicio y el fin vuelta.
      gearPlatecompleteCicleCounter--;
      gearPlateCompleteCicleValue +=gearPlateCurrentPulseValue;
    } else {
      //serialTrace(" --->>> full : ");
      gearPlateLastCompleteCicleValue=gearPlateCompleteCicleValue/GEAR_PLATE_PULSES;
      gearPlatecompleteCicleCounter = GEAR_PLATE_PULSES;
      gearPlateCompleteCicleValue=0;
      //serialTraceLn(gearPlateLastCompleteCicleValue);    
    }

    gearPlateLastPulseTime = millis();
  }
}

void changeModeInt() {
  //serialTraceLn(F("3 changeModeInt"));
  if (millis() > debounceChangeModeTime + DEBOUNCE_TIMETHRESHOLD_CHANGE_MODE) { //Debounce
    if (digitalRead(BRAKE_IN) == LOW) {
      powerModeChangeTrigger = POWERBRAKE_CHANGE_MODE; // cambia el modo de desaceleración
    } else {
      powerModeChangeTrigger = POWER_CHANGE_MODE; // cambia el modo de aceleración
    }
  }
  debounceChangeModeTime = millis();//Debounce
}

// MPU METHODS ***********************************************************************************

float getAxisAngle(int axis) {  
  //serialTraceLn(F("4 getAxisAngle"));
  //Calcular los angulos de inclinacion desde el acelerómetro;

  int16_t ax, ay, az; // Variables de control de Acelerómetro xyz
  //int16_t gx, gy, gz; // Variables de control de Giroscopio xyz   

  if (eStorage.mpuenabled) {
    mpu6050.getAcceleration(&ax, &ay, &az); // Lee los datos del acelerómetro. 
    //mpu6050.getRotation(&gx, &gy, &gz);  // Lee los datos del giroscopio
    //mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Lee los valores de acelerómetro y giroscopio
  } else {
    // Inicializamos valores de inclinación sin acelerómetro.
    ax = ZERO; ay = ZERO; az = 1;
    //gx = 0; gy = 0; gz = 1;
  }

  float accel_ang;
  float levelAxisAngle = LZERO;
  switch (axis) {
  case X:
    accel_ang = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
    break;
  case Y:
    accel_ang = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
    break;
  case Z:
    accel_ang = atan(az / sqrt(pow(ax, 2) + pow(ay, 2))) * (180.0 / 3.14);
    break;
  default:
    accel_ang = ZERO;
    break;
  }

  return accel_ang;
}

boolean isCrashDrop() { // Si El acelerómetro detecta que la bicicleta ha caido al suelo.
  //serialTrace(F("isCrashDrop 5_"));
  float accel_ang_x = getAxisAngle(crashAngleAxis);
  return (accel_ang_x > CRASH_ANGLE || accel_ang_x < -CRASH_ANGLE);
}

// POWER METHODS ***********************************************************************************

float analogInputToVolts(int inputValue){ // 1023 son los pasos que lee la entrada analógica
  return inputValue*5/1023;
}

int analogInputToDac(int inputValue){ // 4096 Son los pasos que tiene la salida del DAC
  return (int) inputValue*4096/5;
}


int getpwm(int inputValue){ // 240 -> 3,58V | 73 -> 1.108V
  return (inputValue / ( (eStorage.powerMaxValue-eStorage.powerMinValue) / 1023) ) * ( (240-73) / 1023);
}

int calculateMaxPower() {
  //serialTraceLn(F("6 - calculateMaxPower"));

  int maxPowerValueTmp;
  
  int currentAngleTmp = (byte) currentAngle>0?currentAngle:0; // Si el ángulo es negativo, lo ponemos a 0 para utilizar la mínima asistencia.
  //serialTraceLn(currentAngleTmp);
//  serialTrace(" > currentAngleTmp: ");
//  serialTrace(currentAngleTmp);
//  serialTrace(" | ");


  float angleAssistanceFraction = ((1.0 * eStorage.powerMaxValue) / eStorage.powerMinAssistenceValue) / eStorage.maxPowerAngle; // Calculamos la fracción del ángulo
//  serialTrace("angleAssistanceFraction: ");
//  serialTrace(angleAssistanceFraction);
//  serialTrace(" | ");
  
  // Calculamos el valor máximo de potencia según el ángulo adaptado a nivel
  int tmpAnglePower = angleAssistanceFraction * currentAngleTmp * eStorage.powerMinAssistenceValue;
//  serialTrace("tmpAnglePower: ");
//  serialTrace(tmpAnglePower);
//  serialTrace(" | ");

  int tmpPedalPower = (1.0 * (eStorage.powerMaxValue - eStorage.powerMinAssistenceValue) / (MIN_PLATE_TIME_SLOW - MAX_PLATE_TIME_FAST)) * (eStorage.powerMinAssistenceValue - (gearPlateLastCompleteCicleValue*GEAR_PLATE_PULSES)) + eStorage.powerMinAssistenceValue;
  if(tmpPedalPower<eStorage.powerMinAssistenceValue)
    tmpPedalPower=eStorage.powerMinAssistenceValue;
  /*serialTrace("gearPlateCompleteCicleTime: ");
  serialTrace(gearPlateCompleteCicleTime);
  serialTrace(" | tmppedalvalue: ");
  serialTraceLn(tmppedalvalue);*/
  
  if(cruisePower>tmpPedalPower) // si el crucero es mayor que la pedalada usamos en crucero.
    tmpPedalPower = cruisePower;
  
  
  // Decidimos cual es la potencia más alta.
  maxPowerValueTmp = tmpAnglePower > tmpPedalPower ? tmpAnglePower : tmpPedalPower;
  maxPowerValueTmp = (currentPowerValue + maxPowerValueTmp) / 2; //Calculamos la media entre el valor actual y el anterior
  // Reajustamos niveles para que se encuentren entre los valores máximos y mínimos permitidos.
  maxPowerValueTmp = maxPowerValueTmp > eStorage.powerMaxValue ? eStorage.powerMaxValue : maxPowerValueTmp;
  maxPowerValueTmp = maxPowerValueTmp < eStorage.powerMinAssistenceValue ? eStorage.powerMinAssistenceValue : maxPowerValueTmp;
  
  return maxPowerValueTmp;
} 

void updatePower() {
  //serialTraceLn(F("7 - updatePower"));
  if (currentPowerValue < maxPowerValue) { // incrementa progresivamente la potencia hasta la máxima.
    if (currentAngle <= CUT_POWER_ANGLE){
      //Si el ángulo es menos a -10º anula la potencia
      currentPowerValue=eStorage.powerMinValue+1;
      maxPowerValue=currentPowerValue;
    } else if (currentAngle <= -5){
      //Si el ángulo es menos a -5º anula la potencia
      currentPowerValue=(eStorage.powerMinAssistenceValue+eStorage.powerMinValue)/2; // aplicamos una pequeña potencia sacada de la media del valor mínimo de asistencia y el valor mínimo.
      maxPowerValue=currentPowerValue;
    } else { // Si el ángulo es mayor de -10º incrementamos la potencia prograsivamente.
      currentPowerValue = currentPowerValue + POWER_STEPTS[eStorage.powerMode];
      if (currentPowerValue > maxPowerValue) { // Si se supera el valor máximo de escala de potencia, este se regula.
        currentPowerValue = maxPowerValue;
      }      
    }

  } else if (currentPowerValue > maxPowerValue) { // decrementa progresivamente la potencia.
    currentPowerValue = currentPowerValue - POWER_STEPTS[eStorage.powerBrakeMode]; // Desacelera en el step más bajo.
    if (currentPowerValue < maxPowerValue) { // Si se rebasa el nivel mínimo de escala de potencia, este se regula.
      currentPowerValue = maxPowerValue;
    }
  }

  if(currentPowerValue < eStorage.powerMinValue){ // Ajusta los mínimos de potencias
    currentPowerValue = eStorage.powerMinValue;
  } else if(currentPowerValue > eStorage.powerMaxValue){ // Ajusta los máximos de potencias
    currentPowerValue = eStorage.powerMaxValue;
  }

  showMaxPowerScreen();

  // DAC DEPENDIENDO DE TENSIÓN EN VOLTIOS
  uint32_t valor=(4096/5)*currentThrottleValue;
  dac4725.setVoltage(valor, false); // fija voltaje en DAC
  // DAC - Convertir currentPowerValue a valor
  //dac4725.setVoltage(currentPowerValue, false); // Actualiza la salida con la potencia de acelerador por medio del DAC. // retocar rango de valores ya que el dac va de 0 a 4096 (0 - 5V)
  analogWrite(POWER_OUT, currentPowerValue / 20); // Actualiza la salida con la potencia de acelerador por medio del PWM.
}

// Listeners Methods ***********************************************************************************
void changeModeListener() {
if (powerModeChangeTrigger > UNDEFINED_CHANGE_MODE){
    //serialTraceLn(F("8 changeModeListener"));
    if (powerModeChangeTrigger == POWER_CHANGE_MODE) {
      if (eStorage.powerMode < (sizeof(POWER_STEPTS) / sizeof(int) - 1)) {
        eStorage.powerMode++;
      } else {
        eStorage.powerMode = ZERO;
      } 
      blinkLed(STATUS_LED_OUT, eStorage.powerMode + 1, (600 / (eStorage.powerMode + 1)));
    } else if (powerModeChangeTrigger == POWERBRAKE_CHANGE_MODE) {
      if (eStorage.powerBrakeMode < (sizeof(POWER_STEPTS) / sizeof(int) - 1)) {
        eStorage.powerBrakeMode++;
      } else {
        eStorage.powerBrakeMode = ZERO;
      }
      blinkLed(STATUS_LED_OUT, eStorage.powerBrakeMode + 1, (600 / (eStorage.powerBrakeMode + 1)));
    }

    currentPowerValue = eStorage.powerMinValue;
    /*
    messageBuilder = F("CHANGE PM|PBM ");
    messageBuilder = messageBuilder + eStorage.powerMode;
    messageBuilder = messageBuilder + F(" | ");
    messageBuilder = messageBuilder + eStorage.powerBrakeMode;
    showAlertScreen(messageBuilder);  // TODO VER PORQUE NO FUNCIONA LA LLAMADA
    */
    powerModeChangeTrigger=ZERO;

  }
}


// EEPROM METHODS ***********************************************************************************
void initDefaultEepromData() {
  //serialTraceLn(F("10 initDefaultEepromData"));
  //serialTraceLn("\n > Initializing default config...");
  blinkLed(STATUS_LED_OUT, 150, 3);
  eStorage.powerMode = POWER_STEPTS_DEFAULT_POSITION;
  eStorage.powerBrakeMode = POWER_STEPTS_DEFAULT_POSITION;
  eStorage.maxPowerAngle = DEFAULT_MAX_POWER_ANGLE;
  eStorage.powerAngleAxis=X;
  eStorage.powerAngleAxisInverter=true;
  eStorage.XAccelOffset=ZERO;
  eStorage.YAccelOffset=ZERO;
  eStorage.ZAccelOffset=ZERO;
  eStorage.XGyroOffset=ZERO;
  eStorage.YGyroOffset=ZERO;
  eStorage.ZGyroOffset=ZERO;
  eStorage.mpuenabled = false;
  eStorage.powerMaxValue = DEFAULT_POWER_MAX_VALUE;
  eStorage.powerMinAssistenceValue = DEFAULT_POWER_MIN_ASSISTENCE_VALUE;
  eStorage.powerMinValue = DEFAULT_POWER_MIN_VALUE;
  eStorage.powerBrakeDivider = DEFAULT_POWER_BRAKE_DIVIDER;

  updateEepromData();
  blinkLed(STATUS_LED_OUT, 15, 20);
}

void updateEepromData() {
  //serialTraceLn(F("11 updateEepromData"));
  EEPROM.put(EEPROM_INIT_ADDRESS, eStorage); // Actualizan los datos de modos entrega de potencia en la eeprom
  showEepromDataScreen();
}


// SCREEN METHODS ***********************************************************************************

void showPedalIcon(uint16_t color) {
  //serialTraceLn(F("12 showPedalIcon"));
  for (byte y = 27; y <= 31; y++) {
    for (byte x = 122; x < 126; x++) {
      oled1306.drawPixel(x, y, color);
    }
  }
  oled1306.display();
}

void cleanDisplay(){
  //serialTraceLn(F("13 cleanDisplay"));
  // Clean display
  for (byte y = ZERO; y <= 31; y++) {
    for (byte x = 12; x < 126; x++) {
      oled1306.drawPixel(x, y, BLACK);
    }
  }
}

void showAlertScreen(String message) {
  //serialTraceLn(F("14 showAlertScreen"));
  byte pos = 21 - message.length();
  if(pos>2)
    pos = pos / 2;
  cleanDisplay();
  oled1306.setCursor(pos*6, LINE_1);
  oled1306.print(message);
  oled1306.display();
}

void showMaxPowerScreen() {
  //serialTraceLn(F("15 showMaxPowerScreen"));
  if (currentPowerValue > eStorage.powerMinValue && currentPowerValue < eStorage.powerMaxValue) {  // Solo se muestra si se está pedaleando o si la potencia no es la mínima
    float coutput = ((float)(maxPowerValue * 3.58) / DEFAULT_POWER_MAX_VALUE);
    float coutput1 = ((float)(currentPowerValue * 3.58) / DEFAULT_POWER_MAX_VALUE);

    oled1306.clearDisplay();
    oled1306.setCursor(START_LINE, LINE_0);
    oled1306.print(eStorage.powerAngleAxis==X?"> X: ":"> Y: ");
    oled1306.print(currentAngle);
    oled1306.print(F(" PLSE:"));
    if(millis()-gearPlateLastPulseTime<MAX_TIME_BETWEEN_GEAR_PLATE_PULSES)
      oled1306.print(gearPlateLastCompleteCicleValue*GEAR_PLATE_PULSES);
    else
      oled1306.print(F("???"));

    oled1306.setCursor(START_LINE, LINE_1);
    oled1306.print(F("> maxPwr: "));
    oled1306.print(coutput);
    //oled1306.print(maxPowerValue);
    oled1306.setCursor(START_LINE, LINE_2);
    oled1306.print(F("> curPwr: "));
    oled1306.print(coutput1);
    //oled1306.print(currentPowerValue);

    oled1306.display();
  }
}

void showEepromDataScreen() {
  //serialTraceLn(F("16 showEepromDataScreen"));
  oled1306.setCursor(START_LINE, LINE_1);
  oled1306.print(F("> pwr M|BM|BD: "));
  oled1306.print(eStorage.powerMode);
  oled1306.print(F("|"));
  oled1306.print(eStorage.powerBrakeMode);
  oled1306.print(F("|"));
  oled1306.print(eStorage.powerBrakeDivider);
  
  oled1306.setCursor(START_LINE, LINE_2);
  if (eStorage.mpuenabled) {
    oled1306.print(F("> mPW:"));
    oled1306.print(eStorage.maxPowerAngle);
  } else {
    oled1306.print(F("> mpu DIS"));
  }
  oled1306.print(F(" > SP: "));
  oled1306.print(SERIAL_PORT);
  oled1306.display();
  delay(2000);
}

void showHomeScreen() {
  //serialTraceLn(F("17 showHomeScreen"));
  oled1306.clearDisplay();
  oled1306.setTextSize(1);
  oled1306.setTextColor(WHITE);

  oled1306.setCursor(15, LINE_0);
  oled1306.print(F("FIIDO ASSISTANCE"));
  oled1306.setCursor(40, LINE_1);
  oled1306.print(F("PROJECT"));
  oled1306.setCursor(70, LINE_2);
  oled1306.print(VERSION);
  oled1306.display();
  oled1306.setCursor(90, LINE_1);
  oled1306.print(F("*"));
  oled1306.display();
  delay(90);
  oled1306.print(F("*"));
  oled1306.display();
  delay(90);
  oled1306.setCursor(20, LINE_1);
  oled1306.print(F("*"));
  oled1306.display();
  delay(90);
  oled1306.print(F("*"));
  oled1306.display();
  delay(1500);
  oled1306.clearDisplay();
  oled1306.display();
}

// INIT METHODS ***********************************************************************************

void initFlashLeds() {
  //serialTraceLn(F("18 initFlashLeds"));
  //serialTrace(F(" ."));
  blinkLed(STATUS_LED_OUT, eStorage.powerMode + 1, 60); // Muestra el modo de aceleración.
  runDelay(1000);
  blinkLed(STATUS_LED_OUT, eStorage.powerBrakeMode + 1, 60); // Muestra el modo de desaceleración.
  runDelay(1500);
  digitalWrite(STATUS_LED_OUT, HIGH); // Muestra el led de fin de inicialización.
  runDelay(500);
  digitalWrite(STATUS_LED_OUT, LOW);
  //serialTraceLn(F("."));
}

// UTILS METHODS ***********************************************************************************

void runDelay(int dlay) { // Ejecuta un delay y sacamos puntos por la consola.
  byte repeat = dlay / 100;
  for (byte i = ZERO; i <= repeat; i++) {
    delay(100);
    //serialTrace(F("."));
  }
}

void blinkLed(byte ledPin, byte repeats, int time) { // Ejecuta un parpadeo en el led durante x repeticiones y con una frecuencia de x_ms.
  for (byte i = ZERO; i < repeats; i++) {
    digitalWrite(ledPin, HIGH);
    delay(time);
    digitalWrite(ledPin, LOW);
    delay(time);
  }
}

template<class T>
void serialTrace(T message){
  if(traceOn)
    Serial.print(message);
}

template<class T>
void serialTraceLn(T message){
  if(traceOn)
    Serial.println(message);
}

int makeDiscount(int value, byte percent){
  return (value * (100.0-percent) / 100);
}

void checkI2cDevices() {
  byte error, address;
  byte nDevices;
  serialTraceLn(F("Scanning..."));

  nDevices = 0;
  for(address = 1; address < 127; address++ ){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      serialTrace(F("I2C device found at address 0x"));
      if (address<16) 
        serialTrace(F("0"));
        serialTrace(String(address,HEX));
        serialTraceLn("  !");
        nDevices++;
    } else if (error==4) {
      serialTrace("Unknown error at address 0x");
      if (address<16) 
        serialTrace(F("0"));
      serialTraceLn(String(address,HEX));
    }    
  }
  serialTrace("> ");
  serialTrace(String(nDevices, DEC));
  serialTraceLn(F(" I2C devices found\n"));
}

void initThrotleMinMax(){
  oled1306.clearDisplay();
  oled1306.setCursor(START_LINE, LINE_0);
  oled1306.print("Calibrando acelerador...");
  oled1306.display();
  int cont = 0;
  int initThrotle;
  int endThrotle;

  while (true){
    initThrotle = analogRead(THROTTLE_IN)*POWER_INPUT_MULTIPLIER;
    delay(500);
    endThrotle = analogRead(THROTTLE_IN)*POWER_INPUT_MULTIPLIER;
    if(initThrotle==endThrotle){
      cont++;
      if(cont==1){
        oled1306.setCursor(START_LINE, LINE_1);
        oled1306.print("MN: ");
        oled1306.print(initThrotle);
        eStorage.powerMinValue=initThrotle;
        oled1306.print(" > ");
        oled1306.display();
        delay(3000);
      }else if(cont==2){
        oled1306.print("MX: ");
        oled1306.print(initThrotle);
        eStorage.powerMaxValue=initThrotle;
        oled1306.display();
        
      }
    }
    if(cont>1){
      oled1306.setCursor(START_LINE, LINE_2);
      oled1306.print("MID: ");        
      eStorage.powerMinAssistenceValue=((eStorage.powerMaxValue-eStorage.powerMinValue)/2)+eStorage.powerMinValue;
      oled1306.print(eStorage.powerMinAssistenceValue);
      oled1306.display();
      break;
    }
    
  }
}

void showThrotleMinMax(){
  oled1306.clearDisplay();
  oled1306.setCursor(START_LINE, LINE_0);
  oled1306.print("> MAX: ");
  oled1306.print(eStorage.powerMaxValue);
  oled1306.setCursor(START_LINE, LINE_1);
  oled1306.print("> MED: ");
  oled1306.print(eStorage.powerMinAssistenceValue);
  oled1306.setCursor(START_LINE, LINE_2);
  oled1306.print("> MIN: ");
  oled1306.print(eStorage.powerMinValue);
}
// SERIAL AT METHODS ***********************************************************************************

void serialAtCommandListener() { // Espera un comando AT con un buffer de hasta SERIAL_BUFFER_SIZE (32)

  static byte ndx = ZERO;
  char endMarker = '\n';
  char rc;
  while (Serial.available() > ZERO && newSerialDataFlag == false) {
    //serialTraceLn(F("19 serialAtCommandListener"));
    rc = Serial.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= SERIAL_BUFFER_SIZE) {
        ndx = SERIAL_BUFFER_SIZE - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      String receivedCharsStr = receivedChars;
      if (receivedCharsStr.startsWith("at") || receivedCharsStr.startsWith("AT")) {
        newSerialDataFlag = true;
      }
    }
  }
}


/*
  AT Commands
  at+save
  at+init
  at+eelist
  at+pwdup=[0-5]
  at+pwddw=[0-5]
  at+mpuon
  at+mpuoff
  at+anglemaxpwr=[0-30]
  at+level
*/
void ATCommandsManager() {

  if (newSerialDataFlag == true) {
    //serialTraceLn(F("20 ATCommandsManager"));
    blinkLed(STATUS_LED_OUT, 1, 500);
    String command = getAtData(receivedChars, '=' , 0);
    command.toLowerCase();
    int value = getAtData(receivedChars, '=' , 1).toInt();

    oled1306.clearDisplay();
    oled1306.setCursor(START_LINE, LINE_0);
    oled1306.print(F("> "));
    String tmpCommand = command;
    tmpCommand.toUpperCase();
    oled1306.print(tmpCommand);
    if (value > 0) {
      oled1306.print(F("?"));
      oled1306.print(value);
    }
    oled1306.setCursor(START_LINE, LINE_1);

    if (command.indexOf("at+save") > -1) {
      updateEepromData();

    } else if (command.indexOf("at+init") > -1) {
      initDefaultEepromData();

    } else if (command.indexOf("at+throtleinit") > -1) {
      initThrotleMinMax();
      
    } else if (command.indexOf("at+showthrotle") > -1) {
      showThrotleMinMax();

    } else if (command.indexOf("at+eelist") > -1) {
      //printEeprom();
      showEepromDataScreen();

    } else if (command.indexOf("at+brakediv") > -1) {
      eStorage.powerBrakeDivider = value>DEFAULT_POWER_BRAKE_DIVIDER?value:DEFAULT_POWER_BRAKE_DIVIDER;
      
    } else if (command.indexOf("at+pwrup") > -1) { // modo de incremento de potencia progresiva.
      eStorage.powerMode = (value < (sizeof(POWER_STEPTS) / 2)) ? value : POWER_STEPTS_DEFAULT_POSITION;
      oled1306.print(F("> powerBrakeMode: "));
      oled1306.print(eStorage.powerMode);

    } else if (command.indexOf("at+pwrdw") > -1) { // modo de decremento de potencia progresiva.
      eStorage.powerBrakeMode = (value < (sizeof(POWER_STEPTS) / 2)) ? value : POWER_STEPTS_DEFAULT_POSITION;
      oled1306.print(F("> powerBrakeMode: "));
      oled1306.print(eStorage.powerBrakeMode);

    } else if (command.indexOf("at+mpuon") > -1) {
      eStorage.mpuenabled = true;

    } else if (command.indexOf("at+mpuoff") > -1) {
      eStorage.mpuenabled = false;

    } else if (command.indexOf("at+shutdown") > -1) {
      asm volatile ("jmp 0");
      
    } else if (command.indexOf("at+i2clist") > -1) {
      checkI2cDevices();
      
    } else if (command.indexOf("at+anglemaxpwr") > -1) { // ángulo para la máxima potencia;
        if (eStorage.mpuenabled) {
          eStorage.maxPowerAngle = value < DEFAULT_MAX_POWER_ANGLE ? value : DEFAULT_MAX_POWER_ANGLE;
          oled1306.print(F("> maxPowerAngle: "));
          oled1306.print(eStorage.maxPowerAngle);
        }
    } else if (command.indexOf("at+angledir") > -1) { // ángulo de control de inclinación; // Y X -Y -X // 1 2 -1 -2 // REVISAR N S E O

      eStorage.powerAngleAxis=(value<-1)?Y:X;
      eStorage.powerAngleAxisInverter=(value>0);
      crashAngleAxis=(eStorage.powerAngleAxis == X)?Y:X; // Eje que controla el ángulo de control de caida

    } else if (command.indexOf("at+mpucalibrate") > -1) { // Calibrar posición de placa.
        if (eStorage.mpuenabled)
          calibrate(mpu6050);
          
    } else {
      oled1306.print(F("Unknown command."));
      
    }
    
    newSerialDataFlag = false;
    blinkLed(STATUS_LED_OUT, 15, 30);
  }
}
  
String getAtData(String data, char separator, int index) { // Split string and get index data.
  //serialTraceLn(F("21 getAtData"));
  int found = ZERO;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (byte i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void calibrate(MPU6050 sensor) {

  // Acelerometer
  int ax, ay, az; // Raw acel
  long f_ax,f_ay, f_az;
  int p_ax, p_ay, p_az;
  
  // Gyro
  int gx, gy, gz; // Raw giro
  long f_gx,f_gy, f_gz;
  int p_gx, p_gy, p_gz;
  
  // Offsets
  int ax_o,ay_o,az_o; //offsets
  int gx_o,gy_o,gz_o;

  int counter=0;

  // Read mpu values
  ax_o=sensor.getXAccelOffset();
  ay_o=sensor.getYAccelOffset();
  az_o=sensor.getZAccelOffset();
  gx_o=sensor.getXGyroOffset();
  gy_o=sensor.getYGyroOffset();
  gz_o=sensor.getZGyroOffset();
  
  long inittime = millis();
  boolean calibdone=true;
  while(calibdone){
    // Leer las aceleraciones y velocidades angulares
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);
  
    // Filtrar las lecturas
    f_ax = f_ax-(f_ax>>5)+ax;
    p_ax = f_ax>>5;
  
    f_ay = f_ay-(f_ay>>5)+ay;
    p_ay = f_ay>>5;
  
    f_az = f_az-(f_az>>5)+az;
    p_az = f_az>>5;
  
    f_gx = f_gx-(f_gx>>3)+gx;
    p_gx = f_gx>>3;
  
    f_gy = f_gy-(f_gy>>3)+gy;
    p_gy = f_gy>>3;
  
    f_gz = f_gz-(f_gz>>3)+gz;
    p_gz = f_gz>>3;
  
    //Cada 100 lecturas corregir el offset
    if (counter==100){
      //Mostrar las lecturas separadas por un [tab]
      oled1306.clearDisplay();
      oled1306.setCursor(START_LINE, LINE_0);
      oled1306.print(p_ax);
      oled1306.print("|");
      oled1306.print(p_ay);
      oled1306.print("|");
      oled1306.print(p_az);
      oled1306.setCursor(START_LINE, LINE_1);      
      oled1306.print(p_gx);
      oled1306.print("|");
      oled1306.print(p_gy);
      oled1306.print("|");
      oled1306.println(p_gz);
  
      //Calibrar el acelerometro a 1g en el eje z (ajustar el offset)
      if (p_ax>0) 
        ax_o--;
      else 
        ax_o++;
        
      if (p_ay>0) 
        ay_o--;
      else 
        ay_o++;
      if (p_az-16384>0) 
        az_o--;
      else 
        az_o++;
      
      sensor.setXAccelOffset(ax_o);
      sensor.setYAccelOffset(ay_o);
      sensor.setZAccelOffset(az_o);
  
      //Calibrar el giroscopio a 0º/s en todos los ejes (ajustar el offset)
      if (p_gx>0) 
        gx_o--;
      else 
        gx_o++;
      if (p_gy>0) 
        gy_o--;
      else 
        gy_o++;
      if (p_gz>0) 
        gz_o--;
      else 
        gz_o++;
      
      sensor.setXGyroOffset(gx_o);
      sensor.setYGyroOffset(gy_o);
      sensor.setZGyroOffset(gz_o);    
  
      //0 0 +16384
      if(p_ax <= 5 && p_ax >= -5 && p_ay <= 5 && p_ay >= -5 && p_az <= 16394 && p_az >= 16374){
        oled1306.setCursor(START_LINE, LINE_2);      
        oled1306.print("\t Calibration Done ");      
        oled1306.print((millis()-inittime)/1000);
        oled1306.print("s");  
        calibdone=false;  
      }
      counter=0;
    }
    counter++;
  }
}
