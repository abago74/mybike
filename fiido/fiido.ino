const String VERSION = "V_2.6.3";

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
const byte DEBOUNCE_TIMETHRESHOLD_GEAR_PLATE = 20; // Tiempo de espera entre pulsos de interrupcion 49ms entre pulsación. 100 Pedaladas(GEAR_PLATE_PULSES*100=1200) por minuto (20 pulsos por segundo). 50ms de debounce para detectar el máximo de 20 pulsos por segundo.
const int DEBOUNCE_TIMETHRESHOLD_CHANGE_MODE = 1200; // Tiempo de espera entre pulsos de interrupcion 1200ms entre pulsación

// ********************** POWER

const int POWER_INPUT_MULTIPLIER = 10; // Máxima potencia del A0 732 - PWM. 4800 -> 3.58v
const int DEFAULT_POWER_MAX_VALUE = 7320; // Máxima potencia del PWM. 4800 -> 3.58v
const int DEFAULT_POWER_MIN_ASSISTENCE_VALUE = 4790; // Mínima potencia de asistencia ?.???v
const int DEFAULT_POWER_MIN_VALUE = 2260; // Mínima potencia del PWM. 1470 -> 1.108v


const int POWER_STEPTS[] = {2, 4, 8, 16, 32, 64, 128, 256, 512}; // Incremento de potencia de la salida POWER por cada paso.
const byte POWER_STEPTS_DEFAULT_POSITION = 3; // El rango (3)=16 es el valor por defecto
const byte UNDEFINED_CHANGE_MODE = 0, POWER_CHANGE_MODE = 1, POWERBRAKE_CHANGE_MODE = 2;

// ********************** PAS PLATE
//**const byte GEAR_PLATE_PULSES = 3; // Pulsos proporcionados por el plato en cada vuelta. 10+ en el caso de la fiido
const int PLATE_TIME_FAST = 60; // Tiempo que se tarda en dar una vuelta de plato
const int PLATE_TIME_SLOW = 200; // Tiempo que se tarda en dar una vuelta de plato
const int MAX_TIME_BETWEEN_GEAR_PLATE_PULSES = 300; // Tiempo mínimo en ms que se tiene que estar pedaleando para empezar a recibir potencia.
const byte MEDIA_PULSE_COUNTER=20; // Pulsos leidos para sacar la media de cadencia.
const byte START_PEDAL_MEDIA_COUNTER=6;

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

  boolean plotterOn=false;
  boolean traceOn=false;
  
};

#define max(a,b) ((a)>(b)?(a):(b))

// ------------------------- VARS -------------------------------

// ********************** Serial Input
char receivedChars[SERIAL_BUFFER_SIZE]; // an array to store the received data ut to SERIAL_BUFFER_SIZE
boolean newSerialDataFlag = false; // Flag que controla cuando entran datos por el puerto seria que dispara la interpretación de comandos AT

// **********************  DEVOUNCE
unsigned long debounceLastGearPlatePulseTime; // Almacena el millis() en el que se tomo medida del pulso anterior del sensor de pedalada para controlar el debounce.
unsigned long debounceChangeModeTime; // Almacena el millis() en el que se tomo medida del último pulso de cambio de modo para controlar el debounce.

// ********************** PAS PLATE
unsigned long gearPlateLastPulseTime; // Tiempo tomado en el último pulso.
unsigned long gearPlatePreviousPulseTime; // millis del pulso anterior.
volatile unsigned long continuousCyclePulseCounter; // contador que se incrementa según se está pedaleando

byte mediaPulseCounter; // Ciclos para realizar la media de los tiempos entre pulsos.
unsigned int calculatedPulseContainer= 0; // Almacena la suma de todos los tiempos de los pulsos para realizar el cáculo de la media.
byte currentPulseCalculatedValue; // Contenedor de calculo de pulso


// ********************** POWER
unsigned int maxPowerValue; // Máxima potencia calculada según pedalada y la inclinación.
unsigned int currentPowerValue; // Potencia actual
int currentThrottleValue; // Lectura de tensión de acelerador.
unsigned int cruisePower;

// ********************** MODE
int powerModeChangeTrigger; // Trigger de solicitud de cambio de modo . La variable es cambiada de estado por el método changeModeInt lanzado por la interrupción para detectar el tipo. Valores 0/1/2.

// ********************** MPU6050
float currentAngle; //Variable que almacena el ángulo de potencia.
long devounceMpuTimeThreshold; // Tiempo entre las medidas de ángulo
byte crashAngleAxis; // Eje que controla el ángulo de control de caida

// ********************** DEFAULT
EStorage eStorage; // Instancia de datos de configuración eeprom

boolean flaginit=false;

float tmpanglepowermedia; // TODO REMOVE IF NOT NEEDED

// init
void setup() {

  // Lee configuración desde la eeprom.
  EEPROM.get(EEPROM_INIT_ADDRESS, eStorage); // Captura los valores desde la eeprom

  dac4725.begin(DAC_ADDR); // Inicializa el DAC

  tmpanglepowermedia = ((eStorage.powerMaxValue-eStorage.powerMinValue)/DEFAULT_MAX_POWER_ANGLE); // Calcula promedio por ángulos
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

  //Inicializamos los pines
  pinMode(LED_BUILTIN, OUTPUT); // Salida led modo/estado. //STATUS_LED_OUT
  pinMode(POWER_OUT, OUTPUT); // Salida PWM acelerador variable.

  pinMode(BRAKE_IN, INPUT_PULLUP); // Entrada de control de freno.
  pinMode(GEAR_PLATE_IN, INPUT_PULLUP); // Entrada Interrupción  - control de pedalaleo.
  pinMode(MODE_IN, INPUT_PULLUP); // Entrada Interrupción - control nodo de operación.

  pinMode(THROTTLE_IN, INPUT); // Entrada lectura potencia acelerador.

  // Inicialización de variables
  continuousCyclePulseCounter=ZERO;
  mediaPulseCounter=MEDIA_PULSE_COUNTER;
  debounceLastGearPlatePulseTime = ZERO; // Inicializamos el timer de control de rebote.
  debounceChangeModeTime = ZERO; // Inicializamos el timer de control de rebote.
  maxPowerValue=DEFAULT_POWER_MIN_VALUE;
  currentPowerValue=DEFAULT_POWER_MIN_VALUE;
  powerModeChangeTrigger = ZERO;
  currentAngle = LZERO;
  devounceMpuTimeThreshold = ZERO; // Tiempo entre las medidas de ángulo
  cruisePower=0; // Desactivamos la potencia de crucero. 
  crashAngleAxis=(eStorage.powerAngleAxis == X)?Y:X; // Eje que controla el ángulo de control de caida

  //initFlashLeds(); // Ejecuta los leds de inicio de script. y muestra los modos.

  // Definición de interrupciones
  attachInterrupt(digitalPinToInterrupt(GEAR_PLATE_IN), gearPlatePulseInt, CHANGE); // Definición de la interrupción para la entrada de pin de pedal.
  attachInterrupt(digitalPinToInterrupt(MODE_IN), changeModeInt, FALLING); // Definición de la interrupción para la entrada de pin de cambio de modo.
  oled1306.print(F(" |"));
  oled1306.display();

  showEepromDataScreen();

  // Carga el inicializador de configuración manual.
  if (digitalRead(BRAKE_IN) == LOW) {
    flaginit=true;
  }
}
int cont=0;
int count=0;

void loop(){
  if(flaginit){ // inicializamos dejando pulsado el pin de freno al iniciar. Después contamos los pulsos para detectar que comando mandar.
    initThrotleMinMax();
    flaginit=false;
  }else{  
    digitalWrite(STATUS_LED_OUT, !digitalRead(STATUS_LED_OUT)); 

    changeModeListener(); // Controla powerModeChangeTrigger para cambiar los modos
    serialAtCommandListener(); // Lee comandos AT por puerto serie
  
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
  
      String messageBuilder = F(" BRAKE!!! ");
      messageBuilder = messageBuilder + currentPowerValue;
      showAlertScreen(messageBuilder);
      
      blinkLed(STATUS_LED_OUT, 10, 10); // Muestra el testigo de frenado. 100ms
          
    }else{
      
      if(continuousCyclePulseCounter>0){ // Si se está pedaleando
        if( (long) millis()-gearPlateLastPulseTime < MAX_TIME_BETWEEN_GEAR_PLATE_PULSES){ //tiempo desde el último pulso < 300
          if(gearPlatePreviousPulseTime != gearPlateLastPulseTime){ // Si el valor del pulso ha cambiado desde el último loop.
            
            if(mediaPulseCounter-- > 0){ // Si decrementamos el contador y no se ha completado el ciclo de cálculo.
              calculatedPulseContainer += (gearPlateLastPulseTime - gearPlatePreviousPulseTime) ; // Incrementamos el contenedor de datos
            }else{ // Si el ciclo de cálculo ha terminado, obtenemos la media.
              currentPulseCalculatedValue = calculatedPulseContainer / MEDIA_PULSE_COUNTER;
              calculatedPulseContainer = (gearPlateLastPulseTime - gearPlatePreviousPulseTime) ;
              mediaPulseCounter = MEDIA_PULSE_COUNTER;
            }
            gearPlatePreviousPulseTime=gearPlateLastPulseTime;
          }  
          
          maxPowerValue = calculateMaxPower();
                  
          serialTrace("pedaleando: (");
          serialTrace(continuousCyclePulseCounter);
          serialTrace(") cpcv: ");
          serialTrace(currentPulseCalculatedValue);
          serialTraceLn("");
          
        } else { // Si no se está pedaleando
          continuousCyclePulseCounter = 1; // Marcamos a 1 como flag de control para solo ejecutar una vez.
          if(--continuousCyclePulseCounter == 0){ 
            serialTraceLn("no pedaleando ");
            currentPulseCalculatedValue=PLATE_TIME_SLOW; // máximo tiempo de cadencia
            maxPowerValue=eStorage.powerMinValue; // mínima potencia de acelerador
          }
        }
      } 
    }
    
    updatePower(); // actualizamos la potencia de salida.
    showMaxPowerScreen();
    
    if(eStorage.plotterOn){
        
        //plotter/
        //Serial.print((((int) currentAngle)*100)+DEFAULT_POWER_MAX_VALUE+1000);
        float t = eStorage.powerMinValue + (currentAngle * tmpanglepowermedia);
        if( t > eStorage.powerMaxValue )
        t =eStorage.powerMaxValue;
        Serial.print((int) t);
        Serial.print("\t");
          
        //plotter
        Serial.print(maxPowerValue);
        Serial.print("\t");
        
        //plotter
        Serial.print(currentPowerValue);
        Serial.print("\t");
        
        Serial.println(currentThrottleValue);
    }         
    
  }
}

// Interruption Methods ***********************************************************************************
void gearPlatePulseInt() { // Método que incrementa el contador de pedal en caso de pulso por el pin de interrupcion.

  //serialTraceLn(F("2 gearPlatePulseInt"));
  if (millis() > gearPlateLastPulseTime + DEBOUNCE_TIMETHRESHOLD_GEAR_PLATE) { // Debounce Si ha pasado el tiempo de control de rebote marca pulso de pedaleo.
    if(continuousCyclePulseCounter++ > START_PEDAL_MEDIA_COUNTER) // si se han detectado al menos n pulsos empieza a calcular media de pedalada
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

int analogInputRangeToDacRange(int inputRangeValue){ // 4096 Son los pasos que tiene la salida del DAC
  // inputRangeValue -> convert to volts -> convert volts to dat range
  return (int) (1.0 * inputRangeValue) * (5.0 / 1023) * (4096 / 5);;
}

float DacRangeToVolts(int dacRangeValue){ // 4096 Son los pasos que tiene la salida del DAC  
  return dacRangeValue * 5.0 / 4096; 
}

int calculateMaxPower() {
  //serialTraceLn(F("6 - calculateMaxPower"));
  int mintomax = eStorage.powerMaxValue - eStorage.powerMinValue;
  float mintomaxforangle = 1.0 * mintomax / eStorage.maxPowerAngle;
  float mintomaxfopedal = 1.0 * mintomax / (PLATE_TIME_SLOW - PLATE_TIME_FAST);
  
  unsigned int currentPulseCalculatedValueTmp = currentPulseCalculatedValue <= PLATE_TIME_FAST ? PLATE_TIME_FAST : currentPulseCalculatedValue;
  currentPulseCalculatedValueTmp = currentPulseCalculatedValueTmp >= PLATE_TIME_SLOW ? PLATE_TIME_SLOW : currentPulseCalculatedValueTmp;
  currentPulseCalculatedValueTmp = PLATE_TIME_SLOW - currentPulseCalculatedValueTmp; // reverse pedal;
  int tmpPedalPower = eStorage.powerMinValue + ((int) (mintomaxfopedal * currentPulseCalculatedValueTmp));

  byte currentAngleTmp = (int) currentAngle <= ZERO ? ZERO : currentAngle; // Si el ángulo es negativo, lo ponemos a 0 para utilizar la mínima asistencia.
  currentAngleTmp = currentAngleTmp >= eStorage.maxPowerAngle ? eStorage.maxPowerAngle : currentAngleTmp; // Si el ángulo es mayor de 30 nivela a máximo.
  // Calculamos el valor máximo de potencia según el ángulo adaptado a nivel
  int tmpAnglePower = eStorage.powerMinValue + ((int) (mintomaxforangle * currentAngleTmp));

  currentThrottleValue= analogInputRangeToDacRange(analogRead(THROTTLE_IN)); // Lee el valor analógico del acelerador. 
 
  int maxPowerValueTmp = max(tmpPedalPower, max(tmpAnglePower,currentThrottleValue));
  
  //if(cruisePower > 0 && cruisePower > maxPowerValueTmp) // si el crucero esta activo y la velocidad calculada es menor que la velocidad de crucero la seleccionamos.
  //  return cruisePower;

  serialTrace(" > currentAngleTmp: ");
  serialTrace(currentAngle);
  serialTrace(F("º # "));
  serialTrace(currentAngleTmp);
  serialTrace(F("º | "));
  serialTrace(F("Angle: "));
  serialTrace(tmpAnglePower); 
  serialTrace(" Pedal: ");
  serialTrace(tmpPedalPower);
  serialTrace(" Throttle: ");
  serialTrace(currentThrottleValue);  
  serialTrace(" maxPowerValueTmp: ");
  serialTrace(maxPowerValueTmp);    

  // Reajustamos niveles para que se encuentren entre los valores máximos y mínimos permitidos.
  maxPowerValueTmp = maxPowerValueTmp > eStorage.powerMaxValue ? eStorage.powerMaxValue : maxPowerValueTmp;
  maxPowerValueTmp = maxPowerValueTmp < eStorage.powerMinAssistenceValue ? eStorage.powerMinAssistenceValue : maxPowerValueTmp;

  serialTrace(" adjust: ");
  serialTrace(maxPowerValueTmp);  
  serialTrace(F("\t")); 
  
  return maxPowerValueTmp;
} 

void updatePower() {
  serialTrace(F("7 - updatePower"));
  if (currentPowerValue < maxPowerValue) { // incrementa progresivamente la potencia hasta la máxima.
    if (currentAngle <= CUT_POWER_ANGLE){
      //Si el ángulo es menos a -10º anula la potencia
      currentPowerValue = eStorage.powerMinValue + 1;
      maxPowerValue = currentPowerValue;
    } else if (currentAngle <= -5){
      //Si el ángulo es menos a -5º anula la potencia
      currentPowerValue=(eStorage.powerMinAssistenceValue + eStorage.powerMinValue) / 2; // aplicamos una pequeña potencia sacada de la media del valor mínimo de asistencia y el valor mínimo.
      maxPowerValue = currentPowerValue;
    } else { // Si el ángulo es mayor de -5º incrementamos la potencia prograsivamente.
      currentPowerValue = currentPowerValue + POWER_STEPTS[eStorage.powerMode];
      if (currentPowerValue > maxPowerValue) { // Si se supera el valor máximo de escala de potencia, este se regula.
        currentPowerValue = maxPowerValue;
      }      
    }

  } else if (currentPowerValue >= maxPowerValue) { // decrementa progresivamente la potencia.
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

  dac4725.setVoltage(currentPowerValue, false); // fija voltaje en DAC

  serialTraceLn(currentPowerValue);
  // DAC DEPENDIENDO DE TENSIÓN EN VOLTIOS
  //int ctv = analogInputRangeToDacRange(currentThrottleValue);
  //dac4725.setVoltage(ctv, false); // fija voltaje en DAC
  // DAC - Convertir currentPowerValue a valor
  //dac4725.setVoltage(currentPowerValue, false); // Actualiza la salida con la potencia de acelerador por medio del DAC. // retocar rango de valores ya que el dac va de 0 a 4096 (0 - 5V)
  //analogWrite(POWER_OUT, currentPowerValue / 20); // Actualiza la salida con la potencia de acelerador por medio del PWM.
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
    float coutput = DacRangeToVolts(maxPowerValue);
    float coutput1 = DacRangeToVolts(currentPowerValue);

    oled1306.clearDisplay();
    oled1306.setCursor(START_LINE, LINE_0);
    oled1306.print(eStorage.powerAngleAxis==X?"> X: ":"> Y: ");
    oled1306.print(currentAngle);
    oled1306.print(F(" PLSE:"));
    if(millis()-gearPlateLastPulseTime<MAX_TIME_BETWEEN_GEAR_PLATE_PULSES)
      oled1306.print(currentPulseCalculatedValue);
    else
      oled1306.print(F("???"));

    oled1306.setCursor(START_LINE, LINE_1);
    oled1306.print(F("> maxPwr: "));
    oled1306.print(coutput);
    oled1306.setCursor(START_LINE, LINE_2);
    oled1306.print(F("> curPwr: "));
    oled1306.print(coutput1);
    oled1306.display();
  }
}

void showEepromDataScreen() {
  //serialTraceLn(F("16 showEepromDataScreen"));
  //oled1306.clearDisplay();
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
  oled1306.setCursor(124 - (VERSION.length()*6), LINE_2);
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
    initThrotle = analogRead(THROTTLE_IN);
    delay(100);
    endThrotle = analogRead(THROTTLE_IN);
    if(initThrotle==endThrotle){
      cont++;
      if(cont==1){
        oled1306.setCursor(START_LINE, LINE_1);
        oled1306.print("MIN: "); // Min value
        eStorage.powerMinValue=analogInputRangeToDacRange(initThrotle);
        dac4725.setVoltage(eStorage.powerMinValue, true); // Seteamos el dac al mínimo y guardamos el valor en la eeprom
        oled1306.print(eStorage.powerMinValue);
        oled1306.print(" > ");
        oled1306.display();
        oled1306.print("MAX: "); // Max value
        delay(2000);
      } else if(cont==2){
        eStorage.powerMaxValue=analogInputRangeToDacRange(initThrotle);
        oled1306.print(eStorage.powerMaxValue);
        oled1306.display();
        oled1306.setCursor(START_LINE, LINE_2);
        oled1306.print("MED: "); // Min assitence value
        int dif = eStorage.powerMaxValue-eStorage.powerMinValue;
        int discount = makeDiscount(dif,80);
        int v = ((int) discount)+eStorage.powerMinValue;
        eStorage.powerMinAssistenceValue=v;
        oled1306.print(eStorage.powerMinAssistenceValue);
        oled1306.display();
        break;
      }
    }
  }
  
  blinkLed(STATUS_LED_OUT, 30, 10);
  EEPROM.put(EEPROM_INIT_ADDRESS, eStorage);
  showThrotleMinMax();
  blinkLed(STATUS_LED_OUT, 30, 100);
  
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
  ATCommandsManager(); // Interpreta comandos AT
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

    } else if (command.indexOf(F("at+plotter")) > -1) {
      if (command.indexOf(F("off")) > -1) {
        eStorage.plotterOn = false;
      }else{
        eStorage.plotterOn = true;
      }
    } else if (command.indexOf("at+trace") > -1) {
      if (command.indexOf(F("off")) > -1) {
        eStorage.traceOn = false;
      }else{
        eStorage.traceOn = true;
      }      
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
      if (command.indexOf(F("on")) > -1) {
        eStorage.mpuenabled = true;
      }else{
        eStorage.mpuenabled = false;
      }      

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

template<class T>
void serialTrace(T message){
  if(eStorage.traceOn)
    Serial.print(message);
}

template<class T>
void serialTraceLn(T message){
  if(eStorage.traceOn)
    Serial.println(message);
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
