const String VERSION = "V_2.3.0";
const boolean traceOn=false;

#include <EEPROM.h>

#define OLED_ADDR 0x3C   // Dirección de memória del display
#define MPU_ADDR  0x68  // Dirección de memória del acelerómetro - Puede ser 0x68 o 0x69 segín el dispositivo.
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

// ********************** Pins
// > Inputs
const byte GEAR_PLATE_IN = 2; // Pin de entrada de sensor de pedal por interrupcion. pull_down 0V
const byte MODE_IN = 3; // Pin de entrada de cambio de modo. pull_down 0V
const byte BRAKE_IN = 6; //Pin del sensor hall de freno. pull_up 5V
const byte THROTTLE_IN = A0; // pin acelerador

// > Outputs
const byte POWER_OUT = 11; // Pin de potencia de salida.
const byte STATUS_LED_OUT = 13; // Pin de visualización de selección de modo

// ********************** Def Consts
const int SERIAL_PORT = 9600; // Configuración de baudios del puerto serie.

const byte GEAR_PLATE_PULSES = 10; // Pulsos proporcionados por el plato en cada vuelta. 10 en el caso de la fiido

const int MIN_PLATE_TIME_SLOW = 2000; // Tiempo que se tarda en dar una vuelta de plato
const int MAX_PLATE_TIME_FAST = 600; // Tiempo que se tarda en dar una vuelta de plato

const int DEFAULT_POWER_MAX_VALUE = 4800; // Máxima potencia del PWM. 4800 -> 3.58v
const int DEFAULT_POWER_MIN_ASSISTENCE_VALUE = 2000; // Mínima potencia de asistencia ?.???v
const int DEFAULT_POWER_MIN_VALUE = 1470; // Mínima potencia del PWM. 1470 -> 1.108v

const int POWER_STEPTS[] = {2, 4, 8, 16, 32, 64, 128}; // Incremento de potencia de la salida PWM por cada paso.
const byte POWER_STEPTS_DEFAULT_POSITION = 2; // El rango (2)=8 es el valor por defecto

const int MAX_TIME_BETWEEN_GEAR_PLATE_PULSES = 300; // Tiempo mínimo en ms que se tiene que estar pedaleando para empezar a recibir potencia.

const byte DEBOUNCE_TIMETHRESHOLD_GEAR_PLATE = 50; // Tiempo de espera entre pulsos de interrupcion 49ms entre pulsación. 100 Pedaladas(GEAR_PLATE_PULSES*100=1200) por minuto (20 pulsos por segundo). 50ms de debounce para detectar el máximo de 20 pulsos por segundo.
const int DEBOUNCE_TIMETHRESHOLD_CHANGE_MODE = 1200; // Tiempo de espera entre pulsos de interrupcion 1200ms entre pulsación
const float BRAKE_DIVIDER = 1.20; // baja la potencia por frenada en pasos de 20%
const byte EEPROM_INIT_ADDRESS = 0; // Posición de memoria que almacena los datos de modo.

const byte DEFAULT_MAX_POWER_ANGLE = 30;
const byte X=0, Y=1, Z=2; // Used by getAxisAngle
const byte CRASH_ANGLE=50;

// Serial Input
const byte serialBufferSize = 32;
char receivedChars[serialBufferSize]; // an array to store the received data ut to serialBufferSize
boolean newSerialData = false;


// ********************** Control de rebote de pulsos
unsigned long debounceLastGearPlatePulseTime = 0; // Almacena el millis() en el que se tomo medida del pulso anterior del sensor de pedalada para controlar el debounce.
unsigned long debounceChangeModeTime = 0; // Almacena el millis() en el que se tomo medida del último pulso de cambio de modo para controlar el debounce.

// ********************** Variables de control de pedalada.
unsigned long gearPlatePulseEnd; // Temporizador que almacena el millis() del último pulso de plato para saber si se está pedaleando.
unsigned long gearPlatePulseInit; // Almacena los millis del primer pulso de la pedalada.
byte gearPlatecompleteCicleCounter; // Contador de pulsos por ciclo de pedalada para controlar las revoluciones por minuto del plato.
int gearPlateCompleteCicleTime = 0; //Tiempo empreado en dar una vuelta de plato. Necesario para calcular maxPowerValue.

// ********************** Variables de control de potencia
int maxPowerValue; // Máxima potencia calculada según pedalada y la inclinación.
int currentPowerValue; // Potencia actual

// ********************** Variables de control de cambio de modo. + EEPROM
int powerModeChangeTrigger; // Trigger de solicitud de cambio de modo . La variable es cambiada de estado por el método changeModeInt lanzado por la interrupción para detectar el tipo. Valores 0/1/2.

// Estructura para almacenar los valores de los modos de control.
struct EStorage {
  boolean mpuenabled = false;
  byte levelXAngle = 0;
  byte levelYAngle = 0;

  byte powerMode = POWER_STEPTS_DEFAULT_POSITION;
  byte powerBrakeMode = POWER_STEPTS_DEFAULT_POSITION;
  int powerMaxValue = DEFAULT_POWER_MAX_VALUE;
  int powerMinAssistenceValue = DEFAULT_POWER_MIN_ASSISTENCE_VALUE;
  int powerMinValue = DEFAULT_POWER_MIN_VALUE;
  
  byte maxPowerAngle = DEFAULT_MAX_POWER_ANGLE;
};

// Instancia de datos de configuración eeprom
EStorage eStorage; // Instancia Variable de control de modos.

// ********************** Variables de control de Acelerómetro MPU6050
byte currentAngle=0; //Variable que almacena el ángulo de potencia.
long devounceMpuTimeThreshold; // Tiempo entre las medidas de ángulo
byte powerAngle=X; // Eje que controla el ángulo de potencia
boolean powerAngleInverter=true; // Inversor del eje de potencia
byte crashAngle=(powerAngle==X)?Y:X; // Eje que controla el ángulo de control de caida

int16_t ax, ay, az; // Variables de control de Acelerómetro xyz
//int16_t gx, gy, gz; // Variables de control de Giroscopio xyz


String messageBuilder;

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
    oled1306.setCursor(0, 0);
    oled1306.print(F(" + MPU_"));
    oled1306.print(mpu6050.testConnection() ? F("OK") : F("KO"));
    devounceMpuTimeThreshold=millis();
  } else {
    // Inicializamos valores de inclinación sin acelerómetro.
    ax = 0; ay = 0; az = 1;
//  gx = 0; gy = 0; gz = 1;
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

  pinMode(THROTTLE_IN, INPUT); // Entrada Interrupción - control nodo de operación.


  // Inicialización de variables
  debounceLastGearPlatePulseTime = 0; // Inicializamos el timer de control de rebote
  gearPlatePulseEnd = 0; // Inicializamos el timer de último pulso de pedaleo.
  gearPlatecompleteCicleCounter = GEAR_PLATE_PULSES; // Inicializamos el contador de pulsos para controlar el ciclo de pedalada. cada n pulsos se calculará el tiempo que se ha tardado en dar una vuelta al disco.

  //initFlashLeds(); // Ejecuta los leds de inicio de script. y muestra los modos.

  // Definición de interrupciones
  attachInterrupt(digitalPinToInterrupt(GEAR_PLATE_IN), gearPlatePulseInt, RISING); // Definición de la interrupción para la entrada de pin de pedal.
  attachInterrupt(digitalPinToInterrupt(MODE_IN), changeModeInt, RISING); // Definición de la interrupción para la entrada de pin de cambio de modo.
  oled1306.print(F(" |"));
  oled1306.display();

  showEepromDataScreen();
  powerModeChangeTrigger = 0; // Init changeModeTrigger Flag.
}

// Main Program
void loop() {
  //serialTraceLn(F("1 loop"));
  changeModeListener(); // Controla powerModeChangeTrigger para cambiar los modos
  //float throttleValue=analogRead(THROTTLE_IN); // Lee el valor analógico del acelerador
  serialAtCommandListener(); // Lee comandos AT por puerto serie
  ATCommandsManager(); // Interpreta comandos AT

  mpu6050.getAcceleration(&ax, &ay, &az); // Lee los datos del acelerómetro.  
  //mpu6050.getRotation(&gx, &gy, &gz);  // Lee los datos del giroscopio
  //mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Lee los valores de acelerómetro y giroscopio
  if (eStorage.mpuenabled && (millis() - devounceMpuTimeThreshold > 2000)) { // leemos el valor de inclinación (Y) cada 2 segundos para ayudar a calcular la potencia.
    currentAngle=getAxisAngle(powerAngle);
    if(powerAngleInverter)
      currentAngle=-currentAngle;
    devounceMpuTimeThreshold = millis();
  }
  
  boolean iscrashdropflag = isCrashDrop();
  if ((digitalRead(BRAKE_IN) == LOW || iscrashdropflag) && currentPowerValue > eStorage.powerMinValue) { // Resetea el acelerador si se toca el freno o se está en estado caida y la potencia de acelerador no es la mínima.
    // Resetea valores cortando la potencia en modo seguridad
    gearPlatePulseEnd = 0; // Reiniciamos el tiempo de control de rebote.
    gearPlatecompleteCicleCounter = GEAR_PLATE_PULSES; // Reiniciamos la posición del contador de vueltas de plato.
    gearPlatePulseInit = 0; // Reiniciamos el timer de fin de pedalada.
    maxPowerValue = eStorage.powerMinValue;
    if(currentPowerValue > eStorage.powerMinValue && !iscrashdropflag){
      currentPowerValue = currentPowerValue / BRAKE_DIVIDER; // Decrementa la potencia un 20% - si se deja pulsado el freno corta toda la potencia en unos 400ms
    }else{
      currentPowerValue = eStorage.powerMinValue;
    }
    
    showAlertScreen(F(" BRAKE!!!"));
    blinkLed(STATUS_LED_OUT, 10, 10); // Muestra el testigo de frenado. 100ms
    showMaxPowerScreen();
  } else {
    if (millis() - gearPlatePulseEnd < MAX_TIME_BETWEEN_GEAR_PLATE_PULSES) { // Si en los ultimos n segundos se ha detectado un pulso de pedaleo
      //serialTraceLn(F("PEDALEANDO!!!!!"));
      showPedalIcon(WHITE);
      oled1306.setCursor(110, 20);
      oled1306.print(gearPlatecompleteCicleCounter);
      oled1306.display();    
      calculateMaxPower();

    } else { // El sensor no detecta pulso de pedaleo.
      //serialTraceLn(F("NO PEDALEANDO!!!!!"));
      showPedalIcon(BLACK);
      gearPlatePulseEnd=0;
      gearPlatePulseInit=0;
      gearPlateCompleteCicleTime=0;
      maxPowerValue = eStorage.powerMinValue;
    }
  }
  updatePower(); // actualizamos la potencia de salida.
}

// Interruption Methods ***********************************************************************************
void gearPlatePulseInt() { // Método que incrementa el contador de pedal en caso de pulso por el pin de interrupcion.
  //serialTraceLn(F("2 gearPlatePulseInt"));

  if (millis() > debounceLastGearPlatePulseTime + DEBOUNCE_TIMETHRESHOLD_GEAR_PLATE) { // Debounce Si ha pasado el tiempo de control de rebote.
    gearPlatePulseEnd = millis(); // Almacena el flag de control de último pulso para evitar rebotes.

    if (gearPlatecompleteCicleCounter > 0) { // Iteramos los pulsos del plato para detectar el inicio y el fin vuelta.
      //serialTrace("> ");
      //serialTrace(gearPlatecompleteCicleCounter);
      //serialTrace(" | ");
      gearPlatecompleteCicleCounter--;
      //serialTrace(gearPlatecompleteCicleCounter);
      //serialTrace(" => ");
    } else {
      gearPlatePulseInit = gearPlatePulseEnd;
      //serialTraceLn("");
      gearPlatecompleteCicleCounter = GEAR_PLATE_PULSES;
    }

    if (gearPlatecompleteCicleCounter == 0){
      //serialTraceLn(gearPlateCompleteCicleTime);
      gearPlateCompleteCicleTime = gearPlatePulseEnd - gearPlatePulseInit;
    }

    debounceLastGearPlatePulseTime = millis(); //Debounce Guardamos el tiempo de la operación para bloquear el rebote.
  }
}

void changeModeInt() {
  //serialTraceLn(F("3 changeModeInt"));
  if (millis() > debounceChangeModeTime + DEBOUNCE_TIMETHRESHOLD_CHANGE_MODE) { //Debounce
    if (digitalRead(BRAKE_IN) == LOW) {
      powerModeChangeTrigger = 2; // cambia el modo de desaceleración
    } else {
      powerModeChangeTrigger = 1; // cambia el modo de aceleración
    }
  }
  debounceChangeModeTime = millis();//Debounce
}


// MPU METHODS ***********************************************************************************

float getAxisAngle(int axis) {
  //serialTraceLn(F("4 getAxisAngle"));
  //Calcular los angulos de inclinacion desde el acelerómetro;
  float accel_ang;
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
    accel_ang = 0;
    break;
  }
  return accel_ang;
}

boolean isCrashDrop() { // Si El acelerómetro detecta que la bicicleta ha caido al suelo.
  //serialTrace(F("isCrashDrop 5_"));
  float accel_ang_x = getAxisAngle(crashAngle);
  return (accel_ang_x > CRASH_ANGLE || accel_ang_x < -CRASH_ANGLE);
}

// POWER METHODS ***********************************************************************************
void calculateMaxPower() {
  //serialTraceLn(F("6 - calculateMaxPower"));

  int currentPowerByAngle = eStorage.powerMinValue; // Variable que almacena la potencia actual por ángulo
  int currentPowerByPedal= eStorage.powerMinValue; // Variable que almacena la potencia actual por pedalada

  // Calcula la potencia dependiendo del ángulo y realiza la media entre el valor actual y el anterior;
  float min_assistence_range = ((1.0 * eStorage.powerMaxValue) / eStorage.powerMinAssistenceValue) / eStorage.maxPowerAngle; // Calculamos la fracción del ángulo
  byte curAngInt = (byte) currentAngle;
  //byte tmpAnglePower = min_assistence_range*curAngInt*eStorage.powerMinAssistenceValue; // Calculamos el valor máximo de potencia según el ángulo

  byte lvlAngle = curAngInt; // Calcula el ángulo real del acelerómetro
  if (eStorage.levelYAngle > 0) {
    lvlAngle = curAngInt - eStorage.levelYAngle; // Si hay desfase de ángulo respecto al nivel, nivelamos el valor.
  } else if (eStorage.levelYAngle < 0) {
    lvlAngle = eStorage.levelYAngle + curAngInt;
  }
  lvlAngle = lvlAngle < 0 ? 0 : lvlAngle; // Si el ángulo es negativo, lo ponemos a 0 para utilizar la mínima asistencia.
  //serialTraceLn(lvlAngle);

  if(gearPlateCompleteCicleTime>0){
    int tmpAnglePower = min_assistence_range * lvlAngle * eStorage.powerMinAssistenceValue; // Calculamos el valor máximo de potencia según el ángulo adaptado a nivel
    currentPowerByAngle = (currentPowerValue + tmpAnglePower) / 2; //Calculamos la media entre el valor actual y el anterior
    //currentPowerByAngle = currentPowerByAngle>eStorage.powerMaxValue?eStorage.powerMaxValue:currentPowerByAngle;
    //currentPowerByAngle = currentPowerByAngle<eStorage.powerMinAssistenceValue?eStorage.powerMinAssistenceValue:currentPowerByAngle;
  
    // Calcula la potencia en base a la pedalada
    int tmpPedalPower = ((eStorage.powerMaxValue - eStorage.powerMinAssistenceValue) / (MIN_PLATE_TIME_SLOW - MAX_PLATE_TIME_FAST)) * (eStorage.powerMinAssistenceValue - gearPlateCompleteCicleTime) + eStorage.powerMinAssistenceValue;
    currentPowerByPedal = (currentPowerValue + tmpPedalPower) / 2;
    //currentPowerByPedal = currentPowerByPedal>eStorage.powerMaxValue?eStorage.powerMaxValue:currentPowerByPedal;
    //currentPowerByPedal = currentPowerByPedal<eStorage.powerMinAssistenceValue?eStorage.powerMinAssistenceValue:currentPowerByPedal;
  }else{
    currentPowerByPedal = eStorage.powerMinAssistenceValue;
  }
  // Decidimos cual es la potencia más alta y ajusta mínimos y máximos.
  maxPowerValue = currentPowerByAngle > currentPowerByPedal ? currentPowerByAngle : currentPowerByPedal;
  maxPowerValue = maxPowerValue > eStorage.powerMaxValue ? eStorage.powerMaxValue : maxPowerValue;
  maxPowerValue = maxPowerValue < eStorage.powerMinAssistenceValue ? eStorage.powerMinAssistenceValue : maxPowerValue;

}

void updatePower() {
  //serialTraceLn(F("7 - updatePower"));
  if (currentPowerValue < maxPowerValue) { // incrementa progresivamente la potencia hasta la máxima.
    currentPowerValue = currentPowerValue + POWER_STEPTS[eStorage.powerMode];
    if (currentPowerValue > maxPowerValue) { // Si se supera el valor máximo de escala de potencia, este se regula.
      currentPowerValue = maxPowerValue;
    }
  } else if (currentPowerValue > maxPowerValue) { // decrementa progresivamente la potencia.
    currentPowerValue = currentPowerValue - POWER_STEPTS[eStorage.powerBrakeMode]; // Desacelera en el step más bajo.
    if (currentPowerValue < maxPowerValue) { // Si se rebasa el nivel mínimo de escala de potencia, este se regula.
      currentPowerValue = maxPowerValue;
    }
  }

  if(currentPowerValue < eStorage.powerMinValue){ // Ajusta los mínimos de potencias
    currentPowerValue = eStorage.powerMinValue;
  }
  
  showMaxPowerScreen();

  //dac4725.setVoltage(currentPowerValue, false); // Actualiza la salida con la potencia de acelerador por medio del DAC. // retocar rango de valores ya que el dac va de 0 a 4096 (0 - 5V)
  analogWrite(POWER_OUT, currentPowerValue / 20); // Actualiza la salida con la potencia de acelerador por medio del PWM.
}

// Listeners Methods ***********************************************************************************
void changeModeListener() {

  if (powerModeChangeTrigger > 0){
    //serialTraceLn(F("8 changeModeListener"));
    if (powerModeChangeTrigger == 1) {
      if (eStorage.powerMode < (sizeof(POWER_STEPTS) / sizeof(int) - 1)) {
        eStorage.powerMode++;
      } else {
        eStorage.powerMode = 0;
      }  
      blinkLed(STATUS_LED_OUT, eStorage.powerMode + 1, (600 / (eStorage.powerMode + 1)));
      showAlertScreen(messageBuilder);
    } else if (powerModeChangeTrigger == 2) {
      if (eStorage.powerBrakeMode < (sizeof(POWER_STEPTS) / sizeof(int) - 1)) {
        eStorage.powerBrakeMode++;
      } else {
        eStorage.powerBrakeMode = 0;
      }
      blinkLed(STATUS_LED_OUT, eStorage.powerBrakeMode + 1, (600 / (eStorage.powerBrakeMode + 1)));
      currentPowerValue = eStorage.powerMinValue;
    }
    
    currentPowerValue = eStorage.powerMinValue;
    messageBuilder = F("CHANGE PM|PBM ");
    messageBuilder = messageBuilder + eStorage.powerMode;
    messageBuilder = messageBuilder + F(" | ");
    messageBuilder = messageBuilder + eStorage.powerBrakeMode;
    showAlertScreen(messageBuilder);
    powerModeChangeTrigger=0;
    
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
  eStorage.levelXAngle = 0;
  eStorage.levelYAngle = 0;
  eStorage.mpuenabled = false;
  eStorage.powerMaxValue = DEFAULT_POWER_MAX_VALUE;
  eStorage.powerMinAssistenceValue = DEFAULT_POWER_MIN_ASSISTENCE_VALUE;
  eStorage.powerMinValue = DEFAULT_POWER_MIN_VALUE;

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
  for (byte y = 0; y <= 31; y++) {
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
  oled1306.setCursor(pos*6, 10);
  oled1306.print(message);
  oled1306.display();
}

void showMaxPowerScreen() {
  //serialTraceLn(F("15 showMaxPowerScreen"));
  if (currentPowerValue > eStorage.powerMinValue && currentPowerValue < eStorage.powerMaxValue) {  // Solo se muestra si se está pedaleando o si la potencia no es la mínima
    float coutput = ((float)(maxPowerValue * 3.58) / DEFAULT_POWER_MAX_VALUE);
    float coutput1 = ((float)(currentPowerValue * 3.58) / DEFAULT_POWER_MAX_VALUE);
  
    oled1306.clearDisplay();
    oled1306.setCursor(0, 0);
    oled1306.print(F("> Y:"));
    oled1306.print(currentAngle);
    oled1306.print(F(" PULSE :"));
    if(gearPlateCompleteCicleTime>0)
      oled1306.print(gearPlateCompleteCicleTime);
    else
      oled1306.print(F("???"));
  
    oled1306.setCursor(0, 10);
    oled1306.print(F("> maxPwr: "));
    oled1306.print(coutput);
    //oled1306.print(maxPowerValue);
    oled1306.setCursor(0, 20);
    oled1306.print(F("> curPwr: "));
    oled1306.print(coutput1);
    //oled1306.print(currentPowerValue);
  
    oled1306.display();
  }
}

void showEepromDataScreen() {
  //serialTraceLn(F("16 showEepromDataScreen"));
  oled1306.setCursor(0, 10);
  oled1306.print(F("> pwr M|BM: "));
  oled1306.print(eStorage.powerMode);
  oled1306.print(F("|"));
  oled1306.print(eStorage.powerBrakeMode);
  oled1306.setCursor(0, 20);
  if (eStorage.mpuenabled) {
    oled1306.print(F("> mPW:"));
    oled1306.print(eStorage.maxPowerAngle);
    oled1306.print(F(" X:"));
    oled1306.print(eStorage.levelXAngle);
    oled1306.print(F(" Y:"));
    oled1306.print(eStorage.levelYAngle);
  } else {
    oled1306.print(F("> mpu DISABLED"));
  }
  oled1306.display();
  delay(2000);
}

void showHomeScreen() {
  //serialTraceLn(F("17 showHomeScreen"));
  oled1306.clearDisplay();
  oled1306.setTextSize(1);
  oled1306.setTextColor(WHITE);

  oled1306.setCursor(15, 0);
  oled1306.print(F("FIIDO ASSISTANCE"));
  oled1306.setCursor(40, 10);
  oled1306.print(F("PROJECT"));
  oled1306.setCursor(80, 20);
  oled1306.print(VERSION);
  oled1306.display();
  oled1306.setCursor(90, 10);
  oled1306.print(F("*"));
  oled1306.display();
  delay(90);
  oled1306.print(F("*"));
  oled1306.display();
  delay(90);
  oled1306.setCursor(20, 10);
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
  for (byte i = 0; i <= repeat; i++) {
    delay(100);
    //serialTrace(F("."));
  }
}

void blinkLed(byte ledPin, byte repeats, int time) { // Ejecuta un parpadeo en el led durante x repeticiones y con una frecuencia de x_ms.
  for (byte i = 0; i < repeats; i++) {
    digitalWrite(ledPin, HIGH);
    delay(time);
    digitalWrite(ledPin, LOW);
    delay(time);
  }
}

void serialTrace(String message){
  if(traceOn)
    Serial.print(message);
}
void serialTraceLn(String message){
  if(traceOn)
    Serial.println(message);
}

/*
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
*/

// SERIAL AT METHODS ***********************************************************************************

void serialAtCommandListener() { // Espera un comando AT con un buffer de hasta serialBufferSize (32)
    
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  while (Serial.available() > 0 && newSerialData == false) {
    //serialTraceLn(F("19 serialAtCommandListener"));
    rc = Serial.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= serialBufferSize) {
        ndx = serialBufferSize - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      String receivedCharsStr = receivedChars;
      if (receivedCharsStr.startsWith("at") || receivedCharsStr.startsWith("AT")) {
        newSerialData = true;
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

  if (newSerialData == true) {
    //serialTraceLn(F("20 ATCommandsManager"));
    blinkLed(STATUS_LED_OUT, 1, 500);
    String command = getAtData(receivedChars, '=' , 0);
    command.toLowerCase();
    int value = getAtData(receivedChars, '=' , 1).toInt();

    oled1306.clearDisplay();
    oled1306.setCursor(0, 0);
    oled1306.print(F("> "));
    String tmpCommand = command;
    tmpCommand.toUpperCase();
    oled1306.print(tmpCommand);
    if (value > 0) {
      oled1306.print(F("?"));
      oled1306.print(value);
    }
    oled1306.setCursor(0, 10);

    if (command.indexOf(F("at+save")) > -1) {
      updateEepromData();

    } else if (command.indexOf(F("at+init")) > -1) {
      initDefaultEepromData();

    } else if (command.indexOf(F("at+eelist")) > -1) {
      //printEeprom();
      showEepromDataScreen();
      
    } else if (command.indexOf(F("at+pwrup")) > -1) { // modo de incremento de potencia progresiva.
      eStorage.powerMode = (value < (sizeof(POWER_STEPTS) / 2)) ? value : POWER_STEPTS_DEFAULT_POSITION;
      oled1306.print(F("> powerBrakeMode: "));
      oled1306.print(eStorage.powerMode);

    } else if (command.indexOf(F("at+pwrdw")) > -1) { // modo de decremento de potencia progresiva.
      eStorage.powerBrakeMode = (value < (sizeof(POWER_STEPTS) / 2)) ? value : POWER_STEPTS_DEFAULT_POSITION;
      oled1306.print(F("> powerBrakeMode: "));
      oled1306.print(eStorage.powerBrakeMode);

    } else if (command.indexOf(F("at+mpuon")) > -1) {
      eStorage.mpuenabled = true;

    } else if (command.indexOf(F("at+mpuoff")) > -1) {
      eStorage.mpuenabled = false;
      ax = 0; ay = 0; az = 1;

    } else if (command.indexOf(F("at+shutdown")) > -1) {
      asm volatile ("  jmp 0");

    } else if (eStorage.mpuenabled) {

      if (command.indexOf(F("at+anglemaxpwr")) > -1) { // ángulo para la máxima potencia;
        eStorage.maxPowerAngle = value < DEFAULT_MAX_POWER_ANGLE ? value : DEFAULT_MAX_POWER_ANGLE;
        oled1306.print(F("> maxPowerAngle: "));
        oled1306.print(eStorage.maxPowerAngle);

      } else if (command.indexOf("at+level") > -1) { // Calibrar posición de placa.
        float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
        float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
        eStorage.levelXAngle = (int) accel_ang_x;
        eStorage.levelYAngle = (int) accel_ang_y;
        oled1306.print(F("> [X|Y]: "));
        oled1306.print(eStorage.levelXAngle);
        oled1306.print(F(" | "));
        oled1306.print(eStorage.levelYAngle);
        oled1306.display();
      }
    }

    newSerialData = false;
    blinkLed(STATUS_LED_OUT, 15, 30);
  }
}

String getAtData(String data, char separator, int index) { // Split string and get index data.
  //serialTraceLn(F("21 getAtData"));
  int found = 0;
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
