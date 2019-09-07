const String VERSION = "V_2.2.14";

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
const int GEAR_PLATE_IN = 2; // Pin de entrada de sensor de pedal por interrupcion. pull_down 0V
const int MODE_IN = 3; // Pin de entrada de cambio de modo. pull_down 0V
const int BRAKE_IN = 6; //Pin del sensor hall de freno. pull_up 5V
const int THROTTLE_IN = A0; // pin acelerador

// > Outputs
const int POWER_OUT = 11; // Pin de potencia de salida.
const int STATUS_LED_OUT = 13; // Pin de visualización de selección de modo

// ********************** Def Consts
const int SERIAL_PORT = 9600; // Configuración de baudios del puerto serie.

const int MAX_GEAR_PLATE_PULSES = 12; // Pulsos proporcionados por el plato en cada vuelta. 10 en el caso de la fiido
const int MIN_GEAR_PLATE_PULSES = 3; // Pulsos proporcionados por el plato en cada vuelta. 10 en el caso de la fiido
const int MAX_TIME_BETWEEN_GEAR_PLATE_PULSES = 500; // Tiempo mínimo en ms que se tiene que estar pedaleando para empezar a recibir potencia.

const int MIN_PLATE_TIME_SLOW = 2000; // Tiempo que se tarda en dar una vuelta de plato
const int MAX_PLATE_TIME_FAST = 600; // Tiempo que se tarda en dar una vuelta de plato

const int DEFAULT_POWER_MAX_VALUE = 4800; // Máxima potencia del PWM. 4800 -> 3.58v
const int DEFAULT_POWER_MIN_ASSISTENCE_VALUE = 2000; // Mínima potencia de asistencia ?.???v
const int DEFAULT_POWER_MIN_VALUE = 1470; // Mínima potencia del PWM. 1470 -> 1.108v

const int POWER_STEPTS[] = {2, 4, 8, 16, 32}; // Incremento de potencia de la salida PWM por cada paso.
const int POWER_STEPTS_DEFAULT_POSITION = 2; // El rango (2)=5 es el valor por defecto


const int DEBOUNCE_TIMETHRESHOLD_CHANGE_MODE = 1200; // Tiempo de espera entre pulsos de interrupcion 1200ms entre pulsación

const int EEPROM_INIT_ADDRESS = 0; // Posición de memoria que almacena los datos de modo.

const int DEFAULT_MAX_POWER_ANGLE = 30;

// Serial Input
const byte serialBufferSize = 32;
char receivedChars[serialBufferSize]; // an array to store the received data
boolean newSerialData = false;


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
struct EStorage {
  int powerMode = POWER_STEPTS_DEFAULT_POSITION;
  int powerBrakeMode = POWER_STEPTS_DEFAULT_POSITION;
  int maxPowerAngle = DEFAULT_MAX_POWER_ANGLE;
  int levelXAngle = 0;
  int levelYAngle = 0;
  boolean mpuenabled = false;
  int powerMaxValue = DEFAULT_POWER_MAX_VALUE;
  int powerMinAssistenceValue = DEFAULT_POWER_MIN_ASSISTENCE_VALUE;
  int powerMinValue = DEFAULT_POWER_MIN_VALUE;
  int gearPlatePulses=MAX_GEAR_PLATE_PULSES;
};

//const int DEBOUNCE_TIMETHRESHOLD_GEAR_PLATE = 50; // Tiempo de espera entre pulsos de interrupcion 49ms entre pulsación. 100 Pedaladas(GEAR_PLATE_PULSES*100=1200) por minuto (20 pulsos por segundo). 50ms de debounce para detectar el máximo de 20 pulsos por segundo.
int debounceTimeThresholdGearPlate; // calculamos debounce dependiendo de los pulsos del plato. Reemplaza a la constante.

EStorage eStorage; // Instancia Variable de control de modos.

// ********************** Variables de control de Acelerómetro MPU6050
int16_t ax, ay, az; // Variables de control de Acelerómetro xyz
int16_t gx, gy, gz; // Variables de control de Giroscopio xyz

String outputMessage = "";

int currentPowerByAngle = eStorage.powerMinValue;
int currentPowerByPedal = eStorage.powerMinValue;

// Control de inicialización de datos de eeprom

int initDefaultEepromDataFlag = -1; //Contador de ciclos pendientes de paso para lanzar el proceso de inicialización.
long initDefaultEepromDataTimer; // Timer para controlar los pulsos necesarios para inicializar la eeprom.
int initDefaultEepromDataStatus; // almacena el estado de freno en cada ciclo

//long up=0;
//long dw=0;

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
    //Serial.println(mpu6050.testConnection() ? F(" > MPU iniciado correctamente.") : F(" * ERROR: Error al iniciar MPU."));
    oled1306.setCursor(0, 0);
    oled1306.print(" + MPU");
  } else {
    // Inicializamos valores de inclinación sin acelerómetro.
    ax = 0; ay = 0; az = 1;
    oled1306.print(" - MPU");
  }

  dac4725.begin(DAC_ADDR); // Inicializa el DAC

  //Inicializamos los pines
  pinMode(LED_BUILTIN, OUTPUT); // Salida led modo/estado. //STATUS_LED_OUT
  pinMode(POWER_OUT, OUTPUT); // Salida PWM acelerador variable.

  pinMode(BRAKE_IN, INPUT_PULLUP); // Entrada de control de freno.
  pinMode(GEAR_PLATE_IN, INPUT_PULLUP); // Entrada Interrupción  - control de pedalaleo.
  pinMode(MODE_IN, INPUT_PULLUP); // Entrada Interrupción - control nodo de operación.

  pinMode(THROTTLE_IN, INPUT); // Entrada Interrupción - control nodo de operación.


  // Inicialización de variables
  debounceTimeThresholdGearPlate = int(100/(eStorage.gearPlatePulses*2));
  debounceLastGearPlatePulseTime = 0; // Inicializamos el timer de control de rebote
  gearPlateLastPulseTime = 0; // Inicializamos el timer de último pulso de pedaleo.
  gearPlatecompleteCicleCounter = eStorage.gearPlatePulses; // Inicializamos el contador de pulsos para controlar el ciclo de pedalada. cada n pulsos se calculará el tiempo que se ha tardado en dar una vuelta al disco.

  //initFlashLeds(); // Ejecuta los leds de inicio de script. y muestra los modos.

  // Definición de interrupciones
  attachInterrupt(digitalPinToInterrupt(GEAR_PLATE_IN), gearPlatePulseInt, RISING); // Definición de la interrupción para la entrada de pin de pedal.
  attachInterrupt(digitalPinToInterrupt(MODE_IN), changeModeInt, RISING); // Definición de la interrupción para la entrada de pin de cambio de modo.
  oled1306.print(" > PINS");
  oled1306.display();

  // Activamos el modo de reseteo de datos de la eeprom desde el pedal de freno. se precisa realizar cambios de estado en el freno durante 10 segundos.
  if (digitalRead(BRAKE_IN) == LOW) {
    oled1306.setCursor(10, 0);
    oled1306.print(" > Waiting init cfg");
    oled1306.display();
    //Serial.print(" > Waiting code for init config...\t");
    initDefaultEepromDataFlag = 6;
    initDefaultEepromDataTimer = millis();
  }

  showEepromDataScreen();
  powerModeChangeTrigger = 0; // Init changeModeTrigger Flag.
  powerCurrentValue=DEFAULT_POWER_MIN_VALUE;
}

// main
void loop() {
  delay(10);
  hardwareConfigResetListener(); // Si detecta el freno pulsado al inicio, entra en modo inicialización por defecto.
  changeModeListener(); // Controla powerModeChangeTrigger para cambiar los modos
  //float throttleValue=analogRead(THROTTLE_IN); // Lee el valor analógico del acelerador
  serialRcvATCommandWithEndMarkerListener(); // Lee comandos AT por puerto serie
  manageATCommands(); // Interpreta comandos AT

  
  if (eStorage.mpuenabled) {
    mpu6050.getAcceleration(&ax, &ay, &az); // Lee los datos del acelerómetro.
    //mpu6050.getRotation(&gx, &gy, &gz);  // Lee los datos del giroscopio
    //mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Lee los valores de acelerómetro y giroscopio
  }

  if ((digitalRead(BRAKE_IN) == LOW || isCrashDrop()) && powerCurrentValue > eStorage.powerMinValue) { // Resetea el acelerador si se toca el freno o se está en estado caida y la potencia de acelerador no es la mínima.
    reset(true); // Resetea valores cortando la potencia en modo seguridad
    updatePower();  // Actualizamos potencia de salida.
    showBrakeSceen();
    blinkLed(STATUS_LED_OUT, 15, 20); // Muestra el testigo de frenado. 300ms
    showMaxPowerScreen(getAngle(), pulseEnd - pulseInit);
  } else {
    if (millis() - gearPlateLastPulseTime < MAX_TIME_BETWEEN_GEAR_PLATE_PULSES) { // Si en los ultimos n segundos se ha detectado un pulso de pedaleo
      // PEDALEANDO
      //up++;
      //dw=0;
      showPedalIcon(WHITE);
      // V En cada ciclo de disco ejecutamos las operaciones necesarias.
      // V Como calcular la potencia dependiendo de la potencia de pedalada. A esto le añadiremos el control de inclinación cuando esté implementado.
      if (completeGearPlateCicle) {
        calculateMaxPower(pulseEnd - pulseInit, getAngle());
      }
    } else { // El sensor no detecta pulso de pedaleo. // TODO decrementa progresivamente la pedalada.
      // PARADO
      //dw++;
      //up=0;
      showPedalIcon(BLACK);
      reset(false); // Resetea valores cortando la potencia en modo progresivo
    }
  }
  updatePower(); // actualizamos la potencia de salida.
  //showStatus(); // Muestra el led de estado
}

void hardwareConfigResetListener(){
  if (initDefaultEepromDataFlag >= 0 && (millis() - initDefaultEepromDataTimer) < 10000) {
    delay(100);
    if (initDefaultEepromDataFlag == 0) {
      initDefaultEepromDataFlag--;
      initDefaultEepromData();
    } else {
      int currentvalue = digitalRead(BRAKE_IN);
      if (currentvalue != initDefaultEepromDataStatus) {
        //Serial.print(initDefaultEepromDataFlag);
        //Serial.print("_");
        initDefaultEepromDataStatus = currentvalue;
        initDefaultEepromDataFlag--;
      }
    }
  }  
}

float getAngle() {
  //Calcular los angulos de inclinacion
  float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  return accel_ang_y;
}

boolean isCrashDrop() { // Si El acelerómetro detecta que la bicicleta ha caido al suelo.
  float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
  return (accel_ang_x > 50 || accel_ang_x < -50);
}

void calculateMaxPower(int timeCicle, float angle) {
  // Calcula la potencia dependiendo del ángulo y realiza la media entre el valor actual y el anterior;
  float min_assistence_range = ((1.0 * eStorage.powerMaxValue) / eStorage.powerMinAssistenceValue) / eStorage.maxPowerAngle; // Calculamos la fracción del ángulo
  int currentAngle = (int) angle;
  //int tmpAnglePower = min_assistence_range*currentAngle*eStorage.powerMinAssistenceValue; // Calculamos el valor máximo de potencia según el ángulo

  int lvlAngle = currentAngle; // Calcula el ángulo real del acelerómetro
  if (eStorage.levelYAngle > 0) {
    lvlAngle = currentAngle - eStorage.levelYAngle; // Si hay desfase de ángulo respecto al nivel, nivelamos el valor.
  } else if (eStorage.levelYAngle < 0) {
    lvlAngle = eStorage.levelYAngle + currentAngle;
  }
  lvlAngle = lvlAngle < 0 ? 0 : lvlAngle; // Si el ángulo es negativo, lo ponemos a 0 para utilizar la mínima asistencia.
  //Serial.println(lvlAngle);

  int tmpAnglePower = min_assistence_range * lvlAngle * eStorage.powerMinAssistenceValue; // Calculamos el valor máximo de potencia según el ángulo adaptado a nivel
  currentPowerByAngle = (currentPowerByAngle + tmpAnglePower) / 2; //Calculamos la media entre el valor actual y el anterior
  //currentPowerByAngle = currentPowerByAngle>eStorage.powerMaxValue?eStorage.powerMaxValue:currentPowerByAngle;
  //currentPowerByAngle = currentPowerByAngle<eStorage.powerMinAssistenceValue?eStorage.powerMinAssistenceValue:currentPowerByAngle;

  // Calcula la potencia en base a la pedalada
  int tmpPedalPower = ((eStorage.powerMaxValue - eStorage.powerMinAssistenceValue) / (MIN_PLATE_TIME_SLOW - MAX_PLATE_TIME_FAST)) * (eStorage.powerMinAssistenceValue - timeCicle) + eStorage.powerMinAssistenceValue;
  currentPowerByPedal = (currentPowerByPedal + tmpPedalPower) / 2;
  //currentPowerByPedal = currentPowerByPedal>eStorage.powerMaxValue?eStorage.powerMaxValue:currentPowerByPedal;
  //currentPowerByPedal = currentPowerByPedal<eStorage.powerMinAssistenceValue?eStorage.powerMinAssistenceValue:currentPowerByPedal;

  // Decidimos cual es la potencia más alta y ajusta mínimos y máximos.
  maxPowerValue = currentPowerByAngle > currentPowerByPedal ? currentPowerByAngle : currentPowerByPedal;
  maxPowerValue = maxPowerValue > eStorage.powerMaxValue ? eStorage.powerMaxValue : maxPowerValue;
  maxPowerValue = maxPowerValue < eStorage.powerMinAssistenceValue ? eStorage.powerMinAssistenceValue : maxPowerValue;

  showMaxPowerScreen(angle, timeCicle);

  completeGearPlateCicle = false; // Reseteamos ciclo de plato.
}

void updatePower() {
  if (powerCurrentValue < maxPowerValue) { // incrementa progresivamente la potencia hasta la máxima.
    powerCurrentValue = powerCurrentValue + POWER_STEPTS[eStorage.powerMode];
    if (powerCurrentValue > maxPowerValue) { // Si se supera el valor máximo de escala de potencia, este se regula.
      powerCurrentValue = maxPowerValue;
    }
  } else if (powerCurrentValue > maxPowerValue) { // decrementa progresivamente la potencia.
    powerCurrentValue = powerCurrentValue - POWER_STEPTS[eStorage.powerBrakeMode]; // Desacelera en el step más bajo.
    if (powerCurrentValue < maxPowerValue) { // Si se rebasa el nivel mínimo de escala de potencia, este se regula.
      powerCurrentValue = maxPowerValue;
    }
  }

  if (millis() - gearPlateLastPulseTime > MAX_TIME_BETWEEN_GEAR_PLATE_PULSES && powerCurrentValue > eStorage.powerMinValue) {
    showMaxPowerScreen(getAngle(), pulseEnd - pulseInit);
  }

  //dac4725.setVoltage(powerCurrentValue, false); // Actualiza la salida con la potencia de acelerador por medio del DAC. // retocar rango de valores ya que el dac va de 0 a 4096 (0 - 5V)
  analogWrite(POWER_OUT, powerCurrentValue / 20); // Actualiza la salida con la potencia de acelerador por medio del PWM.
/*
  Serial.print("CPW " );
  Serial.print(powerCurrentValue);
  Serial.print(" | up" );
  Serial.print(up);
  Serial.print(" - dw " );
  Serial.println(dw);
*/  
  
}


// interruption methods
void gearPlatePulseInt() { // Método que incrementa el contador de pedal en caso de pulso por el pin de interrupcion.

  if (millis() > debounceLastGearPlatePulseTime + debounceTimeThresholdGearPlate) { // Debounce Si ha pasado el tiempo de control de rebote.
    gearPlateLastPulseTime = millis(); // Almacena el flag de control de último pulso para evitar rebotes.

    if (gearPlatecompleteCicleCounter > 0) { // Iteramos los pulsos del plato para detectar el inicio y el fin vuelta.
      gearPlatecompleteCicleCounter--;
    } else {
      gearPlatecompleteCicleCounter = eStorage.gearPlatePulses;
    }

    if (gearPlatecompleteCicleCounter == 0 && pulseInit > 0) { // Si el pulso es final y tenemos timer de inicio
      completeGearPlateCicle = true; // Activamos el flag de ciclo completo.
      pulseEnd = millis(); // Seteamos el timer de fin
    } else if (gearPlatecompleteCicleCounter == eStorage.gearPlatePulses) { // Si el pulso es inicial
      pulseInit = millis(); // Seteamos el timer de inicio
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

void changeModeInt() {
  if (millis() > debounceChangeModeTime + DEBOUNCE_TIMETHRESHOLD_CHANGE_MODE) { //Debounce
    if (digitalRead(BRAKE_IN) == LOW) {
      powerModeChangeTrigger = 2; // cambia el modo de desaceleración
    } else {
      powerModeChangeTrigger = 1; // cambia el modo de aceleración
    }
  }
  debounceChangeModeTime = millis();//Debounce
}

void changeModeListener() {
  if (powerModeChangeTrigger == 1) {
    powerModeChangeTrigger = 0;
    if (eStorage.powerMode < (sizeof(POWER_STEPTS) / sizeof(int) - 1)) {
      eStorage.powerMode++;
    } else {
      eStorage.powerMode = 0;
    }

    blinkLed(STATUS_LED_OUT, eStorage.powerMode + 1, (600 / (eStorage.powerMode + 1)));
    powerCurrentValue = eStorage.powerMinValue;
  } else if (powerModeChangeTrigger == 2) {
    powerModeChangeTrigger = 0;
    if (eStorage.powerBrakeMode < (sizeof(POWER_STEPTS) / sizeof(int) - 1)) {
      eStorage.powerBrakeMode++;
    } else {
      eStorage.powerBrakeMode = 0;
    }
    blinkLed(STATUS_LED_OUT, eStorage.powerBrakeMode + 1, (600 / (eStorage.powerBrakeMode + 1)));
    powerCurrentValue = eStorage.powerMinValue;
  }
}


void reset(boolean brake) {
  gearPlateLastPulseTime = 0; // Reiniciamos el tiempo de control de rebote.
  gearPlatecompleteCicleCounter = 0; // Reiniciamos la posición del contador de vueltas de plato.
  pulseEnd = 0; // Reiniciamos el timer de inicio de pedalada.
  pulseInit = 0; // Reiniciamos el timer de fin de pedalada.
  maxPowerValue = eStorage.powerMinValue;
  if (brake) {
    powerCurrentValue = eStorage.powerMinValue;
  }
}

// init methods
void initFlashLeds() {
  //Serial.print(" .");
  blinkLed(STATUS_LED_OUT, eStorage.powerMode + 1, 60); // Muestra el modo de aceleración.
  runDelay(1000);
  blinkLed(STATUS_LED_OUT, eStorage.powerBrakeMode + 1, 60); // Muestra el modo de desaceleración.
  runDelay(1500);
  digitalWrite(STATUS_LED_OUT, HIGH); // Muestra el led de fin de inicialización.
  runDelay(500);
  digitalWrite(STATUS_LED_OUT, LOW);
  //Serial.println(".");
}

// Serial inputs methods
void serialRcvATCommandWithEndMarkerListener() { // Espera un comando AT con un buffer de hasta serialBufferSize (32)
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  while (Serial.available() > 0 && newSerialData == false) {
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
void manageATCommands() {

  if (newSerialData == true) {
    blinkLed(STATUS_LED_OUT, 1, 500);
    String command = getAtData(receivedChars, '=' , 0);
    command.toLowerCase();
    int value = getAtData(receivedChars, '=' , 1).toInt();

    oled1306.clearDisplay();
    oled1306.setCursor(0, 0);
    oled1306.print("> ");
    String tmpCommand = command;
    tmpCommand.toUpperCase();
    oled1306.print(tmpCommand);
    if (value > 0) {
      oled1306.print("?");
      oled1306.print(value);
    }
    oled1306.setCursor(0, 10);

    if (command.indexOf("at+save") > -1) {
      updateEepromData();

    } else if (command.indexOf("at+init") > -1) {
      initDefaultEepromData();

    } else if (command.indexOf("at+eelist") > -1) {
      //printEeprom();
      showEepromDataScreen();
      
    } else if (command.indexOf("at+gpp") > -1) {
      eStorage.gearPlatePulses = ((value <= eStorage.gearPlatePulses) && (value >= MIN_GEAR_PLATE_PULSES)) ? value : MAX_GEAR_PLATE_PULSES;
      oled1306.print("> gearPlatePulses: ");
      oled1306.print(eStorage.gearPlatePulses);
      
    } else if (command.indexOf("at+pwrup") > -1) { // modo de incremento de potencia progresiva.
      eStorage.powerMode = (value < (sizeof(POWER_STEPTS) / 2)) ? value : POWER_STEPTS_DEFAULT_POSITION;
      oled1306.print("> powerBrakeMode: ");
      oled1306.print(eStorage.powerMode);

    } else if (command.indexOf("at+pwrdw") > -1) { // modo de decremento de potencia progresiva.
      eStorage.powerBrakeMode = (value < (sizeof(POWER_STEPTS) / 2)) ? value : POWER_STEPTS_DEFAULT_POSITION;
      oled1306.print("> powerBrakeMode: ");
      oled1306.print(eStorage.powerBrakeMode);

    } else if (command.indexOf("at+mpuon") > -1) {
      eStorage.mpuenabled = true;

    } else if (command.indexOf("at+mpuoff") > -1) {
      eStorage.mpuenabled = false;
      ax = 0; ay = 0; az = 1;

    } else if (command.indexOf("at+shutdown") > -1) {
      asm volatile ("  jmp 0");

    } else if (eStorage.mpuenabled) {

      if (command.indexOf("at+anglemaxpwr") > -1) { // ángulo para la máxima potencia;
        eStorage.maxPowerAngle = value < DEFAULT_MAX_POWER_ANGLE ? value : DEFAULT_MAX_POWER_ANGLE;
        oled1306.print("> maxPowerAngle: ");
        oled1306.print(eStorage.maxPowerAngle);

      } else if (command.indexOf("at+level") > -1) { // Calibrar posición de placa.
        float accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
        float accel_ang_y = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
        eStorage.levelXAngle = (int) accel_ang_x;
        eStorage.levelYAngle = (int) accel_ang_y;
        oled1306.print("> [X|Y]: ");
        oled1306.print(eStorage.levelXAngle);
        oled1306.print(" | ");
        oled1306.print(eStorage.levelYAngle);
        oled1306.display();
      }
    }

    newSerialData = false;
    blinkLed(STATUS_LED_OUT, 15, 30);
  }
}

String getAtData(String data, char separator, int index) { // Split string and get index data.
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void initDefaultEepromData() {
  //Serial.println("\n > Initializing default config...");
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
  eStorage.gearPlatePulses=MAX_GEAR_PLATE_PULSES;

  updateEepromData();
  blinkLed(STATUS_LED_OUT, 15, 20);
}

void updateEepromData() {
  showEepromDataScreen();
  EEPROM.put(EEPROM_INIT_ADDRESS, eStorage); // Actualizan los datos de modos entrega de potencia en la eeprom
}

// **********  Sceen methods

void showPedalIcon(uint16_t color) {
  int y, x;
  for (y = 27; y <= 31; y++) {
    for (x = 122; x < 126; x++) {
      oled1306.drawPixel(x, y, color);
    }
  }
  oled1306.display();
}

void showBrakeSceen() {
  int y, x;
  // Clean display
  for (y = 0; y <= 31; y++) {
    for (x = 12; x < 126; x++) {
      oled1306.drawPixel(x, y, BLACK);
    }
  }
  oled1306.setCursor(45, 10);
  oled1306.print("BRAKE!!!");
  oled1306.display();
}

void showMaxPowerScreen(float angle, int timeCicle) {

  float coutput = ((float)(maxPowerValue * 3.58) / 4800);
  float coutput1 = ((float)(powerCurrentValue * 3.58) / 4800);

  oled1306.clearDisplay();
  oled1306.setCursor(0, 0);
  oled1306.print("> Y:");
  oled1306.print(angle);
  oled1306.print(" PULSE :");
  oled1306.print(timeCicle);

  oled1306.setCursor(0, 10);
  oled1306.print("> maxPwr: ");
  oled1306.print(coutput);
  //oled1306.print(maxPowerValue);
  oled1306.setCursor(0, 20);
  oled1306.print("> curPwr: ");
  oled1306.print(coutput1);
  //oled1306.print(powerCurrentValue);

  oled1306.display();
}

void showEepromDataScreen() {
  oled1306.setCursor(0, 10);
  oled1306.print("> pwr M|BM: ");
  oled1306.print(eStorage.powerMode);
  oled1306.print("|");
  oled1306.print(eStorage.powerBrakeMode);
  oled1306.setCursor(0, 20);
  if (eStorage.mpuenabled) {
    oled1306.print("> mPW:");
    oled1306.print(eStorage.maxPowerAngle);
    oled1306.print(" X:");
    oled1306.print(eStorage.levelXAngle);
    oled1306.print(" Y:");
    oled1306.print(eStorage.levelYAngle);
  } else {
    oled1306.print("> mpu DISABLED");
  }
  //eStorage.gearPlatePulses
  oled1306.display();
}

void showHomeScreen() {
  oled1306.clearDisplay();
  oled1306.setTextSize(1);
  oled1306.setTextColor(WHITE);

  oled1306.setCursor(15, 0);
  oled1306.print("FIIDO ASSISTANCE");
  oled1306.setCursor(40, 10);
  oled1306.print("PROJECT");
  oled1306.setCursor(80, 20);
  oled1306.print(VERSION);
  oled1306.display();
  oled1306.setCursor(90, 10);
  oled1306.print("*");
  oled1306.display();
  delay(90);
  oled1306.print("*");
  oled1306.display();
  delay(90);
  oled1306.setCursor(20, 10);
  oled1306.print("*");
  oled1306.display();
  delay(90);
  oled1306.print("*");
  oled1306.display();
  delay(1500);
  oled1306.clearDisplay();
  oled1306.display();
}

// **********  Utils methods

void runDelay(int dlay) { // Ejecuta un delay y sacamos puntos por la consola.
  int repeat = dlay / 100;
  for (int i = 0; i <= repeat; i++) {
    delay(100);
    //Serial.print(".");
  }
}

void blinkLed(int ledPin, int repeats, int time) { // Ejecuta un parpadeo en el led durante x repeticiones y con una frecuencia de x_ms.
  for (int i = 0; i < repeats; i++) {
    digitalWrite(ledPin, HIGH);
    delay(time);
    digitalWrite(ledPin, LOW);
    delay(time);
  }
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
