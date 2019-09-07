// https://naylampmechatronics.com/blog/45_Tutorial-MPU6050-Acelerómetro-y-Giroscopio.html
// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;




boolean flag = true;
void setup() {
  Serial.begin(9600);   //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
    
  Serial.println("Offsets:");
  Serial.print(sensor.getXAccelOffset()); Serial.print("\t"); 
  Serial.print(sensor.getYAccelOffset()); Serial.print("\t"); 
  Serial.print(sensor.getZAccelOffset()); Serial.print("\t"); 
  Serial.print(sensor.getXGyroOffset()); Serial.print("\t"); 
  Serial.print(sensor.getYGyroOffset()); Serial.print("\t");
  Serial.print(sensor.getZGyroOffset()); Serial.print("\t");
  Serial.println("nnEnvie cualquier caracter para empezar la calibracionnn");  
  // Espera un caracter para empezar a calibrar
  while (true){
    if (Serial.available()) break;
  }  
  Serial.println("Calibrando..., no mover IMU");  
  
}
void loop(){
  if(flag){
    calibrate();
    flag=false;
  }
}

void calibrate() {

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
      Serial.print("promedio:"); Serial.print("\t");
      Serial.print(p_ax); Serial.print("\t");
      Serial.print(p_ay); Serial.print("\t");
      Serial.print(p_az); Serial.print("\t");
      Serial.print(p_gx); Serial.print("\t");
      Serial.print(p_gy); Serial.print("\t");
      Serial.println(p_gz);
  
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
      if(p_ax <= 10 && p_ax >= -10 && p_ay <= 10 && p_ay >= -10 && p_az <= 16394 && p_az >= 16374){
        Serial.print("\t Calibration Done ");      
        Serial.print((millis()-inittime)/1000);
        Serial.println("s");  
        calibdone=false;  
      }
      counter=0;
    }
    counter++;
  }
}
