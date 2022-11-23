
#include <HardWire.h>
#include <VL53L0X.h>
#include <I2C_MPU6886.h>

#include <Ethernet.h>
#include <EthernetUdp.h>


#include "timer.h"

Timer timer1;
int counter = 0;
int direction[] = {0,-1,0,1};
int speed[] = {0,58,0,40};

//motor variables
int PWM =  11;     //motor PWM pin Orange
int IN2 = 6;       //motor controller pin2 Green
int IN1 = 7;       //motor controller pin1 Yellow

VL53L0X range_sensor;
I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire);

IPAddress ip(192, 168, 10, 240);
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

EthernetUDP udp_server;

char packet_buffer[UDP_TX_PACKET_MAX_SIZE];

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  Ethernet.begin(mac, ip);
  delay(500);
  
  if(!initialize())
     while(true)
       delay(10);

  imu.begin();  
  range_sensor.startContinuous();

  udp_server.begin(8888);
  Serial.println("Setup complete");
   // DC MOTOR
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

bool initialize()
{
  range_sensor.setTimeout(500);
  if(!range_sensor.init())
  {
    Serial.println("Failed to detect and initialize range sensor!");
    return false;
  }
  
  if(Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("Ethernet shield was not found.");
    return false;
  }
  else if(Ethernet.hardwareStatus() == EthernetW5500) 
    Serial.println("Found W5500 ethernet shield");

  if(Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Ethernet::LinkOff: is the cable connected?");
    return false;
  }
  else if(Ethernet.linkStatus() == LinkON)
    Serial.println("Ethernet::LinkOn");
  return true;
}
  
void loop() 
{ 
  int packet_size = udp_server.parsePacket();
  if(packet_size) 
  {
    float accel[3];
    float gyro[3];
    float t;
    float d;
  
    imu.getAccel(&accel[0], &accel[1], &accel[2]);
    imu.getGyro(&gyro[0], &gyro[1], &gyro[2]);
    imu.getTemp(&t);
    d = range_sensor.readRangeContinuousMillimeters();

    String sensor_values;
    sensor_values.concat(accel[0]); sensor_values.concat(",");
    sensor_values.concat(accel[1]); sensor_values.concat(",");
    sensor_values.concat(accel[2]); sensor_values.concat(",");
    sensor_values.concat(d);

    udp_server.read(packet_buffer, UDP_TX_PACKET_MAX_SIZE);
    float estimate = String(packet_buffer).toFloat();
    Serial.print("Est: ");Serial.println(estimate);

    udp_server.beginPacket(udp_server.remoteIP(), udp_server.remotePort());
    udp_server.write(sensor_values.c_str(), sensor_values.length());
    udp_server.endPacket();

   /**
    printVector3('A', accel);
    printVector3('G', gyro);
    printScalar('T', t);
    printScalar('D', d);
    printPackageMetaInfo(packet_size);
   **/
  }
  //Stearing
  if (timer1.timerHasExpired()) {
    //retning, power value
    counter = counter + 1;
    counter = counter % 4;
    timer1.getTimer(700);
  }
  setMotor(direction[counter], speed[counter], PWM, IN1, IN2);
  

}

void printVector3(char label, float *vector)
{
  Serial.print(label); Serial.print(" = ");
  Serial.print(vector[0] > 0.0 ? " " : "");Serial.print(vector[0]);Serial.print(",    \t");
  Serial.print(vector[1] > 0.0 ? " " : "");Serial.print(vector[1]);Serial.print(",    \t");
  Serial.print(vector[2] > 0.0 ? " " : "");Serial.println(vector[2]);
}

void printScalar(char label, float scalar)
{
  Serial.print(label); Serial.print(" = ");
  Serial.print(scalar > 0.0 ? " " : "");Serial.println(scalar);
}

void printPackageMetaInfo(int packet_size)
{
  Serial.print("Received packet of size ");
  Serial.println(packet_size);
  Serial.print("From ");
  IPAddress remote = udp_server.remoteIP();
  for(int i = 0; i < 4; i++) 
  {
    Serial.print(remote[i], DEC);
    if(i < 3) 
      Serial.print(".");
  }
  Serial.print(", port ");
  Serial.println(udp_server.remotePort());
}

//MOTOR
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}