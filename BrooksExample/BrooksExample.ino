/*Firmware to communicate with Brooks MFC and MFM
*
*Author: David Walker
 */
 
// Libraries
#include <PID_v1.h>
#include <math.h>
#include <string.h>
#include <BrooksMaster.h>

//Pins
const int Blower_Pin = 6, RS485_Pin = 2;

const unsigned long Software_Version = 1;
const float R = 287;

//Variables
double Sheath_Flow_SP = 2.25*0.5, Sample_Flow_SP = 1.5*0.5;
double Sheath_Flow = 0, Sample_Flow = 0;
double Sheath_Flow_Drive = 0;
double Makeup_SP = 0.843, Makeup = 0; 

//PID values
float SheathFlowPID_P = 100, SheathFlowPID_I = 100;

union ifloat {
    byte b[4];
    float f;
    unsigned long u;
};

// instantiate BrooksMaster object
BrooksMaster Brooks1, Brooks2, Brooks3;

void EnableRecieve(){
  digitalWrite(RS485_Pin, LOW);
}

void EnableTransmit(){
  digitalWrite(RS485_Pin, HIGH);
}

PID SheathFlowPID(&Sheath_Flow, &Sheath_Flow_Drive, &Sheath_Flow_SP, SheathFlowPID_P, SheathFlowPID_I, 0, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("setting up");
  // Initialise pins
  analogWriteResolution(12); // Analogue pins use 12 bit resolution
  analogWrite(Blower_Pin, Sheath_Flow_Drive);
  pinMode(RS485_Pin, OUTPUT); 

  Serial2.begin(38400, SERIAL_8O1); //for RS484 to Brooks
  Brooks1.begin(1, Serial2);
  Brooks2.begin(2, Serial2);
  Brooks3.begin(3, Serial2);

  //callback function to to enable an RS485 transceiver's Receiver Enable pin, and disable its Driver Enable pin.
  Brooks1.postTransmission(EnableRecieve);
  Brooks1.preTransmission(EnableTransmit);
  Brooks2.postTransmission(EnableRecieve);
  Brooks2.preTransmission(EnableTransmit);
  Brooks3.postTransmission(EnableRecieve);
  Brooks3.preTransmission(EnableTransmit);

  delay(1000);

  Set_Brooks(Sample_Flow_SP, Brooks1);
  Set_Brooks(Makeup_SP, Brooks3);
  SheathFlowPID.SetOutputLimits(0, 4095);
  SheathFlowPID.SetMode(AUTOMATIC);
}

unsigned long loop_start, dT;

void loop() {
  // put your main code here, to run repeatedly:
  loop_start = micros();
  
  Sample_Flow = Read_Brooks(Brooks1);
  Sheath_Flow = Read_Brooks(Brooks2);
  Makeup = Read_Brooks(Brooks3);
  Serial.print("Sample flow: ");
  Serial.print(Sample_Flow);
  Serial.print(" Sheath flow: ");
  Serial.print(Sheath_Flow);
  Serial.print(" Makeup flow: ");
  Serial.println(Makeup);
  
  //Run Flow PID
  SheathFlowPID.Compute();
  analogWrite(Blower_Pin, Sheath_Flow_Drive);
  Serial.print("Blower driver: ");
  Serial.println(Sheath_Flow_Drive);
 
  dT = micros() - loop_start;
  Serial.print("Micros6 ");
  Serial.println(dT);

  while (dT < 50000){
    delay(1);
    dT = micros() - loop_start;
  }
  //delay(500);
}

float Read_Brooks(BrooksMaster node){
  int result = node.readRegister(1); //returns 0 on success
  Serial.print("Status: ");
  Serial.print(node.getStatusBuffer(0));
  Serial.print(" ");
  Serial.println(node.getStatusBuffer(1));
  if (result != 0) {
    // If no response from the slave, print an error message
    Serial.print("Communication error ");
    Serial.println(result, HEX);
  }
  else {
    // If all OK
    //Serial.print("Flow : ");
    // Print the read data from the slave
    ifloat measured;
    measured.b[3] = node.getResponseBuffer(1);
    measured.b[2] = node.getResponseBuffer(2);
    measured.b[1] = node.getResponseBuffer(3);
    measured.b[0] = node.getResponseBuffer(4);
    Serial.print(node.getResponseBuffer(0), HEX);
    Serial.print(measured.b[3], HEX);
    Serial.print(measured.b[2], HEX);
    Serial.print(measured.b[1], HEX);
    Serial.println(measured.b[0], HEX);
    //Serial.println(measured.f);
    measured.f = measured.f;//(2.0*0.01*measured.f * 1.292); //reading in Brooks SLPM 0°C and 1013mbar converted to g/min
    return measured.f;
  }
  
}

int Set_Brooks(float setpoint, BrooksMaster node){
  ifloat Flow;
  Flow.f = 100*(setpoint / 1.292)/2.0; //setpoint in g/min converted to Burkert SLPM 0°C and 1013mbar
  node.clearTransmitBuffer();
  node.setTransmitBuffer(0, 250); //set unit
  node.setTransmitBuffer(1, Flow.b[3]);
  node.setTransmitBuffer(2, Flow.b[2]);
  node.setTransmitBuffer(3, Flow.b[1]);
  node.setTransmitBuffer(4, Flow.b[0]);
  int result = node.writeRegister(236); //returns 0 on success
  Serial.print("Status: ");
  Serial.print(node.getStatusBuffer(0), BIN);
  Serial.print(" ");
  Serial.println(node.getStatusBuffer(1), BIN);
  if (result != 0) {
    // If no response from the slave, print an error message
    Serial.print("Communication error ");
    Serial.println(result, HEX);
  }
  else {
    // If all OK
    Serial.println("Flow set ");
    // Print the read data from the slave
    //Serial.println(node.getResponseBuffer(0));
  }
  
  
  return result;
}
