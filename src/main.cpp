#include <Arduino.h>

#define temperatureSensor1 26
#define temperatureSensor2 27
#define relayStatusPin 21
#define relayOutputPin 22

#define waterFlowSensor1ON 2
#define waterFlowSensor1OFF 3

#define waterFlowSensor2ON 6
#define waterFlowSensor2OFF 7

volatile String incomingMessage = "";

volatile float lowestTemperature = 320;
volatile float highestTemperature = 910;

volatile float chosenTemperature = 0; 
volatile bool waterflowStatus = 0;

volatile bool isSensor1Connected = false;
volatile bool isSensor2Connected = false;

volatile bool isSensorT1Connected = false;
volatile bool isSensorT2Connected = false;

volatile bool error = false;



bool isSensorConnected(int onPin, int offPin) {
  return (digitalRead(onPin) && !digitalRead(offPin)) ||
         (!digitalRead(onPin) && digitalRead(offPin));
}

float readTemperature(int sensorPin) {
  int rawValue = analogRead(sensorPin);
  float temperature = map(rawValue, lowestTemperature, highestTemperature, 0, 100);
  return temperature;
}

String readUARTMessage() {
  String incomingMessage = "";

  while (Serial1.available() > 0) {
    char receivedChar = Serial1.read();
    incomingMessage += receivedChar;
    delay(10);
  }

  return incomingMessage;
}


void setup() {

  //input sensors (wf1,wf2,t1 and t2)
  pinMode(waterFlowSensor1ON, INPUT_PULLDOWN);
  pinMode(waterFlowSensor1OFF, INPUT_PULLDOWN);
  pinMode(waterFlowSensor2ON, INPUT_PULLDOWN);
  pinMode(waterFlowSensor2OFF, INPUT_PULLDOWN);

  pinMode(temperatureSensor1, INPUT);
  pinMode(temperatureSensor2, INPUT);

  //output (one to activate/deactivate the relay and other to check the status)
  pinMode(relayStatusPin, INPUT);
  pinMode(relayOutputPin, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(9600); // initialize UART communication via PINS  0 and 1 (RX and TX)

}

void loop() {
  while(1){ 
    //check if the sensors are connected correctly
    if(isSensorConnected(waterFlowSensor1ON, waterFlowSensor1OFF))
      isSensor1Connected = true;

    else
      isSensor1Connected = false;

    if(isSensorConnected(waterFlowSensor2ON, waterFlowSensor2OFF))
      isSensor2Connected = true;

    else
      isSensor2Connected = false;

    if (isSensor1Connected || isSensor2Connected) {
      error = false;
      if (isSensor1Connected) {
        waterflowStatus = digitalRead(waterFlowSensor1ON);
        Serial.println("Sensor 1 is active");
      }
      if (isSensor2Connected) {
        waterflowStatus = waterflowStatus || digitalRead(waterFlowSensor2ON);
        Serial.println("Sensor 2 is active");
      }
    } else {
      error = true;
    }

    if(error)
    {
      Serial.println("System has no active Waterflow sensors, therefore is not safe");
      digitalWrite(relayOutputPin, LOW);
      delay(250);
      continue;
    }

    int temperature1 = analogRead(temperatureSensor1);
    int temperature2 = analogRead(temperatureSensor2);

    //check if the temperature is in the range

    if(temperature1 > lowestTemperature && temperature1 < highestTemperature){

      temperature1 = readTemperature(temperatureSensor1);
      //Serial.print("Temperature 1: ");
      //Serial.println(temperature1);
      isSensorT1Connected = true;
    }
    else
      isSensorT1Connected = false;

    if(temperature2 > lowestTemperature && temperature2 < highestTemperature){

      temperature2 = readTemperature(temperatureSensor1);
      //Serial.print("Temperature 2: ");
      //Serial.println(temperature2);
      isSensorT2Connected = true;
    }
    else
      isSensorT2Connected = false;


    if(isSensorT1Connected && isSensorT2Connected){ 
      chosenTemperature = max(temperature1,temperature2);
      error = false;
    }
    else if(isSensorT1Connected && !isSensorT2Connected){
      chosenTemperature = temperature1;
      error = false;
    }
    else if(!isSensorT1Connected && isSensorT2Connected){ 
      chosenTemperature = temperature1;
      error = false; 
    }
    else 
      error = true; 

    if(error)
    {
      Serial.println("System has no active Temperature sensors, therefore is not safe");
      digitalWrite(relayOutputPin, LOW);
      delay(250);
      continue;
    }

    // Check if data is available on UART (Serial1)
    if (Serial1.available() > 0) {
        // Read the incoming message from the other system
        String incomingMessage = readUARTMessage();
        // Print the received message to the Serial Monitor
        Serial.print("Received message via UART: ");
        Serial.println(incomingMessage);
    }
    else {
        Serial.println("No data available on UART no information from the Other system");
    }

    //Send The Status of the relay to the other System via UART ( PINS 0 and 1, the default)
    Serial1.print("Relay Status: ");
    Serial1.println(digitalRead(relayStatusPin));

    //if all the sensors are well connected we proceed to the temperature measurement and relay activation 
    if(chosenTemperature >= 70 &&  !waterflowStatus){
      digitalWrite(relayOutputPin, LOW);
      Serial.println("Relay is deactivated, system is not safe to operate");
    }
    else{
      digitalWrite(relayOutputPin, HIGH);
      Serial.println("Relay is activated system is safe to operate");
    }

    Serial.print("Temperature: ");
    Serial.println(chosenTemperature);

    Serial.print("Waterflow: ");
    Serial.println(waterflowStatus);

    delay(250);
  }
}

