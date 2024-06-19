/***
 * @file RS485.ino
 * @brief RS485 communication
 * @details RS485 communication
 * @version 1.0
 * @date 2021-07-08
 * @author J. R.
 * @Board Arduino Mega 2560
 * @license MIT
 */

// #include <ModbusRTUSlave.h>
// #include <SoftwareSerial.h>
// bufferInput class boolean xxx state
class bufferInput {
public:
  bufferInput(int size = 20)
    : maxSize(size), currentSize(0) {
    state = new boolean[maxSize]();
  }

  ~bufferInput() {
    delete[] state;
  }

  void add(boolean value) {
    int index = currentSize % maxSize;
    state[index] = value;
    currentSize++;
  }

  boolean isState() {
    int count = 0;
    for (int i = 0; i < maxSize; i++) {
      if (state[i] == true) {
        count++;
      }
    }
    bool result = count > maxSize * 0.1;
    return result;
  }

private:
  int maxSize;
  int currentSize;
  boolean *state;
};

// ----------------- Outputs ----------------- //
#define BUZZER_PIN A14
// ----------------- Inputs ----------------- //
#define BUFFER_SIZE 25
#define INPUT_PIN1 A15
#define INPUT_PIN2 49
#define INPUT_PIN3 48
#define INPUT_PIN4 47
#define INPUT_PIN5 A13
#define INPUT_PIN6 45
#define INPUT_PIN7 44
#define INPUT_PIN8 43
#define INPUT_PIN9 42
#define INPUT_PIN10 41
#define INPUT_PIN11 40
#define INPUT_PIN12 39
#define INPUT_PIN13 38
#define INPUT_PIN14 37
#define INPUT_PIN15 36
#define INPUT_PIN16 35
#define INPUT_PIN17 34
#define INPUT_PIN18 33
#define INPUT_PIN19 32
#define INPUT_PIN20 31
#define INPUT_PIN21 30
#define INPUT_PIN22 29
#define INPUT_PIN23 28
#define INPUT_PIN24 27
#define INPUT_PIN25 A11
#define INPUT_PIN26 25
#define INPUT_PIN27 24
#define INPUT_PIN28 23
#define INPUT_PIN29 A12
#define INPUT_PIN30 2
#define INPUT_PIN31 3
#define INPUT_PIN32 4
#define INPUT_PIN33 5
#define INPUT_PIN34 6
#define INPUT_PIN35 7
#define INPUT_PIN36 8
#define INPUT_PIN37 9
#define INPUT_PIN38 10
#define INPUT_PIN39 11
#define INPUT_PIN40 12

int inputPins[] = { INPUT_PIN1, INPUT_PIN2, INPUT_PIN3, INPUT_PIN4, INPUT_PIN5, INPUT_PIN6, INPUT_PIN7, INPUT_PIN8, INPUT_PIN9, INPUT_PIN10, INPUT_PIN11, INPUT_PIN12, INPUT_PIN13, INPUT_PIN14, INPUT_PIN15, INPUT_PIN16, INPUT_PIN17, INPUT_PIN18, INPUT_PIN19, INPUT_PIN20, INPUT_PIN21, INPUT_PIN22, INPUT_PIN23, INPUT_PIN24, INPUT_PIN25, INPUT_PIN26, INPUT_PIN27, INPUT_PIN28, INPUT_PIN29, INPUT_PIN30, INPUT_PIN31, INPUT_PIN32, INPUT_PIN33, INPUT_PIN34, INPUT_PIN35, INPUT_PIN36, INPUT_PIN37, INPUT_PIN38, INPUT_PIN39, INPUT_PIN40 };

bufferInput bufferInput1(BUFFER_SIZE);
bufferInput bufferInput2(BUFFER_SIZE);
bufferInput bufferInput3(BUFFER_SIZE);
bufferInput bufferInput4(BUFFER_SIZE);
bufferInput bufferInput5(BUFFER_SIZE);
bufferInput bufferInput6(BUFFER_SIZE);
bufferInput bufferInput7(BUFFER_SIZE);
bufferInput bufferInput8(BUFFER_SIZE);
bufferInput bufferInput9(BUFFER_SIZE);
bufferInput bufferInput10(BUFFER_SIZE);
bufferInput bufferInput11(BUFFER_SIZE);
bufferInput bufferInput12(BUFFER_SIZE);
bufferInput bufferInput13(BUFFER_SIZE);
bufferInput bufferInput14(BUFFER_SIZE);
bufferInput bufferInput15(BUFFER_SIZE);
bufferInput bufferInput16(BUFFER_SIZE);
bufferInput bufferInput17(BUFFER_SIZE);
bufferInput bufferInput18(BUFFER_SIZE);
bufferInput bufferInput19(BUFFER_SIZE);
bufferInput bufferInput20(BUFFER_SIZE);
bufferInput bufferInput21(BUFFER_SIZE);
bufferInput bufferInput22(BUFFER_SIZE);
bufferInput bufferInput23(BUFFER_SIZE);
bufferInput bufferInput24(BUFFER_SIZE);
bufferInput bufferInput25(BUFFER_SIZE);
bufferInput bufferInput26(BUFFER_SIZE);
bufferInput bufferInput27(BUFFER_SIZE);
bufferInput bufferInput28(BUFFER_SIZE);
bufferInput bufferInput29(BUFFER_SIZE);
bufferInput bufferInput30(BUFFER_SIZE);
bufferInput bufferInput31(BUFFER_SIZE);
bufferInput bufferInput32(BUFFER_SIZE);
bufferInput bufferInput33(BUFFER_SIZE);
bufferInput bufferInput34(BUFFER_SIZE);
bufferInput bufferInput35(BUFFER_SIZE);
bufferInput bufferInput36(BUFFER_SIZE);
bufferInput bufferInput37(BUFFER_SIZE);
bufferInput bufferInput38(BUFFER_SIZE);
bufferInput bufferInput39(BUFFER_SIZE);
bufferInput bufferInput40(BUFFER_SIZE);

bufferInput bufferInputs[] = { bufferInput1, bufferInput2, bufferInput3, bufferInput4, bufferInput5, bufferInput6, bufferInput7, bufferInput8, bufferInput9, bufferInput10, bufferInput11, bufferInput12, bufferInput13, bufferInput14, bufferInput15, bufferInput16, bufferInput17, bufferInput18, bufferInput19, bufferInput20, bufferInput21, bufferInput22, bufferInput23, bufferInput24, bufferInput25, bufferInput26, bufferInput27, bufferInput28, bufferInput29, bufferInput30, bufferInput31, bufferInput32, bufferInput33, bufferInput34, bufferInput35, bufferInput36, bufferInput37, bufferInput38, bufferInput39, bufferInput40 };


// const uint8_t slaveID = 1;

// ModbusRTUSlave modBusRTUSlave(Serial);

bool coils[50] = { false };
boolean discreteInputs[50] = { false };
boolean endReceived = false;
String receivedData = "";
uint32_t timeStamp = 0;
void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    if (inChar == '\n')
    {
      endReceived = true;
    }
    else
    {
      receivedData += inChar;
      timeStamp = millis();
    }
  }
}

int TOTAL_INPUTS = 40;
boolean OFF = true;
boolean ON = false;
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  // Set input pins
  for (int i = 0; i < BUFFER_SIZE; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
  }

  // Set output pins
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, OFF);

  // Modbus
  // modBusRTUSlave.begin(slaveID, 9600);
  // modBusRTUSlave.configureCoils(coils, 50);
  // modBusRTUSlave.configureDiscreteInputs(discreteInputs, 50);
}
uint32_t lastTime100 = 0;
uint32_t lastTime200 = 0;
// bool oldState[40] = { false };
void loop() {
  // modBusRTUSlave.poll();
  uint32_t currentTime = millis();
  if (currentTime - lastTime100 > 40) {
    readInputs();
    lastTime100 = currentTime;
  } else if (currentTime < lastTime100) {
    lastTime100 = currentTime;
  }

  if (endReceived)
  {
    if(receivedData == "ON"){
      digitalWrite(BUZZER_PIN, ON);
    }else if(receivedData == "OFF"){
      digitalWrite(BUZZER_PIN, OFF);
    }
    receivedData = "";
    endReceived = false;
  }else if(currentTime - timeStamp > 500 && receivedData.length() > 0){
    endReceived = true;
  }
  // ----------------- Print Inputs ----------------- //
  if (currentTime - lastTime200 > 500) {
    // manageInputs();
    if (coils[42]) {
      digitalWrite(BUZZER_PIN, ON);
      coils[42] = false;
      coils[43] = true;  // For response to Modbus
    }

    if (coils[44]) {
      digitalWrite(BUZZER_PIN, OFF);
      coils[44] = false;
      coils[45] = true;  // For response to Modbus
    }
    
    // Update to serial
    Serial.print("{");
    for (int i = 0; i < TOTAL_INPUTS; i++) {
      Serial.print(bufferInputs[i].isState());
      if (i < TOTAL_INPUTS - 1) {
        Serial.print(",");
      }
    }
    Serial.println("}");
    
    lastTime200 = currentTime;
  } else if (currentTime < lastTime200) {
    lastTime200 = currentTime;
  }

}
void readInputs() {
  for (int i = 0; i < TOTAL_INPUTS; i++) {
    boolean value = digitalRead(inputPins[i]);
    bufferInputs[i].add(!value);  // Add to buffer
  }
}
void manageInputs() {
  coils[0] = true; // 
  discreteInputs[0] = true; // 

  for (int i = 0; i < TOTAL_INPUTS; i++) { //
    boolean state = bufferInputs[i].isState();
    coils[i + 1] = state; //  +1
    discreteInputs[i + 1] = state; // + 1
  }
}