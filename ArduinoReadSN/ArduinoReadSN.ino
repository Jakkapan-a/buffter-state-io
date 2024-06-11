// Define digital pin
const int inputPin = 8;  // Change this to your digital input pin
const int bufferSize = 10;
int buffer[bufferSize];
int index = 0;
unsigned long lastTime = 0;

void setup() {
  pinMode(inputPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Read every 1ms
  if (millis() - lastTime >= 1) {
    lastTime = millis();
    int currentState = digitalRead(inputPin);

    // Store the current state in the buffer
    buffer[index] = currentState;
    index = (index + 1) % bufferSize;
    // Check if the buffer is full and then process
    if (index == 0) {
      processBuffer();
    }
  }else if(millis() < lastTime){
    lastTime = millis();
  }
}

bool old_state, state = false;
void processBuffer() {
  int countHigh = 0;
  int countLow = 0;

  // Count the number of highs and lows in the buffer
  for (int i = 0; i < bufferSize; i++) {
    if (buffer[i] == HIGH) {
      countHigh++;
    } else {
      countLow++;
    }
  }

  // Check if there is a significant change
  if (countHigh > bufferSize * 0.5) {
    state = true;
  } else if (countLow > bufferSize * 0.5) {
    state = false;
  }

  if (state != old_state) {
    if (state) {
      Serial.println("Event: Majority is HIGH");
    } else {
      Serial.println("Event: Majority is LOW");
    }
    old_state = state;
  }
}
