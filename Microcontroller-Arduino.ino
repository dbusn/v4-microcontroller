#include <Wire.h>
#include <ArduinoJson.h>

// Use (uncomment) Serial if communicating over cable
//#define Connection Serial
// Use (uncomment) Serial5 if communicating over bluetooth
#define Connection Serial5

bool motors_currently_on[24] = {};

void setup() {

  // Wire: Pin 19/A5 is SCL0, pin 18/A4 is SDA0.
  Wire.setSCL(19);
  Wire.setSDA(18);

  // Wire1: Pin 16/A2 is SCL1, pin 17/A3 is SDA1.
  Wire1.setSCL(16);
  Wire1.setSDA(17);

  // Start both connections to i2c.
  Wire.begin();
  Wire1.begin();

  // Set serial connection with a set baudrate.
  Connection.begin(115200);

  // Reset speaker controllers.
  resetSpeakers();
}


void loop() {

  // Maximum memory can be higher for Teensy 4.0 if necessary.
  int memoryMaxUsable = 65536;

  if (Connection.available()) {
    // Allocate the JsonDocument.
    DynamicJsonDocument doc(memoryMaxUsable);

    // Parse the JSON input via Connection.
    DeserializationError error = deserializeJson(doc, Connection);

    // Parse succeeded?
    if (error) {
      // Log deserialization error to output.
      logDeserializationError(error);
      return;
    } else {
      
      JsonArray patternArray = doc["pattern"].as<JsonArray>();

      // Perform the vibration pattern.
      performVibrationPattern(patternArray);

      // End the line. DO NOT REMOVE. Python will wait forever if there is no end of the line.
      Connection.write("\n");
    }
  }
}

void performVibrationPattern(const JsonArray &patternArray) {
  // Loop through vibration pattern.
  for (JsonObject vibration : patternArray) { 
    int duration = vibration["time"];
    Connection.write("");

    // Log duration to output.
    logDuration(duration);

    // Set vibration speeds.
    setVibration(vibration);

    // Leave the motors on for a set duration.
    delay(duration);

    // Write carriage return for better readability in console.
    Connection.write("\r");
  }

  // Reset all speakers to non active after pattern
  resetSpeakers();
  memset(motors_currently_on, 0, sizeof(motors_currently_on));
}

/**
 * Sets the vibration using a JsonObject.
 */
void setVibration(const JsonObject &vibration) {
  // Read the patterns from the json
  JsonArray pinArray = vibration["iteration"].as<JsonArray>();

  // Read which motors will be activated in this new pattern and save them
  bool motors_in_new_pattern[24] = {};
  for (JsonObject patternIter : pinArray) {
    int motor = patternIter["coord"];
    motor = constrain(motor, 0, 24);
    motors_in_new_pattern[motor] = 1;
  }

  // Turn motors off that are currently on and will not be turned on in new pattern
  for (int motor = 0; motor < 24; motor = motor + 1) {
    if (motors_in_new_pattern[motor] == 0 && motors_currently_on[motor] == 1 || motors_in_new_pattern[motor] == 0 && motors_currently_on[motor] == 0) {
      disableAmp(motor);
    }
  }

  // Clear array 
  memset(motors_currently_on, 0, sizeof(motors_currently_on));

  for (JsonObject patternIter : pinArray) {
    // Read which motor will be turned on.
    int motor = patternIter["coord"];
    motor = constrain(motor, 0, 24);

    // Document that the motor is on.
    motors_currently_on[motor] = 1;

    int i2cID, potID, channelID;
    extractAmplitudeControllerInfo(motor, i2cID, potID, channelID);

    int frequencyPin = extractFrequencyControllerInfo(motor);

    int frequency = patternIter["frequency"];
    byte amplitude = patternIter["amplitude"];

    setFrequency(frequencyPin, frequency);
    logFrequency(frequencyPin, frequency);

    setAmplitude(i2cID, potID, channelID, amplitude);
    logAmplitude(i2cID, potID, channelID, motor, amplitude);
  }
}

/**
 * Function to extract from the given motor ID (between 0 - 24), what settings have to be used for the pots.
 */
void extractAmplitudeControllerInfo(int motorID, int &i2cID, int &potID, int &channelID) {
  if (motorID < 12) {
    i2cID = 0;
    potID = motorID / 4;
    channelID = motorID % 4;
  } else if (motorID >= 12) {
    i2cID = 1;
    potID = (motorID - 12) / 4;
    channelID = (motorID - 12) % 4;
  }
}

/**
 * Function to extract, based on a given motor ID (between 0 - 24) which frequency pin should be used.
 */
int extractFrequencyControllerInfo(int motorID) {
  int frequencyPins[] = {0, 1, 2, 4, 5, 6, 7, 10, 11, 11, 12, 12, 13, 13, 14, 14, 23, 23, 24, 24, 15, 15, 22, 22};

  return frequencyPins[motorID];
}

/**
 * Disables the frequency of a pin by reducing the duty cycle to 0%.
 */
void disableFrequency(int pin) {
  analogWrite(pin, 0);
  analogWriteFrequency(pin, 0);
}

/**
 * Disables the amplitude of a motor by setting it to 0.
 */
void disableAmp(int motor) {
  int i2cID, potID, channelID;
  extractAmplitudeControllerInfo(motor, i2cID, potID, channelID);
  setAmplitude(i2cID, potID, channelID, 0);
}

/**
 * Sets the frequency of a pin to the given frequency.
 */
void setFrequency(int pin, int frequency) {
  if (frequency == 0) {
    disableFrequency(pin);
  } else {
    // Force minimum and maximum frequency values.
    frequency = constrain(frequency, 20, 400);

    // Set frequency.
    analogWriteFrequency(pin, frequency);

    // As we want a square wave the duty cycle should be 50% and therefore set the value to 128.
    analogWrite(pin, 128);
  }
}


/**
 *  Sets the amplitude of a specific channel on a specific pot to a given amplitude.
 *
 * :i2cID:      The i2c bus to be targeted (either 0 or 1)
 * :potID:      The pot to be targeted (either 0, 1, 2 or 3)
 * :channelID:  The pot channel to be targeted (either 0, 1, 2 or 3)
 * :amplitude:  The amplitude to set the pot to (range from 0 to 256)
**/
void setAmplitude(int i2cID, int potID, int channelID, byte amplitude) {

  byte channel;
  switch (channelID)
  {
    case 0:
      channel = 0b00000010;
      break;
    case 1:
      channel = 0b00010010;
      break;
    case 2:
      channel = 0b01100010;
      break;
    case 3:
      channel = 0b01110010;
      break;
  }

  int potentiometerAddress;
  switch (potID)
  {
    case 0:
      potentiometerAddress = 44;
      break;
    case 1:
      potentiometerAddress = 45;
      break;
    case 2:
      potentiometerAddress = 46;
      break;
    case 3:
      potentiometerAddress = 47;
      break;
  }

  switch (i2cID)
  {
    case 0:
      // Begin transmission to the chosen pot meter.
      Wire.beginTransmission(potentiometerAddress);

      // Write which channel to target.
      Wire.write(channel);

      // Write value to set the wiper to.
      Wire.write((amplitude));

      // End transmission (this sends the whole packet as one).
      Wire.endTransmission();

      break;
    case 1:
      // Begin transmission to the chosen pot meter.
      Wire1.beginTransmission(potentiometerAddress);

      // Write which channel to target.
      Wire1.write(channel);

      // Write value to set the wiper to.
      Wire1.write((amplitude));

      // End transmission (this sends the whole packet as one).
      Wire1.endTransmission();

      break;
  }
}

/**
 * Function that resets all motors by turning all frequencies off and setting all 
 * pot meters to 0.
 */
void resetSpeakers() {
  for (int i = 0; i < 24; i = i + 1) {
    int i2cID, potID, channelID;
    extractAmplitudeControllerInfo(i, i2cID, potID, channelID);
    setAmplitude(i2cID, potID, channelID, 0);
    int freqPin = extractFrequencyControllerInfo(i);
    disableFrequency(freqPin);
  }
}

void logDuration(const int duration) {
  Connection.write("vibrating for ");
  writeNumberUnderThousandOnConnection(duration);
  Connection.write("ms --");
}

void logFrequency(int frequencyPin, int frequency) {
  Connection.write("- ");
  writeNumberUnderThousandOnConnection(frequency);
  Connection.write("Hz ");
}

void logAmplitude(int i2cID, int potID, int channelID, int motor, byte amplitude) {
  Connection.write("on motor ");
  writeNumberUnderThousandOnConnection(motor);
  Connection.write(" with amp ");
  int amp = amplitude;
  writeNumberUnderThousandOnConnection(amp);
  Connection.write(" ");
}

void logDeserializationError(const DeserializationError &error) {
  Connection.write("deserializeJson() returned ");
  Connection.write(error.c_str());
  Connection.write("\n");
}

/**
 * Function to write a number (integer) to the connection.
 */
void writeNumberUnderThousandOnConnection(int number) {
  char buf[100];
  Connection.write(itoa(number, buf, 10));
}
