#include <RadioLib.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <DHT20.h>
#include <SoftwareSerial.h>
#include <IRremote.hpp>


// Display setup
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, 39, 38, U8X8_PIN_NONE);  // OLED display (128x64 pixels), using software I2C interface, clock pin 39, data pin 38

DHT20 dht20;
// Sensor variables
float temperature = 0;
float humidity = 0;

// LSM6DS3TR register addresses
// Defines the register addresses and sensitivity constants related to the LSM6DS3TR accelerometer for subsequent register read and write operations
#define LSM6DS3TR_ADDR 0x6B // Accelerometer I2C address
#define CTRL1_XL 0x10       // Accelerometer control register address
#define CTRL2_G  0x11       // Gyroscope control register address
#define OUTX_L_XL 0x28      // X-axis acceleration data address
#define ACCEL_SENSITIVITY 0.000122 // Accelerometer sensitivity (4g range)

// Write to a register
// This function is used to write an 8-bit value to the specified register of the LSM6DS3TR accelerometer.
// Communicates via the I2C bus. First, it starts the transmission, specifies the device address, then writes the register address and the value to be set, and finally ends the transmission.
void writeRegister(uint8_t reg, uint8_t value) {
  // 1. Start the I2C transmission and specify the accelerometer device address (0x6B)
  Wire.beginTransmission(LSM6DS3TR_ADDR);
  
  // 2. Write the address of the register to be operated on
  Wire.write(reg);
  
  // 3. Write the value to be set
  Wire.write(value);
  
  // 4. End the transmission (actually perform the write operation) - Actually perform the I2C write operation and release the I2C bus
  Wire.endTransmission();
}

// Read from a register
/*
This method is called a "repeated start condition" and is common in I2C read operations.
First, set the read pointer position, then read the data. This is a typical I2C read process.
*/
void readRegister(uint8_t reg, uint8_t *data, uint8_t length) {
  // 1. Start the I2C transmission and specify the accelerometer device address
  Wire.beginTransmission(LSM6DS3TR_ADDR);
  
  // 2. Write the address of the register to start reading from
  Wire.write(reg);
  
  // 3. End the transmission but do not release the bus (false parameter): The false parameter means to send a stop signal but do not release the bus
  Wire.endTransmission(false);
  
  // 4. Request to read the specified length of data from the device
  Wire.requestFrom(LSM6DS3TR_ADDR, length);
  
  // 5. Read the data byte by byte
  for (int i = 0; i < length; i++) {
    data[i] = Wire.read();
  }
}

// New: Define the sensor data structure
struct SensorData {
  float temperature;
  float humidity;
  float accelX;
  float accelY;
  float accelZ;
};


// Constants for button and LED control
// //This is the pin of the external device
const int buttonPin = 14;  // the number of the pushbutton pin
const int ledPin = 12;     // the number of the LED pin
volatile bool ledState = false;  // LED state flag to handle button press

// SoftwareSerial Setup
SoftwareSerial SoftSerial(16, 15);
unsigned char buffer[9600]; // buffer array for data receive over serial port
int count = 0;   // counter for buffer array


// LoRa Setup
// Radio Setup (SX1262)
// Wireless module pins    
SX1262 radio = new Module(7, 6, 4, 5, SPI);// 7：LR_NSS   6: LR_DIO1    4:LR_NRESET    5:LR_BUSY
// flag for packet transmission status
volatile bool transmittedFlag = false;     // Transmission completion flag
int transmissionState = RADIOLIB_ERR_NONE;  // variable to track transmission state


// Radio transmission flag function
void setFlag(void) {
  // Automatically called when the wireless module finishes sending a packet.
  // The main loop detects that transmittedFlag is true → Processes the sending result.
  transmittedFlag = true;  // Set the transmission completion flag
}

// IR Receiver Setup
const uint8_t IR_RECEIVER_PIN = 19;   // Infrared receiver pin

// Debounce variables
unsigned long lastButtonPressTime = 0;
unsigned long debounceDelay = 200;  // 200ms debounce delay

// Pin for constant LED
// This is the LED on the board
const int constantLedPin = 21;  // The pin for the constant LED
void LED()
{
  // Initialize constant LED pin
  pinMode(constantLedPin, OUTPUT);
  digitalWrite(constantLedPin, LOW);  // Turn on constant LED
}

void setup() {
  
  // Initialize the display
  u8g2.begin();// Use software I2C communication (pins 39-SCL, 38-SDA), set the display resolution to 128x64 pixels

  LED();

  // Initialize SoftwareSerial
  SoftSerial.begin(9600);// Initialize Serial1 for communication    Communicate with the GPS module
  // Initialize serial communication
  Serial.begin(9600);   // Communicate with the computer

  // Initialize DHT20 sensor
  Wire.begin(40, 41);     // Set the I2C pins SDA=13, SCL=14  , Initialize the I2C bus
  while (dht20.begin()) {   // Try to initialize the DHT20
    Serial.println("Initialize DHT20 sensor failed");
    delay(10);
  }

  // IR_RECEIVER_PIN is the infrared receiver pin
  IrReceiver.begin(IR_RECEIVER_PIN);

  Serial.println("IR Receiver Initialized.");

  // Initialize button and LED pins
  // This group is for the external button and bulb
  pinMode(buttonPin, INPUT);      // Set the button pin as an input
  pinMode(ledPin, OUTPUT);        // Set the LED pin as an output
  // Interrupt setup: Connect the button pin (14) to an interrupt, specify the interrupt service function as toggleLED, and the trigger condition is RISING (rising edge, i.e., when the button is pressed).
  attachInterrupt(digitalPinToInterrupt(buttonPin), toggleLED, RISING); 

  // Initialize LoRa module
  SPI.begin(8, 17, 18, 7); // Initialize SPI
  Serial.print(F("[SX1262] Initializing ... "));
  // Initialize the wireless module
  int state = radio.begin(
    868.0,       // Frequency: 868MHz
    125.0,       // Bandwidth: 125kHz
    7,           // Spreading factor: 7
    7,           // Coding rate: 4/7
    0x34,         // Sync word
    22,          // Output power: 22dBm
    8,           // Preamble length: 8 symbols
    3.3         // Use TCXO
  );

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("LoRa Initialized successfully!"));
  } else {
    Serial.print(F("LoRa initialization failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // Set the function to be called when a packet is transmitted
  radio.setPacketSentAction(setFlag);
  Serial.println("IR Receiver and LoRa Setup complete.");
  // Start transmitting the first packet
  transmissionState = radio.startTransmit("Hello World!");

  delay(10);  // Initial delay

}

void loop() {
  // // Get temperature and humidity from DHT20 sensor
  // 1. Collect sensor data
  SensorData data;
  data.temperature = dht20.getTemperature();
  data.humidity = dht20.getHumidity() * 100; // Convert to percentage

  // Display sensor readings on the screen
  u8g2.firstPage(); // Start a new page
  do {
    u8g2.setFont(u8g2_font_6x13_tf); // Set the font
    u8g2.setCursor(0, 30); // Set the text position
    u8g2.print("Temp: "); // Output text and data
    u8g2.print(data.temperature);
    u8g2.print("C");

    u8g2.setCursor(0, 50);
    u8g2.print("Humidity: ");
    u8g2.print(data.humidity);
    u8g2.print(" %RH");
  } while (u8g2.nextPage()); // Refresh the display page until all pages are displayed.

  // Output temperature and humidity to the Serial monitor
  Serial.print("Temperature: ");
  Serial.print(data.temperature);
  Serial.print("C");
  Serial.print("  Humidity: ");
  Serial.print(data.humidity);
  Serial.println(" %RH");

  // Read accelerometer data 
  uint8_t accelData[6];
  writeRegister(CTRL1_XL, 0x40);
  readRegister(OUTX_L_XL, accelData, 6);
  data.accelX = (int16_t)(accelData[0] | (accelData[1] << 8)) * ACCEL_SENSITIVITY * 9.80;
  data.accelY = (int16_t)(accelData[2] | (accelData[3] << 8)) * ACCEL_SENSITIVITY * 9.80;
  data.accelZ = (int16_t)(accelData[4] | (accelData[5] << 8)) * ACCEL_SENSITIVITY * 9.80;

  // 2. Build the sending string
  String payload = 
    "Temp:" + String(data.temperature, 1) + 
    ",Hum:" + String(data.humidity, 1) + 
    ",X:" + String(data.accelX, 2) +
    ",Y:" + String(data.accelY, 2) + 
    ",Z:" + String(data.accelZ, 2);

  // Software Serial communication
  if (SoftSerial.available()) {
    while (SoftSerial.available()) {
      char incomingByte = SoftSerial.read();
      Serial.print(incomingByte); // Print each received byte
      buffer[count++] = incomingByte;
      if (count == 256) break;
    }
    clearBufferArray(); // Clear buffer
    count = 0;
  }
  if (Serial.available()) {
    SoftSerial.write(Serial.read()); // Send data from Serial to SoftSerial
  }

  // Handle infrared signals
  if (IrReceiver.decode()) {  // Check if a valid infrared signal is received. If so, return true.
    handleIRCommand(); // Call this function to process the received infrared signal, such as printing the signal's protocol, command, and address.
    IrReceiver.resume(); // Resume() prepares to receive the next signal
  }


  // LoRa transmission logic
  // Check if the LoRa module has completed a data transmission.
  // transmittedFlag is set in the setFlag() callback function, which is triggered when the LoRa module finishes sending a packet.
  if (transmittedFlag) { // Check the transmission completion flag
    transmittedFlag = false; // If the transmission is completed, reset transmittedFlag to false.

    if (transmissionState == RADIOLIB_ERR_NONE) { // Report the transmission status (success/failure)
      Serial.println(F("transmission finished!"));
    } else {
      Serial.print(F("LoRa transmission failed, code "));
      Serial.println(transmissionState);
    }

    // 3. Send the data
    radio.finishTransmit();
    transmissionState = radio.startTransmit(payload);
    Serial.println("Sent: " + payload); // Debug output

  }
  delay(2000); // Change the sending interval to 2 seconds
}

void toggleLED() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastButtonPressTime > debounceDelay) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState ? HIGH : LOW);  // Set the LED state
    lastButtonPressTime = currentMillis;  // Update the last press time
  }
}

void clearBufferArray() {
  memset(buffer, 0, sizeof(buffer)); // Clear the buffer array
}

// IR command handling function
void handleIRCommand() {
  // Print IR data
  Serial.println("\n===== Detected Infrared Signal =====");

  Serial.print("- press -\t");
    
    // According to the value of IrReceiver.decodedIRData.command to determine the button
  switch (IrReceiver.decodedIRData.command) {
    case 0xA2:  // Corresponding to 162
      Serial.println("[CH-]");
      break;
    case 0x62:  // Corresponding to 98
      Serial.println("[CH]");
      break;
    case 0xE2:  // Corresponding to 226
      Serial.println("[CH+]");
      break;
    case 0x22:  // Corresponding to 34
      Serial.println("[PREV]");
      break;
    case 0x02:  // Corresponding to 2
      Serial.println("[NEXT]");
      break;
    case 0xC2:  // Corresponding to 194
      Serial.println("[PLAY/PAUSE]");
      break;
    case 0xE0:  // Corresponding to 224
      Serial.println("[VOL-]");
      break;
    case 0xA8:  // Corresponding to 168
      Serial.println("[VOL+]");
      break;
    case 0x90:  // Corresponding to 144
      Serial.println("[EQ]");
      break;
    case 0x68:  // Corresponding to 104
      Serial.println("[0]");
      break;
    case 0x98:  // Corresponding to 152
      Serial.println("[100+]");
      break;
    case 0xB0:  // Corresponding to 176
      Serial.println("[200+]");
      break;
    case 0x30:  // Corresponding to 48
      Serial.println("[1]");
      break;
    case 0x18:  // Corresponding to 24
      Serial.println("[2]");
      break;
    case 0x7A:  // Corresponding to 122
      Serial.println("[3]");
      break;
    case 0x10:  // Corresponding to 16
      Serial.println("[4]");
      break;
    case 0x38:  // Corresponding to 56
      Serial.println("[5]");
      break;
    case 0x5A:  // Corresponding to 90
      Serial.println("[6]");
      break;
    case 0x42:  // Corresponding to 66
      Serial.println("[7]");
      break;
    case 0x4A:  // Corresponding to 74
      Serial.println("[8]");
      break;
    case 0x52:  // Corresponding to 82
      Serial.println("[9]");
      break;
    default:
      Serial.println("[UNKNOWN]");
      break;
  }
  IrReceiver.resume();  // Continue to receive the next signal
}
