// include the library
#include <RadioLib.h>

// SX1262 has the following connections:
// NSS pin:   7
// DIO1 pin:  6
// NRST pin:  4
// BUSY pin:  5
SX1262 radio = new Module(7, 6, 4, 5, SPI);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif

void setFlag(void) {
  // we got a packet, set the flag
  receivedFlag = true;
}

void setup() {

  pinMode(21, OUTPUT);
  // 设置引脚21为低电平（常亮）
  digitalWrite(21, LOW);

  Serial.begin(9600);
  pinMode(21, OUTPUT);
  pinMode(45, OUTPUT);
  digitalWrite(21, HIGH);
  digitalWrite(45, HIGH);

  SPI.begin(8, 17, 18, 7);
  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  
  int state = radio.begin(868.0, 125.0, 7, 7, 0x34, 22, 8, 3.3);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // set the function that will be called
  // when new packet is received
  radio.setPacketReceivedAction(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[SX1262] Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  radio.setRxBoostedGainMode(true);
}

void loop() {
  if(receivedFlag) {
    receivedFlag = false;
    String str;
    int state = radio.readData(str);

    if (state == RADIOLIB_ERR_NONE) {
      // Parse and format output
      Serial.println("\n=== Sensor Data ===");
      Serial.println("Raw Data: " + str);
      
      // Parse key-value pairs
      parseSensorData(str);
      
      // Signal quality
      Serial.print("RSSI: ");
      Serial.print(radio.getRSSI());
      Serial.println(" dBm");
      Serial.print("SNR: ");
      Serial.print(radio.getSNR());
      Serial.println(" dB");
    }
  }
}

// Function to parse sensor data
void parseSensorData(String rawData) {
  // Example: Temp:25.5,Hum:60.0,X:0.12,Y:-0.03,Z:9.81
  int start = 0;
  while (start < rawData.length()) {
    int end = rawData.indexOf(',', start);
    if (end == -1) end = rawData.length();
    
    String pair = rawData.substring(start, end);
    int colon = pair.indexOf(':');
    if (colon != -1) {
      String key = pair.substring(0, colon);
      String value = pair.substring(colon + 1);
      
      Serial.print(key);
      Serial.print(": ");
      Serial.print(value);
      if (key == "Temp") Serial.print(" °C");
      else if (key == "Hum") Serial.print(" %");
      else if (key.startsWith("X") || key.startsWith("Y") || key.startsWith("Z")) Serial.print(" m/s²");
      Serial.println();
    }
    start = end + 1;
  }
}

