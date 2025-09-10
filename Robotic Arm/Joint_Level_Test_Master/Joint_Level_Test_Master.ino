#include <SoftwareSerial.h>
#include <CRC16.h>  // Use CRC16


// Debug flag (1 = enable, 0 = disable)
const uint8_t debugFlag = 0;




const uint8_t RS485_TX_PIN = 8;
const uint8_t RS485_RX_PIN = 9;
const uint8_t RS485_DE_PIN = 10;

SoftwareSerial rs485(RS485_RX_PIN, RS485_TX_PIN);

const uint8_t TARGET_JOINT_ID = 1;
const unsigned long POLL_INTERVAL = 250;

// CRC calculator
CRC16 crc;

// State machine states
enum State { WAIT_STX, READ_PACKET, CHECK_CRC };

void setup() 
{
  Serial.begin(115200);
  rs485.begin(38400);  // Match
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);  // Start in receive mode

  Serial.println("Master initialized.");
}

void loop() 
{
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll > POLL_INTERVAL) 
  {
    sendPoll();
    receiveAndParseResponse();
    lastPoll = millis();
  }
}


void sendPoll() 
{
  uint8_t pollMessage[5];
  pollMessage[0] = 0x02;  // STX
  pollMessage[1] = TARGET_JOINT_ID;
  pollMessage[2] = 0x01;  // Poll CMD
  crc.restart();
  crc.add(pollMessage[0]);
  crc.add(pollMessage[1]);
  crc.add(pollMessage[2]);
  uint16_t calcCRC = crc.getCRC();
  // MSB first
  pollMessage[3] = (calcCRC >> 8) & 0xFF;
  pollMessage[4] = calcCRC & 0xFF;

  if (debugFlag) {
    for (int i = 0; i < 5; i++) 
    {
      Serial.print(pollMessage[i], HEX); 
      Serial.print(" : "); 
      Serial.println(" (HEX)");
    }
  }

  digitalWrite(RS485_DE_PIN, HIGH);
  delay(1);
  rs485.write(pollMessage, 5);   // <-- fixed: send all 5 bytes (includes both CRC bytes)
  rs485.flush();
  delay(1);
  digitalWrite(RS485_DE_PIN, LOW);

  if (debugFlag) Serial.println("Poll sent to joint " + String(TARGET_JOINT_ID));
}

// New helper function to print parsed data in human-readable format
void printReceivingData(uint16_t motorAngle, uint16_t gearboxAngle, int16_t slippage, bool limit1, bool limit2) {

  Serial.print("Motor Encoder: ");
  Serial.print(motorAngle / 10.0);
  Serial.print(" Degree ||| ");

  Serial.print("Gearbox Encoder: ");
  Serial.print(gearboxAngle / 10.0);
  Serial.print(" Degree ||| ");

  Serial.print("Limit Switch 1: ");
  Serial.print(limit1 ? 1 : 0);
  Serial.print(" ||| ");

  Serial.print("Limit Switch 2: ");
  Serial.print(limit2 ? 1 : 0);
  Serial.print(" ||| ");

  Serial.print("Slippage: ");
  Serial.println(slippage / 10.0);
}

// Receive with state machine
void receiveAndParseResponse() 
{

  if (debugFlag) Serial.println("RECEIVING DATA...");

  State state = WAIT_STX;
  uint8_t response[12];
  uint8_t index = 0;
  unsigned long timeout = millis() + 200;  // 100 ms timeout

  while (millis() < timeout) 
  {
    if (rs485.available()) 
    {
      uint8_t byte = rs485.read();

      switch (state) 
      {
        case WAIT_STX:
          if (debugFlag) Serial.println("WAIT_STX");
          if (byte == 0x02) {
            response[index++] = byte;
            state = READ_PACKET;
          }
          break;
        case READ_PACKET:
          if (debugFlag) Serial.println("READ_PACKET");
          response[index++] = byte;
          
          if (index == 12) {
            state = CHECK_CRC;
            if (debugFlag) Serial.println("CHECK CRC IF");

            // Directly run CRC check
            crc.restart();
            for (int i = 0; i < 10; i++) {
              crc.add(response[i]);
            }
            uint16_t calcCRC = crc.getCRC();
            uint16_t recvCRC = ((uint16_t)response[10] << 8) | response[11];

            if (debugFlag) {
              Serial.print("calcCRC: "); Serial.println(calcCRC);
              Serial.print("recvCRC: "); Serial.println(recvCRC);
            }

            if (calcCRC == recvCRC) 
            {
              uint8_t id = response[1];
              uint8_t cmd = response[2];
              uint16_t motorAngle = (response[3] << 8) | response[4];
              uint16_t gearboxAngle = (response[5] << 8) | response[6];
              int16_t slippage = (response[7] << 8) | response[8];
              uint8_t flags = response[9];
              bool limit1 = flags & 0x01;
              bool limit2 = flags & 0x02;

              if (limit1 && limit2) {
                Serial.println("!!! EMERGENCY PACKET DETECTED !!!");
              } else {
                printReceivingData(motorAngle, gearboxAngle, slippage, limit1, limit2);
              }

            } 
            else 
            {
              Serial.println("CRC mismatch.");
              while (rs485.available()) rs485.read(); // Clear buffer to prevent delayed parsing
              state = WAIT_STX; // Resync
            }

            return;  // Exit after processing
          }

          break;
      }
    }
  }

  if (debugFlag) {
    for (int i = 0; i < 12; i++) 
    {
      Serial.print(response[i], HEX);
      Serial.print(" : ");
      Serial.println(" (HEX)");
    }
  }

  Serial.println("Timeout: No valid response received.");
  
}
