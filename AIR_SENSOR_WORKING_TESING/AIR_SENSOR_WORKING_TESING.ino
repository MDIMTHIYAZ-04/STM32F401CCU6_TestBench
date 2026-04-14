#include <ModbusRTU.h>

HardwareSerial ZphsSerial(PB7, PB6);
HardwareSerial rs485Serial(PA3, PA2);

#define RS485_CTRL PA8
ModbusRTU mb;

byte rxBuf[32];
int  rxLen    = 0;
bool gotResp  = false;
unsigned long lastSend = 0;

void setup() {
  pinMode(RS485_CTRL, OUTPUT);
  digitalWrite(RS485_CTRL, LOW);

  ZphsSerial.begin(9600);
  rs485Serial.begin(9600);

  mb.begin(&rs485Serial, RS485_CTRL);
  mb.slave(1);
  for (int i = 0; i < 20; i++) mb.addHreg(i);
}

void loop() {
  mb.task();

  // Send command every 3 seconds
  if (millis() - lastSend > 3000) {
    lastSend = millis();

    byte cmd[9] = {0xFF,0x01,0x86,0,0,0,0,0,0x79};
    while (ZphsSerial.available()) ZphsSerial.read();
    rxLen   = 0;
    gotResp = false;

    ZphsSerial.write(cmd, 9);
  }

  // Collect bytes
  while (ZphsSerial.available() && rxLen < 32) {
    rxBuf[rxLen++] = ZphsSerial.read();
    gotResp = true;
  }

  // Reg 0 = bytes received count
  mb.Hreg(0, rxLen);
  // Reg 1 = 1 if any bytes came back
  mb.Hreg(1, gotResp ? 1 : 0);
  // Reg 2 = first byte (should be 0xFF = 255)
  mb.Hreg(2, rxLen > 0 ? rxBuf[0] : 0);
  // Reg 3 = second byte (should be 0x86 = 134)
  mb.Hreg(3, rxLen > 1 ? rxBuf[1] : 0);
  // Reg 4 = total bytes expected
  mb.Hreg(4, 26);
}