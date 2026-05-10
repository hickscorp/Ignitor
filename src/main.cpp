#include "main.h"

Iggy::State *state;           // State storage and logic.
HardwareSerial crsfSerial(1); // The CRSF serial.

void setup()
{
    crsfSerial.begin(420000, SERIAL_8N1, CRSF_RX, CRSF_TX);
    state = new Iggy::State(sizeof(seq), seq, io, &crsfSerial);
}

void loop() { state->loop(); }
