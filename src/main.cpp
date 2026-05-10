#include <HardwareSerial.h>
#include "state.h"

const uint8_t Seq[4] = {1, 4, 5, 6};

State *state;                    // Where the magic happens...
HardwareSerial monitorSerial(0); // Our USB tty... If present.
HardwareSerial crsfSerial(2);    // The CRSF module - on GPIO 16 for RX and GPIO 17 for TX.

void setup()
{
  monitorSerial.begin(115200);
  crsfSerial.begin(115200);
  state = new State(Seq, sizeof(Seq), crsfSerial, monitorSerial);
}

void loop() { state->loop(); }
