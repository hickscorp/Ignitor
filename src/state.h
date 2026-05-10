#ifndef STATE_H
#define STATE_H

#include <stdio.h>
#include <Stream.h>

#define PATIENCE 100000                   // How long to wait between pression / depression sequences.
#define CHANNEL_COUNT 16                  // We assume 16 channels are being sent...
#define ELEVEN_BIT_MASK 0x07FF            // Masks 11 bits to keep and the rest to ignore.
#define MAX_PACKET_SIZE 64                // See file header.
#define FRAME_TYPE_RC_CHANNEL_PACKED 0x16 // Marks that a packet we got is for channels as opposed to telemetry.

#define TRG (6)

class State
{
public:
  enum class Phase
  {
    RequiresReset,
    Sequencing,
    Patience,
    Firing
  };

private:
  // Finite state machine state.
  const uint8_t *_seq;
  const uint8_t _seqLen;
  int8_t _seqIdx;

  Phase _phase;

  // Channels data.
  uint16_t _channels[CHANNEL_COUNT];
  // Stream to read receiver data from.
  Stream *_crsfStream;
  // Stream to write debug messages to.
  Stream *_monitorStream;

  // Time tracking / timer.
  uint32_t _tick = 0;
  uint32_t _lastEventTick = 0;

public:
  // Constructor.
  State(const uint8_t *const seq, uint8_t seqLen, Stream &crsfStream, Stream &monitorStream);
  // Update loop.
  void loop();

private:
  void readStream();
  bool updateChannels(const uint8_t *const packet, uint8_t length);

  bool checkTransition();
  bool transition(Phase phase);

  bool patienceNeeded();
  bool latched(uint8_t num);
  bool unlatched(uint8_t num);
};

#endif
