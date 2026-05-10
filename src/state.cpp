#include "state.h"

State::State(const uint8_t *const seq, uint8_t seqLen, Stream &crsfStream, Stream &monitorStream)
    : // Sequence definition and tracking.
      _seq{seq}, _seqLen(seqLen), _seqIdx{0},
      // Phase of finite state machine.
      _phase{Phase::RequiresReset},
      // Various streams - one for reading channel data, one for writing debug.
      _crsfStream{&crsfStream}, _monitorStream{&monitorStream}
{
  _monitorStream->println("CRSF State constructed.");
}

void State::loop()
{
  // Pin the pointer to our state's channels...
  // Work variables.
  static uint8_t packet[MAX_PACKET_SIZE];
  static uint8_t idx = 0;

  // Within this loop iteration, consume as much as possible on the UART line.
  while (_crsfStream->available())
  {
    // Read one byte.
    uint8_t byte = _crsfStream->read();

    // Only process valid packets if we're starting a read and we're seeing a good
    // packet type:
    // - 0xC8 means receiver-addressed when following the standard...
    // - 0xEE when not following it that much :)
    if (idx == 0 && byte != 0xEE && byte != 0xC8)
      continue;

    // Store the received byte in our packet buffer and move the cursor to the next index...
    packet[idx++] = byte;
    // The ExpressLRS standard has the first byte being the target identifier, the second byte is the frame length
    // and the third byte is the type. So if our index is already past the header, we want to check if we have reached
    // the end of the current frame. That's the case when the index is equal to `packet[1] + 2`. When it occurs, we want
    // to decode the frame we've got...
    if (idx >= 2 && idx == packet[1] + 2)
    {
      // Only process that packet if it's a channel update packet.
      // TODO: We probably want to handle other packet types, as they would allow us to measure RSSI and just short circuit if low signal.
      if (updateChannels(packet, idx))
        checkTransition();
      // Since we've reached the end of the current frame, we need to reset our index in case another frame is available in the same loop...
      idx = 0;
    }

    if (idx >= MAX_PACKET_SIZE)
      idx = 0;
  }

  _tick++;
}

bool State::updateChannels(const uint8_t *const packet, uint8_t length)
{
  if (packet[2] != FRAME_TYPE_RC_CHANNEL_PACKED)
    return false;

  // Reset all channels to their zero.
  memset(_channels, 0, CHANNEL_COUNT * sizeof(uint16_t));

  // We're extracting 11 bits per channel...
  _channels[0] = ((packet[3] | packet[4] << 8) & ELEVEN_BIT_MASK);
  _channels[1] = ((packet[4] >> 3 | packet[5] << 5) & ELEVEN_BIT_MASK);
  _channels[2] = ((packet[5] >> 6 | packet[6] << 2 | packet[7] << 10) & ELEVEN_BIT_MASK);
  _channels[3] = ((packet[7] >> 1 | packet[8] << 7) & ELEVEN_BIT_MASK);
  _channels[4] = ((packet[8] >> 4 | packet[9] << 4) & ELEVEN_BIT_MASK);
  _channels[5] = ((packet[9] >> 7 | packet[10] << 1 | packet[11] << 9) & ELEVEN_BIT_MASK);
  _channels[6] = ((packet[11] >> 2 | packet[12] << 6) & ELEVEN_BIT_MASK);
  _channels[7] = ((packet[12] >> 5 | packet[13] << 3) & ELEVEN_BIT_MASK);
  _channels[7] = ((packet[12] >> 5 | packet[13] << 3) & ELEVEN_BIT_MASK);

  // _monitorStream->print("Channels: ");
  // for (int i = 0; i < CHANNEL_COUNT; i++)
  //   _monitorStream->printf("%2d: %4d ", i + 1, _channels[i]);
  // _monitorStream->println();

  return true;
}

bool State::checkTransition()
{
  switch (_phase)
  {
  case Phase::RequiresReset:
    _seqIdx = _lastEventTick = _tick = 0;
    for (uint8_t i = 0; i < _seqLen; i++)
      if (!unlatched(_seq[i]))
        return false;
    return transition(Phase::Sequencing);

  case Phase::Sequencing:
    // Check for channels "before" the current one in the sequence. None of them should have been let go of.
    if (_seqIdx > 0)
      for (uint8_t i = 0; i < _seqIdx; i++)
        if (!latched(_seq[i]))
          return transition(Phase::RequiresReset);
    // Check the channels "after" the current one in the sequence. None of them should have been depressed yet.
    if (_seqIdx <= _seqLen)
      for (uint8_t i = _seqIdx + 1; i < _seqLen; i++)
        if (!unlatched(_seq[i]))
          return transition(Phase::RequiresReset);
    // Ok - we're in a healthy state where none of the held latches were let go of, and none of the later
    // latches are touched. Let's see if our current latch has been depressed yet.
    return latched(_seq[_seqIdx])
               ? transition(_seqIdx < _seqLen - 1 ? Phase::Patience : Phase::Firing)
               : false;

  case Phase::Patience:
    // Check for channels "before" the current one in the sequence. None of them should have been let go of.
    // We're also checking the current latch as it should remain depressed too.
    if (_seqIdx > 0)
      for (uint8_t i = 0; i <= _seqIdx; i++)
        if (!latched(_seq[i]))
          return transition(Phase::RequiresReset);
    // Check the channels "after" the current one in the sequence. None of them should have been depressed yet.
    if (_seqIdx <= _seqLen)
      for (uint8_t i = _seqIdx + 1; i < _seqLen; i++)
        if (!unlatched(_seq[i]))
          return transition(Phase::RequiresReset);
    // Ok - we check our timer... If we've been patient enough, we can transition.
    if (patienceNeeded())
      return false;
    _seqIdx++;
    return transition(
        _seqIdx >= _seqLen
            ? Phase::Firing
            : Phase::Sequencing);

  case Phase::Firing:
    // We're firing for a bit, then back to a need to reset.
    return patienceNeeded()
               ? false
               : transition(Phase::RequiresReset);
  }
  return false;
}

bool State::transition(State::State::Phase phase)
{
  switch (phase)
  {
  case Phase::RequiresReset:
    _monitorStream->println("Requires Reset.");
    break;
  case Phase::Sequencing:
    if (_seqIdx == 0)
      _monitorStream->println("Ready!");
    else
      _monitorStream->printf("Defcon %d reached.\n", _seqIdx);
    break;
  case Phase::Patience:
    _monitorStream->printf("> ...");
    break;
  case Phase::Firing:
    _monitorStream->println("> FIRING!");
    break;
  };
  // If the phase isn't to change - we're done here.
  if (_phase == phase)
    return false;
  // Do the actual transition.
  _phase = phase;
  // Reset our timer.
  _lastEventTick = _tick = 0;
  return true;
}

bool State::patienceNeeded() { return _tick < _lastEventTick + PATIENCE; }
bool State::latched(uint8_t num) { return _channels[num] >= 1200; }
bool State::unlatched(uint8_t num) { return _channels[num] <= 300; }
