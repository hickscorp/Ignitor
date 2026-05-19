#ifndef IGGY_H
#define IGGY_H

#include <Stream.h>
#include <jled.h>

// Crossfire related constants - should work out of the box. Change if you know what you're doing!
#define CHANNEL_COUNT (8)                  // We assume 8 channels are being sent.
#define ELEVEN_BIT_MASK (0x07FF)           // Masks 11 bits to keep and the rest to ignore.
#define MAX_PACKET_SIZE (64)               // See file header.
#define RC_CHANNELS_PACKED_FRAME_ID (0x16) // Marks that a packet we got is for channels as opposed to telemetry.
#define PATIENCE (30000)                   // How long to wait before a push "registers".

namespace Iggy
{
  enum class Phase
  {
    RequiresReset = 0, // Requires clean "zero" from all sequence channels.
    Sequencing,        // Awaiting for "next channel high".
    Patience,          // A sequence part has completed, wait for a bit...
    Firing             // Full sequence has completed - send voltage out to relay.
  };

  struct IO
  {
    uint8_t powerLed, // Power indicator pin.
        feedbackLed,  // Feedback indicator pin.
        relay;        // Relay pin.
  };

  class State
  {
  public:
    // Constructor.
    State( // Sequence channels configuration.
        uint8_t seqLen, const uint8_t *const seq,
        const IO io,
        // Streams...
        Stream *crsfStream)
        : _phase{Phase::RequiresReset}, // Phase of the FSM we're in.
          _powerLed{io.powerLed},       // Power indicator LED output pin.
          _feedbackLed{io.feedbackLed}, // Feedback indicator LED output pin.
          _relay{io.relay},             // Ignition relay pin.
          _seqLen{seqLen},              // The length of the ignition sequence.
          _seq{seq},                    // The ignition sequence.
          _crsfStream{crsfStream}       // The CrossFire stream to read channels from.
    {
      _powerLed.Off();
      _feedbackLed.Off();
      _relay.Off();
    }
    // Update loop.
    void loop()
    {
      _powerLed.Update();
      _feedbackLed.Update();
      _relay.Update();

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
          // In that case, update the channels, and if it is successful, check for transition change.
          if (packet[2] == RC_CHANNELS_PACKED_FRAME_ID)
          {
            // Read channel data starting at the 4th element of the buffer.
            updateChannels(&packet[3]);
            // Based on the channel data, check whether or not a transition should occur.
            checkTransition();
          }
          // Since we've reached the end of the current frame, we need to reset our index in case another frame
          // is available in the same iteration...
          memset(packet, 0, sizeof(uint8_t) * MAX_PACKET_SIZE);
        }

        if (idx >= MAX_PACKET_SIZE)
        {
          idx = 0;
          memset(packet, 0, sizeof(uint8_t) * MAX_PACKET_SIZE);
        }
      }
      // Increment our tick.
      _tick++;
    }

  private:
    const uint8_t *_seq;                 // The ignition sequence.
    const uint8_t _seqLen;               // The length of the ignition sequence.
    int8_t _seqIdx;                      // Where we are in the ignition sequence.
    JLed _powerLed;                      // Power indicator.
    JLed _feedbackLed;                   // Feedback indicator.
    JLed _relay;                         // Technically not an LED - but suitable for a relay too.
    Phase _phase = Phase::RequiresReset; // The FSM phase we're in.
    uint16_t _channels[CHANNEL_COUNT];   // Channels data.
    Stream *_crsfStream = NULL;          // Stream to read receiver data from.
    uint32_t _tick = 0;                  // Current tick.
    uint32_t _lastEventTick = 0;         // The tick at which something happened last.

    /**
     * Reads from the CRSF stream and updates the internal channel buffer with the received values.
     *
     * Internally, this function is the base of the CRSF decoding. It handles 11-bit per channel packed data and shifts
     * it around to get one clean uint16 per channel.
     *
     * Note that this function does **not** verify CRC!
     *
     * @param packet A pointer to an uint8 buffer - typically pointing to the 4th element of a CRSF channel update packet.
     */
    inline void updateChannels(const uint8_t *const packet)
    {
      // Ok - the channels data starts at `packet[3]`. We're extracting 11 bits per channel...
      _channels[0] = ((packet[0] | packet[1] << 8) & ELEVEN_BIT_MASK);
      _channels[1] = ((packet[1] >> 3 | packet[2] << 5) & ELEVEN_BIT_MASK);
      _channels[2] = ((packet[2] >> 6 | packet[3] << 2 | packet[4] << 10) & ELEVEN_BIT_MASK);
      _channels[3] = ((packet[4] >> 1 | packet[5] << 7) & ELEVEN_BIT_MASK);
      _channels[4] = ((packet[5] >> 4 | packet[6] << 4) & ELEVEN_BIT_MASK);
      _channels[5] = ((packet[6] >> 7 | packet[7] << 1 | packet[8] << 9) & ELEVEN_BIT_MASK);
      _channels[6] = ((packet[8] >> 2 | packet[9] << 6) & ELEVEN_BIT_MASK);
      _channels[7] = ((packet[9] >> 5 | packet[10] << 3) & ELEVEN_BIT_MASK);
    }

    /**
     * Checks inputs and current phase and verifies whether a transition should be requested.
     *
     * @return A boolean indicating whether or not a phase transition has occured.
     */
    inline bool checkTransition()
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
        // We are firing for as long as the state is held.
        if (_seqIdx > 0)
          for (uint8_t i = 0; i < _seqLen; i++)
            if (!latched(_seq[i]))
              return transition(Phase::RequiresReset);
        return true;
      }
      return false;
    }

    /**
     * Given a target phase, updates the outputs if an actual transition should occur or does
     * nothing if the current phase is already the given target one.
     *
     * @param phase The target `Iggi::Phase` to try and reach.
     * @return A boolean indicaticating whether or not a transition occured.
     */
    inline bool transition(Iggy::Phase phase)
    {
      // If the phase isn't to change - we're done here.
      if (_phase == phase)
        return false;

      // Blinking frequency - function of the current sequence index. Blinks faster as we get closer
      // to the ignition phase!
      uint16_t stressFrq = 1000 / ((_seqIdx + 1) * 5);

      switch (phase)
      {
      case Phase::RequiresReset:
        _powerLed.Off().Forever();    // Power indicator OFF.
        _feedbackLed.Off().Forever(); // Feedback indicator OFF.
        _relay.Off().Forever();       // Relay OFF.
        break;
      case Phase::Sequencing:
        if (_phase != Phase::Patience)
          _powerLed.Breathe(1200, 200, 300).Forever(); // Power breathing.
        if (_seqIdx == _seqLen - 1)
          _feedbackLed.On().Forever(); // Feedback blinking.
        else
          _feedbackLed.Blink(stressFrq, stressFrq).Forever(); // Feedback blinking.
        _relay.Off().Forever();                               // Relay OFF.
        break;
      case Phase::Patience:
        _feedbackLed.On().Forever(); // Feedback ON.
        break;
      case Phase::Firing:
        _powerLed.FadeOff(350).Repeat(1);  // Keep power LED OFF.
        _feedbackLed.On();                 // Keep feedback LED ON.
        _relay.Blink(250, 250).Repeat(10); // Toggle relay repeatedly.
        break;
      };

      _phase = phase;             // Do the actual transition.
      _lastEventTick = _tick = 0; // Reset our timer.
      return true;
    }

    /**
     * Tells whether or not enough time has elapsed since the last recorded action.
     *
     * @return A boolean indicating whether or not we've waited enough.
     */
    inline bool patienceNeeded() { return _tick < _lastEventTick + PATIENCE; }

    /**
     * Tells whether a given channel is fully latched.
     *
     * @param channel The zero-based index of the channel to check.
     * @return A boolean indicating whether or not the channel is fully latched.
     */
    inline bool latched(uint8_t channel) { return _channels[channel] >= 1200; }

    /**
     * Tells whether a given channel is fully unlatched.
     *
     * @param channel The zero-based index of the channel to check.
     * @return A boolean indicating whether or not the channel is fully unlatched.
     */
    inline bool unlatched(uint8_t chan) { return _channels[chan] <= 300; }
  };
}

#endif // IGGY_H
