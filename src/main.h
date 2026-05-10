#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "iggy.hpp"

// The full arming sequence. Note that because a "spring loaded" stick is in this
// list, it is checked constantly to be near zero - so that it has to be held down
// the entire time until the very end of the sequence, where it then needs to be
// held fully up.
constexpr uint8_t seq[] = {
    4, // Arm switch 1.
    5, // Arm switch 2.
    6, // Arm switch 3.
    1  // Pitch stick axis.
};

// --------------------------------------------------- //
// ESP32 DevKit v1
#if defined(CONFIG_IDF_TARGET_ESP32)

#define CRSF_RX (GPIO_NUM_16) // CRSF serial RX.
#define CRSF_TX (GPIO_NUM_17) // CRSF serial TX.

// LED Pins config.
constexpr Iggy::IO io{
    powerLed : GPIO_NUM_25,
    feedbackLed : GPIO_NUM_33,
    relay : GPIO_NUM_32
};

// --------------------------------------------------- //
// ESP32 C3 Super Mini
#elif defined(CONFIG_IDF_TARGET_ESP32C3)

#define CRSF_RX (GPIO_NUM_5) // CRSF serial RX.
#define CRSF_TX (GPIO_NUM_6) // CRSF serial TX.

// LED Pins config.
constexpr Iggy::IO io{
    powerLed : GPIO_NUM_0,
    feedbackLed : GPIO_NUM_1,
    relay : GPIO_NUM_2
};

#endif

#endif // ifdef MAIN_H