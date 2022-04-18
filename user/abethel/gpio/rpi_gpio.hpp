// Raspberry Pi GPIO I/O class, via the Raspberry Pi's sysfs.

// Based loosely on https://projects-raspberry.com
// /introduction-to-accessing-the-raspberry-pis-gpio-in-c-sysfs/

#pragma once

#include <string>

using namespace std;

enum class GPIODirection {
  Input,
  Output,
};

// A single GPIO pin.
class GPIOPin {
public:
  // Initialize communications with the numbered GPIO pin.
  GPIOPin(unsigned int num);

  // Disconnect from the GPIO pin.
  ~GPIOPin();

  // Set the direction of communication.
  void setdir_gpio(GPIODirection dir);

  // Set the value of the pin, which must be in output mode.
  void setval_gpio(bool val);

  // Get the value of the pin, which must be in input mode.
  bool getval_gpio();

private:
  // Exports the GPIO pin.
  void export_gpio();

  // Unexports the GPIO pin.
  void unexport_gpio();

  // GPIO pin number.
  unsigned int _num;
};

// A GPIO output pin.
class GPIOOut {
public:
  // Initialize communications with the numbered GPIO pin.
  GPIOOut(unsigned int num);

  // Set the value on the pin.
  void set(bool val);

private:
  // The underlying pin object, which should be in output mode.
  GPIOPin _pin;
};

// A GPIO input pin.
class GPIOIn {
public:
  // Initialize communications with the numbered GPIO pin.
  GPIOIn(unsigned int num);

  // Get the value from the pin.
  bool get();

private:
  // The underlying pin object, which should be in input mode.
  GPIOPin _pin;
};
