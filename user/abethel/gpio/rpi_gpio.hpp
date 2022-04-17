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

  unsigned int _num;
  // GPIO number associated with the instance of an object
};
