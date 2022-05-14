// Raspberry Pi GPIO I/O class, via the Raspberry Pi's sysfs.

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

#include "rpi_gpio.hpp"

using namespace std;

// Delay to wait for Linux filesystem race conditions, in
// microseconds.
#define DUMB_RACE_CONDITION_DELAY 0.05 * 1000000

// Initialize communications with the numbered GPIO pin.
GPIOPin::GPIOPin(unsigned int num) {
  _num = num;
  export_gpio();
}

// Move the GPIOPin object.
GPIOPin::GPIOPin(GPIOPin &&source) {
  _num = source._num;
  source._num = -1;
}

  // Disconnect from the GPIO pin.
GPIOPin::~GPIOPin() {
  if (_num != -1)
    unexport_gpio();
}

// Set the direction of communication.
void GPIOPin::setdir_gpio(GPIODirection dir) {
  {
    stringstream filename;
    filename << "/sys/class/gpio/gpio" << _num << "/direction";
    ofstream file(filename.str());
    file.exceptions(ofstream::failbit | ofstream::badbit);

    switch (dir) {
    case GPIODirection::Input:
      file << "in";
    case GPIODirection::Output:
      file << "out";
    }
  }

  // After telling the kernel to set the pin's direction, it takes a
  // moment to actually do the change. Take some extra time here so
  // that later code can rely on the pin actually being in the right
  // direction after this function returns.
  usleep(DUMB_RACE_CONDITION_DELAY);
}

// Set the value of the pin, which must be in output mode.
void GPIOPin::setval_gpio(bool val) {
  stringstream filename;
  filename << "/sys/class/gpio/gpio" << _num << "/value";
  ofstream file(filename.str());
  file.exceptions(ofstream::failbit | ofstream::badbit);

  file << (val ? "1" : "0");
}

// Get the value of the pin.
bool GPIOPin::getval_gpio() {
  stringstream filename;
  filename << "/sys/class/gpio/gpio" << _num << "/value";
  ifstream file(filename.str());
  file.exceptions(ifstream::failbit | ifstream::badbit);

  string val;
  file >> val;

  return val != "0";
}

// Enables software control of the GPIO pin.
void GPIOPin::export_gpio() {
  {
    string filename = "/sys/class/gpio/export";
    ofstream file(filename);
    file.exceptions(ofstream::failbit | ofstream::badbit);

    file << _num;
  }

  // After telling the kernel to export the pin, it takes a moment to
  // actually do the change.
  usleep(DUMB_RACE_CONDITION_DELAY);
}

// Unexports the GPIO pin.
void GPIOPin::unexport_gpio() {
  {
    string filename = "/sys/class/gpio/unexport";
    ofstream file(filename);
    file.exceptions(ofstream::failbit | ofstream::badbit);

    file << _num;
  }
  usleep(DUMB_RACE_CONDITION_DELAY);
}

// Initialize communications with the numbered GPIO pin.
GPIOOut::GPIOOut(unsigned int num) : _pin(num) {
  _pin.setdir_gpio(GPIODirection::Output);
}

// Set the value on the pin.
void GPIOOut::set(bool val) { _pin.setval_gpio(val); }

// Initialize communications with the numbered GPIO pin.
GPIOIn::GPIOIn(unsigned int num) : _pin(num) {
  _pin.setdir_gpio(GPIODirection::Input);
}

// Get the value from the pin.
bool GPIOIn::get() { return _pin.getval_gpio(); }
