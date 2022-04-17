// Raspberry Pi GPIO I/O class, via the Raspberry Pi's sysfs.

// Based loosely on https://projects-raspberry.com
// /introduction-to-accessing-the-raspberry-pis-gpio-in-c-sysfs/

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "rpi_gpio.hpp"

using namespace std;

// Initialize communications with the numbered GPIO pin.
GPIOPin::GPIOPin(unsigned int num) {
  _num = num;
  export_gpio();
}

// Disconnect from the GPIO pin.
GPIOPin::~GPIOPin() {
  unexport_gpio();
}


// Set the direction of communication.
void GPIOPin::setdir_gpio(GPIODirection dir) {
  stringstream filename;
  filename << "/sys/class/gpio/gpio" << _num << "/direction";
  ofstream file(filename.str());

  switch (dir) {
  case GPIODirection::Input:
    file << "in";
  case GPIODirection::Output:
    file << "out";
  }
}

// Set the value of the pin, which must be in output mode.
void GPIOPin::setval_gpio(bool val) {
  stringstream filename;
  filename << "/sys/class/gpio/gpio" << _num << "/value";
  ofstream file(filename.str());

  file << (val ? "1" : "0");
}

// Get the value of the pin.
bool GPIOPin::getval_gpio() {
  stringstream filename;
  filename << "/sys/class/gpio/gpio" << _num << "/value";
  ifstream file(filename.str());

  string val;
  file >> val;

  return val != "0";
}

// Set the direction of communication.
void GPIOPin::export_gpio() {
  string filename = "/sys/class/gpio/export";
  ofstream file(filename);

  file << _num;
}

// Unexports the GPIO pin.
void GPIOPin::unexport_gpio() {
  string filename = "/sys/class/gpio/unexport";
  ofstream file(filename);

  file << _num;
}
