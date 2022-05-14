// Raspberry Pi GPIO I/O class, via the Raspberry Pi's sysfs.

// Based loosely on https://projects-raspberry.com
// /introduction-to-accessing-the-raspberry-pis-gpio-in-c-sysfs/

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

#include "rpi_gpio.hpp"

using namespace std;

// Initialize communications with the numbered GPIO pin.
GPIOPin::GPIOPin(unsigned int num) {
  _num = num;
  export_gpio();
}

// Disconnect from the GPIO pin.
GPIOPin::~GPIOPin() { unexport_gpio(); }

// Set the direction of communication.
void GPIOPin::setdir_gpio(GPIODirection dir) {
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

  usleep(500000);
  cout << "dir ok" << endl;
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
  string filename = "/sys/class/gpio/export";
  ofstream file(filename);
  file.exceptions(ofstream::failbit | ofstream::badbit);

  file << _num;
  usleep(500000);
  cout << "export ok" << endl;
}

// Unexports the GPIO pin.
void GPIOPin::unexport_gpio() {
  string filename = "/sys/class/gpio/unexport";
  ofstream file(filename);
  file.exceptions(ofstream::failbit | ofstream::badbit);

  file << _num;
  usleep(500000);
  cout << "unexport ok" << endl;
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
