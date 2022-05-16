#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "rpi_gpio.hpp"

using namespace std;

static void select_pins(GPIOOut &m0, GPIOOut &m1, string &name, bool set);

int main(int argc, char **argv) {
  vector<string> args(argv, argv + argc);

  GPIOOut modeSelect0(21);
  GPIOOut modeSelect1(20);
  GPIOOut enablePin(12);

  select_pins(modeSelect0, modeSelect1, args.at(1), true);

  enablePin.set(true);
  getchar();

  enablePin.set(false);
  cout << "Done" << endl;
  getchar();
}

static void select_pins(GPIOOut &m0, GPIOOut &m1, string &name, bool set) {
  vector<string> modes = {
      "home",
      "extend",
      "retract",
      "hextend",
  };

  int idx = find(modes.begin(), modes.end(), name) - modes.begin();
  if (idx == modes.size()) {
    cerr << "Unknown mode " << name << endl;
    abort();
  }

  cout << modes[idx] << "ing" << endl;

  if (idx & 0b01)
    m0.set(set);
  if (idx & 0b10)
    m1.set(set);
}
