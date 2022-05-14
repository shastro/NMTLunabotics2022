#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>
#define sleep(x) usleep(1000000 * (x))

#include "rpi_gpio.hpp"

using namespace std;

int main(int argc, char **argv) {
  vector<string> args(argv, argv + argc);

  GPIOOut home(21);
  GPIOOut extend(20);
  GPIOOut retract(12);
  GPIOOut half_extend(16);

  if (args.at(1) == "home") {
    cout << "Homing" << endl;
    home.set(true);
    getchar();
    home.set(false);
  } else if (args.at(1) == "extend") {
    cout << "Extending" << endl;
    extend.set(true);
    getchar();
    extend.set(false);
  } else if (args.at(1) == "retract") {
    cout << "Retracting" << endl;
    retract.set(true);
    getchar();
    retract.set(false);
  } else if (args.at(1) == "hextend") {
    cout << "Half-extending" << endl;
    half_extend.set(true);
    getchar();
    half_extend.set(false);
  } else {
    cout << "Unknown command " << args.at(1) << endl;
    return 0;
  }

  cout << "Done" << endl;

  getchar();
}
