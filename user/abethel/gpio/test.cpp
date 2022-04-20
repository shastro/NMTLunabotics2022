#include <iostream>
#include <unistd.h>

#include "rpi_gpio.hpp"

using namespace std;

int main(int argc, char **argv) {
  GPIOOut pin(21);
  for (int i = 0; i < 10; i++) {
    cout << "1" << endl;
    pin.set(true);
    sleep(1);

    cout << "0" << endl;
    pin.set(false);
    sleep(1);
  }
}
