// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#include "8Bitdoh.hpp"
#include "joystick.hh"
#include <unistd.h>

int main(int argc, char **argv) {
  // Create an instance of Joystick
  Joystick joystick("/dev/input/js0");

  EightBitPro2Button button;
  // Ensure that it was found and that we can use it
  if (!joystick.isFound()) {
    printf("[ERROR] failed to open joystick.\n");
    exit(1);
  }

  while (true) {
    // Restrict rate
    usleep(1000);

    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event)) {
      if (event.isButton()) {
        button = static_cast<EightBitPro2Button>(event.number);
        switch (button) {
        case EightBitPro2Button::X {
          printf("Pressed X")
        }
        }
        if (button == EightBitPro2Button::X) {
          printf("FUCK\n");
        } else if (button == EightBitPro2Button::Y) {
          printf("SHIT\n");
        }
        printf("Button %u is %s\n", event.number,
               event.value == 0 ? "up" : "down");
      } else if (event.isAxis()) {
        printf("Axis %u is at position %d\n", event.number, event.value);
      }
    }
  }
}
