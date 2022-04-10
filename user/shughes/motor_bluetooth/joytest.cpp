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
  Joystick joystick("/dev/input/js0", true);

  // Create joystick enums
  Pro2Button button;
  Pro2Axis axis;

  // Ensure that it was found and that we can use it
  if (!joystick.isFound()) {
    printf("[ERROR] failed to open joystick.\n");
    exit(1);
  }

  while (true) {
    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event)) {
      if (event.isButton()) {
        button = static_cast<Pro2Button>(event.number);

        switch (button) {

        case Pro2Button::X:
          printf("Pressed X");
          printf("Value %d \n", event.value);

        case Pro2Button::Y:
          printf("Pressed Y");
          printf("Value %d \n", event.value);

        default:
          printf("Pressed %d\n", event.number);
        }
      } else if (event.isAxis()) {
        axis = static_cast<Pro2Axis>(event.number);

        switch (axis) {
        case Pro2Axis::rightTrigger:
          if ( event.value > -30000 ) {
              printf("Pressed rightTrigger %d\n", event.value);
          }
        }
      }
    }
  }
}
