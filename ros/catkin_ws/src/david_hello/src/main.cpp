#include <iostream>
#include <unistd.h>

using namespace std;

int main(int argc, char **argv) {
  for (int i = 0;; i++) {
    cout << "hello " << i << endl;
    sleep(1);
  }
}
