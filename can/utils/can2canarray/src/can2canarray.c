// can2canarray

// Copyright (c) 2021 NMT Lunabotics.
// All rights reserved.

#include "cansockutils.h"


int main(void){
    int cansockfd; //can socket fd
    int nbytes = 0;
    struct can_frame frame;

    // Establish cansocket and bind. 
    cansockfd = establish_cansock("vcan0");

    while (true){
      nbytes = read(cansockfd, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Read");
            return 1;
        }
        print_frame(&frame);
    }
    if (close(cansockfd) < 0) {
    perror("Close");
    return 1;
    }

return 0;
}
