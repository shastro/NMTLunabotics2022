// cansockutils.h 

// Copyright (c) 2021 NMT Lunabotics.
// All rights reserved.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// Function to print a can_frame
// Takes in a point to a can_frame struct
//Prints 0x<id in hex> [<num bytes in data>] <bytes of data in hex separated by spaces>
int print_frame(struct can_frame *frame){

  printf("0x%03X [%d] ", frame->can_id, frame->can_dlc);

  for (int i = 0; i < frame->can_dlc; i++)
    printf("%02X ",frame->data[i]);

  printf("\r\n");

  return 0;
}

// Create a connection to a can interface and bind to it.
// Returns -1 on error or the cansock file descriptor
int establish_cansock(char *interface){
  int cansockfd;
  struct sockaddr_can addr;
  struct ifreq ifr;

  // Create a can socket (RAW variety) using socketcan
  if ((cansockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW))< 0){
    perror("Socket");
    return -1;
  }

  // Set up interface Request
  strcpy(ifr.ifr_name, "vcan0");
  // Request index number of interface
  // Places the index in the ifreq struct irf.ifindex
  if (ioctl(cansockfd, SIOCGIFINDEX, &ifr) < 0){
    perror("Ioctl");
    return -1;
  }

  // Setup addr struct
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  //Bind to the socket
  if (bind(cansockfd,(struct sockaddr*)&addr, sizeof(addr)) < 0){
    perror("Bind");
    return -1;
  }

  return cansockfd;

}
