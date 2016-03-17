/*
 * File:   UDPNode.h
 * Author: daniele
 *
 * Created on 4 novembre 2015, 12.08
 */

#ifndef BONMETCOMMONS_H
#define BONMETCOMMONS_H

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <string>

namespace lar_bonmet {

typedef unsigned char byte;
typedef unsigned short UINT16;
typedef unsigned int UINT32;
typedef float FLOAT32;


/**
 * Print message on screen
 */
void printMessage(byte* message,int len){
        printf("Message:\n");
        for(int i = 0; i < len; i++) {
                printf("- Data %d: %d\n",i,message[i]);
        }
}

/**
 * Converts 2 byte into short(int16) value. Input is a byte array and start index
 */
UINT16 bytesToINT16(byte* data,int start_index){
        return (data[start_index] << 8) | data[start_index+1];
}

/**
 * Converts 4 byte into short(int16) value. Input is a byte array and start index
 */
UINT32 bytesToINT32(byte* data,int start_index){
        return
                (data[start_index] << 24) |
                (data[start_index+1] << 16) |
                (data[start_index+2] << 8) |
                data[start_index+3];
}

/**
 * Converts 4 byte into float32 value. Input is a byte array and start index
 */
FLOAT32 bytesToFLOAT32(byte* data,int start_index){
        UINT32 dw = bytesToINT32(data,start_index);
        FLOAT32 f;
        memcpy(&f,&dw,4);
}

/**
 * Converts short(int16) to bytes writing on a buffer
 */
void UINT16toBytes(UINT16 value,byte* data,int start_index){
        data[start_index] = value >> 8;
        data[start_index+1] = value ;
}

void FLOAT32toBytes(UINT16 value,byte* data,int start_index){
      printf("ATTENZIONE: FUNZIONE FLOAT32toBytes NON VERIFICATA!\n");
      usleep(10000);
      data[start_index] = value >> 24;
      data[start_index+1] = value >> 16;
      data[start_index+2] = value >> 8;
      data[start_index+3] = value ;

}



}


#endif  /* BONMETCOMMONS_H */
