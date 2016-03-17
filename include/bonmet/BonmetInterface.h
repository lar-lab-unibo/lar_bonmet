/*
 * File:   UDPNode.h
 * Author: daniele
 *
 * Created on 4 novembre 2015, 12.08
 */

#ifndef BONMETINTERFACE_H
#define BONMETINTERFACE_H


#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <string>
#include <vector>
#include "BonmetCommons.h"

#define BONMETINTERFACE_HEADER_LEN 6
#define BONMETINTERFACE_MANAGE_CMD_SWITCH_ON 1
#define BONMETINTERFACE_MANAGE_CMD_SWITCH_OFF 2
#define BONMETINTERFACE_MANAGE_CMD_RESET_ERROR 3


#define BONMETINTERFACE_JOG_CMD_ENABLE_STOP 0
#define BONMETINTERFACE_JOG_CMD_ENABLE_POSITIVE 1
#define BONMETINTERFACE_JOG_CMD_ENABLE_NEGATIVE 2

// CMD list
#define BONMETINTERFACE_CMD_GET_SYS_CONFIG 0x01               // DONE
#define BONMETINTERFACE_CMD_GET_AXIS_CONFIG 0x02              // DONE
#define BONMETINTERFACE_CMD_GET_AXIS_RT_DATA 0x03             // DONE
#define BONMETINTERFACE_CMD_GET_GROUP_CONFIG 0x04             // NOT DONE
#define BONMETINTERFACE_CMD_GET_GROUP_RT_DATA 0x05            // NOT DONE
#define BONMETINTERFACE_CMD_SET_AUTORITY 0x06                 // DONE
#define BONMETINTERFACE_CMD_GET_AUTORITY 0x07                 // NOT DONE
#define BONMETINTERFACE_CMD_AXIS_MANAGE_COMMAND_SEND 0x40     // DONE  (Non testata!)
#define BONMETINTERFACE_CMD_AXIS_JOG_COMMAND 0x41             // DONE
#define BONMETINTERFACE_CMD_AXIS_HOME_COMMAND 0x42            // DONE
#define BONMETINTERFACE_CMD_GROUP_MANAGE_COMMAND_SEND 0x50    // NOT DONE
#define BONMETINTERFACE_CMD_GROUP_JOG_COMMAND 0x51            // NOT DONE


/**
*   JOINTS LIMITS (If HOMING procedure changes these values may need to be change accordingly)
*/

#define BONMETINTERFACE_LOWER_LIMIT_J_0 -110
#define BONMETINTERFACE_UPPER_LIMIT_J_0 +130
#define BONMETINTERFACE_LOWER_LIMIT_J_1 -25
#define BONMETINTERFACE_LOWER_LIMIT_J_2 -45
#define BONMETINTERFACE_LOWER_LIMIT_J_4 -95


namespace lar_bonmet {


struct PIDController{
  FLOAT32 P = 1;
  FLOAT32 D = 0;
  FLOAT32 I = 0;
  FLOAT32 getActuation(FLOAT32 error,FLOAT32 velocity,FLOAT32 integral){
    return  P*error + D*velocity + I*integral;
  }
};


/**
 * TCP Client for Motion Controller
 */
struct MotionControllerServer {
        int sock;
        struct sockaddr_in address;

        MotionControllerServer(std::string host,int port){
                //Socket init
                if ((sock = socket(AF_INET, SOCK_STREAM,0)) < 0) {
                        printf("Error Creating socket\n");
                }
                //Address creation
                address.sin_family = AF_INET;
                address.sin_addr.s_addr = inet_addr(host.c_str());
                address.sin_port = htons(port);

                //Connection
                if (connect(sock,(struct sockaddr*) &address, sizeof(address))<0) {
                        printf("Connection failed\n");
                }
        }

        bool receive(byte*& data, int data_length) {
            if (recv(this->sock, data, data_length, 0)) {
                return false;
            }
            return true;
        }

        bool send(byte*& data, int data_length) {
            return sendto(this->sock, data, data_length, 0, (struct sockaddr *) & address, sizeof (address));

        }
};


/**
 * BonmetInterface for Motion Controller
 */
class BonmetInterface {

public:
        BonmetInterface(std::string host,int port);
        ~BonmetInterface();
        FLOAT32* get_joint_state();
        int Get_Sys_Config();
        int Set_Autority(int);
        void AxisJogCmd(int, int, UINT16, int, int);
        void AxisManageCmdSend(int, int);
        void GoToAngle(int, FLOAT32);
        bool isConnected();
        void setPID(int axis, FLOAT32 p, FLOAT32 d, FLOAT32 i);
        void GoToConfiguration(FLOAT32, FLOAT32, FLOAT32, FLOAT32, FLOAT32, FLOAT32);
        void GoToPosition(FLOAT32, FLOAT32, FLOAT32);
        void AxisHomeCommand(int, int, int, FLOAT32, UINT16, UINT16, UINT16);
        void GetAxisRTData();
private:
        void createMessageHeader(byte*, int, int);
        byte* send_receive_pkg(byte*, int);
        bool XOR(bool, bool);


        byte* token;
        std::string host;
        int port;
        MotionControllerServer* server;
        std::vector<PIDController> controllers;

};

}
#endif  /* BONMETINTERFACE_H */
