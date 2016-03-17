/*
 * File:   UDPNode.h
 * Author: daniele
 *
 * Created on 4 novembre 2015, 12.08
 */

#include "BonmetInterface.h"

namespace lar_bonmet {

BonmetInterface::BonmetInterface(std::string host, int port){
        this->host = host;
        this->port = port;
        this->server = new  MotionControllerServer(host,port);
        this->controllers.resize(6);
        printf("BONMET INTERFACE CREATED: %s:%d!!\n",host.c_str(),port);

}

BonmetInterface::~BonmetInterface(){
}

bool BonmetInterface::isConnected()
{
        return true;
}

bool BonmetInterface::XOR(bool a, bool b)
{
        return (a + b) % 2;
}

void BonmetInterface::createMessageHeader(byte* message, int command_type, int data_len)
{
        //MAGIC
        message[0] = 0x41;
        message[1] = 0x54;

        //COMMAND TYPE
        message[2] = 0x00;
        message[3] = command_type;

        //DATA LEN
        message[4] = data_len & 0xFF00;
        message[5] = data_len & 0x00FF;
}


void BonmetInterface::setPID(int axis, FLOAT32 p, FLOAT32 i, FLOAT32 d){
        this->controllers[axis].P = p;
        this->controllers[axis].I = i;
        this->controllers[axis].D = d;

}
void GoToPosition(FLOAT32 x, FLOAT32 y, FLOAT32 z){

/*
  function q = ik (K)
  Anthropomorphic arm with 6 DOF and spherical wrist
  It calculates the Inverse Kinematic of an Anthropomorphic arm with 6 DOF.
  'q' is the solutions in radiant and K is the direct Kinematic matrix.
*/

/* EXAMPLE of K matrix
                   K = [ n s a p;
                       0 0 0 1]
    where n, s, a are three vectors fo 3 elements that represents the
    end-effector's orientation, and p is the desired end-effector position.
*/

/*
  // Denavit-Hartenberg's Parameters
  a1=0;           // [m]
  a2=0.2;         // [m]
  a3=0.2;         // [m]
  d6=0.1;         // [m]
  alfa1=pi/2;     // [rad]
  alfa2=0;        // [rad]
  alfa3=0;        // [rad]

  % dk=[n s a p; 0 0 0 1]
  % n, s, a: They are 3 vector for the end-effector's orientation
  dk=K;          % Position and orientation of end-effector
  n=dk(1:3,1);
  s=dk(1:3,2);
  a=dk(1:3,3);
  R=[n s a];
  dk=K;          % Direct kinematics matrix

  % Inverse Kinematic
  p_ot=dk(1:3,4); % End-effector's position
  pw=p_ot-d6*a;   % Wrist's position
  pw_x=dk(1,4);   % Vector's components that representes the wrist's position
  pw_y=dk(2,4);
  pw_z=dk(3,4);

  c3=(pw_x^2+pw_y^2+pw_z^2-a2^2-a3^2)/(2*a2*a3);  % cos(teta3)
  s3=-sqrt(1-c3^2);        % sin(teta3)
  teta3=atan2(s3,c3);

  c2=(sqrt(pw_x^2+pw_y^2)*(a2+a3*c3)+pw_z*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);      % cos(teta2)
  s2=(pw_z*(a2+a3*c3)-sqrt(pw_x^2+pw_y^2)*a3*s3)/(a2^2+a3^2+2*a2*a3*c3);      % sin(teta2)
  teta2=atan2((a2+a3*c3)*pw_z-a3*s3*sqrt(pw_x^2+pw_y^2),(a2+a3*c3)*sqrt(pw_x^2+pw_y^2)+a3*s3*pw_z);

  teta1=atan2(pw_y,pw_x);

  R3_0=[cos(teta1)*cos(teta2+teta3) -cos(teta1)*sin(teta2+teta3) sin(teta1);      % Matrix for the Euler's angle of 3dof arm
      sin(teta1)*cos(teta2+teta3) -sin(teta1)*sin(teta2+teta3) -cos(teta1);
      sin(teta2+teta3) cos(teta2+teta3) 0];

  R6_3=R3_0'*R;        % Matrix for the Euler's angle of spherical wrist

  % Inverse kinematic for the spherical wrist
  teta4=atan2(R6_3(2,3),R6_3(1,3));
  teta5=atan2(sqrt((R6_3(1,3))^2+(R6_3(2,3))^2),R6_3(3,3));
  teta6=atan2(R6_3(3,2),R6_3(3,1));

  q=[teta1 teta2 teta3 teta4 teta5 teta6]';      % Solutions in radiant

*/
}


void BonmetInterface::GoToConfiguration(FLOAT32 target_angle_0, FLOAT32 target_angle_1, FLOAT32 target_angle_2, FLOAT32 target_angle_3, FLOAT32 target_angle_4, FLOAT32 target_angle_5){
    GoToAngle(0, target_angle_0);
    GoToAngle(1, target_angle_1);
    GoToAngle(2, target_angle_2);
    GoToAngle(3, target_angle_3);
    GoToAngle(4, target_angle_4);
    GoToAngle(5, target_angle_5);
}


void BonmetInterface::GoToAngle(int target_axis, FLOAT32 target_angle)
{

        /**
         *   JOINTS LIMITS SAFETY CHECK
         */

        switch (target_axis) {

        case 0:
                if (target_angle < BONMETINTERFACE_LOWER_LIMIT_J_0) {
                        target_angle = BONMETINTERFACE_LOWER_LIMIT_J_0;
                }

                if (target_angle > BONMETINTERFACE_UPPER_LIMIT_J_0) {
                        target_angle = BONMETINTERFACE_UPPER_LIMIT_J_0;
                }
                break;
        case 1:
                if (target_angle < BONMETINTERFACE_LOWER_LIMIT_J_1) {
                        target_angle = BONMETINTERFACE_LOWER_LIMIT_J_1;
                }
                break;
        case 2:
                if (target_angle < BONMETINTERFACE_LOWER_LIMIT_J_2) {
                        target_angle = BONMETINTERFACE_LOWER_LIMIT_J_2;
                }
                break;
        case 4:
                if (target_angle < BONMETINTERFACE_LOWER_LIMIT_J_4) {
                        target_angle = BONMETINTERFACE_LOWER_LIMIT_J_4;
                }
                break;

        default: break;
        }


        FLOAT32* pos = get_joint_state();
        FLOAT32 axis_angle = pos[target_axis*4];
        FLOAT32 error = target_angle - axis_angle;
        //printf("axis_angle: %f\n", axis_angle);
        //printf("target_angle: %f\n", target_angle);
        //usleep(10000000);
        FLOAT32 velocity = -pos[target_axis*4+1];
        FLOAT32 integral = 0;

        PIDController controller = this->controllers[target_axis];

        if (fabs(error) > 0.0002 ) {
                int PID = controller.getActuation(error,velocity,integral);
                //PID = PID > 500 ? 500 : PID;
                if ( PID > 0) {
                        AxisJogCmd(target_axis, BONMETINTERFACE_JOG_CMD_ENABLE_POSITIVE, abs(PID), 0, 0);
                }
                else if ( PID < 0) {
                        AxisJogCmd(target_axis, BONMETINTERFACE_JOG_CMD_ENABLE_NEGATIVE, abs(PID), 0, 0);
                }
        }else{
                AxisJogCmd(target_axis, BONMETINTERFACE_JOG_CMD_ENABLE_STOP, 0, 0, 0);
        }


        /*

           //  printf("axis_angle: %f\n", axis_angle);
           //  printf("Error: %f\n", error);
           //AxisManageCmdSend(target_axis, 1);
           int i = 0;

           if (abs(error*1000) > 2 )
           {
           if (fabs(error) > 10)
           {
                PID = (150+i)*fabs(error)/error;
                i = (i+1) % 2;
           }
           else
           {
                PID =controller.getActuation(error,velocity,integral);
           }


           if (PID > 200)
           {PID = 200; }

           if ( PID > 0)
                AxisJogCmd(target_axis, BONMETINTERFACE_JOG_CMD_ENABLE_POSITIVE, fabs(PID), 5, 5);
           else if ( PID < 0)
                AxisJogCmd(target_axis, BONMETINTERFACE_JOG_CMD_ENABLE_NEGATIVE, fabs(PID), 5, 5);

           pos = get_joint_state();
           axis_angle = pos[target_axis*4];
           error = target_angle - axis_angle;
           velocity = -pos[target_axis*4+1];

           if (abs(error) < 1 )
           { integral = integral + error; }
           else integral = 0;


           //      printf("AXIS: %d. Target Angle: %f. Actual Angle: %f\n", target_axis, target_angle, axis_angle);
           //      printf("error: %f\n", P*error);
           //      printf("velocity: %f\n", D*velocity);
           //      printf("integral: %f\n", I*integral);
           //      printf("PID: %d\n\n", PID);
           //      printf("F_PID: %f\n\n", fabs(PID));


           }else{
           //      printf("angle: %f\n", axis_angle);
           //      printf("Error: %f\n\n\n\n\n\n\n\n\n\n", error);
           AxisJogCmd(target_axis, BONMETINTERFACE_JOG_CMD_ENABLE_STOP, 0, 0, 0);

           //AxisManageCmdSend(target_axis, BONMETINTERFACE_MANAGE_CMD_SWITCH_OFF);
           }
           //  usleep(10000000);
         */

}

void BonmetInterface::AxisHomeCommand(int target_axis, int HC, int HM, FLOAT32 home_position, UINT16 home_speed_1, UINT16 home_speed_2, UINT16 home_accel)
{
  printf("\nAXIS_HOME_COMMAND never tested!\n");
  usleep(1000000);

  /*
  *Encoding for "HC" (homing command) and "HM" (homing method):
  HC = 0 (RES): reset/stop previous homing commands (HM ignored); if previous status
      was HOMING_DONE, homing status will be switched back to HOMING_OFF
  HC = 1 (COE): start a servodrive-side managed homing with method specified by HM
  HC = 2 (MAH): start a master-side managed homing procedure specified by HM
  other values of HT: reserved
  */

  int data_len = 16;
  byte* message = new byte[BONMETINTERFACE_HEADER_LEN + data_len];
  createMessageHeader(message,BONMETINTERFACE_CMD_AXIS_HOME_COMMAND,data_len);
  int index=BONMETINTERFACE_HEADER_LEN;
  message[index++] = target_axis; //axis_number
  message[index++] = HC; //homing command
  message[index++] = HM; //homing Method
  message[index++] = 0; // not used
  FLOAT32toBytes(home_position, message, index);
  index = index+4;
  UINT16toBytes(home_speed_1, message, index); // Thousand-th of rated
  index = index+2;
  UINT16toBytes(home_speed_2, message, index); // Thousand-th of rated
  index = index+2;
  UINT16toBytes(home_accel, message, index); // As decimal of second
  index = index+2;
  message[index++] = 0; // not used
  message[index++] = 0; // not used

  byte* response_payload = send_receive_pkg(message, 22);
  int axis_number = response_payload[0];
  int result = response_payload[1];
  printf("Axis Home Command: ");
  switch(result)
  {
  case (1):
          printf("Axis Number Error. \n");
          break;
  case (2):
          printf("Command Error.\n");
          break;
  case (3):
          printf("Error Scheduling Command.\n");
          break;
  case (4):
          printf("Not Allowed. (Authorit Error).\n");
          break;
  case (5):
          printf("Axis Not Powered.\n");
          break;
  case (6):
          printf("Position out of Limits.\n");
          break;
  case (10):
          printf("Command Scheduled Correctly.\n");
          break;

  default: break;
  }
  printf("Axis Number: %d\n", axis_number);
}


int BonmetInterface::Set_Autority(int Component_ID)
{
        Get_Sys_Config();

        byte* message = new byte[38];

        char pwd[16];
        for(int i = 0; i < 16; i++)
                pwd[i] = 0;
        switch(Component_ID)
        {
        case (1):
                pwd[0] = 'P';
                pwd[1] = 'L';
                pwd[2] = 'C';
                break;
        case (2):
                pwd[0] = 'G';
                pwd[1] = 'U';
                pwd[2] = 'I';
                break;
        }

        int msg_data_len = 32;

        createMessageHeader(message, BONMETINTERFACE_CMD_SET_AUTORITY,msg_data_len);
        int index=6;
        message[index++] = Component_ID; //1 = PLC, 2 = GUI

        for (int i = 0; i < 16; i+=2)
        {
                message[i+7] = this->token[0] ^ pwd[0+i];
                message[i+8] = this->token[1] ^ pwd[1+i];
        }


        for(int i = 23; i < 38; i++)
                message[i] = 0;

        //printMessage(message,msg_data_len);
        byte* response_payload = send_receive_pkg(message, msg_data_len+6);
        printf("Set Autority Command: ");
        switch(response_payload[0])
        {
        case (1):
                printf("Autority Successfully SET to %c%c%c.\n", pwd[0], pwd[1], pwd[2]);
                break;
        case (2):
                printf("Component-ID error\n");
                break;
        case (3):
                printf("Not Allowed\n");
                break;
        case (4):
                printf("Password invalid\n");
                break;

        default: break;
        }
        return response_payload[0];
}

void BonmetInterface::AxisJogCmd(int Axis, int enable, UINT16 speed, int Acc_time, int Dec_time)
{
        int data_len = 16;
        byte* message = new byte[BONMETINTERFACE_HEADER_LEN + data_len];
        createMessageHeader(message,BONMETINTERFACE_CMD_AXIS_JOG_COMMAND,data_len);
        int index=BONMETINTERFACE_HEADER_LEN;
        message[index++] = Axis; //axis_number
        message[index++] = enable; //enable
        //message[index++] = 0; //Speed
        //message[index++] = Speed; //speed
        UINT16toBytes(speed, message,index);
        index+=2;
        UINT16toBytes(Acc_time, message,index);
        index+=2;
        UINT16toBytes(Dec_time, message,index);
        index+=2;
        byte* response_payload = send_receive_pkg(message, 22);
        //printf("Axis Number: %d\n", response_payload[0]);
        //printf("Result: %d\n", response_payload[1]);
}


void BonmetInterface::AxisManageCmdSend(int Axis, int cmd)
{
        //printf("Command %d\n",cmd);
        int data_len = 16;
        byte* message = new byte[BONMETINTERFACE_HEADER_LEN + data_len];
        createMessageHeader(message,BONMETINTERFACE_CMD_AXIS_MANAGE_COMMAND_SEND,data_len);
        int index = BONMETINTERFACE_HEADER_LEN;
        message[index++] = Axis; //axis_number
        message[index++] = cmd; //enable

        byte* response_payload = send_receive_pkg(message, 22);
        //printf("Axis Number: %d\n", response_payload[0]);
        //printf("Result: %d\n", response_payload[1]);
}



byte* BonmetInterface::send_receive_pkg(byte* message, int len)
{
        this->server->send(message,len);
        byte* response_header = new byte[len];;
        this->server->receive(response_header,6);
        int response_payload_len = bytesToINT16(response_header,4);
        byte* response_payload = new byte[response_payload_len];
        this->server->receive(response_payload,response_payload_len);
        return response_payload;
}


int BonmetInterface::Get_Sys_Config()
{
        byte* message = new byte[22];
        createMessageHeader(message,BONMETINTERFACE_CMD_GET_SYS_CONFIG,16);
        int index=6;
        message[index++] = 0; //axis_number
        message[index++] = 0; //enable
        message[index++] = 0; //Speed
        message[index++] = 0; //speed
        message[index++] = 0; //Accel_time
        message[index++] = 0; //Accel_time
        message[index++] = 0; //Decel_time
        message[index++] = 0; //Decel_time

        byte* response_payload = send_receive_pkg(message, 22);

        UINT16 Token = bytesToINT16(response_payload,42);
        this->token = new byte[2];
        this->token[0] = response_payload[42];
        this->token[1] = response_payload[43];
        return Token;
}


FLOAT32* BonmetInterface::get_joint_state()
{
  byte* message = new byte[22];
  createMessageHeader(message,BONMETINTERFACE_CMD_GET_AXIS_RT_DATA,16);
  int index=6;
  message[index++] = 0; //axis_number          6
  message[index++] = 6; //number of axes         7
  message[index++] = 0; //byte offset of data to read      8
  message[index++] = 24; //byte offset of data to read     9
  message[index++] = 0; //byte size of axis data to read (AX_LEN)  10
  message[index++] = 16; //byte size of axis data to read (AX_LEN)  11
  message[index++] = 0; //zero (not used)        12
  message[index++] = 0; //zero (not used)				 13

  byte* response_payload = send_receive_pkg(message, 22);

  int mes_7 = message[7];
  FLOAT32* value = new FLOAT32[mes_7*4];

  for(int i = 0; i < mes_7*4; i=i+4)
  {
          int start_buf = i*message[11]/4;
          value[i] = bytesToFLOAT32(response_payload, start_buf);
          value[i+1] = bytesToFLOAT32(response_payload, start_buf+4);
          value[i+2] = bytesToFLOAT32(response_payload, start_buf+8);
          value[i+3] = bytesToFLOAT32(response_payload, start_buf+12);
  }
  return value;
}


void BonmetInterface::GetAxisRTData()
{
        byte* message = new byte[22];
        createMessageHeader(message,BONMETINTERFACE_CMD_GET_AXIS_RT_DATA,16);
        int index=6;
        message[index++] = 0; //axis_number          6
        message[index++] = 6; //number of axes         7
        message[index++] = 0; //byte offset of data to read      8
        message[index++] = 0; //byte offset of data to read     9
        message[index++] = 0; //byte size of axis data to read (AX_LEN)  10
        message[index++] = 128; //byte size of axis data to read (AX_LEN)  11
        message[index++] = 0; //zero (not used)        12
        message[index++] = 0; //zero (not used)				 13

        byte* response_payload = send_receive_pkg(message, 22);

        int mes_7 = message[7];
        FLOAT32* value = new FLOAT32[mes_7*4];
        int home_status[6];
        for (int i = 0; i < 6; i++)
        {
          home_status[i] = response_payload[i*message[11]+3];
        }

        for(int i = 0; i < mes_7*4; i=i+4)
        {
                int start_buf = i*message[11]/4+24;
                value[i] = bytesToFLOAT32(response_payload, start_buf);
                value[i+1] = bytesToFLOAT32(response_payload, start_buf+4);
                value[i+2] = bytesToFLOAT32(response_payload, start_buf+8);
                value[i+3] = bytesToFLOAT32(response_payload, start_buf+12);
        }

        for(int i = 0; i < 6; i++)
        {
                printf("Home Status Axis %d: ", i);
                switch(home_status[i])
                {
                case (1):
                        printf("Homing OFF\n");
                        break;
                case (2):
                        printf("Homing In Progress\n");
                        break;
                case (3):
                        printf("Homing Error Setup\n");
                        break;
                case (4):
                        printf("Homing Error Procedure\n");
                        break;
                case (5):
                        printf("Homing Timeout\n");
                        break;
                case (6):
                        printf("Homing Error Finalize\n");
                        break;
                case (7):
                        printf("Homing Not Allowed\n");
                        break;
                case (8):
                        printf("Homing Aborted\n");
                        break;
                case (9):
                        printf("Homing Not Possible\n");
                        break;
                case (10):
                        printf("Homing Done\n");
                        break;
                default: break;
              }
        }

        //return value;

}
}
