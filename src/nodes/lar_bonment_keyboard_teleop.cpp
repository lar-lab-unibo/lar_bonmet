//ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <keyboard/Key.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>

#include "BonmetInterface.h"

using namespace lar_bonmet;
using namespace std;

//ROS
ros::NodeHandle* nh;
ros::Publisher joint_state_publisher;
ros::Subscriber joint_state_subscriber;
ros::Subscriber keydown_subscriber;

//NETWORK
std::string motion_controller_host;
int motion_controller_port;

//TESTING
FLOAT32 target_angle;
int target_axis = 2;


struct TestSetpoint {
        int axis = 0;
        FLOAT32 target_angle = 0;

};

TestSetpoint currentSetpoint;
TestSetpoint lastSetpoint;

/**
 * Initialize message
 */
void createJointStateMessage(sensor_msgs::JointState& joint_state,FLOAT32* pos) {
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position.resize(6);
        joint_state.name[0] ="base_to_link1";
        joint_state.name[1] ="link1_to_link2";
        joint_state.name[2] ="link2_to_link3";
        joint_state.name[3] ="link3_to_link4";
        joint_state.name[4] ="link4_to_link5";
        joint_state.name[5] ="link5_to_link6";
        for(int i =0; i < 6; i++) {
                joint_state.position[i] = pos[i*4];
        }
}

void printCommandList(){
        cout << endl;
        cout << endl;
        cout << "Command List:" << endl;
        cout << "-------------" << endl;
        cout << "NUMBERS form 1 to 6 -> Select Axis";
        cout << endl;
        cout << "LEFT/RIGHT ARROW -> Decrease/Increase Axis Number";
        cout << endl;
        cout << "UP/DOWN ARROW -> Increase/Descrease position Reference";
        cout << endl;
        cout << "-------------";
        cout << endl;
        cout << endl;
}


/**
 * Joint State Callback
 */
void joint_state_callback(const sensor_msgs::JointState& joint_state){
        printf("PUBLISHED\n");
}

/**
 * Keyboard keydown Callback
 */
void keyboard_keydown_callback(const keyboard::Key& msg){

        switch (msg.code) {

        case 105:
                printCommandList();
                break;

        case 273:
                currentSetpoint.target_angle+=10;
                break;
        case 274:
                currentSetpoint.target_angle-=10;
                break;
        case 275:
                currentSetpoint.axis+=1;
                currentSetpoint.axis=currentSetpoint.axis%6;

                // Joint 3 Temporary broken
                if (currentSetpoint.axis == 3 ) {
                        ROS_INFO("Joint Temporary Broken, Not usable.");
                        currentSetpoint.axis++;
                }
                break;
        case 276:
                currentSetpoint.axis-=1;
                currentSetpoint.axis=currentSetpoint.axis%6;
                // Joint 3 Temporary broken
                if (currentSetpoint.axis == 3 ) {
                        ROS_INFO("Joint Temporary Broken, Not usable.");
                        currentSetpoint.axis--;
                }
                break;
        case 48:
                currentSetpoint.axis = 0;
                break;
        case 49:
                currentSetpoint.axis = 1;
                break;
        case 50:
                currentSetpoint.axis = 2;
                break;
        case 51:
                // Joint 3 Temporary broken
                ROS_INFO("Joint Temporary Broken, Not usable.");
                //currentSetpoint.axis = 4;

                break;
        case 52:
                currentSetpoint.axis = 4;
                break;
        case 53:
                currentSetpoint.axis = 5;
                break;

        case 13:
                lastSetpoint = currentSetpoint;
                ROS_INFO("\n ------ \n Command Sent! \n ------ \n");
                break;


        default:
                ROS_INFO("INVALID COMMAND...Try again. \n \n Command List:\n ------------- \n NUMBERS form 1 to 6 -> Select Axis\n \n LEFT/RIGHT ARROW -> Decrease/Increase Axis Number\n \n UP/DOWN ARROW -> Increase/Descrease position Reference\n ------------- \n \n" );
                break;
        }

        ROS_INFO("Selected Axis: %d \n Target Angle: %f \n ------ \n",currentSetpoint.axis,currentSetpoint.target_angle);
        //ROS_INFO("Last\nCurrentAxis: %d \n CurrentAngle: %f \n ------ \n",lastSetpoint.axis,lastSetpoint.target_angle);
}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        // Initialize ROS
        ros::init(argc, argv, "lar_bonment_keyboard_teleop");
        ROS_INFO("lar_bonment_keyboard_teleop node started...");
        nh = new ros::NodeHandle("~");

        //Topics
        joint_state_publisher = nh->advertise<sensor_msgs::JointState>("/bonmet/joint_state", 1);
        joint_state_subscriber = nh->subscribe("/bonmet/joint_state_publisher",1,joint_state_callback);
        keydown_subscriber = nh->subscribe("/keyboard/keydown",1,keyboard_keydown_callback);

        nh->param<std::string>("host", motion_controller_host, "192.168.80.105");
        nh->param<int>("port", motion_controller_port, 19787);
        nh->param<float>("target_angle", target_angle, 0.0f);
        nh->param<int>("target_axis", target_axis, 0);

        //Bonmet Interface
        BonmetInterface* bonmet = new BonmetInterface(motion_controller_host,motion_controller_port);
        bonmet->Set_Autority(2); // 1 PLC, 2 GUI;



        // Joint 3 Temporary broken
        bonmet->AxisManageCmdSend(3,  BONMETINTERFACE_MANAGE_CMD_SWITCH_OFF);

        /**
         *   Send initial error reset and switch on commands for all Joints
         */
        for(int i = 0; i < 6; i++) {

                // Joint 3 Temporary broken
                if (i==3) {
                        continue;
                }
                //ROS_INFO("Loop Iteration: %d", i);
                bonmet->AxisManageCmdSend(i,  BONMETINTERFACE_MANAGE_CMD_RESET_ERROR);
                bonmet->AxisManageCmdSend(i,  BONMETINTERFACE_MANAGE_CMD_SWITCH_ON);
                bonmet->AxisJogCmd(i, BONMETINTERFACE_JOG_CMD_ENABLE_STOP, 0, 0, 0);
        }


        // PID GAINS SETTING
        bonmet->setPID(0,30,0,3);
        bonmet->setPID(1,30,0,3);
        bonmet->setPID(2,30,0,3);
        // Joint 3 Temporary broken
        //bonmet->setPID(3,30,0,3);
        bonmet->setPID(4,30,0,3);
        bonmet->setPID(5,20,0,2);

        FLOAT32 home_position= 0;
        UINT16 home_speed_1 = 0;
        UINT16 home_speed_2 = 0;
        UINT16 home_accel = 0;
        FLOAT32* pos;

        int t_axis = 3;

        /**
           *Commenti temporanei, ripristinare
         */

        pos = bonmet->get_joint_state();
        printf("Angle: %f\n", pos[t_axis*4]);

        usleep(1000000);
        //Security STOP (BUG deimessaggi uguali!!!)

        /**
           *Commenti temporanei, ripristinare
         */

        //bonmet->GetAxisRTData();

        usleep(1000);
        //bonmet->AxisHomeCommand(t_axis, 0,0, home_position, home_speed_1, home_speed_2, home_accel);

        /**
           *Commenti temporanei, ripristinare
         */

        //bonmet->GetAxisRTData();

        sensor_msgs::JointState joint_states;


        ros::Rate r(1000);

        /**
         *   STARTUP MESSAGE
         */
        printCommandList();


        while (nh->ok() && bonmet->isConnected() ) {

                bonmet->GoToAngle(lastSetpoint.axis, lastSetpoint.target_angle);
                //bonmet->GoToConfiguration(lastSetpoint.target_angle,lastSetpoint.target_angle,lastSetpoint.target_angle,lastSetpoint.target_angle,lastSetpoint.target_angle,lastSetpoint.target_angle);
                pos = bonmet->get_joint_state();
                //printf("Axis %d: %f\n", t_axis, pos[t_axis*4]);
                createJointStateMessage(joint_states,pos);
                joint_state_publisher.publish(joint_states);
                ros::spinOnce();
                r.sleep();
                // usleep(100000);
        }


}
