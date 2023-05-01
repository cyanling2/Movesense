#include "myApp.h"
#include "interface.h"

// sensor libraries for data interperting
#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"
#include "meas_magn/resources.h"
#include "meas_imu/resources.h"

//Commands formated as byte array with [Command, data?...] (data optional)
enum Commands 
{
    HELLO = 0,
    BEGIN_SUB=1,
    END_SUB=2,
    LED=3
};
//Responses formated as byte array with [Response, tag, data?...] (data optional)
enum Responses 
{
    COMMAND_RESULT = 1,
    DATA = 2,
    ERROR = 3,
};
          
const char IMUPath[]="/Meas/IMU6/52";
const uint8_t DEFAULT_REFERENCE=99; //appears as 63 in hex
const float threshold_wrist = -16; //acc x
const float threshold_ankle_stance = -70; // gyro z
const float threshold_ankle_swing = 120; //gyro z
int state_wrist = 0; // 0 for above threshold, 1 for below
int state_ankle_stance = 0; // 0 for above threshold, 1 for below
int state_ankle_swing = 1; // 0 for above threshold, 1 for below
int step_wrist = 0;
int step_ankle = 0;
bool stance_flag = false;
uint16_t pattern[6];
void myApp::handleCommand(uint8_t cmd, const uint8_t values[], size_t len){
    switch (cmd)
    {
        case Commands::HELLO:
        {
            // Hello response, for use as a sanity check <3
            uint8_t helloMsg[] = {'H','e','l','l','o','!'};
            //tags aren't explicitly necessary, but they're a good way of grouping responses.
            //The included console uses them to filter and format responses.
            uint8_t tag=1;
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
        }
        break;
        case Commands::BEGIN_SUB:
        {
            //unsubscribes to prevent duplicate subscriptions
            unsubscribe(DEFAULT_REFERENCE);
            //subscribes to the path given above, in this case the IMU at 104hz
            step_wrist = 0;
            step_ankle = 0;
            stance_flag = false;
            subscribe(IMUPath, sizeof(IMUPath), DEFAULT_REFERENCE);
        }
        break;
        case Commands::END_SUB:
        {
            //unsubscribes only from default service
            unsubscribe(DEFAULT_REFERENCE);
        }
        break;
        case Commands::LED: 
        {
            for (int i=0; i<6; i++) pattern[i] = i%2==1? 500:0;
            for (size_t j = 0; j < 3 && j<len; j++) {
                pattern[j*2] = (uint16_t) values[j]*1000;
            }
            ledSetPattern_n(pattern, 6);
        }
    }
}

void myApp::processData(wb::ResourceId resourceId, const wb::Value &value){
    if (findDataSub(resourceId) -> clientReference != DEFAULT_REFERENCE) 
        return;
    
    const WB_RES::IMU6Data &data = value.convertTo<WB_RES::IMU6Data&>();
    float magnitudes_acc[16], magnitudes_gyro[16];
    const wb::Array<wb::FloatVector3D> &accData= data.arrayAcc;
    const wb::Array<wb::FloatVector3D> &gyroData= data.arrayGyro;
    float averageMagnitude = 0;
    size_t i;
    
    for (i=0; i<15 && i<accData.size(); i++) {
        //accelerometer
        wb::FloatVector3D a = accData[i];
        float magnitude_acc = a.x;
        magnitudes_acc[i + 1] = magnitude_acc;
        averageMagnitude += magnitude_acc;
        if (magnitude_acc < threshold_wrist && state_wrist == 0) 
            state_wrist = 1;
        if (magnitude_acc > threshold_wrist && state_wrist == 1) {
            state_wrist = 0;
            step_wrist++;
            ledSetPattern(200, 10, 1, true);
        }

        //gyroscope
        wb::FloatVector3D g = gyroData[i];
        float magnitude_gyro = g.z;
        if (!stance_flag) {
            if (magnitude_gyro < threshold_ankle_stance && state_ankle_stance == 0) 
                state_ankle_stance = 1;
            if (magnitude_gyro > threshold_ankle_stance && state_ankle_stance == 1) {
                state_ankle_stance = 0;
                step_ankle++;
                stance_flag = true;
                uint8_t stanceMsg[] = {'s', 't', 'a', 'n', 'c', 'e'};
                sendPacket(stanceMsg, sizeof(stanceMsg), 5, Responses::COMMAND_RESULT);
                ledSetPattern(200, 10, 1, true);
                int* ptr = &step_ankle;
                sendPacket((uint8_t*)ptr, 4, 7, Responses::COMMAND_RESULT);    
                
            }
        } else {
            if (magnitude_gyro > threshold_ankle_swing && state_ankle_swing == 1) 
                state_ankle_swing = 0;
            if (magnitude_gyro < threshold_ankle_swing && state_ankle_swing == 0) {
                state_ankle_swing = 1;
                step_ankle++;
                stance_flag = false;
                uint8_t swingMsg[] = {'s', 'w', 'i', 'n', 'g'};
                sendPacket(swingMsg, sizeof(swingMsg), 6, Responses::COMMAND_RESULT);
                ledSetPattern(200, 10, 1, true);
                int* ptr = &step_ankle;
                sendPacket((uint8_t*)ptr, 4, 7, Responses::COMMAND_RESULT);    
            }
        }

    }
    averageMagnitude /= i;

    *((char*)magnitudes_acc + 3) = averageMagnitude < 3.0f? 1:0;
    uint8_t tag = 5;
    
    // sendPacket((uint8_t*)magnitudes+3, 1+i*sizeof(float), tag, Responses::COMMAND_RESULT);

    

}
