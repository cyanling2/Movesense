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
const float threshold = -16;
int state = 0; // 0 for above threshold, 1 for below
int step = 0;
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
            step = 0;
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
    float magnitudes[16];
    const wb::Array<wb::FloatVector3D> &accData= data.arrayAcc;
    float averageMagnitude = 0;
    size_t i;
    
    for (i=0; i<15 && i<accData.size(); i++) {
        wb::FloatVector3D a = accData[i];
        float magnitude = a.x;
        magnitudes[i + 1] = magnitude;
        averageMagnitude += magnitude;
        if (magnitude < threshold && state == 0) 
            state = 1;
        if (magnitude > threshold && state == 1) {
            state = 0;
            step++;
            ledSetPattern(200, 10, 1, true);
        }
    }
    averageMagnitude /= i;

    *((char*)magnitudes + 3) = averageMagnitude < 3.0f? 1:0;
    uint8_t tag = 5;
    
    // sendPacket((uint8_t*)magnitudes+3, 1+i*sizeof(float), tag, Responses::COMMAND_RESULT);

    int* ptr = &step;
    sendPacket((uint8_t*)ptr, 4, 7, Responses::COMMAND_RESULT);

}
