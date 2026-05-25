#include "uNN.h"
#include "main.h"
#include "td3_model.h"

UNN nn;

void UNN::setup() {
    // initialize the NN
    ml.begin(td3_model);
}

float* UNN::predict(float inputs[NUMBER_OF_INPUTS]) {
    // pass input to NN and get the output
    // // if (Serial.available())
    // Serial.println("inputs: ");
    // for (uint8_t i =0; i< NUMBER_OF_INPUTS; i++)
    //     Serial.println(inputs[i], 4);

    ml.predict(inputs, NN_out);

    // apply tanh to the output
    for (uint8_t i = 0; i< NUMBER_OF_OUTPUTS; i++)
        NN_out[i] = tanh(NN_out[i]);
    
    // apply scaling to the output
    for (uint8_t i =0; i< NUMBER_OF_OUTPUTS; i++)
        NN_out[i] = NN_out[i] * 10;
    
    // Serial.println("NN output: ");
    // for (uint8_t i =0; i< NUMBER_OF_OUTPUTS; i++)
    //     Serial.println(NN_out[i], 10);
        
    return NN_out;
}

// float UNN::vel_filter(float vel_ref) {
//     const float tn = 0.45f;
//     const float wn = 1.0f / tn;
//     const float zeta = 0.9f;
    
//     const float K = 2.0f / robot.SAMPLETIME;
//     const float wn2 = wn * wn;
//     const float K2 = K * K;

//     const float a0 = K2 + 2.0f * zeta * wn * K + wn2;
//     const float a1 = 2.0f * (wn2 - K2);
//     const float a2 = K2 - 2.0f * zeta * wn * K + wn2;

//     const float b0 = wn2;
//     const float b1 = 2.0f * wn2;
//     const float b2 = wn2;

//     const float y = (b0 * vel_ref + b1 * vel_x1 + b2 * vel_x2
//                    - a1 * vel_y1 - a2 * vel_y2) / a0;

//     vel_x2 = vel_x1;
//     vel_x1 = vel_ref;
//     vel_y2 = vel_y1;
//     vel_y1 = y;

//     return y;
// }

// float UNN::ref_filter(float ref){
//     const float w0 = 10.0f * 2.0f * M_PI; 
//     const float Ts = robot.SAMPLETIME;        // sample time (s)
//     const float alpha = expf(-w0 * Ts);       // exact ZOH factor

//     y = alpha * y + (1.0f - alpha) * ref;
//     return y;
// }



void UNN::tick(void){

    float input[NUMBER_OF_INPUTS] = {
        encoder.wheelVelocityEst[0], //m/s
        encoder.wheelVelocityEst[1], //m/s
        encoder.robotVelocity,        //m/s
        control.mission_vel_ref,      //m/s

        //encoder.robotTurnrate,       //1/m
        // TurnRate reference, Cannot be specified in RegbotGUI.
        //control.mission_turn_radius, //rad/s // The turn radius command can be used if wanted.

        imu2.gyroTiltRate / 10, // rad/s // division by 10, as NN was trained on a gyro signal divided by 10.
        encoder.pose[3] //tilt in radians
        };

    // for (int i = 0; i < NUMBER_OF_INPUTS; i++) {
    //     Serial.print(input[i], 8);
    //     Serial.print(", ");
    // }
    // Serial.println();

    float* voltage = nn.predict(input); //pass inputs to NN and get the output
    // Serial.print("NN output: ");
    // for (uint8_t i =0; i< NUMBER_OF_OUTPUTS; i++){
    //     Serial.print(voltage[i], 4);
    // }
    // Serial.println();
    
    // set motor voltage based on NN output
    if (NUMBER_OF_OUTPUTS == 1) {
        // apply voltage to motors
        motor.motorVoltage[0] = voltage[0];
        motor.motorVoltage[1] = voltage[0];

        // motor.motorVoltage[0] = 5;
        // motor.motorVoltage[1] = 5;

    } else if (NUMBER_OF_OUTPUTS == 2) {
        // apply voltage to motors  
        motor.motorVoltage[0] = voltage[0];
        motor.motorVoltage[1] = voltage[1];
    }
    else {
        // error
    }
    
}
