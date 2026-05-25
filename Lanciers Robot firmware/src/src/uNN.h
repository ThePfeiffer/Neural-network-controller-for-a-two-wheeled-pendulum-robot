#ifndef UNN_H
#define UNN_H

#include <stdint.h>
#include "usubss.h"
#include "td3_model.h"
#include <EloquentTinyML.h>
#include "uencoder.h"
#include "uimu2.h"
#include "ucontrol.h"
#include "umotor.h"
#include "urobot.h"


#define NUMBER_OF_INPUTS 6
#define NUMBER_OF_OUTPUTS 1
#define TENSOR_ARENA_SIZE (10*1024)


class UNN : public USubss
{
public:
    void setup();
    float* predict(float inputs[NUMBER_OF_INPUTS]);
    void tick(void);

private:
    float NN_out[NUMBER_OF_OUTPUTS];
    Eloquent::TinyML::TfLite<NUMBER_OF_INPUTS, NUMBER_OF_OUTPUTS, TENSOR_ARENA_SIZE> ml;

};

extern UNN nn;

#endif