import os
os.environ["TF_USE_LEGACY_KERAS"] = "1"

#    This file was created by
#    MATLAB Deep Learning Toolbox Converter for TensorFlow Models.
#    12-May-2026 10:11:13

import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

def create_model():
    input = keras.Input(shape=(6,))
    ActorFC1 = layers.Dense(31, name="ActorFC1_")(input)
    ActorRelu1 = layers.ReLU()(ActorFC1)
    ActorFC2 = layers.Dense(31, name="ActorFC2_")(ActorRelu1)
    ActorRelu2 = layers.ReLU()(ActorFC2)
    ActorFC3 = layers.Dense(1, name="ActorFC3_")(ActorRelu2)

    model = keras.Model(inputs=[input], outputs=[ActorFC3])
    return model
