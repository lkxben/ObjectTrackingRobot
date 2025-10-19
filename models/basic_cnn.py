import tensorflow as tf
from tensorflow.keras import layers, models

def build_basic_cnn(input_shape=(224, 224, 3), num_keypoints=21):
    model = models.Sequential([
        layers.Input(shape=input_shape), 
        layers.Conv2D(32, (3,3), activation='relu'),
        layers.MaxPooling2D(2,2),

        layers.Conv2D(64, (3,3), activation='relu'),
        layers.MaxPooling2D(2,2),

        layers.Conv2D(128, (3,3), activation='relu'),
        layers.MaxPooling2D(2,2),

        layers.Flatten(),
        layers.Dense(512, activation='relu'),
        layers.Dense(num_keypoints * 4)
    ])
    return model