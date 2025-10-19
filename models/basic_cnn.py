import tensorflow as tf
from tensorflow.keras import layers, models

def build_basic_cnn(input_shape=(224, 224, 3), num_keypoints=42):
    model = models.Sequential([
        layers.Input(shape=input_shape), 
        layers.Conv2D(32, (3,3), activation='relu'),
        layers.BatchNormalization(),
        layers.MaxPooling2D(2,2),
        layers.Dropout(0.1),

        layers.Conv2D(64, (3,3), activation='relu'),
        layers.BatchNormalization(),
        layers.MaxPooling2D(2,2),
        layers.Dropout(0.1),

        layers.Conv2D(128, (3,3), activation='relu'),
        layers.BatchNormalization(),
        layers.MaxPooling2D(2,2),
        layers.Dropout(0.1),

        layers.Flatten(),
        layers.Dense(512, activation='relu'),
        layers.Dense(num_keypoints * 3)
    ])
    return model