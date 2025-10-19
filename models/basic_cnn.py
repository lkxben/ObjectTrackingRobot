import tensorflow as tf
from tensorflow.keras import layers, models
from tensorflow.keras.applications import EfficientNetB0

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
        layers.Dense(num_keypoints * 3, activation='sigmoid')
    ])
    return model

def build_mobilenet_keypoints(input_shape=(224,224,3), num_keypoints=42):
    base = tf.keras.applications.MobileNetV2(
        input_shape=input_shape,
        include_top=False,
        weights='imagenet'
    )
    base.trainable = False  # freeze for small dataset

    x = layers.GlobalAveragePooling2D()(base.output)
    x = layers.Dense(512, activation='relu')(x)
    output = layers.Dense(num_keypoints*3, activation='sigmoid')(x)

    model = models.Model(inputs=base.input, outputs=output)
    return model

def build_efficientnet_keypoint_model(input_shape=(224, 224, 3), num_keypoints=42, pretrained=True):
    base_model = EfficientNetB0(
        include_top=False,
        weights='imagenet' if pretrained else None,
        input_shape=input_shape,
        pooling='avg'
    )
    base_model.trainable = True  # set False to freeze backbone

    inputs = layers.Input(shape=input_shape)
    x = base_model(inputs, training=True)
    x = layers.Dense(512, activation='relu')(x)
    outputs = layers.Dense(num_keypoints * 3, activation='sigmoid')(x)

    model = models.Model(inputs=inputs, outputs=outputs)
    return model