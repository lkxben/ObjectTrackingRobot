import os
from models.basic_cnn import build_basic_cnn
from scripts.data_utils import load_annotations, RHDDataGenerator
import tensorflow as tf
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau
import matplotlib.pyplot as plt

from tensorflow.keras import mixed_precision
mixed_precision.set_global_policy('mixed_float16')

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_PATH = os.path.join(BASE_DIR, "../dataset")
TRAIN_DIR = os.path.join(DATASET_PATH, "training")
EVAL_DIR = os.path.join(DATASET_PATH, "evaluation")

# Parameters
input_shape = (224, 224, 3)
num_keypoints = 21
batch_size = 16
epochs = 50

# Load annotations
train_annotations = load_annotations(os.path.join(TRAIN_DIR, "anno_training.pickle"))
val_annotations = load_annotations(os.path.join(EVAL_DIR, "anno_evaluation.pickle"))

# Generators
train_gen = RHDDataGenerator(os.path.join(TRAIN_DIR, "color"), train_annotations, batch_size=batch_size)
val_gen = RHDDataGenerator(os.path.join(EVAL_DIR, "color"), val_annotations, batch_size=batch_size)

# Callbacks
checkpoint_dir = os.path.join(BASE_DIR, "../models/checkpoints")
os.makedirs(checkpoint_dir, exist_ok=True)
checkpoint = ModelCheckpoint(
    filepath=os.path.join(checkpoint_dir, 'best.keras'),
    monitor='val_loss',
    save_best_only=True,
    save_weights_only=True,
    verbose=1
)

early_stop = EarlyStopping(
    monitor='val_loss',
    patience=5,
    restore_best_weights=True,
    verbose=1
)

reduce_lr = ReduceLROnPlateau(
    monitor='val_loss',
    factor=0.5,
    patience=3,
    verbose=1
)

# Build model
model = build_basic_cnn(input_shape=input_shape, num_keypoints=num_keypoints)
model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=1e-4),
    loss='mse',
    metrics=['mae']
)

# Train
weights_path = os.path.join(checkpoint_dir, 'best.keras')
if os.path.exists(weights_path):
    model.load_weights(weights_path)
    print("Loaded best weights from previous training")

history = model.fit(
    train_gen,
    validation_data=val_gen,
    epochs=epochs,
    verbose=1,
    validation_freq=5,
    callbacks=[checkpoint, early_stop, reduce_lr]
)

# Plot training metrics
plt.plot(history.history['loss'], label='train_loss')
plt.plot(history.history['val_loss'], label='val_loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()
plt.show()

plt.plot(history.history['mae'], label='train_mae')
plt.plot(history.history['val_mae'], label='val_mae')
plt.xlabel('Epoch')
plt.ylabel('Mean Absolute Error')
plt.legend()
plt.show()

# Save model
model.save('../models/hand_keypoint_model.h5')
print("Model saved!")