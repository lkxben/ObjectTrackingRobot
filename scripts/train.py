import os
import random
from models.basic_cnn import build_basic_cnn, build_mobilenet_keypoints, build_efficientnet_keypoint_model
import tensorflow as tf
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau
import matplotlib.pyplot as plt
from scripts.data_utils import create_tf_dataset, load_annotations

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DATASET_PATH = os.path.join(BASE_DIR, "../dataset")
TRAIN_DIR = os.path.join(DATASET_PATH, "training")
EVAL_DIR = os.path.join(DATASET_PATH, "evaluation")

# Parameters
INPUT_SHAPE = (224, 224, 3)
NUM_KEYPOINTS = 42
BATCH_SIZE = 16
EPOCHS = 50

# Load annotations
train_annotations = load_annotations(os.path.join(TRAIN_DIR, "anno_training.pickle"))
val_annotations = load_annotations(os.path.join(EVAL_DIR, "anno_evaluation.pickle"))

# Use only a subset for faster iteration
TRAIN_SUBSET_SIZE = 200
VAL_SUBSET_SIZE = 50

train_keys = list(train_annotations.keys())
val_keys = list(val_annotations.keys())

train_subset_keys = random.sample(train_keys, min(TRAIN_SUBSET_SIZE, len(train_keys)))
val_subset_keys = random.sample(val_keys, min(VAL_SUBSET_SIZE, len(val_keys)))

train_subset_annotations = {k: train_annotations[k] for k in train_subset_keys}
val_subset_annotations = {k: val_annotations[k] for k in val_subset_keys}

# Generators
# train_gen = create_tf_dataset(os.path.join(TRAIN_DIR, "color"), train_annotations, batch_size=BATCH_SIZE)
# val_gen = create_tf_dataset(os.path.join(EVAL_DIR, "color"), val_annotations, batch_size=BATCH_SIZE, shuffle=False)
train_gen = create_tf_dataset(os.path.join(TRAIN_DIR, "color"), train_subset_annotations, batch_size=BATCH_SIZE)
val_gen = create_tf_dataset(os.path.join(EVAL_DIR, "color"), val_subset_annotations, batch_size=BATCH_SIZE, shuffle=False)

# Callbacks
checkpoint_dir = os.path.join(BASE_DIR, "../models/checkpoints")
os.makedirs(checkpoint_dir, exist_ok=True)
checkpoint = ModelCheckpoint(
    filepath=os.path.join(checkpoint_dir, 'best_weights.weights.h5'),
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

# Loss
def xy_v_mse(y_true, y_pred, v_weight=0.1):
    y_true = tf.reshape(y_true, (tf.shape(y_true)[0], 42, 3))
    y_pred = tf.reshape(y_pred, (tf.shape(y_pred)[0], 42, 3))

    xy_true = y_true[:, :, :2]
    xy_pred = y_pred[:, :, :2]
    v_true = y_true[:, :, 2:3]
    v_pred = y_pred[:, :, 2:3]

    xy_loss = tf.reduce_sum(tf.square(xy_true - xy_pred) * v_true) / tf.reduce_sum(v_true)
    v_loss = tf.reduce_mean(tf.square(v_true - v_pred))

    return xy_loss + v_weight * v_loss

def xy_v_mae(y_true, y_pred, v_weight=0.05):
    y_true = tf.reshape(y_true, (tf.shape(y_true)[0], 42, 3))
    y_pred = tf.reshape(y_pred, (tf.shape(y_pred)[0], 42, 3))

    xy_true = y_true[:, :, :2]
    xy_pred = y_pred[:, :, :2]
    v_true = y_true[:, :, 2:3]
    v_pred = y_pred[:, :, 2:3]

    xy_loss = tf.reduce_sum(tf.abs(xy_true - xy_pred) * v_true) / tf.reduce_sum(v_true)
    v_loss = tf.reduce_mean(tf.abs(v_true - v_pred))

    return xy_loss + v_weight * v_loss

# Build model
model = build_efficientnet_keypoint_model(input_shape=INPUT_SHAPE, num_keypoints=NUM_KEYPOINTS)
model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=1e-5),
    loss=xy_v_mse,
    metrics=[xy_v_mae]
)

# Train
# weights_path = os.path.join(checkpoint_dir, 'best_weights.weights.h5')
# if os.path.exists(weights_path):
#     model.load_weights(weights_path)
#     print("Loaded best weights from previous training")

history = model.fit(
    train_gen,
    validation_data=val_gen,
    epochs=EPOCHS,
    verbose=1,
    validation_freq=1,
    callbacks=[checkpoint]
)

# Plot training metrics
plt.plot(history.history['loss'], label='train_loss')
plt.plot(history.history['val_loss'], label='val_loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()
plt.show()

plt.plot(history.history['xy_v_mae'], label='xy_v_mae')
plt.plot(history.history['val_xy_v_mae'], label='val_xy_v_mae')
plt.xlabel('Epoch')
plt.ylabel('Mean Absolute Error')
plt.legend()
plt.show()