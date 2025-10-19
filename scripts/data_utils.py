import os
import pickle
import tensorflow as tf

IMG_SIZE = 224
ORIGINAL_SIZE = (320.0, 320.0)

def load_annotations(pickle_path):
    with open(pickle_path, "rb") as f:
        annotations = pickle.load(f)
    return annotations

def preprocess(image_path, keypoints):
    image = tf.io.read_file(image_path)
    image = tf.image.decode_png(image, channels=3)
    image = tf.image.resize(image, (IMG_SIZE, IMG_SIZE))
    image = tf.cast(image, tf.float32) / 255.0

    keypoints = tf.cast(keypoints, tf.float32)
    uv = keypoints[:, :2] / 320.0
    visibility = keypoints[:, 2:]
    keypoints = tf.concat([uv, visibility], axis=-1)
    keypoints = tf.reshape(keypoints, [-1])

    return image, keypoints

def create_tf_dataset(image_dir, annotations, batch_size=32, shuffle=True):
    keys = list(annotations.keys())
    image_paths = [os.path.join(image_dir, f"{k:05d}.png") for k in keys]
    keypoints = [annotations[k]["uv_vis"] for k in keys] 

    image_paths = tf.constant(image_paths)
    keypoints = tf.constant(keypoints, dtype=tf.float32)

    dataset = tf.data.Dataset.from_tensor_slices((image_paths, keypoints))

    if shuffle:
        dataset = dataset.shuffle(buffer_size=len(image_paths), reshuffle_each_iteration=True)

    dataset = dataset.map(preprocess, num_parallel_calls=tf.data.AUTOTUNE)
    dataset = dataset.batch(batch_size).prefetch(tf.data.AUTOTUNE)

    return dataset