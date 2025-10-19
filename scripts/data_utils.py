import os
import pickle
import tensorflow as tf

IMG_SIZE = 224
ORIGINAL_SIZE = (320.0, 320.0)

def augment(image, keypoints):
    keypoints = tf.reshape(keypoints, (42, 3))

    # Horizontal flip
    if tf.random.uniform([]) < 0.5:
        image = tf.image.flip_left_right(image)
        keypoints = tf.stack([1.0 - keypoints[:, 0], keypoints[:, 1], keypoints[:, 2]], axis=-1)

    # Random brightness
    if tf.random.uniform([]) < 0.5:
        image = tf.image.random_brightness(image, max_delta=0.2)

    # Random contrast
    if tf.random.uniform([]) < 0.5:
        image = tf.image.random_contrast(image, lower=0.8, upper=1.2)

    # Random saturation
    if tf.random.uniform([]) < 0.5:
        image = tf.image.random_saturation(image, lower=0.8, upper=1.2)

    # Random hue
    if tf.random.uniform([]) < 0.5:
        image = tf.image.random_hue(image, max_delta=0.05)

    # # Optional: small random rotation
    # if tf.random.uniform([]) < 0.5:
    #     angle = tf.random.uniform([], -0.1, 0.1)  # radians
    #     image = tfa.image.rotate(image, angle)  # If you really want rotation, else skip
    #     # rotation for keypoints requires more work; skip for now

    keypoints = tf.reshape(keypoints, [-1])
    return image, keypoints

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
    dataset = dataset.map(augment)
    dataset = dataset.batch(batch_size).prefetch(tf.data.AUTOTUNE)

    return dataset