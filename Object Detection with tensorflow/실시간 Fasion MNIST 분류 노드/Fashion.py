import tensorflow as tf

def create_and_save_model():
    # Load the Fashion MNIST dataset
    (train_images, train_labels), (test_images, test_labels) = tf.keras.datasets.fashion_mnist.load_data()

    # Normalize the images to [0, 1] range
    train_images = train_images / 255.0
    test_images = test_images / 255.0

    # Define the model structure
    model = tf.keras.Sequential([
        tf.keras.layers.Flatten(input_shape=(28, 28)),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dense(10, activation='softmax')
    ])

    # Compile the model
    model.compile(optimizer='adam',
                  loss='sparse_categorical_crossentropy',
                  metrics=['accuracy'])

    # Train the model
    model.fit(train_images, train_labels, epochs=10)

    # Save the trained model
    model.save("fashion_mnist_model.h5")
    print("Model saved successfully.")

if __name__ == "__main__":
    create_and_save_model()
