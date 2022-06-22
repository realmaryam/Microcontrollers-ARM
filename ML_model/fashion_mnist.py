import tensorflow as tf
import pandas as pd
import os

%cd /content/drive/MyDrive/
train = pd.read_csv("fashion-mnist_train.csv")
test = pd.read_csv("fashion-mnist_test.csv")
# print(train)
# train_data = train.drop(columns=['label']).to_numpy()
# test_data = test.drop(columns=['label']).to_numpy()
# train_data = train_data.to_numpy()
# test_data = test_data.to_numpy()
# print(train_data.dtype, train_data.shape)
# print(train_data)
# print(test_data.dtype, test_data.shape)

# https://pandas.pydata.org/pandas-docs/stable/reference/api/pandas.DataFrame.pop.html
train_label = train.pop('label').to_numpy()
test_label = test.pop('label').to_numpy()
# print(train)
train_data = train.to_numpy()
test_data = test.to_numpy()
train_data = train_data/255.
test_data = test_data/255.
print(train_data.dtype, train_data.shape)
print(test_data.dtype, test_data.shape)

model = tf.keras.models.Sequential()
model.add(tf.keras.layers.InputLayer(input_shape=[28*28]))
model.add(tf.keras.layers.Dense(300, activation=tf.keras.activations.relu))
model.add(tf.keras.layers.Dense(100, activation=tf.keras.activations.relu))
model.add(tf.keras.layers.Dense(10, activation=tf.keras.activations.softmax))

model.compile(loss=tf.keras.losses.sparse_categorical_crossentropy, optimizer=tf.keras.optimizers.SGD(),
              )
model.fit(x=train_data, y=train_label, validation_split=0.1, epochs=30)

model_version = "0001"
model_name = "D:\docker\kafka_broker\serving\models\my_mnist_model"
model_path = os.path.join(model_name, model_version)
tf.saved_model.save(model, model_path)
model.save("model.h5")

# Convert Keras model to a tflite model
import tensorflow as tf
import tensorflow.keras as keras
model = keras.models.load_model('/content/drive/MyDrive/model.h5')

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
tflite_model = converter.convert()
print(tflite_model, type(tflite_model))
# Save the model.
%mkdir /content/drive/MyDrive/FMN
%cd /content/drive/MyDrive/FMN
with open('model.tflite', 'wb') as f:
  f.write(tflite_model)

# open(tflite_model + '.tflite', 'wb').write(tflite_model)

from google.colab import drive
drive.mount('/content/drive')