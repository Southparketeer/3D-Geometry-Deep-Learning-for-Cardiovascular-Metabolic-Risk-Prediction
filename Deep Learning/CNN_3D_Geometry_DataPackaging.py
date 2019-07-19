from PIL import Image, ImageFile
import numpy as np

IMAGE_WIDTH = 128
IMAGE_HEIGHT = 128
IMAGE_CHANNELS = 1

training_data = []
training_data2 = []
for k in range(1, 2001):
    ImageFile.LOAD_TRUNCATED_IMAGES = False
    path = 'D:\Interview YaoLu\TensorFlow\caesar-norm-wsx-fitted-meshes\CCylinder\CAESAR_Cylinder_Depth_' + str(k) + '.png'
    print(path)
    img = Image.open(path)
    img.load()
    img = img.resize((IMAGE_WIDTH, IMAGE_HEIGHT), Image.BILINEAR)
    array = np.asarray(img)
    array = (array - 10000)/ float(5000) - 1;
    training_data.append(array)
    '''
    for i in range(0 , array.shape[0]):
            print(array[i])
    print([np.amax(array), np.amin(array)])
    '''

training_data = np.reshape(training_data, (-1, IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_CHANNELS))
print("Saving training image binary...")
np.save("train_Cylinder_depth_cnn", training_data)
print("Done.")


