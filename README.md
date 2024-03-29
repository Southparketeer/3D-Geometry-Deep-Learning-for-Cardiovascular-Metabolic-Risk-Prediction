# 3D Geometry Deep Learning for Cardiovascular Metabolic Risk Prediction
Map 3D geometry information to 2D. Train the 2D embedded maps with convolutional neural networks to predict health related bio-marker.  

## Overview

## Information Embedding
We propose to unify the 3D geometry space with the 2D image space for each individual body shape sample to make 3D geometry trainable in a convolutional neural network. The unified 3D-2D space [1][2][3] aims to transform arbitrary 3D geometry into a representation that lossless convertible between 3D and 2D space. 

<p align="center">
<img width="500" src= demo/3D_Shape_Embedding.png>
</p>

## Training Model
### Convolutional Neural Network Architecture (Visualize with TensorBoard)
<p align="center">
<img width="650" src= demo/TensorBoard.png>
</p>

### Accuracy Evaluation
We evaluate the training accuracy based on our baseline CNN structure (as shown above). The sample size N = 2000, we split data into 75% training and 25% testing.

<p align="center">
<img width="800" src= demo/TensorBoard_epoch.png>
</p>

<p align="center">
<img width="400" src= demo/ROC.png>
</p>

For more implementation and accuracy evaluation details, please check out the code:

https://github.com/Southparketeer/3D-Geometry-Deep-Learning-for-Cardiovascular-Metabolic-Risk-Prediction/blob/master/BaselineCNN_Visual_TensorBoard_ROC.ipynb

### Hyperparameter Tuning with Bayesian Optimization
To design a deep learning model, there are many hyperparameters that you can adjust, e.g., number of layers, number of neurons, activation functions, different coefficients like learning rate, drop rate, etc. Different with the weight (i.e., parameters) of the neural network, which can be optimized in backpropagation, the hyperparameters cannot be learned as part of neural network training. To specify the these hyperparameters so that we optimize the model design, we adopt Bayesian Hyperparameter Optimization. We apply Bayesian optimization on three hyperparameters that we care about: number of CNN layers, learning rate, and dropout rate.     

<p align="center">
<img width="900" src= demo/HyperParameter.PNG>
</p>

For more implementation and validation details, please check out the code:

https://github.com/Southparketeer/3D-Geometry-Deep-Learning-for-Cardiovascular-Metabolic-Risk-Prediction/blob/master/Hyperparameter_Analysis_Bayesian_Optimization.ipynb

## Install
#### __3D Geometry Embedding__ (C++)

Dependency:

* VCG library http://vcg.isti.cnr.it/vcglib/ for geometry processing
* libpng http://www.libpng.org/pub/png/libpng.html for read and write image as .png
* zlib https://www.zlib.net/ for installing libpng

#### __BaselineCNN_Visual_TensorBoard_ROC.ipynb__ (Python, Tensorflow, Kreas, TensorBoard)

* Google CoLab
* Runtime config: Python3, GPU

#### __Hyperparameter_Analysis_Bayesian_Optimization.ipynb__ (Python, Tensorflow, Kreas, bayesian-optimization)

* Google CoLab
* Runtime config: Python3, GPU

## Reference
[1] Lu, Yao, Scott McQuade, and James K. Hahn. "3d shape-based body composition prediction model using machine learning." In 2018 40th Annual International Conference of the IEEE Engineering in Medicine and Biology Society. IEEE, 2018.

[2] Lu, Yao, James Kwangjune Hahn, and Xiaoke Zhang. "3D Shape-based Body Composition Inference Model Using A Bayesian Network." IEEE journal of biomedical and health informatics (2019).

[3] Wang, Qiyue, Lu, Yao, Zhang, Xiaoke, and James K. Hahn, "A novel hybrid model for VAT prediction using shape descriptors." In 2019 41th Annual International Conference of the IEEE Engineering in Medicine and Biology Society. IEEE, 2019.



