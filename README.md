# 3D Geometry Deep Learning for Cardiovascular Metabolic Risk Prediction
Map 3D geometry information to 2D. Train the 2D embedded maps with convolutional neural networks to predict health related bio-marker.  

## Overview

## Information Embedding
We propose to unify the 3D geometry space with the 2D image space for each individual body shape sample to make 3D geometry trainable in a convolutional neural network. The unified 3D-2D space [1] aims to transform arbitrary 3D geometry into a representation that lossless convertible between 3D and 2D space. 

<p align="center">
<img width="450" src= demo/3D_Shape_Embedding.png>
</p>

## Training Model
### Convolutional Neural Network Architecture
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

### Hyperparameter Tuning with Bayesian Optimization

## Reference
[1] Lu, Yao, Scott McQuade, and James K. Hahn. "3d shape-based body composition prediction model using machine learning." In 2018 40th Annual International Conference of the IEEE Engineering in Medicine and Biology Society (EMBC), pp. 3999-4002. IEEE, 2018.


