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

### Accuracy Evaluation
<p align="center">
<img width="450" src= demo/ROC.png>
</p>
Accuracy: 0.93 (N = 2000, 80% Train, 20% Test) 
### Hyperparameter Tuning with Bayesian Optimization

## Reference
[1] Lu, Yao, Scott McQuade, and James K. Hahn. "3d shape-based body composition prediction model using machine learning." In 2018 40th Annual International Conference of the IEEE Engineering in Medicine and Biology Society (EMBC), pp. 3999-4002. IEEE, 2018.


