# 3D Geometry Deep Learning for Cardiovascular Metabolic Risk Prediction
Map 3D geometry information to 2D. Train the 2D embedded maps with convolutional neural networks to predict health related bio-marker.  

## Overview

## Information Mapping
We propose a novel method to unify the 3D geometry space with the 2D image space for each individual body shape sample and to unify features in span of shape dataset. 

<p align="center">
<img width="600" src= demo/3D_Shape_Embedding.png>
</p>

The unified 3D-2D space aims to transform arbitrary 3D geometry into a representation that lossless convertible between 3D and 2D space.  In the transformation, two properties should be taken into consideration are data fidelity and mapping uniformity. The mapping between input 3D geometry (original shape) and the 3D geometry shape under unified space (unified shape) should be as high-fidelity as possible. The 2D image representation (unified image) of 3D original shape facilitates the raw feature extraction by quantifying the feature on 2D grid and makes raw features comparable in span of dataset once they are transformed into the same unified 3D-2D space.

## Training Model
### Convolutional Neural Network Architecture

### Accuracy Evaluation

### Hyperparameter Tuning with Bayesian Optimization



