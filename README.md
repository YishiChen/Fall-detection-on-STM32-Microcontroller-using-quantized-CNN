# This is a course project for the course Machine Learning on Microcontrollers at ETH Zürich.

This repo contains code for pre-processing the kFall dataset for training a real-time fall detection model. It also contains code for training using this dataset, and quantizing and pruning a CNN network to fit on a STM32 microcontroller.
The CubeIDE code is included as well.

The aim of this project is to develop a quantized and pruned CNN model for fall detection using real-time sensor data from the accelerometer and gyroscope on a STM32 microcontroller.

The model was implemented and tested on the microcontroller itself and performed close to state-of-the-art models proposed in litterature, while having much smaller complexity.

