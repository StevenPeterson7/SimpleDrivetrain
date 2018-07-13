# SimpleDrivetrain
Version: v0.7

A Python library that facilitates the control of robot drivetrains with complex motor arrangements.

## Table of Contents
* [Overview](#overview)
* [Current Features](#current features)
* [Roadmap](#roadmap)
* [Requirements](#requirements)
* [Installation](#installation)
* [How to Use](#how to use)
  - [Import simpledrivetrain](#import simpledrivetrain)
  - [Creating a SimpleDrivetrain object](#creating a simpledrivetrain object)
  - [Adding and removing drive motors](#adding and removing drive motors)
  - [Updating drivetrain orientation](#updating drivetrain orientation)
  - [Getting motor velocities](#getting motor velocities)
* [License](#license)

## Overview
SimpleDrivetrain provides an easy way to define a drivetrain by the location and orientation of its motors, and calculate the desired velocities for each motor given the overall desired translational velocity and rotational velocities for the drivetrain.

## Current Features
* Local-oriented 3-axis translation and rotation
* Motor-level PWM scaling from user-defined PWM ranges or custom scaling functions

## Roadmap
* Field-oriented 3-axis translation and rotation
* Support for loading drivetrains from an XML file
* Motion profiles
* Control loop

## Requirements
* Python 2.7+/3.5+
* Numpy

## Installation
Install SimpleDrivetrain from PyPI by opening a terminal and typing the following command:
```python
$ pip install simpledrivetrain
```

## How to Use
### Import simpledrivetrain
```python
import simpledrivetrain
```
### Creating a SimpleDrivetrain object
```python
drivetrain = simpledrivetrain.SimpleDrivetrain()
```
### Adding and removing drive motors
* Motors can be added to the drivetrain by calling the ```add_new_motor``` 
method and supplying:
    - The ```name``` of the motor, a unique string identifier
    - The ```position``` of the motor relative to the drivetrain's 
      center in the form of a 3D coordinate list e.g. [x, y, z]
    - The motor's positive drive ```direction``` in the form of a 3D cartesian direction vector
      e.g. [x, y, z]
    - Optionally, a boolean value ```inverted``` to set whether the motor is inverted 
    - Optionally, a 3-tuple ```pwm_bounds``` in the form of (FULL_REVERSE, FULL STOP, FULL_FORWARD) 
      to which to scale the motor velocity
    - Optionally, a function ```pwm_scaling_func``` which accepts a motor velocity in [-1, 1] and 
      scales it to a desired pwm range
    ```python
    drivetrain.add_new_motor(name, position, direction, inverted=False, 
                             pwm_bounds=(0, 512, 1024), pwm_scaling_func=None)
    ```
* Motors can be removed from the drivetrain by calling the ```remove_motor_by_name```
  method and supplying the ```name``` of the motor to remove
  ```python
  drivetrain.remove_motor_by_name(name)
  ```
### Updating drivetrain orientation
```python
drivetrain.orientation = (pitch, roll, yaw)    
```
### Getting motor velocities
* Local-oriented
    * Motor velocities scaled in [-1, 1], stored in a list by order of motor 
    addition, can be calculated by calling the
    ```get_motor_vels_local``` method and supplying:
      - ```translation```, a 3D vector representing the drivetrain's desired 
        overall velocity, e.g. [x, y, z]
      - ```rotation```, a 3D vector representing the drivetrain's desired 
        rotational velocity, e.g. [pitching, rolling, yawing]
        ```python
        drivetrain.get_motor_vels_local(translation, rotation)
        ```
## License
SimpleDrivetrain is distributed under the terms of the [MIT License](https://choosealicense.com/licenses/mit/#)
