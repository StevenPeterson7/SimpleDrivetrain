# SimpleDrivetrain
Version: v1.0.1

A Python library that facilitates the control of robot drivetrains with complex motor arrangements.

## Table of Contents
* [Overview](#overview)
* [Current Features](#current-features)
* [Roadmap](#roadmap)
* [Requirements](#requirements)
* [Installation](#installation)
* [How to Use](#how-to-use)
  - [Import simpledrivetrain](#import-simpledrivetrain)
  - [Creating a SimpleDrivetrain object](#creating-a-simpledrivetrain-object)
  - [Adding, removing, and accessing drive motors](#adding-removing-and-accessing-drive-motors)
  - [Updating drivetrain orientation](#updating-drivetrain-orientation)
  - [Getting motor velocities](#getting-motor-velocities)
* [License](#license)

## Overview
SimpleDrivetrain provides an easy way to define a drivetrain by the location and orientation of its motors, and calculate the desired velocities for each motor given the overall desired translational velocity and rotational velocities for the drivetrain.

## Current Features
* Local-oriented and field-oriented 3-axis translation and rotation
* Motor-level PWM scaling from user-defined PWM ranges or custom scaling functions
* Support for loading drivetrains from an XML file

## Roadmap
* Motion profiles
* Control loop

## Requirements
* Python 2.7+/3.x
* Numpy
* lxml

## Installation
Install SimpleDrivetrain from PyPI by opening a terminal and typing the following command:
```
$ pip install simpledrivetrain
```

## How to Use
### Import simpledrivetrain
```python
from simpledrivetrain.simple_drivetrain import SimpleDrivetrain
```
### Creating a SimpleDrivetrain object
```python
drivetrain = SimpleDrivetrain()
```
### Adding, removing, and accessing drive motors
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
  alternatively, a motor can also be removed from the drivetrain by calling the 
  ```remove_motor_by_index``` method and supplying the motor's ```index``` 
  of addition
  ```python
  drivetrain.remove_motor_by_index(index)
  ```
* Motors can be accessed by calling the ```get_motor_by_name``` method and supplying 
  the ```name``` of the motor to retrieve
  ```python
  motor = drivetrain.get_motor_by_name(name)
  ```
  alternatively, a motor can also be accessed by calling ```get_motor_by_index``` 
  method and supplying the motor's ```index``` of addition
  ```python
  motor = drivetrain.get_motor_by_index(index)
  ```  
* Motors can also be accessed through the motors instance variable, which stores the 
  motors by order of addition in a list
  ```python
  motorlist = drivetrain.motors
  ```
* Alternatively, motors and orientation can be loaded from an xml file by calling 
  the ```load_drivetrain_from_file``` method and supplying the ```filepath``` string 
  pointing to the xml file:
  ```python
  drivetrain.load_drivetrain_from_file(filepath)
  ```
  For an example file, see [Example SimpleDrivetrain xml FIle](tests/drivetrain_test.xml).

### Updating drivetrain orientation
```python
drivetrain.orientation = (pitch, roll, yaw)    
```
### Getting motor velocities
* Motor velocities scaled in [-1, 1], stored in a list by order of motor 
    addition, can be calculated by calling the
    ```get_motor_vels``` method and supplying:
    - ```translation```, a 3D vector representing the drivetrain's desired 
        overall velocity, e.g. [x, y, z]
    - ```rotation```, a 3D vector representing the drivetrain's desired 
        rotational velocity, e.g. [pitching, rolling, yawing]
    - ```force_local_oriented```, a boolean value which, if set to true, 
    ignores current drivetrain orientation and calculates local-oriented 
    motor values. It is set to false by default.
    ```python
    drivetrain.get_motor_vels(translation, rotation, force_local_oriented=False)
    ```
* Motor velocities scaled according to user-defined, motor-level PWM ranges 
    or scaling functions, stored in a list by order of motor addition, can be 
    calculated by calling the ```get_motor_vels_scaled``` method and supplying
    the same ```translation```, ```rotation```, and ```force_local_oriented```
    parameters as the ```get_motor_vels``` method:
    ```python
    drivetrain.get_motor_vels_scaled(translation, rotation, force_local_oriented=False)
    ```

## License
SimpleDrivetrain is distributed under the terms of the [MIT License](https://choosealicense.com/licenses/mit/#).
