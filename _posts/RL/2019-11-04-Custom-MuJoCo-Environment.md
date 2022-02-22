---
layout: post
title: Custom MuJoCo Environment in Openai Gym
date: 2019-11-04 11:12:00-0400
description: How to make a custom reinforcement learning environment based on Openai Gym in MuJoCo
categories: Reinforcement-Learning Programming
---

### Pre-Requisites

[MuJuCo](http://www.mujoco.org/index.html) is a proprietary software which can be used for physics based simulation. First thing is to get a license as described in [here](https://www.roboti.us/license.html). Then install [mujoco-py](https://github.com/openai/mujoco-py) as described in the ```Readme.md```.

### Procedure
The general structure of the package creation for registering openai-gym environments is as follows

```
FolderA/
     README.md
     setup.py
     FolderB/
           __init__.py
           envs/
                __init__.py
                MyRobotEnv.py
```

```README.md``` usually contain description of the project. The ```FolderA/setup.py``` should contain the following lines
``` python
from setuptools import setup
setup(name='FolderB',
      version='0.1',
      install_requires=['gym',
                       'numpy']) # And any other dependencies required
```
The ```FolderA/FolderB/__init__.py``` should have
```python
from gym.envs.registration import register
register(id='RobotName-v0',
         entry_point='FolderB.envs:EnvClassName',)
```

```EnvClassName``` is the ```class``` described inside ```MyRobotEnv.py``` file. The file ```FolderA/FolderB/envs/__init__.py``` should contain
```python
from FolderB.envs.MyRobotEnv import EnvClassName
```
### Setup MuJoCo Environment
The existing MuJoCo environments are available [here](https://github.com/openai/gym/blob/master/gym/envs/mujoco/). MuJoCo XML files for several robots and an extensive documentations are published here([openai-github]( https://github.com/openai/gym/tree/master/gym/envs/mujoco/assets), [Modeling](http://mujoco.org/book/modeling.html)).
The structure of the ```MyRobotEnv.py``` will be as follows.

``` python
import numpy as np
import os
from gym import utils, error, spaces
from gym.envs.mujoco import mujoco_env
from mujoco_py import MjViewer, functions

class EnvClassName(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        utils.EzPickle.__init__(self)
        FILE_PATH = '' # Absolute path to your .xml MuJoCo scene file 
                        # OR.
        # FILE_PATH = os.path.join(os.path.dirname(__file__), "MyRobot.xml")
        frame_skip = 5
        mujoco_env.MujocoEnv.__init__(self, FILE_PATH, frame_skip)

    def step(self, a):
        # Carry out one step 
        # Don't forget to do self.do_simulation(a, self.frame_skip)

    def viewer_setup(self):
        # Position the camera

    def reset_model(self):
        # Reset model to original state. 
        # This is called in the overall env.reset method
        # do not call this method directly. 

    def _get_obs(self):
      # Observation of environment fed to agent. This should never be called
      # directly but should be returned through reset_model and step
```

### Installation

1. Inside the folder where the ```setup.py``` is, run

``` pip install -e . ```

## Test Installation

``` python
import gym
import FolderB
env = gym.make('RobotName-v0')
```
