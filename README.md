# OsimToBiomod
This librairy convert Opensim model into biorbd model format. As the optimisation is performed on a third part software ([bioptim](https://github.com/pyomeca/bioptim)) so some features are not implemented directly in the model. 

## How to install 
To install the program run the following comand in the main directory

```bash
python setup.py install
```

## What's implemented
This converter works only for osim model for version 40000 and latest. If you have a model in a previous version please open and save the model using Opensim 4.x. 

There is a enum.py file which contains all the implemented model features. Features are also listed bellow:
  - Joints : CustomJoint, WeldedJoints
  - Forces : Muscles (only via points are implemented)
  - Kinematics constraint : None
  - Probes : None
  - Controller : None
  - ContactGeometry : None
  - Component : None

An example is provided to show how to convert a file. There are options to set the Biomod muscle type and muscle state type. Also the muscle applied, joint clamped and joint fixed options in opensim model can be ignored while set the option to True. 
If muscle applied not ignored : the muscle which is not applied will be put in the biomod as a comment. 
If joint clamped is not ignored: the joint range will be put in the biomod as a comment. Biorbd will assume range in [-pi, pi].
If joint fixed is not ignored: the degree of freedom will be ignored in the biomod. 

## Test the model
Is comming ...
