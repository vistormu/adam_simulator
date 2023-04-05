# 1.0.0
The ADAM Simulator has seen a complete overhaul. Now instead of a simple simulation it features a framework to test and send command to the real robot (still work in progress). Now the code is more modular, offering separate controllers for each actuator: left manipulator, right manipulator and base.

New features:
- The base can now be moved!
- The manipulators can be controlled with velocity now
- The robot can now be controlled through Inverse Kinematics

API changes:
- Now each actuator has its own controller. For that matter, most of the methods of the `Simulation` class have been deprecated. Check out the new API reference for more info.
- Changed the `ConfigurationsManager` class name to `DataManager`
- Changed the `load_scene` method name to `load`
- Changed the package name from `adam` to `adam_sim` to match the pip package name

Deprecations:
- The `MapMaker` feature is now a standalone package
- Deprecated the `extend_collisions` method
- Deprecated the fps argument for the render method as it should always render in real-time

General changes:
- Implemented the velocity returning info for the manipulators and the base
- The collisions of the manipulators are no longer cylinders but the .stl themselves
- The menu is hidden by default

Visual changes:
- Added materials and textures to the robot
- The environment now is prettier

Improvements and fixes:
- Now the collisions are checked much faster
- Fixed the systems return info on both manipulators


# 0.4.1
- Split the body into four parts to better collision checking
- Changed the systems of the returning info to match the DH transformations
- Added mock attributes to the return info for future updates
- Moved the base a tiny teeny bit up
- The base of the robot is now black

# 0.4.0
- Changed the collision model of the body to be more precise
- Added additional information about the systems in the returning info
- Implemented three new entities for the simulator: point, vector and system
- Fixed a bug where the menu could be hid
- Changed radically the data structure of the returning info
- Changed the name of the `Data` class to `AdamInfo`
- Changed the fps to be maximum by default
- Removed the light source from the robot
- Changed the light source of the scene

# 0.3.1
- Deprecated the Logger class
- Made the angles in the control mode more precise
- Implemented a hide menu option in the render function

# 0.3.0
- Deprecated the is_alive attribute of the ```Simulation``` class
- Migrated the documentation to Read the Docs
- Implemented a new feature to control the position of the manipulators via a slider
- Added docstrings to the code
- Modularized even more the ```Simulation``` class
- Changed the name of some methods of the MapMaker class for the sake of clarity

# 0.2.4
- The set view method now stores the default value of the azimuth, elevation and center so they are not passed every time

# 0.2.3
- Fixed a bug in the close method of the Simulation class

# 0.2.2
- Added more info to the Collision class separating the self collision and the environment collision
- Fixed a bug where not calling the render method loaded the MuJoCo screen
- Updated the Logger class
- Changed the check collisions function to return separately the self collisions and the env collisions as a numpy array

# 0.2.1
- Added a method to export the adam scene to a given directory
- Implemented a method to include the created map to the scene

# 0.2.0
- Added a whole new feature where bodies can be added to the scene by code.
- Added a method to use the MuJoCo engine to just check the collisions on the given configuration list

# 0.1.1
- Added a default key-value pair to the collision dictionary
- Added a method for the ```Simulation``` class where the collision dictionary can be extended
- Updated the API reference

# 0.1.0
- First version of the ADAM package!