# 0.3.1
- DEprecated the Logger class
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