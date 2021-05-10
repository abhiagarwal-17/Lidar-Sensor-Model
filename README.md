# Lidar-Sensor-Model

Use the Jupyter notebook for the lidar model readings in a square obstacle space. 
In order to do the readings for a circular obstacle, replace the helper methods in the first few cells of
the jupyter notebook for square obstacles with equivalents in geometry_methods.py

The most important distincttion: gemoetry_methods.py evaluates the obstacles and bounding walls separately. 

The square obstacles have both obstalces and walls in the same list. Therefore, the equivalent of
stopping_point method for circles has an extra argument for list of bounding walls. 
