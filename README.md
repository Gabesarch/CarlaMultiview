# CarlaMultiview

Requires carla97

Clone this repo in <path_to_carla_97>/PythonAPI/examples

cd <path_to_carla_97>/PythonAPI/examples/CarlaMultiview

Make sure this egg file path is correct with respect to the CarlaMultiview repo: ../../carla/dist/carla-*%d.%d-%s.egg. If not, change it in the python file.

Run any script.


Possible issues:

If you get b'' in output, then that means there is a carla server already running that you need to kill

If you get 'no module named carla' check the path of the dist file. Make sure that is correct

CODES:

Single vehicle - carla_multiprocess_unique.py

Single vehicle far cam - carla_multiprocess_unique_far_camera.py

Two vehicle close camera - carla_multiprocess_two_unique.py

Two vehicle far camera - carla_multiprocess_two_vehicle_far_camera.py

Multi vehicle scenes - carla_multiprocess_n_vehicles.py

Multi vehicle scenes with stereo cameras - carla_multiprocess_n_vehicles_depthpred.py