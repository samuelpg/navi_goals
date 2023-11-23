from distutils.core import setup

setup(
    version='0.1.0',
    scripts=['scripts/ground_truth.py', 'scripts/new_tf.py', 'scripts/lowest_distance.py'],
    packages=['gazebo_msgs', 'std_msgs', 'geometry_msgs', 'tf', 'sensor_msgs'],
    package_dir={'': 'scripts'}
)