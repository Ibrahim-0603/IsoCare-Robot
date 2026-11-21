from setuptools import find_packages, setup

package_name = 'isocare_marker_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim0603',
    maintainer_email='201350446+Ibrahim-0603@users.noreply.github.com',
    description='Detect ArUco markers and publish them relative to the robot.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'marker_detector = isocare_marker_detector.marker_detector:main',
            'calibrate_camera = isocare_marker_detector.calibrate_camera:main',
        ],
    },
)
