from glob import glob

from setuptools import find_packages, setup

package_name = 'isocare_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim0603',
    maintainer_email='201350446+Ibrahim-0603@users.noreply.github.com',
    description="IsoCare decision_engine: judges the vitals and applies the doctor's drug rules.",
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'decision_engine = isocare_decision.decision_engine:main',
            'clear_dose_history = isocare_decision.clear_history:main',
        ],
    },
)
