from setuptools import find_packages, setup

package_name = 'isocare_patient_id'

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
    description='Identify patients from wristband QR codes.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'patient_id = isocare_patient_id.patient_id:main',
        ],
    },
)
