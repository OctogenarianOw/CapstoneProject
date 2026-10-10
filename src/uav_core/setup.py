from setuptools import find_packages, setup

package_name = 'uav_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doria',
    maintainer_email='dorian.veloso@student.uts.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['offboard_demo = uav_core.offboard_demo:main',
                            'square_mission = uav_core.square_mission:main',
        ],
    },
)
