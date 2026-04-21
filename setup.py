from setuptools import find_packages, setup

package_name = 'fire_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/fire_sim']),
        ('share/fire_sim', ['package.xml']),
	('share/fire_sim/launch', ['launch/simulation.launch.py']),
    	('share/fire_sim/worlds', ['worlds/fire_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pes2ug23cs505',
    maintainer_email='pes2ug23cs505@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
