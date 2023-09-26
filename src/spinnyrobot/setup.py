from setuptools import find_packages, setup

package_name = 'spinnyrobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={package_name: ['models/model.sdf']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='michael@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spinnyrobot = spinnyrobot.spinnyrobot:main'
        ],
    },
)
