from setuptools import find_packages, setup

package_name = 'my_package'

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
    maintainer='jesus',
    maintainer_email='jesus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nodo_mando = my_package.nodo_mando:main',
            'nodo_cinematica = my_package.nodo_cinematica:main',
            'pose_moveit = my_package.nodo_pose_moveit:main',
            'nodo_IO = my_package.nodo_IO:main',
            'nodo_mensaje_recibido = my_package.nodo_mensaje_recibido:main'
        ],
    },
)
