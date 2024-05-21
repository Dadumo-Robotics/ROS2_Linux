from setuptools import setup
import os
from glob import glob

package_name = 'dadumo_capture_image'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')) # Añadimos esta linea
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adminrobotico',
    maintainer_email='hugomarescrihuela@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capturar = dadumo_capture_image.capturar:main'
        ],
    },
)
