import os
from glob import glob

from setuptools import setup


package_name = 'vlm_tts_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'captures'), glob('captures/*')),
        (os.path.join('share', package_name, 'model', 'InternVL3_1B'),
            glob(os.path.join('model', 'InternVL3_1B', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JasperXzy',
    maintainer_email='jasper.zhengyi.xu@gmail.com',
    description='Triggered image-to-text and TTS demo for RDK S100.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vlm_tts_demo_node = vlm_tts_demo.vlm_tts_demo_node:main',
        ],
    },
)
