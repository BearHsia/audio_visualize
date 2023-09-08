from setuptools import setup

package_name = 'audio_visualize'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xyztlab',
    maintainer_email='liao163@purdue.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'audio_visualizer_node = audio_visualize.audio_visualizer:main',
        	'audio_record_node = audio_visualize.UMA16_publisher:main',
        	'audio_logging_node = audio_visualize.UMA16_logging:main',
        ],
    },
)
