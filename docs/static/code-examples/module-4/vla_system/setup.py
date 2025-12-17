from setuptools import setup
import os
from glob import glob

package_name = 'vla_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'prompts'), glob('prompts/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@robotics.edu',
    description='Vision-Language-Action system for humanoid robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_node = vla_system.whisper_transcription_node:main',
            'llm_planner = vla_system.llm_planner_node:main',
            'action_executor = vla_system.action_executor:main',
            'grounding_node = vla_system.visual_grounding_node:main',
        ],
    },
)
