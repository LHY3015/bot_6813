from setuptools import find_packages, setup

package_name = 'bot_6813'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bot_6813_navigation.launch.py']),
    ],
    install_requires=['setuptools','launch', 'launch_ros'],

    zip_safe=True,
    maintainer='lhy',
    maintainer_email='186386042+LHY3015@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'initial_alignment = bot_6813.initial_alignment:main',
            'navigate = bot_6813.navigate:main',
        ],
    },
)