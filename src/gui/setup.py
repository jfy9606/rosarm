from setuptools import setup, find_packages

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'gui_python'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gui.launch.py']),
        ('lib/' + package_name, ['scripts/tkgui_main.py', 'scripts/joint_state_publisher.py', 'scripts/test_tkinter.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='GUI interface for robotic arm control using Tkinter',
    license='TODO',
    tests_require=['pytest'],
    python_requires='>=3.11',  # 指定Python 3.11+
    classifiers=[
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.11',
    ],
    entry_points={
        'console_scripts': [
            'tkgui_main = gui.scripts.tkgui_main:main',
            'joint_state_publisher = gui.scripts.joint_state_publisher:main',
            'test_tkinter = gui.scripts.test_tkinter:main',
        ],
    },
) 