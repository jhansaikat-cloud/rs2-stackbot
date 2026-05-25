from setuptools import find_packages, setup

package_name = 'perception_pkg'

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
    maintainer='arjun',
    maintainer_email='arjun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'camera_viewer = perception_pkg.camera_viewer:main',
		'hsv_tuner = perception_pkg.hsv_tuner:main',
		'cube_detector  = perception_pkg.cube_detector:main',
		'pose_estimator  = perception_pkg.pose_estimator:main',
		'charuco_handeye_calibration = perception_pkg.charuco_handeye_calibration:main',
		'charuco_handeye_validate = perception_pkg.charuco_handeye_validate:main',
		'publish_handeye_tf = perception_pkg.publish_handeye_tf:main',
		'pose_estimator_v2 = perception_pkg.pose_estimator_v2:main',		
        ],
    },
)


