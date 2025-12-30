from setuptools import find_packages, setup

package_name = 'tiago_box_check'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # ✅ ROS 패키지 setup.py에서는 파이썬 패키지 설치를 최소화하는 게 안정적입니다.
    #    (torch/numpy/opencv 꼬임 방지)
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TIAGO perception: YOLO 2D box detection + QR checker',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'qr_checker = tiago_box_check.nodes.qr_checker:main',
            'yolo_box_detector = tiago_box_check.nodes.yolo_box_detector_node:main',
            'yolo_overlay = tiago_box_check.nodes.yolo_overlay_node:main',
        ],
    },
)
