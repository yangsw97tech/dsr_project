from setuptools import find_packages, setup

package_name = 'dsr_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/board_pipeline.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jy',
    maintainer_email='jy@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'erasing = dsr_project.erasing:main',
            'erasing_abs = dsr_project.erasing_abs:main',
            'eraser_pick_and_place = dsr_project.eraser_pick_and_place:main',
            'erasing_pausable = dsr_project.erasing_pausable:main',
            'eraser_stop = dsr_project.eraser_stop:main',
            'oiling = dsr_project.oiling:main',
            'emergency_stop_client = dsr_project.emergency_stop_client:main',
            'motion_control_node = dsr_project.motion_control_node:main',
            'Pyqt_pub = dsr_project.Pyqt_pub:main',
            'Pyqt_GUI = dsr_project.Pyqt_GUI:main',
            'firebase_publishing2 = dsr_project.firebase_publishing2:main',
            'move_basic = dsr_project.move_basic:main',
            'move_stop = dsr_project.move_stop:main',
            'erasing_action_server = dsr_project.erasing_action_server:main',
            'erasing_action_server_stop = dsr_project.erasing_action_server_stop:main',
            'erasing_action_client = dsr_project.erasing_action_client:main',

            'brushing_action_server = dsr_project.brushing_action_server:main',
            'brushing_action_server_multi = dsr_project.brushing_action_server_multi:main',
            'brushing_action_client = dsr_project.brushing_action_client:main',
            'action_client = dsr_project.action_client:main',
            'action_server = dsr_project.action_server:main',
            'oiling_action_client = dsr_project.oiling_action_client:main',
            'oiling_action_server = dsr_project.oiling_action_server:main',
            'main_node = dsr_project.main_node:main',
            'main_node_gem = dsr_project.main_node_gem:main',
            'Doma_a_pick_server = dsr_project.Doma_a_pick_server:main',
            'Doma_a_pick_client = dsr_project.Doma_a_pick_client:main',
            'Doma_a_place_server = dsr_project.Doma_a_place_server:main',
            'Doma_a_place_client = dsr_project.Doma_a_place_client:main',
            'cam_publisher = dsr_project.cam_publisher:main',
            'cam_subscriber = dsr_project.cam_subscriber:main',
            'test_gui = dsr_project.test_gui:main',
            'test_sub = dsr_project.test_sub:main',
            'test_pub = dsr_project.test_pub:main',
            
            


            'brushing_stop_server = dsr_project.brushing_stop_server:main',
            'doma_pick_stop_server = dsr_project.doma_pick_stop_server:main',
            'doma_place_stop_server = dsr_project.doma_place_stop_server:main',

            'oiling_stop_server = dsr_project.oiling_stop_server:main',

            'erasing_stop_server = dsr_project.erasing_stop_server:main',

            'stop_control = dsr_project.stop_control:main',

            'doma_pick_final_server = dsr_project.doma_pick_final_server:main',
            'doma_place_final_server = dsr_project.doma_place_final_server:main',
            'erasing_final_server = dsr_project.erasing_final_server:main',
            'brushing_final_server = dsr_project.brushing_final_server:main',
            'oiling_final_server = dsr_project.oiling_final_server:main',
            
            'doma_pick_final_client = dsr_project.doma_pick_final_client:main',
            'doma_place_final_client = dsr_project.doma_place_final_client:main',
            'erasing_final_client = dsr_project.erasing_final_client:main',
            'brushing_final_client = dsr_project.brushing_final_client:main',
            'oiling_final_client = dsr_project.oiling_final_client:main',


        ],
    },
)

