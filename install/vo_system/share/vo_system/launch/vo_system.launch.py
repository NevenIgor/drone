        Node(
            package='icm20948_driver',
            executable='icm20948_node',
            name='imu_node',
            output='screen',
            parameters=[
                {'i2c_bus': '/dev/i2c-1'},
                {'i2c_address': 0x68},  # ← Верно!
                {'frequency': 200.0}
            ]
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'pixel_format': 'raw_mjpeg',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'io_method': 'mmap'
            }]
        ),

        Node(
            package='msp_bridge',
            executable='msp_bridge_node',
            name='msp_bridge',
            output='screen',
            parameters=[{
                'uart_port': '/dev/ttyUSB0',
                'baudrate': 115200,
                'debug': False
            }]
        )
