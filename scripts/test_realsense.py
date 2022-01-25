#!/usr/bin/env python3
import pyrealsense2 as rs
import cv2
import sys
import numpy as np
import rospy

# in another ROS package
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# in this ROS package
from cal import get_custom_intrinsics
from rs_util import get_intrinsics


use_custom_intrinsics = True
vis_opencv = True
publish_ros = True
camera_lookup = {'032622074588':'A', '028522072401':'B', '032622073024': 'C'}


def step(pipeline, publisher):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        print('didnt get color frames ')
        return

    # convert image to numpy array
    color_image = np.asarray(color_frame.get_data())
    # convert to grayscale
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # publish image
    if publish_ros:
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(color_image, encoding="passthrough")
        publisher.publish(image_message)

    # show image
    if vis_opencv:
        cv2.imshow('Camera View', color_image)
        cv2.waitKey(1)


def configure_camera(serial_number):
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    # And get the device info
    print(f'Connected to {serial_number}')

    # get the camera intrinsics
    if use_custom_intrinsics:
        print('Using custom intrinsics from camera.')
        intrinsics = get_custom_intrinsics(camera_lookup[serial_number])
    else:
        print('Using default intrinsics from camera.')
        intrinsics = get_intrinsics(pipeline_profile)

    publisher = None
    if publish_ros:
        publisher = rospy.Publisher('camera_%s' % camera_lookup[serial_number],
                                    Image,
                                    queue_size=10)

    return pipeline, publisher


def main():
    print('Listing available realsense devices...')

    serial_numbers = []
    for i, device in enumerate(rs.context().devices):
        serial_number = device.get_info(rs.camera_info.serial_number)
        serial_numbers.append(serial_number)
        print(f'{i+1}. {serial_number}')

    pipelines, publishers = [], []
    for serial_number in serial_numbers:
        pipeline, publisher = configure_camera(serial_number)
        pipelines.append(pipeline)
        publishers.append(publisher)

    if len(pipelines) == 0:
        print('Could not find any Realsense Devices.')
        sys.exit(1)

    if publish_ros:
        rospy.init_node('publish_images')

    # loop and get camera data
    try:
        if publish_ros:
            while not rospy.is_shutdown():
                for pipeline, publisher in zip(pipelines, publishers):
                    step(pipeline, publisher)
        else:
            while True:
                for pipeline, publisher in zip(pipelines, publishers):
                    step(pipeline, publisher)

    finally:
        # Stop streaming
        for pipeline in pipelines:
            pipeline.stop()


if __name__ == '__main__':
    main()
