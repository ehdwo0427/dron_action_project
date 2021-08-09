#!/usr/bin/env python3

"""
This example shows how to use the manual controls plugin.

Note: Manual inputs are taken from a test set in this example to decrease complexity. Manual inputs
can be received from devices such as a joystick using third-party python extensions

Note: Taking off the drone is not necessary before enabling manual inputs. It is acceptable to send
positive throttle input to leave the ground. Takeoff is used in this example to decrease complexity
"""

import sys

# sys.path.insert(1, '../pyKinectAzure/')
sys.path.insert(1, '/home/ncl/pyKinectAzure/pyKinectAzure/')

import asyncio
import random
from mavsdk import System
import numpy as np
from pyKinectAzure import pyKinectAzure, _k4a
from kinectBodyTracker import kinectBodyTracker, _k4abt
import cv2
import time
from multiprocessing import Process, Queue
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Path to the module
# TODO: Modify with the path containing the k4a.dll from the Azure Kinect SDK
# modulePath = 'C:\\Program Files\\Azure Kinect SDK v1.4.1\\sdk\\windows-desktop\\amd64\\release\\bin\\k4a.dll'
modulePath = r'/usr/lib/x86_64-linux-gnu/libk4a.so'
# bodyTrackingModulePath = 'C:\\Program Files\\Azure Kinect Body Tracking SDK\\sdk\\windows-desktop\\amd64\\release\\bin\\k4abt.dll'
bodyTrackingModulePath = r'/usr/lib/libk4abt.so'
# under x86_64 linux please use r'/usr/lib/x86_64-linux-gnu/libk4a.so'
# In Jetson please use r'/usr/lib/aarch64-linux-gnu/libk4a.so'


# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]
manual_inputs = [
    [0, 0, 0.5, 0],  # no movement
    [-1, 0, 0.5, 0],  # minimum roll
    [1, 0, 0.5, 0],  # maximum roll
    [0, -1, 0.5, 0],  # minimum pitch
    [0, 1, 0.5, 0],  # maximum pitch
    [0, 0, 0.5, -1],  # minimum yaw
    [0, 0, 0.5, 1],  # maximum yaw
    [0, 0, 1, 0],  # max throttle
    [0, 0, 0, 0],  # minimum throttle
]


def plot_func3(q):
    fig, axs = plt.subplots(3)
    values = [[]] * 4  # [t, x, y, z]
    for i in range(4):
        values[i] = deque([], 10)

    def animate(i, q, values):
        while not q.empty():
            #            values.append(q.get())
            val = q.get()
            for i in range(4):
                values[i].append(val[i])

        #        print('plot:', ts)

        for i in range(3):
            axs[i].clear()
            axs[i].plot(values[0], values[i + 1], 'o-')
            axs[i].set_ylim(-1000, 1000)
            axs[i].set_yscale('symlog')

        axs[0].set_ylabel('x')
        axs[0].set_xticks([])

        axs[1].set_ylabel('y')
        axs[1].set_xticks([])

        axs[2].set_ylabel('z')
        axs[2].set_xlabel('t (sec)')

    ani = animation.FuncAnimation(fig, animate, fargs=(q, values), interval=100)
    plt.show()


def calc_angle(A, B, C):  # calculate angle ABC
    A = np.array([A.x, A.y, A.z])
    #       A = np.array([A[0], A[1], A[2]])
    B = np.array([B.x, B.y, B.z])
    C = np.array([C.x, C.y, C.z])

    x1, y1, z1 = A - B
    x2, y2, z2 = C - B
    norm1 = np.sqrt(x1 * x1 + y1 * y1 + z1 * z1)
    norm2 = np.sqrt(x2 * x2 + y2 * y2 + z2 * z2)
    result = (x1 * x2 + y1 * y2 + z1 * z2) / norm1 / norm2
    return np.rad2deg(np.arccos(result))


async def manual_controls():
    """Main function to connect to the drone and input manual controls"""
    # Connect to the Simulation
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # This waits till a mavlink based drone is connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone with UUID:")
            break

    # Checking if Global Position Estimate is ok
    async for global_lock in drone.telemetry.health():
        if global_lock.is_global_position_ok:
            print("-- Global position state is ok")
            break

    # set the manual control input after arming
     #  ldj
    await drone.manual_control.set_manual_control_input(
        float(0), float(0), float(0.5), float(0)
    )

    # Arming the drone
    print("-- Arming")
    await drone.action.arm()

    # Takeoff the vehicle
    #print("-- Taking off")
    #await drone.action.takeoff()  # ldj
    #await asyncio.sleep(5)  # ldj

    # start manual control
    #print("-- Starting manual control")
    #await drone.manual_control.start_position_control()

    # skeleton code start ldj
    pyK4A = pyKinectAzure(modulePath)

    # Open device
    pyK4A.device_open()

    # Modify camera configuartion
    device_config = pyK4A.config
    device_config = pyK4A.config
    #   device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_OFF
    device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_720P
    device_config.depth_mode = _k4a.K4A_DEPTH_MODE_NFOV_UNBINNED
    #   device_config.camera_fps = _k4a.K4A_FRAMES_PER_SECOND_30
    device_config.camera_fps = _k4a.K4A_FRAMES_PER_SECOND_15
    #   device_config.camera_fps = _k4a.K4A_FRAMES_PER_SECOND_5
    print(device_config)
    #   exit()

    # Start cameras using modified configuration
    pyK4A.device_start_cameras(device_config)

    # Initialize the body tracker
    pyK4A.bodyTracker_start(bodyTrackingModulePath)
    #   exit()

    #   q = Queue()
    #   plot = Process(target=plot_func3, args=(q,))
    #   plot.start()

    start_time = pre_cap_time = time.time()

    k = 0
    flag = True

    while True:
        # Get capture
        pyK4A.device_get_capture()

        cur_cap_time = time.time()
        #	print('cap interval(sec):', cur_cap_time - pre_cap_time)
        pre_cap_time = cur_cap_time

        # Get the depth image from the capture
        depth_image_handle = pyK4A.capture_get_depth_image()
        color_image_handle = pyK4A.capture_get_color_image()


        # Check the image has been read correctly
        if depth_image_handle and color_image_handle:
            #	if color_image_handle:
            # Perform body detection
            pyK4A.bodyTracker_update()
            # enqueue_capture(self.capture_handle)
            # detectBodies()

            #		==> body_tracker.segmented_body_img, body_tracker.bodiesNow
            #               track_update_time = time.time()
            #               print('track update time(sec):', track_update_time - cur_cap_time)

            # Read and convert the image data to numpy array:
            color_image = pyK4A.image_convert_to_numpy(color_image_handle)

            #               transformed_depth_image = pyK4A.transform_depth_to_color(depth_image_handle, color_image_handle)
            #               print(type(depth_image_handle), type(pyK4A.body_tracker.segmented_body_img))

            #               if pyK4A.body_tracker.segmented_body_img:
            #               transformed_body_image = pyK4A.transform_body_to_color(depth_image_handle, color_image_handle, pyK4A.body_tracker.segmented_body_img)
            #               print(transformed_body_image.shape)
            #               print(type(color_image_handle), type(color_image), type(pyK4A.body_tracker.segmented_body_img))
            #               print(pyK4A.image_get_format(color_image_handle), pyK4A.image_get_format(depth_image_handle), pyK4A.image_get_format(pyK4A.body_tracker.segmented_body_img))
            # 0, 4, 6
            # 0: K4A_IMAGE_FORMAT_COLOR_MJPG
            # 4: K4A_IMAGE_FORMAT_DEPTH16
            # 6: K4A_IMAGE_FORMAT_CUSTOM8
            #               exit()

            depth_image = pyK4A.image_convert_to_numpy(depth_image_handle)
            depth_color_image = cv2.convertScaleAbs(depth_image,
                                                    alpha=0.05)  # alpha is fitted by visual comparison with Azure k4aviewer results
            depth_color_image = cv2.cvtColor(depth_color_image, cv2.COLOR_GRAY2RGB)

            # Get body segmentation image
            #               body_image_color = pyK4A.bodyTracker_get_body_segmentation()
            #
            # self.body_tracker.segmented_body_img ==> body segmentation image
            # k4a_image_t ==> numpy
            #

            #               color_image = cv2.resize(color_image, (640, 576))
            #               print(color_image.shape, depth_image.shape, transformed_depth_image.shape, body_image_color.shape)
            #               (720, 1280, 3) (576, 640) (720, 1280) (576, 640, 3)

            #                combined_image = cv2.addWeighted(depth_color_image, 0.8, body_image_color, 0.2, 0)

####
            for i, body in enumerate(pyK4A.body_tracker.bodiesNow):
                neck = body.skeleton.joints[3]
                if i == 0:
                    min_idx = 0
                    min_neck = neck
                else:
                    if neck.position.v[2] < min_neck.position.v[2]:
                        min_idx = i
                        min_neck = neck

            body = pyK4A.body_tracker.bodiesNow[min_idx]

            left_wrist = body.skeleton.joints[7]
            right_wrist = body.skeleton.joints[14]
            head = body.skeleton.joints[26]
            right_shoulder = body.skeleton.joints[12]
            right_elbow = body.skeleton.joints[13]
            neck = body.skeleton.joints[3]
            spine_navel = body.skeleton.joints[1]
            hand_left = body.skeleton.joints[8]
            handtip_left = body.skeleton.joints[9]
            handtip_right = body.skeleton.joints[16]
            elbow_left = body.skeleton.joints[6]

            input_index = 0

            if flag != False:
                if handtip_left.position.v[1] < spine_navel.position.v[1] and handtip_right.position.v[1] < \
                        spine_navel.position.v[1]:
                    print('Start Control')
                    flag = False
                    print('Take 0ff')
                    await drone.action.takeoff()
                    await asyncio.sleep(5)  # need to next takeoff code

                    print("-- Starting manual control --")
                    await drone.manual_control.start_position_control()

            else:
                move_hand_time = time.time()
                if handtip_left.position.v[1] > spine_navel.position.v[1] and handtip_right.position.v[1] > \
                        spine_navel.position.v[1]:
                    flag = True
                    await drone.action.land()
                    print('Drone Land')
                    await asyncio.sleep(5)

                if hand_left.confidence_level >= 2:
                    if hand_left.position.v[0] < spine_navel.position.v[0]:
                        input_index = 4
                        print(cur_cap_time - start_time, ': go right!')
                        #print(cur_cap_time - start_time, ': go right!', move_hand_time - cur_cap_time)

                    if handtip_left.position.v[0] > elbow_left.position.v[0]:
                        input_index = 3
                        print(cur_cap_time - start_time, ': go left!')
                        #print(cur_cap_time - start_time, ': go left!', move_hand_time - cur_cap_time)

                    if left_wrist.position.v[1] < neck.position.v[1]:
                        input_index = 2
                        print(cur_cap_time - start_time, ': go straight!')
                        #print(cur_cap_time - start_time, ': go straight!', move_hand_time - cur_cap_time)

                    if left_wrist.position.v[1] > spine_navel.position.v[1]:
                        input_index = 1
                        print(cur_cap_time - start_time, ': go back!')
                        #print(cur_cap_time - start_time, ': go back!', move_hand_time - cur_cap_time)

                    if handtip_right.position.v[0] > spine_navel.position.v[0]:
                        input_index = 5
                        print(cur_cap_time - start_time, ': pin left!')
                        #print(cur_cap_time - start_time, ': pin left!', move_hand_time - cur_cap_time)

                if right_wrist.position.v[1] < neck.position.v[1]:
                    input_index = 7
                    print(cur_cap_time - start_time, ': go up!')
                    #print(cur_cap_time - start_time, ': go up!', move_hand_time - cur_cap_time)
                #  if handtip_right.position.v[0] > spine_navel.position.v[0]:
                #           input_index = 3
                #           print(cur_cap_time - start_time, ':go left!')
                if right_wrist.position.v[1] > spine_navel.position.v[1]:
                    input_index = 8
                    print(cur_cap_time - start_time, ': go down!')
                    #print(cur_cap_time - start_time, ': go down!', move_hand_time - cur_cap_time)
                if handtip_right.position.v[0] < right_elbow.position.v[0]:
                    input_index = 6
                    print(cur_cap_time - start_time, ': pin right!')
                    #print(cur_cap_time - start_time, ': pin right!', move_hand_time - cur_cap_time)
                #                                   print(cur_cap_time - start_time, ': right wrist is lower than navel!')

                #                                   if right_shoulder.confidence_level >= 2 and right_elbow.confidence_level >= 2:
                #                                              right_arm_angle = calc_angle(right_shoulder.position.v, right_elbow.position.v, right_wrist.position.v)
                #                                              right_arm_angle = calc_angle(right_shoulder.position.xyz, right_elbow.position.xyz, right_wrist.position.xyz)
                #                                              print(cur_cap_time - start_time, ': angle of right arm is', right_arm_angle)

                # grabs a random input from the test list
                # WARNING - your simulation vehicle may crash if its unlucky enough
                input_list = manual_inputs[input_index]

                # get current state of roll axis (between -1 and 1)
                roll = float(input_list[0])
                # get current state of pitch axis (between -1 and 1)
                pitch = float(input_list[1])
                # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
                throttle = float(input_list[2])
                # get current state of yaw axis (between -1 and 1)
                yaw = float(input_list[3])

                await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
                # print time deley
                move_drone_time = time.time()
                if input_index != 0:
                    print(move_drone_time - move_hand_time, ' : move drone delay')
                await asyncio.sleep(0.1)

            ####
            skeleton2D_onColor = pyK4A.bodyTracker_project_skeleton(body.skeleton,
                                                                    dest_camera=_k4a.K4A_CALIBRATION_TYPE_COLOR)
            color_image = pyK4A.body_tracker.draw2DSkeleton(skeleton2D_onColor, body.id, color_image)


            #                       cv2.imshow('Segmented Depth Image', combined_image)
            #                       cv2.imshow('Segmented Depth Image', depth_color_image)

            color_image = color_image[:, 150:-150, :]
            color_image = cv2.resize(color_image, None, fx=0.5, fy=0.5)
            #                       print(color_image.shape)
            #                       (360, 490, 3)

            cv2.imshow('Segmented Color Image', color_image)
            #                       cv2.imshow('Segmented Body Image', body_image_color)
            k = cv2.waitKey(1)

            # Release the image
            pyK4A.image_release(depth_image_handle)
            pyK4A.image_release(color_image_handle)
            pyK4A.image_release(pyK4A.body_tracker.segmented_body_img)

            pyK4A.body_tracker.release_frame()
            #q
            # k4abt_tracker_pop_result() 함수에서 body_frame_handle 사용.
            # Once body_frame data is read, the user must call k4abt_frame_release() to return the allocated memory to the SDK.
            #

        pyK4A.capture_release()

        if k == 27:  # Esc key to stop
            break
        elif k == ord('q'):
            cv2.imwrite('outputImage.jpg', combined_image)

    pyK4A.device_stop_cameras()
    pyK4A.device_close()

    '''
                for body in pyK4A.body_tracker.bodiesNow:
                    left_wrist = body.skeleton.joints[7]
                    right_wrist = body.skeleton.joints[14]
                    head = body.skeleton.joints[26]
                    right_shoulder = body.skeleton.joints[12]
                    right_elbow = body.skeleton.joints[13]
                    neck = body.skeleton.joints[3]
                    spine_navel = body.skeleton.joints[1]
                    hand_left = body.skeleton.joints[8]
                    handtip_left = body.skeleton.joints[9]
                    handtip_right = body.skeleton.joints[16]
                    elbow_left = body.skeleton.joints[6]

                    input_index = 0
                    if neck.position.v[2] < 1000 and neck.position.v[2] > 400:
                        if flag != False:
                            if handtip_left.position.v[1] < spine_navel.position.v[1] and handtip_right.position.v[1] < spine_navel.position.v[1]:
                                print('Start Control')
                                flag = False
                                print('Take 0ff')
                                await drone.action.takeoff()
                                await asyncio.sleep(5)      # need to next takeoff code

                                print("-- Starting manual control --")
                                await drone.manual_control.start_position_control()

                        else:
                            move_hand_time = time.time()
                            if handtip_left.position.v[1] > spine_navel.position.v[1] and handtip_right.position.v[1] > \
                                    spine_navel.position.v[1]:
                                flag = True
                                await drone.action.land()
                                print('Drone Land')
                                await asyncio.sleep(5)

                            if hand_left.confidence_level >= 2:
                                if hand_left.position.v[0] < spine_navel.position.v[0]:
                                    input_index = 4
                                    print(cur_cap_time - start_time, ': go right!', move_hand_time - cur_cap_time)

                                if handtip_left.position.v[0] > elbow_left.position.v[0]:
                                    input_index = 3
                                    print(cur_cap_time - start_time, ': go left!', move_hand_time - cur_cap_time)

                                if left_wrist.position.v[1] < neck.position.v[1]:
                                    input_index = 2
                                    print(cur_cap_time - start_time, ': go straight!', move_hand_time - cur_cap_time)

                                if left_wrist.position.v[1] > spine_navel.position.v[1]:
                                    input_index = 1
                                    print(cur_cap_time - start_time, ': go back!', move_hand_time - cur_cap_time)

                                if handtip_right.position.v[0] > spine_navel.position.v[0]:
                                    input_index = 5
                                    print(cur_cap_time - start_time, ': pin left!', move_hand_time - cur_cap_time)

                            if right_wrist.position.v[1] < neck.position.v[1]:
                                input_index = 7
                                print(cur_cap_time - start_time, ': go up!', move_hand_time - cur_cap_time)
                            #  if handtip_right.position.v[0] > spine_navel.position.v[0]:
                            #           input_index = 3
                            #           print(cur_cap_time - start_time, ':go left!')
                            if right_wrist.position.v[1] > spine_navel.position.v[1]:
                                input_index = 8
                                print(cur_cap_time - start_time, ': go down!',move_hand_time - cur_cap_time)
                            if handtip_right.position.v[0] < right_elbow.position.v[0]:
                                input_index = 6
                                print(cur_cap_time - start_time, ': pin right!', move_hand_time - cur_cap_time)
                            #                                   print(cur_cap_time - start_time, ': right wrist is lower than navel!')

                            #                                   if right_shoulder.confidence_level >= 2 and right_elbow.confidence_level >= 2:
                            #                                              right_arm_angle = calc_angle(right_shoulder.position.v, right_elbow.position.v, right_wrist.position.v)
                            #                                              right_arm_angle = calc_angle(right_shoulder.position.xyz, right_elbow.position.xyz, right_wrist.position.xyz)
                            #                                              print(cur_cap_time - start_time, ': angle of right arm is', right_arm_angle)

                            # grabs a random input from the test list
                            # WARNING - your simulation vehicle may crash if its unlucky enough
                            input_list = manual_inputs[input_index]

                            # get current state of roll axis (between -1 and 1)
                            roll = float(input_list[0])
                            # get current state of pitch axis (between -1 and 1)
                            pitch = float(input_list[1])
                            # get current state of throttle axis (between -1 and 1, but between 0 and 1 is expected)
                            throttle = float(input_list[2])
                            # get current state of yaw axis (between -1 and 1)
                            yaw = float(input_list[3])

                            await drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
                            # print time deley
                            move_drone_time = time.time()
                            if input_index != 0:
                                print(move_drone_time - move_hand_time, ' : move drone delay')
                            await asyncio.sleep(0.1)




                    # skeleton2D_onColor = pyK4A.bodyTracker_project_skeleton_on_color(body.skeleton)
                    skeleton2D_onColor = pyK4A.bodyTracker_project_skeleton(body.skeleton,
                                                                            dest_camera=_k4a.K4A_CALIBRATION_TYPE_COLOR)
                    color_image = pyK4A.body_tracker.draw2DSkeleton(skeleton2D_onColor, body.id, color_image)

                    # Overlay body segmentation on depth image
    '''

#
if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(manual_controls())
