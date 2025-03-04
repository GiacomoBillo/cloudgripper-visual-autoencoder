import cv2
from client.cloudgripper_client import GripperRobot
import os
import time

# if x_norm and y_norm are not normalized
def normalize(states):
    x_norm_min = 1.1427142857142856
    x_norm_max = 2.1424285714285713
    y_norm_min = -1.1427142857142856
    y_norm_max = -0.14285714285714285

    states["x_norm"] = abs(round((states["x_norm"] - x_norm_min) / (x_norm_max - x_norm_min),2))
    states["y_norm"] = abs(round((states["y_norm"] - y_norm_min) / (y_norm_max - y_norm_min),2))


def check_normalization_state(state):
    for key in ["x_norm", "y_norm", "z_norm", "rotation", "claw_norm"]:
        state[key] = max(state[key],0)
        if key=="rotation":
            state[key] = min(state[key], 180)
        else:
            state[key] = min(state[key], 1)


token = os.environ['CLOUDGRIPPER_TOKEN']
robot = GripperRobot('robot10', token)

image1, image2, states, timestamp = robot.get_all_states()
height = min(image1.shape[0], image2.shape[0])
image1_resized = cv2.resize(image1, (int(image1.shape[1] * height / image1.shape[0]), height))
image2_resized = cv2.resize(image2, (int(image2.shape[1] * height / image2.shape[0]), height))
concatenated_image = cv2.hconcat([image1_resized, image2_resized])
cv2.imshow("Cloudgripper cameras stream", concatenated_image)
print(states)
angle = 0

while True:

    image1, image2, states, timestamp = robot.get_all_states()
    height = min(image1.shape[0], image2.shape[0])
    image1_resized = cv2.resize(image1, (int(image1.shape[1] * height / image1.shape[0]), height))
    image2_resized = cv2.resize(image2, (int(image2.shape[1] * height / image2.shape[0]), height))
    # Concatenate the images horizontally
    concatenated_image = cv2.hconcat([image1_resized, image2_resized])
    cv2.imshow("Cloudgripper cameras stream", concatenated_image)
    
    command = cv2.waitKey(1) & 0xFF
    if command != 255:
        if command == ord('q'):
            break 
        if command == ord('w'):
            robot.step_forward()
        elif command == ord('s'):
            robot.step_backward()
        elif command == ord('d'):
            robot.step_right()
        elif command == ord('a'):
            robot.step_left()

        elif command == ord('r'):
            angle += 30
            robot.rotate(angle)
        elif command == ord('t'):
            robot.rotate(0)
        
        elif command == ord('o'):
            robot.move_gripper(0)
        elif command == ord('i'):
            robot.move_gripper(1)
        elif command == ord('u'):
            robot.move_gripper(0.5)

        elif command == ord('z'):
            robot.move_z(states["z_norm"]+0.1)
        elif command == ord('x'):
            robot.move_z(states["z_norm"]-0.1)

        elif command == ord('1'):
            robot.move_xy(0.0,0.0)
        elif command == ord('2'):
            robot.move_xy(1.0,0.0)
        elif command == ord('3'):
            robot.move_xy(0.0,1.0)
        elif command == ord('4'):
            robot.move_xy(1.0,1.0)
        elif command == ord('5'):
            robot.move_xy(0.5, 0.5)
        elif command == ord('6'):
            robot.move_xy(0.9, 0.9)
        time.sleep(2.5)
        
        states, timestamp = robot.get_state()
        # normalize(states)
        print(f"x_norm={states['x_norm']}\n"
          f"y_norm={states['y_norm']}\n"
          f"z_norm={states['z_norm']}\n"
          f"rotation={states['rotation']}\n"
          f"claw_norm={states['claw_norm']}\n\n")
        

