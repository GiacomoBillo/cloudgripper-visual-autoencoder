import cv2
from client.cloudgripper_client import GripperRobot
import os
import time

token = os.environ['CLOUDGRIPPER_TOKEN']
robot = GripperRobot('robot10', token)

image1, image2, states, timestamp = robot.get_all_states()
height = min(image1.shape[0], image2.shape[0])
image1_resized = cv2.resize(image1, (int(image1.shape[1] * height / image1.shape[0]), height))
image2_resized = cv2.resize(image2, (int(image2.shape[1] * height / image2.shape[0]), height))
#image1, timestamp = robot.get_image_base()
#cv2.imshow("Cloudgripper base camera stream", image)
#image2, timestamp = robot.get_image_top()
#cv2.imshow("Cloudgripper top camera stream", image)
# Concatenate the images horizontally
concatenated_image = cv2.hconcat([image1_resized, image2_resized])
cv2.imshow("Cloudgripper cameras stream", concatenated_image)
print(states)
angle = 0

while True:
    
    command = cv2.waitKey(1) & 0xFF
    if command != 255:
        if command == ord('q'):
            break 
        if command == ord('8') or command == ord('w'):
            robot.step_forward()
        elif command == ord('2') or command == ord('s'):
            robot.step_backward()
        elif command == ord('6') or command == ord('d'):
            robot.step_right()
        elif command == ord('4') or command == ord('a'):
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
        time.sleep(1)
        
        image1, image2, states, timestamp = robot.get_all_states()
        height = min(image1.shape[0], image2.shape[0])
        image1_resized = cv2.resize(image1, (int(image1.shape[1] * height / image1.shape[0]), height))
        image2_resized = cv2.resize(image2, (int(image2.shape[1] * height / image2.shape[0]), height))
        # Concatenate the images horizontally
        concatenated_image = cv2.hconcat([image1_resized, image2_resized])
        cv2.imshow("Cloudgripper cameras stream", concatenated_image)
        
        print(f"x_norm={states['x_norm']}\n"
          f"y_norm={states['y_norm']}\n"
          f"z_norm={states['z_norm']}\n"
          f"rotation={states['rotation']}\n"
          f"claw_norm={states['claw_norm']}\n\n")