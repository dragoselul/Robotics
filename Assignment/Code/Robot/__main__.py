import cv2
import numpy as np
# Import your classes (adjust import paths as needed)
try:
    from .robot_controller import RobotKinematicsController 
    from .robot import Robot  # Your hardware class
except ImportError:  # fallback when running as a standalone script
    from robot_controller import RobotKinematicsController
    from robot import Robot

from Part1_Problem_3 import execute_circle_movement
# ... import other dependencies ...



def main():
    # 1. Initialize Robot
    # -------------------
    # Assuming 'Robot' is your hardware interface class
    hardware = Robot(device_name='/dev/tty.usbmodem14401', baudrate=1_000_000, dxl_ids=[1, 2, 3, 4]) 
    controller = RobotKinematicsController(hardware, verbose=True)
    controller.initialize()
    
    # Load your calibration so we know the Z-height!
    # (Or hardcode it if you know it, e.g., 0.122 from your previous logs)
    controller.load_calibration('calibration.npy')
    # Get keyboard surface height from calibration
    keyboard_z = controller.workspace['z_height']  # 0.0895m

    keyboard_center = controller.workspace['center']  # [0.028, -0.026, 0.089]

    # Define a target key position (e.g., center of keyboard)
    target_key = [

        keyboard_center[0]+0.030,      # x: 0.028m
        keyboard_center[1]-0.030,      # y: -0.026m  
        keyboard_z-0.0               # z: 0.089m (surface height)
    ]

    # Move directly using robot base coordinates
    controller.move_to_position_smooth(
        target_key[0], 
        target_key[1], 
        target_key[2]
    )

    execute_circle_movement(controller=controller, center=[0.150, 0, 0.120], radius=0.032)

    # # 2. Setup Camera
    # # -------------------
    # # cap = cv2.VideoCapture(2) # Adjust ID if you have multiple cams
    
    # # Global variables for mouse callback
    # click_point = None

    # def mouse_callback(event, x, y, flags, param):
    #     nonlocal click_point
    #     if event == cv2.EVENT_LBUTTONDOWN:
    #         click_point = (x, y)
    #         print(f"Clicked at pixel: {click_point}")

    # cv2.namedWindow("Robot Vision")
    # cv2.setMouseCallback("Robot Vision", mouse_callback)

    # print(">>> CLICK on a detected key to move the robot there. Press 'ESC' to quit.")

    # # 3. Main Vision Loop
    # # -------------------
    # while True:
    #     ret, frame = cap.read()
    #     if not ret:
    #         break

    #     # --- Your Detection Code ---
    #     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #     blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    #     edges = cv2.Canny(blurred, 50, 150)
    #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #     for cnt in contours:
    #         approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
    #         if len(approx) == 4:
    #             # Draw the key
    #             cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
                
    #             # Optional: Calculate Center of the key for auto-targeting
    #             M = cv2.moments(cnt)
    #             if M["m00"] != 0:
    #                 cX = int(M["m10"] / M["m00"])
    #                 cY = int(M["m01"] / M["m00"])
    #                 cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

    #     cv2.imshow("Robot Vision", frame)

    #     # --- Handle User Input ---
        
    #     # Check if user clicked a point
    #     if click_point is not None:
    #         u, v = click_point
            
    #         # CALL THE ROBOT MOVE FUNCTION
    #         print(f"Commanding Robot to move to pixel ({u}, {v})...")
    #         controller.move_to_pixel(u, v, keyboard_z_height=keyboard_z)
            
    #         # Reset click
    #         click_point = None

    #     # Exit on ESC
    #     if cv2.waitKey(1) & 0xFF == 27:
    #         break

    # # Cleanup
    # cap.release()
    # cv2.destroyAllWindows()
    controller.close()

if __name__ == "__main__":
    main()
