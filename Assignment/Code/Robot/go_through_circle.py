import cv2
import numpy as np
import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from robot import Robot
from robot_controller import RobotKinematicsController

# ==================================================================================
# CONFIGURATION
# ==================================================================================

# Vision Param (Meters) - CRITICAL: Must be accurate!
REAL_RADIUS = 0.035       

# Movement Params (Millimeters)
APPROACH_DISTANCE_MM = 100.0 
THROUGH_DISTANCE_MM = 100.0   

CAMERA_INDEX = 0

def detect_circle_in_frame(frame):
    """Detects the largest circle in the frame."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        param1=50, param2=30, minRadius=10, maxRadius=200
    )
    if circles is not None:
        circles = np.uint16(np.around(circles))
        largest_circle = max(circles[0, :], key=lambda c: c[2])
        return largest_circle
    return None
  
def capture_circle_data(cap, prompt_text="Capture"):
    print(f"--- {prompt_text} ---")
    print("Press 'SPACE' to capture, 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret: break
        detected = detect_circle_in_frame(frame)
        display_frame = frame.copy()
        if detected is not None:
            cx, cy, r = detected
            cv2.circle(display_frame, (cx, cy), 1, (0, 100, 100), 3)
            cv2.circle(display_frame, (cx, cy), r, (255, 0, 255), 3)
            circle_data = detected
            cv2.putText(display_frame, "LOCKED", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
             cv2.putText(display_frame, "SEARCHING...", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Robot Vision", display_frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord(' '):
            if circle_data is not None:
                return circle_data
        elif key & 0xFF == ord('q'):
            return None
    return None

def main():
    print("Initializing Robot...")
    robot = Robot(device_name='/dev/ttyACM0', baudrate=1_000_000, dxl_ids=[1, 2, 3, 4])
    controller = RobotKinematicsController(robot)
    controller.initialize()
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    try:
        # -------------------------------------------------------------------------
        # STEP 1: OBSERVE
        # -------------------------------------------------------------------------
        print("\n1. Moving to Observation Pose...")
        controller.move_to_position(140, 0, 125, [1, 0, 0]) 
        time.sleep(2.0)
        
        # Capture Circle
        data = capture_circle_data(cap, "Align Circle")
        if data is None: return 
        
        u, v, r_pix = data
        current_joints = controller.current_joint_angles

        # -------------------------------------------------------------------------
        # STEP 2: CALCULATE TARGET (Direct Single-View)
        # -------------------------------------------------------------------------
        print("\n2. Calculating Target...")
        
        # This returns [X, Y, Z] in MILLIMETERS
        target_mm = controller.vision.get_object_pos_from_camera(
            u, v, r_pix, REAL_RADIUS, current_joints
        )
        
        print(f"TARGET DETECTED AT (MM): {target_mm}")
        print(f"ROBOT CURRENT POS (MM):  {controller.get_end_effector_pose()[:3]}")
        
        # Sanity Check: Is the target ridiculously close to the base?
        # A target X < 150mm means it's basically inside/behind the gripper.
        if target_mm[0] < 150.0:
            print("WARNING: Target is suspiciously close to base.")
            print("   -> Check R_cam_tool matrix (Are Z and X swapped?)")
            print("   -> Check REAL_RADIUS (Is it accurate?)")

        # -------------------------------------------------------------------------
        # STEP 3: EXECUTE POKE
        # -------------------------------------------------------------------------
        
        # Calculate Approach Vector (Direction from Base to Target)
        # We ignore Z for the orientation vector (keep tool horizontal)
        target_x, target_y, _ = target_mm
        theta_yaw = np.arctan2(target_y, target_x)
        approach_dir = np.array([np.cos(theta_yaw), np.sin(theta_yaw), 0.0])
        
        print(f"Approach Direction: {approach_dir}")
        
        # Calculate Waypoints (in MM)
        approach_pos = target_mm - (APPROACH_DISTANCE_MM * approach_dir)
        through_pos  = target_mm + (THROUGH_DISTANCE_MM * approach_dir)
        
        print(f"Moving to Approach: {approach_pos}")
        
        # 1. Approach
        controller.move_to_position_smooth(
            approach_pos[0], approach_pos[1], approach_pos[2], 
            orientation=approach_dir
        )
        time.sleep(1.0)
        
        # 2. Poke Through
        print("POKING THROUGH!...")
        controller.move_to_position_smooth( 
            through_pos[0], through_pos[1], through_pos[2], 
            orientation=approach_dir
        )
        time.sleep(1.0)
        
        # 3. Retract
        print("Retracting...")
        controller.move_to_position_smooth(
            approach_pos[0], approach_pos[1], approach_pos[2], 
            orientation=approach_dir
        )
        
        # Go Home
        controller.move_to_position(140, 0, 125, [1, 0, 0])

    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        cap.release()
        cv2.destroyAllWindows()
        controller.close()
           
if __name__ == "__main__":
    main()