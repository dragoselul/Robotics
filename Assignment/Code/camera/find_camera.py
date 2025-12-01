import cv2

def find_cameras(max_test=100):
    available_cameras = []
    for i in range(max_test):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Camera index {i} is available")
            available_cameras.append(i)
            cap.release()
    return available_cameras

cameras = find_cameras()
print("Available camera indices:", cameras)
