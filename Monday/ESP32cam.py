"""import cv2

# ESP32CAM URL
URL = "http://192.168.215.89"
AWB = True

# Video capture setup
cap = cv2.VideoCapture(URL + ":81/stream")

if __name__ == '__main__':
    while True:
        if cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                continue

            # Show the frame
            cv2.imshow("ESP32CAM Stream", frame)

            key = cv2.waitKey(1)
            if key == 27:  # Press ESC to exit
                break

    cap.release()
    cv2.destroyAllWindows()
"""


import cv2

# ESP32CAM URL
URL = "http://192.168.215.89"
AWB = True

# Video capture setup
cap = cv2.VideoCapture(URL + ":81/stream")

# Desired window size
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600

if __name__ == '__main__':
    while True:
        if cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                continue

            # Resize the frame to desired dimensions
            frame = cv2.resize(frame, (WINDOW_WIDTH, WINDOW_HEIGHT))

            # Show the frame
            cv2.imshow("ESP32CAM Stream", frame)

            key = cv2.waitKey(1)
            if key == 27:  # Press ESC to exit
                break

    cap.release()
    cv2.destroyAllWindows()
