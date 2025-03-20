import cv2
import numpy as np
import requests
import face_recognition

# ESP32CAM URL
URL = "http://192.168.246.89"
AWB = True

# Load and encode known faces
known_image1 = face_recognition.load_image_file("TA.jpg")
known_encoding1 = face_recognition.face_encodings(known_image1)[0]

known_image2 = face_recognition.load_image_file("Pratik.jpg")
known_encoding2 = face_recognition.face_encodings(known_image2)[0]

# Video capture setup
cap = cv2.VideoCapture(URL + ":81/stream")

if __name__ == '__main__':
    while True:
        if cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                continue

            # Convert frame to RGB (face_recognition uses RGB format)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Detect faces
            face_locations = face_recognition.face_locations(rgb_frame)
            face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)
            
            for face_encoding, face_location in zip(face_encodings, face_locations):
                # Compare detected face with known encodings
                matches1 = face_recognition.compare_faces([known_encoding1], face_encoding)
                matches2 = face_recognition.compare_faces([known_encoding2], face_encoding)
                name = "Unknown"
                
                if matches1[0]:
                    name = "TA"
                elif matches2[0]:
                    name = "Pratik"

                # Draw rectangle and label
                top, right, bottom, left = face_location
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                cv2.putText(frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Show the frame
            cv2.imshow("ESP32CAM Face Recognition", frame)

            key = cv2.waitKey(1)
            if key == 27:  # Press ESC to exit
                break

    cap.release()
    cv2.destroyAllWindows()
