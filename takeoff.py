from pymavlink import mavutil
import time
import cv2
from mediapipe.python.solutions import hands

# Initialize the camera
cap = cv2.VideoCapture(0)

# Start a connection listening on a TCP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5762')

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
mode_id = the_connection.mode_mapping()['GUIDED']

# Arm the drone (correction)
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Arm ACK: {ack_msg}")

# Set the mode to GUIDED (correction)
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                     0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Change Mode ACK: {ack_msg}")

# Initialize MediaPipe Hands
mp_hands = hands.Hands()

# Gesture recognition loop
while True:
    ret, frame = cap.read()
    
    # Perform hand tracking using MediaPipe
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = mp_hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        # Get hand landmarks
        for landmarks in results.multi_hand_landmarks:
            # Implement gesture recognition logic here
            # For example, you can calculate the position of fingers, thumbs, etc.
            # and map them to specific drone control actions
            
            # Example: Detect thumb down gesture
            thumb_tip = landmarks.landmark[hands.HandLandmark.THUMB_TIP]
            index_tip = landmarks.landmark[hands.HandLandmark.INDEX_FINGER_TIP]

            # Assuming thumb down gesture when thumb tip is below index finger tip
            if thumb_tip.y > index_tip.y:
                # Print a prompt in the terminal
                print("Gesture Detected: Gesture Integrated")
                
                # Takeoff to an altitude of 10 meters
                the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 2, 0, 0, 0, 0, 0, 0, 10)
                ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                print(f"Takeoff ACK: {ack_msg}")
    
    cv2.imshow("Gesture Detection", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()