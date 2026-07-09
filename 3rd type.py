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
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Change Mode ACK: {ack_msg}")

# Initialize MediaPipe Hands
mp_hands = hands.Hands()

# Initialize a flag to prevent repeated actions
gesture_detected = False

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
            
            # Example: Detect thumb down gesture for takeoff
            thumb_tip = landmarks.landmark[hands.HandLandmark.THUMB_TIP]
            index_tip = landmarks.landmark[hands.HandLandmark.INDEX_FINGER_TIP]
            middle_tip = landmarks.landmark[hands.HandLandmark.MIDDLE_FINGER_TIP]
            pinky_tip = landmarks.landmark[hands.HandLandmark.PINKY_TIP]
            ring_tip = landmarks.landmark[hands.HandLandmark.RING_FINGER_TIP]

            if thumb_tip.y < index_tip.y and not gesture_detected:
                # Print a prompt in the terminal
                print("Gesture Detected: Thumbs-Up (Takeoff)")
                
                # Takeoff to an altitude of 10 meters
                the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 1, 0, 0, 0, 0, 0, 0, 20)
                ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                print(f"Takeoff ACK: {ack_msg}")

                # Set the flag to prevent repeated actions
                gesture_detected = True

            elif middle_tip.y < index_tip.y and not gesture_detected:
                the_connection.mav.set_position_target_global_int_send(the_connection.target_system,the_connection.target_component,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,int(0b110111111000),int(-35.360916 * 10 ** 7),int(149.177985 * 10 ** 7),10,0,0,0,0,0,0,1.57,0.5)
                ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                print(f"Set Position Target ACK: {ack_msg}")
                print("Waypoint 1")           

            elif thumb_tip.y < pinky_tip.y and not gesture_detected:
                the_connection.mav.set_position_target_global_int_send(the_connection.target_system,the_connection.target_component,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,int(0b110111111000),int(-35.3803029 * 10 ** 7),int(149.193306 * 10 ** 7),10,0,0,0,0,0,0,1.57,0.5)
                ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                print(f"Set Position Target ACK: {ack_msg}")    
                print("Waypoint 2")

            elif middle_tip.y < ring_tip.y and not gesture_detected:
                the_connection.mav.set_position_target_global_int_send(the_connection.target_system,the_connection.target_component,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,int(0b110111111000),int(-35.360916 * 10 ** 7),int(149.177985 * 10 ** 7),10,0,0,0,0,0,0,1.57,0.5)
                ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                print(f"Set Position Target ACK: {ack_msg}")
                print("Waypoint 3") 



            elif middle_tip.y and not gesture_detected:
                print("Land Sequence")
                # Land
                the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 1, 0, 0, 0, 0, 0, 0, 0)
                ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                print(f"Land ACK: {ack_msg}")

                # Set the flag to prevent repeated actions
                gesture_detected = True
    
    # Reset the flag if no gestures are detected
    if not results.multi_hand_landmarks:
        gesture_detected = False
    
    cv2.imshow("Gesture Detection", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()