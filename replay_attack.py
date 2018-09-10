#!/usr/bin/python3.6
import numpy as np
import cv2

# Set up to default camera
cap = cv2.VideoCapture(0)

replay_attack = []  # List of frames for replay attack
recording = False   # Recording current frames
play_attack = False # Playing attack
attack_idx = 0      # Frame index for attack loop

# Titles for the different frames
FRAME_TITLE = 'Live Camera'
REPLAY_TITLE = 'Attacked Camera'

# Change the size of the frames. This can significantly
# decrease lag on less powerful machines
RESIZE_FACTOR = 0.5

while True:
    # Get the frame (and a T/F value)
    ret, frame = cap.read()
    size=np.shape(frame)
    frame = cv2.resize(frame,None,fx=RESIZE_FACTOR,fy=RESIZE_FACTOR)
    # If recording, append the current frame to the list
    if (recording):
        replay_attack.append(frame)

    # Show the live frames
    cv2.imshow('Frame', frame)

    # Show the replay attack frames
    if (play_attack and not recording):
        # Safety check if the play button was pressed,
        # but there aren't any recordings
        if (len(replay_attack) == 0):
            cv2.imshow(REPLAY_TITLE, frame)
        else:
            # Display the frames from the list of recordings
            attack_frame = replay_attack[attack_idx]
            # Change each pixel by [-1,1]
            dup_frame = np.add(attack_frame,
                            (np.random.rand(len(attack_frame), len(attack_frame[0]), 3)*4-2))
            # Clip the pixel values at 0 and 255 so they can all be displayed
            np.clip(dup_frame, 0, 255, out=dup_frame)
            # Convert the array back to values of uint8
            dup_frame = dup_frame.astype('uint8')
            # Show the frame
            cv2.imshow(REPLAY_TITLE, dup_frame)

#            cv2.imshow(REPLAY_TITLE, replay_attack[attack_idx])
            attack_idx = (attack_idx+1)%len(replay_attack)
    else:
        # If not playing the attack, show the live camera feed
        cv2.imshow(REPLAY_TITLE, frame)

    # cv2.imshow requires a cv2.waitKey() command.
    # Display each frame for ~20ms and check
    # what keys were pressed to enable different features
    keyPressed = cv2.waitKey(20) & 0xFF

    # q -> quit
    if (keyPressed == ord('q')):
        break
    # r -> start/stop recording
    elif (keyPressed == ord('r')):
        if (not recording):
            replay_attack = [] # Clear the previous recording
        recording = not recording
    # p -> play attack or play live stream
    elif (keyPressed == ord('p')):
        play_attack = not play_attack
        attack_idx = 0 # reset index on replay attack loop

# Release the hold on the webcam and ensure all
# windows are closed
cap.release()
cv2.destroyAllWindows()

