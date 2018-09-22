#!/usr/bin/python3.6
import numpy as np
import cv2
import time

from copy import deepcopy

# Set up to default camera
cap = cv2.VideoCapture(0)

replay_attack = []  # List of frames for replay attack
recording = False   # Recording current frames
play_attack = False # Playing attack
attack_idx = 0      # Frame index for attack loop
motion_detected = False # Note if the current frame differs from background frame

# Titles for the different frames
FRAME_TITLE = 'Live Camera'
REPLAY_TITLE = 'Attacked Camera'

# Change the size of the frames. This can significantly
# decrease lag on less powerful machines
RESIZE_FACTOR = 1

RECHECK_TIME_DELAY = 3

COMP_THRESH = 5
PERCENT_CHANGE_THRESH = .01

comp_frame = None
movement = False
last_check_time = time.time()

while True:
    # Get the frame (and a T/F value)
    ret, frame = cap.read()
    if (not ret):
        continue

    size=np.shape(frame)
    # Resize and convert to grayscale
    frame = cv2.resize(frame,None,fx=RESIZE_FACTOR,fy=RESIZE_FACTOR)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # If recording, append the current frame to the list
    if (recording):
        replay_attack.append(frame)

    if (time.time()-last_check_time > RECHECK_TIME_DELAY or not movement):
        last_check_time = time.time()
        
        # Blur the image to smooth out edges
        blurred = cv2.GaussianBlur(frame,(21,21),0) 

        if (comp_frame is None):
            comp_frame = blurred

        # Find the difference between the blurred frame and the
        # comparison frame
        diff = cv2.absdiff(comp_frame, blurred)
        cv2.imshow('diff', diff)
        
        # Use the threshold to make all pixels > threshold
        # white and leave the rest black. cv2.threshold returns
        # (ret, dst), and we only want dst, so [1] is used
        thresh = cv2.threshold(diff, COMP_THRESH, 255, cv2.THRESH_BINARY)[1]
        
        # Dilate the threshold image to blob together nearby pixels
        kernel = np.ones((5,5),np.uint8)
        dilated = cv2.dilate(thresh, kernel, iterations=4)

        # If the number of changed pixels exceeds the threshold, there is movement
        if ((dilated.sum()/255)/dilated.size > PERCENT_CHANGE_THRESH):
            movement = True
        else:
            movement = False

        should_record = not movement

        # Update the comp frame
        comp_frame = blurred

    if (movement):
        cv2.putText(frame,'Movement Detected', (10,10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,100,100), 1)
    else:
        cv2.putText(frame,'No Movement Detected', (10,10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,100,100), 1)

    # Show whether recording is True or False
    if (recording):
        cv2.putText(frame,'recording', (10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,100,100), 1)

   
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
                            (np.random.rand(len(attack_frame), len(attack_frame[0]))*4-2))
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


    # Show images of frames to gather movement data
    cv2.imshow('blurred', blurred)
    cv2.imshow('thresh', thresh)
    cv2.imshow('dilated', dilated)

    if (should_record):
        replay_attack = [] # Clear the previous recording
        recording = True
    else:
        recording = False
 
 
    # cv2.imshow requires a cv2.waitKey() command.
    # Display each frame for ~20ms and check
    # what keys were pressed to enable different features
    keyPressed = cv2.waitKey(20) & 0xFF

    # q -> quit
    if (keyPressed == ord('q')):
        break
    # r -> start/stop recording
#    elif (keyPressed == ord('r') or should_record):
#        if (not recording):
#            replay_attack = [] # Clear the previous recording
#        recording = not recording
    # p -> play attack or play live stream
    elif (keyPressed == ord('p')):
        play_attack = not play_attack
        attack_idx = 0 # reset index on replay attack loop

# Release the hold on the webcam and ensure all
# windows are closed
cap.release()
cv2.destroyAllWindows()

