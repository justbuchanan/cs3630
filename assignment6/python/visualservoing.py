import time
from Scribbler2 import *
import cv2
import numpy as np
import math
from matplotlib import pyplot as plt


# Connect to the scribbler
# Set timeout to 0 to read instantly, non-blocking

# TODO: uncomment to connect to scribbler robot
# fname = "log-%d.txt" % time.time()
# s = Scribbler2('/dev/tty.Fluke2-0610-Fluke2',fname)

# Set timeout to zero
# print 'Connected!'
# s.setIRPower(140)
# s.setForwardness(1)
# Create a list of commands
# Command is a list [cmd, leftMotor, rightMotor, time]
# Setting motors to 200 will drive 
# forward with the fluke facing forward


# returns a contour object and an image with the contour overlaid on it (for debuggint)
# if @debug is True, displays all intermediate images and waits for a key press before returning
def find_black_square(imgPath, debug = False):
    # read grayscale image
    origImg = cv2.imread(imgPath)

    gray = cv2.cvtColor(origImg, cv2.COLOR_BGR2GRAY)

    # apply gaussian filter to reduce noise before we do edge detection
    kernel = np.ones((5,5),np.float32)/25
    filtered = cv2.filter2D(gray,-1,kernel)

    # run Canny edge-detection
    edged = cv2.Canny(filtered, 100, 100)

    # blur the edged image to help merge any lines that don't connect but are very close
    kernel2 = np.ones((2,2),np.float32)/4
    edgedBlurred = cv2.filter2D(edged, -1, kernel2)

    # get contours
    cnts, _ = cv2.findContours(edgedBlurred.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # sort contours from largets to smallest
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)

    # find the largest rectangle in the contours list
    square = None
    for c in cnts[1:]:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)

        if len(approx) == 4:
            square = approx
            break

    # draw the original image overlaid with the contour we think is the target black rect
    cntAnnotatedImg = origImg.copy()
    cv2.drawContours(cntAnnotatedImg, [square], -1, (0,255,0), 3)

    if debug:
        cv2.imshow('original', origImg)
        cv2.imshow('filtered', filtered)
        cv2.imshow('edged', edged)
        cv2.imshow('edgedBlurred', edgedBlurred)
        cv2.imshow('contours', cntAnnotatedImg)
        
        cv2.waitKey(0)

    # sort by ascending x-value
    square = sorted(square, key=lambda pt: pt[0][0])
    
    # extract left and right sides and sort by ascending y value
    leftPts = sorted(square[0:2], key=lambda pt: pt[0][1])
    rightPts = sorted(square[2:4], key=lambda pt: pt[0][1])

    # rearrange in order of increasing quadrant (pretending origin is at the center of the square)
    square = [rightPts[0], leftPts[0], leftPts[1], rightPts[1]]

    # convert from np.array to tuple
    square = [(pt[0][0], pt[0][1]) for pt in square]

    return square, cntAnnotatedImg



# demo!
sq1, img1 = find_black_square('../start.jpg')
sq2, img2 = find_black_square('../goal.jpg')
print("Square 1: " + str(sq1))
print("Square 2: " + str(sq2))
cv2.imshow('Start', img1)
cv2.imshow('Goal', img2)
cv2.waitKey(0)





# # Run the robot
# commands = []
# commands.append([100, 100, 1])
# commands.append([0,0,1])
# s.runCommands(commands);

# ## Take picture
# pic_fname = "pic-%d.jpg" % time.time()
# picture = s.takePicture('jpeg');
# s.savePicture(picture, pic_fname)


# # Run the robot again
# commands = []
# commands.append([100, 100, 1])
# commands.append([0,0,1])
# s.runCommands(commands);


# ## Take picture
# pic_fname = "pic-%d.jpg" % time.time()
# picture = s.takePicture('jpeg');
# s.savePicture(picture, pic_fname)


# ## Close (Do not remove this line)
# s.close();
