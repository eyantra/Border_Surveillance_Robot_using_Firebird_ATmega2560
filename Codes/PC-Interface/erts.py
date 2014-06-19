"""**************************************************************************************************
		Platform: Python 2.x and 2.x.x
		Title: Border Surveillance Bot
		Author: 
			1.Harish Tummalacherla
			2.Sameer Mohammed
**************************************************************************************************/

/********************************************************************************

   Copyright (c) 2010, ERTS Lab, IIT Bombay.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************"""
"""/*******************************************************************************
This code does image processing for face detection.
The code also handles false faces that may creep in.
'S' ----> Move Left
'T' ----> Move Right
'Q' ----> Move Forward
'R' ----> Move Backward
'B' ----> Beep for one second
'Z' ----> Move Fast Left 
********************************************************************************/"""


import sys	#importing system for handling signals for exit
import cv	#importing opencv for face detection
import time     #for sleep function
import serial   #importing pyserial for serial communication
count = 0
# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
	port='/dev/ttyUSB0',	#The port where the serial communication usb is present.
	baudrate=9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS
)

ser.open()
ser.isOpen()


def detect(image):

    #Getting size of the image for handling generic image resolution, i.e. handling webcam with arbitrary resolution
    image_size = cv.GetSize(image)

    # create grayscale version
    grayscale = cv.CreateImage(image_size, 8, 1)	#creating a blank image with the given image's resolution
    cv.CvtColor(image, grayscale, cv.CV_BGR2GRAY)	#copying the black and white version of the image into the blank image

    # create storage
    storage = cv.CreateMemStorage(0)			#creating required storage for face detection

    # equalize histogram
    cv.EqualizeHist(grayscale, grayscale)

    # show processed image
    cv.ShowImage('Processed', grayscale)

    # detect objects
    cascade = cv.Load('haarcascade_frontalface_alt.xml')	#loading the Haar Cascade
    faces = cv.HaarDetectObjects(grayscale, cascade, storage, 1.2, 2, cv.CV_HAAR_DO_CANNY_PRUNING)	#detecting the faces in the image
    #These parameters are tweaked to RGB video captures, please refer to http://opencv.willowgarage.com/documentation/python/objdetect_cascade_classification.html for tweaking your parameters.
    print faces	#printing the rectangles circumscribing the face in the image
    #drawing rectangles around the faces in the image
    if faces:
        for i in faces:
            cv.Rectangle(image,
                         (i[0][0], i[0][1]),
                         (i[0][0] + i[0][2], i[0][1] + i[0][3]),
                         (0, 255, 0),
                         3,
                         8,
                         0)
    return faces    


if __name__ == "__main__":
    print "Press ESC to exit ..."

    # create windows
    cv.NamedWindow('Raw', cv.CV_WINDOW_AUTOSIZE)	#creating autosizable windows for captured frame from webcam
    cv.NamedWindow('Processed', cv.CV_WINDOW_AUTOSIZE)	#creating autosizable windows for processed image

    # create capture device
    device = 1 # assume we want second capture device(USB webcam), use i for the capure device /dev/videoi
    capture = cv.CaptureFromCAM(device)
    cv.SetCaptureProperty(capture, cv.CV_CAP_PROP_FRAME_WIDTH, 640) #setting capture width to 640
    cv.SetCaptureProperty(capture, cv.CV_CAP_PROP_FRAME_HEIGHT, 480) #setting capture height to 480
    #If you want to capture at native resolution of the web cam don't set obove width and height parameters, the processing speed will be slower for larger image 	resolutions

    # check if capture device is OK
    if not capture:
        print "Error opening capture device"
        sys.exit(1)
    faceBeep = 0 		#Used for beeping control
    firstFaceDetected = 0	#Used for face detection
    falseDetection = 0		#For false detection	
    faceNotDetected = 0		#used for checking if no face is found
    multipleFaces = 0		#for indicating that multiple faces
    multipleFacesInit = 0
    forward = 0
    left = 0
    right = 0
    while 1:
        # do forever

        # capture the current frame
        frame = cv.QueryFrame(capture)
        if frame is None:
            continue

        # mirror
        #cv.Flip(frame, None, 1)

        # face detection
        faces = detect(frame)
	if len(faces) > 0:
		firstFaceDetected = 1
		#multiple faces
		if len(faces) > 1:
			multipleFacesInit = 0
			multipleFaces = 1
			ser.write('Q'+'\r\n')
			forward = forward + 1
			faceBeep = 0
		#Single face detected
		else:
			if multipleFacesInit == 1:
				multipleFaces = 0
				multipleFacesInit = 0
			#algining itself to face the enemy				
			if faces[0][0][0] + float(faces[0][0][2]/2) < 260:
		    		ser.write('S'+'\r\n')
				left = left + 1
			else:
				if faces[0][0][0] + float(faces[0][0][2]/2) > 380:
		    			ser.write('T'+'\r\n')
					right = right + 1
				else:
					faceBeep = faceBeep + 1
					#alarming for detected enemy
					if faceBeep > 3:
						faceBeep = 0
						print 'Beeping for Faces'
						ser.write('B'+'\r\n')
						time.sleep(1)
						ser.write('B'+'\r\n')
						time.sleep(1)
						ser.write('B'+'\r\n')
						time.sleep(1)
						ser.write('B'+'\r\n')
						time.sleep(1)
						ser.write('B'+'\r\n')
						if multipleFaces == 0:
							firstFaceDetected = 0
	else:
		#scouting
		if firstFaceDetected == 0:
			ser.write('Z'+'\r\n')
			#print "batu\n"
		else:
			if multipleFaces == 0:
				falseDetection = falseDetection + 1
			#handling false detection
			if falseDetection > 10:
				falseDetection = 0
				firstFaceDetected = 0
			#retracing to position and orientation of multiple face detection
			faceNotDetected = faceNotDetected + 1
			if faceNotDetected > 10 and multipleFaces == 1:
				faceNotDetected = 0
				multipleFacesInit = 1
				#Realign to initial position of multiple face detection.
				while forward > 0:
					ser.write('R'+'\r\n')
					forward = forward - 1
					time.sleep(1)	#waiting for clearing of serial buffer				
				#Realign to initial orientation of multiple face detection.
				if left > right:
					x = left - right
					left = 0
					right = 0			
					while x > 0:
						ser.write('T'+'\r\n')
						x = x - 1
						time.sleep(1)	#waiting for clearing of serial buffer				
				else:
					x = right - left
					right = 0
					left = 0
					while x > 0:
						ser.write('S'+'\r\n')
						x = x - 1
						time.sleep(1)	#waiting for clearing of serial buffer				

        # display webcam image
        cv.ShowImage('Raw', frame)

        # handle events
        k = cv.WaitKey(10)

        if k == 0x1b: # ESC
            print 'ESC pressed. Exiting ...'
            break	#Exiting the program
