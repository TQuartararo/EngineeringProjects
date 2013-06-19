import cv,time,math,serial
'''As of 3/24/13 this program tracks two specified colors and draws rectangles around them in a displayed
image. When the two colors line up within a certain bounds, green circles are plaed about their centroids
to indicate that a second criteria has been met i.e. they are the right color and they are vertically 
aligned. The serial is not working properly, neither is the distance calculator. When the distances are 
known, the location of the stereo rig can be found with respect to the beacon coordinate frame using the
circleIntersections() function.

To Do:  [Create third criteria of specific color above or below another color]
	Get Serial working
	Get distance working, specifically distance to beacon 1, beacon 2, etc.
	Make circleIntersections() reject points far away

3/25/13: Accomplished the task of adding a third criteria of having a specific color above or below another.
3/27/13: Color detection is now performed on the undistorted images. Undistortion is accomplished using the
chessBoardCalibration.py program.
	 The color detection program is now based on specified lists of upper and lower Hue bounds.
	 Serial communication seems to be working, but it has not been tested with an Arduino yet.
	 Get the width of the beacon in pixels, use this to find pixels per inch of beacon. Then find
	 distance to center of the camera in inches and localize like that. I have the width of the
 	 beacon in pixels, but getting distance isn't working yet.

	 Currently, the robot works by keeping the centroid of the beacon at the center of the camera
	 at all times.
'''
########################################################################################################
def findBeacons(frame,flag,upper,lower):
	'''This function is used to find the location of specified colors within an image. The function
	calculates and draws a bounding rectangle around the specified colors and returns this image 
	as well as the centroid of the rectangle. Then this function calls some test functions to 
	determine if the found blob is a beacon.'''

	#using only one of the two cameras currently in order to find the horizontal distacne to the left camera
	#flags needed to determine if this function is being called by the left or right camera
	if flag == 'L':
		holderImage = cv.QueryFrame(captureL)#hold the real image
		colorImage = undistort(holderImage,flag)#undistort the image
	elif flag == 'R':
		holderImage = cv.QueryFrame(captureR)#hold the real image
		colorImage = undistort(holderImage,flag)#undistort the image
	#holderImage = cv.QueryFrame(captureL)#hold the real image
	#colorImage = undistort(holderImage,flag)#undistort the image

	imdraw=cv.CreateImage(cv.GetSize(frame),8,3)#create blank image to draw on
	cv.SetZero(imdraw)#clear the array
	cv.Flip(colorImage,colorImage,1)#flip the captured image for more natural viewing
	cv.Smooth(colorImage, colorImage, cv.CV_GAUSSIAN, 1, 0)#smooth image to reduce noise

	threshImage=specifyColor(colorImage,upper,lower)#returns the specified color range thresholded image
	
	#image processing magic
	#cv.Erode(threshImage,threshImage,None,3)
	#cv.Dilate(threshImage,threshImage,None,3)
	#cv.ShowImage("test",threshImage)
	img2=cv.CloneImage(threshImage)#reassign images
	storage = cv.CreateMemStorage(0)#memory storage location needed for next function
	
	#this cvFunction finds contours that match the specified colors
	contour = cv.FindContours(threshImage, storage, cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
	
	#define new lists
	pt1 = []
	pt2 = [] 
	centroidx = []
	centroidy = []
	width1 = []
	width2 = []

	#cycle through all of the contours and draw red bounding rectangles around them. This is helpful
	#in showing what the camera is detecting with the first detection criteria.
	while contour:
		# Draw bounding rectangles
		bound_rect = cv.BoundingRect(list(contour))#draw a rectangle around the contours
		contour = contour.h_next()#h_next() points to the next reference in the list
	
		#Store vertices of the rectangle in order to draw it on the image.
		pt1 = (bound_rect[0], bound_rect[1])#assign vertex 1 
		pt2 = (bound_rect[0] + bound_rect[2], bound_rect[1] + bound_rect[3])#assign vertex 2
		
		#add the x-values of points(vertices) 1 and 2 to the list points[] to find the width of the beacon		
		width1.append(bound_rect[1])
		width2.append(bound_rect[1]+bound_rect[3])

		#draw a rectangle onto colorImage. pt1 and pt2 are the rectangles vertices, then the color
		cv.Rectangle(colorImage, pt1, pt2, cv.CV_RGB(255,0,0), 1)
	
		#Calculate the centroids, which will be used in following tests
		centroidx.append(cv.Round((pt1[0]+pt2[0])/2))#round the average of the x-coordinates of the two points
		centroidy.append(cv.Round((pt1[1]+pt2[1])/2))#round the average of the y-coordinates of the two points
	
	#attempt to find a beacon. If a beacon is found, return all of the results and a signal stating success (1)
	#otherwise, send all of the values and a signal stating failure (0)
	try:
		#Begin testing to find beacons!
		colorImage,beaconx,beacony,beaconHits = testBeacons(centroidx,centroidy,colorImage)
		pixelWidth =  width2[0] - width1[0]
		#distanceFromCenter = getHorizontal(pixelWidth,centroidy[0])
		print pixelWidth
		#return image, coordinates of the beacon, and a flag representing a beacon was found
		return colorImage,beaconx,beacony,pixelWidth,1
	except:
		pixelWidth = 1000;
		return colorImage,beaconx,beacony,pixelWidth,0
	
########################################################################################################
def getHorizontal(pixelWidth,centroid):
	inchesPerPixel = pixelWidth/3.5 #width of beacon in pixels divided by width of beacon in inches
	distanceFromCenter = ((1920/2) - centroid)/inchesPerPixel
	return distanceFromCenter	
########################################################################################################
def undistort(holderImage,flag):
	'''Use the calibration matrices calculated using chessboardCalibration.py to undistort the left and right
	cameras before finding beacons.'''

	if flag == 'L':
		colorImage=cv.CreateImage(cv.GetSize(holderImage),8,3)#remap the camera frame
		mapxL = cv.CreateImage( cv.GetSize(imageL), cv.IPL_DEPTH_32F, 1 );
		mapyL = cv.CreateImage( cv.GetSize(imageL), cv.IPL_DEPTH_32F, 1 );
		cv.InitUndistortMap(intrinsicL,distortionL,mapxL,mapyL)
		cv.Remap( holderImage, colorImage, mapxL, mapyL )
	elif flag == 'R':
		colorImage=cv.CreateImage(cv.GetSize(holderImage),8,3)#create blank image called imdraw
		mapxR = cv.CreateImage( cv.GetSize(imageR), cv.IPL_DEPTH_32F, 1 );
		mapyR = cv.CreateImage( cv.GetSize(imageR), cv.IPL_DEPTH_32F, 1 );
		cv.InitUndistortMap(intrinsicR,distortionR,mapxR,mapyR)
		cv.Remap( holderImage, colorImage, mapxR, mapyR)
	return colorImage
########################################################################################################
def specifyColor(im,upper,lower):
	'''Convert RGB into HSV for easy colour detection and threshold it with specific colors as white 		
	and all other colors as black,then return that image for processing'''
	#Create a blank image to store the converted RGB2HSV image
	imghsv=cv.CreateImage(cv.GetSize(im),8,3)
	cv.CvtColor(im,imghsv,cv.CV_BGR2HSV)

	#create the blank images to store the 2 specified color images
	imgColor1=cv.CreateImage(cv.GetSize(im),8,1)
	imgColor2=cv.CreateImage(cv.GetSize(im),8,1)
	
	#create blank image to hold complete threshold image
	imgthreshold=cv.CreateImage(cv.GetSize(im),8,1)
	
	#poster boards green[(60,124,160) to (66,132,190)], orange[(12,210,200) to (16,255,245)], pink[(168,195,210) to (173,201,252)]
	# Select a range of colors (source, lower bound, upper bound, destination)
	cv.InRangeS(imghsv,cv.Scalar(lower[0],210,160),cv.Scalar(upper[0],255,245),imgColor1)
	cv.InRangeS(imghsv,cv.Scalar(lower[1],185,180),cv.Scalar(upper[1],240,255),imgColor2)

	#adds the color arrays to imgthreshold
	cv.Add(imgColor1,imgColor2,imgthreshold)
	return imgthreshold 
########################################################################################################
def testBeacons(centroidx,centroidy,colorImage):
	'''This function is used to run all of the tests on the image to determine the location of the beacon. 
	Tests include:
		-alignmentTest: determine if centroids vertically aligned
		-colorTest: determine if correct color is above the other
		-cicleTest: is the blob a circle?**To be implemented
	'''		

	#test all detected blobs if their centroids are vertically aligned
	colorImage,beaconx,beacony,beaconHits = alignmentTest(centroidx,centroidy,colorImage)
	#test all of the blobs that pass the first test if a specified color is on top of another	
	colorImage,beaconx,beacony,beaconHits = colorTest(beaconHits,beaconx,beacony,colorImage,upper,lower)
	return colorImage,beaconx,beacony,beaconHits
########################################################################################################
def alignmentTest(centroidx,centroidy,colorImage):
	'''Determine if the detected bounding rectangles are within a specified distance of one another in order to
	check if we are looking at the beacon by cycling through all of the rectangles.'''

	#lists to store successful blob locations
	beaconx = []	
	beacony = []
	beaconHits = []#list of the position of success in array

	#cycle through all blobs with respect to eachother
	for i in range(len(centroidx)):
		for j in range(len(centroidx)):
			#skip if we're looking at the same blob
			if i == j:
				continue
			else:
				#if the centroids of the two detected objects line up within a allowable error
				a =  (centroidx[j] + 0.05*centroidx[j])
				b =  (centroidx[j] - 0.05*centroidx[j])
				if (b <= centroidx[i] <= a):
					#if we find a beacon, save the centroid location
					beaconx.append(centroidx[i])
					beacony.append(centroidy[i])
					beaconHits.append(i)
					#draw a green circle on the image to indicate beacon centroid location
					cv.Circle(colorImage,(centroidx[i],centroidy[i]),20,cv.CV_RGB(0,255,0))
	#	print beaconHits
	#return all of the parameters that we want
	return colorImage,beaconx,beacony,beaconHits
########################################################################################################
def colorTest(beaconHits,centroidx,centroidy,colorImage,upper,lower):
	'''Detect the color of the centroids of objects that met the previous criteria. If the colors are within
	the allowable range and in the correct order, then we return that as a definite beacon.'''

	#new list of final beacon locations (final since there are only two tests right now)
	beaconFinalx = []
	beaconFinaly = []
	beaconHitsFinal = []
	#convert image to HSV for processing
	colorTestImage = cv.CreateImage(cv.GetSize(colorImage),8,3)
	cv.CvtColor(colorImage,colorTestImage,cv.CV_BGR2HSV) 

	k = 0 #beacon counter
	for i in range(len(centroidx)):
		for j in range(len(centroidx)):
			if i == j:
				continue
			else:
				a =  (centroidx[j] + 0.05*centroidx[j])
				b =  (centroidx[j] - 0.05*centroidx[j])
				
				#return the HSV value of pixel in a 1x3 list
				color1 = cv.Get2D(colorTestImage,centroidy[j],centroidx[j])#coordinates given in (y,x) 
				color2 = cv.Get2D(colorTestImage,centroidy[i],centroidx[i])
				#(if color2 = red and color1 = blue)
				if ((lower[0] <= color2[0] <= upper[0]) and (lower[1]<= color1[0] <= upper[1])):
					#This ensures that the blobs we're looking at are aligned. There should be a way
					#to use alignmentTest to not have to do this again.
					if ((centroidy[j] > centroidy[i]) and (b <= centroidx[i] <= a)):
						beaconFinalx.append((centroidx[j]+centroidx[i])/2)
						beaconFinaly.append((centroidy[j]+centroidy[i])/2)
						for i in range(len(beaconHits)):
							if (beaconHits[i] == beaconHits[j]):
								beaconHitsFinal.append(beaconHits[j])
						cv.Circle(colorImage,(beaconFinalx[k],beaconFinaly[k]),30,cv.CV_RGB(255,255,0),10)	
						k = k+1
	
	#return all of the parameters that we want
	#print beaconHitsFinal
	return colorImage,beaconFinalx,beaconFinaly,beaconHitsFinal
########################################################################################################
def straightLine(beaconX,integralError,pastError):
	'''This will be the basic CV program that attempts to keep the beacon at the center of the camera
	at all times. This is not stereovision, or robust, but should allow the robot to move in a straight
	line. The PID algorithm will be built in to this function'''
	#PID values
	Kp = 80
	Ki = 8
	Kd = 0
	setpoint = 1920/2 #middle pixel with 640x480 resolution
	span = 1920.0 #motor controller take 99 (stop) to 255 (full forward) 
	
	error = (beaconX - setpoint)/span #normalize the error to get +/- 0 to 100% error
	integralError = 0.9*(error + integralError) #0.9 acts as a decay factor to avoid accumulated error
	derivativeError = (error - pastError)
	pastError = error

	#The correction needs to be a positive integer in order to be sent to the arduino. The zero command
	#to stop the motors is 99. Therefore, this correction will move about the zero point of 99 to correct. 
	correction = ((int)((Kp*error + Ki*integralError + Kd*derivativeError))) + 130

	#send the correction value to the arduino to control the robot.
	arduinoCom(correction)
	print correction
	return integralError,pastError

########################################################################################################
def arduinoCom(correction):
	'''This establishes serial communication with the arduino via pySerial, allowing this program
	to send signals to the actuators. Currently this isn't really doing anything, also I don't have
	the PID program written.'''
	ser.write(chr(correction))#chr(number) formats the value correctly to be read by the arduino
	return 0

########################################################################################################
#initalize parameters
captureL=cv.CaptureFromCAM(1)
cv.SetCaptureProperty(captureL,cv.CV_CAP_PROP_FRAME_WIDTH,1920)#set the resolution of the cameras 
cv.SetCaptureProperty(captureL,cv.CV_CAP_PROP_FRAME_HEIGHT,1080)#1920x1080
imageL=cv.QueryFrame(captureL)

# Load all of the parameters from the chessboard calibration program
intrinsicL = cv.Load("IntrinsicsLeft.xml")
distortionL = cv.Load("DistortionLeft.xml")
print " loaded all distortion parameters"

#define the upper and lower Hue values of desired beacon colors in lists. These values are found
#using pickAndTrack.py #red construction paper [0,5], purple [117,128]

#poster boards green[(60,124,160) to (66,132,190)], orange[(9,210,200) to (16,255,245)], pink[(168,195,210) to (173,201,252)] 
#upper = [66,16,173]
#lower = [60,12,168]
#circles on paper
#upper = [9,106]
#lower = [1,100]
upper = [16,180]
lower = [9,168]

#establish serial connection
ser = serial.Serial('/dev/ttyACM0', 115200)#open the serial port *(check what serial port the arduino is connected to and adjust)
time.sleep(0.5)#give some time to establish the connection

#prepare display
cv.NamedWindow("Output",cv.CV_WINDOW_NORMAL)
#cv.NamedWindow("test",cv.CV_WINDOW_NORMAL)

#initialize the errors for the PID program
integralError = 0;
pastError = 0;
counter = 25
turnSignal = 50#size of the beacon when you want to turn
#Begin Program
while(1):
	#capture the images from the camera to process
	imageL=cv.QueryFrame(captureL)
	tL = cv.CloneImage(imageL);
	
	
	#attempt to detect and calculate distance to beacons
	im_drawL=cv.CreateImage(cv.GetSize(imageL),8,3)
	im_drawL,beaconLx,beaconLy,pixelWidth,test = findBeacons(imageL,'L',upper,lower)#find beacon Camera 1

	#for safety, if the motors are given a value that is too large the robot is sent a kill command and the
	#program is stopped.
	try:
		#The function called above findBeacons() returns a value of 0 or 1 to test indicating either
		#failure to find a beacon, or success. Using these cases, appropriate measures can be taken.
		if test == 1:
			counter = 175 #this is a reset for the slow down safety feature if no beacon is detected.
			for i in range(len(beaconLx)):
				#keep the beacon in the center of the camera's view at all times, constantly
				#update the integral and past errors.
				integralError,pastError = straightLine(beaconLx[i],integralError,pastError)
					
		elif test == 0:#no beacon detected
			#If the beacon is lost, a short amount of time will be spent trying to locate or the robot
			#is eventually stopped.
			if counter > 25:#(99 is the stop command to motors)
				counter = counter - 1
				arduinoCom(2) #signal motor slow down
				print ("No Beacon Detected, Slowing Down: " + (str)(counter))
			elif counter <= 0:
				print "I've lost sight of the beacon for too long"
				print "*****ABORT*****"
				arduinoCom(1)#signal motor kill
				break
		elif (pixelWidth >= turnSignal):
			arduinoCom(3)#signal a turn
			#time.sleep(1)#delay for one second
			#while (ser.read() == 0) #something like that
				#wait until the arduino makes the robot turn, and then continue
			
	except ValueError:
		arduinoCom(1)#kill motors
		print "You sent a value that the motors could not handle"
		print "*****ABORT MISSION*****"
		break

	#display the undistorted images with bounding rectangles
	cv.ShowImage( "Output", im_drawL )

	c = cv.WaitKey(33)
	if c==1048603:# enter esc key to exit
		break

arduinoCom(1)#stop the motors
ser.close()#close the serial port
print "Everything is going to be OK"#signal end of program
































