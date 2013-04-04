'''	 chessboardCalibration.py
This code is an augmented version of calibration code by Abid.K (mail me at abidrahman2@gmail.com), which allows for the 
calibration of two cameras simultaneously for stereo vision.

'''

import cv2.cv as cv
import time,sys
n_boards=16	#no of boards
board_w=8	# number of horizontal corners
board_h=5	# number of vertical corners
board_n=board_w*board_h		# no of total corners
board_sz=(board_w,board_h)	#size of board




#creation of memory storages
image_pointsL=cv.CreateMat(n_boards*board_n,2,cv.CV_32FC1)
object_pointsL=cv.CreateMat(n_boards*board_n,3,cv.CV_32FC1)
point_countsL=cv.CreateMat(n_boards,1,cv.CV_32SC1)
intrinsic_matrixL=cv.CreateMat(3,3,cv.CV_32FC1)
distortion_coefficientL=cv.CreateMat(5,1,cv.CV_32FC1)

image_pointsR=cv.CreateMat(n_boards*board_n,2,cv.CV_32FC1)
object_pointsR=cv.CreateMat(n_boards*board_n,3,cv.CV_32FC1)
point_countsR=cv.CreateMat(n_boards,1,cv.CV_32SC1)
intrinsic_matrixR=cv.CreateMat(3,3,cv.CV_32FC1)
distortion_coefficientR=cv.CreateMat(5,1,cv.CV_32FC1)

#capture frames of specified properties and modification of matrix values
i=0
z=0		# to print number of frames
successesL=0
successesR=0
captureL=cv.CaptureFromCAM(1)
#cv.SetCaptureProperty(captureL,cv.CV_CAP_PROP_FRAME_WIDTH,1280)#set the resolution of the cameras to max
#cv.SetCaptureProperty(captureL,cv.CV_CAP_PROP_FRAME_HEIGHT,720)
captureR=cv.CaptureFromCAM(2)
#cv.SetCaptureProperty(captureR,cv.CV_CAP_PROP_FRAME_WIDTH,1280)#set the resolution of the cameras to max
#cv.SetCaptureProperty(captureR,cv.CV_CAP_PROP_FRAME_HEIGHT,720)
#	capturing required number of views
while(successesL<n_boards):
	found=0
	imageL=cv.QueryFrame(captureL)
	imageR=cv.QueryFrame(captureR)
	gray_imageL=cv.CreateImage(cv.GetSize(imageL),8,1)
	gray_imageR=cv.CreateImage(cv.GetSize(imageR),8,1)
	cv.CvtColor(imageL,gray_imageL,cv.CV_BGR2GRAY)
	cv.CvtColor(imageR,gray_imageR,cv.CV_BGR2GRAY)
	
	(foundL,cornersL)=cv.FindChessboardCorners(gray_imageL,board_sz,cv.CV_CALIB_CB_ADAPTIVE_THRESH| cv.CV_CALIB_CB_FILTER_QUADS)
	(foundR,cornersR)=cv.FindChessboardCorners(gray_imageR,board_sz,cv.CV_CALIB_CB_ADAPTIVE_THRESH| cv.CV_CALIB_CB_FILTER_QUADS)
	cornersL=cv.FindCornerSubPix(gray_imageL,cornersL,(11,11),(-1,-1),(cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER,30,0.1)) 
	cornersR=cv.FindCornerSubPix(gray_imageR,cornersR,(11,11),(-1,-1),(cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER,30,0.1)) 	
	# if got a good image,draw chess board
	if (foundL==1 and foundR == 1):
		print "found frame number {0}".format(z+1)
		cv.DrawChessboardCorners(imageL,board_sz,cornersL,1) 
		cv.DrawChessboardCorners(imageR,board_sz,cornersR,1) 
		corner_countL=len(cornersL)
		corner_countR=len(cornersR)
		z=z+1
		time.sleep(2)
		
	# if got a good image, add to matrix
	if (len(cornersL)==board_n and len(cornersR)==board_n):
		stepL=successesL*board_n
		kL=stepL
		stepR=successesR*board_n
		kR=stepR
		for j in range(board_n):
			cv.Set2D(image_pointsL,kL,0,cornersL[j][0])
			cv.Set2D(image_pointsL,kL,1,cornersL[j][1])
			cv.Set2D(object_pointsL,kL,0,float(j)/float(board_w))
			cv.Set2D(object_pointsL,kL,1,float(j)%float(board_w))
			cv.Set2D(object_pointsL,kL,2,0.0)
			kL=kL+1
		cv.Set2D(point_countsL,successesL,0,board_n)
		successesL=successesL+1
		
		for q in range(board_n):
			cv.Set2D(image_pointsR,kR,0,cornersR[q][0])
			cv.Set2D(image_pointsR,kR,1,cornersR[q][1])
			cv.Set2D(object_pointsR,kR,0,float(q)/float(board_w))
			cv.Set2D(object_pointsR,kR,1,float(q)%float(board_w))
			cv.Set2D(object_pointsR,kR,2,0.0)
			kR=kR+1
		cv.Set2D(point_countsR,successesR,0,board_n)
		successesR=successesR+1
		time.sleep(2)
		print "-------------------------------------------------"
		print "\n"
		
	
	cv.ShowImage("Test FrameL",imageL)
	cv.ShowImage("Test FrameR",imageR)
	cv.WaitKey(33)

print "checking is fine	,all matrices are created"
cv.DestroyWindow("Test FrameL")
cv.DestroyWindow("Test FrameR")

# now assigning new matrices according to view_count
object_pointsL2=cv.CreateMat(successesL*board_n,3,cv.CV_32FC1)
image_pointsL2=cv.CreateMat(successesL*board_n,2,cv.CV_32FC1)
point_countsL2=cv.CreateMat(successesL,1,cv.CV_32SC1)
object_pointsR2=cv.CreateMat(successesR*board_n,3,cv.CV_32FC1)
image_pointsR2=cv.CreateMat(successesR*board_n,2,cv.CV_32FC1)
point_countsR2=cv.CreateMat(successesR,1,cv.CV_32SC1)

#transfer points to matrices

for i in range(successesL*board_n):
	cv.Set2D(image_pointsL2,i,0,cv.Get2D(image_pointsL,i,0))
	cv.Set2D(image_pointsL2,i,1,cv.Get2D(image_pointsL,i,1))
	cv.Set2D(object_pointsL2,i,0,cv.Get2D(object_pointsL,i,0))
	cv.Set2D(object_pointsL2,i,1,cv.Get2D(object_pointsL,i,1))
	cv.Set2D(object_pointsL2,i,2,cv.Get2D(object_pointsL,i,2))
for i in range(successesL):
	cv.Set2D(point_countsL2,i,0,cv.Get2D(point_countsL,i,0))

cv.Set2D(intrinsic_matrixL,0,0,1.0)
cv.Set2D(intrinsic_matrixL,1,1,1.0)

rcvL = cv.CreateMat(n_boards, 3, cv.CV_64FC1)
tcvL = cv.CreateMat(n_boards, 3, cv.CV_64FC1)

for i in range(successesR*board_n):
	cv.Set2D(image_pointsR2,i,0,cv.Get2D(image_pointsR,i,0))
	cv.Set2D(image_pointsR2,i,1,cv.Get2D(image_pointsR,i,1))
	cv.Set2D(object_pointsR2,i,0,cv.Get2D(object_pointsR,i,0))
	cv.Set2D(object_pointsR2,i,1,cv.Get2D(object_pointsR,i,1))
	cv.Set2D(object_pointsR2,i,2,cv.Get2D(object_pointsR,i,2))
for i in range(successesR):
	cv.Set2D(point_countsR2,i,0,cv.Get2D(point_countsR,i,0))

cv.Set2D(intrinsic_matrixR,0,0,1.0)
cv.Set2D(intrinsic_matrixR,1,1,1.0)

rcvR = cv.CreateMat(n_boards, 3, cv.CV_64FC1)
tcvR = cv.CreateMat(n_boards, 3, cv.CV_64FC1)

print "checking camera calibration............."
# camera calibration
cv.CalibrateCamera2(object_pointsL2,image_pointsL2,point_countsL2,cv.GetSize(imageL),intrinsic_matrixL,distortion_coefficientL,rcvL,tcvL,0)
cv.CalibrateCamera2(object_pointsR2,image_pointsR2,point_countsR2,cv.GetSize(imageR),intrinsic_matrixR,distortion_coefficientR,rcvR,tcvR,0)
print " checking camera calibration.........................OK	"	
	
# storing results in xml files
cv.Save("IntrinsicsL.xml",intrinsic_matrixL)
cv.Save("DistortionL.xml",distortion_coefficientL)
cv.Save("IntrinsicsR.xml",intrinsic_matrixR)
cv.Save("DistortionR.xml",distortion_coefficientR)
cv.Save("objectPts.xml",object_pointsL2)
cv.Save("numPoints.xml",point_countsL2)
cv.Save("imagePtsLeft.xml",image_pointsL2)
cv.Save("imagePtsRight.xml",image_pointsR2)
# Loading from xml files
intrinsicL = cv.Load("IntrinsicsL.xml")
distortionL = cv.Load("DistortionL.xml")
intrinsicR = cv.Load("IntrinsicsR.xml")
distortionR = cv.Load("DistortionR.xml")
print " loaded all distortion parameters"

mapxL = cv.CreateImage( cv.GetSize(imageL), cv.IPL_DEPTH_32F, 1 );
mapyL = cv.CreateImage( cv.GetSize(imageL), cv.IPL_DEPTH_32F, 1 );
cv.InitUndistortMap(intrinsicL,distortionL,mapxL,mapyL)

mapxR = cv.CreateImage( cv.GetSize(imageR), cv.IPL_DEPTH_32F, 1 );
mapyR = cv.CreateImage( cv.GetSize(imageR), cv.IPL_DEPTH_32F, 1 );
cv.InitUndistortMap(intrinsicR,distortionR,mapxR,mapyR)
cv.NamedWindow( "Undistort" )
print "all mapping completed"
print "Now relax for some time"
time.sleep(4)

print "now get ready, camera is switching on"
while(1):
	imageL=cv.QueryFrame(captureL)
	tL = cv.CloneImage(imageL);
	imageR=cv.QueryFrame(captureR)
	tR = cv.CloneImage(imageR);
	cv.ShowImage( "CalibrationL", imageL )
	cv.Remap( tL, imageL, mapxL, mapyL )
	cv.ShowImage("UndistortL", imageL)
	cv.ShowImage( "CalibrationR", imageR )
	cv.Remap( tR, imageR, mapxR, mapyR )
	cv.ShowImage("UndistortR", imageR)
	c = cv.WaitKey(33)
	if(c == 1048688):		# enter 'p' key to pause for some time
		cv.WaitKey(2000)
	elif c==1048603:		# enter esc key to exit
		break
		
print "everything is fine"

