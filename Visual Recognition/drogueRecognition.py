import cv2
import numpy as np
import math
#drone.m4v

def moveUnits(units):	
	h2 = units*du
	x2 = (h2/D) * x1
	y2 = (h2/D) * y1

	newcorX = int(round(startX - x2))
	newcorY = int(round(startY - y2))

	return newcorX,newcorY




# cap = cv2.VideoCapture('drogue.m4v')
cap = cv2.VideoCapture('newDrogue-trim.mp4')
i = 0
while(True):    
	
	inputs = 3 

	ret, image = cap.read()	
	if(ret==False):
		break

	frame2 = cv2.resize(image,None,fx=0.5,fy=0.5)		
	h, w, c = frame2.shape

	boundaries = [ ([250, 250, 250], [255, 255, 255]) ]
	for (lower, upper) in boundaries:
		lower = np.array(lower, dtype = "uint8")
		upper = np.array(upper, dtype = "uint8")
		mask = cv2.inRange(frame2, lower, upper)
		output2 = cv2.bitwise_and(frame2, frame2, mask = mask)		

	output = np.invert(output2)
	params = cv2.SimpleBlobDetector_Params()
	params.minThreshold = 10
	params.maxThreshold = 250 
	params.filterByArea = True
	params.minArea = 100 
	params.filterByCircularity = True
	params.minCircularity = 0.7
	params.filterByConvexity = True
	params.minConvexity = 0.67	    
	params.filterByInertia = True
	params.minInertiaRatio = 0.01
	detector = cv2.SimpleBlobDetector_create(params)
	keypoints = detector.detect(output)
	xList = []
	yList = []
	rList = []
	print("======================================")
	for kp in keypoints:
	    x = kp.pt[0]
	    y = kp.pt[1]
	    d = kp.size #diameter 
	    # print("x: ",x,"\ty: ",y, "\td: ",d)
	    xList = np.append(xList,round(int(x)))
	    yList = np.append(yList,round(int(y)))
	    rList = np.append(rList,round(int(d/2)))
	
	im_with_keypoints = cv2.drawKeypoints(frame2, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
	
	if len(xList)==6:
		# print(rList)
		print(xList)
		print(yList)

		RightPointX = max(xList)
		RightPointY = yList[xList.argmax()]
		print(RightPointX, RightPointY)

		LeftPointX = min(xList)
		LeftPointY = yList[xList.argmin()]
		print(LeftPointX, LeftPointY)
	
		TopPointY = min(yList)
		TopPointX = xList[yList.argmin()]
		print(TopPointX, TopPointY)

		BottomPointY = max(yList)
		BottomPointX = xList[yList.argmax()]
		print(BottomPointX, BottomPointY)
	
		CenterPointX = round(int(((((RightPointX+LeftPointX)/2) + ((TopPointX+BottomPointX)/2))/2)))
		CenterPointY = round(int(((((RightPointY+LeftPointY)/2) + ((TopPointY+BottomPointY)/2))/2)))

		print(CenterPointX)
		print(CenterPointY)

		FrameCenterPointX = int(w/2)
		FrameCenterPointY = int(h/2)
		
		cv2.putText(im_with_keypoints,'X',(CenterPointX,CenterPointY),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,255,255),2)

		# cv2.putText(im_with_keypoints,'o',(FrameCenterPointX,h),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,255,255),2)
		# im_with_keypoints = cv2.line(im_with_keypoints,(FrameCenterPointX,h),(CenterPointX,CenterPointY),(255,255,255),1)

		# inputarray = [3,4,5]
		inputarray = [3,8,1,5,2]
		# inputarray = [2,10]

		startX = FrameCenterPointX
		startY = h
		endX = CenterPointX
		endY = CenterPointY

		x1 = startX-endX
		y1 = startY-endY
		D = math.sqrt((x1)**2 + (y1)**2)
		sf = len(inputarray)
		units = sum(inputarray)*sf 							# sf = spacing factor
		du = D/units

		su = (units-sum(inputarray))/(sf+1) 				# su = space units
		print('D',D,'units',units,'du',du,'su',su)
		# print('->',su)

		print('<<<<<<<<<<<<<-------')
		h = len(inputarray)
		g = 1
		for arrindex in inputarray:
			print('g:',g,'SS:',sum(inputarray[h:len(inputarray)]),'LS:',sum(inputarray[h-1:len(inputarray)]))
			newcorXS, newcorYS = moveUnits(g*su + sum(inputarray[h:len(inputarray)]))		# Moving Spaces
			newcorXL, newcorYL = moveUnits(g*su + sum(inputarray[h-1:len(inputarray)]))		# Moving Lines
			g = g+1
			h = h-1
			im_with_keypoints = cv2.line(im_with_keypoints,(newcorXS,newcorYS),(newcorXL,newcorYL),(0,255,0),3)

		print('<<<<<<<<<<<<<-------')



	else:
		print("LED Count is NOT 5")
	
	cv2.imshow("Keypoints", im_with_keypoints)
	cv2.imshow("output", output)

	if cv2.waitKey(0) & 0xFF == ord('q'):
		break







