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




cap = cv2.VideoCapture('outside-trimmed.mp4')
# cap = cv2.VideoCapture('drone-window.mp4')
i = 0
while(True):    
	 
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
	params.minArea = 5#20 #5
	params.filterByCircularity = True
	params.minCircularity = 0.7#0.85 # 0.7
	params.filterByConvexity = True
	params.minConvexity = 0.87#0.95 # 0.87	    
	params.filterByInertia = True
	params.minInertiaRatio = 0.01#0.8 #0.01
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
	
	if len(xList)==5:
		# print(rList)
		xListSorted = []
		xListSorted = np.append(xListSorted,sorted(xList,reverse=True))

		index = np.where(xList==xListSorted[2])
		i = index[0]
		yList[i]

		dist1 = round(int(math.sqrt( ((xListSorted[2]-xList[0])**2)+((yList[i]-yList[0])**2) )))
		dist2 = round(int(math.sqrt( ((xListSorted[2]-xList[1])**2)+((yList[i]-yList[1])**2) )))
		dist3 = round(int(math.sqrt( ((xListSorted[2]-xList[2])**2)+((yList[i]-yList[2])**2) )))
		dist4 = round(int(math.sqrt( ((xListSorted[2]-xList[3])**2)+((yList[i]-yList[3])**2) )))
		dist5 = round(int(math.sqrt( ((xListSorted[2]-xList[4])**2)+((yList[i]-yList[4])**2) )))

		print("d1:",dist1,"d2:",dist2,"d3:",dist3,"d4:",dist4,"d5:",dist5)
		
		cv2.putText(im_with_keypoints,str((dist1)),(round(int(xList[0])),round(int(yList[0]-25))),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),2)
		cv2.putText(im_with_keypoints,str((dist2)),(round(int(xList[1])),round(int(yList[1]-25))),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),2)
		cv2.putText(im_with_keypoints,str((dist3)),(round(int(xList[2])),round(int(yList[2]-25))),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),2)
		cv2.putText(im_with_keypoints,str((dist4)),(round(int(xList[3])),round(int(yList[3]-25))),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),2)
		cv2.putText(im_with_keypoints,str((dist5)),(round(int(xList[4])),round(int(yList[4]-25))),cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),2)
		
		# inputarray = [3,4,5]
		inputarray = [3,8,1,5,2]
		# inputarray = [2,10]

		FrameCenterPointX = int(w/2)
		FrameCenterPointY = int(h/2)
		startX = FrameCenterPointX
		startY = h
		endX = xListSorted[2]
		endY = int(yList[i])

		x1 = startX-endX
		y1 = startY-endY
		D = math.sqrt((x1)**2 + (y1)**2)
		sf = len(inputarray)
		units = sum(inputarray)*sf 							# sf = spacing factor
		du = D/units

		su = (units-sum(inputarray))/(sf+1) 				# su = space units
		print('D',D,'units',units,'du',du,'su',su)

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







