import cv2
import numpy as np
import serial
import time

# Connect to the ESP32
ser = serial.Serial(
    port = "/dev/ttyUSB0", 
    baudrate = 115200, 
    timeout = 1,
    xonxoff= False,
    rtscts= False,
    dsrdtr= False,
    write_timeout= 2)

time.sleep(0.5)

value = [0xFF, 0x00, 0x00, 0x00, 0x01, 0x00]
ser.write(serial.to_bytes(value))
print(serial.to_bytes(value))

def nil(self):
    pass

def dropTrophy():
    value = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x02]
    ser.write(serial.to_bytes(value))

    time.sleep(2)

    value = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x01]
    ser.write(serial.to_bytes(value))
    print(serial.to_bytes(value))   

    time.sleep(2)

    value = [0xFF, 0x00, 0x00, 0x00, 0x01, 0x01]
    ser.write(serial.to_bytes(value))
    print(serial.to_bytes(value))                 

    state5 = False

# Open the camera
cap = cv2.VideoCapture(0)
cap.set(3, 160)
cap.set(4, 120)

# yellow color
YHueL = 145
YHueH = 0
YSatL = 255
YSatH = 150
YValL = 255
YValH = 170

# blue color
BHueL = 255
BHueH = 0
BSatL = 140
BSatH = 0
BValL = 90
BValH = 0   

# White color
WHueL = 255
WHueH = 140
WSatL = 255
WSatH = 155
WValL = 255
WValH = 0

# Final state yellow color
YFHueL = 145
YFHueH = 0
YFSatL = 255
YFSatH = 115
YFValL = 255
YFValH = 120

# Final state blue color
BFHueL = 255
BFHueH = 70
BFSatL = 80
BFSatH = 0
BFValL = 50
BFValH = 0   

# times reach the blue color
counter = 0
bcounter = 0

# times reach the white color
state4counter = 0
wcounter = 0

# record the direction which the car will turn in State 3
directionR = False
directionL = False

# variables that if there are arrow scaned
pointSexist = False
pointEexist = False

# adjustment need to make when the state start
adjustState2 = False
adjustState2_2 = False
adjustState2_3 = False
adjustState4 = False

# variable to record the which state is at the moment
state1 = True
state2 = False
state3 = False
state4 = False
state5 = False

while True:
    success, frameY = cap.read()
    success, frameB = cap.read()
    success, inputImage = cap.read()
    success, frameW = cap.read()
    success, frameYF = cap.read()
    success, frameBF = cap.read()

    inputImage1 = inputImage

    # percent of original size (arrow need to be smaller)
    scale_percent = 90
    width = int(inputImage.shape[1] * scale_percent / 100)
    height = int(inputImage.shape[0] * scale_percent / 100)
    dim = (width, height)

    inputImage = cv2.resize(inputImage, dim, interpolation=cv2.INTER_AREA)

    # crop the image restrict the view of the car (line detection needed)
    Ycrop_img = frameY[60:120, 0:130]
    Bcrop_img = frameB[60:120, 0:130]

    # Find yellow line
    Ylow_b = np.uint8([YHueL, YSatL, YValL])
    Yhigh_b = np.uint8([YHueH, YSatH, YValH])
    Ymask = cv2.inRange(Ycrop_img, Yhigh_b, Ylow_b)
    Ycontours, Yhierarchy = cv2.findContours(Ymask, 1, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(Ycrop_img, Ycontours, -1, (0, 255, 0), 1)

    # Find blue line
    Blow_b = np.uint8([BHueL, BSatL, BValL])
    Bhigh_b = np.uint8([BHueH, BSatH, BValH])
    Bmask = cv2.inRange(Bcrop_img, Bhigh_b, Blow_b)
    Bcontours, Bhierarchy = cv2.findContours(Bmask, 1, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(Bcrop_img, Bcontours, -1, (0, 255, 0), 1)

    # Find black arrow
    blackLow_b = np.uint8([70, 60, 60])
    blackHigh_b = np.uint8([0, 0, 20])
    blackMask = cv2.inRange(inputImage, blackHigh_b, blackLow_b)
    blackcontours, blackhierarchy = cv2.findContours(blackMask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    # Find whtie color
    Wlow_b = np.array([WHueL, WSatL, WValL])
    Whigh_b = np.array([WHueH, WSatH, WValH])
    Wmask = cv2.inRange(inputImage, Whigh_b, Wlow_b)
    Wcontours, Whierarchy = cv2.findContours(Wmask, 1, cv2.CHAIN_APPROX_NONE)

    # Find yellow color in final state with larger sceen
    YFlow_b = np.uint8([YFHueL, YFSatL, YFValL])
    YFhigh_b = np.uint8([YFHueH, YFSatH, YFValH])
    YFmask = cv2.inRange((frameYF), YFhigh_b, YFlow_b)
    YFcontours, YFhierarchy = cv2.findContours(YFmask, 1, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(frameYF, YFcontours, -1, (0, 255, 0), 1)

    # Find yellow color in final state with larger sceen
    BFlow_b = np.uint8([BFHueL, BFSatL, BFValL])
    BFhigh_b = np.uint8([BFHueH, BFSatH, BFValH])
    BFmask = cv2.inRange((frameBF), BFhigh_b, BFlow_b)
    BFcontours, BFhierarchy = cv2.findContours(BFmask, 1, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(frameBF, BFcontours, -1, (0, 255, 0), 1)

    if state1 == True:
        if len(Ycontours) > 0:                                                  # detected yellow line               
            c = max(Ycontours, key=cv2.contourArea)                             # find the biggest contour
            M = cv2.moments(c)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print("CX: " + str(cx) + " CY: " + str(cy))

                cv2.line(Ycrop_img, (cx,0), (cx, 720), (255, 0, 0), 1)
                cv2.line(Ycrop_img, (0, cy), (1280, cy), (255, 0, 0), 1)

                # using the reference to control the car
                if cx >= 85 and cy >= 50:
                    value = [0xFF, 0x00, 0x00, 0x00, 0x01, 0x00]
                    ser.write(serial.to_bytes(value))
                    
                if cx > 45 and cx < 85:
                    value = [0xFF, 0x00, 0x40, 0x00, 0x01, 0x00]
                    ser.write(serial.to_bytes(value))

                if cx <= 30:
                    if cx <= 5 and cy >= 60:
                        value = [0xFF, 0x00, 0x00, 0xC0, 0x01, 0x00]  # turn right
                        ser.write(serial.to_bytes(value))

                    elif cx <= 10 and cy >= 50:
                        value = [0xFF, 0x48, 0xF9, 0x00, 0x01, 0x00]
                        ser.write(serial.to_bytes(value))
                    
                    elif cy >= 15:
                        value = [0xFF, 0x48, 0x09, 0x00, 0x01, 0x00]
                        ser.write(serial.to_bytes(value))

                    else:
                        value = [0xFF, 0x00, 0x00, 0x00, 0x01, 0x00]
                        ser.write(serial.to_bytes(value))

        else:
            value = [0xFF, 0x48, 0xF9, 0x00, 0x01, 0x00]
            ser.write(serial.to_bytes(value))


        if len(Bcontours) > 1:                                       # detected blue line

            Bc = max(Bcontours, key=cv2.contourArea)                 # find the biggest contour
            blueArea = cv2.contourArea(Bc)
            print("Blue area= " + str(blueArea))

            if blueArea >= 500 and blueArea <= 3000:                    # if match the criteria, start counter
                counter = 1

            if counter:
                counter += 1
                if counter == 3:
                    bcounter += 1
                    counter = 0

            if bcounter >= 2:                                            # if detected blue contours twice

                if blueArea >= 500 and blueArea <= 2200:
                    value = [0xff, 0x00, 0x00, 0x00, 0x01, 0x00]
                    ser.write(serial.to_bytes(value))

                    time.sleep(5)
                    state1 = False
                    state2 = True

            
    if state2 == True:

        if len(blackcontours) != 0:                                    # if black contour (arrow) is not detected
            blackc = max(blackcontours, key=cv2.contourArea)           # find the biggest contour (c) by the area
            blackarea = cv2.contourArea(blackc)
            print(blackarea)

            if blackarea >= 150:                                        # Stop when arrow is detected and the area is greater than 150
                value = [0xff, 0x00, 0x00, 0x00, 0x01, 0x00]
                ser.write(serial.to_bytes(value))
                print(serial.to_bytes(value))
                time.sleep(5)
                state2 = False
                state3 = True

        if len(Ycontours) > 0:                                          # move and follow the yellow line
            if (adjustState2_2 == True):
                adjustState2_3 = True    

            c = max(Ycontours, key=cv2.contourArea)
            M = cv2.moments(c)

            if M["m00"] != 0:
                s2cx = int(M["m10"] / M["m00"])
                s2cy = int(M["m01"] / M["m00"])
                print("S2CX: " + str(s2cx) + " S2CY: " + str(s2cy))

                cv2.line(Ycrop_img, (s2cx,0), (s2cx, 720), (255, 0, 0), 1)
                cv2.line(Ycrop_img, (0, s2cy), (1280, s2cy), (255, 0, 0), 1)

            if adjustState2 == False:                                    # adjust the car face to yellow line
                if s2cx >= 90:
                    value = [0xFF, 0x00, 0x00, 0x35, 0x01, 0x00]
                    ser.write(serial.to_bytes(value))
                    print('left')
                    print(serial.to_bytes(value))


                if s2cx > 55 and s2cx < 90:
                        adjustState2 = True

                if s2cx <= 55:
                    value = [0xFF, 0x00, 0x00, 0xCF, 0x01, 0x00]
                    ser.write(serial.to_bytes(value))
                    print(serial.to_bytes(value))
            
            if adjustState2_2 == False:                                        # adjustment move due to the uneven ground
                if s2cx >= 5 and s2cy >= 5:

                    value = [0xff, 0xC0, 0x05, 0x00, 0x01, 0x00]
                    ser.write(serial.to_bytes(value))

                else:
                    adjustState2_2 = True

            else:                                                               # move and follow the yellow line after adjustment move done
                if s2cx >= 90:
                    if s2cy >= 15:
                        value = [0xff, 0xC0, 0x05, 0x00, 0x01, 0x00]
                        ser.write(serial.to_bytes(value))
                        print(serial.to_bytes(value))


                elif s2cx > 55 and s2cx < 90:
                    value = [0xff, 0x00, 0x34, 0x00, 0x01, 0x00]
                    ser.write(serial.to_bytes(value))
                    print(serial.to_bytes(value))

                elif s2cx <= 20:
                        value = [0xff, 0x00, 0x34, 0x00, 0x01, 0x00]
                        ser.write(serial.to_bytes(value))
                        print(serial.to_bytes(value))

                elif s2cx <= 55 and s2cx >= 35:
                        value = [0xff, 0x00, 0x00, 0xCF, 0x01, 0x00]
                        ser.write(serial.to_bytes(value))
                        print(serial.to_bytes(value))
                    # else:
                    #     value = [0xff, 0x00, 0x00, 0xBF, 0x00, 0x00]
                    #     ser.write(serial.to_bytes(value))
                    #     print(serial.to_bytes(value))

                else:
                    value = [0xff, 0xC0, 0xF9, 0x00, 0x01, 0x00]
                    ser.write(serial.to_bytes(value))
                    print(serial.to_bytes(value))
        else:
            if adjustState2_3 == False:                                        # adjustment move due to the uneven ground
                value = [0xff, 0x00, 0x50, 0x00, 0x01, 0x00]
                ser.write(serial.to_bytes(value))
                print(serial.to_bytes(value))
            else:
                value = [0xff, 0x00, 0x50, 0x00, 0x01, 0x00]
                ser.write(serial.to_bytes(value))
                print(serial.to_bytes(value))
  
    if state3 == True:
 
        if blackarea >= 100:                                                    # if the detected arrow area is greater than 100 
            if pointSexist == False or pointEexist == False:
                if len(blackcontours) != 0:
                    # find the biggest contour (c) by the area
                    blackc = max(blackcontours, key=cv2.contourArea)
                    blackarea = cv2.contourArea(blackc)
                    print(blackarea)

                    # for i, c in enumerate(contours):
                        # Approximate the contour to a polygon
                    contoursPoly = cv2.approxPolyDP(blackc, 1, True)

                    # Create a new black background image to store the arrow
                    img = np.zeros((350, 700,1), dtype=np.uint8)
                    cv2.fillPoly(img, [contoursPoly], (255, 255, 255))

                    cv2.polylines(inputImage, [contoursPoly], True, (255, 0, 0), 2)

                    # Convert the polygon to a bounding rectangle
                    boundRect = cv2.boundingRect(contoursPoly)

                    # Get the Bounding rect's data:
                    rectX = boundRect[0]
                    rectY = boundRect[1]
                    rectWidth = boundRect[2]
                    rectHeight = boundRect[3]

                    # Get the rect's area:
                    rectArea = rectWidth * rectHeight
                    print("rectArea= " + str(rectArea))

                    miniBlobArea = 200
                    
                    # Check if blob is above min area
                    if rectArea > miniBlobArea:
                        # Crop the roi:
                        croppedImg = img[rectY:rectY + rectHeight, rectX: rectX + rectWidth]

                        # Extend the borders for the skeleton
                        borderSize = 10
                        croppedImg = cv2.copyMakeBorder(croppedImg, borderSize, borderSize, borderSize, borderSize,
                                                        cv2.BORDER_CONSTANT)
                        
                        # Store a deep copy of the crop for results
                        grayscaleImageCopy = cv2.cvtColor(croppedImg, cv2.COLOR_GRAY2BGR)

                        # Compute the skeleton
                        skeleton = cv2.ximgproc.thinning(croppedImg, None, 1)

                        _, binartImage = cv2.threshold(skeleton, 128, 10, cv2.THRESH_BINARY)

                        # Sert the end-points kernel:
                        h = np.array([[1, 1, 1],
                                    [1, 10, 1],
                                    [1, 1, 1]])
                        
                        # Convolve the image with the kernel
                        imgFiltered = cv2.filter2D(binartImage, -1, h)

                        # Extract only the end-points pixels, those with an intensity value of 110
                        binartImage = np.where(imgFiltered == 110, 255, 0)

                        # The above operation converted the image to 32-bit float
                        # convert back to 8-bit uint
                        binartImage = binartImage.astype(np.uint8)

                        # Find the X, Y location of all the end-points
                        Y, X = binartImage.nonzero()

                        # Check if i got points on my arrays
                        if len(X) > 0 or len(Y) > 0:
                            # Resharp the arrays for K-means
                            Y = Y.reshape(-1, 1)
                            X = X.reshape(-1, 1)
                            Z = np.hstack((X, Y))

                            # k-means operaters on 32-bit float data
                            floatPoints = np.float32(Z)

                            # Set the convergence criteria and call k-means
                            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
                            _, label, center = cv2.kmeans(floatPoints, 2 , None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

                            # Set the cluster count, find the points belonging
                            # ro cluster 0 and cluster 1
                            cluster1Count = np.count_nonzero(label)
                            cluster0Count = np.shape(label)[0] - cluster1Count

                            # Look for the cluster of max number of points
                            # That cluster will be the tip of the arrow
                            maxCluster = 0
                            if cluster1Count > cluster0Count:
                                maxCluster = 1

                            # Check out the centers of each cluster
                            matRows, matCols = center.shape

                            # We need at least 2 points for this operation
                            if matCols >= 2:
                                # Store the ordered end-points here:
                                orderedPoints = [None] * 2

                                # Let's identify and draw the two end-points of the arrow
                                for b in range(matRows):
                                    # Get cluster center:
                                    pointX = int(center[b][0])
                                    pointY = int(center[b][1])

                                    if b == maxCluster:
                                        color = (0, 0, 255)
                                        orderedPoints[0] = (pointX, pointY)
                                        startpointX = pointX
                                        startpointY = pointY
                                        pointSexist = True

                                    # Get the "tail"
                                    else:
                                        color = (255, 0,0)
                                        orderedPoints[1] = (pointX, pointY)
                                        endpointX = pointX
                                        endpointY = pointY
                                        pointEexist = True

                                    # Draw tip and tail
                                    cv2.circle(grayscaleImageCopy, (pointX, pointY), 3, color, -1)

                                cv2.imshow("Direction of arrow", grayscaleImageCopy)
                                cv2.imshow('skelton', skeleton)
                                cv2.imshow('imFiltered', imgFiltered)    


                                if pointSexist == True and pointEexist == True:                        # prevent duplicate action
                                    value = [0xff, 0x00, 0x00, 0x00, 0x01, 0x00]
                                    ser.write(serial.to_bytes(value))
                                    print(serial.to_bytes(value))
                                    if startpointX > endpointX:
                                        directionR = True
                                        print("Direction: Right")

                                    else:
                                        directionL = True
                                        print("Direction: Right")
                                
                                    state3 = False
                                    state4 = True




    if state4 == True:
        if adjustState4 == False:                                            # turn the car in the correct driection
            if len(Ycontours)> 0:
                yc = max(Ycontours, key=cv2.contourArea)
                yM = cv2.moments(yc)
                State4Yarea = cv2.contourArea(yc)
                print("Area: " + str(State4Yarea))

                if State4Yarea >= 900:
                    adjustState4 = True

                else:
                    if directionR == True:
                        value = [0xFF, 0x00, 0x00, 0x45, 0x01, 0x00]
                        ser.write(serial.to_bytes(value))
                        print(serial.to_bytes(value))

                    if directionL == True:
                        value = [0xFF, 0x00, 0x00, 0xC5, 0x01, 0x00]
                        ser.write(serial.to_bytes(value))
                        print(serial.to_bytes(value))

        else:
            if len(Wcontours)> 0:                                                # if trohpy is detected
                wc = max(Wcontours, key=cv2.contourArea)
                wM = cv2.moments(wc)
                State4Warea = cv2.contourArea(wc)
                print("WArea: " + str(State4Warea))

                if State4Warea >= 800:                                            # if the detected trophy area is greater than 800, start the counter
                    state4counter = 1

                if state4counter:
                    state4counter += 1
                    if state4counter == 3:
                        wcounter += 1
                        state4counter = 0

            if len(Ycontours)> 0:                                                # move until the car in front of the trophy
                yc = max(Ycontours, key=cv2.contourArea)
                yM = cv2.moments(yc)
                State4Yarea = cv2.contourArea(yc)
                print("YArea: " + str(State4Yarea))

                if wcounter < 1:
                    if directionR == True:
                       if yM["m00"] != 0:
                            s4cx = int(yM["m10"] / yM["m00"])
                            s4cy = int(yM["m01"] / yM["m00"])
                            print("S4CX: " + str(s4cx) + " S4CY: " + str(s4cy))

                            cv2.line(Ycrop_img, (s4cx,0), (s4cx, 720), (255, 0, 0), 1)
                            cv2.line(Ycrop_img, (0, s4cy), (1280, s4cy), (255, 0, 0), 1)

                            if s4cx >= 87:
                                value = [0xFF, 0x00, 0x00, 0x34, 0x01, 0x00]
                                ser.write(serial.to_bytes(value))
                                print('left')
                                print(serial.to_bytes(value))


                            if s4cx > 58 and s4cx < 87:
                                value = [0xFF, 0x00, 0x37, 0x00, 0x01, 0x00]
                                ser.write(serial.to_bytes(value))
                                print(serial.to_bytes(value))

                            if s4cx <= 58:
                                value = [0xFF, 0x00, 0x00, 0xCE, 0x01, 0x00]
                                ser.write(serial.to_bytes(value))
                                print(serial.to_bytes(value))

                    else:
                        if yM["m00"] != 0:
                            s4cx = int(yM["m10"] / yM["m00"])
                            s4cy = int(yM["m01"] / yM["m00"])
                            print("S4CX: " + str(s4cx) + " S4CY: " + str(s4cy))

                            cv2.line(Ycrop_img, (s4cx,0), (s4cx, 720), (255, 0, 0), 1)
                            cv2.line(Ycrop_img, (0, s4cy), (1280, s4cy), (255, 0, 0), 1)

                            if s4cx >= 87:
                                value = [0xFF, 0x00, 0x00, 0x34, 0x01, 0x00]
                                ser.write(serial.to_bytes(value))
                                print('left')
                                print(serial.to_bytes(value))


                            if s4cx > 58 and s4cx < 87:
                                value = [0xFF, 0x00, 0x37, 0x00, 0x01, 0x00]
                                ser.write(serial.to_bytes(value))
                                print(serial.to_bytes(value))

                            if s4cx <= 58:
                                value = [0xFF, 0x00, 0x00, 0xCE, 0x01, 0x00]
                                ser.write(serial.to_bytes(value))
                                print(serial.to_bytes(value))


            if wcounter >= 1:                                                    # if the trophy is in front of the car, the counter should be 1
                value = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00]
                ser.write(serial.to_bytes(value))
                print(serial.to_bytes(value))

                time.sleep(2)
                value = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x02]
                ser.write(serial.to_bytes(value))
                print(serial.to_bytes(value))
                
                state4 = False
                state5 = True

    if state5 == True:

        if directionL == True:

            if len(BFcontours) <= 0:

                if len(YFcontours) > 0:                                                            # move according to the yellow border line until meet the blue contour
                    YFc = max(YFcontours, key=cv2.contourArea)
                    YFM = cv2.moments(YFc)

                    if YFM["m00"] != 0:
                        s5cx = int(YFM["m10"] / YFM["m00"])
                        s5cy = int(YFM["m01"] / YFM["m00"])
                        print("S5CX: " + str(s5cx) + " S5CY: " + str(s5cy))

                        cv2.line(frameYF, (s5cx,0), (s5cx, 720), (255, 0, 0), 1)
                        cv2.line(frameYF, (0, s5cy), (1280, s5cy), (255, 0, 0), 1)

                        if s5cx <80 and s5cx >= 45:
                            value = [0xFF, 0xC0, 0x00, 0x00, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))
                            print(serial.to_bytes(value))

                        elif s5cx >= 80:
                            value = [0xFF, 0x00, 0x00, 0x35, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))
                            print(serial.to_bytes(value))

                        elif s5cx < 45 and s5cx >= 15:
                            value = [0xFF, 0x00, 0x00, 0x35, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))
                            print(serial.to_bytes(value))

                        elif s5cx < 15:
                            value = [0xFF, 0x00, 0x00, 0x35, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))
                            print(serial.to_bytes(value))

                        elif s5cy >= 45 and s5cy <= 60:
                            value = [0xFF, 0xC0, 0x00, 0x00, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))
                            print(serial.to_bytes(value))

                        elif s5cy >60 and s5cy <= 85:
                            value = [0xFF, 0x00, 0xC0, 0x00, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))
                            print(serial.to_bytes(value))

                        else:
                            value = [0xFF, 0x00, 0x00, 0x35, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))
                            print(serial.to_bytes(value))

            else:                                                                                  # move and drop the trophy if the criteria is met
                FBc = max(BFcontours, key=cv2.contourArea)                                       
                FblueArea = cv2.contourArea(FBc)
                # Bx, By, Bw, Bh = cv2.boundingRect(Bc)
                print("Blue area= " + str(FblueArea))

                BFM = cv2.moments(FBc)

                if BFM["m00"] != 0:
                    s5Bcx = int(BFM["m10"] / BFM["m00"])
                    s5Bcy = int(BFM["m01"] / BFM["m00"])
                    print("S5BCX: " + str(s5Bcx) + " S5BCY: " + str(s5Bcy))

                    cv2.line(frameYF, (s5Bcx,0), (s5Bcx, 720), (255, 0, 0), 1)
                    cv2.line(frameYF, (0, s5Bcy), (1280, s5Bcy), (255, 0, 0), 1)

                    if s5Bcx >= 90 and s5Bcx <= 160:
                        value = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))
                        print(serial.to_bytes(value))

                        if s5Bcy >= 30:

                            value = [0xFF, 0x00, 0x00, 0x35, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))

                    elif s5Bcx >= 80 and s5Bcx < 90:

                        if (s5Bcy >= 65):
                            value = [0xFF, 0x00, 0x00, 0x35, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))

                    elif (FblueArea <= 100 and FblueArea > 60):
                            value = [0xFF, 0x00, 0x35, 0x00, 0x00, 0x02]
                            ser.write(serial.to_bytes(value))

                    elif (FblueArea < 1 and FblueArea > 0 and s5Bcx <= 40):
                        value = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (FblueArea > 750):
                            if (s5Bcx >= 70):
                                value = [0xFF, 0x00, 0x35, 0x00, 0x00, 0x02]
                                ser.write(serial.to_bytes(value))
                            else:
                                value = [0xFF, 0x00, 0x00, 0x35, 0x00, 0x02]
                                ser.write(serial.to_bytes(value))

                    elif (s5Bcx <= 10 and s5Bcy >= 100):
                        value = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (s5Bcx <= 20 and s5Bcy >= 90 and FblueArea <= 200):
                         dropTrophy()

                    
                    elif (s5Bcx <= 50 and s5Bcy >= 85 and FblueArea <= 120 and FblueArea >= 20):
                        dropTrophy()

                    elif (s5Bcx <= 100 and s5Bcy >= 85 and FblueArea <= 120 and FblueArea >= 20):
                        dropTrophy()

                    else:
                        value = [0xFF, 0x00, 0x35, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

        if directionR == True:
            if len(BFcontours) <= 0:

                if len(YFcontours) > 0:
                    YFc = max(YFcontours, key=cv2.contourArea)
                    YFM = cv2.moments(YFc)

                    if YFM["m00"] != 0:
                        s5cx = int(YFM["m10"] / YFM["m00"])
                        s5cy = int(YFM["m01"] / YFM["m00"])
                        print("S5CX: " + str(s5cx) + " S5CY: " + str(s5cy))

                        cv2.line(frameYF, (s5cx,0), (s5cx, 720), (255, 0, 0), 1)
                        cv2.line(frameYF, (0, s5cy), (1280, s5cy), (255, 0, 0), 1)

                    if s5cx >= 60 and s5cx <80 and s5cy >= 40 and s5cy < 70:
                        value = [0xFF, 0x38, 0x00, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (s5cx <= 55 and s5cy >= 50):
                        value = [0xFF, 0x00, 0x00, 0xC0, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (s5cx >= 80 and s5cy >= 70):
                        value = [0xFF, 0x00, 0x00, 0xC0, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (s5cx >= 70 and s5cy <= 30):
                        value = [0xFF, 0x00, 0x00, 0xC0, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (s5cx <= 10 and s5cy <= 50):
                        value = [0xFF, 0x00, 0x00, 0xC0, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    else:
                        value = [0xFF, 0x38, 0x00, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

            else:
                FBc = max(BFcontours, key=cv2.contourArea)
                FblueArea = cv2.contourArea(FBc)
                # Bx, By, Bw, Bh = cv2.boundingRect(Bc)
                print("Blue area= " + str(FblueArea))

                BFM = cv2.moments(FBc)

                if BFM["m00"] != 0:
                    s5Bcx = int(BFM["m10"] / BFM["m00"])
                    s5Bcy = int(BFM["m01"] / BFM["m00"])
                    print("S5BCX: " + str(s5Bcx) + " S5BCY: " + str(s5Bcy))

                    cv2.line(frameYF, (s5Bcx,0), (s5Bcx, 720), (255, 0, 0), 1)
                    cv2.line(frameYF, (0, s5Bcy), (1280, s5Bcy), (255, 0, 0), 1)

                    if (s5Bcx <= 15 and s5Bcy <= 45):
                        value = [0xFF, 0x40, 0x00, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (s5Bcx <= 20 and s5Bcx >5 and s5Bcy <= 100 and s5Bcy >= 98 and FblueArea <= 370):            
                        value = [0xFF, 0x00, 0x35, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (s5Bcx <= 20 and s5Bcx >5 and s5Bcy <= 100 and s5Bcy >= 80):            
                        value = [0xFF, 0x00, 0x35, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (s5Bcx <= 20 and s5Bcx >5 and s5Bcy <= 100 and s5Bcy >= 80):            
                        value = [0xFF, 0x00, 0x35, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    elif (s5Bcx <= 15 and s5Bcx >5 and s5Bcy <= 107 and FblueArea <= 150):
                        dropTrophy()

                    elif (s5Bcx <= 20 and s5Bcx >5 and s5Bcy <= 100 and s5Bcy >= 98 and FblueArea <= 370):
                        dropTrophy()

                    elif (s5Bcx >= 145 and s5Bcy >= 95 and FblueArea <= 300):
                        dropTrophy()

                    elif (s5Bcx <= 136 and s5Bcx >= 10 and s5Bcy >= 10 and s5Bcy <= 110 and FblueArea <= 100 and FblueArea >= 10):
                        dropTrophy()

                    elif (s5Bcx >= 23 and s5Bcx < 150 and s5Bcy >= 5 and s5Bcy < 120):
                        value = [0xFF, 0x00, 0x35, 0x00, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

                    else:
                        value = [0xFF, 0x00, 0x00, 0xC0, 0x00, 0x02]
                        ser.write(serial.to_bytes(value))

    if cv2.waitKey (1) & 0xff == 27:
        break



cap.release()
cv2.destroyAllWindows
