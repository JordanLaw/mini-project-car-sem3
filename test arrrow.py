import cv2
import numpy as np
import math

video_capture = cv2.VideoCapture(0)
video_capture.set(3, 160)
video_capture.set(4, 120)

def nil(self):
    pass

# Trackbar for finding the right color
cv2.namedWindow('TrackBar')
cv2.createTrackbar('HueLow', 'TrackBar', 0, 255, nil)
cv2.createTrackbar('SatLow', 'TrackBar', 0, 255, nil)
cv2.createTrackbar('ValLow', 'TrackBar', 0, 255, nil)
cv2.createTrackbar('HueHigh', 'TrackBar', 0, 255, nil)
cv2.createTrackbar('SatHigh', 'TrackBar', 0, 255, nil)
cv2.createTrackbar('ValHigh', 'TrackBar', 0, 255, nil)


pointE = False
pointS = False

# Reading an image

# inputImage = cv2.imread("/home/jetson/python project/TESTR1.JPEG")

# scale_percent = 50
# width = int(inputImage.shape[1] * scale_percent / 100)
# height = int(inputImage.shape[0] * scale_percent / 100)
# dim = (width, height)

# inputImage = cv2.resize(inputImage, dim, interpolation=cv2.INTER_AREA)

while True:

    ret, inputImage1 = video_capture.read()
    inputImage = inputImage1

    # percent of original size
    scale_percent = 90
    width = int(inputImage.shape[1] * scale_percent / 100)
    height = int(inputImage.shape[0] * scale_percent / 100)
    dim = (width, height)

    inputImage = cv2.resize(inputImage, dim, interpolation=cv2.INTER_AREA)

    BlackHueL = cv2.getTrackbarPos('HueLow', 'TrackBar')
    BlackHueH = cv2.getTrackbarPos('HueHigh', 'TrackBar')
    BlackSatL = cv2.getTrackbarPos('SatLow', 'TrackBar')
    BlackSatH = cv2.getTrackbarPos('SatHigh', 'TrackBar')
    BlackValL = cv2.getTrackbarPos('ValLow', 'TrackBar')
    BlackValH = cv2.getTrackbarPos('ValHigh', 'TrackBar')

    blackLow_b = np.uint8([BlackHueL, BlackSatL, BlackValL])
    blackHigh_b = np.uint8([BlackHueH, BlackSatH, BlackValH])

    blackLow_b = np.uint8([70, 60, 60])
    blackHigh_b = np.uint8([0, 0, 15])

    blackMask = cv2.inRange(inputImage, blackHigh_b, blackLow_b)
    
    # Find the big contours/ blobs on the binary image:
    contours, hierarchy = cv2.findContours(blackMask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) != 0:
        # find the biggest contour (c) by the area
        blackc = max(contours, key=cv2.contourArea)
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
                            pointS = True

                        # Get the "tail"
                        else:
                            color = (255, 0,0)
                            orderedPoints[1] = (pointX, pointY)
                            endpointX = pointX
                            endpointY = pointY
                            pointE = True

                        # Draw tip and tail
                        cv2.circle(grayscaleImageCopy, (pointX, pointY), 3, color, -1)
                        cv2.imshow("Direction of arrow", grayscaleImageCopy)
                        cv2.imshow('skelton', skeleton)
                        cv2.imshow('imFiltered', imgFiltered)

                        print("PointS: " + str(pointS))
                        print("PointE: " + str(pointE))

                    
    cv2.imshow('cam', inputImage)
    cv2.imshow('camO', inputImage1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows










