import cv2 as cv
import numpy as np


def scale(img, scaleNum):
    return cv.resize(img, (int(img.shape[1] * scaleNum), int(img.shape[0] * scaleNum)))


def approxDraw(inputImage, outputImage, areaReq=500):
    img = inputImage
    drawImage = outputImage.copy()
    contours, hierarchy = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cones = []

    for cnt in contours:
        if cv.contourArea(cnt) > areaReq:
            epsilon = .002 * cv.arcLength(cnt, True)
            approx = cv.approxPolyDP(cnt, epsilon, True)
            cv.drawContours(drawImage, [approx], 0, (122, 92, 0), 3)
            cones.append(approx)

    return drawImage, cones


def isCone(inputImage, refCone):
    objects = []
    matchPercentages = []

    hsvImg = cv.cvtColor(inputImage, cv.COLOR_BGR2HSV)
    threshold = cv.inRange(hsvImg, blue_lower, blue_upper)
    contourImg, objects = approxDraw(threshold, inputImage)

    cv.imshow("contours", contourImg)
    for cnt in range(len(objects)):
        match = cv.matchShapes(objects[cnt], refCone, 1, 0.0)
        matchPercentages.append(match)

    return matchPercentages


# blue_lower = np.array([55, 20, 55])
blue_lower = np.array([62, 62, 81])
blue_upper = np.array([179, 255, 255])
refBlue = scale(cv.imread('blueCone.png'), 0.5)
blueCone = scale(cv.imread('ConeBlue.png'), 0.5)
coneRef = []
cones = []
matches = []

# cv.imshow('refBlue', hsvPic)
hsvPic = cv.cvtColor(refBlue, cv.COLOR_BGR2HSV)
thresh = cv.inRange(hsvPic, blue_lower, blue_upper)
coneContour, coneRef = approxDraw(thresh, refBlue)

matches = isCone(blueCone, coneRef[0])

print(matches)

cv.imshow('contour', coneContour)
cv.imshow('thresh', thresh)

cv.waitKey(0)
