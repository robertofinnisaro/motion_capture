#!/usr/bin/env python3

import cv2
import numpy as np


def cvshow(name, img):
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.destroyAllWindows()


def sift_kp(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    sift = cv2.SIFT_create()
    sift = cv2.SIFT_create()
    kp, des = sift.detectAndCompute(image, None)
    kp_image = cv2.drawKeypoints(gray_image, kp, None)
    return kp_image, kp, des


def get_good_match(des1, des2):
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1, des2, k=2)  # des1 is the template image, des2 is the matching image
    matches = sorted(matches, key=lambda x: x[0].distance / x[1].distance)
    good = []
    for m, n in matches:
        if m.distance < 0.55 * n.distance:
            good.append(m)
    return good


def drawMatches(imageA, imageB, kpsA, kpsB, matches, status):
    # Initialize the visualization picture, connect the A and B pictures left and right together
    (hA, wA) = imageA.shape[:2]
    (hB, wB) = imageB.shape[:2]
    vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
    vis[0:hA, 0:wA] = imageA
    vis[0:hB, wA:] = imageB

    # Joint traversal, draw matching pairs
    for ((trainIdx, queryIdx), s) in zip(matches, status):
        # When the point pair is matched successfully, draw it on the visualization
        if s == 1:
            # Draw matching pairs
            ptA = (int(kpsA[queryIdx][0]), int(kpsA[queryIdx][1]))
            ptB = (int(kpsB[trainIdx][0]) + wA, int(kpsB[trainIdx][1]))
            cv2.line(vis, ptA, ptB, (0, 255, 0), 1)

    # Return visualization results
    return vis


# Panorama stitching
def siftimg_rightlignment(img_right, img_left):
    _, kp1, des1 = sift_kp(img_right)
    _, kp2, des2 = sift_kp(img_left)
    goodMatch = get_good_match(des1, des2)
    # When the matching pairs of the filter items are greater than 4 pairs: calculate the perspective transformation matrix
    if len(goodMatch) > 4:
        # Get the point coordinates of the matching pair
        ptsA = np.float32([kp1[m.queryIdx].pt for m in goodMatch]).reshape(-1, 1, 2)
        ptsB = np.float32([kp2[m.trainIdx].pt for m in goodMatch]).reshape(-1, 1, 2)
        ransacReprojThreshold = 4
        H, status = cv2.findHomography(ptsA, ptsB, cv2.RANSAC, ransacReprojThreshold)

        print(H)
        #H = np.array([[-3.95002617e-01,-7.49813070e-02, 4.43642683e+02], [-4.06655962e-01,5.27365057e-01, 1.20636875e+02],[-1.60149798e-03, -3.69708507e-05, 1.00000000e+00]])

        # The function of this function is to first use RANSAC to select the best four sets of pairing points, and then calculate the H matrix. H is a 3*3 matrix

        # Change the angle of view to the right of the picture, result is the transformed picture
        result = cv2.warpPerspective(img_right, H, (img_right.shape[1] + img_left.shape[1], img_right.shape[0]))
        cvshow('result_medium', result)
        # Pass the picture left to the left end of the result picture
        result[0:img_left.shape[0], 0:img_left.shape[1]] = img_left
        return result
