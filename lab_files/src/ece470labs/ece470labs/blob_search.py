#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration (fill in after running the calibration in Appendix B)
theta = 0.0   # radians, rotation between camera frame and world frame
beta = 0.0    # pixels per meter
tx = 0.0      # meters, world-frame origin expressed in the camera frame (x)
ty = 0.0      # meters, world-frame origin expressed in the camera frame (y)
 
# Principal point: center of the 640 x 480 image
Or = 240.0    # row
Oc = 320.0    # column
 
# HSV thresholds per color (OpenCV: H 0-180, S 0-255, V 0-255).
# These are starting points -- tune them under your lab's lighting.
HSV_RANGES = {
    "green":  ((40, 70, 50),   (80, 255, 255)),
    "yellow": ((20, 100, 100), (35, 255, 255)),
    "orange": ((5, 120, 120),  (18, 255, 255)),   # calibration stick targets
    "blue":   ((110, 50, 50),  (130, 255, 255)),  # original template color
}
 
_warned_uncalibrated = False
 
# Function that converts image coord to world coord
def IMG2W(col, row):
    """Pixel (col, row) -> world (xw, yw) in meters.
    Note: keypoint.pt is (x, y) = (col, row), which is why col comes first.
    """
    global _warned_uncalibrated
    if beta == 0.0:
        if not _warned_uncalibrated:
            print("IMG2W: beta is 0 -- camera not calibrated yet, returning pixel coords.")
            _warned_uncalibrated = True
        return (col, row)
 
    # Intrinsic: pixels -> meters in the camera frame on the tabletop
    xc = (row - Or) / beta
    yc = (col - Oc) / beta
 
    # Extrinsic: p_w = R_cw^T (p_c - O_cw)
    dx = xc - tx
    dy = yc - ty
    xw = np.cos(theta) * dx + np.sin(theta) * dy
    yw = -np.sin(theta) * dx + np.cos(theta) * dy
    return (float(xw), float(yw))
 
 
def compute_calibration(p1, p2, p_known, world_known):
    """Helper to get beta, theta, tx, ty from measured pixel centroids.
 
    p1, p2      : (col, row) of the two orange circles with the stick lined up
                  along the world y-axis; p2 is the circle farther along +y_w.
    p_known     : (col, row) of a single target placed at a known world point.
    world_known : (xw, yw) of that target in meters.
 
    Prints and returns (beta, theta, tx, ty). Copy the values into the
    constants at the top of this file.
    """
    c1, r1 = p1
    c2, r2 = p2
    b = np.hypot(r2 - r1, c2 - c1) / 0.10          # circles are 10 cm apart
    th = np.arctan2(-(r2 - r1), c2 - c1)
 
    c0, r0 = p_known
    x0, y0 = world_known
    xc0 = (r0 - Or) / b
    yc0 = (c0 - Oc) / b
    t_x = xc0 - (np.cos(th) * x0 - np.sin(th) * y0)
    t_y = yc0 - (np.sin(th) * x0 + np.cos(th) * y0)
 
    print(f"beta = {b:.4f}\ntheta = {th:.6f}\ntx = {t_x:.6f}\nty = {t_y:.6f}")
    return b, th, t_x, t_y
# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color: blobs are white (255) in the mask
    params.filterByColor = True
    params.blobColor = 255
 
    # Filter by Area: rejects specks of noise and large regions.
    # Tune to how many pixels one block covers in your camera image.
    params.filterByArea = True
    if color == "orange":
        params.minArea = 50       # calibration circles are smaller than blocks
        params.maxArea = 2000
    else:
        params.minArea = 150
        params.maxArea = 2500
 
    # Filter by Circularity
    params.filterByCircularity = False
 
    # Filter by Inerita
    params.filterByInertia = False
 
    # Filter by Convexity
    params.filterByConvexity = False
 
    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    lower, upper = HSV_RANGES.get(color, HSV_RANGES["blue"])
 
    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)
 
    # Optional cleanup: removes small speckles and fills small holes in the blobs
    kernel = np.ones((3, 3), np.uint8)
    mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_OPEN, kernel)
    mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_CLOSE, kernel)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block (red circle sized to the blob)
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, np.array([]), (0, 0, 255),
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # Mark the centroid itself with a dot
    for (c, r) in blob_image_center:
        cv2.circle(im_with_keypoints, (int(round(c)), int(round(r))), 3, (0, 0, 255), -1)
 
    # Pixel centroids are what you need for calibration
    if color == "orange":
        print("orange centroids (col, row):", [(round(c, 1), round(r, 1)) for c, r in blob_image_center])

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    # cv2.namedWindow("Keypoint View")
    # cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
