from __future__ import print_function
import os
import numpy as np
from sklearn.preprocessing import normalize
import cv2
import matplotlib.pyplot as plt
import open3d as o3d

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    #github lol
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

def main():
    window_size = 3
    left_matcher = cv2.StereoSGBM_create(
                        minDisparity = -39,
                        numDisparities = 144,
                        blockSize = 5,
                        P1 = 8 * 3 * window_size ** 2,
                        P2 = 32 * 3 * window_size ** 2,
                        disp12MaxDiff = 1,
                        uniquenessRatio = 10,
                        speckleWindowSize = 100,
                        speckleRange = 32,
                        preFilterCap = 63,
                        mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY
                        )

    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    wls_filter.setLambda(80000)
    wls_filter.setSigmaColor(1.3)

    for i in range(60,81):
        #Paths may differ
        imgL = cv2.imread( "./img2/img2/" + "00000004" + str(i) + ".png" , 0)
        imgR = cv2.imread( "./img3/img3/" + "00000004" + str(i) + ".png"  , 0)
        #greyscale images

        imgL_c = cv2.imread("img2/img2/" + "00000004" + str(i) + ".png" )
         # colour Image

        left_disp = left_matcher.compute(imgL,imgR).astype(np.float32)
        right_disp = right_matcher.compute(imgR,imgL).astype(np.float32)
        left_disp = np.int16(left_disp)
        right_disp = np.int16(right_disp)
        #computing disparity from both pairs to input to the wls filter and saving as type int

        Img_Filtered = wls_filter.filter(left_disp, imgL, None, right_disp)
        Img_Filtered = cv2.normalize(src=Img_Filtered, dst=Img_Filtered, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX).astype(np.float32);
        #filtering the image, this filter is an edge preserving filter in that
        #it removes high frequency components (noise) but preserves edges by using a technique
        #called gradient matching

        #Img_Filtered = np.uint8(Img_Filtered)
        #imgplot=plt.imshow(Img_Filtered)
        #plt.show()
        #shows the disparity map

        calib = np.array([ 7.215377000000e+02, 0.000000000000e+00, 6.095593000000e+02, 4.485728000000e+01,
                           0.000000000000e+00, 7.215377000000e+02, 1.728540000000e+02, 2.163791000000e-01,
                           0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00,2.745884000000e-03])

        P0 = calib.reshape((3,4))

        print('Generating 3d point cloud for stero pair ' + str(i))
        h, w = imgL.shape[:2]
        f = P0[0][0]
        #focal length
        Q = np.float32([[1, 0, 0, -0.5*w],
                        [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                        [0, 0, 0,     -f], # so that y-axis looks up
                        [0, 0, 1,      0]])
        #Baseline matrix
        points = cv2.reprojectImageTo3D(Img_Filtered, Q, handleMissingValues = 1)
        colors = cv2.cvtColor(imgL_c, cv2.COLOR_BGR2RGB)
        #as opencv reads in the bgr colourspace. Coming back to the RGB colourspace

        #print(Img_Filtered.shape)
        mask = Img_Filtered > Img_Filtered[0][0]
        out_points = points[mask]
        out_colors = colors[mask]

        out_fn = "out" + str(i) + ".ply"
        write_ply(out_fn, out_points, out_colors)


main()

#Visualising the reconstructed point clouds
for i in range(60,81):
    pcd = o3d.io.read_point_cloud("./out" + str(i) + ".ply")
    o3d.visualization.draw_geometries([pcd])
