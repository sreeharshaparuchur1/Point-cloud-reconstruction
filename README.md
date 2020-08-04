# Point-cloud-reconstruction 
Below is a brief overview of the prcocess followed as well as disparity maps and point clouds for all 21 stereo pairs.

Stereo vision systems can be used to generate real time disparity maps and extract 3D information from scenes without the use of super expensive sensors like LiDAR's. All that's needed is an image pair with sufficient qualilty. A drawback of using this method to obtain 3D world points of a scene is that there should be sufficient lighting. If the 3D world points are obtained, point clouds for the scene can be manufactured and visualised with the help of software such as open3D. 

Initially, we use a semi global block matching algorithm to find correspondances in the two images. This algorithm works by performing a line optimisation (along the epipolar line) in multiple directions and computing an averge cost function between the intensities of each greyscale image but is highly memory inefficient. Doing this results in a disparity map which is fed to the next part of the code.

We start off processing our coloured image with a Weighted Least Squares (wls) filter. This is done to remove high frequency components such as noise. Edges are also high frequency components but they are preserved as this filtering uses a technique called gradient matching to detect edges and leave them as is. This image is then normalised for easy processing.

In order to reproject the image points into the 3D space, we need something called a baseline matrix. This turns points 180 degrees around the x-axis such that the y-axis looks up. This along with the normalised, filtered image performs matrix multiplication to bring the 2D points into 3D. This matrix is of dimension 4x4 as it deals with homogenous coordinates.

We now have the points. We extract the colours from the image and bring the colourspace from BGR (which is how opencv reads images) back to RGB.

With the 3D points and their corresponding colours, we can generate a point cloud for a given stereo pair.

Disparity Maps:
https://iiitaphyd-my.sharepoint.com/:w:/g/personal/tanmay_garg_students_iiit_ac_in/EcOyf8eWhh9Kq5mvAVGKNWcBUgYUd_XVSEu9G1akoH03xg?e=1DmQPf

open3D visualisation:
https://iiitaphyd-my.sharepoint.com/:w:/g/personal/venkata_surya_students_iiit_ac_in/EQLH67uNjKVAgUEZMLOtCPABs7veaaGDpMuBur9PJ8a7bw?e=qochQM
