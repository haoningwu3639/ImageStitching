# ImageStitching
A project developed by MATLAB for Image Stitching based on SIFT and RANSAC. It's also a personal project of SJTU EE229 Digital Image Processing.

# Requirements
Code tested on following environments, other version should also work:
* Matlab 2020a

# Project Structure
* [`Problem.m`](codes/Problem.m): Main Function;
* [`sift.m`](codes/sift.m): Scale-Invariant Feature Transform;
* [`RANSAC.m`](codes/RANSAC.m): RANdom SAmple Consensus;
* [`stitch.m`](codes/stitch.m): Image Stitching function;
* [`deriv3D.m`](codes/deriv3D.m), [`hessian3D.m`](codes/hessian3D.m): Util functions for SIFT;
* [`drawFeatures.m`](codes/drawFeatures.m), [`drawMatched.m`](codes/drawMatched.m): Visualization Functions for SIFT;
* [`concateImages`](codes/cocateImages): Util function, conpare the sizes of the two images and pad the smaller one with zeros;
* [`findHomography.m`](codes/findHomography.m), [`solveHomography.m`](codes/solveHomography.m): Utils function for RANSAC.

# References

 If you gonna learn more about SIFT and Image Stitching, I recommend these repositories for you: [OpenCV](https://github.com/opencv/opencv) and [image mosaic using SIFT](https://www.mathworks.com/matlabcentral/fileexchange/30849-image-mosaic-using-sift?s_tid=srchtitle).


# Miscs

 Since I still have a lot of work to do, I have no time to maintain this project now. I hope that I can further improve this repository when I'm free in the future. If you have any questions, please contact me by email. Thanks.
