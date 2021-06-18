clear;
clc;

tic;

sigma = 1.6;
S = 3;

img1 = imread('../inputs/img1.jpg');
img2 = imread('../inputs/img2.jpg');

% img2 = imnoise(img2, 'gaussian', 0, 0.008);
% img2(size(img2, 1)/2:end, size(img2, 2)/2:end, :) = 255;
% img2 = rot90(img2, 2);
% img2 = imresize(img2, 0.5, 'bicubic');

[descriptors1, kpts1] = sift(img1, sigma, S);
[descriptors2, kpts2] = sift(img2, sigma, S);

drawFeatures(img1, kpts1);
drawFeatures(img2, kpts2);

[matched, locs1, locs2] = drawMatched(img1, img2, kpts1, kpts2, descriptors1, descriptors2);

stitched_img = stitch(img1, img2, locs1, locs2, 1);
imshow(stitched_img);

toc;
