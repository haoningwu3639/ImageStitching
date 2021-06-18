function [output] = stitch(img1, img2, matchLoc1, matchLoc2, adjColor)
    % img1, img2: can (both) be RGB or gray, double or uint8
    % matchLoc1, matchLoc2: matched keypoints based on SIFT
    % adjColor: default = 1
    % it will try to adjust the color(rgb) or grayscale(gray) of img1 linearly
    
    % Use RANSAC to find homography matrix
    [H, corrPtIdx] = findHomography(matchLoc2', matchLoc1');
    % corrPtIdx: a list of indices
    
    T = maketform('projective', H');
    img21 = imtransform(img2, T); % reproject img2
    
    % Adjust color or grayscale linearly, using corresponding infomation
    [M1, N1, dim] = size(img1);
    [M2, N2, ~] = size(img2);
    if exist('adjColor', 'var') && adjColor == 1
        radius = 2;
        x1_corr = matchLoc1(corrPtIdx, 1);
        y1_corr = matchLoc1(corrPtIdx, 2);
        x2_corr = matchLoc2(corrPtIdx, 1);
        y2_corr = matchLoc2(corrPtIdx, 2);
        corrPtLen = length(corrPtIdx);
        s1 = zeros(1, corrPtLen);
        s2 = zeros(1, corrPtLen);
        for color = 1:dim
            for p = 1:corrPtLen
                left = round(max(1, x1_corr(p)-radius));
                right = round(min(N1, left+radius+1));
                up = round(max(1, y1_corr(p)-radius));
                down = round(min(M1, up+radius+1));
                s1(p) = sum(sum(img1(up:down, left:right, color)));
            end
            for p = 1:corrPtLen
                left = round(max(1, x2_corr(p)-radius));
                right = round(min(N2, left+radius+1));
                up = round(max(1, y2_corr(p)-radius));
                down = round(min(M2, up+radius+1));
                s2(p) = sum(sum(img2(up:down, left:right, color)));
            end
            sc = (radius*2+1)^2 * corrPtLen;
            adjcoef = polyfit(s1/sc, s2/sc, 1);
            img1(:,:,color) = img1(:, :, color) * adjcoef(1) + adjcoef(2);
        end
    end

    % Image Stitch
    pt = zeros(3,4);
    pt(:, 1) = H * [1; 1; 1];
    pt(:, 2) = H * [N2; 1; 1];
    pt(:, 3) = H * [N2; M2; 1];
    pt(:, 4) = H * [1; M2; 1];
    x2 = pt(1, :) ./ pt(3, :);
    y2 = pt(2, :) ./ pt(3, :);

    up = round(min(y2));
    Yoffset = 0;
    if up <= 0
        Yoffset = -up + 1;
        up = 1;
    end

    left = round(min(x2));
    Xoffset = 0;
    if left<=0
        Xoffset = -left + 1;
        left = 1;
    end

    [M3, N3, ~] = size(img21);
    output(up:up+M3-1, left:left+N3-1, :) = img21;
    output(Yoffset+1:Yoffset+M1, Xoffset+1:Xoffset+N1, :) = img1;
    
end
