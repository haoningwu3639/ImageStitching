function [H, corrPtIdx] = findHomography(pts1, pts2)
    % Find the homography between two planes using a set of corresponding
    % points and RANSAC. pts1 = [x1,x2,...; y1,y2,...].
    % H: homography matrix
    % corrPtIdx: the index of inliers.

    coef.minPtNum = 4;
    coef.iterNum = 30;
    coef.thDist = 4;
    coef.thInlrRatio = 0.1;
    [H, corrPtIdx] = RANSAC(pts1, pts2, coef, @solveHomography, @calcDist);

end


function dis = calcDist(H, pts1, pts2)
    % Transform pts1 to pts3 using H, then calcultate the distances between pts2 and pts3

    n = size(pts1, 2);
    pts3 = H * [pts1; ones(1,n)];
    pts3 = pts3(1:2,:) ./ repmat(pts3(3,:), 2, 1);
    dis = sum((pts2-pts3).^2, 1);

end