function [f, inlierIdx] = RANSAC(x, y, ransacCoef, funcFindF, funcDist)
    % Use RANSAC(RANdom SAmple Consensus) to find a fit from X to Y.
    % X: M * n matrix including n points with dim M 
    % Y: N * n
    % f: Fit 
    % inlierIdx: the indices of inliers
    % ransacCoef: minPtNum, iterNum, thDist, thInlrRatio
    % minPtNum: the minimum number of points with whom can we find a fit.
    % For line fitting, it's 2. For homography, it's 4.
    % iterNum: the number of iteration, 
    % thDist: the inlier distance threshold 
    % funcFindF: findHomography
    % funcDist: calDist, L2 norm between pts2 and pts3(projected pts1)
    
    minPtNum = ransacCoef.minPtNum;
    iterNum = ransacCoef.iterNum;
    thInlrRatio = ransacCoef.thInlrRatio;
    thDist = ransacCoef.thDist;
    ptNum = size(x, 2);
    Inlier_thres = round(thInlrRatio * ptNum);

    inlier_Num = zeros(1,iterNum);
    fLib = cell(1,iterNum);

    for p = 1:iterNum
        % 1. Fit with random points
        sampleIdx = randIndex(ptNum, minPtNum);
        f1 = funcFindF(x(:, sampleIdx), y(:, sampleIdx));

        % 2. Count inliers, if more than thInlr, refit; else iterate
        dist = funcDist(f1, x, y);
        inlier1 = find(dist < thDist);
        inlier_Num(p) = length(inlier1);
        
        if length(inlier1) < Inlier_thres 
            continue; 
        end
        
        fLib{p} = funcFindF(x(:, inlier1),y(:, inlier1));
    end

    % 3. Choose the coefficient with the most inliers
    [~, idx] = max(inlier_Num);
    f = fLib{idx};
    dist = funcDist(f,x,y);
    inlierIdx = find(dist < thDist);
    
end



function index = randIndex(maxIndex, len)
    % Utils function
    % Randomly, non-repeatedly select LEN integers from 1:MAXINDEX

    if len > maxIndex
        index = [];
        return
    end

    index = zeros(1,len);
    available = 1:maxIndex;
    rs = ceil(rand(1,len).*(maxIndex:-1:maxIndex-len+1));
    
    for p = 1:len
        while rs(p) == 0
            rs(p) = ceil(rand(1)*(maxIndex-p+1));
        end
        index(p) = available(rs(p));
        available(rs(p)) = [];
    end
end