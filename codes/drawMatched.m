function [matched, locs1, locs2] = drawMatched(img1, img2, kpts1, kpts2, descriptors1, descriptors2)
    % Plot matched keypoints in the two images and show them side by side.
    % img1, img2: Original image for visualization
    % kpts1, kpts2: Keypoints detected by SIFT
    % descriptors1 descriptors2: Feature descriptors extracted by SIFT
    % matched: Matched keypoint indices in two images
    % locs1, locs2: Matched keypoint coordinates
    
    img3 = concateImages(img1, img2);
    
    matched = []; 
    locs1 = [];
    locs2 = [];
    distance = [];
    threshold = 0.1;
    
    for kpt_i = 1:size(descriptors1, 2)
        for kpt_j = 1:size(descriptors2, 2)
            % Euclidean Distance
            if (norm(descriptors1(:, kpt_i) - descriptors2(:, kpt_j), 2) < threshold)
                distance = [distance, norm(descriptors1(:, kpt_i) - descriptors2(:, kpt_j), 2)];
                matched = [matched; kpt_i, kpt_j];
            end
        end
    end
    
    figure;
    imshow(img3);
    hold on;
    % Draw keypoints of Figure 1
    for kpt_i = 1:size(kpts1, 1)
        kpt = kpts1(kpt_i, :);
        plot(kpt(4), kpt(3), 'r.', 'MarkerSize', 10);
    end
    
    % Draw Keypoints of Figure 2
    for kpt_i = 1:size(kpts2, 1)
        kpt = kpts2(kpt_i, :);
        % Side by Side
        plot(kpt(4) + size(img1, 2), kpt(3), 'r.', 'MarkerSize', 10);
    end
    
    % Draw matched correspondence
    for i = 1 : size(matched, 1)
        kpt1 = kpts1(matched(i, 1), :);
        kpt2 = kpts2(matched(i, 2), :);
        locs1 = [locs1; kpt1(4), kpt1(3)];
        locs2 = [locs2; kpt2(4), kpt2(3)];
        % Width, Height
        line([kpt1(4), kpt2(4) + size(img1, 2)], [kpt1(3), kpt2(3)], 'LineWidth', 1, 'Color', [rand(),rand(),rand()])
        
    end

    hold off;
    
    fprintf('Found %d matches.\n', size(matched, 1));
    
end

