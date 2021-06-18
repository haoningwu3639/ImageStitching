function drawFeatures(img, kpts)
    % Draw SIFT feature keypoints with Scale information and Main
    % Orientation information

    figure;
    imshow(img);
    hold on;
    for kpt_i = 1:size(kpts, 1)
        kpt = kpts(kpt_i, :);
        octave_i = kpt(1);
        kpt_h = kpt(3) * 2^(octave_i - 1);
        kpt_w = kpt(4) * 2^(octave_i - 1);
        scale = kpt(5);
        main_ori = kpt(6);
        
        % Different scales
        radius = scale * 2^(octave_i - 1);
        color = [rand(), rand(), rand()];
        rectangle('position', [kpt_w - radius, kpt_h - radius, radius*2, radius*2], 'curvature', [1, 1], 'EdgeColor', color, 'LineWidth', 1);
        
        % Rotation Orientation
        kpt_w2 = kpt_w + radius  * cos(main_ori * pi / 180);
        kpt_h2 = kpt_h - radius * sin(main_ori * pi / 180);
        plot([kpt_w, kpt_w2], [kpt_h, kpt_h2], 'Color', color, 'LineWidth', 1);
        % plot(kpt_w, kpt_h, 'r.', 'MarkerSize', 20, 'LineWidth', 1);
    end
    hold off;
end