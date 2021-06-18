function [descriptors, kpts] = sift(input_img, sigma, S)
    % SIFT Algorithm implemented by Haoning Wu
    
    % sigma: initial scale, default = 1.6
    % S: default = 3
    % Num of difference images of each Octave: (S+2)   
    % Num of Gaussian blur images of each Octave: (S+3)
    % input_img: the input image for keypoints detection and descriptor generation
    % descriptor: 128-dimensional
    % kpts: octave, layer, height, width, scale, main_ori
    
    fprintf('SIFT \n');
    % Convert to grayscale
    if(size(input_img, 3) == 3) 
        img = rgb2gray(input_img);
    else
        img = input_img;
    end
    [height, width] = size(img);
 
    img = imresize(img, 2, 'bicubic'); 
    
    % Gaussian Filter
    sigma_init = sqrt(sigma^2 - 0.5^2 * 4); % Preprocess sigma
    img = imgaussfilt(img, sigma_init);
    img = double(img);
    
    
    % Build pyramid
    fprintf('Build Pyramid... \n');
    octave = round(log2(min(height, width)) - log2(8) + 1);
    % The default length and width of the topmost image of the pyramid is set to 8
    K = 2 ^ (1/S); % Scale space layer
    img = imgaussfilt(img, sigma);
    
    % Gaussian Pyramid & Difference of Gaussian Pyramid
    DoG_pyramid = cell(octave, 1);
    G_pyramid = cell(octave, 1);
    img_i = img;
    
    for octave_i = 1 : octave
        G_pyramid_i = zeros([size(img_i) S+3]); 
        % The first image does not need to be blurred
        G_pyramid_i(:, :, 1) = img_i; 
        % Filter images in an Octave (except the first one)
        for layer = 2:S+3 
            sigma_prev = K^(layer-2) * sigma;
            sigma_i = sqrt((K*sigma_prev)^2 - sigma_prev^2);
            img_i = imgaussfilt(img_i, sigma_i);
            G_pyramid_i(:, :, layer) = img_i;
        end
        
        G_pyramid{octave_i} = G_pyramid_i;
        % Obtain DoG image
        DoG_pyramid{octave_i} = G_pyramid_i(:, :, 1 : end-1) - G_pyramid_i(:, :, 2:end);
        img_i = imresize(G_pyramid_i(:, :, end-2), 0.5, 'bicubic');
        % The sigma of the first Gaussian blur image of the next Octave is
        % the sigma of the third-to-last Gaussian blur image of the previous Octave.
    end
    
    clear img_i G_pyramid_i sigma_i layer octave_i 
    
    
    % Find Extremum
    fprintf('Find Extremum... \n');
    extremum = [];
    num = 0;
    
    for octave_i = 1 : size(DoG_pyramid, 1)
        DoG = DoG_pyramid{octave_i};
        [DoG_h, DoG_w, layer] = size(DoG);

        for DoG_h_i = 2:DoG_h-1
            for DoG_w_i = 2:DoG_w-1
            
                DoG_pixel = DoG(DoG_h_i-1 : DoG_h_i+1, DoG_w_i-1 : DoG_w_i+1, :);
                DoG_pixel = reshape(DoG_pixel, 9, layer);
           
                for layer_i = 2:layer-1
                    curr_pixel = DoG_pixel(:, layer_i);
                    px = curr_pixel(5);
                    curr_pixel(5) = [];
                    if(abs(px)>=floor(0.5*0.04/S*255) && ((px>=max(DoG_pixel(:, layer_i-1)) && px>=max(curr_pixel) && px>=max(DoG_pixel(:, layer_i+1))) || (px<=min(DoG_pixel(:, layer_i-1)) && px<=min(curr_pixel) && px<=min(DoG_pixel(:, layer_i+1)))))
                        num = num + 1;
                        sigma_i = K^(layer_i-1) * sigma;
                        extremum(num, :) = [octave_i, layer_i, DoG_h_i, DoG_w_i, sigma_i, 0];
                    end
                end
            end
        end
    end
    
    clear sigma_i layer_i DoG DoG_height DoG_width layer DoG_pixel curr_pixel px octave_i
    
    
    % Select Keypoints
    fprintf('Select KeyPoints... \n');
    keypoints = [];
    for kpt_i = 1 : num

        kpt = num2cell(extremum(kpt_i, :));
        [oct, layer, kpt_h, kpt_w, ~, ~] = deal(kpt{:});
        
        % Step 1: Remove the point with a large deviation from the exact
        % extreme value, and see whether the deviation of the extreme point
        % can meet the requirements after interpolation.
        
        drop_flag = true; 
        for i = 1:5 % 5 iterations
            DoG = DoG_pyramid{oct}(:, :, layer) ./ 255;
            DoG_prev = DoG_pyramid{oct}(:, :, layer - 1) ./ 255;
            DoG_next = DoG_pyramid{oct}(:, :, layer + 1) ./ 255;
           
            % Derivative
            dD = deriv3D(DoG, kpt_h, kpt_w, DoG_prev, DoG_next);
            % Hessian Matrix
            H = hessian3D(DoG, kpt_h, kpt_w, DoG_prev, DoG_next);
            
            dxx = H(1, 1);
            dyy = H(2, 2);
            dxy = H(1, 2);
            
            % Calculate difference
            % If the Hessian matrix is not invertible, remove the extremum point 
            if(abs(dxx * dyy - dxy * dxy) < 10^-12)
                break;
            end
            x_hat = -H \ dD;
            
            % Adjust the coordinates of the Extremum for the next interpolation calculation
            kpt_w = kpt_w + round(x_hat(1));
            kpt_h = kpt_h + round(x_hat(2)); 
            layer = layer + round(x_hat(3));
            
            if(all(abs(x_hat) < 0.5))  % threshold = 0.5
                drop_flag = false;
                break;
            end

            if(kpt_h<=1 || kpt_h>=size(DoG, 1) || kpt_w<=1 || kpt_w>=size(DoG, 2) || layer<2 || layer>S+1)
               break; 
            end
        end
    
        if(drop_flag)
            continue;
        end
   
        % Step2: Remove extreme points whose response is too small
        D_hat = DoG(kpt_h, kpt_w) + dD' * x_hat / 2;
        if(abs(D_hat) * S < 0.04) 
            continue;
        end
        
        % Step3: Remove Edge keypoints
        trH = dxx + dyy;
        detH = dxx * dyy - dxy * dxy;
        edgeThreshold = 10;
        if(detH<=0 || trH^2 * edgeThreshold >= (edgeThreshold+1)^2 * detH)
            continue;
        end
        sigma_temp = K^(layer-1) * sigma;
        kpt_temp = [oct, layer, kpt_h, kpt_w, sigma_temp, 0];
        keypoints = [keypoints; kpt_temp];
    end

    num = size(keypoints, 1);
    
    
    fprintf('Calculate Main Orientation... \n')
    % Calculate Main Orientation
    kpts = []; 
    for kpt_i = 1 : num

        kpt = keypoints(kpt_i, :);
        octave_i = kpt(1); 
        layer = kpt(2);
        kpt_h = kpt(3);
        kpt_w = kpt(4);
        scale = kpt(5);
     
        radius = round(3 * 1.5 * scale);
        G_pyramid_i = G_pyramid{octave_i}(:, :, layer);
        % (radius*2+1) * (radius*2+1)

        % absolute coordinates of pixels
        img_h = 0; 
        img_w = 0; 
        
        % 1, 2 3~38, 39, 40 for interpolation convenience
        hist_temp = zeros(1, 40);
    
        for i = -radius : radius % column
            img_w = kpt_w + i;
            if (img_w<=1 || img_w>=size(G_pyramid_i, 2)) 
                continue;
            end
            for j = -radius : radius % row
                img_h = kpt_h + j;
                if(img_h<=1 || img_h>=size(G_pyramid_i, 1))
                    continue;
                end

                % Gradient
                dx = G_pyramid_i(img_h, img_w+1) - G_pyramid_i(img_h, img_w-1);
                dy = G_pyramid_i(img_h-1, img_w) - G_pyramid_i(img_h+1, img_w);

                magnitude = sqrt(dx^2 + dy^2);

                % angle: -pi ~ pi
                if((dx>=0 && dy>=0) || (dx<=0 && dy>=0))
                    angle = atan2(dy, dx);
                else
                    angle = atan2(dy, dx) + 2*pi;
                end
                angle = angle * 360/(2 * pi);

                % Calculate the bin indexes that fall into the histogram
                bin_index = round(36 / 360 * angle);
                if(bin_index >= 36)
                    bin_index = bin_index - 36;
                elseif(bin_index < 0) 
                    bin_index = bin_index + 36;
                end

                % Gaussian weight
                W_G = exp(-(i^2 + j^2) / (2 * (1.5*scale)^2));

                % hist_temp: 1,2,3~38,39,40
                hist_temp(bin_index+3) = hist_temp(bin_index+3) + magnitude * W_G;
            end
        end
 
        % Smooth Orietation Histogram
        hist = zeros(1, 36);
        hist_temp(1:2) = hist_temp(37:38);
        hist_temp(39:40) = hist_temp(3:4);
        for k = 3:38   %
            hist(k-2) = (hist_temp(k-2) + hist_temp(k+2)) * (1/16) + (hist_temp(k-1) + hist_temp(k+1)) * (4/16) + hist_temp(k) * (6/16);
        end 
        
        % Traverse the histogram to calculate the primary and secondary directions
        hist_threshold = 0.8 * max(hist);
        for k = 1:36
            if(k==1) 
                k_left = 36;
            else
                k_left = k - 1;
            end
            
            if(k==36)
                k_right = 1;
            else
                k_right = k + 1;
            end
            % Histogram bins need to exceed threshold and be higher than adjacent bins
            if(hist(k)>hist(k_left) && hist(k)>hist(k_right) && hist(k)>=hist_threshold)
                % Parabolic Interpolation
                bin_index = k + 0.5 * (hist(k_left) - hist(k_right)) / (hist(k_left) - 2*hist(k) + hist(k_right));
                if(bin_index < 0)
                    bin_index = bin_index + 36;
                elseif(bin_index >= 36)
                    bin_index = bin_index - 36;
                end

                angle =  bin_index * (360/36);
                if(abs(angle - 360) < 10^-12)
                    angle = 0;
                end

                % Update keypoints
                kpt_temp = [kpt(1:5), angle];
                kpts = [kpts; kpt_temp];
            end
        end
    end


    % Generate Feature Descriptor
    fprintf('Generate Feature Descriptor... \n');
    d = 4; % Number of subspaces
    n = 8; % 8 angle layers
    % 4 * 4 * 8
    descriptors = [];
    % Calculate 128-dimension feature vector for each keypoint
    for kpt_i = 1:size(kpts, 1)
        kpt = kpts(kpt_i, :);
        octave_i = kpt(1);
        layer = kpt(2);
        kpt_h = kpt(3);
        kpt_w = kpt(4);
        scale = kpt(5);
        main_ori = kpt(6);

        G_pyramid_i = G_pyramid{octave_i}(:, :, layer);
        [G_pyramid_i_h, G_pyramid_i_w] = size(G_pyramid_i);
        
        % Calculate rotation matrix parameters
        cos_r = cos(main_ori * pi / 180);
        sin_r = sin(main_ori * pi / 180);

        % The length of each subregion
        hist_width = 3 * scale;
        % The radius of all the participating pixels
        radius = round(hist_width * sqrt(2) * (d+1) * 0.5); 
        radius = min(radius, floor(sqrt(G_pyramid_i_h^2 + G_pyramid_i_w^2))); 
        % Normalize so that (h_rot, w_rot) is at the scale of subregion
        cos_r = cos_r / hist_width;
        sin_r = sin_r / hist_width;
        % rows in subregion coordinates, columns in subregion coordinates, bins in histogram
        hist = zeros(d+2, d+2, n+2);
        
        % Calculate Histogram
        for i = -radius:radius
            for j = -radius:radius 
            
                % Relative coordinate after rotation(subregion scale)
                w_rot = j * cos_r - i * sin_r;
                h_rot = j * sin_r + i * cos_r;
                
                % The translation of 0.5 makes the intersection points of
                % the coordinate axes of the subregion all fall on the 
                % upper left corner of the subregion, which is convenient 
                % for later interpolation.
                h_bin = h_rot + d/2 - 0.5;
                w_bin = w_rot + d/2 - 0.5;

                % Absolute coordinate
                img_h = kpt_h + i;
                img_w = kpt_w + j;

                if (h_bin>-1 && h_bin<d && w_bin>-1 && w_bin<d && img_h>1 && img_h<G_pyramid_i_h && img_w>1 && img_w<G_pyramid_i_w)

                    dx = G_pyramid_i(img_h, img_w + 1) - G_pyramid_i(img_h, img_w - 1);
                    dy = G_pyramid_i(img_h - 1, img_w) - G_pyramid_i(img_h + 1, img_w);

                    magnitude = sqrt(dx^2 + dy^2);

                    if (dx>=0 && dy>=0 || dx<=0 && dy>=0)
                        angle = atan2(dy, dx);
                    else
                        angle = atan2(dy, dx) + 2*pi;
                    end
                    angle = angle * 360/(2 * pi); 
                    
                    % height, width, thickness
                    t_bin = (angle - main_ori) * (n / 360); % 0~7
                    
                    W_G = exp(-(h_rot^2 + w_rot^2) / (0.5 * d^2));

                    magnitude = magnitude * W_G;

                    h0 = floor(h_bin);
                    w0 = floor(w_bin);
                    t0 = floor(t_bin);

                    % Coordinate in the cube
                    h_bin = h_bin - h0;
                    w_bin = w_bin - w0;
                    t_bin = t_bin - t0;

                    if (t0 < 0)
                        t0 = t0 + n;
                    elseif (t0 >= n)
                        t0 = t0 - n;
                    end
                    
                    % Trilinear Interpolation
                    % Contribute pixel amplitude to the histogram of the four surrounding subregions
                    w_hwt000 = h_bin * w_bin * t_bin;
                    w_hwt001 = h_bin * w_bin * (1 - t_bin);

                    w_hwt010 = h_bin * (1 - w_bin) * t_bin;
                    w_hwt011 = h_bin * (1 - w_bin) * (1 - t_bin);

                    w_hwt100 = (1 - h_bin) * w_bin * t_bin;
                    w_hwt101 = (1 - h_bin) * w_bin * (1 - t_bin);                

                    w_hwt110 = (1 - h_bin) * (1 - w_bin) * t_bin;
                    w_hwt111 = (1 - h_bin) * (1 - w_bin) * (1 - t_bin);

                    temp = zeros(2, 2, 2);
                    temp(:, :, 1) = [w_hwt000, w_hwt010; w_hwt100, w_hwt110];
                    temp(:, :, 2) = [w_hwt001, w_hwt011; w_hwt101, w_hwt111];
                    hist(h0+2:h0+3, w0+2:w0+3, t0+1:t0+2) = hist(h0+2:h0+3, w0+2:w0+3, t0+1:t0+2) + magnitude .* temp;
                    % h_bin: 1~6, h0: 1~3; w_bin: 1~6, w0: 1~3; t_bin: 1~8, t0: 0~7;  
                end 
            end
        end
        
        descriptor_i = [];
        % Traverse subregions and obtain eigenvectors from the histogram
        for i = 1:d
            for j = 1:d
                hist(i+1, j+1, 1) = hist(i+1, j+1, 1) + hist(i+1, j+1, 9);
                hist(i+1, j+1, 2) = hist(i+1, j+1, 2) + hist(i+1, j+1, 10);
                descriptor_i = [descriptor_i; reshape(hist(i+1, j+1, 1:8), 8, 1)];
            end
        end

        % Process the component with large amplitude of eigenvector to 
        % improve the influence of nonlinear illumination
        descriptor_norm = norm(descriptor_i, 2);
        descriptor_threshold = 0.2 * descriptor_norm;
        descriptor_i(descriptor_i > descriptor_threshold) = descriptor_threshold;

        % Normalization
        descriptor_norm = norm(descriptor_i, 2);
        descriptor_i = descriptor_i ./ descriptor_norm;
        descriptors = [descriptors, descriptor_i];

    end
    
    % Transform coordinate scale
    for kpt_i = 1:size(kpts, 1)
        kpts(kpt_i, 3:4) = kpts(kpt_i, 3:4) .* 2^(kpts(kpt_i, 1) - 2);  
    end


end