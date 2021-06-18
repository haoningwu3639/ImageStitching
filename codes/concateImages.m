function concated_img = concateImages(img1, img2)
    % Concatenate two images, pad zero pixels to the image which is smaller.
    
    row1 = size(img1, 1);
    row2 = size(img2, 1);
    column1 = size(img1, 2);
    column2 = size(img2, 2);
    row = max(row1, row2);
    column = max(column1, column2);
    
    image1 = zeros(row, column, size(img1, 3));
    image2 = zeros(row, column, size(img2, 3));
    
    image1(1:row1, 1:column1, :) = img1(1:row1, 1:column1, :);
    image2(1:row2, 1:column2, :) = img2(1:row2, 1:column2, :);
    
    concated_img = [image1, image2];
    concated_img = uint8(concated_img);
end
