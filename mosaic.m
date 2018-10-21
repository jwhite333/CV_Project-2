% Load images
f_sigma = 1.4;
images = [];
for image_num = 1:3
    image_name = sprintf('resources/DanaHallWay1/DSC_028%d.JPG',image_num);
    images = cat(3,images, imgaussfilt(imread(image_name),f_sigma));
end

% Get size
dim = size(images);
x_max = dim(1);
y_max = dim(2);
image_num = dim(3);

% Calculate derivatives
gx = zeros(x_max,y_max,image_num);
gy = zeros(x_max,y_max,image_num);
for counter = 1:image_num
    [gx(:,:,counter),gy(:,:,counter)] = imgradientxy(images(:,:,counter));
end

% Classification parameters
t_magnitude = 1e+06;
k = 0.04;

% Classification
corners = zeros(x_max,y_max,image_num);
edges = zeros(x_max,y_max,image_num);
for counter = 1:image_num
    for x = 2:x_max-1
        for y = 2:y_max-1
            % Create M
            M = [0,0;0,0];
            for i = -1:1
                for j = -1:1
                    M(1,1) = M(1,1) + gx(x+i,y+j,counter)^2;
                    M(1,2) = M(1,2) + gx(x+i,y+j,counter)*gy(x+i,y+j,counter);
                    M(2,1) = M(2,1) + gx(x+i,y+j,counter)*gy(x+i,y+j,counter);
                    M(2,2) = M(2,2) + gy(x+i,y+j,counter)^2;
                end
            end
            % Make decision
            R = det(M)-k*(trace(M))^2;
            if R < -t_magnitude
                edges(x,y,counter) = abs(R);
            elseif R > t_magnitude
                corners(x,y,counter) = R;
            end
        end
    end
end

% Do non-max suppression on edges and corners using 7x7 filter
for counter = 1:image_num
    for x = 4:x_max-3
        for y = 4:y_max-3
            % Edges
            e_pixel = edges(x,y,counter);
            e_pixel_is_max = 1;
            % Corners
            c_pixel = corners(x,y,counter);
            c_pixel_is_max = 1;
            for i = -3:3
                for j = -3:3
                    if e_pixel < edges(x+i,y+j,counter)
                        e_pixel_is_max = 0;
                    end
                    if c_pixel < corners(x+i,y+j,counter)
                        c_pixel_is_max = 0;
                    end
                end
            end

            % Suppress
            if e_pixel_is_max == 0
                edges(x,y,counter) = 0;
            end
            if c_pixel_is_max == 0
                corners(x,y,counter) = 0;
            end
        end
    end
end

% Show results
figure(1)
imshow(corners(:,:,1));

figure(2)
imshow(edges(:,:,1));
