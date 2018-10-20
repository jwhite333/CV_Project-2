
% Load images
f_sigma = 2;
images = [];
for image_num = 1:3
    image_name = sprintf('DanaHallWay1/DSC_028%d.JPG',image_num);
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
t_magnitude = 100;
t_similarity = 100;

% Classification
corners = zeros(x_max,y_max,image_num);
x_edges = zeros(x_max,y_max,image_num);
y_edges = zeros(x_max,y_max,image_num);
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
            D = eig(M,'matrix');
            if D(1,1) > t_magnitude && D(2,2) > t_magnitude
                if D(2,2) - D(1,1) < t_similarity
                    corners(x,y,counter) = 1;
                end
            elseif D(1,1) > t_magnitude
                x_edges(x,y,counter) = 1;
            elseif D(2,2) > t_magnitude
                y_edges(x,y,counter) = 1;
            end
        end
    end
end
