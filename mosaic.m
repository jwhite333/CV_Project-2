clc;
clear;
% Load images
f_sigma = 2;
images = [];
for image_num = 1:2
    image_name = sprintf('resources/DanaHallWay1/DSC_028%d.JPG',image_num);
    images = cat(3,images,rgb2gray(imread(image_name)));
end

% Get size
dim = size(images);
x_max = dim(1);
y_max = dim(2);
image_num = dim(3);

% Calculate derivatives
gx = zeros(x_max,y_max,image_num);
gy = zeros(x_max,y_max,image_num);
dx = [-1 0 1; -1 0 1; -1 0 1];
dy = [-1 -1 -1; 0 0 0; 1 1 1];

for counter= 1:image_num
    Ix(:,:,counter) = imfilter(double(images(:,:,counter)), dx);
    Iy(:,:,counter) = imfilter(double(images(:,:,counter)), dy);
end


% Classification parameters
k = 0.04;
threshold = 2e+06;;
% Classification
corners = zeros(x_max,y_max,image_num);

cxx = imgaussfilt(Ix.^2, 2 ,'FilterSize',9);
cyy = imgaussfilt(Iy.^2, 2 ,'FilterSize',9);
cxy = imgaussfilt(Ix.*Iy, 2 ,'FilterSize',9);

corners=zeros(x_max,y_max,image_num);
for counter = 1:image_num
    for x = 1:x_max
        for y = 1:y_max
            % Create M
            M = [cxx(x,y,counter), cxy(x,y,counter);cxy(x,y,counter),cyy(x,y,counter)];
            % Make decision
            R = det(M)-k*(trace(M))^2;
            if (R > threshold)
                corners(x,y,counter) = R;
            end
        end
    end
end

% Do non-max suppression on edges and corners using 7x7 filter
for counter = 1:image_num
    for x = 4:x_max-3
        for y = 4:y_max-3
            % Corners
            c_pixel = corners(x,y,counter);
            c_pixel_is_max = 1;
            for i = -3:3
                for j = -3:3
                    if c_pixel < corners(x+i,y+j,counter)
                        c_pixel_is_max = 0;
                    end
                end
            end

            % Suppress
            if c_pixel_is_max == 0
                corners(x,y,counter) = 0;
            end
        end
    end
end

% Show results
% figure(1)
% imshow(corners(:,:,1));
% 
% figure(2)
% imshow(corners(:,:,2));

%% b correspondences between 2 images
%images1

[row, col] = find(corners(:,:,1));
num = size(row);
matches1=[];
counter = 1;
for x = 1:num(1)
    if(row(x,1) >3 && row(x,1) <= x_max-3) && (col(x,1) > 3 && col(x,1) <=y_max -3)
        g = images(row(x,1)-3:row(x,1)+3, col(x,1)-3:col(x,1)+3, 1);
        f = images(:,:,2);
        
        NCC = normxcorr2(g,f);
        [ypeak, xpeak] = find(NCC==max(NCC(:)));
        matches1(counter,1) = row(x,1);
        matches1(counter,2) = col(x,1);
        matches1(counter,3) = ypeak;
        matches1(counter,4) = xpeak;
        
        counter = counter +1;
    end
end

[row, col] = find(corners(:,:,2));
num = size(row);
matches2=[];
counter = 1;
for x = 1:num(1)
    if(row(x,1) >3 && row(x,1) <= x_max-3) && (col(x,1) > 3 && col(x,1) <=y_max -3)
        g = images(row(x,1)-3:row(x,1)+3, col(x,1)-3:col(x,1)+3, 2);
        f = images(:,:,1);
        
        NCC = normxcorr2(g,f);
        [ypeak, xpeak] = find(NCC==max(NCC(:)));
        matches2(counter,1) = row(x,1);
        matches2(counter,2) = col(x,1);
        matches2(counter,3) = ypeak;
        matches2(counter,4) = xpeak;
        counter = counter +1;
    end
    
end

%% homography
dim = size(matches1);




