clc;
clear;

% Load images
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
dx = [-1 0 1; -1 0 1; -1 0 1];
dy = [-1 -1 -1; 0 0 0; 1 1 1];

for counter= 1:image_num
    Ix(:,:,counter) = imfilter(double(images(:,:,counter)), dx);
    Iy(:,:,counter) = imfilter(double(images(:,:,counter)), dy);
end


% Classification parameters
k = 0.04;
threshold = 2e+06;
f_sigma = 2;

% Classification
corners = zeros(x_max,y_max,image_num);
cxx = imgaussfilt(Ix.^2, f_sigma,'FilterSize',9);
cyy = imgaussfilt(Iy.^2, f_sigma,'FilterSize',9);
cxy = imgaussfilt(Ix.*Iy, f_sigma,'FilterSize',9);

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

%% b correspondences between 2 images

% Image 1
[row, col] = find(corners(:,:,1) > 0);
num = size(row);
matches1=[];
counter = 1;
for x = 1:num(1)
    if(row(x,1) > 3 && row(x,1) <= x_max-3) && (col(x,1) > 3 && col(x,1) <= y_max-3)
        g = images(row(x,1)-3:row(x,1)+3, col(x,1)-3:col(x,1)+3, 1);
        f = images(:,:,2);
        
        NCC = normxcorr2(g,f);
        [ypeak, xpeak] = find(NCC==max(NCC(:)));
        matches1(counter,1) = row(x,1);
        matches1(counter,2) = col(x,1);
        matches1(counter,3) = ypeak-3;
        matches1(counter,4) = xpeak-3;
        
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
        matches2(counter,3) = ypeak-3;
        matches2(counter,4) = xpeak-3;
        counter = counter +1;
    end
    
end

% Find real correspondences
true_matches = [];
tolerence = 2;
total_matches1 = size(matches1);
for iter1 = 1:total_matches1(1)
   image1_x = matches1(iter1,1);
   image1_y = matches1(iter1,2);
   image2_x = matches1(iter1,3);
   image2_y = matches1(iter1,4);
   
   total_matches2 = size(matches2);
   for iter2 = 1:total_matches2(1)
       if abs(matches2(iter2,1)-matches1(iter1,3)) < tolerence
           if abs(matches2(iter2,2)-matches1(iter1,4)) < tolerence
               if abs(matches2(iter2,3)-matches1(iter1,1)) < tolerence
                   if abs(matches2(iter2,4)-matches1(iter1,2)) < tolerence
                       true_matches = [true_matches; image1_x,image1_y,image2_x,image2_y];
                   end
               end
           end
       end
   end
end


%% homography
dim = size(matches1);

% Show results
figure(1)
scatter(true_matches(:,1),true_matches(:,2))
figure(2)
scatter(true_matches(:,3),true_matches(:,4))


