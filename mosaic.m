clc;
clear;

% Initilization
% 1 = 'resources/DanaHallWay1/DSC_028%d.JPG';
% 2 = 'resources/DanaHallWay2/DSC_028%d.JPG';
% 3 = 'resources/DanaOffice/DSC_03%d.JPG';
SourcePath = 2;

% for resources/DanaOffice/DSC_028%d.JPG
% 1 = DSC_0308 and DSC_0309
% 2 = DSC_0310 and DSC_0311
% 3 = DSC_0311 and DSC_0312
% 4 = DSC_0313 and DSC_0314
% 5 = DSC_0315 and DSC_0316  
% 6 = DSC_0316 and DSC_0317
% for other two folder can be anything
option = 1;


[path, x, y] = sourceSelect(SourcePath, option);


% Load images
images = [];

for image_num = x:y
    image_name = sprintf(path,image_num);
    temp = im2double(imread(image_name));
    images = cat(3,images,rgb2gray(temp));
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
    Ix(:,:,counter) = imfilter(images(:,:,counter), dx);
    Iy(:,:,counter) = imfilter(images(:,:,counter), dy);
end


% Classification parameters
k = 0.04;           % Value for Harris corner detector
f_sigma = 2;        % Gaussian filtering variance
tolerence = 1;      % Homography pixel error tolerence

% Classification
corners = zeros(x_max,y_max,image_num);
cxx = imgaussfilt(Ix.^2, f_sigma,'FilterSize',9);
cyy = imgaussfilt(Iy.^2, f_sigma,'FilterSize',9);
cxy = imgaussfilt(Ix.*Iy, f_sigma,'FilterSize',9);
R = zeros(x_max,y_max,image_num);
for counter = 1:image_num
    for x = 1:x_max
        for y = 1:y_max
            % Create M
            M = [cxx(x,y,counter), cxy(x,y,counter);cxy(x,y,counter),cyy(x,y,counter)];
            % Make decision
            R(x,y,counter) = det(M)-k*(trace(M))^2;
            
        end
    end
end

for counter = 1:image_num
    temp = max(max(R(:,:,counter)));
    maxR(1,counter) = 0.001*temp;
end


% Do non-max suppression on corners
for counter = 1:image_num
    for x = 2:x_max-1
        for y = 2:y_max-1
            if R(x,y,counter)>maxR(1,counter) && R(x,y,counter)>R(x-1,y-1,counter) ...
                && R(x,y,counter)>R(x-1,y,counter) && R(x,y,counter)>R(x-1,y+1,counter) ...
                && R(x,y,counter)>R(x,y-1,counter) && R(x,y,counter)>R(x,y+1,counter) ...
                && R(x,y,counter)>R(x+1,y-1,counter) && R(x,y,counter)>R(x+1,y,counter) ...
                && R(x,y,counter)>R(x+1,y+1,counter)
                    corners(x,y,counter) = 1;
            end
        end
    end
end

figure(1)
[r,c] = find(corners(:,:,1));
imshow(images(:,:,1));
hold on
plot(c,r,"yd");

figure(2)
[r,c] = find(corners(:,:,2));
imshow(images(:,:,2));
hold on
plot(c,r,"yd");

%% b Correspondences between 2 images

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

% Image 2
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

% Remove false matches
true_matches = [];
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

% Display correspondence pairs on images
figure(5)
imshowpair(images(:,:,1),images(:,:,2),'montage');
t=size(true_matches);
for index = 1:t(1) 
    hold on
    plot([true_matches(index, 2) true_matches(index,4)+512], ...
        [true_matches(index, 1) true_matches(index,3)]);
end

%% Homography using RANSAC
%[H, inliner] = RANSAC_homogrpahy(true_matches);

accuracy = 0.0;
while accuracy < 0.9
    % Choose 4 points at random
    samples = randi([1,t(1)],1,4);
    A = zeros(8,9);
    
    % Create the A matrix
    for iter = 0:3
        x1 = true_matches(samples(iter+1),1);
        y1 = true_matches(samples(iter+1),2);
        x1p = true_matches(samples(iter+1),3);
        y1p = true_matches(samples(iter+1),4);
        A(2*iter+1,:) = [x1,y1,1,0,0,0,-x1*x1p,-y1*x1p,-x1p];
        A(2*iter+2,:) = [0,0,0,x1,y1,1,-x1*y1p,-y1*y1p,-y1p];
    end
    
    % Get SVD
    [U,~,~] = svd(A.'*A);
    h = U(:,9);
    
    % Find percentage of points that agree with this homography
    correct_homs = 0;
    inliers = [];
    H = [h(1),h(2),h(3);h(4),h(5),h(6);h(7),h(8),h(9)];
    for iter = 1:t(1)
        x = true_matches(iter,1);
        y = true_matches(iter,2);
        guess = H*[x;y;1];
        x1p_guess = guess(1)/guess(3);
        y1p_guess = guess(2)/guess(3);
        if abs(x1p_guess-true_matches(iter,3)) < tolerence && abs(y1p_guess-true_matches(iter,4)) < tolerence
            
            % Set the "inliers" to be the points that agree with the
            % homography
            inliers = [inliers; true_matches(iter,:)];
            correct_homs = correct_homs+1;
        end
    end
    accuracy = correct_homs/t(1);
end

figure(6)
imshowpair(images(:,:,1),images(:,:,2),'montage');
t = size(inliers);
for index = 1:t(1)
    hold on
    plot([inliers(index, 2) inliers(index,4)+512], ...
        [inliers(index, 1) inliers(index,3)]);
end


%% Combine images 1+2

% Find dimentions of mosaic
hom = H*[1;1;1];
width_diff = 1-hom(1)/hom(3);
height_diff = 1-hom(2)/hom(3);
mosaic_width = ceil(dim(1)+abs(width_diff));
%mosaic_height = ceil(2*dim(2)-2*height_diff);
mosaic_height = ceil(dim(2)+abs(height_diff));
mosaic_img = zeros(mosaic_width,mosaic_height);

% Place right image (image 2)
x_offset = 1;
if width_diff > 0
    x_offset = ceil(width_diff);
end
mosaic_img(x_offset+1:x_offset+dim(1),mosaic_height-dim(2)+1:mosaic_height) = images(:,:,2);

x0 = x_offset+1;
y0 = mosaic_height-dim(2)+1;

% Add in image 1
for x1 = 1:mosaic_width
    for y1 = 1:mosaic_height
        inv_hom = H\[x1-x0;y1-y0;1];
        x = round(inv_hom(1)/inv_hom(3));
        y = round(inv_hom(2)/inv_hom(3));
        if x >= 1 && x <= dim(1) && y >= 1 && y <= dim(2)
            if mosaic_img(x1,y1) ~= 0
                mosaic_img(x1,y1) = (mosaic_img(x1,y1)+images(x,y,1))/2;
            else
                mosaic_img(x1,y1) = images(x,y,1);
            end
        end
    end
end

% Show mosaic
figure(7)
imshow(mosaic_img);
