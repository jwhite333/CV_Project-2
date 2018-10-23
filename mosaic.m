clc;
clear;

% Load images
images = [];
for image_num = 1:2
    image_name = sprintf('resources/DanaHallWay1/DSC_028%d.JPG',image_num);
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
k = 0.04;
f_sigma = 2;

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


% Do non-max suppression on edges and corners using 7x7 filter
for counter = 1:image_num
    for x = 2:x_max-1
        for y = 2:y_max-1
            % Corners
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
tolerence = 10;
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

%Show results
% figure(3)
% scatter(true_matches(:,2),true_matches(:,1))
% figure(4)
% scatter(true_matches(:,4),true_matches(:,3))

figure(5)
imshowpair(images(:,:,1),images(:,:,2),'montage');
t=size(true_matches);
for index = 1:t(1) 
    hold on
    plot([true_matches(index, 2) true_matches(index,4)+512], ...
        [true_matches(index, 1) true_matches(index,3)]);
end

%% homography
[H, inliner] = RANSAC_homogrpahy(true_matches);

%least-squares homgraphy
lengthH = length(inliner);
A =[];
for i=1:lengthH
    A =[A; 
        inliner(i,1) inliner(i,2) 1 0 0 0 -inliner(i,1)*inliner(i,3) -inliner(i,2)*inliner(i,3) -inliner(i,3);
        0 0 0 inliner(i,1) inliner(i,2) 1 -inliner(i,1)*inliner(i,4) -inliner(i,2)*inliner(i,4) -inliner(i,4)];      
end

[U,S,V] = svd(A.'*A);
%%
figure(6)
imshowpair(images(:,:,1),images(:,:,2),'montage');
t=size(inliner);
for index = 1:t(1) 
    hold on
    plot([inliner(index, 1) inliner(index,3)+512], ...
        [inliner(index, 2) inliner(index,4)]);
end
