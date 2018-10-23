function [New_H, New_inliner] = RANSAC_homogrpahy(true_matches)
itr = 20000;
threshold = 5;
sizeM = length(true_matches);
max = 0;
for counter=1:itr
    numCount =0;
    
    
    I1 = randperm(length(true_matches),4);
    I2 = randperm(length(true_matches),4);
    
    point1 = [true_matches(I1(1,1), 2) true_matches(I1(1,1), 1); 
        true_matches(I1(1,2), 2)  true_matches(I1(1,2), 1);
        true_matches(I1(1,3), 2) true_matches(I1(1,3), 1); 
        true_matches(I1(1,4), 2)  true_matches(I1(1,4), 1)];
     
    point2 = [true_matches(I2(1,1), 4) true_matches(I2(1,1), 3); 
        true_matches(I2(1,2), 4)  true_matches(I2(1,2), 3);
        true_matches(I2(1,3), 4) true_matches(I2(1,3), 3); 
        true_matches(I2(1,4), 4)  true_matches(I2(1,4), 3)];
     
    A = [point1(1,1), point1(1,2), 1, 0, 0, 0, -point1(1,1)*point2(1,1), -point1(1,2)*point2(1,1);
        0, 0, 0, point1(1,1), point1(1,2), 1, -point1(1,1)*point2(1,2), -point1(1,2)*point2(1,2);
        point1(2,1), point1(2,2), 1, 0, 0, 0, -point1(2,1)*point2(2,1), -point1(2,2)*point2(2,1);
        0, 0, 0, point1(2,1), point1(2,2), 1, -point1(2,1)*point2(2,2), -point1(2,2)*point2(2,2);
        point1(3,1), point1(3,2), 1, 0, 0, 0, -point1(3,1)*point2(3,1), -point1(3,2)*point2(3,1);
        0, 0, 0, point1(3,1), point1(3,2), 1, -point1(3,1)*point2(3,2), -point1(3,2)*point2(3,2);
        point1(4,1), point1(4,2), 1, 0, 0, 0, -point1(4,1)*point2(4,1), -point1(4,2)*point2(4,1);
        0, 0, 0, point1(4,1), point1(4,2), 1, -point1(4,1)*point2(4,2), -point1(4,2)*point2(4,2)];
    
    B = [point2(1,1);point2(1,2);point2(2,1);point2(2,2);point2(3,1);point2(3,2);point2(4,1);point2(4,2)];
    
    H = A\B;
    
    inliner=[];
    %Map all points using the homagraphy and comparing distances between 
    %pre-dicted and observed locations to determine the number of inliers.
    for i =1:sizeM - 3
        Points = [true_matches(i, 4) true_matches(i, 3); 
            true_matches(i+1, 4) true_matches(i+1, 3);
            true_matches(i+2, 4) true_matches(i+2, 3); 
            true_matches(i+3, 4) true_matches(i+3, 3)];
       
        point_X = (H(1,1).*Points(:,1) + H(2,1).*Points(:,2) + H(3,1))./(H(7,1).*Points(:,1)+ H(8,1).*Points(:,2) + 1);
        point_Y = (H(4,1).*Points(:,1) + H(5,1).*Points(:,2) + H(6,1))./(H(7,1).*Points(:,1)+ H(8,1).*Points(:,2) + 1);
       
     
        dist = sqrt((point_X - true_matches(i:i+3, 2)).^2 - (point_Y - true_matches(i:i+3, 1)).^2);
        for j= 1:4
            if(dist(j,1) < threshold)
               numCount = numCount+1; 
               inliner=[inliner; point_X(j,1), point_Y(j,1), Points(j,1), Points(j,2)];
            end
        end
        inliner =unique(inliner,'rows');
    end

    if numCount > max
        max = numCount;
        New_inliner = inliner;
        New_H = H;
    end
end
end