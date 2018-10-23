function [path, x, y] = sourceSelect(SourcePath, option)
% Source folder
switch SourcePath
    case 1
        path = 'resources/DanaHallWay1/DSC_028%d.JPG';
    case 2
        path = 'resources/DanaHallWay2/DSC_028%d.JPG';
    case 3
        path = 'resources/DanaOffice/DSC_031%d.JPG';
end


if SourcePath == 1
    x= 1;
    y = 2;
elseif SourcePath == 2
    x = 5;
    y = 6;
elseif SourcePath == 3
    switch option
        case 1
            path = 'resources/DanaOffice/DSC_030%d.JPG';
            x = 8;
            y = 9;
        case 2
            x = 0;
            y = 1;
        case 3
            x = 1;
            y = 2;
        case 4
            x = 3;
            y = 4;
        case 5
            x = 5;
            y = 6;
        case 6
            x = 6;
            y = 7;
    end
end
end