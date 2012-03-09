function [ ] = visualize( data )
%VISUALIZE Attempts to visualize grid data in a meaningful way
% Author: Tyler Sanderson (tysand@umich.edu)
%   1D - Plot is used
%   2D - Surf is used
%   3D - Scanning color is used
%   Sparse 3D - Structured color is used

if ndims(data) == 2
    if size(data,1) == 1 || size(data,2) == 1
        plot(data);
    else
        data(data == bitmax) = 0;
        surf(data);
    end
end

% Detect sparse 3D
if ndims(data) == 3 && any(data(:) == bitmax)
    % Normalize values
    dims = size(data);
    data = data(:);
    minimum = min(data);
    maximum = max(data(data ~= bitmax));
    data = (data - minimum) ./ (maximum-minimum);
    data(data > 1.0) = bitmax;
    % Plot all valid data as cubes
    for ii = 1:length(data)
        if data(ii) ~= bitmax
            [x,y,z] = ind2sub(dims, ii);
            plot_cube(x,y,z,data(ii));
        end
    end
elseif ndims(data) == 3 % Dense 3D
    % Normalize values
    dims = size(data);
    data = data(:);
    minimum = min(data);
    maximum = max(data);
    data = (data - minimum) ./ (maximum-minimum);
    data = reshape(data, dims);
    ptime = 10 / size(data,3);
    % Plot one z layer at a time
    axis manual;
    xlim([0 size(data,1)+1])
    ylim([0 size(data,2)+1])
    zlim([0 size(data,3)+1])
    view(-37.5, 50);
    for z = 1:size(data,3)
        ii = 0;
        for x = 1:size(data,1)
            for y = 1:size(data,2)
                ii = ii + 1;
                handles(ii) = plot_square(x,y,z,data(x,y,z));
            end
        end
        pause(ptime);
        delete(handles);
    end
end
end

function h = plot_square(x,y,z,value)
    cx=[0 1 1 0] + x*ones(1,4);
    cy=[0 0 1 1] + y*ones(1,4);
    cz= z*ones(1,4);
    col=[1-value, value, 0];
    h = patch(cx,cy,cz,col);
end

function plot_cube(x,y,z,value)
    cx=[0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0] + x*ones(4,6);
    cy=[0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1] + y*ones(4,6);
    cz=[0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1] + z*ones(4,6);
    col=[1-value, value, 0];
    for i=1:6
        patch(cx(:,i),cy(:,i),cz(:,i),col);
    end
end

