function [id,closest_point] = closest_neighbour(point,ori_points)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
points = ori_points(:,1:2);
point = point(1:2);
points = points - point;
local_distance = sum(points.^2,2);
[~,I] = min(local_distance); 
closest_point = ori_points(I(1),:);
id = I(1);
end

