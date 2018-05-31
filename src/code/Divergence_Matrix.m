function N = Divergence_Matrix(point,points)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
num = max(size(points));
points = points - point;
N = 0;
for i = 1:num
    N = N + norm(points(i,:));
end
N = N/num;
end

