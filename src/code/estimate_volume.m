function V = estimate_volume(points)
X = points(:,1);
Y = points(:,2);
Z = points(:,3);
[~, V] = convhull(X,Y,Z);
end

