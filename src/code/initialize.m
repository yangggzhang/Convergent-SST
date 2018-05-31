function points = initialize(x,y,r,N)
z = @(x,y) 3*y + sin(x + x*y);

points = zeros(N,3);
for i = 1:N
    rand_x = x + rand(1)*2*r - r;
    rand_y = y + rand(1)*2*r - r;
    points(i,:) = [rand_x,rand_y,bsxfun(z,rand_x,rand_y)];
end

