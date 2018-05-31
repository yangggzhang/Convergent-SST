function [points_next,Da] = convergent(points,R_theta,du,dt)
%if convergent or not
[num,~] = size(points);


%Hill Climb function
grad_x = @(x,y) cos(x+x.*y).*(1+y);
grad_y = @(x,y) 3 + cos(x+x.*y).*x;
direction = @(x,y) atan2(y,x);
z = @(x,y) 3*y + sin(x + x.*y);


dx = bsxfun(grad_x,points(:,1),points(:,2));
dy = bsxfun(grad_y,points(:,1),points(:,2));
theta = bsxfun(direction,dx(:),dy(:));
theta = theta + R_theta;
dx = cos(theta);
dy = sin(theta);
dz = bsxfun(z,points(:,1) + dx,points(:,2) + dy) - bsxfun(z,points(:,1),points(:,2));

norm3D = zeros(num,1);
for i = 1:num
    norm3D(i) = norm([dx(i),dy(i),dz(i)]);
end

points_next = points;

dx = dx./norm3D*du;
dy = dy./norm3D*du;

points_next(:,1) = points(:,1) + dx;
points_next(:,2) = points(:,2) + dy;
points_next(:,3) = bsxfun(z, points(:,1),points(:,2));

Da = average_divergence(points,points_next,dt);


% diff_de = zeros(num,1);
% diff_nu = zeros(num,1);
% for i = 1:num
%     diff_de(i) = norm(points(i,:) - nominal_point);
%     diff_nu(i) = norm(points_next(i,:) - nominal_point_next);
% end
% 
% ratio = diff_nu./diff_de;
% 
% [~,I] = max(ratio);
% Dm = 1/dt*log(ratio(I(1)));
% 
% if Dm < threshold
%     bool = 1;
% end

end

