function [next_point] = ori_biased_move(x_near,points,q_rand,du,dt,b)
%Hill Climb function
grad_x = @(x,y) cos(x+x*y)*(1+y);
grad_y = @(x,y) 3 + cos(x+x*y)*x;
fun_z = @(x,y) 3*y + sin(x + x*y);

local_close_nominal_point = zeros(1,3);
nominal_point = zeros(1,3);

x = x_near(1);
y = x_near(2);

dx = bsxfun(grad_x,x,y);
dy = bsxfun(grad_y,x,y);

theta_hill = atan2(dy,dx);
short_dist = inf;
for iter = 1:20
    R_theta = rand(1)*2*pi;
    theta = theta_hill + R_theta;
    
    dx = cos(theta);
    dy = sin(theta);
    dz = bsxfun(fun_z,x + dx,y + dy) - bsxfun(fun_z,x,y);
    
    norm_d = norm([dx,dy,dz]);
    
    local_x_next = x + dx/norm_d*du;
    local_y_next = y + dy/norm_d*du;
    local_z_next = bsxfun(fun_z, local_x_next,local_y_next);
    local_point_next = [local_x_next,local_y_next,local_z_next];
    
    [local_new_points,local_Da] = convergent(points,R_theta,du,dt);
    local_dist = norm(local_point_next(1:2) - q_rand(1:2)) * exp(b*local_Da);
    if local_dist < short_dist
        x_new = local_point_next;
        new_points = local_new_points;
        short_dist = local_dist;  
        Da = local_Da;
    end
end
next_point.pos = x_new;
next_point.points = new_points;
next_point.Da = Da;

end

