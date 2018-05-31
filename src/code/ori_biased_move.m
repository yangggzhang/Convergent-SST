function [next_point] = ori_biased_move(x_near,points,q_rand,du,dt,b)
%Hill Climb function
fun_z = @(x,y) 3*y + sin(x + x*y);

local_close_nominal_point = zeros(1,3);
nominal_point = zeros(1,3);

x = x_near(1);
y = x_near(2);

tspan = 0:dt/100:dt;

short_dist = inf;
for iter = 1:20
    R_theta = rand(1)*2*pi;

    [t_sim, x_sim] = ode1(@ClimbHill, tspan, [x; y], [du/dt, R_theta]);

    local_x_next = x_sim(end,1);
    local_y_next = x_sim(end,2);
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

