function [points_next,Da] = convergent(points,R_theta,du,dt)
%if convergent or not

z = @(x,y) 3*y + sin(x + x.*y);

[num,~] = size(points);

tspan_ode1 = 0:dt/100:dt;
tspan_ode45 = [0, dt];

for i = 1:num
	% [t_sim, x_sim] = ode1(@ClimbHill_ode1, tspan_ode1, [points(i,1); points(i,2)], [du/dt, R_theta]);
	[t_sim, x_sim] = ode45(@ClimbHill_ode45, tspan_ode45, [points(i,1); points(i,2); du/dt; R_theta]);
	points_next(i,1) = x_sim(end,1);
	points_next(i,2) = x_sim(end,2);
end

points_next(:,3) = bsxfun(z, points_next(:,1), points_next(:,2));

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

