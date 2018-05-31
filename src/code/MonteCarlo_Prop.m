function x_new = MonteCarlo_Prop(x_nearest, T_prop, u)
	% Generate random control sequence
%     u = rand();
%     sim(point_system);
%     x_new.pos(1) = output;
    grad_x = @(x,y) cos(x+x*y)*(1+y);
    grad_y = @(x,y) 3 + cos(x+x*y)*x;
    fun_z = @(x,y) 3*y + sin(x + x*y);
    
    x = x_nearest.pos(1);
    y = x_nearest.pos(2);

    dx = bsxfun(grad_x,x,y);
    dy = bsxfun(grad_y,x,y);

    theta_hill = atan2(dy,dx);
    
    R_theta = rand(1)*2*pi;
    theta = theta_hill + R_theta;
    
    dx = cos(theta);
    dy = sin(theta);
    dz = bsxfun(fun_z,x + dx,y + dy) - bsxfun(fun_z,x,y);
    
    norm_d = norm([dx,dy,dz]);
    
    local_x_next = x + dx/norm_d*(u*T_prop);
    local_y_next = y + dy/norm_d*(u*T_prop);
    local_z_next = bsxfun(fun_z, local_x_next,local_y_next);
    x_new.pos = [local_x_next,local_y_next,local_z_next];
    
    x_new.parent = x_nearest;
    x_new.cost = x_nearest.cost + dist(x_new, x_nearest);

%     open_system('point_system');
%     set_param('point_system/u', 'Value', num2str(u));
%     set_param('point_system/T', 'Value', num2str(T));
%     set_param('point_system/xi', 'Value', num2str(xi));
%     coder.extrinsic('sim');
%     out = sim('point_system','SrcWorkspace','current');
%     new_out = out.get('yout').get('').Values.Data(1);
	% Simulate from x_nearest with the control sequence of duration T_prop
% 
%     x_new.pos(1) = x_nearest.pos(1) + rand() * T_prop;
%     x_new.pos(2) = x_nearest.pos(2) + rand() * T_prop;
% 	x_new.parent = x_nearest;
% 	x_new.cost = x_nearest.cost + dist(x_new, x_nearest);
