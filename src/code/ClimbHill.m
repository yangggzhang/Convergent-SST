function dx = ClimbHill(t, state, control)
	grad_x = @(x,y) cos(x+x*y)*(1+y);
	grad_y = @(x,y) 3 + cos(x+x*y)*x;
	fun_z = @(x,y) 3*y + sin(x + x*y);

	u = control(1);
	theta = control(2);

	dhdx = bsxfun(grad_x, state(1), state(2));
	dhdy = bsxfun(grad_y, state(1), state(2));

	theta_heading = atan2(dhdy, dhdx) + theta;

	deltax = cos(theta_heading);
	deltay = sin(theta_heading);
	deltah = bsxfun(fun_z, state(1) + deltax, state(2) + deltay) - bsxfun(fun_z, state(1), state(2));

	norm_d = norm([deltax, deltay, deltah]);

	dx(1,1) = deltax/norm_d*u;
	dx(2,1) = deltay/norm_d*u;

end