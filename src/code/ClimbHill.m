function dx = ClimbHill(t, state, control)
	grad_x = @(x,y) cos(x+x*y)*(1+y);
	grad_y = @(x,y) 3 + cos(x+x*y)*x;
	fun_z = @(x,y) 3*y + sin(x + x*y);

	u = control(1);
	theta = control(2);

	dhdx = bsxfun(grad_x, state(1), state(2));
	dhdy = bsxfun(grad_y, state(1), state(2));

	norm_ = sqrt(dhdx^2 + dhdy^2);

	dx(1,1) = u*(dhdx * cos(theta)/norm_ - dhdy * sin(theta)/norm_);
	dx(2,1) = u*(dhdx * sin(theta)/norm_ + dhdy * cos(theta)/norm_);

end