function x_new = Conver_MonteCarlo_Prop(Uncertain, x_nearest, T_prop)
	% Generate random control sequence
%     u = rand();
%     sim(point_system);
%     x_new.pos(1) = output;
%     u = 1;
%     T = 1;
%     xi = 1;
%     open_system('point_system');
%     set_param('point_system/u', 'Value', num2str(u));
%     set_param('point_system/T', 'Value', num2str(T));
%     set_param('point_system/xi', 'Value', num2str(xi));
%     coder.extrinsic('sim');
%     out = sim('point_system','SrcWorkspace','current');
%     new_out = out.get('yout').get('').Values.Data(1);
	% Simulate from x_nearest with the control sequence of duration T_prop
    while true
    u = [rand()*2 - 1; rand()*2-1];
    T_prop_temp = rand() * T_prop;
    x_new.pos = x_nearest.pos + u * T_prop_temp;
    for i = 1:1:length(Uncertain)
        ratio(i) = dist
	x_new.parent = x_nearest;
	x_new.cost = x_nearest.cost + dist(x_new, x_nearest);
    end
