function  sim_BRRT(origin,goal,u,dt,num_points,b,track)
tic;
start_point.pos = [origin(1),origin(2),Mountain_Height(origin(1),origin(2))];
stop_point.pos = [goal(1),goal(2),Mountain_Height(goal(1),goal(2))];


dim = 2; upperlimits = [2,2.5]; lowerlimits = [-2,0];

NumStep = 100000;

init_r = 0.25*u;

du = u*dt;

x0.pos = start_point.pos; x0.parent = x0;
x0.points = initialize(start_point.pos(1), start_point.pos(2), init_r, num_points);
x0.Da = 0;

Vactive = [x0];

if (track)
    figure();
end

for i = 1:NumStep
    
    if (mod(i,200) == 0 && track)
        close();
        figure();
        hold on;
        for j = 1:1:length(Vactive)
            plot3(Vactive(j).pos(1), Vactive(j).pos(2), Vactive(j).pos(3), 'b.');
            plot3([Vactive(j).pos(1) Vactive(j).parent.pos(1)], [Vactive(j).pos(2) Vactive(j).parent.pos(2)], [Vactive(j).pos(3) Vactive(j).parent.pos(3)], 'b');
        end
        
        title(["Step",num2str(i)]);
    end
    
    s_sample = Sample_Hill(dim, upperlimits, lowerlimits);
    x_nearest = BestNear_Naive(Vactive, s_sample);
    [x_new] = ori_biased_move(x_nearest.pos,x_nearest.points,s_sample.pos,du,dt,b);
    x_new.parent = x_nearest;
    Vactive = [Vactive, x_new];
    
    if dist(x_new,stop_point) <= du
       
        close(1);
        figure(1);
        hold on;
        for j = 1:1:length(Vactive)
            plot3(Vactive(j).pos(1), Vactive(j).pos(2), Vactive(j).pos(3), 'b.');
            plot3([Vactive(j).pos(1) Vactive(j).parent.pos(1)], [Vactive(j).pos(2) Vactive(j).parent.pos(2)], [Vactive(j).pos(3) Vactive(j).parent.pos(3)], 'b');
        end
        
        title(["Step",num2str(i)]);
        Da_record = [];
        stop_point.parent = x_new;
        figure();
        hold on;
        last_point = stop_point;
        current_point = x_new;
        while (~point_equal(current_point,start_point))
            Da_record = [current_point.Da,Da_record];
            plot3([current_point.pos(1),last_point.pos(1)],[current_point.pos(2),last_point.pos(2)],[current_point.pos(3),last_point.pos(3)],'b');
            last_point = current_point;
            current_point = last_point.parent;
        end
        title('Path');
        Da_record = [0,Da_record];
        Da_cum = cumsum(Da_record);
        Da_cum = Da_cum*dt;
        Path_cost = exp(Da_cum);
        figure();
        plot(Path_cost);
        disp("Finish");
        xlabel('Step');
        ylabel('Path Matrix');
        break;
    end
end
toc;
end

