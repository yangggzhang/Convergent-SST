%Hill Climb
%x 0-3 y 0-3
clear;close all;
addpath('code');

% Particles radius and number
init_r = 0.05;
num_points = 100;

% Setup System
u = 0.3; % Forward speed
dt = 0.5; % Propagation time
du = u*dt;
b = 1; % Biased constant

% x0 initial point
% x data structure: x.pos position of x; x.parent parent of x
start_point.pos = [0, 0, Mountain_Height(0,0)];
stop_point.pos = [2,2.3,Mountain_Height(2,2.3)];
dim = 2; upperlimits = [2,2.5]; lowerlimits = [-2,0];

%%
NumStep = 22000;

b_test = [0.25, 0.75, 0.5, 1.0, 1.25, 1.5]; % Test for different b

for b_count = 1:1:length(b_test)
    b = b_test(b_count)
    tic;
    
    
    x0.pos = start_point.pos; x0.parent = x0;
    x0.points = initialize(start_point.pos(1), start_point.pos(2), init_r, num_points);
    x0.Da = 0; x0.true_cost = 0; x0.cost = 0;
    
    Vactive = [x0];
    
    Path_cost_min = [];
    Path_cost_end = [];
    Time_consuming = [];
    Path_quality = [];
    Biased_quality = [];
    Nodes_num = [];
    
    
    
    for i = 1:NumStep
        s_sample = Sample_Hill(dim, upperlimits, lowerlimits);
        x_nearest = BestNear_Naive(Vactive, s_sample);
        [x_new] = ori_biased_move(x_nearest.pos,x_nearest.points,s_sample.pos,du,dt,b);
        x_new.parent = x_nearest;
        temp_dist = dist(x_nearest,x_new);
        x_new.cost = x_nearest.cost + temp_dist*exp(b*x_new.Da);
        x_new.true_cost = x_nearest.true_cost + temp_dist;
        Vactive = [Vactive, x_new];
        
        if mod(i, 2000) == 0
            min_cost = [];
            end_cost = [];
            quality = [];
            biased_quality = [];
            for j = 1:1:length(Vactive)
                current_point = Vactive(j);
                if isLeaf(Vactive, [], current_point)
                    Da_record = [];
                    temp_quality = current_point.true_cost;
                    temp_biased_quality = current_point.cost;
                    while (~point_equal(current_point, start_point))
                        Da_record = [current_point.Da, Da_record];
                        last_point = current_point;
                        current_point = last_point.parent;
                        temp_quality = temp_quality + current_point.true_cost;
                        temp_biased_quality = temp_biased_quality + current_point.cost;
                    end
                    Da_cum = cumsum(Da_record);
                    Da_cum = Da_cum*dt;
                    min_cost = [min_cost, min(exp(Da_cum))];
                    end_cost = [end_cost, exp(Da_cum(end))];
                    quality = [quality, temp_quality];
                    biased_quality = [biased_quality, temp_biased_quality];
                end
            end
            Path_cost_min = [Path_cost_min, min(min_cost)]
            Path_cost_end = [Path_cost_end, mean(end_cost)]
            Time_consuming = [Time_consuming, toc]
            Path_quality = [Path_quality, mean(quality)]
            Biased_quality = [Biased_quality, mean(biased_quality)]
            Nodes_num = [Nodes_num, length(Vactive)]
        end
        
    end
    toc;
    
    save(['brrt_b', num2str(100*b) ,'.mat'], 'Path_cost_min', 'Path_cost_end', 'Time_consuming', 'Path_quality', 'Biased_quality', 'Nodes_num');
    
end


