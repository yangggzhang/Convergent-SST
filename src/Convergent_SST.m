clear; clc; clear;
addpath('code');
tic

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

x0.pos = start_point.pos; x0.parent = x0; 
x0.cost = 1; % Divergence cost (cost = exp(b*Da))
% x0.cost = 0; % Biased cost (cost = dist*exp(b*Da))
x0.points = initialize(start_point.pos(1), start_point.pos(2), init_r, num_points);
x0.Da = 0; x0.true_cost = 0;

% Setup the tree and SST parameter
Vactive = [x0]; % Active vertices set
Vinactive = []; % Inactive vertices set
s0.pos = x0.pos; s0.rep = x0;
S = [s0]; % Witness set

delta_s = du*0.9; % Witness representation radius
delta_v = 0.00; % Radius for computing BestNear

%% 
NumStep = 60000;
num_prune = 0;

Path_cost_min = []; % Store minimum cost along the path
Path_cost_end = []; % Store the average cost of origin to all the leaf nodes
Time_consuming = []; % Store computational time
Path_quality = [];  % Store path quality defined as the average length of all paths
Biased_quality = []; % Store biased quality defined as the average biased legnth of all paths
Nodes_num = []; % Store number of nodes

for i = 1:1:NumStep
    s_sample = Sample_Hill(dim, upperlimits, lowerlimits);
    x_nearest = BestNear(Vactive, s_sample, delta_v);
    [x_new] = biased_move(x_nearest.pos,x_nearest.points,s_sample.pos,du,dt,b);
    x_new.parent = x_nearest;
    temp_dist = dist(x_nearest,x_new);
    x_new.cost = x_nearest.cost*exp(b*x_new.Da); % Divergence Cost
%     x_new.cost = x_nearest.cost+temp_dist*exp(b*x_new.Da); % Biased Cost
    x_new.true_cost = x_nearest.true_cost + temp_dist;
    
    if CollisionFree(x_new)
        [s_new, wit_index] = Nearest(S, x_new);
        if dist(x_new, s_new) > delta_s
            temp_s.pos = x_new.pos;
            temp_s.rep = 0;
            S = [S, temp_s];
            wit_index = length(S);
            s_new.pos = x_new.pos;
            s_new.rep = 0;
        end
        x_peer = s_new.rep;
        if ~isstruct(x_peer)
            S(wit_index).rep = x_new;
            s_new.rep = x_new;
            Vactive = [Vactive, x_new];
        elseif x_new.cost < x_peer.cost
            % Remove(x_peer, Vactive);
            index_remove = SearchIndex(Vactive, x_peer);
            Vactive(index_remove) = [];
            
            Vinactive = [Vinactive, x_peer];
            S(wit_index).rep = x_new;
            s_new.rep = x_new;
            Vactive = [Vactive, x_new];
            while (isLeaf(Vactive, Vinactive, x_peer) && isIn(Vinactive, x_peer))
                x_parent = x_peer.parent;
                Vinactive_temp = PruneLeaf(Vinactive, x_peer);
                Vinactive = Vinactive_temp;
                x_peer = x_parent;
                num_prune = num_prune + 1;
            end
        end
        
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
%             Biased_quality = [Biased_quality, mean(biased_quality)]
            Nodes_num = [Nodes_num, length(Vactive)+length(Vinactive)]
        end
        
    end
end
toc

save(['s_method2_deltas09_b', num2str(100*b) ,'.mat'], 'Path_cost_min', 'Path_cost_end', 'Time_consuming', 'Path_quality', 'Nodes_num')

figure(); hold on;

for j = 1:1:length(Vactive)
    plot3(Vactive(j).pos(1), Vactive(j).pos(2), Vactive(j).pos(3), 'b.');
    plot3([Vactive(j).pos(1) Vactive(j).parent.pos(1)], [Vactive(j).pos(2) Vactive(j).parent.pos(2)], [Vactive(j).pos(3) Vactive(j).parent.pos(3)], 'b');
end

for j = 1:1:length(Vinactive)
    plot3(Vinactive(j).pos(1), Vinactive(j).pos(2), Vinactive(j).pos(3), 'r.');
    plot3([Vinactive(j).pos(1) Vinactive(j).parent.pos(1)], [Vinactive(j).pos(2) Vinactive(j).parent.pos(2)], [Vinactive(j).pos(3) Vinactive(j).parent.pos(3)], 'b');
end
