clear; clc; close all;

% figure(1);
b_test = [0.5, 1.0, 1.5];


for b_count = 1:1:length(b_test)
    b = b_test(b_count);
    num_data_plot = 10;
    figure(b_count + 1);
    hold on;
    filename1 = ['s_method1_deltas09_b',num2str(100*b), '.mat'];
    load(filename1);
    x = [1:1:length(Path_quality(1:num_data_plot))] * 2000;
    h1 = plot(x,Path_quality(1:num_data_plot),'-ks',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    
    filename2 = ['s_method2_deltas09_b',num2str(100*b), '.mat'];
    load(filename2);
    x = [1:1:length(Path_quality(1:num_data_plot))] * 2000;
    h2 = plot(x,Path_quality(1:num_data_plot),'-k^',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    title(['b = ', num2str(b), ': Average Path Quality'], 'FontSize', 16);
    xlabel('Number of Iterations', 'FontSize', 14);
    set(gca,'fontsize',14)
    
    legend([h1 h2], {'Method 1','Method 2'}, 'FontSize',14);
    
    %%
    
    figure(b_count + 2*length(b_test));
    hold on;
    filename1 = ['s_method1_deltas09_b',num2str(100*b), '.mat'];
    load(filename1);
    x = [1:1:length(Path_cost_end(1:num_data_plot))] * 2000;
    h1 = plot(x,Path_cost_end(1:num_data_plot),'-ks',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    
    filename2 = ['s_method2_deltas09_b',num2str(100*b), '.mat'];
    load(filename2);
    x = [1:1:length(Path_cost_end(1:num_data_plot))] * 2000;
    h2 = plot(x,Path_cost_end(1:num_data_plot),'-k^',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    title(['b = ', num2str(b), ': Average Path Divergence at End'], 'FontSize', 16);
    xlabel('Number of Iterations', 'FontSize', 14);
    set(gca,'fontsize',14)
    
    legend([h1 h2], {'Method 1','Method 2'}, 'FontSize',14);
    
    
    %%
    
    figure(b_count + 3*length(b_test));
    hold on;
    filename1 = ['s_method1_deltas09_b',num2str(100*b), '.mat'];
    load(filename1);
    x = [1:1:length(Time_consuming(1:num_data_plot))] * 2000;
    h1 = plot(x,Time_consuming(1:num_data_plot),'-ks',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    
    filename2 = ['s_method2_deltas09_b',num2str(100*b), '.mat'];
    load(filename2);
    x = [1:1:length(Time_consuming(1:num_data_plot))] * 2000;
    h2 = plot(x,Time_consuming(1:num_data_plot),'-k^',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    title(['b = ', num2str(b), ': Time Consuming'], 'FontSize', 16);
    xlabel('Number of Iterations', 'FontSize', 14);
    set(gca,'fontsize',14)
    
    legend([h1 h2], {'Method 1','Method 2'}, 'FontSize',14);
    
    
    %%
    figure(b_count + 4*length(b_test));
    hold on;
    filename1 = ['s_method1_deltas09_b',num2str(100*b), '.mat'];
    load(filename1);
    x = [1:1:length(Nodes_num(1:num_data_plot))] * 2000;
    h1 = plot(x,Nodes_num(1:num_data_plot),'-ks',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    
    filename2 = ['s_method2_deltas09_b',num2str(100*b), '.mat'];
    load(filename2);
    x = [1:1:length(Nodes_num(1:num_data_plot))] * 2000;
    h2 = plot(x,Nodes_num(1:num_data_plot),'-k^',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    title(['b = ', num2str(b), ': Number of Nodes'], 'FontSize', 16);
    xlabel('Number of Iterations', 'FontSize', 14);
    set(gca,'fontsize',14)
    
    legend([h1 h2], {'Method 1','Method 2'}, 'FontSize',14);

end
    
    
