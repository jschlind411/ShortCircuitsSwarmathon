%Data for one Robot in cluster
one_cluster_data = dlmread('1Robot_Cluster.csv',',',1,0);
mean_one_cluster_base_data = floor(mean(one_cluster_data(:,3)));
mean_one_cluster_SC_data = floor(mean(one_cluster_data(:,2)));

%Data for three robots in cluster
three_cluster_data = dlmread('3Robot_Cluster.csv',',',1,0);
mean_three_cluster_base_data = floor(mean(three_cluster_data(:,3)));
mean_three_cluster_SC_data = floor(mean(three_cluster_data(:,2)));

%Data for thee robots in random
three_random_data = dlmread('3Robot_Random.csv',',',1,0);
mean_three_random_base_data = floor(mean(three_random_data(:,3)));
mean_three_random_SC_data = floor(mean(three_random_data(:,2)));

%Label for one robot bar graph
one_c = categorical({'ClusterBase','ClusterRGFPA'});
one_cluster_mean = [mean_one_cluster_base_data mean_one_cluster_SC_data];

%Figure 1
figure
bar(one_c,one_cluster_mean);
ylim([0 5]);
hold on;
title('One Simulated Robot');
xlabel('Resource Distribution');
ylabel('Average collected within 15 mins')
hold off;

%Label for three robots bar graph
three_c = categorical({'ClusterBase', 'ClusterRGFPA', 'RandomBase', 'RandomRGFPA'})
three_robot_mean = [mean_three_cluster_base_data mean_three_cluster_SC_data mean_three_random_base_data mean_three_random_SC_data];

%Figure 2
figure
bar(three_c,three_robot_mean)
ylim([0 20])
hold on;
title('Three Simulated Robots')
xlabel('Resource Distribution')
ylabel('Average collected within 15 mins')
hold off;

% %Subplots
% FigH     = figure;
% 
% SPH1 = subplot(2,1,1);
% bar(one_c,one_cluster_mean)
% ylim([0 5])
% 
% title('One Robot Stimulation')
% xlabel('Resource Distribution Type')
% % ylabel('Average cubes collected within 15 mins')
% 
% SP2H = subplot(2,1,2);
% three_c = categorical({'ClusterBase', 'ClusterSC', 'RandomBase', 'RandomSC'})
% three_robot_mean = [mean_three_cluster_base_data mean_three_cluster_SC_data mean_three_random_base_data mean_three_random_SC_data];
% bar(three_c,three_robot_mean)
% ylim([0 20])
% 
% title('Three Robots Simulation')
% xlabel('Resource Distribution Type')
% % ylabel('Average cubes collected within 15 mins')
% 
% YLabel1H = get(SP1H,'YLabel');
% set(YLabel1H,'String','YLabelThatIWant');
% set(YLabel1H,'Position',[-0.0542 0 0]);
% Title1H = get(SP1H,'Title');
% set(Title1H,'String','TheTitleThatIWant');
