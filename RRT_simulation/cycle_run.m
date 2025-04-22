clear
close all
global path_len pathNode_size sampleNode_size time sum_ang
count = 100; % 执行次数
rrt_star_connecttotal_len = zeros(1,count);
% rrt_star_connecttotal_pathNode = zeros(1,count);
rrt_star_connecttotal_sampleNode = zeros(1,count);
rrt_star_connecttotal_time = zeros(1,count);
rrt_star_connecttotal_ang = zeros(1,count);
count = 200; % 执行次数
for index = 1:1:count  
    run('RRT_Star_Connect.m');  % 执行脚本
    % run('RRT_star1.m');
    % run('RRT_Connect.m');
    % run('RRT.m');
    rrt_star_connecttotal_len(index) = path_len;
    % rrt_star_connecttotal_pathNode(i) = pathNode_size;
    rrt_star_connecttotal_sampleNode(index) = sampleNode_size;
    rrt_star_connecttotal_time(index) = time;
    rrt_star_connecttotal_ang(index) = sum_ang;
    progress = index * 100 / count
end
average_len = sum(rrt_star_connecttotal_len) / count
% average_pathNode = sum(rrt_star_connecttotal_pathNode) / count
average_sampleNode = sum(rrt_star_connecttotal_sampleNode) / count
average_time = sum(rrt_star_connecttotal_time) / count
average_ang = sum(rrt_star_connecttotal_ang) / count