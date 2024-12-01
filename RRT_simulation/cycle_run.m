global len pathNode sampleNode time
total_len = 0;
total_pathNode = 0;
total_sampleNode = 0;
total_time = 0;
count = 1000; % 执行次数
for index = 1:1:count  
%     run('RRT_Star_Connect.m');  % 执行脚本
%     run('RRT_star1.m');
%     run('RRT_Connect.m');
    run('RRT.m');
    total_len = total_len + len;
    total_pathNode = total_pathNode + pathNode;
    total_sampleNode = total_sampleNode + sampleNode;
    total_time = total_time + time;
    progress = index * 100 / count
end
average_len = total_len / count
average_pathNode = total_pathNode / count
average_sampleNode = total_sampleNode / count
average_time = total_time / count