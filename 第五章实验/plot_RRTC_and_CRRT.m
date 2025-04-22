clear
close all
% load('D:\Flycontrol\Matlab_simulation\第五章实验\RRT-Connect.mat')
load('D:\Flycontrol\Matlab_simulation\第五章实验\RRT_Star_Connect_V0.mat')

RRTC = figure;
map = fun_defMap; % 定义地图
%% 可视化地图
% 画起点和终点
scatter(startPos(1),startPos(2),80,'b','filled','pentagram')
hold on
scatter(goalPos(1),goalPos(2),80,'g','filled','pentagram')
rectangle('Position', [0, 0, 70, 50]);
for i = 1:size(map.obs_polygon,2)
    % per obs
    vertexs = map.obs_polygon{i}{1};
    fill(vertexs(:,1),vertexs(:,2), [0.3 0.3 0.3],'EdgeColor', 'none', 'HandleVisibility', 'off'); hold on;
end

t = linspace(0, 2*pi);
for i = 1:length(map.obs_circle)-1
    x = cos(t)*map.obs_circle(i,3)+map.obs_circle(i,1);
    y = sin(t)*map.obs_circle(i,3)+map.obs_circle(i,2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none','HandleVisibility', 'off'); hold on;
end
    x = cos(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),1);
    y = sin(t)*map.obs_circle(length(map.obs_circle),3)+map.obs_circle(length(map.obs_circle),2);
    fill(x,y,[0.3 0.3 0.3],'EdgeColor', 'none'); hold on;

axis equal
xlim([0 map_size(1)]);
ylim([0 map_size(2)]);

%% 画采样点与路径
for i = 2:size(treeNodes1,2)-1
    plot([treeNodes1(i).parentNode(1),treeNodes1(i).node(1)],...
        [treeNodes1(i).parentNode(2),treeNodes1(i).node(2)],'b','linewidth',0.5,'HandleVisibility', 'off');
end
    plot([treeNodes1(size(treeNodes1,2)).parentNode(1),treeNodes1(size(treeNodes1,2)).node(1)],...
        [treeNodes1(size(treeNodes1,2)).parentNode(2),treeNodes1(size(treeNodes1,2)).node(2)],'b','linewidth',0.5);
for i = 2:size(treeNodes2,2)-1
    plot([treeNodes2(i).parentNode(1),treeNodes2(i).node(1)],...
        [treeNodes2(i).parentNode(2),treeNodes2(i).node(2)],'g','linewidth',0.5,'HandleVisibility', 'off');
end
    plot([treeNodes2(size(treeNodes2,2)).parentNode(1),treeNodes2(size(treeNodes2,2)).node(1)],...
        [treeNodes2(size(treeNodes2,2)).parentNode(2),treeNodes2(size(treeNodes2,2)).node(2)],'g','linewidth',0.5);

plot(path_opt(:,1),path_opt(:,2),'r','linewidth',1.5)

% plot(smoothed_path(:,1),smoothed_path(:,2),'r','linewidth',1.5)
axis off;
h = legend('\fontname{宋体}起点', '\fontname{宋体}目标点','\fontname{宋体}障碍物', ...
    '\fontname{Times New Roman}T_1\fontname{宋体}采样路径', ...
    '\fontname{Times New Roman}T_2\fontname{宋体}采样路径','\fontname{宋体}最终路径');
% h = legend('\fontname{宋体}起点', '\fontname{宋体}目标点','\fontname{宋体}障碍物', ...
%     '\fontname{宋体}障碍物膨胀边缘');
h.ItemTokenSize(1) = 15;
set(h,'NumColumns',1,'location','eastoutside','Box','off');
set(RRTC.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', ...
    1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')

fig = gcf;
fig.Units = 'centimeters';
fig.Position = [5 10 14 7]; 

% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\chapter5\RRT-Connect算法随机树生成示意图.pdf';
% PlotToFileColorPDF(RRTC,fimename,14,7);

% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\chapter5\C_RRT_Star算法随机树生成示意图.pdf';
% PlotToFileColorPDF(RRTC,fimename,14,7);
