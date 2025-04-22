clear
close all
load('D:\Flycontrol\Matlab_simulation\第五章实验\low_density_simple_data.mat')


% load('D:\Flycontrol\Matlab_simulation\第五章实验\high_density_simple_data_V2.mat')

colors = [ ...
    0.9 0.4 0.4;  % 红色
    0.4 0.6 0.9;  % 蓝色
    0.5 0.8 0.5;  % 绿色
    0.8 0.6 0.9]; % 紫色
%%  de
Time = figure;   %角度
ax1 = subplot(4, 1, 1);
data_matrix = [rrt_total_time',rrt_Connect_total_time',rrt_star_total_time',rrt_star_connecttotal_time'];
p = boxplot(data_matrix, 'Labels', {'RRT','RRT_Connect','RRT*','C-RRT*'});
yticks([0.01, 0.1, 1, 10, 50]);
set(gca, 'YTick', yticks);
ylabel('\fontname{宋体}搜索耗时\fontname{Times New Roman}(s)');
grid on;
% 设置纵坐标为对数刻度
set(gca, 'YScale', 'log');
% ylim([0.02 100])
ylim([0 50])
h = findobj(gca, 'Tag', 'Box'); % 获取箱体对象
for i=1:length(h)
    patch(get(h(i), 'XData'), get(h(i), 'YData'), colors(i,:), 'FaceAlpha', 0.5); % 修改箱体颜色
end
medians = findobj(p, 'Tag', 'Median');
for i = 1:length(medians)
    medians(i).LineWidth = 0.75;      % 加粗线条
    medians(i).Color = [1 0 0];  % 改为深紫色（你可替换为想要的颜色）
end
% 隐藏异常值（默认显示）
set(findobj(gca, 'Tag', 'Outliers'), 'Visible', 'off');
set(Time.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% fig = gcf;
% fig.Units = 'centimeters';
% fig.Position = [0 0 16 6]; 
% fimename = 'D:\Flycontrol\SCUT_thesis\Fig\chapter5\低密度障碍物下四种算法搜索时间对比.pdf';
% PlotToFileColorPDF(Time,fimename,16,6);

%%
% sampleNode = figure;   %角度
ax2 = subplot(4, 1, 2);
data_matrix = [rrt_total_sampleNode',rrt_Connect_total_sampleNode',rrt_star_total_sampleNode',rrt_star_connecttotal_sampleNode'];
p = boxplot(data_matrix, 'Labels', {'RRT','RRT_Connect','RRT*','C-RRT*'});
ylabel('\fontname{宋体}采样点数\fontname{Times New Roman}(\fontname{宋体}个\fontname{Times New Roman})');
grid on;
% 设置纵坐标为对数刻度
set(gca, 'YScale', 'log');
yticks([50, 100, 500, 1000]);
set(gca, 'YTick', yticks);
% ylim([40 1000])
ylim([45 1100])
h = findobj(gca, 'Tag', 'Box'); % 获取箱体对象
for i=1:length(h)
    patch(get(h(i), 'XData'), get(h(i), 'YData'), colors(i,:), 'FaceAlpha', 0.5); % 修改箱体颜色
end
medians = findobj(p, 'Tag', 'Median');
for i = 1:length(medians)
    medians(i).LineWidth = 0.75;      % 加粗线条
    medians(i).Color = [1 0 0];  % 改为深紫色（你可替换为想要的颜色）
end
% 隐藏异常值（默认显示）
set(findobj(gca, 'Tag', 'Outliers'), 'Visible', 'off');
set(Time.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')
% 
% %%
% len = figure;   %角度
ax3 = subplot(4, 1, 3);
data_matrix = [rrt_total_len',rrt_Connect_total_len',rrt_star_total_len',rrt_star_connecttotal_len'];
p = boxplot(data_matrix, 'Labels', {'RRT','RRT_Connect','RRT*','C-RRT*'});
ylabel('\fontname{宋体}路径长度\fontname{Times New Roman}(m)');
grid on;
h = findobj(gca, 'Tag', 'Box'); % 获取箱体对象
for i=1:length(h)
    patch(get(h(i), 'XData'), get(h(i), 'YData'), colors(i,:), 'FaceAlpha', 0.5); % 修改箱体颜色
end
medians = findobj(p, 'Tag', 'Median');
for i = 1:length(medians)
    medians(i).LineWidth = 0.75;      % 加粗线条
    medians(i).Color = [1 0 0];  % 改为深紫色（你可替换为想要的颜色）
end
% ylim([79 125])
ylim([78 115])
% 隐藏异常值（默认显示）
set(findobj(gca, 'Tag', 'Outliers'), 'Visible', 'off');
set(Time.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')

% 
% %%
% ang = figure;   %角度
ax4 = subplot(4, 1, 4);
data_matrix = [rrt_total_ang',rrt_Connect_total_ang',rrt_star_total_ang',rrt_star_connecttotal_ang'];
p = boxplot(data_matrix, 'Labels', {'RRT','RRT_Connect','RRT*','C-RRT*'});
ylabel('\fontname{宋体}路径弯曲度\fontname{Times New Roman}(rad)');
grid on;
h = findobj(gca, 'Tag', 'Box'); % 获取箱体对象
for i=1:length(h)
    patch(get(h(i), 'XData'), get(h(i), 'YData'), colors(i,:), 'FaceAlpha', 0.5); % 修改箱体颜色
end
medians = findobj(p, 'Tag', 'Median');
for i = 1:length(medians)
    medians(i).LineWidth = 0.75;      % 加粗线条
    medians(i).Color = [1 0 0];  % 改为深紫色（你可替换为想要的颜色）
end
% ylim([0 50])
ylim([0 45])
% 隐藏异常值（默认显示）
set(findobj(gca, 'Tag', 'Outliers'), 'Visible', 'off');
set(Time.CurrentAxes, 'FontSize', 10,'FontName','Times New Roman','LabelFontSizeMultiplier', 1,'TitleFontSizeMultiplier',1,'LineWidth',0.5,'Xcolor','black','Ycolor','black','Zcolor','black')


set(ax1, 'Position', [0.1, 0.75, 0.8, 0.19]); % 第一个子图
set(ax2, 'Position', [0.1, 0.52, 0.8, 0.19]); % 第二个子图
set(ax3, 'Position', [0.1, 0.29, 0.8, 0.19]); % 第三个子图
set(ax4, 'Position', [0.1, 0.06, 0.8, 0.19]); % 第三个子图

fig = gcf;
fig.Units = 'centimeters';
fig.Position = [0 0 16 20]; 
fimename = 'D:\Flycontrol\SCUT_thesis\Fig\chapter5\低密度障碍物下四种算法性能对比_V2.pdf';
PlotToFileColorPDF(Time,fimename,16,20);