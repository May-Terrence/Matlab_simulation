% 型值点反求
clc
clear
close all

%% 数据定义
d = 3.5;
k = 3;                                    % k阶、k-1次B样条
classFlag = 1;                                  %1,2分别绘制均匀B样条曲线、准均匀B样条曲线
Q = [0,-d/2; 10,-d/2; 20,-d/2; 30,d/2; 40,d/2; 50,d/2]';
P = dataPoint2ControlPoint(Q);

n = size(P,2)-1;                          % n是控制点个数，从0开始计数

%% 生成B样条曲线
Bik = zeros(n+1, 1);
nodeVector = getNodeVector(n, k, classFlag);
path = [];
for u = (k-1)/(n+k) : 0.001 : 1-(k-1)/(n+k)
    for i = 0 : 1 : n
        Bik(i+1, 1) = BaseFunction(i, k-1 , u, nodeVector);
    end
    p_u = P * Bik;
    path(end+1,:) = p_u;
end


%% 计算长度和曲率
x = path(:,1)';
y = path(:,2)';
diffX = diff(path(:,1));
diffY = diff(path(:,2));
cumLength = cumsum(sqrt(diffX.^2 + diffY.^2));   %长度
heading = atan2(diffY, diffX);
for i = 1:length(x)-2
    cur(i) = getCur(x(i:i+2)',y(i:i+2)');
end
cur(end+1) = cur(end);

%% 画曲率图
figure
hold on
grid on
% 主体图形绘制
plot(cumLength,cur,'LineWidth', 3,  'Color', 'k');
% 坐标轴
set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
hXLabel = xlabel('路径长度/m');
hYLabel = ylabel('曲率/m^-^1');
% 修改刻度标签字体和字号
set(gca, 'FontSize', 16),...
    set([hXLabel, hYLabel], 'FontName',  'simsun')
set([hXLabel, hYLabel], 'FontSize', 16)

% 画航向角图
figure
hold on
grid on
% 主体图形绘制
plot(cumLength, heading,'LineWidth', 3,  'Color', 'b');
% 坐标轴
set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
hXLabel = xlabel('路径长度/m');
hYLabel = ylabel('航向角/rad');
% 修改刻度标签字体和字号
set(gca, 'FontSize', 16),...
set([hXLabel, hYLabel], 'FontName',  'simsun')
set([hXLabel, hYLabel], 'FontSize', 16)
%% 画图
d = 3.5;               % 道路标准宽度
W = 1.8;               % 汽车宽度
L = 4.7;               % 车长
figure
len_line = 60;

% 画灰色路面图
GreyZone = [-10,-d-0.5; -10,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([path(1,1),path(1,1),path(1,1)-L,path(1,1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'y')
fill([35,35,35-L,35-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')

% 画分界线
plot([-10, len_line],[0, 0], 'w--', 'linewidth',2);  %分界线
plot([-10,len_line],[d,d],'w','linewidth',2);     %左边界线
plot([-10,len_line],[-d,-d],'w','linewidth',2);  %左边界线

% 设置坐标轴显示范围
axis equal
set(gca, 'XLim',[-10 len_line]);
set(gca, 'YLim',[-4 4]);

% 绘制路径
plot(path(:,1),path(:,2), 'y','linewidth',2);%路径点
scatter(P(1,:),P(2,:),100,'r.')
scatter(Q(1,:),Q(2,:),100,'b.')
plot(Q(1,:),Q(2,:),'b');%路径点
plot(P(1,:),P(2,:),'r');%路径点

