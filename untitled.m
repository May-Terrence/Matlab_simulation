close all
clear
n = 9;
k = 2;
Mat = [];
nodeVector = getNodeVector(n, k, 1);
delta = diff(nodeVector);
for i = k-2 :n+1
    for j = 0 : 1 : n
        Bik(j+1, 1) = BaseFunction(j, k-1 , i/(n+k), nodeVector);
    end
    Mat(end + 1,:) = Bik;
end
% Mat(end + 1,:) = [-1/24, -10/24, 0, 10/24, 1/24, 0, 0, 0, 0, 0]/delta(1);
% Mat(end + 1,:) = [0, 0, 0, 0, 0, -1/24, -10/24, 0, 10/24, 1/24]/delta(1);
% Mat(end + 1,:) = [1/6, 2/6, -6/6, 2/6, 1/6, 0, 0, 0, 0, 0]/delta(1)/delta(1);
% Mat(end + 1,:) = [0, 0, 0, 0, 0, 1/6, 2/6, -6/6, 2/6, 1/6]/delta(1)/delta(1);
Mat(end + 1,:) = [-1/2, 0, 1/2, 0, 0, 0, 0, 0, 0, 0]/delta(1);
Mat(end + 1,:) = [0, 0, 0, 0, 0, 0, 0, -1/2, 0, 1/2]/delta(1);
Mat(end + 1,:) = [1, -2, 1, 0, 0, 0, 0, 0, 0, 0]/delta(1)/delta(1);
Mat(end + 1,:) = [0, 0, 0, 0, 0, 0, 0, 1, -2, 1]/delta(1)/delta(1);
P = [0, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0;
     1, 2, 2, 3, 5, 8, 7, 8, 0, 0, 0, 0];                       % 5个控制点，可以满足曲率连续
% figure

Q =  pinv(Mat) * P';
P = Q';

% P = [0, 1, 2, 3, 4, 5, 5;
%     0, 1, 1, 2, 3, 4, 5];
% P = [P(:,1), P(:,1), P(:,:), P(:,end), P(:,end)];

n = size(P,2) - 1; 
path = [];
Bik = zeros(n+1, 1);

    nodeVector = getNodeVector(n, k, 1);
        path_Uniform_BSpline = [];
        for u = (k-1)/(n+k) : 0.01 : 1-(k-1)/(n+k) 
            for j = 0 : 1 : n
                Bik(j+1, 1) = BaseFunction(j, k-1 , u, nodeVector);
            end
            p_u = P * Bik;
            path_Uniform_BSpline(end+1,:) = p_u;
        end
 % figure

hold on
grid on
% 主体图形绘制
plot(P(1,:), P(2,:),'LineWidth', 3,  'Color', 'b');
% plot(path_Uniform_BSpline(:,1),path_Uniform_BSpline(:,2),'LineWidth', 1,  'Color', 'r')
plot(path_Uniform_BSpline(:,1),path_Uniform_BSpline(:,2),'LineWidth', 2,  'Color', 'k')
scatter(P(1,:), P(2,:), 40,'MarkerEdgeColor','g',...
    'MarkerFaceColor','g');
% 坐标轴
set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
hXLabel = xlabel('x');
hYLabel = ylabel('y');
% 修改刻度标签字体和字号
set(gca, 'FontSize', 16),...
set([hXLabel, hYLabel], 'FontName',  'simsun')
set([hXLabel, hYLabel], 'FontSize', 16)