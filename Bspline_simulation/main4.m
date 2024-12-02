% ֱ�����ÿ��Ƶ�滮B�������߻���·��
clc
clear
close all

%% ���ݶ���
d = 3.5;
k = 4;                                    % k�ס�k-1��B����
classFlag = 2;                                  %1,2�ֱ���ƾ���B�������ߡ�׼����B��������
P = [0,-d/2; 10,-d/2; 20,-d/2; 30,d/2; 40,d/2; 50,d/2]';
n = size(P,2)-1;                          % n�ǿ��Ƶ��������0��ʼ����

%% ����B��������
Bik = zeros(n+1, 1);
nodeVector = getNodeVector(n, k, classFlag);
path = [];
for u = 0 : 0.005 : 1-0.005
    for i = 0 : 1 : n
        Bik(i+1, 1) = BaseFunction(i, k-1 , u, nodeVector);
    end
    p_u = P * Bik;
    path(end+1,:) = p_u;
end


%% ���㳤�Ⱥ�����
x = path(:,1)';
y = path(:,2)';
diffX = diff(path(:,1));
diffY = diff(path(:,2));
cumLength = cumsum(sqrt(diffX.^2 + diffY.^2));   %����
heading = atan2(diffY, diffX);
for i = 1:length(x)-2
    cur(i) = getCur(x(i:i+2)',y(i:i+2)');
end
cur(end+1) = cur(end);



%% ������ͼ
figure
hold on
grid on
% ����ͼ�λ���
plot(cumLength,cur,'LineWidth', 3,  'Color', 'k');
% ������
set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
hXLabel = xlabel('·������/m');
hYLabel = ylabel('����/m^-^1');
% �޸Ŀ̶ȱ�ǩ������ֺ�
set(gca, 'FontSize', 16),...
    set([hXLabel, hYLabel], 'FontName',  'simsun')
set([hXLabel, hYLabel], 'FontSize', 16)


% �������ͼ
figure
hold on
grid on
% ����ͼ�λ���
plot(cumLength, heading,'LineWidth', 3,  'Color', 'b');
% ������
set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
hXLabel = xlabel('·������/m');
hYLabel = ylabel('�����/rad');
% �޸Ŀ̶ȱ�ǩ������ֺ�
set(gca, 'FontSize', 16),...
set([hXLabel, hYLabel], 'FontName',  'simsun')
set([hXLabel, hYLabel], 'FontSize', 16)
%% ������ͼ
d = 3.5;               % ��·��׼���
W = 1.8;               % �������
L = 4.7;               % ����
figure
len_line = 50;

% ����ɫ·��ͼ
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([P(1,1),P(1,1),P(1,1)-L,P(1,1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'y')
fill([35,35,35-L,35-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')

% ���ֽ���
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %�ֽ���
plot([-5,len_line],[d,d],'w','linewidth',2);     %��߽���
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %��߽���

% ������������ʾ��Χ
axis equal
set(gca, 'XLim',[-5 len_line]);
set(gca, 'YLim',[-4 4]);

% ����·��
scatter(P(1,:),P(2,:),100,'r.')
plot(P(1,:),P(2,:),'r');%·����
plot(path(:,1),path(:,2), 'y','linewidth',2);%·����

