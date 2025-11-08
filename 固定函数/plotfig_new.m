function [] = plotfig_new(q_0,q_e,q_i,t_start,t_end,T,CurrentFolder,t_end_new)
% 用于绘制给定时间T之前的无人艇安全护航演示曲线
% 输入变量：
% T:绘制时长;
% q_0(2,T):主舰状态; q_e(2,T,Enemy_number_max):敌舰状态; q_i(3,T,USV_number):第i个无人艇状态
% t_start,t_end：各敌舰的工作开始和结束时间。
% CurrentFolder:存放保存图片的路径

USV_number = size(q_i,3);
Enemy_number_max = size(q_e,3);

% 生成存放轨迹颜色的数组
% 设置各无人艇的颜色
icolor_USV = zeros(USV_number,3);
for i =1:USV_number
%    icolor_USV(i,:) = [i/USV_number,(USV_number-i)/USV_number,1];
    icolor_USV(i,:) = [0,0,1];
end
% 设置各敌舰的颜色
icolor_Enemy = zeros(Enemy_number_max,3);
for i =1:Enemy_number_max
%    icolor_Enemy(i,:) = [(Enemy_number_max-i)/Enemy_number_max,1,i/Enemy_number_max];
    icolor_Enemy(i,:) = [0,0,0];
end

% 计算图窗尺寸和点线尺寸
temp = q_e;
temp(isinf(temp)) = 0;
xmax = 1.2*max([max(q_0(1,:)),max(max(temp(1,:,:))),max(max(q_i(1,:,:)))]);
xmin = 1.2*min([min(q_0(1,:)),min(min(temp(1,:,:))),min(min(q_i(1,:,:)))]);
ymax = 1.2*max([max(q_0(2,:)),max(max(temp(2,:,:))),max(max(q_i(2,:,:)))]);
ymin = 1.2*min([min(q_0(2,:)),min(min(temp(2,:,:))),min(min(q_i(2,:,:)))]);
line_USV = 1;%船舶运动轨迹线宽
point = 3;%船舶当前位置标记点大小
point_end = 5;
line_point = 1;
%line_limit = 1;%约束范围线宽

% 循环开始

for k=1:length(t_end_new)
    fig = figure(k);
    t=t_end_new(k);
% 绘制主舰轨迹和位置
    figure(k)
            grid on; % 关键步骤：启用网格
    plot(q_0(1,1:t),q_0(2,1:t),'r-.','linewidth',line_USV);
    axis([xmin,xmax,ymin,ymax])
    hold on;
    plot(q_0(1,t),q_0(2,t),'rp','markersize',point,'linewidth',line_point);   
    axis([xmin,xmax,ymin,ymax])
    hold on;
% 绘制敌舰轨迹和位置
    figure(k)
    for i=1:Enemy_number_max
        if t>=t_start(i,1) && t<=t_end(i,1)
            plot(squeeze(q_e(1,t_start(i,1):t,i)),squeeze(q_e(2,t_start(i,1):t,i)),'-','color',icolor_Enemy(i,:),'linewidth',line_USV);
            axis([xmin,xmax,ymin,ymax])
            axis off;
            hold on;
            plot(squeeze(q_e(1,t,i)),squeeze(q_e(2,t,i)),'x','color',icolor_Enemy(i,:),'markersize',point,'linewidth',line_point);   
            axis([xmin,xmax,ymin,ymax])
            hold on; 
        elseif t>t_end(i,1)
            plot(squeeze(q_e(1,t_start(i,1):t_end(i,1),i)),squeeze(q_e(2,t_start(i,1):t_end(i,1),i)),'k--','linewidth',line_USV);
            axis([xmin,xmax,ymin,ymax])
            axis off;
            hold on;
            plot(squeeze(q_e(1,t_end(i,1),i)),squeeze(q_e(2,t_end(i,1),i)),'kx','markersize',point_end,'linewidth',line_point);   
            axis([xmin,xmax,ymin,ymax])
            hold on; 
        end
    end
 

% 绘制无人艇轨迹和位置
    for i=1:USV_number
        figure(k);
        plot(squeeze(q_i(1,1:t,i)),squeeze(q_i(2,1:t,i)),'-','color',icolor_USV(i,:),'linewidth',line_USV);
        axis([xmin,xmax,ymin,ymax])
        hold on;        
        plot(squeeze(q_i(1,t,i)),squeeze(q_i(2,t,i)),'*','color',icolor_USV(i,:),'markersize',point,'linewidth',line_point);
        axis([xmin,xmax,ymin,ymax])
        hold on;
    end


% 调整图形属性，确保网格完整显示
set(gcf, 'InvertHardcopy', 'off'); % 防止导出时重置背景色
set(gca, 'Position', [0 0 1 1]);    % 坐标轴充满整个figure窗口，避免空白边框
% % 标注主舰和无人艇约束
%     if flag==1     % 判断是否需要标注
%         % 主舰防碰撞约束
%         theta = linspace(0,2*pi);
%         x1 = r(3)*cos(theta)+eta0(1,t);
%         y1 = r(3)*sin(theta)+eta0(2,t);
%         plot(x1,y1,'r:','linewidth',line_limit);
%         hold on;
%         % 主舰通信约束
%         theta = linspace(0,2*pi);
%         x2 = r(4)*cos(theta)+eta0(1,t);
%         y2 = r(4)*sin(theta)+eta0(2,t);
%         plot(x2,y2,'r:','linewidth',line_limit);
%         hold on;    
%     end
% 保存当前图窗中的图片
    exportgraphics(fig, [CurrentFolder,'\',num2str(t,'%04d'),'.eps'], 'ContentType','vector', 'Resolution',600);
    % saveas(fig,[CurrentFolder,'\',num2str(t,'%04d')],'jpeg')
% 清除当前图窗中所有图像 
    if t~=T
        hold off;  
    end
end
end

