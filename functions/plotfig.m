function [] = plotfig(q_0,q_e,q_i,t_start,t_end,T,CurrentFolder)

USV_number = size(q_i,3);
Enemy_number_max = size(q_e,3);


icolor_USV = zeros(USV_number,3);
for i =1:USV_number
    icolor_USV(i,:) = [0,0,1];
end
icolor_Enemy = zeros(Enemy_number_max,3);
for i =1:Enemy_number_max
    icolor_Enemy(i,:) = [0,0,0];
end

temp = q_e;
temp(isinf(temp)) = 0;
xmax = 1.2*max([max(q_0(1,:)),max(max(temp(1,:,:))),max(max(q_i(1,:,:)))]);
xmin = 1.2*min([min(q_0(1,:)),min(min(temp(1,:,:))),min(min(q_i(1,:,:)))]);
ymax = 1.2*max([max(q_0(2,:)),max(max(temp(2,:,:))),max(max(q_i(2,:,:)))]);
ymin = 1.2*min([min(q_0(2,:)),min(min(temp(2,:,:))),min(min(q_i(2,:,:)))]);
line_USV = 1;
point = 3;
point_end = 5;
line_point = 1;

fig = figure(1);
for t=1:T 
    figure(1)
    plot(q_0(1,1:t),q_0(2,1:t),'r-.','linewidth',line_USV);
    axis([xmin,xmax,ymin,ymax])
    hold on;
    plot(q_0(1,t),q_0(2,t),'rp','markersize',point,'linewidth',line_point);   
    axis([xmin,xmax,ymin,ymax])
    hold on;
    figure(1)
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
 

    for i=1:USV_number
        figure(1);
        plot(squeeze(q_i(1,1:t,i)),squeeze(q_i(2,1:t,i)),'-','color',icolor_USV(i,:),'linewidth',line_USV);
        axis([xmin,xmax,ymin,ymax])
        hold on;        
        plot(squeeze(q_i(1,t,i)),squeeze(q_i(2,t,i)),'*','color',icolor_USV(i,:),'markersize',point,'linewidth',line_point);
        axis([xmin,xmax,ymin,ymax])
        hold on;
    end

    saveas(fig,[CurrentFolder,'\',num2str(t,'%04d')],'jpeg')
    if t~=T
        hold off;  
    end
end
end

