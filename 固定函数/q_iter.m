function [q_new] = q_iter(q,nu)
% q_iter函数：用于更新无人艇状态
% 输入量：q:当前时刻的状态量，nu：当前时刻给定的驱动量，delta_t:每次更新驱动量的时间间隔/单次系统迭代的时间，h：每次状态更新的步长，delta_t应是h的整数倍，即：delta_t=k*h。
% 过程量：q_temp:当前时刻到下一时刻的迭代过程中的状态量，随迭代次数变动并趋近于最终量。
% 输出量：q_new:下一时刻的状态量
% 检查delta_t是否为h的整数倍
global delta_t_es;
global h;
delta_t = delta_t_es;
if mod(delta_t,h)~=0
    disp('error:delta_t/h~=0');
    return
end
%开始求解
    q_temp=zeros(size(q));
    for k=1:(delta_t/h)
        if (k==1)&&((delta_t/h)~=1)
            for i=1:size(nu,2) 
                q_temp(:,i)=q(:,i)+h*JETA(q(3,i))*nu(:,i) ;       
            end
        elseif k==(delta_t/h)
            for i=1:size(nu,2) 
                q_new(:,i)=q_temp(:,i)+h*JETA(q_temp(3,i))*nu(:,i) ;       
            end
        elseif (k==1)&&(k==(delta_t/h))
            for i=1:size(nu,2) 
                q_new(:,i)=q(:,i)+h*JETA(q(3,i))*nu(:,i) ;      
            end    
        else
            for i=1:size(nu,2) 
                q_temp(:,i)=q_temp(:,i)+h*JETA(q_temp(3,i))*nu(:,i) ;       
            end
        end
    end
end

