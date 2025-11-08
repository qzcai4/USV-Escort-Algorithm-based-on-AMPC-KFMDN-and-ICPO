function [nu_e] = APF(q_e,q_0,q_i)
% APF:利用人工势场法生成敌舰的驱动量
% 输入变量：q_e(3,t,Enemy_number):t时刻所有敌舰当前位置。q_0(2,t):t时刻主舰当前位置。
%          q_i(2,t,Enemy_number):t时刻所有我方无人艇当前位置。
% 输出变量：nu_e(2,Enemy_number)：所有敌舰当前时刻至下一时刻驱动量。

%% 输入变量处理
    % 变量降维，删去时间维度
    q_e = squeeze(q_e);
    q_i = squeeze(q_i(1:2,:,:));
    q_0 = squeeze(q_0(1:2,:));    
    % 提取信息
    USV_number = size(q_i,2);
    Enemy_number = size(q_e,2);

    nu_e = zeros(3,Enemy_number);
%% APF参数初始化
    
    att = 0.05*ones(Enemy_number,1);%引力增益系数,当敌舰离主舰近的时候，会在计算引力时增强系数
    req = -100;%斥力增益系数
    e_def = 350;%障碍物产生影响的最大距离，当障碍与移动目标之间距离大于e_def时，斥力为0。
    %n = length(obstacle(:,1));%障碍物个数
    v = 25;
    omega = 25*pi/180;

%% 引力计算
    V_att = zeros(2,Enemy_number);
    r_att = zeros(Enemy_number,1);
    P_att = zeros(2,Enemy_number);
    for i=1:Enemy_number
        V_att(:,i) = q_0 - squeeze(q_e(1:2,i));%敌舰到主舰的向量
        r_att(i) = norm(V_att(:,i));%路径点到目标点的欧氏距离
        if r_att(i) < 100 && r_att(i)>50
            att(i) = 0.10;
        elseif r_att(i) < 50
            att(i) = 0.2;
        end
        P_att(:,i) = att(i) * V_att(:,i);%引力
    end
    
%% 斥力计算
    %改进的人工势场法，将斥力分散一部分到引力方向。通过添加随机扰动r_att^n实现，r_att为路径点到目标点的欧氏距离，本文n取2。
    V_req = zeros(2,Enemy_number,USV_number);
    r_req = zeros(Enemy_number,USV_number);
    for i = 1:Enemy_number
        for j = 1:USV_number
            V_req(:,i,j) = q_i(:,j) - q_e(1:2,i);%路径点到各个障碍物的向量
            r_req(i,j) = norm(V_req(:,i,j));%路径点到各个障碍物的欧氏距离
        end 
    end
    P_req = zeros(2,Enemy_number,USV_number);
    
    for i = 1:Enemy_number
        for j = 1:USV_number
            if r_req(i,j) <= e_def
                P_req1 = req * (1 / r_req(i,j) - 1 / e_def) * r_att(i)^2 / r_req(i,j)^2;%斥力分量1：障碍物指向路径点的斥力
                P_req2 = req * (1 / r_req(i,j) - 1 / e_def)^2 * r_att(i);%斥力分量2：路径点指向目标点的分引力
                P_req(:,i,j) = P_req1 / r_req(i,j) * V_req(:,i,j) + P_req2 / r_att(i) * V_att(:,i);%合力分散到x,y方向得到无人艇j对敌舰i的斥力
            end     
        end
    end
    %% 合力计算(确定方向，并加入不确定性)
    P = P_att + squeeze(sum(P_req,3));
    theta = atan2(P(2,:),P(1,:));
    delta = q_e(3,:)-theta; % 当前角度与期望转动角度的误差，作为均值
    %% 实际速度计算
    for i = 1:Enemy_number
        if r_att(i)<v
            nu_e(1,i) = r_att(i);
        else
            nu_e(1,i) = v;
        end
            nu_e(2,i) = 0;
    end
    delta(delta>omega) = omega;
    delta(delta<-omega) = -omega;
    nu_e(3,:) = - delta;
end