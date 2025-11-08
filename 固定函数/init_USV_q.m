function [q_i] = init_USV_q(q0,USV_number,r_safe,r_def)
% init_USV_q:生成所有无人艇的初始时刻状态，具备按照一定规则随机生成或以最优护卫性能固定生成两种模式
% 输入变量：q0：主舰的初始位置,USV_number：无人艇数量,r_safe：无人艇与主舰的安全距离,
%           r_def：无人艇有效拦截距离，当输入值为0时，选择随机生成满足要求的无人艇位置。
% 输出变量：q_i(3,USV_number)：所有无人艇初始时刻的状态信息

% 初始化
q_i = zeros(3,USV_number);

%生成初始位置
if r_def==0 % 随机生成，规则为在据主舰安全距离与二倍安全距离间随机生成位置（暂不考虑无人艇间碰撞）
    for i=1:USV_number
        r = r_safe*rand(1)+r_safe;
        theta = 2*pi*rand(1);
        q_i(1,i) = q0(1)+r*cos(theta);
        q_i(2,i) = q0(2)+r*sin(theta); 
    end
else
    theta = 2*pi/USV_number;
    r = r_def/(sin(theta/2));
    if r < r_safe
        r = r_safe;
    end
    for i = 1:USV_number
    q_i(1,i) = q0(1)+r*cos(i*theta);
    q_i(2,i) = q0(2)+r*sin(i*theta);    
    end
end

end

