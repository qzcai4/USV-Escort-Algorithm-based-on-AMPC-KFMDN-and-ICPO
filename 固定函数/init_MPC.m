function Initial_solution=init_MPC
% ICPO1_1
% init_SMPC:生成SMPC模型的初始解，对于单个无人艇i其在t时刻各场景驱动量相同，其余时刻不同场景均可以不同
% 输入变量：T_MPC：MPC时间长度，USV_number：无人艇数量,nu_limit：无人艇速度限幅
%           Scenario_number：SMPC模型中的场景数量
% 返回值Initial_solution(3,Enemy_number,T_MPC,SearchAgents_number)
    global T_MPC; % MPC的时长
    global Enemy_number;
    global nu_limit; % 生成初始解需服从的约束
    global SearchAgents_number;  % 优化算法种群数量
    
    ub = nu_limit(:,1);
    lb = nu_limit(:,2);
    
    Initial_solution_temp =  rand(3,T_MPC,Enemy_number,SearchAgents_number);
% 控制参数
    mu = 3.9;  % Logistic映射的控制参数,3.5<mu<=4
    for j = 1:10
        % 迭代Sine映射以生成遍历性
        for i = 1:20  % 迭代次数可以根据需要调整
            Initial_solution_temp = sin(pi * Initial_solution_temp);
        end
        % 对Sine映射的结果应用Logistic映射
        Initial_solution_temp = mu * Initial_solution_temp .* (1 - Initial_solution_temp);
    end
    Initial_solution = Initial_solution_temp.* (ub - lb) + lb;
    

end


