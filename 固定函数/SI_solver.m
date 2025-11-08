function SI_solver(q_i,time)
% ICPO2: Crested Porcupine Optimization 算法


%% 声明全局变量
global q_i_gwo;
global task;
global nu_limit; % 生成初始解需服从的约束
global nu_i_gwo;% 存放所有可行解(3,T_MPC,USV_number,Scenario_number, SearchAgents_number,T,Max_iteration)
global fitness;
global Scenario; %  存放所有场景(2,2,T_MPC+1,Enemy_number_max,Scenario_number,SearchAgents_number,T,Max_iteration);
global Scenario_alpha; % 存放最优解对应场景(2,2,T_MPC+1,Enemy_number_max,Scenario_number,T,Max_iteration);
global t_cal;
global Best_score;
global Best_pos;
global Best_pos_all;
global Enemy_number; % 当前敌舰数量,在现有1对1匹配的机制下，与拦截的无人艇数量等同，即USV_number = Enemy_number。
global SearchAgents_number; % 灰狼算法种群数量
global Max_iteration; % 灰狼算法最大迭代次数。
global T_MPC; % MPC的区间段数，实际MPC的区间时长为：区间段数与单次决策时间间隔的乘积，即T_MPC*delta_t。
global q_i_pre;
global Scenario_alpha_final;
global J;
global J_best;
global alphaValue;
t_start=tic;% 开始计时

% 初始化搜索边界
Lowerbound = nu_limit(:,2);
Upperbound = nu_limit(:,1);

% 初始化控制参数
N_min = 0.5*SearchAgents_number; % 种群规模最小值
T = 5; % 循环数量
repeat = fix(0.3*T);
alpha = 0.2; % 收敛率
Tf = 0.8; % 第三和第四防御机制之间的权衡百分比
Search_Agents = SearchAgents_number;

% 初始化种群和适应度
nu_i_gwo(:,:,task,:,time,1) = init_MPC; % 生成初始解维度为(3,T_MPC,Enemy_number(USV_number(task)), SearchAgents_number))
CPOfitness = zeros(1, Search_Agents);



% 计算初始适应度
for i = 1:Search_Agents
    q_i_iter(nu_i_gwo(:,:,task,i,time,1),q_i,time);
    q_i_gwo(:,:,task,i,time,1) = q_i_pre(:,:,task,time);
    Scenario_build(time,i,1);
    fit_SMPC(nu_i_gwo(:,:,task,i,time,1),time,i,1);
    CPOfitness(i) = fitness(time,i,1);
end

% 初始化全局最优
[Score, index] = min(CPOfitness);
Best_pos(:,task,time) = nu_i_gwo(:,1,task,index,time,1);
CPOBest_pos = nu_i_gwo(:,:,task,index,time,1);
% 存储每个峰冠豪猪个人最佳位置
Xp = nu_i_gwo(:,:,task,:,time,1);
dimensions = size(squeeze(nu_i_gwo(:,:,task,1,1,1)));
% 主循环
k = 1;
Best_score(time,k) = Score;
while k < Max_iteration
    r2 = rand;
    for i = 1:Search_Agents
        U1 = rand(dimensions) > rand;
        
        if rand < rand % 探索阶段
            if rand < rand % 第一防御机制
                rand_index = randi(Search_Agents);
                y = (nu_i_gwo(:,:,task,i,time,k) + nu_i_gwo(:,:,task,rand_index,time,k)) / 2;
                nu_i_gwo(:,:,task,i,time,k+1) = nu_i_gwo(:,:,task,i,time,k) + (randn) .* abs(2*rand*CPOBest_pos - y);
            else % 第二防御机制
                rand_index1 = randi(Search_Agents);
                rand_index2 = randi(Search_Agents);
                y = (nu_i_gwo(:,:,task,i,time,k) + nu_i_gwo(:,:,task,rand_index1,time,k)) / 2;
                nu_i_gwo(:,:,task,i,time,k+1) = (U1) .* nu_i_gwo(:,:,task,i,time,k) + (1-U1) .* (y + rand*(nu_i_gwo(:,:,task,rand_index1,time,k) - nu_i_gwo(:,:,task,rand_index2,time,k)));
            end
        else
            Yt = 2*rand*(1-k/Max_iteration)^(k/Max_iteration);
            U2 = rand(dimensions) < 0.5*2-1;
            S = rand*U2;
            if rand < Tf % 第三防御机制
                St = exp(CPOfitness(i)/(sum(CPOfitness)+eps));
                S = S.*Yt.*St;
                rand_index1 = randi(Search_Agents);
                rand_index2 = randi(Search_Agents);
                rand_index3 = randi(Search_Agents);
                nu_i_gwo(:,:,task,i,time,k+1) = (1-U1).*nu_i_gwo(:,:,task,i,time,k) + U1.*(nu_i_gwo(:,:,task,rand_index1,time,k) + St*(nu_i_gwo(:,:,task,rand_index2,time,k) - nu_i_gwo(:,:,task,rand_index3,time,k)) - S);
            else % 第四防御机制
                Mt = exp(CPOfitness(i)/(sum(CPOfitness)+eps));
                vt = nu_i_gwo(:,:,task,i,time,k);
                rand_index = randi(Search_Agents);
                Vtp = nu_i_gwo(:,:,task,rand_index,time,k);
                Ft = rand(dimensions).*(Mt*(-vt+Vtp));
                S = S.*Yt.*Ft;
                nu_i_gwo(:,:,task,i,time,k+1) = (CPOBest_pos + (alpha*(1-r2)+r2)*(U2.*CPOBest_pos-nu_i_gwo(:,:,task,i,time,k))) - S;
            end
        end
% 边界检查
        nu_i_gwo(:,:,task,i,time,k+1) = max(nu_i_gwo(:,:,task,i,time,k+1), Lowerbound);
        nu_i_gwo(:,:,task,i,time,k+1) = min(nu_i_gwo(:,:,task,i,time,k+1), Upperbound);
        
        % 计算新适应度
        q_i_iter(nu_i_gwo(:,:,task,i,time,k+1),q_i,time);
        q_i_gwo(:,:,task,i,time,k+1) = q_i_pre(:,:,task,time);
        Scenario_build(time,i,k+1);
        fit_SMPC(nu_i_gwo(:,:,task,i,time,k+1),time,i,k+1);
        new_fitness = fitness(time,i,k+1);
        
        % 更新个体最优和全局最优
        if new_fitness < CPOfitness(i)
            Xp(:,:,:,i) = nu_i_gwo(:,:,task,i,time,k+1);
            CPOfitness(i) = new_fitness;
            if new_fitness < Score
                CPOBest_pos = nu_i_gwo(:,:,task,i,time,k+1);
                Score = new_fitness;
            end
        else
            nu_i_gwo(:,:,task,i,time,k+1) = Xp(:,:,:,i);
        end
    end

    if (k)<(repeat*Max_iteration/T)
        for i = Search_Agents+1:SearchAgents_number
            if rand <0.8 % 以0.2的概率随机重启
                if rand < rand % 第一防御机制
                    rand_index = randi(Search_Agents);
                    y = (nu_i_gwo(:,:,task,i,time,k) + nu_i_gwo(:,:,task,rand_index,time,k)) / 2;
                    nu_i_gwo(:,:,task,i,time,k+1) = nu_i_gwo(:,:,task,i,time,k) + (randn) .* abs(2*rand*CPOBest_pos - y);
                else % 第二防御机制
                    rand_index1 = randi(Search_Agents);
                    rand_index2 = randi(Search_Agents);
                    y = (nu_i_gwo(:,:,task,i,time,k) + nu_i_gwo(:,:,task,rand_index1,time,k)) / 2;
                    nu_i_gwo(:,:,task,i,time,k+1) = (U1) .* nu_i_gwo(:,:,task,i,time,k) + (1-U1) .* (y + rand*(nu_i_gwo(:,:,task,rand_index1,time,k) - nu_i_gwo(:,:,task,rand_index2,time,k)));
                end
            else % 随机重启
                temp = init_MPC;
                nu_i_gwo(:,:,task,i,time,k+1) = temp(:,:,:,1);
            end
            % 边界检查
        nu_i_gwo(:,:,task,i,time,k+1) = max(nu_i_gwo(:,:,task,i,time,k+1), Lowerbound);
        nu_i_gwo(:,:,task,i,time,k+1) = min(nu_i_gwo(:,:,task,i,time,k+1), Upperbound);
        
        % 计算新适应度
        q_i_iter(nu_i_gwo(:,:,task,i,time,k+1),q_i,time);
        q_i_gwo(:,:,task,i,time,k+1) = q_i_pre(:,:,task,time);
        Scenario_build(time,i,k+1);
        fit_SMPC(nu_i_gwo(:,:,task,i,time,k+1),time,i,k+1);
        new_fitness = fitness(time,i,k+1);
        
        % 更新个体最优和全局最优
        if new_fitness < CPOfitness(i)
            Xp(:,:,:,i) = nu_i_gwo(:,:,task,i,time,k+1);
            CPOfitness(i) = new_fitness;
            if new_fitness < Score
                CPOBest_pos = nu_i_gwo(:,:,task,i,time,k+1);
                Score = new_fitness;
            end
        else
            nu_i_gwo(:,:,task,i,time,k+1) = Xp(:,:,:,i);
        end
        end 
    end
        
    
    k = k + 1
    Best_score(time,k) = Score;
    Best_pos(:,task,time) = CPOBest_pos(:,1,:);

    % 动态调整种群规模
    New_Search_Agents = fix(N_min + (Search_Agents - N_min) * (1 - (rem(k, Max_iteration/T) / (Max_iteration/T))));
    
    if New_Search_Agents < Search_Agents
        [~, sorted_indexes] = sort(CPOfitness);
        nu_i_gwo(:,:,task,:,time,k) = nu_i_gwo(:,:,task,sorted_indexes,time,k);
        Xp = Xp(:,:,:,sorted_indexes);
        CPOfitness = CPOfitness(sorted_indexes);
        Search_Agents = New_Search_Agents;
    end
end
t_cal=[t_cal,toc(t_start)];
end


