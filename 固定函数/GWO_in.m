% 灰狼算法
function GWO_in(q_i,time)
% GWO_in:计算拦截舰的驱动量，采用SMPC模型
% 函数输入变量说明：
% GWO_in_para=[SearchAgents_number,Max_iter,T_SMPC,Scenario_number];
% SearchAgents_number:种群个数，Max_iter:最大迭代次数，T_SMPC:SMPC预测窗口时长，Scenario_number：场景数量
% nu_limit：无人艇驱动量限幅，q_i,q_e:当前时刻无人艇和敌舰的状态,其顺序即为拦截的匹配顺序，q_0：MPC预测区间内主舰的状态
% Scenario:对于敌方行动的预测Scenario(2,2,Enemy_number_max,T_MPC,Scenario_number,SearchAgents_number,T,Max_iteration)
% Initialize the positions of search agents  初始化搜索代理的位置



%% 声明全局变量
global q_i_gwo;
global task;
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
global nu_limit;
global alphaValue;
t_start=tic;% 开始计时
nu_i_gwo(:,:,task,:,time,1) = init_MPC; % 生成初始解维度为(3,T_MPC,Enemy_number(USV_number(task)), SearchAgents_number))


% initialize alpha, beta, and delta_pos 初始化头狼信息
Alpha_pos=zeros(3,T_MPC,Enemy_number);
Alpha_score=inf; %change this to -inf for maximization problems

Beta_pos=zeros(3,T_MPC,Enemy_number);
Beta_score=inf; %将其更改为 -INF 以解决最大化问题

Delta_pos=zeros(3,T_MPC,Enemy_number);
Delta_score=inf; %change this to -inf for maximization problems

l=1;  %循环计数器
% 主循环
while l<=Max_iteration
    for i=1:SearchAgents_number

        % Calculate objective function for each search agent  计算每个搜索代理的适应度
        q_i_iter(nu_i_gwo(:,:,task,i,time,l),q_i,time);
        q_i_gwo(:,:,task,i,time,l) = q_i_pre(:,:,task,time);
        Scenario_build(time,i,l);
        fit_SMPC(nu_i_gwo(:,:,task,i,time,l),time,i,l);
        fit = fitness(time,i,l);
        % Update Alpha, Beta, and Delta  更新头狼
        if (fit>Alpha_score) && (fit>Beta_score) && (fit<Delta_score)
            Delta_score=fit; % Update delta
            Delta_pos=nu_i_gwo(:,:,task,i,time,l);
        elseif (fit>Alpha_score) && (fit<Beta_score)
            Beta_score=fit; % Update beta
            Beta_pos=nu_i_gwo(:,:,task,i,time,l);
        elseif fit<Alpha_score
            Alpha_score=fit; % Update alpha
            Best_score(time,l) = Alpha_score;
            Alpha_pos=nu_i_gwo(:,:,task,i,time,l);
            Best_pos_all(:,task,time,l) = nu_i_gwo(:,1,task,i,time,l);
            Best_pos(:,task,time) = nu_i_gwo(:,1,task,i,time,l);
            Scenario_alpha(:,:,:,time,l) = Scenario(:,:,:,i,time,l);
            Scenario_alpha_final(:,:,:,time) = Scenario(:,:,:,i,time,l);
            J_best(:,:,:,time) = J(:,:,:,time,i,l);
            alphaValue(:,time) = [i,l];

        end
    end

    a=2-(l-1)*((2)/Max_iteration); % a decreases linearly fron 2 to 0  收敛因子线性递减

    % Update the Position of search agents including omegas
    % 更新包括ω在内的搜索代理的位置
    for i=1:SearchAgents_number  %1:15

        r1=rand(); % r1 is a random number in [0,1]
        r2=rand(); % r2 is a random number in [0,1]

        A1=2*a*r1-a; % Equation (3.3)
        C1=2*r2; % Equation (3.4)

        D_alpha=abs(C1*Alpha_pos-nu_i_gwo(:,:,task,i,time,l)); % Equation (3.5)-part 1
        X1=Alpha_pos-A1*D_alpha; % Equation (3.6)-part 1

        r1=rand();
        r2=rand();

        A2=2*a*r1-a; % Equation (3.3)
        C2=2*r2; % Equation (3.4)

        D_beta=abs(C2*Beta_pos-nu_i_gwo(:,:,task,i,time,l)); % Equation (3.5)-part 2
        X2=Beta_pos-A2*D_beta; % Equation (3.6)-part 2

        r1=rand();
        r2=rand();

        A3=2*a*r1-a; % Equation (3.3)
        C3=2*r2; % Equation (3.4)

        D_delta=abs(C3*Delta_pos-nu_i_gwo(:,:,task,i,time,l)); % Equation (3.5)-part 3
        X3=Delta_pos-A3*D_delta; % Equation (3.5)-part 3

        nu_i_gwo_temp=(X1+X2+X3)/3;
        nu_i_gwo(1,:,task,i,time,l+1)= nu_limit(1);
        nu_i_gwo(3,:,task,i,time,l+1)= nu_i_gwo_temp(3,:,:);% Equation (3.7)  round()四舍五入的取整函数
        % temp2 = nu_i_gwo(:,:,task,i,time,l+1);
    end
    l=l+1;
end
t_cal=[t_cal,toc(t_start)];
end



