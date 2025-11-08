%% 清屏并开始计时
clc,clear;

%% 配置环境信息
rng('default'); % 初始化随机数生成器设置
start=tic; % 开始计时
CurrentFolder = pwd; % 获取当前工作文件夹路径，便于存放生成图像
AllFolders = dir(CurrentFolder);
isub = [AllFolders(:).isdir];
FoldersList = {AllFolders(isub).name}';
FoldersList(ismember(FoldersList,{'.','..'})) = [];
FoldersList(ismember(FoldersList,{'固定函数','model','__pycache__'})) = [];
% 验证并配置python环境
pyexe = pyenv();
if strcmp(pyexe.Executable,'D:\Anaconda3\envs\mdn_matlab\python.exe')~=1
    pyenv('Version', 'D:\Anaconda3\envs\mdn_matlab\python.exe');
    % pyversion D:\Anaconda3\envs\an_quan_hu_hang\python.exe
end
pyexe.Executable
% setenv('KMP_DUPLICATE_LIB_OK', 'TRUE')
try
    py.importlib.import_module('mdn_matlab');
    has_mdn_matlab = true;
catch
    has_mdn_matlab = false;
end
if has_mdn_matlab
    py_dict = py.sys.modules;
    py_dict.pop('mdn_matlab');
    disp("'mdn_matlab'库已移除");
else
    disp("'mdn_matlab'库不存在");
end
% 导入基础函数
addpath(fullfile(CurrentFolder, '固定函数'));

%% 配置对比实验相关参数
global Enemy_number_max;Enemy_number_max = 3; % 定义仿真过程最大敌舰数量

dataStruct = struct(); % 创建空结构体数组

Algo_number = 2;

%% 开始仿真

    global t_start;t_start = randi([3,40],Enemy_number_max,1); % 定义仿真过程敌舰出现初始时间



        %% 配置全局变量
        % 实验参数配置
        global T;T = 100; % 仿真总时长，单位：s
        global T_MPC;T_MPC = 5; % MPC的区间段数，实际MPC的区间时长为：区间段数与单次决策时间间隔的乘积，即T_MPC*delta_t
        global delta_t;delta_t = 1; % 拦截舰每次更新驱动量的时间间隔/单次系统迭代的时间;MPC单次决策的时间间隔，单位：s。
        global delta_t_es;delta_t_es=0.01; % 护卫舰每次编队更新驱动量的时间间隔/单次系统迭代的时间
        global h;h=0.002; % 每次状态更新的步长，驱动量更新之间，以h步长进行状态更新，降低模型非线性影响
        global USV_number;USV_number = 6; % 定义初始护卫舰数量
        global Enemy_number;Enemy_number = 0; % 定义当前敌舰数量
        global Enemy_label;
        global SearchAgents_number;SearchAgents_number=15; % 灰狼算法种群数量
        global Max_iteration;Max_iteration=20; % 灰狼算法最大迭代次数
        global nu_limit;nu_limit = [30 0 25*pi/180;...
            0 0 -25*pi/180]'; % 给定无人艇驱动向量限幅，最大速度25m/s，最小速度0m/s,最大转角25°,最小转角-25°,目前全体我方采用同构
        global r_def;r_def = 15; % 成功拦截距离15m
        global r_safe;r_safe = 20; % 与主舰的碰撞安全距离20m
        global d_def;d_def = 1000; % 任务分配触发距离为敌舰与我方主舰距离小于1000m,主舰可探测到敌舰的范围
        global r_form;r_form = 40; % 可参与编队的无人艇距主舰距离阈值
        global dyna;dyna = [1000,0.6,0.59]; % 无人艇动力学参数，参考刘彬博士论文
        global task;task = zeros(Enemy_number,1); % 给定任务标志初值
        global notask;notask = setdiff(1:USV_number,task); % 给定跟随舰标志初值

        % 保存迭代过程量
        global q_i_gwo;q_i_gwo = zeros(3,T_MPC+1,USV_number,SearchAgents_number,T,Max_iteration);
        global nu_i_gwo;nu_i_gwo = zeros(3,T_MPC,USV_number,SearchAgents_number,T,Max_iteration); % 存放所有可行解
        global fitness;fitness = zeros(T,SearchAgents_number,Max_iteration); % 存放迭代过程中所有适应度值
        global Best_score; Best_score = zeros(T,Max_iteration); % 存放GWO的每次迭代的最优适应度的个体的适应度
        % global Kall;Kall = zeros(3,T,SearchAgents_number,Max_iteration);
        % global K1all;K1all = zeros(Enemy_number_max,T,SearchAgents_number,Max_iteration);
        % global Kall_best;Kall_best = zeros(3,T);
        % global K1all_best;K1all_best = zeros(Enemy_number_max,T);
        global J;J = zeros(Enemy_number_max,T_MPC,5,T,SearchAgents_number,Max_iteration);
        global J_best;J_best = zeros(Enemy_number_max,T_MPC,5,T);
        global Scenario;% 存放所有场景
        Scenario = zeros(3,T_MPC+1,Enemy_number_max,SearchAgents_number,T,Max_iteration);
        global Scenario_alpha;% 存放迭代过程中所有最优解对应场景
        Scenario_alpha = zeros(3,T_MPC+1,Enemy_number_max,T,Max_iteration);
        global Scenario_alpha_final;% 存放迭代过程中所有最优解对应场景
        Scenario_alpha_final = zeros(3,T_MPC+1,Enemy_number_max,T);% 存放迭代过程中最终采用的最优解的对应场景
        global t_cal;t_cal = []; % 存储单次迭代的实际时间
        global Best_pos;Best_pos = zeros(3,USV_number,T); % 存放GWO迭代最终得到的最优适应度的个体对应的决策量
        % global Best_pos_all;Best_pos_all = zeros(3,USV_number,T,Max_iteration); % 存放GWO的每次迭代的最优适应度的个体对应的决策量
        global q_i_pre;q_i_pre = zeros(3,T_MPC+1,USV_number,T); % 存放无人艇i（跟随艇）的预测数据(3,T_MPC+1,USV_number)
        global q_0_pre;q_0_pre = zeros(3,T_MPC+1,T); % 存放主舰预测数据(3,T_MPC+1)
        K = delta_t/delta_t_es;
        % global q_i_pre_temp1; q_i_pre_temp1 = zeros(3,K,USV_number,T);
        % global nu_i_temp1;nu_i_temp1 = zeros(3,K,USV_number,T_MPC,T);
        global nu_i_pre1;nu_i_pre1 = zeros(3,T_MPC,USV_number,T);
        %% Kalman Filter
        global Q;Q = repmat(eye(2),[1,1,Enemy_number_max]);
        global R;R = repmat(eye(2),[1,1,Enemy_number_max]);
        global P;P = repmat(eye(2),[1,1,Enemy_number_max]);
        global Kal;Kal = repmat(eye(2),[1,1,Enemy_number_max]);
        global EstimateValue;EstimateValue = zeros(2,T_MPC,Enemy_number_max,T,SearchAgents_number,Max_iteration);
        global PredictValue;PredictValue = zeros(2,T_MPC,Enemy_number_max,T,SearchAgents_number,Max_iteration);
        global KalmanValue;KalmanValue = zeros(2,T_MPC,Enemy_number_max,T,SearchAgents_number,Max_iteration);
        global EstimateValueAlpha;EstimateValueAlpha = zeros(2,T_MPC,Enemy_number_max,T);
        global PredictValueAlpha;PredictValueAlpha = zeros(2,T_MPC,Enemy_number_max,T);
        global KalmanValueAlpha;KalmanValueAlpha = zeros(2,T_MPC,Enemy_number_max,T);
        global alphaValue;alphaValue = zeros(2,T);
        global HistoryPredictionValue;HistoryPredictionValue = zeros(2,2);



        %% 初始化变量信息
        q_0 = zeros(3,T); % 主舰的状态信息
        nu_0 = zeros(3,T); % 主舰的驱动信息
        global q_e;q_e = inf(3,T,Enemy_number_max); % 敌舰的实际状态信息
        nu_e = zeros(3,T,Enemy_number_max); % 敌舰的驱动信息
        q_i = zeros(3,T,USV_number); % 无人艇的状态信息
        nu_i = zeros(3,T,USV_number); % 无人艇的驱动信息
        USV_status = zeros(USV_number,1); % 无人艇的任务状态标志位存储，0则为护卫主舰，1~Enemy_number则为拦截对应编号的敌舰。
        task_flag = zeros(Enemy_number_max,T); % 事件触发标志位，受敌我距离影响,0为无需拦截或拦截成功，1为需要拦截。
        D_ei = zeros(USV_number,Enemy_number_max,T); % 存放所有敌舰和我方无人艇的距离

        D_ei1 = zeros(USV_number,Enemy_number_max,T); % 存放所有敌舰和我方无人艇的距离
        task_check = zeros(Enemy_number_max,2,T);

        task_alltime = zeros(USV_number,T);
        Event_trigger = zeros(T,1); % 存放事件触发标志位，为1的时刻发生任务分配

        %% 生成主舰运动轨迹，并生成无人艇的初始状态
        % 生成主舰运动方式，后续随跟随舰更新状态
        q_0(3,:) = deg2rad(45); % 全程以与正东方向逆时针45°的角度前进
        nu_0(1,:) = 10; % 全程以10m/s的速度，朝向船体正前方前进

        % 生成无人艇初始状态
        q_i(:,1,:) = init_USV_q(q_0(:,1),USV_number,r_safe,r_def);

        % 在时刻1~40s间，以整秒形式为每艘敌舰生成出现时刻并修改标志位
        t_end = T*ones(Enemy_number_max,1); % 存放敌舰结束运动时间
        for i = 1:Enemy_number_max
            task_flag(i, t_start(i):t_end(i)) = 1; % 存放敌方存在的标志，从出现到结束的时刻为1
        end


        %% 开始循环
        for t=1:T

            % 随机生成多个敌舰的初始位置和运动轨迹（随机），过程中可能也会生成，if语句，在特定时刻生成。
            %% 生成敌舰初始状态和任务标志位
            if any(t_start==t)
                if algonumber == 1
                    en_start = (1:Enemy_number_max);
                    en_start = en_start((t_start==t));
                    q_e(:,t,en_start) = init_USV_qe(q_0(:,t),d_def,size(en_start,2));
                else
                    en_start = (1:Enemy_number_max);
                    en_start = en_start((t_start==t));
                    q_e(:,t,en_start) = dataStruct(id).q_e(:,t,en_start);
                end
            end

            % 计算敌舰与我方无人艇距离
            if t>1 && sum(task_flag(:,t-1))~=0
                for i=1:USV_number
                    for j=1:Enemy_number
                        D_ei(i,Enemy_label(j),t) = (norm(squeeze(q_i(1:2,t,i))-squeeze(q_e(1:2,t,Enemy_label(j))))); % 求解各无人艇与敌舰距离,aij为第i艘无人艇和第j艘敌舰的距离
                    end
                end

                % 判断敌舰与我方无人艇是否有距离小于拦截距离的，若有则判断为拦截成功，并修改标志位和结束时间以及敌舰后续状态。
                Cond1 = any(D_ei(:,Enemy_label,t)<=r_def,'all');
                Cond2 = any(t_end==t, 1);
                if Cond1||Cond2
                    [~,temp] = find(D_ei(:,Enemy_label,t)<=r_def);
                    temp = unique(temp); % 得到距离矩阵中出现小于防御距离的列，其对应列编号即为已被防守成功的敌舰编号
                    task_flag(Enemy_label(temp),t:T)=0;
                    t_end(Enemy_label(temp))=t;
                    q_e(1:2,t+1:T,Enemy_label(temp))=inf(2,T-t,length(temp));
                end
            end
            % 根据任务状态标志位变化，判断是否需要进行任务再分配（事件触发）
            change1 = (t==1 && sum(task_flag(:,t))~=0);
            change2 = (t>1 && sum(task_flag(:,t)-task_flag(:,t-1))~=0);
            Event_trigger(t) = change1||change2;
            if Event_trigger(t)
                Enemy_label = (1:Enemy_number_max);
                Enemy_label = Enemy_label(task_flag(:,t)==1); % 获取当前需防御的敌舰编号
                Enemy_number = numel(Enemy_label); % 获取当前需防御的敌舰总数
                % 初始化任务分配效率矩阵E并求解
                E_task = zeros(USV_number,Enemy_number);
                for i=1:USV_number
                    for j=1:Enemy_number
                        E_task(i,j) = norm(squeeze(q_i(1:2,t,i))-squeeze(q_e(1:2,t,Enemy_label(j))))/nu_limit(1);
                    end
                end
                % 求解敌舰拦截优先级
                Prio = zeros(Enemy_number,1);
                for i=1:Enemy_number
                    Prio(i) = 1/(norm(squeeze(q_0(1:2,t))-squeeze(q_e(1:2,t,Enemy_label(i)))));
                end
                % 任务分配
                [~,label1] = sort(E_task);
                [~,label2] = sort(Prio);
                task = zeros(Enemy_number,1);% 对当前需防卫的敌舰的任务匹配，编号为Task（1）的无人艇需拦截编号为Enemy_label（1）的敌舰
                for i=1:Enemy_number
                    j=1;
                    while j<=size(E_task,1)
                        if any(task == label1(j,label2(i)))
                            j=j+1;
                        else
                            task(label2(i))=label1(j,label2(i));
                            break;
                        end
                    end
                end
                % 获取执行护航任务的无人艇的编号
                notask = setdiff(1:USV_number,task);
            end

            % 敌舰状态更新
            if any(task_flag(:,t)==1)
                nu_e(:,t,Enemy_label) = APF(q_e(:,t,Enemy_label),q_0(:,t),q_i(:,t,:));
                q_e(:,t+1,Enemy_label) = q_iter_in(squeeze(q_e(:,t,Enemy_label)),squeeze(nu_e(:,t,Enemy_label)));
            end

            % 主舰状态更新
            q_0(:,t+1) = q_iter_0(q_0(:,t),nu_0(:,t));
            % 将更新后的主舰状态保存到预测值中，便于场景生成调用
            q_0_pre(:,1,t) = q_0(:,t); % 保存主舰当前时刻状态
            if t<T
                q_0_pre(:,2,t) = q_0(:,t+1); % 保存主舰下一时刻状态
            else
                q_0_pre(:,2,t) = q_0(:,t); % 保存主舰当前时刻状态
            end
            % 求解无人艇驱动量
            % 护卫主舰部分无人艇
            if Enemy_number<=USV_number
                K = delta_t/delta_t_es;
                %nu_0_temp = zeros(3,K);
                q_0_temp = zeros(3,K);
                nu_i_temp = zeros(3,K,size(notask,2));
                q_i_temp = zeros(3,K,size(notask,2));
                for k=1:K
                    if k ==1
                        q_0_temp(:,k) = q_0(:,t)+delta_t_es*JETA(q_0(3,t))*nu_0(:,t);
                        nu_i(:,t,notask) = leader_follower(q_0(:,t),nu_0(:,t),squeeze(q_i(:,t,notask)));
                        q_i_temp(:,k,:) = q_iter(squeeze(q_i(:,t,notask)),squeeze(nu_i(:,t,notask)));
                    elseif k==K
                        nu_i_temp(:,k-1,:) = leader_follower(q_0_temp(:,k-1),nu_0(:,t),squeeze(q_i_temp(:,k-1,:)));
                        q_i(:,t+1,notask) = q_iter(squeeze(q_i_temp(:,k-1,:)),squeeze(nu_i_temp(:,k-1,:)));
                    else
                        q_0_temp(:,k) = q_0_temp(:,k-1)+delta_t_es*JETA(q_0_temp(3,k-1))*nu_0(:,t);
                        nu_i_temp(:,k-1,:) = leader_follower(q_0_temp(:,k-1),nu_0(:,t),squeeze(q_i_temp(:,k-1,:)));
                        q_i_temp(:,k,:) = q_iter(squeeze(q_i_temp(:,k-1,:)),squeeze(nu_i_temp(:,k-1,:)));
                    end
                end
                 [temp,form]= leader_follower_q(q_0(:,t),nu_0(:,t),squeeze(q_i(:,t,notask)));
                 q_i(:,t+1,notask(form)) = temp;
            end
            % 将更新后的护卫舰状态保存到预测值中，便于场景生成调用
            q_i_pre(:,1,notask,t) = q_i(:,t,notask); % 保存护卫舰当前时刻状态
            q_i_pre(:,2,notask,t) = q_i(:,t+1,notask); % 保存护卫舰下一时刻状态
            % 拦截敌舰部分无人艇
            if Enemy_number>0
                if Event_trigger(t)||(mod(t,delta_t)==0) % 如果达到了单次决策时间间隔或者触发了任务分配，重新计算驱动量，否则保持之前的驱动量
                    % 预测主舰行动(若主舰运动模式变化，需调整nu_0(:,t))
                    for k = 2:(min(t+T_MPC,T)-t)
                        q_0_pre(:,k+1,t) =  q_iter_0(q_0_pre(:,k,t),nu_0(:,t));
                    end
                    if (t+T_MPC)>T
                        q_0_pre(:,(T-t+2):(T_MPC+1),t) =   repmat(q_0_pre(:,T-t+1,t),[1,t+T_MPC-T]);
                    end
                    % 预测跟随舰行动(后续敌我数量相同时加入预测判断)
                    Pridict_es(nu_0(:,t),t);
                    % 将更新前的当前敌舰状态信息复制到所有灰狼和迭代场景
                    Scenario(:,1,Enemy_label,:,t,:) = repmat(q_e(:,t,Enemy_label),[1,1,1,SearchAgents_number,Max_iteration]);
                    Scenario(:,2,Enemy_label,:,t,:) = repmat(q_e(:,t+1,Enemy_label),[1,1,1,SearchAgents_number,Max_iteration]);
                    % 求解拦截舰驱动量
                    SI_solver(q_i(:,t,:),t);
                    nu_i(:,t,task)=Best_pos(:,task,t);
                    q_i(:,t+1,task) = q_iter_in(squeeze(q_i(:,t,task)),squeeze(nu_i(:,t,task)));
                else
                    nu_i(:,t,task) = nu_i(:,t-1,task);
                    q_i(:,t+1,task) = q_iter_in(squeeze(q_i(:,t,task)),squeeze(nu_i(:,t,task)));
                end
                task_check(1:size(task,1),1,t) = task;
                task_check(1:size(Enemy_label,2),2,t) = Enemy_label';
            end
            all_task = ones(USV_number,1);
            all_task(notask) = 0;
            task_alltime(:,t) = all_task;
            disp(t);

        end
    
%% 画图

plotfig(q_0,q_e,q_i,t_start,t_end,T,CurrentFolder);
im2gif(CurrentFolder); % 生成动图

