function [fitnessValue,all_J] = fit_SMPC(nu,time,agentlabel,iteration)
% 变权重MPC计算适应度
global Enemy_number;
global Scenario;% 存放所有场景(2,2,T_MPC+1,Enemy_number_max,Scenario_number,SearchAgents_number,T,Max_iteration);
global T_MPC;
global fitness;
global J;
global task;
global Enemy_label;
global q_i_pre;% (3,T_MPC+1,USV_number,Scenario_number,T)
global nu_limit;
global r_def;


q_e_Scenario = zeros(2,T_MPC+1,Enemy_number);

if Enemy_number == 1
    q_e_Scenario(:,:,1) = squeeze(Scenario(1:2,:,Enemy_label,agentlabel,time,iteration));
else
    q_e_Scenario(:,:,:) = squeeze(Scenario(1:2,:,Enemy_label,agentlabel,time,iteration));
end


% J1
Distance_temp = squeeze(q_i_pre(1:2,1,task,time))-squeeze(q_e_Scenario(:,1,:));
CurrentDistance = sqrt(sum(Distance_temp.^2));
K1 = 100;
K2 = 100;
K3 = 1000;
K4 = 10e5;
delta_xy = q_i_pre(1:2,1,task,time)-q_e_Scenario(:,1,:);
gamma = squeeze(1./sqrt(sum(delta_xy.^2, 1)));
k_values = 1:T_MPC;
T_values = T_MPC - k_values + 1;
alpha = gamma * T_values;

J1 = inf(Enemy_number,T_MPC);
flag = zeros(Enemy_number,T_MPC);
target_angel = inf(Enemy_number,T_MPC);
J2 = inf(Enemy_number,T_MPC);
C1 = zeros(Enemy_number,T_MPC);
C2 = zeros(Enemy_number,T_MPC);
C3 = zeros(Enemy_number,T_MPC);


%% 这里可以向量化处理！！！！！！
T_MPC_adapt = (T_MPC+1)*ones(Enemy_number,1);


for i=1:Enemy_number
    if CurrentDistance(i) <= 5*r_def
        T_MPC_adapt(i) = 2;
    elseif CurrentDistance(i) <= 5*r_def && CurrentDistance(i) > 3*r_def
        T_MPC_adapt(i) = min([3,T_MPC_adapt(i)]);
    elseif CurrentDistance(i) <= 7*r_def && CurrentDistance(i) > 5*r_def
        T_MPC_adapt(i) = min([4,T_MPC_adapt(i)]);
    end
    for t=2:T_MPC_adapt(i)
        J1(i,t-1) = norm((squeeze(q_i_pre(1:2,t,task(i),time))-squeeze(q_e_Scenario(:,t,i))));
        target_angel(i,t-1) = atan2(q_e_Scenario(2,t,i)-q_i_pre(2,t,task(i),time),q_e_Scenario(1,t,i)-q_i_pre(1,t,task(i),time));
        J2(i,t-1) = mod(abs(target_angel(i,t-1)-q_i_pre(3,t,task(i),time)),2*pi);
        if J1(i,t-1)<r_def
            flag(i,t-1:end) = 1;
            break;
        end
        C1(i,t-1) = nu(1,t-1,i)-nu_limit(1,1);
        C2(i,t-1) = -nu(1,t-1,i)+nu_limit(1,2);
        C3(i,t-1) = abs(nu(3,t-1,i))-nu_limit(3,1);
    end
end
C1(C1<0) = 0;
C2(C2<0) = 0;
C3(C3<0) = 0;
J1(flag==1) = 0;
J2(flag==1) = 0;
J1(J1==inf) = 0;
J2(J2==inf) = 0;

J(Enemy_label,:,:,time,agentlabel,iteration) = cat(5,J1,J2,C1,C2,C3);
J1 = J1.*alpha;
J2 = J2.*alpha;

% J2 = J2*K2; % 如果有问题就将K2改成100

% prob_m(m) = prod(squeeze(probility(:,:,:,m)),'all');
sum_J1 = sum(squeeze(J1(:,:)),'all');
sum_J2 = sum(squeeze(J2(:,:)),'all');
sum_C1 = sum(squeeze(C1(:,:)),'all');
sum_C2 = sum(squeeze(C2(:,:)),'all');
sum_C3 = sum(squeeze(C3(:,:)),'all');

sum_all = (K1*sum_J1+K2*sum_J2+K3*sum_C1+K3*sum_C2+K4*sum_C3);
all_J=[K1,sum_J1,sum_J2,sum_C1,sum_C2,sum_C3,sum_all];

fitness(time,agentlabel,iteration) = sum(sum_all,'all');

end