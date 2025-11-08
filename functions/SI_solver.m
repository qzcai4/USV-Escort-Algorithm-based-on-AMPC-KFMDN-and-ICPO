function SI_solver(q_i,time)

global q_i_gwo;
global task;
global nu_limit; 
global nu_i_gwo;
global fitness;
global Scenario; 
global Scenario_alpha; 
global t_cal;
global Best_score;
global Best_pos;
global Best_pos_all;
global Enemy_number; 
global SearchAgents_number; 
global Max_iteration; 
global T_MPC; 
global q_i_pre;
global Scenario_alpha_final;
global J;
global J_best;
global alphaValue;
t_start=tic;

Lowerbound = nu_limit(:,2);
Upperbound = nu_limit(:,1);

N_min = 0.5*SearchAgents_number; 
T = 5; 
repeat = fix(0.3*T);
alpha = 0.2; 
Tf = 0.8; 
Search_Agents = SearchAgents_number;

nu_i_gwo(:,:,task,:,time,1) = init_MPC; 
CPOfitness = zeros(1, Search_Agents);

for i = 1:Search_Agents
    q_i_iter(nu_i_gwo(:,:,task,i,time,1),q_i,time);
    q_i_gwo(:,:,task,i,time,1) = q_i_pre(:,:,task,time);
    Scenario_build(time,i,1);
    fit_SMPC(nu_i_gwo(:,:,task,i,time,1),time,i,1);
    CPOfitness(i) = fitness(time,i,1);
end

[Score, index] = min(CPOfitness);
Best_pos(:,task,time) = nu_i_gwo(:,1,task,index,time,1);
CPOBest_pos = nu_i_gwo(:,:,task,index,time,1);
Xp = nu_i_gwo(:,:,task,:,time,1);
dimensions = size(squeeze(nu_i_gwo(:,:,task,1,1,1)));

k = 1;
Best_score(time,k) = Score;
while k < Max_iteration
    r2 = rand;
    for i = 1:Search_Agents
        U1 = rand(dimensions) > rand;
        
        if rand < rand 
            if rand < rand 
                rand_index = randi(Search_Agents);
                y = (nu_i_gwo(:,:,task,i,time,k) + nu_i_gwo(:,:,task,rand_index,time,k)) / 2;
                nu_i_gwo(:,:,task,i,time,k+1) = nu_i_gwo(:,:,task,i,time,k) + (randn) .* abs(2*rand*CPOBest_pos - y);
            else 
                rand_index1 = randi(Search_Agents);
                rand_index2 = randi(Search_Agents);
                y = (nu_i_gwo(:,:,task,i,time,k) + nu_i_gwo(:,:,task,rand_index1,time,k)) / 2;
                nu_i_gwo(:,:,task,i,time,k+1) = (U1) .* nu_i_gwo(:,:,task,i,time,k) + (1-U1) .* (y + rand*(nu_i_gwo(:,:,task,rand_index1,time,k) - nu_i_gwo(:,:,task,rand_index2,time,k)));
            end
        else
            Yt = 2*rand*(1-k/Max_iteration)^(k/Max_iteration);
            U2 = rand(dimensions) < 0.5 * 2-1;
            S = rand*U2;
            if rand < Tf 
                St = exp(CPOfitness(i)/(sum(CPOfitness)+eps));
                S = S.*Yt.*St;
                rand_index1 = randi(Search_Agents);
                rand_index2 = randi(Search_Agents);
                rand_index3 = randi(Search_Agents);
                nu_i_gwo(:,:,task,i,time,k+1) = (1-U1).*nu_i_gwo(:,:,task,i,time,k) + U1.*(nu_i_gwo(:,:,task,rand_index1,time,k) + St*(nu_i_gwo(:,:,task,rand_index2,time,k) - nu_i_gwo(:,:,task,rand_index3,time,k)) - S);
            else 
                Mt = exp(CPOfitness(i)/(sum(CPOfitness)+eps));
                vt = nu_i_gwo(:,:,task,i,time,k);
                rand_index = randi(Search_Agents);
                Vtp = nu_i_gwo(:,:,task,rand_index,time,k);
                Ft = rand(dimensions).*(Mt*(-vt+Vtp));
                S = S.*Yt.*Ft;
                nu_i_gwo(:,:,task,i,time,k+1) = (CPOBest_pos + (alpha*(1-r2)+r2)*(U2.*CPOBest_pos-nu_i_gwo(:,:,task,i,time,k))) - S;
            end
        end
        nu_i_gwo(:,:,task,i,time,k+1) = max(nu_i_gwo(:,:,task,i,time,k+1), Lowerbound);
        nu_i_gwo(:,:,task,i,time,k+1) = min(nu_i_gwo(:,:,task,i,time,k+1), Upperbound);
        
        q_i_iter(nu_i_gwo(:,:,task,i,time,k+1),q_i,time);
        q_i_gwo(:,:,task,i,time,k+1) = q_i_pre(:,:,task,time);
        Scenario_build(time,i,k+1);
        fit_SMPC(nu_i_gwo(:,:,task,i,time,k+1),time,i,k+1);
        new_fitness = fitness(time,i,k+1);
        
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
            if rand <0.8 
                if rand < rand 
                    rand_index = randi(Search_Agents);
                    y = (nu_i_gwo(:,:,task,i,time,k) + nu_i_gwo(:,:,task,rand_index,time,k)) / 2;
                    nu_i_gwo(:,:,task,i,time,k+1) = nu_i_gwo(:,:,task,i,time,k) + (randn) .* abs(2*rand*CPOBest_pos - y);
                else 
                    rand_index1 = randi(Search_Agents);
                    rand_index2 = randi(Search_Agents);
                    y = (nu_i_gwo(:,:,task,i,time,k) + nu_i_gwo(:,:,task,rand_index1,time,k)) / 2;
                    nu_i_gwo(:,:,task,i,time,k+1) = (U1) .* nu_i_gwo(:,:,task,i,time,k) + (1-U1) .* (y + rand*(nu_i_gwo(:,:,task,rand_index1,time,k) - nu_i_gwo(:,:,task,rand_index2,time,k)));
                end
            else 
                temp = init_MPC;
                nu_i_gwo(:,:,task,i,time,k+1) = temp(:,:,:,1);
            end
        nu_i_gwo(:,:,task,i,time,k+1) = max(nu_i_gwo(:,:,task,i,time,k+1), Lowerbound);
        nu_i_gwo(:,:,task,i,time,k+1) = min(nu_i_gwo(:,:,task,i,time,k+1), Upperbound);
        
        q_i_iter(nu_i_gwo(:,:,task,i,time,k+1),q_i,time);
        q_i_gwo(:,:,task,i,time,k+1) = q_i_pre(:,:,task,time);
        Scenario_build(time,i,k+1);
        fit_SMPC(nu_i_gwo(:,:,task,i,time,k+1),time,i,k+1);
        new_fitness = fitness(time,i,k+1);
        
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
        
    Best_score(time,k) = Score;
    Best_pos(:,task,time) = CPOBest_pos(:,1,:);

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