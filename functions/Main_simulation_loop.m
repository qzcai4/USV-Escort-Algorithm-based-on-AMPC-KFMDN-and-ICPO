function Main_simulation_loop()
    global Enemy_number_max t_start T T_MPC delta_t delta_t_es h USV_number t_end;
    global Enemy_number Enemy_label SearchAgents_number Max_iteration;
    global nu_limit r_def r_safe d_def r_form dyna task notask;
    global q_i_gwo nu_i_gwo fitness Best_score J J_best Scenario;
    global Scenario_alpha Scenario_alpha_final t_cal Best_pos q_i_pre;
    global q_0_pre nu_i_pre1 Q R P Kal EstimateValue PredictValue;
    global KalmanValue EstimateValueAlpha PredictValueAlpha KalmanValueAlpha;
    global alphaValue HistoryPredictionValue q_e q_i nu_i USV_status;
    global task_flag D_ei D_ei1 task_check task_alltime Event_trigger;
    global q_0 nu_0 t_end algonumber dataStruct;

    for i = 1:Enemy_number_max
        task_flag(i, t_start(i):t_end(i)) = 1;
    end

    for t = 1:T
        if any(t_start == t)
                en_start = (1:Enemy_number_max);
                en_start = en_start((t_start == t));
                q_e(:, t, en_start) = init_USV_qe(q_0(:, t), d_def, size(en_start, 2));
        end

        if t > 1 && sum(task_flag(:, t-1)) ~= 0
            for i = 1:USV_number
                for j = 1:Enemy_number
                    D_ei(i, Enemy_label(j), t) = norm(squeeze(q_i(1:2, t, i)) - squeeze(q_e(1:2, t, Enemy_label(j))));
                end
            end

            Cond1 = any(D_ei(:, Enemy_label, t) <= r_def, 'all');
            Cond2 = any(t_end == t, 1);
            if Cond1 || Cond2
                [~, temp] = find(D_ei(:, Enemy_label, t) <= r_def);
                temp = unique(temp);
                task_flag(Enemy_label(temp), t:T) = 0;
                t_end(Enemy_label(temp)) = t;
                q_e(1:2, t+1:T, Enemy_label(temp)) = inf(2, T-t, length(temp));
            end
        end

        change1 = (t == 1 && sum(task_flag(:, t)) ~= 0);
        change2 = (t > 1 && sum(task_flag(:, t) - task_flag(:, t-1)) ~= 0);
        Event_trigger(t) = change1 || change2;

        if Event_trigger(t)
            Enemy_label = (1:Enemy_number_max);
            Enemy_label = Enemy_label(task_flag(:, t) == 1);
            Enemy_number = numel(Enemy_label);

            E_task = zeros(USV_number, Enemy_number);
            for i = 1:USV_number
                for j = 1:Enemy_number
                    E_task(i, j) = norm(squeeze(q_i(1:2, t, i)) - squeeze(q_e(1:2, t, Enemy_label(j)))) / nu_limit(1);
                end
            end

            Prio = zeros(Enemy_number, 1);
            for i = 1:Enemy_number
                Prio(i) = 1 / norm(squeeze(q_0(1:2, t)) - squeeze(q_e(1:2, t, Enemy_label(i))));
            end

            [~, label1] = sort(E_task);
            [~, label2] = sort(Prio);
            task = zeros(Enemy_number, 1);
            for i = 1:Enemy_number
                j = 1;
                while j <= size(E_task, 1)
                    if any(task == label1(j, label2(i)))
                        j = j + 1;
                    else
                        task(label2(i)) = label1(j, label2(i));
                        break;
                    end
                end
            end
            notask = setdiff(1:USV_number, task);
        end

        if any(task_flag(:, t) == 1)
            nu_e(:, t, Enemy_label) = APF(q_e(:, t, Enemy_label), q_0(:, t), q_i(:, t, :));
            q_e(:, t+1, Enemy_label) = q_iter_in(squeeze(q_e(:, t, Enemy_label)), squeeze(nu_e(:, t, Enemy_label)));
        end

        q_0(:, t+1) = q_iter_0(q_0(:, t), nu_0(:, t));
        q_0_pre(:, 1, t) = q_0(:, t);
        if t < T
            q_0_pre(:, 2, t) = q_0(:, t+1);
        else
            q_0_pre(:, 2, t) = q_0(:, t);
        end

        if Enemy_number <= USV_number
            K = delta_t / delta_t_es;
            q_0_temp = zeros(3, K);
            nu_i_temp = zeros(3, K, size(notask, 2));
            q_i_temp = zeros(3, K, size(notask, 2));
            
            for k = 1:K
                if k == 1
                    q_0_temp(:, k) = q_0(:, t) + delta_t_es * JETA(q_0(3, t)) * nu_0(:, t);
                    nu_i(:, t, notask) = leader_follower(q_0(:, t), nu_0(:, t), squeeze(q_i(:, t, notask)));
                    q_i_temp(:, k, :) = q_iter(squeeze(q_i(:, t, notask)), squeeze(nu_i(:, t, notask)));
                elseif k == K
                    nu_i_temp(:, k-1, :) = leader_follower(q_0_temp(:, k-1), nu_0(:, t), squeeze(q_i_temp(:, k-1, :)));
                    q_i(:, t+1, notask) = q_iter(squeeze(q_i_temp(:, k-1, :)), squeeze(nu_i_temp(:, k-1, :)));
                else
                    q_0_temp(:, k) = q_0_temp(:, k-1) + delta_t_es * JETA(q_0_temp(3, k-1)) * nu_0(:, t);
                    nu_i_temp(:, k-1, :) = leader_follower(q_0_temp(:, k-1), nu_0(:, t), squeeze(q_i_temp(:, k-1, :)));
                    q_i_temp(:, k, :) = q_iter(squeeze(q_i_temp(:, k-1, :)), squeeze(nu_i_temp(:, k-1, :)));
                end
            end
            
            [temp, form] = leader_follower_q(q_0(:, t), nu_0(:, t), squeeze(q_i(:, t, notask)));
            q_i(:, t+1, notask(form)) = temp;
        end

        q_i_pre(:, 1, notask, t) = q_i(:, t, notask);
        q_i_pre(:, 2, notask, t) = q_i(:, t+1, notask);

        if Enemy_number > 0
            if Event_trigger(t) || (mod(t, delta_t) == 0)
                for k = 2:(min(t+T_MPC, T)-t)
                    q_0_pre(:, k+1, t) = q_iter_0(q_0_pre(:, k, t), nu_0(:, t));
                end
                
                if (t+T_MPC) > T
                    q_0_pre(:, (T-t+2):(T_MPC+1), t) = repmat(q_0_pre(:, T-t+1, t), [1, t+T_MPC-T]);
                end

                Pridict_es(nu_0(:, t), t);
                Scenario(:, 1, Enemy_label, :, t, :) = repmat(q_e(:, t, Enemy_label), [1, 1, 1, SearchAgents_number, Max_iteration]);
                Scenario(:, 2, Enemy_label, :, t, :) = repmat(q_e(:, t+1, Enemy_label), [1, 1, 1, SearchAgents_number, Max_iteration]);
                SI_solver(q_i(:, t, :), t);
                nu_i(:, t, task) = Best_pos(:, task, t);
                q_i(:, t+1, task) = q_iter_in(squeeze(q_i(:, t, task)), squeeze(nu_i(:, t, task)));
            else
                nu_i(:, t, task) = nu_i(:, t-1, task);
                q_i(:, t+1, task) = q_iter_in(squeeze(q_i(:, t, task)), squeeze(nu_i(:, t, task)));
            end
            
            task_check(1:size(task, 1), 1, t) = task;
            task_check(1:size(Enemy_label, 2), 2, t) = Enemy_label';
        end

        all_task = ones(USV_number, 1);
        all_task(notask) = 0;
        task_alltime(:, t) = all_task;

       
        disp(['Time step: ', num2str(t), '/', num2str(T)]);
    end
end