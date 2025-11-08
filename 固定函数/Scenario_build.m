function Scenario_build(time,agentlabel,iteration)
%Scenario_build 场景生成函数
% MDN-KF
global T_MPC;
global q_i_pre; % 存放无人艇i（跟随艇）的预测数据(3,T_MPC+1,USV_number,T);
global q_0_pre; % 存放主舰预测数据(3,T_MPC+1,T);
global Scenario; % 存放所有场景(2,T_MPC+1,Enemy_number_max,SearchAgents_number,T,Max_iteration);
% Scenario_alpha_final = zeros(3,T_MPC+1,Enemy_number_max,T);% 存放迭代过程中最终采用的最优解的对应场景
global Enemy_label;
global Enemy_number_max;
global Enemy_number
% global
global q_e
global Q;
global R;
global P;
global EstimateValue;
global PredictValue;
global KalmanValue;
global USV_number;
global t_start;
global Kal;
global HistoryPredictionValue;

H = eye(2);
A = eye(2);
B = eye(2);
q_i_current = squeeze(q_i_pre(1:2,1,:,time));
q_i_current = reshape(q_i_current,[2*USV_number,1]);
EstimateState = zeros(2,Enemy_number_max);
NewPredictValue = zeros(2,Enemy_number_max);



for i = 1:Enemy_number
    
    DataX = [squeeze(q_e(1:2,time,Enemy_label(i)));
        q_0_pre(1:2,1,time);
        q_i_current;
        ];
    if (time-2) >= t_start(Enemy_label(i)) % 有前两个时刻信息，使用Kalman预测
        if iteration == 1 && agentlabel ==1
            DataX01 = [squeeze(q_e(1:2,time-1,Enemy_label(i)));
                q_0_pre(1:2,1,time-1);
                reshape(squeeze(q_i_pre(1:2,1,:,time-1)),[2*USV_number,1]);
                ];
            DataX02 = [squeeze(q_e(1:2,time-2,Enemy_label(i)));
                q_0_pre(1:2,1,time-2);
                reshape(squeeze(q_i_pre(1:2,1,:,time-2)),[2*USV_number,1]);
                ];
            % MDN初步预测
            tempx =  py.mdn_matlab.predict_value(DataX01,'xe');% 获取单个返回值
            tempy =  py.mdn_matlab.predict_value(DataX01,'ye');% 获取单个返回值
            HistoryPredictionValue(1,2) = double(tempx);% 前1个时刻的直接预测
            HistoryPredictionValue(2,2) = double(tempy);
            tempx =  py.mdn_matlab.predict_value(DataX02,'xe');% 获取单个返回值
            tempy =  py.mdn_matlab.predict_value(DataX02,'ye');% 获取单个返回值
            HistoryPredictionValue(1,1) = double(tempx);% 前2个时刻的直接预测
            HistoryPredictionValue(2,1) = double(tempy);
            lambda = 0.95; % 遗忘因子
            residual = HistoryPredictionValue(:,2) - HistoryPredictionValue(:,1) - q_e(1:2,time,Enemy_label(i)) +  q_e(1:2,time-1,Enemy_label(i));
            K_t(:,:,Enemy_label(i)) = R(:,:,Enemy_label(i)) * H' / (H * R(:,:,Enemy_label(i)) * H + lambda);
            errorlearn(:,:,Enemy_label(i)) = lambda * (residual * residual');
            R(:,:,Enemy_label(i)) = (eye(2) - K_t(:,:,Enemy_label(i)) * H) + errorlearn(:,:,Enemy_label(i));
            % 利用敌方角度变化率描述敌方动态不确定性，角度变化越大，不确定性越大，后续可能酌情加上系数
            Q(:,:,Enemy_label(i)) = eye(2)*(abs(atan2(q_e(2,time-1,Enemy_label(i))-q_e(2,time-2,Enemy_label(i)),q_e(1,time-1,Enemy_label(i))-q_e(1,time-2,Enemy_label(i)))-...
                atan2(q_e(2,time,Enemy_label(i))-q_e(2,time-1,Enemy_label(i)),q_e(1,time,Enemy_label(i))-q_e(1,time-1,Enemy_label(i)))));
            EstimateP = A * P(:,:,Enemy_label(i)) * A' + Q(:,:,Enemy_label(i));
            Kal(:,:,Enemy_label(i)) = EstimateP * H'/(H * EstimateP * H' + R(:,:,Enemy_label(i)));
            P(:,:,Enemy_label(i)) = (eye(size(P(:,:,Enemy_label(i)))) - Kal(:,:,Enemy_label(i)) * H) * EstimateP;
        end
        
        
        CurrentState = q_e(1:2,time,Enemy_label(i));
        CurrentU = q_e(1:2,time,Enemy_label(i))-q_e(1:2,time-1,Enemy_label(i)); % 利用上一时刻的速度作为估计控制输入
        HistoryU = q_e(1:2,time-1,Enemy_label(i))-q_e(1:2,time-2,Enemy_label(i)); % 利用上一时刻的速度作为估计控制输入

        w = q_e(1:2,time,Enemy_label(i)) - ((q_e(1:2,time-1,Enemy_label(i))-q_e(1:2,time-2,Enemy_label(i))) ...
            + q_e(1:2,time-1,Enemy_label(i))); % 利用上一时刻的估计误差作为当前时刻的噪声。
        % 利用敌方角度变化率描述敌方动态不确定性，角度变化越大，不确定性越大，后续可能酌情加上系数
        Q(:,:,Enemy_label(i)) = eye(2)*(abs(atan2(q_e(2,time-1,Enemy_label(i))-q_e(2,time-2,Enemy_label(i)),q_e(1,time-1,Enemy_label(i))-q_e(1,time-2,Enemy_label(i)))-...
            atan2(q_e(2,time,Enemy_label(i))-q_e(2,time-1,Enemy_label(i)),q_e(1,time,Enemy_label(i))-q_e(1,time-1,Enemy_label(i)))));
        
        %         prediction(:,1) = squeeze(Scenario_alpha_final(1:2,2,Enemy_label(i),time-1));
        
        for t = 1:T_MPC
            % MDN初步预测
            tempx =  py.mdn_matlab.predict_value(DataX,'xe');% 获取单个返回值
            tempy =  py.mdn_matlab.predict_value(DataX,'ye');% 获取单个返回值
            mdnprediction(1,t) = double(tempx);
            mdnprediction(2,t) = double(tempy);
            
            % Kalman修正
            EstimateState(:,Enemy_label(i)) = A*CurrentState(:) + B*CurrentU(:) + Q(:,:,Enemy_label(i))*w; % 计算当前状态估计，更改为按运动学更新
            if t == 1
                PredictValueTemp(:,Enemy_label(i)) = CurrentState(:) + mdnprediction(:,t) - HistoryPredictionValue(:,2);  
            else
                PredictValueTemp(:,Enemy_label(i)) = CurrentState(:) + mdnprediction(:,t) - mdnprediction(:,t-1);
            end
            % EstimateP = A * P(:,:,Enemy_label(i)) * A' + Q(:,:,Enemy_label(i));
            % K(:,:,Enemy_label(i)) = EstimateP * H'/(H * EstimateP * H' + R(:,:,Enemy_label(i)));
            % % if t == 1
            % NewPredictValue(:,Enemy_label(i)) = EstimateState(:,t) + K(:,:,Enemy_label(i)) * (PredictValueTemp(:,Enemy_label(i)) - H * EstimateState(:,Enemy_label(i)));
            % else
            NewPredictValue(:,Enemy_label(i)) = EstimateState(:,Enemy_label(i)) + Kal(:,:,Enemy_label(i)) * (PredictValueTemp(:,Enemy_label(i)) - H * EstimateState(:,Enemy_label(i)));
            % end
            % NewPredictValue(:,Enemy_label(i)) = EstimateState(:,t) + K(:,:,Enemy_label(i)) * (prediction(:,Enemy_label(i))-prediction(:,t-1)+CurrentState(:) - H * EstimateState(:,Enemy_label(i)));
            % P(:,:,Enemy_label(i)) = (eye(size(P(:,:,Enemy_label(i)))) - K(:,:,Enemy_label(i)) * H) * EstimateP;
            
            % save all value
            EstimateValue(:,t,Enemy_label(i),time,agentlabel,iteration) = EstimateState(:,Enemy_label(i));
            PredictValue(:,t,Enemy_label(i),time,agentlabel,iteration) = PredictValueTemp(:,Enemy_label(i));
            KalmanValue(:,t,Enemy_label(i),time,agentlabel,iteration) = NewPredictValue(:,Enemy_label(i));
            if t == 1 
            Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) = EstimateValue(:,t,Enemy_label(i),time,agentlabel,iteration);
            else 
            Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) = KalmanValue(:,t,Enemy_label(i),time,agentlabel,iteration);    
            end
            
            % currentu = delta——Estimation，newcurrent = estimation
            CurrentState(:) = squeeze(Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration));
%             CurrentU(:) = Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) - Scenario(1:2,t,Enemy_label(i),agentlabel,time,iteration);
            CurrentU(:) = CurrentU(:) + CurrentU(:) - HistoryU; % mpc越往后，currentU的估计越不精准
            q_i_current = squeeze(q_i_pre(1:2,t+1,:,time));
            q_i_current = reshape(q_i_current,[2*USV_number,1]);
            DataX = [squeeze(NewPredictValue(:,Enemy_label(i)));
                q_0_pre(1:2,t+1,time);
                q_i_current;
                ];
            % currentu = delta——current + delta——prediction，newcurrent = newprediction
            
            % currentu = delta——Estimation，newcurrent = newprediction
            
            % currentu = delta——prediction，newcurrent = estimation
            
            
            
%             % Kalman修正
%             EstimateState(:,t) = A*CurrentState(:) + B*CurrentU(:) + Q(:,:,Enemy_label(i))*w; % 计算当前状态估计，更改为按运动学更新
%             PredictValue(:,t) = CurrentState(:) + prediction(:,t) - prediction(:,t-1);
%             EstimateP = A * P(:,:,Enemy_label(i)) * A' + Q(:,:,Enemy_label(i));
%             K(:,:,Enemy_label(i)) = EstimateP * H'/(H * EstimateP * H' + R(:,:,t));
%             NewPredictValue(:,t) = EstimateState(:,t) + K(:,:,t) * (prediction(:,t)-prediction(:,t-1)+CurrentState(:) - H * EstimateState(:,t));
%             P(:,:,t+1) = (eye(size(P(:,:,t))) - K(:,:,t) * H) * EstimateP;
%             
%             %% 可能更新方式需要调整
%             CurrentState(:) = NewPredictValue(:,t);
%             CurrentU(:) =
            
            
%             Scenario(:,t,Enemy_label(i),m,agentlabel,time,iteration) = Ptemp;
            
            
            
            
        end
    elseif (time-1)==t_start(Enemy_label(i)) % 在敌舰刚开始任务的阶段，仅使用当前信息预测,待改进
        
        for t = 1:T_MPC
            % MDN初步预测
            tempx =  py.mdn_matlab.predict_value(DataX,'xe');% 获取单个返回值
            tempy =  py.mdn_matlab.predict_value(DataX,'ye');% 获取单个返回值
            Ptemp(1,1) = double(tempx);
            Ptemp(2,1) = double(tempy);
            % save all value
%             EstimateValue(:,t,Enemy_label(i),time,agentlabel,iteration) = EstimateState(:,Enemy_label(i));
%             PredictValue(:,t,Enemy_label(i),time,agentlabel,iteration) = PredictValue(:,Enemy_label(i));
%             KalmanValue(:,t,Enemy_label(i),time,agentlabel,iteration) = NewPredictValue(:,Enemy_label(i));
            Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) = Ptemp;
            
            % currentu = delta——Estimation，newcurrent = estimation
            CurrentState(:) = Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration);
            CurrentU(:) = Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) - Scenario(1:2,t,Enemy_label(i),agentlabel,time,iteration);
            q_i_current = squeeze(q_i_pre(1:2,t,:,time));
            q_i_current = reshape(q_i_current,[2*USV_number,1]);
            DataX = [squeeze(Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration));
                q_0_pre(1:2,t,time);
                q_i_current;
                ];
        end
    elseif time==(t_start(Enemy_label(i))) % 在敌舰刚开始任务的阶段，仅使用当前信息估计，此外，保留预测结果，待改进！！！！
        for t = 1:T_MPC
            % MDN初步预测
            tempx =  py.mdn_matlab.predict_value(DataX,'xe');% 获取单个返回值
            tempy =  py.mdn_matlab.predict_value(DataX,'ye');% 获取单个返回值
            Ptemp(1,1) = double(tempx);
            Ptemp(2,1) = double(tempy);
            % save all value
%             EstimateValue(:,t,Enemy_label(i),time,agentlabel,iteration) = EstimateState(:,Enemy_label(i));
%             PredictValue(:,t,Enemy_label(i),time,agentlabel,iteration) = PredictValue(:,Enemy_label(i));
%             KalmanValue(:,t,Enemy_label(i),time,agentlabel,iteration) = NewPredictValue(:,Enemy_label(i));
            Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) = Ptemp;
            
            % currentu = delta——Estimation，newcurrent = estimation
            CurrentState(:) = Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration);
            CurrentU(:) = Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) - Scenario(1:2,t,Enemy_label(i),agentlabel,time,iteration);
            q_i_current = squeeze(q_i_pre(1:2,t,:,time));
            q_i_current = reshape(q_i_current,[2*USV_number,1]);
            DataX = [squeeze(Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration));
                q_0_pre(1:2,t,time);
                q_i_current;
                ];
        end
    end
end

end