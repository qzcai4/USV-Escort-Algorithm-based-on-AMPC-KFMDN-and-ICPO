function Scenario_build(time,agentlabel,iteration)
global T_MPC;
global q_i_pre;
global q_0_pre;
global Scenario;
global Enemy_label;
global Enemy_number_max;
global Enemy_number;
global q_e;
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
    if (time-2) >= t_start(Enemy_label(i))
        if iteration == 1 && agentlabel ==1
            DataX01 = [squeeze(q_e(1:2,time-1,Enemy_label(i)));
                q_0_pre(1:2,1,time-1);
                reshape(squeeze(q_i_pre(1:2,1,:,time-1)),[2*USV_number,1]);
                ];
            DataX02 = [squeeze(q_e(1:2,time-2,Enemy_label(i)));
                q_0_pre(1:2,1,time-2);
                reshape(squeeze(q_i_pre(1:2,1,:,time-2)),[2*USV_number,1]);
                ];
            tempx =  py.mdn_matlab.predict_value(DataX01,'xe');
            tempy =  py.mdn_matlab.predict_value(DataX01,'ye');
            HistoryPredictionValue(1,2) = double(tempx);
            HistoryPredictionValue(2,2) = double(tempy);
            tempx =  py.mdn_matlab.predict_value(DataX02,'xe');
            tempy =  py.mdn_matlab.predict_value(DataX02,'ye');
            HistoryPredictionValue(1,1) = double(tempx);
            HistoryPredictionValue(2,1) = double(tempy);
            lambda = 0.95;
            residual = HistoryPredictionValue(:,2) - HistoryPredictionValue(:,1) - q_e(1:2,time,Enemy_label(i)) +  q_e(1:2,time-1,Enemy_label(i));
            K_t(:,:,Enemy_label(i)) = R(:,:,Enemy_label(i)) * H' / (H * R(:,:,Enemy_label(i)) * H + lambda);
            errorlearn(:,:,Enemy_label(i)) = lambda * (residual * residual');
            R(:,:,Enemy_label(i)) = (eye(2) - K_t(:,:,Enemy_label(i)) * H) + errorlearn(:,:,Enemy_label(i));
            Q(:,:,Enemy_label(i)) = eye(2)*(abs(atan2(q_e(2,time-1,Enemy_label(i))-q_e(2,time-2,Enemy_label(i)),q_e(1,time-1,Enemy_label(i))-q_e(1,time-2,Enemy_label(i)))-...
                atan2(q_e(2,time,Enemy_label(i))-q_e(2,time-1,Enemy_label(i)),q_e(1,time,Enemy_label(i))-q_e(1,time-1,Enemy_label(i)))));
            EstimateP = A * P(:,:,Enemy_label(i)) * A' + Q(:,:,Enemy_label(i));
            Kal(:,:,Enemy_label(i)) = EstimateP * H'/(H * EstimateP * H' + R(:,:,Enemy_label(i)));
            P(:,:,Enemy_label(i)) = (eye(size(P(:,:,Enemy_label(i)))) - Kal(:,:,Enemy_label(i)) * H) * EstimateP;
        end
        
        
        CurrentState = q_e(1:2,time,Enemy_label(i));
        CurrentU = q_e(1:2,time,Enemy_label(i))-q_e(1:2,time-1,Enemy_label(i));
        HistoryU = q_e(1:2,time-1,Enemy_label(i))-q_e(1:2,time-2,Enemy_label(i));

        w = q_e(1:2,time,Enemy_label(i)) - ((q_e(1:2,time-1,Enemy_label(i))-q_e(1:2,time-2,Enemy_label(i))) ...
            + q_e(1:2,time-1,Enemy_label(i)));
        Q(:,:,Enemy_label(i)) = eye(2)*(abs(atan2(q_e(2,time-1,Enemy_label(i))-q_e(2,time-2,Enemy_label(i)),q_e(1,time-1,Enemy_label(i))-q_e(1,time-2,Enemy_label(i)))-...
            atan2(q_e(2,time,Enemy_label(i))-q_e(2,time-1,Enemy_label(i)),q_e(1,time,Enemy_label(i))-q_e(1,time-1,Enemy_label(i)))));
        
        for t = 1:T_MPC
            tempx =  py.mdn_matlab.predict_value(DataX,'xe');
            tempy =  py.mdn_matlab.predict_value(DataX,'ye');
            mdnprediction(1,t) = double(tempx);
            mdnprediction(2,t) = double(tempy);
            
            EstimateState(:,Enemy_label(i)) = A*CurrentState(:) + B*CurrentU(:) + Q(:,:,Enemy_label(i))*w;
            if t == 1
                PredictValueTemp(:,Enemy_label(i)) = CurrentState(:) + mdnprediction(:,t) - HistoryPredictionValue(:,2);  
            else
                PredictValueTemp(:,Enemy_label(i)) = CurrentState(:) + mdnprediction(:,t) - mdnprediction(:,t-1);
            end
            NewPredictValue(:,Enemy_label(i)) = EstimateState(:,Enemy_label(i)) + Kal(:,:,Enemy_label(i)) * (PredictValueTemp(:,Enemy_label(i)) - H * EstimateState(:,Enemy_label(i)));
            
            EstimateValue(:,t,Enemy_label(i),time,agentlabel,iteration) = EstimateState(:,Enemy_label(i));
            PredictValue(:,t,Enemy_label(i),time,agentlabel,iteration) = PredictValueTemp(:,Enemy_label(i));
            KalmanValue(:,t,Enemy_label(i),time,agentlabel,iteration) = NewPredictValue(:,Enemy_label(i));
            if t == 1 
            Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) = EstimateValue(:,t,Enemy_label(i),time,agentlabel,iteration);
            else 
            Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) = KalmanValue(:,t,Enemy_label(i),time,agentlabel,iteration);    
            end
            
            CurrentState(:) = squeeze(Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration));
            CurrentU(:) = CurrentU(:) + CurrentU(:) - HistoryU;
            q_i_current = squeeze(q_i_pre(1:2,t+1,:,time));
            q_i_current = reshape(q_i_current,[2*USV_number,1]);
            DataX = [squeeze(NewPredictValue(:,Enemy_label(i)));
                q_0_pre(1:2,t+1,time);
                q_i_current;
                ];
        end
    elseif (time-1)==t_start(Enemy_label(i))
        
        for t = 1:T_MPC
            tempx =  py.mdn_matlab.predict_value(DataX,'xe');
            tempy =  py.mdn_matlab.predict_value(DataX,'ye');
            Ptemp(1,1) = double(tempx);
            Ptemp(2,1) = double(tempy);
            Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) = Ptemp;
            
            CurrentState(:) = Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration);
            CurrentU(:) = Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) - Scenario(1:2,t,Enemy_label(i),agentlabel,time,iteration);
            q_i_current = squeeze(q_i_pre(1:2,t,:,time));
            q_i_current = reshape(q_i_current,[2*USV_number,1]);
            DataX = [squeeze(Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration));
                q_0_pre(1:2,t,time);
                q_i_current;
                ];
        end
    elseif time==(t_start(Enemy_label(i)))
        for t = 1:T_MPC
            tempx =  py.mdn_matlab.predict_value(DataX,'xe');
            tempy =  py.mdn_matlab.predict_value(DataX,'ye');
            Ptemp(1,1) = double(tempx);
            Ptemp(2,1) = double(tempy);
            Scenario(1:2,t+1,Enemy_label(i),agentlabel,time,iteration) = Ptemp;
            
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