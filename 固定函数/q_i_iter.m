function [q,q_0] = q_i_iter(nu,q_i,time)
%UNTITLED2 此处显示有关此函数的摘要

global T_MPC;
global task;
global q_i_pre;% 存放无人艇i（跟随艇）的预测数据(3,T_MPC+1,USV_number,Scenario_number)


q_i_pre(:,1,task,time)= q_i(:,:,task);

   for t=2:T_MPC+1
       q_i_pre(:,t,task,time)= q_iter_in(q_i_pre(:,t-1,task,time),nu(:,t-1,:));  
   end 


end