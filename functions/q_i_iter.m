function [q,q_0] = q_i_iter(nu,q_i,time)

global T_MPC;
global task;
global q_i_pre;


q_i_pre(:,1,task,time)= q_i(:,:,task);

   for t=2:T_MPC+1
       q_i_pre(:,t,task,time)= q_iter_in(q_i_pre(:,t-1,task,time),nu(:,t-1,:));  
   end 


end