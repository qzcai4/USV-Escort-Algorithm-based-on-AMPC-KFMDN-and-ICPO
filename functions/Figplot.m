function Figplot()
global q_0 q_e q_i t_start t_end T;
CurrentFolder = pwd;
plotfig(q_0,q_e,q_i,t_start,t_end,T,CurrentFolder);
im2gif(CurrentFolder); 
end