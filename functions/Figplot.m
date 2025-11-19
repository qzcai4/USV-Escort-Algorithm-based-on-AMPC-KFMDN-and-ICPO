function Figplot()
    global q_0 q_e q_i t_start t_end T;
    CurrentFolder = pwd; % 获取当前工作文件夹路径，便于存放生成图像
    plotfig(q_0,q_e,q_i,t_start,t_end,T,CurrentFolder);
    im2gif(CurrentFolder); % 生成动图
end