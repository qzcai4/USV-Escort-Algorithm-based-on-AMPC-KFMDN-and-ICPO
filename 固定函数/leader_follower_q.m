function [q_i_expect,form] = leader_follower_q(q_0,nu_0,q_i)
%LEADER_FOLLOWER 利用领导跟随策略生成护卫舰编队的驱动量
% 输入量：q_0(3)：当前时刻主舰的状态量，nu_0(3)：当前时刻主舰的驱动量,q_i(3,i)：当前时刻各无人艇的状态量,
%         r_form：参与编队的半径范围，dyna：无人艇的动力学参数[m11,m22,d22],r_def：无人艇的拦截距离，r_safe：无人艇与主舰的安全距离
%         nu_limit：无人艇速度限幅
% 输出量：nu_new(3,i):下一时刻的无人艇的驱动量

%% 声明全局变量
h=1; % 每次状态更新的步长，驱动量更新之间，以h步长进行状态更新，降低模型非线性影响
global nu_limit; % 给定无人艇驱动向量限幅，最大速度25m/s，最小速度0m/s,最大转角25°,最小转角-25°,目前全体我方采用同构
global r_def; % 成功拦截距离15m
global r_safe; % 与主舰的碰撞安全距离20m
global r_form; % 可参与编队的无人艇距主舰距离阈值
global dyna; % 无人艇动力学参数，参考刘彬博士论文
global Enemy_number;
global USV_number;


form_number = (USV_number-Enemy_number);
%% 初始化变量空间并提取信息
alpha = dyna(1)/dyna(2);
beta = dyna(3)/dyna(2);
nu_i_new = zeros(size(q_i));
q_i_new = zeros(size(q_i));
%% 筛选出参与编队的无人艇
% 计算主舰与我方无人艇距离
d_i = sqrt(sum((squeeze(q_i(1:2, 1:form_number)) - squeeze(q_0(1:2))).^2, 1));
% 获取参与编队和暂不参与编队的无人艇的标签
if any(d_i>r_form)
    noform = 1:form_number;
    noform(d_i <= r_form) = [];
    form = 1:form_number;
    form(noform) = [];
else
    noform = [];
    form = 1:form_number;
end
% 计算下一时刻主舰状态
q_0_new = q_0+h*JETA(q_0(3))*nu_0(:);
%% 若存在不参与编队的无人艇，则单独生成其驱动量
if ~isempty(noform)
    nu_i_new(1, noform) = nu_limit(1);
    nu_i_new(2, noform) = 0;
    delta_x = q_0_new(1) - q_i(1, noform);
    delta_y = q_0_new(2) - q_i(2, noform);
    theta_ref = atan2(delta_y, delta_x) + 2 * pi;
    nu_i_new(3, noform) = theta_ref - q_i(3, noform);
end
%% 生成编队，并匹配编队位置与无人艇
if ~isempty(form)
USV_number_form = size(form,2);

if USV_number_form>2
    theta = 2*pi/USV_number_form;
    r = r_def/(sin(theta/2));
    angles = (0:USV_number_form-1) * theta;
    q_i_new(1, form) = q_0_new(1) + r * cos(angles);
    q_i_new(2, form) = q_0_new(2) + r * sin(angles);
    q_i_new(3, form) = q_0_new(3);
elseif USV_number_form==2
    r = r_safe;
    q_i_new(:,form(1)) = q_0_new+[r 0 0]';
    q_i_new(:,form(2)) = q_0_new-[r 0 0]';    
elseif USV_number_form==1
    r = r_safe;
    q_i_new(:,form(1)) = q_0_new+[r 0 0]';
end

    q_i_expect = squeeze(q_i_new(:,form));

end
end


