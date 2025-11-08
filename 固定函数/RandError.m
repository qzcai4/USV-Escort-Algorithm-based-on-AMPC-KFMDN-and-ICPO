function [Error] = RandError(mu)
mu1 = mu;  
mu2 = -mu;  
sigma1 = mu/3; % 初始标准差，需要调整  
sigma2 = sigma1; % 由于分布是对称的  

% 生成随机数
weight = randi([0,1]);
Error = weight*normrnd(mu1, sigma1)+(1-weight)*normrnd(mu2, sigma2);
 
end