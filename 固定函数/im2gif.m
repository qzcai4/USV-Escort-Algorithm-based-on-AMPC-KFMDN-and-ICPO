function im2gif(CurrentFolder)
% 功能：将导出的图片生成动图
% 输入变量：CurrentFolder：存放图片的路径
% === 参数 =====
fname = ['效果图.gif']; % 文件名
Ncolor = 64; % 颜色数（最大 256）
dt = 0.1; % 间隔时间（秒）
dt_beg = 0; % 第一帧时间（秒）
dt_end = 1; % 最后一帧时间（秒）
% ================
[filename,path] = uigetfile([CurrentFolder,'\*.jpg'], 'multiselect', 'on');
filename = sort(filename);
cd(path);
I = imread(filename{1});
[X,cmap] = rgb2ind(I,Ncolor,'nodither');
imwrite(X, cmap, fname, 'gif', 'Loopcount', inf, 'DelayTime', dt_beg);
figure;
N = numel(filename);
for ii = 2:N
    if strcmp(filename{ii}, fname)
        continue;
    end
    I = imread(filename{ii});
    [X,cmap] = rgb2ind(I,Ncolor,'nodither');
    imshow(X, cmap); drawnow;
    if ii == N
        dt = dt_end;
    end
    imwrite(X, cmap, fname, 'gif', 'WriteMode', ....
            'append', 'DelayTime', dt);
end
% 删除用于生成gif的原始图片
   delete ([CurrentFolder,'\*.jpg']);
end