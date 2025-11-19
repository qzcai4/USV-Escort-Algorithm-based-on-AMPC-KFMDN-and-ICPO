function im2gif(CurrentFolder)
fname = ['figure.gif']; 
Ncolor = 64; 
dt = 0.1; 
dt_beg = 0;
dt_end = 1; 
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
   delete ([CurrentFolder,'\*.jpg']);
end