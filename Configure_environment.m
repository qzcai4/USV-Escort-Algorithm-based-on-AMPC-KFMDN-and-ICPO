function Configure_environment()
%% 配置环境信息
rng('default');
CurrentFolder = pwd;
AllFolders = dir(CurrentFolder);
isub = [AllFolders(:).isdir];
FoldersList = {AllFolders(isub).name}';
FoldersList(ismember(FoldersList,{'.','..'})) = [];
FoldersList(ismember(FoldersList,{'functions','model','__pycache__'})) = [];
addpath(fullfile(CurrentFolder, 'functions'));

pyexe = pyenv();

fprintf('Please select your python interpreter.\n');

[file, path] = uigetfile('python.exe', 'Select Python Interpreter');
if isequal(file, 0)
    selected_python_path = '';
    error('User canceled the selection operation\n');
else
    selected_python_path = fullfile(path, file);
    
    pyexe = pyenv();
    
    if strcmp(pyexe.Executable, selected_python_path) ~= 1
        pyenv('Version', selected_python_path);
        fprintf('Python interpreter has been updated\n');
    else
        fprintf('Python interpreter does not need to be changed\n');
    end
    
    fprintf('Your current Python interpreter path is:\n%s\n', pyexe.Executable);
end
try
    py.importlib.import_module('mdn_matlab');
    has_mdn_matlab = true;
catch
    has_mdn_matlab = false;
end
if has_mdn_matlab
    py_dict = py.sys.modules;
    py_dict.pop('mdn_matlab');
end

end