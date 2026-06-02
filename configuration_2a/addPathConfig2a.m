function [] = addPathConfig2a()

this_file_path = fileparts(mfilename('fullpath'));
cd(this_file_path);
addpath(genpath(this_file_path));
cd ..
addpath(genpath('modules'));
addpath(genpath('functions'));
cd(this_file_path);
addpath(genpath('params'));
addpath(genpath('data'));

end