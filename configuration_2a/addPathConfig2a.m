function [] = addPathConfig2a()

this_file_path = fileparts(mfilename('fullpath'));
cd(this_file_path);
addpath(genpath(this_file_path));
cd ..
addpath(genpath('functions'));
addpath(genpath('library'));
addpath(genpath('models'));
addpath(genpath('modules'));
cd(this_file_path);
addpath(genpath('params'));
addpath(genpath('data'));

end