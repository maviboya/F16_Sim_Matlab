function setup_f16()
%SETUP_F16 Kaynak, test ve doğrulama işlevlerini MATLAB yoluna ekler.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'src'));
addpath(fullfile(root,'tests'));
addpath(fullfile(root,'validation'));
end
