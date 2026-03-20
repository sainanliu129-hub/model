addpath(genpath(fullfile(pwd, 'applications', 'Body_GravityPara_Iden')));
addpath(genpath(fullfile(pwd, 'FrankaEmikaPandaDynModel', 'matlab')));
addpath(genpath(fullfile(pwd, 'utility_function')));

% Set debug_level to 1 for more diagnostics
debug_level = 1;

run('applications/Body_GravityPara_Iden/test/test_ReMatrix_E1_limb_URDF.m');