function CSV_data=DataFromCsv(fileName)
% ---------从aubo_control的诊断文件中读取各种算法需要的数据------

data = readtable(fileName,...
    'ReadVariableNames',true,'HeaderLines',26);
% data_name=data.Properties.VariableNames;

%% --------读取电流力矩常数--------
CSV_data.constant=csvread(fileName,7,1,[7 1 7 6]);
CSV_data.gravity = csvread(fileName,5,1,[5 1 5 3]);

%% --------读取摩擦力参数-----------
fric_para0 = csvread(fileName,9,1,[9 1 23 6]);
miu = csvread(fileName,23,1,[23 1 23 6]);

fric_para0 = [fric_para0' miu'];

for i=1:6
    CSV_data.fric_j_para(i,1) = fric_para0(i,2);%Fs
    CSV_data.fric_j_para(i,2) = fric_para0(i,1);%Fc
    CSV_data.fric_j_para(i,3) = fric_para0(i,10);%Vs
    CSV_data.fric_j_para(i,4) = fric_para0(i,15);%miu
    CSV_data.fric_j_para(i,5:8) = fric_para0(i,6:9);%Fv
    CSV_data.fric_j_para(i,9:11) = fric_para0(i,3:5);%Ft
    CSV_data.fric_j_para(i,12) = fric_para0(i,11)/CSV_data.constant(i);%c
    CSV_data.fric_j_para(i,13) = fric_para0(i,12)/CSV_data.constant(i);
end

%% --------time计算------------------
[row, col] = size(data);
time = 1:5:5*row;
CSV_data.time = time'/1000;

%% --------诊断文件数据解析-----
CSV_data.target_q=data{:,{'target_q1','target_q2','target_q3','target_q4','target_q5','target_q6'}};
CSV_data.target_qd=data{:,{'target_qd1','target_qd2','target_qd3','target_qd4','target_qd5','target_qd6'}};
CSV_data.target_qdd=data{:,{'target_qdd1','target_qdd2','target_qdd3','target_qdd4','target_qdd5','target_qdd6'}};
CSV_data.target_curr=data{:,{'target_curr1','target_curr2','target_curr3','target_curr4','target_curr5','target_curr6'}};

CSV_data.q = data{:,{'q1','q2','q3','q4','q5','q6'}};
CSV_data.qd = data{:,{'qd1','qd2','qd3','qd4','qd5','qd6'}};
CSV_data.qdd = data{:,{'qdd1','qdd2','qdd3','qdd4','qdd5','qdd6'}};

CSV_data.current = data{:,{'current1','current2','current3','current4','current5','current6'}};
CSV_data.temperature =data{:,{'temperature1','temperature2','temperature3','temperature4','temperature5','temperature6'}};
CSV_data.friction = data{:,{'friction1','friction2','friction3','friction4','friction5','friction6'}};

CSV_data.mass = data{:,{'mass','com_x','com_y','com_z'}};


end