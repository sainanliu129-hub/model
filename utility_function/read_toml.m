%%
%该函数用于读取.toml文件中的机械臂模型相关原始参数
%可识别现有类型参数，后续添加变量按照现有模板在.toml文件中添加，此程序不用修改
%%
function parameter = read_toml(filename)
    
    % 打开文件
    fid = fopen(filename, 'r');
    if (fid == -1)
        error('无法打开文件：%s', filename);
    end
    
    % 初始化结果结构
    data = struct();
    currentSection = '';
    i = 0;
    % 逐行读取文件内容
    while ~feof(fid)
        line = fgetl(fid);
        
        % 忽略注释和空行
        if (isempty(line) || startsWith(strtrim(line), '#'))
            continue;
        end
        
        % 检查是否是节标题
        if (startsWith(line, '[') && endsWith(line, ']'))
            currentSection = strtrim(line(2:end-1));
            data.(currentSection) = struct();
        else
            % 解析键值对
            [key, value] = strtok(line, '=');
            key = strtrim(key);
            %判断是哪种类型的数据
            if ((contains(line,'=') && contains(line, '[') && ~contains(line, ']'))|| (value == ""))
                %处理 data = [0.0, 4.801280, 20.105400, , ... ,9.282290]类型的数据
                i = i + 1;
                if(i == 1)
                    keylast = key;                 
                else   
                    key = keylast;  
                end
                line = strrep(line, '[', ',');
                line = strrep(line, ']', '');
                value = str2double(strsplit(line, ','));
                value = value(~isnan(value));
                if(i == 1)            
                    data.(currentSection).(key) = value;
                else
                    data.(currentSection).(key) = [data.(currentSection).(key), value];
                end
            else
                %处理 Fs  = [1.8, 1.364, 1.2, 0.41, 0.3645, 0.3]类型的数据
                value = strtrim(value(2:end));
                value = str2num(value); % 尝试将值转换为数值
                % 将键值对存储到当前节中
                if (~isempty(key) && ~isempty(value))
                    data.(currentSection).(key) = value;
                end
            end
        end
    end   
    % 关闭文件
    fclose(fid);
    
    parameter = data;
    
end
