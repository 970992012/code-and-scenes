function simin = locomotion_control(t_total, dt, delta_z)
%LOCOMOTION_CONTROL 四足机器人逆运动学计算与Simulink联合Adams仿真数据生成
%   语法:
%   simin = locomotion_control(t_total, dt, delta_z)
%   其中:
%       t_total - 总仿真时间（秒）
%       dt - 步长（秒）
%       delta_z - 站立高度变化（毫米）
%
%   输出:
%   simin - 包含时间节点和关节角度变化量的数据结构
%       simin.time - 时间向量
%       simin.signals.values - 角度变化量矩阵 (12列，每列一个关节)
%       simin.signals.dimensions - 数据维度


% 仿真参数设置
t = (0:dt:t_total)'; % 时间列向量
n_steps = length(t); % 步数

if delta_z < 0
    delta_z = 0;
end

% 定义四条腿的足端相对于机身0号坐标系初始位置和末位置，并换算为相对于前半身1号坐标系的坐标
% 格式: [x, y, z]
% 左前腿 (LF)
p0_LF = [212.3087, 207.5641, -40.8488];  % 初始位置，是左前脚趴在地面的启动位置[212.3087, 207.5641, -40.8488]
pf_LF = [199.75, 85+93.55, -40.8488-delta_z]; % 末位置

p0_LF = transform(p0_LF);
pf_LF = transform(pf_LF);

% 右前腿 (RF) ,
p0_RF = [212.3087, -207.5641, -40.8488];   % 初始位置[212.3087, -207.5641, -40.8488]
pf_RF = [199.75, -(85+93.55), -40.8488-delta_z]; % 末位置

%y标坐标取相反数再变化
p0_RF(2) = -p0_RF(2);
pf_RF(2) = -pf_RF(2);
p0_RF = transform(p0_RF);
pf_RF = transform(pf_RF);

% 左后腿 (LH)
p0_LR = [-187.1913, 207.5641, -40.8488]; % 初始位置[-187.1913, 207.5641, -40.8488]
pf_LR = [-199.75, 85+93.55, -40.8488-delta_z]; % 末位置
%x坐标偏移399.5mm再变换
p0_LR(1) = p0_LR(1)+399.5;
pf_LR(1) = pf_LR(1)+399.5;

p0_LR = transform(p0_LR);
pf_LR = transform(pf_LR);

% 右后腿 (RH)
p0_RR = [-187.1913, -207.5641, -40.8488];  % 初始位置[-187.1913, -207.5641, -40.8488]
pf_RR = [-199.75, -(85+93.55), -40.8488-delta_z];  % 末位置
%y标坐标取相反数，x坐标偏移399.5mm
p0_RR(2) = -p0_RR(2);
pf_RR(2) = -pf_RR(2);
p0_RR(1) = p0_RR(1)+399.5;
pf_RR(1) = pf_RR(1)+399.5;

p0_RR = transform(p0_RR);
pf_RR = transform(pf_RR);

% 定义各腿关节角度的符号因子（变换关节旋转方向）
% 格式: [sign_theta1, sign_theta2, sign_theta3]
sign_RF = [-1, -1, -1];    % 右前腿符号因子
sign_LR = [-1, 1, 1];   % 左后腿符号因子
sign_RR = [1, -1, -1];  % 右后腿符号因子

% 初始化关节角度数组
theta_LF = zeros(n_steps, 3); % 左前腿关节角度
theta_RF = zeros(n_steps, 3); % 右前腿关节角度
theta_LR = zeros(n_steps, 3); % 左后腿关节角度
theta_RR = zeros(n_steps, 3); % 右后腿关节角度

% 计算每条腿的轨迹（直线插值）
for i = 1:n_steps
    % 计算当前时间点的插值系数
    alpha = (i-1) / (n_steps-1);
    %alpha=1角度会跳变
    if alpha == 1
        alpha = (n_steps - 2)/(n_steps-1);
    end
    
    % 计算各腿当前足端位置
    p_LF = p0_LF + alpha * (pf_LF - p0_LF);
    p_RF = p0_RF + alpha * (pf_RF - p0_RF);
    p_LR = p0_LR + alpha * (pf_LR - p0_LR);
    p_RR = p0_RR + alpha * (pf_RR - p0_RR);
    
    % 计算左前腿逆运动学
    [theta1, theta2, theta3] = inverse_kinematics(p_LF(1), p_LF(2), p_LF(3));
    theta_LF(i, :) = [theta1, theta2, theta3];
    
    % 计算右前腿逆运动学，
    [theta1_ref, theta2_ref, theta3_ref] = inverse_kinematics(p_RF(1), p_RF(2),p_RF(3));

    theta_RF(i, :) = [sign_RF(1)*theta1_ref, sign_RF(2)*theta2_ref, sign_RF(3)*theta3_ref];
    
    % 计算左后腿逆运动学，
    [theta1_ref, theta2_ref, theta3_ref] = inverse_kinematics(p_LR(1), p_LR(2), p_LR(3));

    theta_LR(i, :) = [sign_LR(1)*theta1_ref, sign_LR(2)*theta2_ref, sign_LR(3)*theta3_ref];
    
    % 计算右后腿逆运动学，
    [theta1_ref, theta2_ref, theta3_ref] = inverse_kinematics(p_RR(1), p_RR(2),p_RR(3));

    theta_RR(i, :) = [sign_RR(1)*theta1_ref, sign_RR(2)*theta2_ref, sign_RR(3)*theta3_ref];
end

% 计算关节角度
all_theta = zeros(n_steps, 12);
all_theta(:, 1:3) = theta_LF;   % 左前腿
all_theta(:, 4:6) = theta_RF;   % 右前腿
all_theta(:, 7:9) = theta_LR;   % 左后腿
all_theta(:, 10:12) = theta_RR; % 右后腿
% 计算相对于初始位置的关节角度变化量（从零开始）
delta_theta = zeros(n_steps, 12);
% 第一行（初始时刻）为零
delta_theta(1, :) = zeros(1, 12);
% 后续时刻减去初始时刻的角度值
for i = 2:n_steps
    delta_theta(i, :) = all_theta(i, :) - all_theta(1, :);
end


% 创建数据格式（结构体形式）
simin.time = t;  % 时间向量
simin.signals.values = delta_theta; % 角度变化量矩阵
simin.signals.dimensions = 12; % 数据维度

% 保存数据到MAT文件（可选）
save('joint_angle_deltas.mat', 'simin');

% 显示数据信息
disp('Simulink输入数据结构:');
disp('simin.time - 时间向量');
disp('simin.signals.values - 角度变化量矩阵 (12列，每列一个关节)');
disp(['数据维度: ', num2str(size(delta_theta))]);

% 绘制关节角度变化量曲线
figure;
% 设置图形窗口大小
% 固定窗口大小
set(gcf, 'Position', [800, 300, 600, 600]);  % 设置窗口位置和大小 [左, 上, 宽, 高]
set(gcf, 'Resize', 'off');                    % 禁止调整窗口大小

% 手动设置子图大小参数
subplot_width = 0.3;   % 子图宽度 (0-1之间)
subplot_height = 0.3;  % 子图高度 (0-1之间)
subplot_gap_x = 0.15;   % 子图水平间距 (0-1之间)
subplot_gap_y = 0.1;   % 子图垂直间距 (0-1之间)
start_x = 0.05;         % 起始x位置 (0-1之间)
start_y = 0.05;         % 起始y位置 (0-1之间，注意y轴从下往上)
for i = 1:12
    subplot(4,3,i);
    plot(t, delta_theta(:, i));
     title(['关节', num2str(i), '角度变化量']);
    xlabel('时间(s)');
    ylabel('\Delta{\it\theta}(rad)');
    grid on;

    % 设置中文字体为宋体，英文字体为Times New Roman
    set(gca, 'FontName', '宋体', 'FontSize',10.5);
    set(gcf, 'DefaultAxesFontName', '宋体');

    % 获取当前坐标轴的所有文本对象并设置字体
    texts = findall(gca, 'Type', 'text');
    for j = 1:length(texts)
        if contains(get(texts(j), 'String'), '时间') || ...
           contains(get(texts(j), 'String'), '变化量') || ...
           contains(get(texts(j), 'String'), '关节')
            set(texts(j), 'FontName', '宋体', 'FontSize', 10.5);
        else
            set(texts(j), 'FontName', 'Times New Roman', 'FontSize', 10.5);
        end
    end
end
end

function p1 = transform(p0)
% transformPoint 将点从0坐标系转换到1坐标系
% 输入:
%   p0 - 1x3 行向量，表示点在0坐标系下的坐标
% 输出:
%   p1 - 1x3 行向量，表示点在1坐标系下的坐标

    % 旋转矩阵 (0系到1系)
    R = [0 -1 0;
         1  0 0;
         0  0 1];

    % 平移向量 (1系原点在0系下的坐标)
    t = [0.75, 0, 0];

    % 变换公式: p1 = (p0 - t) * R
    p1 = (p0 - t) * R;
end


% 逆运动学函数
function [theta1, theta2, theta3] = inverse_kinematics(x_p, y_p, z_p)
    % 逆运动学求解函数，计算关节角度θ2、θ3和θ4
    % 输入：x_p, y_p, z_p - 点P的坐标（单位：mm）
    % 输出：theta2, theta3, theta4 - 关节角度（单位：弧度）

    % 常量定义
    L1 = 93.55;   % mm
    L3 = 250;     % mm
    L4 = 250;     % mm
    offset_y = 85; % y方向偏移
    offset_x = 199; % x方向偏移

    % 计算L2
    term = (x_p - offset_y)^2 + z_p^2 - L1^2;
    if term < 0
        error('点不可达：L2的平方根内值为负');
    end
    L2 = sqrt(term);

    % 计算θ1
    y2 = z_p * L1 + (x_p - offset_y) * L2;
    x2 = (x_p - offset_y) * L1 - z_p * L2;
    theta1 = atan2(y2, x2) + pi/2;

    % 计算|O3P|^2
    O3P_sq = (x_p - offset_y)^2 + (y_p + offset_x)^2 + z_p^2 - L1^2;

    % 计算θ3
    cos_theta3 = (L3^2 + L4^2 - O3P_sq) / (2 * L3 * L4);
    
    if abs(cos_theta3) > 1
        error('点不可达：acos参数超出范围');
    end
    theta3 = acos(cos_theta3) - pi;

    % 计算A、B、C、D
    A = cos(theta3) + 1;
    B = sin(theta3);
    C = -(y_p + offset_x) / L3;

    % 检查cos(theta2)是否为零
    % if abs(cos(theta1)) < 1e-10
    %     error('cos(theta2)为零，无法计算D');
    % end
    D = (x_p - L1 * sin(theta1) - offset_y) / (L3 * cos(theta1));

    % 计算θ2
    numerator_theta3 = A * C - B * D;
    denominator_theta3 = B * C + A * D;
    theta2_raw = atan2(numerator_theta3, denominator_theta3);

    % ----------- 连续性修正 -----------
    persistent theta2_prev
    if isempty(theta2_prev)
        theta2_prev = -pi/2; % 初始值设在中间
    end

    theta2 = theta2_raw;
    if abs(theta2 - theta2_prev) > pi/2
        if theta2 > theta2_prev
            theta2 = theta2 - 2*pi;
        else
            theta2 = theta2 + 2*pi;
        end
    end

    % ----------- 物理限制 [-150°, -30°] -----------
    % theta2 = max(min(theta2, deg2rad(-30)), deg2rad(-150));

    % 更新历史值
    theta2_prev = theta2;
end