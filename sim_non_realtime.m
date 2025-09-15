%调用远程API控制CoppeliaSim仿真
client = RemoteAPIClient();
sim = client.require('sim');
%所有关节的句柄
jointhandle = [20, 32, 59, 47, 23, 35, 62, 50, 26, 38, 65, 53, 17, 44];
%初始化
hip_init = deg2rad(25.17);
thigh_init = deg2rad(71.61);
shank_init = deg2rad(159.26);

targetPosition = rad2deg(sim.getJointTargetPosition(jointhandle(9)));
position = rad2deg(sim.getJointPosition(jointhandle(9)));
% 小腿位置要做下处理，默认位置：
% 左侧小腿在仿真器里的位置是-33.518度   （+33.518=0）
% 右侧小腿是+33.518度                （-33.518=0）
e = deg2rad(33.518);
% 初始化卧姿,腰部不做处理
for i = 1:12 

if i < 5
sim.setJointPosition(jointhandle(1), hip_init);
sim.setJointTargetPosition(jointhandle(1), hip_init,[]);

sim.setJointPosition(jointhandle(2), -hip_init);
sim.setJointTargetPosition(jointhandle(2), -hip_init,[]);

sim.setJointPosition(jointhandle(3),-hip_init);
sim.setJointTargetPosition(jointhandle(3), -hip_init,[]);

sim.setJointPosition(jointhandle(4), hip_init);
sim.setJointTargetPosition(jointhandle(4), hip_init,[]);
elseif i < 9
sim.setJointPosition(jointhandle(i), (-1)^(i+1)*thigh_init);
sim.setJointTargetPosition(jointhandle(i), (-1)^(i+1)*thigh_init,[]);
else
sim.setJointPosition(jointhandle(i), (-1)^i*(shank_init + e));
sim.setJointTargetPosition(jointhandle(i), (-1)^i*(shank_init + e),[]);
end

end
%关节角度变化量计算
simin = locomotion_control(2, 0.05, 250);
time_points = simin.time;
angle_changes = simin.signals.values;
% 启动仿真
sim.startSimulation(); 
loop = 1;
LF_hip_pos = sim.getJointPosition(jointhandle(1));
RF_hip_pos = sim.getJointPosition(jointhandle(2));
LR_hip_pos = sim.getJointPosition(jointhandle(3));
RR_hip_pos = sim.getJointPosition(jointhandle(4));
LF_thigh_pos = sim.getJointPosition(jointhandle(5));
RF_thigh_pos = sim.getJointPosition(jointhandle(6));
LR_thigh_pos = sim.getJointPosition(jointhandle(7));
RR_thigh_pos = sim.getJointPosition(jointhandle(8));
LF_shank_pos = sim.getJointPosition(jointhandle(9));
RF_shank_pos = sim.getJointPosition(jointhandle(10));
LR_shank_pos = sim.getJointPosition(jointhandle(11));
RR_shank_pos = sim.getJointPosition(jointhandle(12));

while true
    time = sim.getSimulationTime();
    if loop == 1
        for i = 1:length(time_points)
                 
                % 获取当前时间点的所有关节角度变化量
                current_angles = angle_changes(i, :);
                % 将角度变化量应用到对应的关节
                % 计算目标角度 = 当前角度 + 角度变化量
                % 注意关节索引可能需要根据实际映射关系调整
            
                % 左前腿髋关节
                target_angle = LF_hip_pos + current_angles(1);
                sim.setJointTargetPosition(jointhandle(1), target_angle, []);
                % 右前腿髋关节
                target_angle = RF_hip_pos + current_angles(4);
                sim.setJointTargetPosition(jointhandle(2), target_angle, []);
                % 左后腿髋关节
                target_angle = LR_hip_pos + current_angles(7);
                sim.setJointTargetPosition(jointhandle(3), target_angle, []);
                % 右后腿髋关节
                target_angle = RR_hip_pos + current_angles(10);
                sim.setJointTargetPosition(jointhandle(4), target_angle, []);
            
                % 左前腿大腿关节
                target_angle = LF_thigh_pos + current_angles(2);
                sim.setJointTargetPosition(jointhandle(5), target_angle, []);
                % 右前腿大腿关节
                target_angle = RF_thigh_pos + current_angles(5);
                sim.setJointTargetPosition(jointhandle(6), target_angle, []);
                % 左后腿大腿关节
                target_angle = LR_thigh_pos + current_angles(8);
                sim.setJointTargetPosition(jointhandle(7), target_angle, []);
                % 右后腿大腿关节
                target_angle = RR_thigh_pos + current_angles(11);
                sim.setJointTargetPosition(jointhandle(8), target_angle, []);
            
                % 左前腿小腿关节
                target_angle = LF_shank_pos + current_angles(3);
                sim.setJointTargetPosition(jointhandle(9), target_angle, []);
                % 右前腿小腿关节
                target_angle = RF_shank_pos + current_angles(6);
                sim.setJointTargetPosition(jointhandle(10), target_angle, []);
                % 左后腿小腿关节
                target_angle = LR_shank_pos + current_angles(9);
                sim.setJointTargetPosition(jointhandle(11), target_angle, []);
                % 右后腿小腿关节
                target_angle = RR_shank_pos + current_angles(12);
                sim.setJointTargetPosition(jointhandle(12), target_angle, []);

                time = sim.getSimulationTime();
                fprintf('Simulation time: %.2f s\n', time);

        end
        loop = 0;
    end
    % fprintf('Simulation time: %.2f s\n', time); 
    if time > 5; break; end
end
sim.stopSimulation();
