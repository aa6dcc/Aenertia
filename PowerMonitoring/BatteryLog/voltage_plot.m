% File: plot_voltage_vs_energy_avg.m

% 读取 CSV 文件
data = readtable('power_log.csv');

% 提取列
Voltage = data.Voltage;
Current_Motor = data.Current_Motor;
Current_Board = data.Current_Board;

% 时间间隔
dt = 0.01; % 10 毫秒

Current_Motor = 0.8 * (Current_Motor ~= 0);

% 总电流
Total_Current = Current_Motor + Current_Board;

% 功率 = 电压 * 电流
Power = Voltage .* Total_Current;

% 能量（焦耳）= 功率积分
Energy_Used = cumtrapz(Power) * dt;

% 分段平均处理参数
segment_size = 100; % 每 100 个点取一次平均
num_segments = floor(length(Voltage) / segment_size);

% 预分配空间
Voltage_avg = zeros(num_segments, 1);
Energy_avg = zeros(num_segments, 1);

% 对每段取平均
for i = 1:num_segments
    idx_start = (i-1) * segment_size + 1;
    idx_end = i * segment_size;
    
    Voltage_avg(i) = mean(Voltage(idx_start:idx_end));
    Energy_avg(i) = mean(Energy_Used(idx_start:idx_end));
end

% 绘图
figure;
plot(Energy_avg, Voltage_avg, 'LineWidth', 1.5);
xlabel('Energy Used (Joules)');
ylabel('Battery Voltage (V)');
title('Battery Voltage vs Energy Used (Averaged)');
grid on;
