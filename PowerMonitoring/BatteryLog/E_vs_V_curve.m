% File: plot_voltage_vs_energy_linear_regression.m

% ========== STEP 1: 读取数据 ==========
data = readtable('power_log.csv');

Voltage = data.Voltage;
Current_Motor = data.Current_Motor;
Current_Board = data.Current_Board;

% ========== STEP 2: 处理 Current_Motor ==========
Current_Motor = Current_Motor/10 + 0.75 * (Current_Motor ~= 0);

% ========== STEP 3: 计算能量 ==========
dt = 0.1; % 100 毫秒
Total_Current = Current_Motor + Current_Board;
Power = Voltage .* Total_Current;
Energy_Used = cumtrapz(Power) * dt;
Energy_Remain = max(Energy_Used) - Energy_Used;

% ========== STEP 4: 取平均每 20 个点 ==========
segment_avg = 20;
num_avg = floor(length(Voltage) / segment_avg);

Voltage_avg = zeros(num_avg, 1);
Energy_avg = zeros(num_avg, 1);

for i = 1:num_avg
    idx_start = (i-1)*segment_avg + 1;
    idx_end = i*segment_avg;
    
    Voltage_avg(i) = mean(Voltage(idx_start:idx_end));
    Energy_avg(i) = mean(Energy_Remain(idx_start:idx_end));
end

% ========== STEP 5: 阈值分段 ==========
idx1 = find(Voltage_avg < 14.8, 1);
idx2 = find(Voltage_avg < 14.1, 1);
idx3 = find(Voltage_avg < 12.5, 1);

if isempty(idx1), idx1 = length(Voltage_avg)+1; end
if isempty(idx2), idx2 = length(Voltage_avg)+1; end
if isempty(idx3), idx3 = length(Voltage_avg)+1; end

segments = {
    1, idx1-1;
    idx1, idx2-1;
    idx2, idx3-1;
    idx3, length(Voltage_avg)
};

% ========== STEP 6: 线性回归拟合每段 ==========
figure;
hold on;
grid on;
xlabel('Voltage (V)');
ylabel('Energy Used (J)');
title('Energy vs Voltage - Linear Regression Per Segment');
set(gca, 'XDir', 'reverse');
colors = lines(4);

fprintf('\n=== 拟合结果（线性回归）：Energy = k * Voltage + b ===\n');

for i = 1:4
    idx_range = segments{i,1}:segments{i,2};
    if isempty(idx_range) || length(idx_range) < 2
        continue;
    end
    
    V = Voltage_avg(idx_range);
    E = Energy_avg(idx_range);
    
    % 使用 polyfit 进行线性回归
    p = polyfit(V, E, 1); % E = p(1)*V + p(2)
    E_fit = polyval(p, V);
    
    plot(V, E_fit, 'LineWidth', 2, 'Color', colors(i,:));
    
    % 图中标注
    eqn = sprintf('E = %.3fV + %.1f', p(1), p(2));
    text(V(round(end/2)), E_fit(round(end/2)), eqn, 'Color', colors(i,:), 'FontSize', 10);
    
    % 命令行输出
    fprintf('Segment %d: %s\n', i, eqn);
end

% 原始曲线（可选）
plot(Voltage_avg, Energy_avg, '--k', 'LineWidth', 1);
legend('Segment 1','Segment 2','Segment 3','Segment 4','Original');
