% File: plot_voltage_vs_energy_fitted.m

% ========== STEP 1: 读取数据 ==========
data = readtable('power_log.csv');

Voltage = data.Voltage;
Current_Motor = data.Current_Motor;
Current_Board = data.Current_Board;

% ========== STEP 2: 处理 Current_Motor ==========
Current_Motor = Current_Motor/10 + 0.7 * (Current_Motor ~= 0);

% ========== STEP 3: 计算能量 ==========
dt = 0.1; % 100 毫秒
Total_Current = Current_Motor + Current_Board;
Power = Voltage .* Total_Current;
Energy_Used = cumtrapz(Power) * dt;

% ========== STEP 4: 取平均每 100 个点 ==========
segment_avg = 20;
num_avg = floor(length(Voltage) / segment_avg);

Voltage_avg = zeros(num_avg, 1);
Energy_avg = zeros(num_avg, 1);

for i = 1:num_avg
    idx_start = (i-1)*segment_avg + 1;
    idx_end = i*segment_avg;
    
    Voltage_avg(i) = mean(Voltage(idx_start:idx_end));
    Energy_avg(i) = mean(Energy_Used(idx_start:idx_end));
end

% ========== STEP 5: 阈值分段 ==========
idx1 = find(Voltage_avg < 14.8, 1);
idx2 = find(Voltage_avg < 14.1, 1);
idx3 = find(Voltage_avg < 12.5, 1);

% 处理找不到的情况
if isempty(idx1), idx1 = length(Voltage_avg)+1; end
if isempty(idx2), idx2 = length(Voltage_avg)+1; end
if isempty(idx3), idx3 = length(Voltage_avg)+1; end

segments = {
    1, idx1-1;
    idx1, idx2-1;
    idx2, idx3-1;
    idx3, length(Voltage_avg)
};

% ========== STEP 6: 拟合与绘图 ==========
figure;
hold on;
grid on;
xlabel('Energy Used (Joules)');
ylabel('Battery Voltage (V)');
title('Voltage vs Energy - Piecewise Linear Fit');
colors = lines(4);

for i = 1:4
    idx_range = segments{i,1}:segments{i,2};
    if isempty(idx_range) || length(idx_range) < 2
        continue;
    end
    
    x = Energy_avg(idx_range);
    y = Voltage_avg(idx_range);
    
    p = polyfit(x, y, 1);
    y_fit = polyval(p, x);
    
    plot(x, y_fit, 'LineWidth', 2, 'Color', colors(i,:));
    
    % 标注拟合方程
    eqn = sprintf('y = %.3fx + %.2f', p(1), p(2));
    text(x(round(end/2)), y_fit(round(end/2)), eqn, 'Color', colors(i,:), 'FontSize', 10);
end

% 原始数据（可选）
plot(Energy_avg, Voltage_avg, '--k', 'LineWidth', 1);

legend('Segment 1','Segment 2','Segment 3','Segment 4','Original');
