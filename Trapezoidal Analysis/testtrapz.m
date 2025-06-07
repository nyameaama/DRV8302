% Number of states
num_states = 6;

max_period = 2000;
max_duty_cycle = 1000;

time_per_state = 1; 

total_time = num_states * time_per_state;

t = linspace(0, total_time, num_states + 1);

% Initialize phase voltage arrays
Va = zeros(1, num_states + 1);
Vb = zeros(1, num_states + 1);
Vc = zeros(1, num_states + 1);

for state = 1:num_states
    switch state - 1
        case 0
            % State 0: Phase A High (+V), Phase B Low (-V), Phase C Floating (0V)
            Va(state) = 1;  % +V
            Vb(state) = -1; % -V
            Vc(state) = 0;  % 0V
        case 1
            % State 1: Phase A High (+V), Phase B Floating (0V), Phase C Low (-V)
            Va(state) = 1;  % +V
            Vb(state) = 0;  % 0V
            Vc(state) = -1; % -V
        case 2
            % State 2: Phase A Floating (0V), Phase B High (+V), Phase C Low (-V)
            Va(state) = 0;  % 0V
            Vb(state) = 1;  % +V
            Vc(state) = -1; % -V
        case 3
            % State 3: Phase A Low (-V), Phase B High (+V), Phase C Floating (0V)
            Va(state) = -1; % -V
            Vb(state) = 1;  % +V
            Vc(state) = 0;  % 0V
        case 4
            % State 4: Phase A Low (-V), Phase B Floating (0V), Phase C High (+V)
            Va(state) = -1; % -V
            Vb(state) = 0;  % 0V
            Vc(state) = 1;  % +V
        case 5
            % State 5: Phase A Floating (0V), Phase B Low (-V), Phase C High (+V)
            Va(state) = 0;  % 0V
            Vb(state) = -1; % -V
            Vc(state) = 1;  % +V
    end
end

Va(end) = Va(end-1);
Vb(end) = Vb(end-1);
Vc(end) = Vc(end-1);

% Plot phase voltages
figure;
subplot(3,1,1);
stairs(t, Va, 'r', 'LineWidth', 2);
ylabel('Phase A Voltage');
ylim([-1.5, 1.5]);
grid on;

subplot(3,1,2);
stairs(t, Vb, 'g', 'LineWidth', 2);
ylabel('Phase B Voltage');
ylim([-1.5, 1.5]);
grid on;

subplot(3,1,3);
stairs(t, Vc, 'b', 'LineWidth', 2);
ylabel('Phase C Voltage');
xlabel('Time');
ylim([-1.5, 1.5]);
grid on;

% Calculate line-to-line voltages
Vab = Va - Vb;
Vbc = Vb - Vc;
Vca = Vc - Va;

% Plot line-to-line voltages
figure;
subplot(3,1,1);
stairs(t, Vab, 'k', 'LineWidth', 2);
ylabel('V_{AB}');
ylim([-2, 2]);
grid on;

subplot(3,1,2);
stairs(t, Vbc, 'm', 'LineWidth', 2);
ylabel('V_{BC}');
ylim([-2, 2]);
grid on;

subplot(3,1,3);
stairs(t, Vca, 'c', 'LineWidth', 2);
ylabel('V_{CA}');
xlabel('Time');
ylim([-2, 2]);
grid on;

% Combine all phase voltages in one plot
figure;
stairs(t, Va, 'r', 'LineWidth', 2); hold on;
stairs(t, Vb, 'g', 'LineWidth', 2);
stairs(t, Vc, 'b', 'LineWidth', 2);
ylabel('Phase Voltages');
xlabel('Time');
ylim([-1.5, 1.5]);
legend('Phase A', 'Phase B', 'Phase C');
grid on;
