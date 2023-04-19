% Define system model
A = [1.1 0.2; 0.1 0.9];
B = [1; 0];
C = [1 1];
D = 0;
Ts = 0.1;

% Define prediction horizon and control horizon
N = 10;
M = 2;

% Define weight matrices
Q = diag([1 1]);
R = 1;

% Define terminal constraint and weight matrices
x_lb = [-1; -1];
x_ub = [1; 1];
P = eye(2);
Q_f = P;

% Define uncertainty set and disturbance
Delta = [0.1 0; 0 0.1];
d = [0; 0];

% Define state and control bounds
x_min = [-10; -10];
x_max = [10; 10];
u_min = -10;
u_max = 10;

% Define initial states and setpoints
x0 = [0; 0];
r = [2; 2];

% Pre-allocate matrices for closed-loop simulation
Nsim = 50;
x_cl = zeros(2, Nsim+1);
u_cl = zeros(1, Nsim);
y_cl = zeros(1, Nsim);
x_cl(:, 1) = x0;

% Loop through prediction horizon
for i = 1:Nsim
    % Formulate optimization problem
    H = R + B'*(P + Delta)*B;
    f = -2*r'*(P + Delta)*B*A;
    Aeq = [eye(2) -B; C 0];
    beq = [A*x_cl(:, i) + d; r];
    LB = [x_min; u_min];
    UB = [x_max; u_max];
    
    % Solve optimization problem using quadprog
    options = optimoptions('quadprog', 'Display', 'off');
    [u_opt, ~, exitflag] = quadprog(H, f, [], [], Aeq, beq, LB, UB, [], options);
    
    % Check if optimization problem was solved successfully
    if exitflag ~= 1
        error('Optimization problem not solved successfully.');
    end
    
    % Apply control action and simulate system
    u_cl(i) = u_opt(1);
    y_cl(i) = C*x_cl(:, i);
    x_cl(:, i+1) = A*x_cl(:, i) + B*u_cl(i) + d;
end

% Plot results
Tsim = Ts*(0:Nsim);
figure;
subplot(2,1,1);
plot(Tsim, u_cl);
xlabel('Time (s)');
ylabel('Control input');
title('Robust Tube MPC control input');
subplot(2,1,2);
plot(Tsim, [y_cl' r(1)*ones(1, Nsim)]);
hold on;
xline(Tsim(end), '--');
xlabel('Time (s)');
ylabel('Output');
title('Robust Tube MPC output and setpoint');
legend('Output', 'Setpoint', 'End of simulation');

