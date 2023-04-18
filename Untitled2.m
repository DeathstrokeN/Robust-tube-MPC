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

% Compute terminal weight matrix
P = dare(A,B,Q,R);

% Define initial states and setpoints
x0 = [0; 0];
r = [2; 2];

% Simulate MPC controller
Tsim = 5;
T = 0:Ts:Tsim;
x = x0;
u = zeros(1, length(T)-1);
y = zeros(1, length(T));
for i = 1:length(T)-1
    % Compute predicted states and outputs
    x_pred = zeros(size(A,1), N+1);
    y_pred = zeros(size(C,1), N+1);
    x_pred(:,1) = x;
    y_pred(:,1) = C*x;
    for j = 1:N
        x_pred(:,j+1) = A*x_pred(:,j) + B*u(i+j-1);
        y_pred(:,j+1) = C*x_pred(:,j+1);
    end
    
    % Compute predicted cost function
    J_pred = 0;
    for j = 1:N
        J_pred = J_pred + y_pred(:,j)'*Q*y_pred(:,j) + u(i+j-1)'*R*u(i+j-1);
    end
    J_pred = J_pred + y_pred(:,N+1)'*P*y_pred(:,N+1);
    
    % Compute control action
    options = optimoptions('quadprog', 'Display', 'off');
    u_opt = quadprog(2*R, -2*B'*P*x_pred(:,2), [], [], [], [], [], [], [], options);
    u(i) = u_opt(1);
    
    % Simulate system and update states
    y(i) = C*x;
    x = A*x + B*u(i);
end

% Plot results
figure;
subplot(2,1,1);
plot(T(1:end-1), u);
xlabel('Time (s)');
ylabel('Control input');
title('MPC control input');
subplot(2,1,2);
plot(T, [y' r']);
xlabel('Time (s)');
ylabel('Output');
title('MPC output and setpoint');
legend('Output', 'Setpoint');
