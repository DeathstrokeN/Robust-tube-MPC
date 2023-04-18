% Define the system dynamics
A = [0.8, 0.1; 0.2, 0.9];
B = [0.2; 0.1];
C = [1, 1];

% Define the prediction horizon and control horizon
N = 10;
Nu = 2;

% Define the size of the system
n = size(A,1);
m = size(B,2);

% Define the initial conditions
x0 = [1; -1];

% Define the state and input constraints
x_min = [-10; -10];
x_max = [10; 10];
u_min = -1;
u_max = 1;

% Define the weighting matrices for the cost function
Q = eye(n);
R = eye(m);

% Define the uncertainty set
Delta_max = [0.2; 0.2];
D = [1, 0; 0, 1]; % assume a box uncertainty set

% Define the MPC controller
x_sim = zeros(n, N+1);
u_sim = zeros(m, N);
x_sim(:,1) = x0;
for k = 1:100
    % Compute the robust feasible set
    Delta = Delta_max;
    for j = 1:N
        x_next = A*x_sim(:,j) + B*u_sim(:,j) + Delta;
        x_min_j = max(x_min - x_next, zeros(n,1));
        x_max_j = max(x_next - x_max, zeros(n,1));
        Delta_j = [x_min_j, x_max_j] * D';
        Delta = max(Delta_j', [], 1)';
    end
    
    % Compute the robust control input
    u_opt = zeros(m,1);
    for j = 1:Nu
        u = u_min + (u_max - u_min)*rand;
        x_next = A*x_sim(:,1) + B*u + Delta;
        if all(x_min <= x_next) && all(x_next <= x_max)
            u_opt = u;
            break;
        end
    end
    
    % Simulate the system with disturbance
    x_sim(:,2:end) = A*x_sim(:,1:end-1) + B*u_sim + repmat([0.1*sin(k); 0.1*cos(k)], 1, N) + Delta;
    u_sim = [u_sim(:,2:end), u_opt];
    
    % Plot the results
    if mod(k,10) == 0
        t = 0:N;
        figure;
        subplot(2,1,1);
        plot(t, x_sim(1,:), 'b-', t, x_sim(2,:), 'r-');
        xlabel('Time step');
        ylabel('State');
        legend('x_1', 'x_2');
        subplot(2,1,2);
        plot(t, u_sim, 'g-');
        xlabel('Time step');
        ylabel('Control input');
        ylim([u_min-0.1, u_max+0.1]);
    end
end
