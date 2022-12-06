close all; clear; clc;

% Replace with your casadi path if not in MATLAB path
addpath('../casadi-matlab')

import casadi.*


params.Ts = 3.0; % Total simulation time
params.delta = 0.02; % Control sampling time
params.Tc = params.Ts; % Control horizon
params.N = params.Tc/params.delta;


m = 20;
I = 0.5;
g0 = 9.81;
W = 0.2;

z0 = 1;


%% MPC SETUP


x0 = [0.12; 0; z0; 0; zeros(2,1)];

% Declare model variables
px = SX.sym('px');
vx = SX.sym('vx');
pz = SX.sym('pz');
vz = SX.sym('vz');
theta = SX.sym('theta');
omega = SX.sym('omega');
x = [px; vx; pz; vz; theta; omega];


f1x = SX.sym('f1x');
f1z = SX.sym('f1z');
f2x = SX.sym('f2x');
f2z = SX.sym('f2z');
u = [f1x;f1z;f2x;f2z];



% Model equations
xdot = [vx; 
        (f1x+f2x)/m; 
        vz; 
        (f1z+f2z)/m - g0;
        omega
        (pz*(f1x+f2x) + (-W/2-px)*f1z + (W/2-px)*f2z)/I ];

nx = size(xdot,1);
nu = size(u,1);

% Continuous time dynamics
f = Function('f', {x, u}, {xdot});


dims.nx = nx;
dims.nu = nu;

F = discretize(f, params.delta, dims);


uref = SX.sym('uref', nu);
xref = SX.sym('xref', nx);

Wx = diag([10000 1 0.1 0.1 1 100]);

Wfx = 1;
Wfz = 0.1;

% Objective term
cost = Wfx*(u(1)^2 + u(3)^2) + Wfz*(u(2) + u(4) - m*g0)^2 +...
    (x-xref)'*Wx*(x-xref);

L = Function('L', {x, u, xref, uref}, {cost}, {'x','u', 'xref', 'uref'}, {'Lk'});



Uref = MX.sym('Uref', params.N*nu);
Xref = MX.sym('Xref', (params.N)*nx);

nlp_params = {Xref, Uref};


% Start with an empty NLP
w = {}; % Decision variables (x(0) u(0) x(1) u(1) ... x(N-1) u(N-1) x(N))
w0 = []; % Initial guess
lbw = []; % Lower and upper bounds for w: lbw <= w <= ubw
ubw = []; %
J = 0; % Cost function
g = {}; % Nonlinear constraints (dynamics)
lbg = []; % Lower and upper bound for g: lbg <= g(w) <= ubg
ubg = []; %

% "Lift" initial conditions
Xk = MX.sym('X_0', nx);
w = {w{:}, Xk};
lbw = [lbw; x0];
ubw = [ubw; x0];
w0 = [w0; x0];

% Formulate the NLP
for k=0:params.N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)], nu);
    w = {w{:}, Uk};
    lbw = [lbw; -inf; 1 ; -inf; 1]; % input constraints
    ubw = [ubw;  inf; inf; inf; inf];
    w0 = [w0;  0; m*g0/2; 0; m*g0/2]; % initial guess for input

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'u', Uk);
    Lk = L('x', Xk, 'u', Uk, 'xref', Xref(k*nx+1:(k+1)*nx), 'uref', Uref(k*nu+1:(k+1)*nu));
    Xk_end = Fk.xf;
    J=J+Lk.Lk;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], nx);
    w = [w, {Xk}];
    lbw = [lbw; [-inf ; -inf; 0.0; -inf; -1*pi/2; -inf] ];
    ubw = [ubw; [inf ; inf; inf ; inf; 1*pi/2; inf]];
    w0 = [w0; x0];

    % Add equality constraint
    g = [g, {Xk_end-Xk}];
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];
end

lbw(end-nx+1:end) = [0; 0; z0; 0; 0; 0;];
ubw(end-nx+1:end) = [0; 0; z0; 0; 0; 0;];

lbw(end-nx-nu+1:end-nx) = [0; m*g0/2; 0; m*g0/2];
ubw(end-nx-nu+1:end-nx) = [0; m*g0/2; 0; m*g0/2];


% Create an NLP solver
prob = struct('f', J,... % cost function
              'x', vertcat(w{:}),... % decision variables
              'g', vertcat(g{:}),... % constraints
              'p', vertcat(nlp_params{:}));


opts.ipopt.print_level = false;
opts.print_time = false;
% opts.jit = true;
opts.expand = true;
% opts.compiler = 'shell';
% opts.jit_options.flags = {'-O3 -march=native'};
% opts.jit_options.verbose = true;

solver = nlpsol('solver', 'ipopt', prob, opts);


tic


Sdes = reshape([zeros(2,params.N); z0*ones(1,params.N); zeros(3,params.N)], [nx*params.N, 1]);

Udes = zeros(nu*params.N,1);



% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
            'lbg', lbg, 'ubg', ubg, 'p', [Sdes; Udes]);

w_opt = full(sol.x);

toc

state_traj = [w_opt(1:(nx+nu):end)';
              w_opt(2:(nx+nu):end)';
              w_opt(3:(nx+nu):end)';
              w_opt(4:(nx+nu):end)';
              w_opt(5:(nx+nu):end)';
              w_opt(6:(nx+nu):end)'];

input_traj = [w_opt(7:(nx+nu):end)';
              w_opt(8:(nx+nu):end)';
              w_opt(9:(nx+nu):end)';
              w_opt(10:(nx+nu):end)'];



%%

srb_video(state_traj, input_traj, [], params, '')


%%
function F = discretize(f, delta, dims)
% Formulate discrete time dynamics and cost

% Fixed step Runge-Kutta 4 integrator
nx = dims.nx;
nu = dims.nu;
M = 10; % RK4 steps per interval
DT = delta/M;
X0 = casadi.MX.sym('X0', nx);
U = casadi.MX.sym('U', nu);


X = X0;
Q = 0;
for j=1:M
   [k1] = f(X, U);
   [k2] = f(X + DT/2 * k1, U);
   [k3] = f(X + DT/2 * k2, U);
   [k4] = f(X + DT * k3, U);
   X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
end



F = casadi.Function('F', {X0, U}, {X}, {'x0','u'}, {'xf'});

end