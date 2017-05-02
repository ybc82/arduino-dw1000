function pf = pf_init_2D()

%% Initialization
% Constants
MAX_SPEED = 1;      % 1m/s
TIME_STEP = 0.2;    % 0.2s per cycle
anchor1 = [-1.7; 3.58];    % position of anchor K, [X; Y; Z] (m)
anchor2 = [-0.37; -3.85]; 
anchor3 = [1.143; 1.127];
% anchor1 = [2; 0; 0];
% anchor2 = [1; 0; 0];
% anchor3 = [3; 0; 0];

% anchor4 = [2; 1; 0];
% CHECK nv
pf.anchors = [anchor1, anchor2, anchor3];
N_PARTICLES = 200;  % number of particles
T = 1;              % number of time steps

% Process equation x[k] = sys(k, x[k-1], u[k])
nx = 2;     % number of states [x y]
sys = @(k, xkm1, uk) xkm1 + uk; % (returns column vector)

% Observation equation y[k] = obs(k, x[k], v[k])
pf.ny = 3;
pf.obs = @(k, xk, vk) sqrt(sum((pf.anchors - repmat(xk, [1 pf.ny])).^2, 1))';

% PDF of process noise and noise generator function
nu = 2;             % size of the vector of process noise
sigma_u = ones(nu, 1) * TIME_STEP * MAX_SPEED / 3;  % 10 cm for 3 sigma
p_sys_noise     = @(u) normpdf(u(1), 0, sigma_u(1))*normpdf(u(2), 0, sigma_u(2));
gen_sys_noise   = @(u) [normrnd(0, sigma_u(1)); normrnd(0, sigma_u(2))];     % sample from p_sys_noise (return column vector)

% PDF of observation noise and noise generator function
nv = 3;             % size of the vector of observation
pf.sigma_v = ones(nv, 1) * 0.3 / 3; % 10 cm for 3 sigma % CHANGE THIS BACK TO 0.1/3
pf.p_obs_noise     = @(v) normpdf(v(1), 0, pf.sigma_v(1)) * normpdf(v(2), 0, pf.sigma_v(2)) * normpdf(v(3), 0, pf.sigma_v(3));
gen_obs_noise   = @(v) normrnd(0, pf.sigma_v(1)) * normrnd(0, pf.sigma_v(2)) * normrnd(0, pf.sigma_v(3));     % sample from p_obs_noise (returns column vector)

% Initial PDF
% p_x0 = @(x) normpdf(x, 0, sqrt(10);
gen_x0 = @(x) normrnd(0, sqrt(10) * ones(nx, 1));

% Observation likelihood PDF p(y[k] | x[k])
p_yk_given_xk = @(k, yk, xk) pf.p_obs_noise(yk - pf.obs(k, xk, 0));

% memory space
x = zeros(nx, T);   % state
y = zeros(pf.ny, T);   % observation
u = zeros(nu, T);   % process noise
v = zeros(nv, T);   % observation noise

% state initialization
xh0 = zeros(nx, 1);            % initial state
u(:, 1) = zeros(nu, 1);
v(:, 1) = gen_obs_noise(pf.sigma_v);
x(:, 1) = xh0;
y(:, 1) = pf.obs(1, xh0, v(:, 1));

% particle filter
pf.k            = 2;                    % initial iteration number
pf.Ns           = N_PARTICLES;          % number of particles
pf.w            = zeros(pf.Ns, T);      % weights of particles
pf.particles    = zeros(nx, pf.Ns, T);  % particles
pf.gen_x0       = gen_x0;               % function for sampling from initial pdf p_x0
pf.p_yk_given_xk= p_yk_given_xk;        % function of the observation likelihood PDF p(y[k] | x[k])
pf.gen_sys_noise= gen_sys_noise;        % function for generating system noise
pf.sys          = sys;                  % process equation
