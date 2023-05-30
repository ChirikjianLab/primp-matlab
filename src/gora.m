classdef gora < handle
    % Globally Optimal Reparameterization Algorithm (GORA) for SE(3) 
    % trajectory
    %
    % This code implements the Theorem 1 in the paper by:
    % 1. Calculating the difference between adjacent frames of a sequence
    % 2. Globally minimizing the functional of the form
    %     ``J = \int_0^1 g(\tau) \dot{\tau} dt''
    % 3. Reparameterizing the sequence with the optimal time steps
    %
    % Input:
    %   tau  : original temporal parameterizations
    %   g_init    : signal sequence
    %   t_opt: time steps for globally optimal temporal reparameterizations
    %
    % Authors:
    %   Sipu Ruan, 2023
    %
    % Earlier version collaborated with Thomas Mitchel

    properties
        g_init
        n_step_init
        t0

        n_step
        tau_opt
        g_opt
    end

    methods
        function obj = gora(g_init, n_step)
            if nargin == 1
                n_step = 50;
            end

            obj.g_init = g_init;

            obj.n_step_init = size(g_init, 3);
            obj.t0 = linspace(0, 1.0, obj.n_step_init);

            obj.n_step = n_step;
            obj.tau_opt = linspace(0, 1.0, obj.n_step);
            obj.g_opt = zeros(4, 4, obj.n_step);
        end

        function run(obj)
            % Run main routines

            g = obj.g_tau();
            obj.temporalReparam(g);
            obj.g_opt = interp_SE3_traj(obj.t0, obj.g_init, obj.tau_opt);
        end

        function cost = get_cost_functional(obj, tau, g_tau)
            % Compute cost functional
            %
            % Inputs
            %   tau  : Parameterization of the trajectory
            %   g_tau: SE(3) trajectory parameterized by tau
            %
            % Outputs
            %   cost : Cost functional value

            d_tau = gradient(tau, obj.t0);
            cost = trapz(d_tau^2 * g_tau, obj.t0);
        end

        function g = g_tau(obj)
            % Generate g(\tau) for the input of Euler-Lagrange equation

            dg = df_vect(obj.t0, obj.g_init, 1, 2, 3);
            g = zeros(obj.n_step_init, 1);

            for i = 1:obj.n_step_init
                g(i) = g(i) + norm( get_exp_coord(obj.g_init(:,:,i) \...
                    dg(:,:,i)) )^2;
            end
        end

        function temporalReparam(obj, g)
            % Compute the globally optimal time sequence by Theorem 1

            F = cumtrapz(obj.t0, g.^(1/2));
            F = unique( F/F(end) );

            tau = 0:1/(length(F)-1):1;
            obj.tau_opt = interp1(F, tau, obj.tau_opt, 'pchip');
        end
    end

end