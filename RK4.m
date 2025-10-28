function [t, y] = RK4(f, t0, y0, tf, h)
    % f: function handle for the derivative (dy/dt = f(t, y))
    % t0: initial time
    % y0: initial value of y
    % tf: final time
    % h: step size

    % Initialize time and solution arrays
    t = t0:h:tf; 
    n = length(t);
    y = zeros(1, n);
    y(1) = y0;

    % Runge-Kutta 4th order integration
    for i = 1:n-1
        k1 = h * f(t(i), y(i));
        k2 = h * f(t(i) + h/2, y(i) + k1/2);
        k3 = h * f(t(i) + h/2, y(i) + k2/2);
        k4 = h * f(t(i) + h, y(i) + k3);
        
        y(i+1) = y(i) + (k1 + 2*k2 + 2*k3 + k4) / 6;
    end
end