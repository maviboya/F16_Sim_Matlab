function stateNext = RK4(derivative, time, state, dt, varargin)
%RK4 Klasik dördüncü dereceden Runge-Kutta yöntemiyle bir zaman adımı.
%   stateNext = RK4(derivative,time,state,dt)
%   stateNext = RK4(derivative,time,state,dt,extra1,extra2,...)
% dx/dt = derivative(t,x,...) başlangıç değer problemini t'den t+dt'ye taşır.
% time: mevcut zaman; state: skaler veya vektör durum; dt: zaman adımı.
% derivative, state ile aynı boyutta durum türevi döndüren fonksiyon tutamacıdır.
% İsteğe bağlı ek girdiler her türev çağrısına aynı sırayla aktarılır.
% k1..k4 durum türevleridir; dt çarpanı son ağırlıklı toplamda uygulanır.
% Örnek: xNext = RK4(@(t,x) -2*x,0,1,0.01);
% Zaman döngüsü ve modele özgü normalizasyon/sınırlar çağırana aittir.

% Başlangıç, iki orta nokta ve adım sonundaki eğimler.
k1 = derivative(time,state,varargin{:});
k2 = derivative(time+dt/2,state+dt*k1/2,varargin{:});
k3 = derivative(time+dt/2,state+dt*k2/2,varargin{:});
k4 = derivative(time+dt,state+dt*k3,varargin{:});
stateNext = state+dt*(k1+2*k2+2*k3+k4)/6;
end
