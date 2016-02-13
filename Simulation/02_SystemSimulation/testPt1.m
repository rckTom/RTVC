Kp = 1;
T = 1;


dt = 0.001;
tmax = 20;

solver = class_rk4(dt);
y = 1;
x(1) = 0;
for i = 1:tmax/dt
  xdot = @(x)(Kp/T*y-1/T*x);
  [x(i+1),xd(i+1)] = solver.solve(xdot,x(i));
end

plot(x)