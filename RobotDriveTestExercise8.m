% Parameters
d = .25; % wheel base (meters)
syms t
dilation = 5;
ellipse = sym([0.5*cos(t/5); .75*sin(t/5); 0]);
T = diff(ellipse);
MagT = norm(T);
That = T./MagT;
N = diff(That);
w = cross(That, N);
w = w(3);
% Send wheel speeds via ROS
vel = norm(diff(ellipse));
o = matlabFunction(w);
velocity = matlabFunction(vel);
pub = rospublisher('/raw_vel');
sub  = rossubscriber('/encoders');
msg = rosmessage(pub);
time = tic;
a = 1;
while a == 1
t = toc(time);
if t < 40
omega = o(t)% angular velocity (radians/second)
% velocity (meters/second)
test = velocity(t);
R = test/omega;
vl = test - omega*d/2 % left wheel speed (meters/second)
vr = test + omega*d/2 % right wheel speed (meters/second)
msg.Data = [vl, vr];
send(pub, msg);

else
a = 2;
end

end
msg.Data = [0, 0];
send(pub, msg);