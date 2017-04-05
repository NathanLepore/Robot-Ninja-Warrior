% Parameters
d = .25; % wheel base (meters)
syms t 
dilation = 5;
a = 0.4;
l = 0.4;
%Our cardioid equation
cardioid = sym([-2*a*(((l-cos(t/dilation))*cos(t/dilation) + (1-l))); 2*a*(((l-cos(t/dilation))*sin(t/dilation))); 0]);
%Finding the That vector
T = diff(cardioid);
MagT = norm(T);
That = T./MagT;
%Calculating the normal vector
N = diff(That);
%Calculating angular velocity
w = cross(That, N);
w = w(3);

%calculating total velocity
vel = norm(diff(cardioid));
o = matlabFunction(w);
velocity = matlabFunction(vel);
%Making our ros objects 
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
%time object
time = tic;
run = 1;

while run == 1
%define t as our current time
t = toc(time);
    if t < 19
        omega = o(t);% angular velocity (radians/second)
        test = velocity(t); % velocity (meters/second)
        vl = test - omega*d/2 % left wheel speed (meters/second)
        vr = test + omega*d/2 % right wheel speed (meters/second)
        msg.Data = [vl, vr];
        send(pub, msg);

    else
        run = 2;
    end

end
msg.Data = [0, 0];
send(pub, msg);