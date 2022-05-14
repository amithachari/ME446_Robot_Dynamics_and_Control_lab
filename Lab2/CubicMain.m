clear

global a1 b1 b2 a2 c1 c2 d1 d2 modeflag k
a1 = -1;
b1 = 1.5;
c1 = 0;
d1 = 0;

a2 = 1;
b2 = -4.5;
c2 = 6;
d2 = -2;
modeflag = 0;
k = 0;

t = 0:1:4001;
thetas = zeros(3,4001);
for i = 1:2
    for j = 0:2000
        k = k +1;
        if j == 1000
            modeflag = 1;
        end
        thetas(:,k) = cubicfunction(j/1000);
    end
    modeflag = 0;
end


plot(t, thetas(1,:))
hold on
plot(t, thetas(2,:))
plot(t, thetas(3,:))
hold off
function thetaarray = cubicfunction(t)
global a1 b1 b2 a2 c1 c2 d1 d2 modeflag


if (modeflag == 0)
    theta = a1*t^3+b1*t^2+c1*t+d1;
    theta_dot = 3*a1*t^2+2*b1*t+c1;
    theta_dotdot = 6*a1*t+2*b1;
elseif (modeflag == 1)
    theta = a2*t^3+b2*t^2+c2*t+d2;
    theta_dot = 3*a2*t^2+2*b2*t+c2;
    theta_dotdot = 6*a2*t+2*b2;
    
else
    theta = 0;
    theta_dot = 0;
    theta_dotdot = 0;
end
thetaarray = [theta; theta_dot; theta_dotdot];
end
