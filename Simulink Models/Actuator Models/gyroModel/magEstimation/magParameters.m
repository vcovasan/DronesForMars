time = linspace(0,8.9,890);
gyroData = [time' magReadings(:,1)];

bias = -0.85;
w = 12.5*2*pi;
wn = 1000*2*pi;
zeta = sqrt(2)/2;
z = zeta;
sensitivity = 0.00875;
power = 0.002;


realMagNoBias = magReadings(:,1) - mean(magReadings(:,1));
[retval, s, errorb, tau] = allan(time, realMagNoBias'*0.0092,0)