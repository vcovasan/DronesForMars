a = arduino('/dev/cu.usbmodem14201', 'Uno', 'Libraries', 'I2C');
fs = 100;
imu = mpu9250(a,'SampleRate',fs,'OutputFormat','matrix');


tic;
stopTimer = 100;
newmagReadings=[];
newaccReadings=[];
newgyroReadings=[];

while(toc<stopTimer)
    % Rotate the sensor around x axis from 0 to 360 degree.
    % Take 2-3 rotations to improve accuracy.
    % For other axes, rotate around that axis.
    [acc,gyro,mag] = read(imu);
    newaccReadings = [newaccReadings;acc];
    newmagReadings = [newmagReadings;mag];
    newgyroReadings = [newgyroReadings;gyro];
end

% time = linspace(0,8.9,890);
% gyroData = [time' magReadings(:,1)];
% 
% bias = mean(magReadings(:,2))*0.00875;

%q = compFilt(accReadings, newgyroReadings);
