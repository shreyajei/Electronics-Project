clear all;
close('all')
ports = serialportlist;
pb = PyBench(ports(end));
model = IMU_3D();
N = 50;
tic;
gx = 0; gy =0; sf = 0.7; x_angle =0; y_angle =0;
fig1 = figure(1);
while true
    for i = 1:N
        [p,r] = pb.get_accel();
        [x,y,z] = pb.get_gyro();
        dt = toc;
        tic;
        pitch = p*180/pi;
        roll = r*180/pi;
        gx = max(min(gx+x*dt, pi/2),-pi/2);
        gy = max(min(gy+y*dt,pi/2),-pi/2);
        x_angle = sf *(x_angle+gx*dt)+ (1-sf)*r;
        y_angle = sf *(y_angle+gy*dt)+ (1-sf)*p;
        clf(fig1);
        subplot(3,1,1);
        model.draw(fig1 , p, r, 'Accelerometer');
        subplot(3,1,2);
        model.draw(fig1, gy, gx, 'Gyroscope');
        subplot(3,1,3);
        model.draw(fig1, y_angle, x_angle, 'complimentary filter');
        pause(0.1);
    end
end