clear all
close('all')
ports = serialportlist;
pb = PyBench(ports(end));
N = 500;
end_time = 10.0;
gx =0; gy = 0;
while true
    figure(1)
    clf(1)
    axis([0 end_time -90 90]);
    title('Gyroscope Pitch & Roll Angles', 'FontSize', 16);
    ylabel(' Angles (deg)' ,'FontSize', 14);
    xlabel ('Tme (sec)','FontSize' , 14);
    grid on; hold on; %makes grid, hold on doesn't erase old value
    timestamp=0;
    tic;
    for i = 1:N
        [x,y,z] = pb.get_gyro();
        dt =toc;
        tic;
        timestamp = timestamp +dt;
        gx = max(min(gx+x*dt, pi/2),-pi/2);
        gy = max(min(gy+y*dt,pi/2),-pi/2);
           plot(timestamp , gy*180/pi, '.b');
        plot(timestamp, gx*180/pi,'.r');
        pause(0.001);
    end
    end_time = timestamp;
end
