try
    device = serialport("COM15",115200);
catch
    device.delete
    device = serialport("COM15",115200);
end
pause(1);
device.flush();
while(device.NumBytesAvailable);
   buffer = readline(device);
   fprintf(buffer);
end
commands = struct('target',1.3,'Kp',4000,'Ki',0,'Kd',0,'runtime',3)
device.writeline(jsonencode(commands));
pause(1);
data = struct('time',0,'deg',0,'error',0,'pwm_theoretical',0,'pwm_actual',0,'dir',0);
i = 0;
while(device.NumBytesAvailable)
    i = i+1;
    buffer = readline(device);
    try
        json_data = jsondecode(buffer);
        data(i) = json_data;
    catch
        warning("Data not in proper json format");
        error = buffer
    end
    if (device.NumBytesAvailable == 0)
        pause(0.1);
    end
end
device.delete;
length(data)

xtimes=[];
yerror=[];
ydir=[];
ypwm=[];
for i=1:length(data)
    xtimes(i)=(data(i).time)/(10^9);
    yerror(i)=(data(i).error)/180;
    ydir(i)=data(i).dir;
    ypwm(i)=(data(i).pwm_actual)/65535;
end
figure; hold on
line1 = plot(xtimes,yerror,'-ro'); line1_label = "error %";
line2 = plot(xtimes,ydir,'-g'); line2_label = "direction";
line3 = plot(xtimes,ypwm,'-bx'); line3_label = "pwm %";
xlabel('time (s)')
legend([line1,line2,line3],[line1_label,line2_label,line3_label]);
hold off