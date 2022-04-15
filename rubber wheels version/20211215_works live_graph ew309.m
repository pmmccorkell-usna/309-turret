% # Patrick McCorkell
% # November 2021
% # US Naval Academy
% # Robotics and Control TSD
% #

%16V: 2000 8000 200  32000

classdef ew309 < matlab.mixin.SetGet
	properties(GetAccess='public', SetAccess='public')
		com_port
		com_port_available
		serial_device
		target
		Kp
		Ki
		Kd
		runtime
		min_drive
		data
	end

	methods(Access='public')
		% absolute necessary args in: COM port, target location
		% optional args in: Kp, Ki, Kd, runtime, min_drive
		function obj=ew309(varargin)
			obj.com_port = string("COM"+string(varargin{1}));

			% obj.init_serial();
			obj.com_port_available = 0;
			obj.Kp = 4000;
			obj.Ki = 0;
			obj.Kd = 0;
			obj.runtime = 2.0;
			obj.min_drive = 20000;

			obj.is_port_available();
			if (obj.com_port_available)
				obj.init_serial();
			else
				fprintf("Serial Port " + obj.com_port + " not available.\nClear port and try again manually\n");
			end

		end % end function

		function delete(obj)
			obj.deinit_serial();
		end % end function

		function deinit_serial(obj)
			% fclose(obj.serial_device)
			obj.serial_device = 0
	 	end % end function

		function is_port_available(obj)
			obj.com_port_available = 0;
			available_ports = serialportlist("available");
			for i=1:length(available_ports)
				if (available_ports(i) == obj.com_port)
					obj.com_port_available = 1;
				end
			end
		end % end function

		function init_serial(obj)
			obj.serial_device = serialport(obj.com_port,115200);
			pause(0.5);
			while(obj.serial_device.NumBytesAvailable)
				buffer = readline(serial.device);
				fprintf(buffer);
			end
			obj.serial_device.flush();
		end % end function

		% obj.send_commands(target,runtime,Kp,Ki,Kd,min_drive)
		% At least 'target' must be sent as an argument.
		% Optional arguments: runtime, Kp, Ki, Kd, min_drive
		function send_commands(obj,varargin)
			obj.target = varargin{1};
			if nargin > 2
				obj.runtime = varargin{2};
			end

			if nargin >3
				obj.Kp = varargin{3};
			end

			if nargin > 4
				obj.Ki = varargin{4};
			end

			if nargin > 5
				obj.Kd = varargin{5};
			end

			if nargin > 6
				obj.min_drive = varargin{6};
			end

			obj.serial_device.flush();
			pause(0.1)
			commands = struct('target',obj.target,'Kp',obj.Kp,'Ki',obj.Ki,'Kd',obj.Kd,'runtime',obj.runtime,'min_drive',obj.min_drive)
			obj.serial_device.writeline(jsonencode(commands));

			obj.read_serial();
			disp(obj.data);
			% obj.plot_data();
            last_error = obj.data(end).error;
            disp(last_error);
            
		end % end function
		
		function read_serial(obj)
			obj.data = struct('time',0,'deg',0,'error',0,'pwm_theoretical',0,'pwm_actual',0,'dir',0);
			i=0;
			pause(0.1);
            
   			xtimes=[];
			yerror=[];
			ydir=[];
			ypwm=[];

			figure;
			xlabel('time (s)');

			while(obj.serial_device.NumBytesAvailable)
				i = i+1;
				buffer = readline(obj.serial_device);

				try
					json_data = jsondecode(buffer);
					obj.data(i) = json_data;
				catch
					warning("Data not in proper json format");
					error = buffer
                end

                xtimes(i)=(obj.data(i).time)/(10^9);
                yerror(i)=(obj.data(i).error) / 1.8;
                ydir(i)= (obj.data(i).dir)*100;
                ypwm(i) = (obj.data(i).pwm_actual) / 655.35;

                hold on
                line1 = plot(xtimes(end),yerror(end),'-rx');
                line1_label = "error %";
                line2 = plot(xtimes(end),ydir(end),'-gx'); line2_label = "direction";
                line3 = plot(xtimes(end),ypwm(end),'-bx'); line3_label = "pwm %";
                legend([line1,line2,line3],[line1_label,line2_label,line3_label]);
                hold off
                drawnow;

                % Make sure it's really 0 and stays 0.
                if (obj.serial_device.NumBytesAvailable==0)
                    pause(0.1);
                end

			end % end while
		end % end function        
        
		function plot_data(obj)
			xtimes=[];
			yerror=[];
			ydir=[];
			ypwm=[];
			for i=1:length(obj.data)
				xtimes(i)=(obj.data(i).time)/(10^9);
				yerror(i)=(obj.data(i).error) / 1.8;
				ydir(i)=obj.data(i).dir*100;
				ypwm(i) = (obj.data(i).pwm_actual) / 655.35;
            end

			figure; hold on;
            if length(data) < 50
			    line1 = plot(xtimes,yerror,'-ro'); line1_label = "error %";
			    line2 = plot(xtimes,ydir,'-g'); line2_label = "direction";
			    line3 = plot(xtimes,ypwm,'-bx'); line3_label = "pwm %";
            end
			xlabel('time (s)');
			legend([line1,line2,line3],[line1_label,line2_label,line3_label]);
			hold off;

		end % end function

	end % end methods

end % end class