%% Establish Connection (run this section only once)
% Change the COM port to be the one that your device attaches to the
% Octopus Board

COMPort = "COM4";

% If you get an error saying that it was unable to connect to the serial
% port, first unplug and replug the USB. Try again. If that doesn't work,
% cycle power on the board. 

ser = serialport(COMPort, 250000);
fopen(ser);

%% Send move command (run this as many times as needed)
% This moves the actuator to an absolute position. If you send the same
% comment twise, the actuator will not move the second time because it is
% already at the requested position

% To move 1 mm in X, Y, or Z, you must tell it to move 16 units. The
% following command would move the outermost tube (X) to 1 mm ahead of the
% home position:

% fprintf(ser, 'G0 X16 Y0 Z0 A0 B0 C0\n');

fprintf(ser, 'G0 X0 Y0 Z0 A0 B0 C0\n');

%% Set the current position to home (run as many times as needed)
% This resets the "home" position to the given position. Pass all 0s as
% shown below to set the current position as home

fprintf(ser, 'G92 X0 Y0 Z0 A0 B0 C0\n');

%% Close the serial connection (run once at the end)
fclose(ser)