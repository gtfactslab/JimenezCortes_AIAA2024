function MODEL = CONFIG_MODEL(SAMPLING_TIME)
%--------------------------------------------------------------------------
% GENERAL PARAMETERS
%--------------------------------------------------------------------------
% Control sampling time (s)
MODEL.PARAM.SAMPLING_TIME = SAMPLING_TIME;
% Simulation sampling time (s)
MODEL.PARAM.SIM_SAMPLING_TIME = SAMPLING_TIME;
% Simulation final time (s)
MODEL.PARAM.SIM_FINAL_TIME = 600;
MODEL.PARAM.FLIGHT_STATUS = 0;
MODEL.PARAM.SWTCH_FLAG = 0;
%--------------------------------------------------------------------------
% SECOND ORDER UNICYCLE PARAMETERS
%--------------------------------------------------------------------------
% Displaced point distance (m)
MODEL.PARAM.DISP_P = 0.5;

%--------------------------------------------------------------------------
% MODEL INPUT
%--------------------------------------------------------------------------
% Virtual Controller Input Tau1
MODEL.INPUT.TAU1 = 0;
% Virtual Controller Input Tau2
MODEL.INPUT.TAU2 = 0;
% Aircraft Altitude (feet)
MODEL.INPUT.ALTITUDE = 0;
% Aircraft Latitude (rad)
MODEL.INPUT.LATITUDE = 0;
% Aircraft Longitude (rad)
MODEL.INPUT.LONGITUDE = 0;

%--------------------------------------------------------------------------
% MODEL OUTPUT
%--------------------------------------------------------------------------
% Aircraft Altitude (feet)
MODEL.OUTPUT.ALTITUDE = 10e3;
% Aircraft Latitude (rad)
% MODEL.OUTPUT.LATITUDE = 0;
MODEL.OUTPUT.LATITUDE = 5.89e-01;
% Aircraft Longitude (rad)
% MODEL.OUTPUT.LONGITUDE = 0;
MODEL.OUTPUT.LONGITUDE = -1.47;
% Aircraft pitch (radians)
MODEL.OUTPUT.PITCH = pi*0;
% Center of Mass Position x1
% MODEL.OUTPUT.X1 = 30;
MODEL.OUTPUT.X1 = 1.7497e+06;
% Center of Mass Position x2
% MODEL.OUTPUT.X2 = -5; 
MODEL.OUTPUT.X2 = -1.7300e+07;

%--------------------------------------------------------------------------
% STATES
%--------------------------------------------------------------------------
% Displaced Point Dynamics Y = {y1, y2, vy1, vy2} - 1 TO 4
% Unicycle Dynamics X = {s, theta, omega} - 5 TO 7
% y1 = STATE(1)
% y2 = STATE(2)
% vy1 = STATE(3)
% vy2 = STATE(4)
% s = STATE(5)
% theta = STATE(6)
% omega = STATE(7)
MODEL.STATE = zeros(7,1);
MODEL.STATE(5) = -1.5*0;
MODEL.STATE(6) = 0;
MODEL.STATE(7) = 0;
MODEL.STATE(1) = MODEL.OUTPUT.X1 + MODEL.PARAM.DISP_P*cos(MODEL.STATE(6));
MODEL.STATE(2) = MODEL.OUTPUT.X2 + MODEL.PARAM.DISP_P*sin(MODEL.STATE(6));
MODEL.STATE(3) =  MODEL.STATE(5)*cos(MODEL.STATE(6)) - MODEL.PARAM.DISP_P*MODEL.STATE(7)*sin(MODEL.STATE(6));
MODEL.STATE(4) =  MODEL.STATE(5)*sin(MODEL.STATE(6)) + MODEL.PARAM.DISP_P*MODEL.STATE(7)*cos(MODEL.STATE(6));

% Output x = sqrt(x1^2 + x2^2)
MODEL.OUTPUT.X = sqrt(MODEL.OUTPUT.X1^2 + MODEL.OUTPUT.X2^2);
% Output y = sqrt(y1^2 + y2^2)
MODEL.OUTPUT.Y = sqrt(MODEL.STATE(1)^2 + MODEL.STATE(2)^2);
return