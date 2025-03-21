function [flight_time, X, Y, Z] = Flight_sim(Xo,Yo,Zo, motor, moment_arm)
%FLIGHT_SIM uses the physical
%   Detailed explanation goes here

% Initial Conditions
m = 0.452
n = 1;
dt = 0.1;
X(n) = Xo;
Y(n) = Yo;
Z(n) = Zo;
% dX(n) = 0;
% dY(n) = 0;
dZ(n) = 0;
T(n) = 0;


while Z(n) > 0
current_thrust = thrust(motor, T)
fnet = forces(current_thrust, 0.75, m, dZ)
dZ(n+1) = dZ(n) + (-9.81 + (fnet/m)) *dt
Z(n+1) = Z(n) + W(n+1)*dt
T(n+1) = T(n) + dt;
n = n + 1;
end

end % end of main function


%% FINDS CURRENT THRUST
function thrust = thrust(motor, T)
    % defining the motor times and thrust
    times = motor.time
    thrusts = motor.thrust

    abs_time_diff = abs(T(end) - times); % represents the absolute difference in values of times, the minimum of this should be the index of the thrust
    current_thrust_index = find(abs_time_diff == min(abs_time_diff)  ); % finds the indices of time which are closest to the current time
    current_thrust_index = current_thrust_index(1); 
    thrust = thrusts(current_thrust_index)
end

function fnet = forces(thrust, Cd, mass, speed)
    Fgravity = mass*9.81;
    Fdrag = 0.5*1.225*((speed(end))^2)*Cd*0.00486451275   %1/2 * air density * velocity^2 * Coeff drag( assumed .75 ) * cross-section area
    Fmotor = motor
    Fnet = Fmotor - Fdrag - Fmotor;
end


function TVC = TVC(servo_angle, moment_arm, Thrust)
end
