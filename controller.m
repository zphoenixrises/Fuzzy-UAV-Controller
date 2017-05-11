function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
persistent  phi_c_old;

if isempty(phi_c_old)
    phi_c_old = 0;
end
u1 = 0;
u2 = 0;
m = params.mass;
g = params.gravity;

kp_z =470;
kv_z =.15*2*sqrt(kp_z);

kp_y = 35;
kv_y =.95*2*sqrt(kp_y);

kp_phi =500;
kv_phi =.6*2*sqrt(kp_phi);
dt = 0.01;
z_ddot_des = des_state.acc(2);
y_ddot_des = des_state.acc(1);

z_dot_des = des_state.vel(2);
y_dot_des = des_state.vel(1);
z_dot = state.vel(2);
y_dot = state.vel(1);

z_des = des_state.pos(2);
y_des = des_state.pos(1);
z = state.pos(2);
y = state.pos(1);
Ixx = params.Ixx;
phi = state.rot;
phi_dot = state.omega;




phi_c = (-1/g)*(y_ddot_des+kv_y*(y_dot_des - y_dot)+kp_y*(y_des-y));
phi_ddot_c = 0;
phi_dot_c = (phi_c-phi_c_old)/dt;
phi_c_old = phi_c;
phi_dot_c = 0;
u1 = m*(g + z_ddot_des + kv_z*(z_dot_des-z_dot) + kp_z*(z_des-z));
u2 = Ixx*(phi_ddot_c + kv_phi*(phi_dot_c-phi_dot) + kp_phi*(phi_c-phi));


end

