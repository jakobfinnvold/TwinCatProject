clear; close all; clc; 
%Constants
x0 = 0;
v_0 = 0;
x_ref = 0.3;
v_ref = 0.2;
tRamp = 2;
tWait = 4;
t0 = 0;

x_SetPoint = x_ref - x0;
vs = v_ref; 
slopeExt = v_0-vs;
slopeRetr = -vs-v_0; 

as = vs/tRamp; % Find acceleration
s_acc = (vs^2-v_0^2)/as; % Acceleration sp

tHold = (x_SetPoint-s_acc)/vs;

if tHold < 0
    Err = 1;
else
    Err = 0;
end 

t1 = tRamp;
t2 = tHold;
t3 = tRamp;
t4 = tWait;
t5 = t1;
t6 = t2;
t7 = t3; 

% Ramp Equations:
x1 = x0 + v_0*((t0+t1)-t0) - (slopeExt/t1)*((t0+t1)-t0)^2/2; 
x2 = x1 + vs * ((t0+t1+t2)-(t0+t1)); 
x4 = x_ref - v_0 * ((t0+t1+t2+t3+t4+t5)-(t0+t1+t2+t3+t4)) + (slopeRetr/t5)*((t0+t1+t2+t3+t4+t5)-(t0+t1+t2+t3+t4))^2/2; 
x5 = x4 - vs*((t0+t1+t2+t3+t4+t5+t6)-(t0+t1+t2+t3+t4+t5)); 

if Err == 1
    x = x0;
    v = v_0;
elseif t >= 0 && t<t0
    x = x0;
    v = v_0;
elseif t >= t0 && t < (t0+t1)
    x = x0 + v_0*(t-t0) - (slopeExt/t1)*(t-t0)^2/2;
    v = v_0 - (slopeExt/t1)*(t-t0); 
elseif t >=(t0+t1) && t < (t0+t1+t2)
    x = x1 + vs*(t-(t0+t1));
    v = vs;
elseif t>=(t0+t1+t2) && t<(t0+t1+t2+t3)
    x = x2 + vs*(t-(t0+t1+t2)) + (slopeExt/t3)*(t-(t0+t1+t2))^2/2;
    v = vs + (slopeExt/t3)*(t-(t0+t1+t2)); 
elseif t >=(t0+t1+t2+t3) && t<(t0+t1+t2+t3+t4)
    x = x_ref;
    v = v_0;
elseif t>=(t0+t1+t2+t3+t4) && t < (t0+t1+t2+t3+t4+t5)
    x = x_ref - v_0*(t-(t0+t1+t2+t3+t4))+(slopeRetr/t5)*(t-(t0+t1+t2+t3+t4))^2/2;
    v = v_0 + (slopeRetr/t5)*(t-(t0+t1+t2+t3+t4)); 
elseif t>=(t0+t1+t2+t3+t4+t5) && t < (t0+t1+t2+t3+t4+t5+t6)
    x = x4 - vs*(t-(t0+t1+t2+t3+t4+t5));
    v = -vs;
elseif t >= (t0+t1+t2+t3+t4+t5+t6) && t < (t0+t1+t2+t3+t4+t5+t6+t7)
    x = x5 - vs*(t-(t0+t1+t2+t3+t4+t5+t6)) - (slopeRetr/t3)*(t-(t0+t1+t2+t3+t4+t5+t6))^2/2;
    v = -vs-(slopeRetr/t3)*(t-(t0+t1+t2+t3+t4+t5+t6));
else
    x = x0;
    v = v0;
end 

T = t0+t1+t2+t3+t4+t5+t6+t7; 