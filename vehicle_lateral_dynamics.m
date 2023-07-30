function xup=vehicle_lateral_dynamics(x,u,Ts,vx)
    lf=1.2;
    lr=1.6;
    m = 1575;
    Iz = 2875;
    Cf = 29500;
    Cr = 31000;
    A=[-2*(Cf+Cr)/(m*vx) 0 -vx+2*(Cr*lr-Cf*lf)/(m*vx) 0
        0                0      1                     0
        2*(Cr*lr-Cf*lf)/(Iz*vx) 0 -2*(Cr*lr^2+Cf*lf^2)/(Iz*vx) 0
        1                vx     0                      0];
    B=[2*Cf/m 0 2*lf*Cf/Iz 0]';
    xdot=A*x+B*u;
    xup=x+xdot*Ts;
end