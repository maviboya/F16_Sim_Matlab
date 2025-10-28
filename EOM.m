function [states,dcm] = EOM(F,M,m,I,V,w,X,eul,h)

omega = [0 -w(3) w(2)
        w(3) 0 -w(1)
        -w(2) w(1) 0];

Vdot = F/m - omega*V;
wdot = I\(M - omega*I*w);

V = EulerIntegration(h,V,Vdot);
w = EulerIntegration(h,w,wdot);
dcm = angle2dcm( eul(3), eul(2), eul(1), 'ZYX' );

X = EulerIntegration(h,X,dcm*V);
euldot = [1 sin(eul(1))*tan(eul(2)) cos(eul(1))*tan(eul(2))
         0 cos(eul(1)) -sin(eul(1))
         0 sin(eul(1))*sec(eul(2)) cos(eul(1))*sec(eul(2))]*w;

eul = EulerIntegration(h,eul,euldot);




states = [V,w,X,eul];

end