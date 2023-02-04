clear
clc

step = 10;
tfin=2*pi;
k_tq=tfin/step;
tq= 0:k_tq:tfin;

tx = 0:pi/4:2*pi;
vx = [0,0,0,0,0,0,0,0,0];

ty = 0:pi/4:2*pi;
vy = cos(ty);

tz = [0,0.5,1,1.5,2,2.5,3];
vz = [0,0.2,0.4,0.6,0.8,1,1.2];

figure
vxq1 = interp1(tx,vx,tq,'linear');
plot(tx,vx,'o',tq,vxq1,':.');
xlim([0 tfin]);
title('(Default) Linear Interpolation');

figure
vyq1 = interp1(ty,vy,tq,'cubic');
plot(ty,vy,'o',tq,vyq1,':.');
xlim([0 tfin]);
title('(Default) Linear Interpolation');

% figure
% plot(vx,vy,'o',vxq1,vyq1,':.');

figure
vzq1 = interp1(tz,vz,tq,'linear');
plot(tz,vz,'o',tq,vzq1,':.');
xlim([0 tfin]);
title('(Default) Linear Interpolation');


%%


plot3(vxq1,vyq1,vzq1)
xlabel('X') 
ylabel('y') 
zlabel('Z') 
