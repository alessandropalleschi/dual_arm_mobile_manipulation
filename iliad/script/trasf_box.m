

boxPos = [0;2;0.8;1];
% boxZYX = [0;0;0]; %euler ZYX
boxZYX = [pi/2;0;0]; %euler ZYX


boxtf_01 = [[eul2rotm(boxZYX', 'ZYX')],boxPos(1:3); ...
            0 0 0                               1   ];


body_or = boxtf_01*[0,0,0,1]';
body_x = boxtf_01*[1,0,0,1]';
body_y = boxtf_01*[0,1,0,1]';
body_z = boxtf_01*[0,0,1,1]';

ptoA = [0.10;-0.1;+0.1;1];
ptoA_rspBox = boxtf_01 * ptoA;
plot3(ptoA_rspBox(1),ptoA_rspBox(2),ptoA_rspBox(3),'.','LineWidth',5) %%pto in coordinate world rispetto il box
% boxx = inv(toPandatf)*boxPos;


%%
figure;hold on;
grid on;
xlabel('X axis'), ylabel('Y axis'), zlabel('Z axis')
axis ([-5,5,-5,5,-5,5])
set(gca,'CameraPosition',[1 2 3]);


fixed_x = [1,0,0];
fixed_y = [0,1,0];
fixed_z = [0,0,1];
fixed_or = [0,0,0];
plot3([fixed_or(1) fixed_x(1)],[fixed_or(2) fixed_x(2)],[fixed_or(3) fixed_x(3)],'r--', 'LineWidth',1);
plot3([fixed_or(1) fixed_y(1)],[fixed_or(2) fixed_y(2)],[fixed_or(3) fixed_y(3)],'g--', 'LineWidth',1);
plot3([fixed_or(1) fixed_z(1)],[fixed_or(2) fixed_z(2)],[fixed_or(3) fixed_z(3)],'b--', 'LineWidth',1);

plot3([body_or(1) body_x(1)],[body_or(2) body_x(2)],[body_or(3) body_x(3)],'r-', 'LineWidth',1);
plot3([body_or(1) body_y(1)],[body_or(2) body_y(2)],[body_or(3) body_y(3)],'g-', 'LineWidth',1);
plot3([body_or(1) body_z(1)],[body_or(2) body_z(2)],[body_or(3) body_z(3)],'b-', 'LineWidth',1);

% plot3([body_or(1) boxPos(1)],[body_or(2) boxPos(2)],[body_or(3) boxPos(3)],'y-', 'LineWidth',1);
% plot3(boxPos(1),boxPos(2),boxPos(3),'.','LineWidth',5)
% plot3([fixed_or(1) boxx(1)],[fixed_or(2) boxx(2)],[fixed_or(3) boxx(3)],'y-', 'LineWidth',1);
% plot3(boxx(1),boxx(2),boxx(3),'.','LineWidth',5)





