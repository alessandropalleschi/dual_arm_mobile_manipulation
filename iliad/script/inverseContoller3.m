clc
%%

ur10e = init_ur10e();
show(ur10e)
%%
gik = generalizedInverseKinematics('RigidBodyTree', ur10e, ...
    'ConstraintInputs', {'pose'});

poseConst = constraintPoseTarget('velvet_link_end');
poseConst.ReferenceBody='base_link';
poseConst.OrientationTolerance = 0;
poseConst.PositionTolerance = 0;
initialguess =   [ -0.6417    0.6650   -2.4016    1.7425    0.9312   -3.1416         ]';

distance_10 = 0.2;
dimBox   = [0.2;0.2;0.13]; %like a box but we know it's a cilinder
dimboxUp = [0.23;0.23;0.03]; 
box_center = [0.115;0.115;0.065]; %to mach the origin of stl with real 3D origin of the cilinder

toSumChapa = [0;0;0.51];
toUR10e = [0.35;0.32+0.02;0.05];
velvet_length = 0.28;
velvet_height = 0.08;
base_ur_offset = [0;0;0.035];

boxPos =[0.503;-1.33;0.513];
boxZYX =[0.0421;0.00208;0.000768];

sumPos = [0.551;0.194;0];
sumZYX = [-1.56;0.00042;0];

boxPos_offset = [0;dimBox(2)/2;-dimBox(3)/2]+box_center;
ZYX_d = [-pi/2,0,0];
ZYX_d = [-pi/2,0,0];
distance = [[eul2rotm([0,0,0],'ZYX')],[0,0,0]'; ...
                       0 0 0              1   ];

% trasformazione world -> box::frame
boxtf_01 = [[eul2rotm(boxZYX', 'ZYX')],boxPos; ...
            0 0 0                               1           ];

% Trasformazione world -> summit::base_footprint
sumtf_01 = [[eul2rotm(sumZYX', 'ZYX')],sumPos; ...
            0 0 0                               1           ];
% Trasformazione summit::base_footprint -> UR10e::base
sumtoUR10etf_12 = [[eul2rotm([0,0,0], 'ZYX')],toSumChapa+toUR10e+base_ur_offset; ...
            0 0 0                               1           ];
% Trasformazione world -> UR10e::base
toUR10etf = sumtf_01*sumtoUR10etf_12;

% Trasformo la posa da world frame a UR10e::base_frame per l'IK
Point_d0 = boxtf_01*[[eul2rotm(ZYX_d,'ZYX')],[boxPos_offset(1);boxPos_offset(2);boxPos_offset(3)]; ...
                                0 0 0                               1           ];

Pose = distance*inv(toUR10etf)*Point_d0;

poseConst.TargetTransform = Pose;

[configSoln,solnInfo] = gik(initialguess,poseConst);
close
show(ur10e,configSoln)

hold on

plot3(poseConst.TargetTransform(1,4),poseConst.TargetTransform(2,4),poseConst.TargetTransform(3,4),...
        '.','Color','b','MarkerSize',50)

box_centerT = inv(toUR10etf)*boxtf_01*[[eul2rotm(ZYX_d,'ZYX')],[box_center(1);box_center(2);box_center(3)]; ...
                                                 0 0 0                               1           ];

plot3(box_centerT(1,4),box_centerT(2,4),box_centerT(3,4),'.','Color','b','MarkerSize',50)

pp = [dimBox(1)/2,dimBox(2)/2,-dimBox(3)/2]'+box_center;
box_A = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [-dimBox(1)/2,dimBox(2)/2,-dimBox(3)/2]'+box_center;
box_B = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [-dimBox(1)/2,-dimBox(2)/2,-dimBox(3)/2]'+box_center;
box_C = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [dimBox(1)/2,-dimBox(2)/2,-dimBox(3)/2]'+box_center;
box_D = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
plot3(box_A(1,4),box_A(2,4),box_A(3,4),'.','Color','g','MarkerSize',50)
plot3(box_B(1,4),box_B(2,4),box_B(3,4),'.','Color','g','MarkerSize',50)
plot3(box_C(1,4),box_C(2,4),box_C(3,4),'.','Color','g','MarkerSize',50)
plot3(box_D(1,4),box_D(2,4),box_D(3,4),'.','Color','g','MarkerSize',50)

pp = [dimBox(1)/2,dimBox(2)/2,dimBox(3)/2]'+box_center;
box_A2 = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [-dimBox(1)/2,dimBox(2)/2,dimBox(3)/2]'+box_center;
box_B2 = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [-dimBox(1)/2,-dimBox(2)/2,dimBox(3)/2]'+box_center;
box_C2 = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [dimBox(1)/2,-dimBox(2)/2,dimBox(3)/2]'+box_center;
box_D2 = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
plot3(box_A2(1,4),box_A2(2,4),box_A2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_B2(1,4),box_B2(2,4),box_B2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_C2(1,4),box_C2(2,4),box_C2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_D2(1,4),box_D2(2,4),box_D2(3,4),'.','Color','g','MarkerSize',50)


%%

franka = init_franka();
% show(franka,[0,0,0,0,0,0,0]')
%%

gik = generalizedInverseKinematics('RigidBodyTree', franka, ...
    'ConstraintInputs', {'pose'});

poseConst = constraintPoseTarget('hand_link_end');
poseConst.ReferenceBody='panda_link0';
poseConst.OrientationTolerance = 0;
poseConst.PositionTolerance = 0;
initialguess = [0;-0.7853;0;-2.35619;0;1.57079;0.785398];

toSumChapa = [0;0;0.51];
toFranka = [0.25;-0.22;0.15];

sumPos = [0.551;0.194;0];
boxPos_offset = [0;0;0]+box_center;
ZYX_d = [pi+pi/2,-pi/2,0];
ZYX_d = [pi/2,-pi/16,0];

% Trasformazione world -> box::frame
boxtf_01 = [[eul2rotm(boxZYX', 'ZYX')],boxPos; ...
        0 0 0                               1           ];
% Trasformazione world -> summit::base_footprint
sumtf_01 = [[eul2rotm(sumZYX', 'ZYX')],sumPos; ...
            0 0 0                               1           ];

% Trasformazione summit::base_footprint -> panda::base
sumtopandatf_12 = [[eul2rotm([0,pi/4,0], 'ZYX')],toSumChapa+toFranka; ...
            0 0 0                               1           ];

% Trasformazione world -> panda::base
toPandatf = sumtf_01*sumtopandatf_12;

% Trasformo la posizione da world frame a panda::base frame per l'IK
Pose_d0 = boxtf_01*[[eul2rotm(ZYX_d,'ZYX')],[boxPos_offset(1);boxPos_offset(2);boxPos_offset(3)]; ...
                            0 0 0                               1           ];

distance = [[eul2rotm([0,0,0],'ZYX')],[-0.1,0,0]'; ...
                       0 0 0              1   ];

Pose = distance*inv(toPandatf)*Pose_d0;

poseConst.TargetTransform = Pose;

[configSoln,solnInfo] = gik(initialguess,poseConst);
figure
show(franka,configSoln)
hold on

plot3(poseConst.TargetTransform(1,4),poseConst.TargetTransform(2,4),poseConst.TargetTransform(3,4),...
        '.','Color','b','MarkerSize',50)

box_centerT = inv(toPandatf)*boxtf_01*[[eul2rotm(ZYX_d,'ZYX')],[box_center(1);box_center(2);box_center(3)]; ...
                                                 0 0 0                               1           ];

plot3(box_centerT(1,4),box_centerT(2,4),box_centerT(3,4),'.','Color','b','MarkerSize',50)

pp = [dimBox(1)/2,dimBox(2)/2,-dimBox(3)/2]'+box_center;
box_A = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [-dimBox(1)/2,dimBox(2)/2,-dimBox(3)/2]'+box_center;
box_B = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [-dimBox(1)/2,-dimBox(2)/2,-dimBox(3)/2]'+box_center;
box_C = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [dimBox(1)/2,-dimBox(2)/2,-dimBox(3)/2]'+box_center;
box_D = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
plot3(box_A(1,4),box_A(2,4),box_A(3,4),'.','Color','g','MarkerSize',50)
plot3(box_B(1,4),box_B(2,4),box_B(3,4),'.','Color','g','MarkerSize',50)
plot3(box_C(1,4),box_C(2,4),box_C(3,4),'.','Color','g','MarkerSize',50)
plot3(box_D(1,4),box_D(2,4),box_D(3,4),'.','Color','g','MarkerSize',50)

pp = [dimBox(1)/2,dimBox(2)/2,dimBox(3)/2]'+box_center;
box_A2 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [-dimBox(1)/2,dimBox(2)/2,dimBox(3)/2]'+box_center;
box_B2 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [-dimBox(1)/2,-dimBox(2)/2,dimBox(3)/2]'+box_center;
box_C2 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
pp = [dimBox(1)/2,-dimBox(2)/2,dimBox(3)/2]'+box_center;
box_D2 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],pp; ...
                                            0 0 0                        1           ];
plot3(box_A2(1,4),box_A2(2,4),box_A2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_B2(1,4),box_B2(2,4),box_B2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_C2(1,4),box_C2(2,4),box_C2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_D2(1,4),box_D2(2,4),box_D2(3,4),'.','Color','g','MarkerSize',50)
%%
%-------------------------------------------------------------------------%


function franka = init_franka()

franka = loadrobot("frankaEmikaPanda",'DataFormat','column');
removeBody(franka,'panda_hand');
tform = [[eul2rotm([pi 0 5*pi/4], 'xyz')],[0.091*0.707+0.05 -0.091*0.707-0.05 0.05]'; ...
                      0 0 0                     1   ];
addVisual(franka.Bodies{8},"Mesh",'hand_new.stl',tform)

hand_link = rigidBody('hand_link');
jnt = rigidBodyJoint('handjnt','fixed');
tform = [[eul2rotm([pi 0 5*pi/4], 'xyz')],[0.091*0.707+0.05 -0.091*0.707-0.05 0.05]'; ...
                      0 0 0                     1   ];
setFixedTransform(jnt,tform);
hand_link.Joint = jnt;
addBody(franka,hand_link,'panda_link8')

hand_link_end = rigidBody('hand_link_end');
jnt = rigidBodyJoint('hand_end_jnt','fixed');
tform = [[eul2rotm([pi 0 5*pi/4], 'xyz')],[(0.091+0.16)*0.707+0.05 -(0.091+0.16)*0.707-0.05 0.07]'; ...
                      0 0 0                     1   ];
setFixedTransform(jnt,tform);
hand_link_end.Joint = jnt;
addBody(franka,hand_link_end,'panda_link8')
end

function ur10e = init_ur10e()
ur10e= loadrobot("universalUR10",'DataFormat','column');
tform = [[eul2rotm([0,0,0], 'XYZ')],[0,0,0.14+0.04]'; ...
                    0 0 0                     1   ];
addVisual(ur10e.Bodies{10},"Mesh",'gazebo_assieme.stl',tform)

velvet_link_center = rigidBody('velvet_link_center');
jnt = rigidBodyJoint('velv_cent_jnt','fixed');
tform = [[eul2rotm([0 0 0], 'xyz')],[0.20+0.04,0,0.05]'; ...
                      0 0 0                     1   ];
setFixedTransform(jnt,tform);
velvet_link_center.Joint = jnt;
addBody(ur10e,velvet_link_center,'ee_link')

velvet_link_end= rigidBody('velvet_link_end');
jnt = rigidBodyJoint('velv_end_jnt','fixed');
tform = [[eul2rotm([0 0 0], 'xyz')],[0.28+0.04,0,0.05]'; ...
                      0 0 0                     1   ];
setFixedTransform(jnt,tform);
velvet_link_end.Joint = jnt;
addBody(ur10e,velvet_link_end,'ee_link')
end