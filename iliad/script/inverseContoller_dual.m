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


distance_10 = 0.35;
dimBox=[0.38,0.18,0.14];
toSumChapa = [0;0;0.51];
toUR10e = [0.35;0.32+0.02;0.05];
base_ur_offset = [0;0;0.035];

boxPos_offset = [0;+dimBox(2)/2;-dimBox(3)/2];

boxPos =[1;0;0.65];     %pose in terna summit::base_footprint
boxZYX =[-pi/2;0;0];

sumPos = [0;0;0];
sumZYX = [0;0;0];


ZYX_d = [pi/2,0,0];

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

% Trasformo la posa da world frame a UR10e::base frame per l'IK
Point_d0 = boxtf_01*[[eul2rotm(ZYX_d,'ZYX')],[boxPos_offset(1);boxPos_offset(2);boxPos_offset(3)]; ...
                                0 0 0                               1           ];

Pose = inv(toUR10etf)*Point_d0;

poseConst.TargetTransform = Pose;

[configSoln,solnInfo] = gik(initialguess,poseConst);
close
show(ur10e,configSoln)

hold on

plot3(poseConst.TargetTransform(1,4),poseConst.TargetTransform(2,4),poseConst.TargetTransform(3,4),...
        '.','Color','b','MarkerSize',50)
box_center = inv(toUR10etf)*boxtf_01;
plot3(box_center(1,4),box_center(2,4),box_center(3,4),'.','Color','b','MarkerSize',50)

box_A = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[dimBox(1)/2,dimBox(2)/2,-dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_B = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[-dimBox(1)/2,dimBox(2)/2,-dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_C = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[-dimBox(1)/2,-dimBox(2)/2,-dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_D = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[dimBox(1)/2,-dimBox(2)/2,-dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
plot3(box_A(1,4),box_A(2,4),box_A(3,4),'.','Color','g','MarkerSize',50)
plot3(box_B(1,4),box_B(2,4),box_B(3,4),'.','Color','g','MarkerSize',50)
plot3(box_C(1,4),box_C(2,4),box_C(3,4),'.','Color','g','MarkerSize',50)
plot3(box_D(1,4),box_D(2,4),box_D(3,4),'.','Color','g','MarkerSize',50)

box_A2 = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[dimBox(1)/2,dimBox(2)/2,dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_B2 = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[-dimBox(1)/2,dimBox(2)/2,dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_C2 = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[-dimBox(1)/2,-dimBox(2)/2,dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_D2 = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[dimBox(1)/2,-dimBox(2)/2,dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
plot3(box_A2(1,4),box_A2(2,4),box_A2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_B2(1,4),box_B2(2,4),box_B2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_C2(1,4),box_C2(2,4),box_C2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_D2(1,4),box_D2(2,4),box_D2(3,4),'.','Color','g','MarkerSize',50)

box_ur1 = inv(toUR10etf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[0,-dimBox(2)/2,-dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
plot3(box_ur1(1,4),box_ur1(2,4),box_ur1(3,4),'.','Color','y','MarkerSize',50)




%%

franka = init_franka();

gik = generalizedInverseKinematics('RigidBodyTree', franka, ...
    'ConstraintInputs', {'pose'});

poseConst = constraintPoseTarget('hand_link');
poseConst.ReferenceBody='panda_link0';
poseConst.OrientationTolerance = 0;
poseConst.PositionTolerance = 0;
initialguess = [0;-0.7853;0;-2.35619;0;1.57079;0.785398];

dimBox=[0.38,0.18,0.14];
toSumChapa = [0;0;0.51];
toFranka = [0.25;-0.22;0.15];

boxtf_01 = [[eul2rotm(boxZYX', 'ZYX')],boxPos; ...
            0 0 0                               1           ];


boxPos_offset = [0;dimBox(2)/2-0.3;+0.3+dimBox(3)/2];
% boxPos_offset = [0;dimBox(2)/2;+dimBox(3)/2+0.05];
% boxPos_offset = [0;-dimBox(2)/2-0.05;+dimBox(3)/2+0.05];
boxPos_offset = [0;+dimBox(2)/2-0.06;dimBox(3)/2+0.015];

% Calcolo il pto di arrivo in relazione alla posa del box, trasformando gli
% offset dalla terna body in terna world
boxPos0 = boxtf_01*[boxPos_offset;1];

% Trasformazione world -> summit::base_footprint
sumtf_01 = [[eul2rotm(sumZYX', 'ZYX')],sumPos; ...
            0 0 0                               1           ];

% Trasformazione summit::base_footprint -> panda::base
sumtopandatf_12 = [[eul2rotm([0,pi/4,0], 'ZYX')],toSumChapa+toFranka; ...
            0 0 0                               1           ];

% Trasformazione world -> panda::base
toPandatf = sumtf_01*sumtopandatf_12;


boxtf_01 = [[eul2rotm(boxZYX', 'ZYX')],boxPos; ...
            0 0 0                               1  ];

boxPose2 = boxtf_01*[[eul2rotm([pi+pi/2,-pi/8,0],'ZYX')],[boxPos_offset(1);boxPos_offset(2);boxPos_offset(3)]; ...
                                0 0 0                               1           ];

poseConst.TargetTransform = inv(toPandatf)*boxPose2;

[configSoln,solnInfo] = gik(initialguess,poseConst);
%figure
show(franka,configSoln)
hold on

plot3(poseConst.TargetTransform(1,4),poseConst.TargetTransform(2,4),poseConst.TargetTransform(3,4),...
        '.','Color','b','MarkerSize',50)
box_center = inv(toPandatf)*boxtf_01;
plot3(box_center(1,4),box_center(2,4),box_center(3,4),'.','Color','b','MarkerSize',50)

box_A = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[dimBox(1)/2,dimBox(2)/2,-dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_B = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[-dimBox(1)/2,dimBox(2)/2,-dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_C = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[-dimBox(1)/2,-dimBox(2)/2,-dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_D = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[dimBox(1)/2,-dimBox(2)/2,-dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
plot3(box_A(1,4),box_A(2,4),box_A(3,4),'.','Color','g','MarkerSize',50)
plot3(box_B(1,4),box_B(2,4),box_B(3,4),'.','Color','g','MarkerSize',50)
plot3(box_C(1,4),box_C(2,4),box_C(3,4),'.','Color','g','MarkerSize',50)
plot3(box_D(1,4),box_D(2,4),box_D(3,4),'.','Color','g','MarkerSize',50)

box_A2 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[dimBox(1)/2,dimBox(2)/2,dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_B2 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[-dimBox(1)/2,dimBox(2)/2,dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_C2 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[-dimBox(1)/2,-dimBox(2)/2,dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
box_D2 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[dimBox(1)/2,-dimBox(2)/2,dimBox(3)/2]'; ...
                                            0 0 0                        1           ];
plot3(box_A2(1,4),box_A2(2,4),box_A2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_B2(1,4),box_B2(2,4),box_B2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_C2(1,4),box_C2(2,4),box_C2(3,4),'.','Color','g','MarkerSize',50)
plot3(box_D2(1,4),box_D2(2,4),box_D2(3,4),'.','Color','g','MarkerSize',50)

box_ur1 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[0;dimBox(2)/2;+dimBox(3)/2+0.05]; ...
                                            0 0 0                        1           ];
plot3(box_ur1(1,4),box_ur1(2,4),box_ur1(3,4),'.','Color','y','MarkerSize',50)
box_ur1 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[0;-dimBox(2)/2-0.05;+dimBox(3)/2+0.05]; ...
                                            0 0 0                        1           ];
plot3(box_ur1(1,4),box_ur1(2,4),box_ur1(3,4),'.','Color','y','MarkerSize',50)
box_ur1 = inv(toPandatf)*boxtf_01*[[eul2rotm([0,0,0],'ZYX')],[0;+dimBox(2)/2-0.06;dimBox(3)/2+0.015]; ...
                                            0 0 0                        1           ];
plot3(box_ur1(1,4),box_ur1(2,4),box_ur1(3,4),'.','Color','y','MarkerSize',50)
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