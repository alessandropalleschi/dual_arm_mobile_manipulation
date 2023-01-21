%% UR10e Robot definition with velvet

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

% show(franka,[0,0,0,0,0,0,0]')

handDim = [0.2,0.0,0.0];

%% Declaration of box dimensions and positions
boxDim = [0.38,0.18,0.14];

boxPos = [0.6,0.20,0.2];
boxZYX = [0,0,0];

task_vol = collisionBox(0.18,0.38,0.14);
tform = [[eul2rotm(boxZYX, 'ZYX')],boxPos'; ...
                    0 0 0             1   ];
task_vol.Pose = tform;


%%


gik = generalizedInverseKinematics('RigidBodyTree', franka, ...
    'ConstraintInputs', {'pose'});

poseConst = constraintPoseTarget('hand_link');
poseConst.TargetTransform= [1     0    0    boxPos(1)-handDim(3)-boxDim(2)/2
                            0     1    0    boxPos(2)
                            0     0    1    boxPos(3)+handDim(2)/2+boxDim(3)/2
                            0     0    0    1];

poseConst.TargetTransform = [[eul2rotm([pi,-pi/4,0], 'ZYX')], [boxPos(1)+boxDim(2)/2;boxPos(2);boxPos(3)+handDim(2)/2+boxDim(3)/2]; ...
                                        0 0 0                                  1   ];
% 
% T1 = [[eul2rotm([-pi/4,0,0], 'ZYX')],[0,0,0]'; ...
%                     0 0 0             1   ];
% pp=[-0.0,0,0,1]';
% 
% pp1=T1*pp
% 
% T2 = [[eul2rotm([0,0,0], 'ZYX')],[pp1(1:3)]; ...
%                     0 0 0             1   ];
% 
% 
% poseConst.TargetTransform = poseConst.TargetTransform*T2; % assi correnti



poseConst.ReferenceBody='panda_link0';
poseConst.OrientationTolerance = 0;
poseConst.PositionTolerance = 0;

initialguess = franka.homeConfiguration;



[configSoln,solnInfo] = gik(initialguess,poseConst);



show(franka,configSoln,'Visuals','on');
hold on
drawnow

[~,B] = show(task_vol);
B.FaceColor = 'blue';
B.FaceAlpha = 0.1;
drawnow

