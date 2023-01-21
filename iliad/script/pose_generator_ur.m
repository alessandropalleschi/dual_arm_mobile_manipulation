%% UR10e Robot definition with velvet

ur10e= loadrobot("universalUR10",'DataFormat','column');
%%removeBody(ur10e,'tool0');
 tform = [[eul2rotm([0,0,0], 'XYZ')],[0,0,0.14]'; ...
                    0 0 0                     1   ];
 addVisual(ur10e.Bodies{10},"Mesh",'gazebo_assieme.stl',tform)

 VelvetDim = [0.14,0.08,0.28];
%show(ur10e)
%% Declaration of box dimensions and positions
boxDim = [0.38,0.18,0.14];

boxPos = [1,-0.1,0.2];
boxZYX = [0,0,0];

task_vol = collisionBox(0.18,0.38,0.14);
tform = [[eul2rotm(boxZYX, 'ZYX')],boxPos'; ...
                    0 0 0             1   ];
task_vol.Pose = tform;


%%


gik = generalizedInverseKinematics('RigidBodyTree', ur10e, ...
    'ConstraintInputs', {'pose'});

poseConst = constraintPoseTarget('ee_link');
poseConst.TargetTransform= [1     0    0    boxPos(1)-VelvetDim(3)-boxDim(2)/2
                            0     1    0    boxPos(2)
                            0     0    1    boxPos(3)-VelvetDim(2)/2-boxDim(3)/2
                            0     0    0    1];
poseConst.ReferenceBody='base_link';
poseConst.OrientationTolerance = 0;
poseConst.PositionTolerance = 0;

initialguess = ur10e.homeConfiguration;



[configSoln,solnInfo] = gik(initialguess,poseConst);



show(ur10e,configSoln,'Visuals','on');
hold on
drawnow

[~,B] = show(task_vol);
B.FaceColor = 'blue';
B.FaceAlpha = 0.1;
drawnow

