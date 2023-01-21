
ur10e= loadrobot("universalUR10",'DataFormat','column');
%%removeBody(ur10e,'tool0');
 tform = [[eul2rotm([0,0,0], 'XYZ')],[0,0,0.14]'; ...
                    0 0 0                     1   ];
 addVisual(ur10e.Bodies{10},"Mesh",'gazebo_assieme.stl',tform)
%show(ur10e)

gik = generalizedInverseKinematics('RigidBodyTree', ur10e, ...
    'ConstraintInputs', {'pose'});

poseConst = constraintPoseTarget('ee_link');
poseConst.TargetTransform= [1     0    0    0.5
                            0     1    0    -0.22
                            0     0    1    -0.1
                            0     0    0    1];
poseConst.ReferenceBody='base_link';
poseConst.OrientationTolerance = 0;
poseConst.PositionTolerance = 0;

initialguess = ur10e.homeConfiguration;

[configSoln,solnInfo] = gik(initialguess,poseConst);
show(ur10e,configSoln,'Visuals','on');
hold on
drawnow

task_vol = collisionBox(0.18,0.38,0.14);
tform = trvec2tform([0.6,-0.22,0]);
task_vol.Pose = tform;
[~,B] = show(task_vol);
B.FaceColor = 'blue';
B.FaceAlpha = 0.1;
drawnow

%%
franka = loadrobot("frankaEmikaPanda",'DataFormat','column');
removeBody(franka,'panda_hand');
 tform = [[eul2rotm([pi 0 5*pi/4], 'xyz')],[0.091*0.707+0.05 -0.091*0.707-0.05 0.05]'; ...
                    0 0 0                     1   ];
 addVisual(franka.Bodies{8},"Mesh",'hand_new.stl',tform)

%show(franka,[0,0,0,0,0,0,0]')


%%

p=[0.25;-0.22;0.15];

phi = pi/4;

Ty = [cos(phi) 0 -sin(phi);
         0     1    0     ;
      sin(phi) 0 cos(phi)];

p1 = Ty*p


tform = [[eul2rotm([0,pi/4,0], 'XYZ')],[0.25,-0.22,0.15]'; ...
            0 0 0                               1           ]

%%
TT = [[eul2rotm([0,pi/4,0], 'XYZ')]',[0 0 0]'; ...
            0 0 0                               1           ];
pos_goal = [1     0    0    0.8
            0    -1    0    0.2
            0     0   -1    0.0
            0     0    0    1.0000];

pos_goal = TT*pos_goal;


gik = generalizedInverseKinematics('RigidBodyTree', franka, ...
    'ConstraintInputs', {'pose'});

poseConst = constraintPoseTarget('panda_link8');
poseConst.TargetTransform= pos_goal;
poseConst.ReferenceBody='panda_link0';
poseConst.OrientationTolerance = 0;
poseConst.PositionTolerance = 0;

initialguess = franka.homeConfiguration;


    [configSoln,solnInfo] = gik(initialguess,poseConst);
    show(franka,configSoln,'Visuals','on');
    hold on
    drawnow

    %%

    show(franka,out.panda_config.signals.values(:,:,20))
