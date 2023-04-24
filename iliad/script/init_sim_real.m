clear 
clc

franka = init_franka();
% show(franka,[0,0,0,0,0,0,0]')

ur10e = init_ur10e();
show(ur10e)

fprintf("Robot models loaded. \n");

fprintf("Ready to run simulation: \n" + ...
        "first run 'roslaunch iliad sim.launch' from terminal...\n" + ...
        "then run simulation in 'controller_stateflow.slx   \n");

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
tform = [[eul2rotm([-pi/6,0,0], 'XYZ')],[0,0.07,0.14+0.04]'; ...
                    0 0 0                     1   ];
addVisual(ur10e.Bodies{10},"Mesh",'gazebo_assieme.stl',tform)

velvet_link_center = rigidBody('velvet_link_center');
jnt = rigidBodyJoint('velv_cent_jnt','fixed');
tform = [[eul2rotm([0 pi/6 0], 'xyz')],[0.20+0.04,0,0.05-sin(pi/6)*0.2]'; ...
                      0 0 0                     1   ];
setFixedTransform(jnt,tform);
velvet_link_center.Joint = jnt;
addBody(ur10e,velvet_link_center,'ee_link')

velvet_link_end= rigidBody('velvet_link_end');
jnt = rigidBodyJoint('velv_end_jnt','fixed');
tform = [[eul2rotm([0 pi/6 0], 'xyz')],[0.28+0.04,0,0.05-sin(pi/6)*0.28]'; ...
                      0 0 0                     1   ];
setFixedTransform(jnt,tform);
velvet_link_end.Joint = jnt;
addBody(ur10e,velvet_link_end,'ee_link')
end