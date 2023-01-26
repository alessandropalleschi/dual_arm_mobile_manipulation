clc

franka = init_franka();
% show(franka,[0,0,0,0,0,0,0]')

ur10e = init_ur10e();
%show(ur10e)

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
end

function ur10e = init_ur10e()
ur10e= loadrobot("universalUR10",'DataFormat','column');
tform = [[eul2rotm([0,0,0], 'XYZ')],[0,0,0.14]'; ...
                    0 0 0                     1   ];
addVisual(ur10e.Bodies{10},"Mesh",'gazebo_assieme.stl',tform)

end