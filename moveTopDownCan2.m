% B. Top-Down Approach
% Task: create a new .m file (script) called moveTopDownCan.m
% Expand code from section A. This m-file will execute 2 independent motions. The
% first one is to move with a constant height directly on top of the rCan3. The second
% will produce a straight top-down move such that the end-effector hugs the top of
% rCan3 ready to pick it up.
% Deliverables:
% moveTopDownCan.m
% Inputs (none)
% Outputs: an array with the [x,y,z,r,p,y] of your end-effector at the time it
% touches the top of the can.
% You will upload/push moveTopDownCan.m to your own github repo. Follow
% the following steps to do so:
% 1. Tell git to state the file moveTopDownCan.m
% git add moveTopDownCan.m
% 2. Commit the changes you have made to this file to history via the commit
% command:
% git commit -m “enter some descriptive message here about changes you
% made. I.e. created moveToCan.m to reach top of can. ”
% 3. Send this commit to your yours repository in the cloud:
% git push yours main

clc
clear

%Reset
goHome('qr');
pause(2); 
resetWorld;
pause(2);  

%Gripper
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory')
gripGoal = rosmessage(grip_client);
gripPos=0; 
gripGoal = packGripGoal(gripPos,gripGoal);
sendGoal(grip_client,gripGoal);

%Arm
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory');
trajGoal= rosmessage(trajAct);
trajAct.FeedbackFcn = [];
jointSub = rossubscriber("/joint_states");
jointStateMsg = jointSub.LatestMessage;
UR5e = loadrobot('universalUR5e', DataFormat="row");
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;   
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0])); % 3 is shoulder pan joint
tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0])); % 4 is shoulder lift joint 
tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0])); % 7 is wrist 2 joint 
ik = inverseKinematics("RigidBodyTree",UR5e);
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1];
jointStateMsg = receive(jointSub,3);
initialIKGuess = homeConfiguration(UR5e);
jointStateMsg.Name;
initialIKGuess(1) = jointStateMsg.Position(4); 
initialIKGuess(2) = jointStateMsg.Position(3);
initialIKGuess(3) = jointStateMsg.Position(1);
initialIKGuess(4) = jointStateMsg.Position(5);
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);
show(UR5e,initialIKGuess);

%Can Position & Orientation
rCan3X = -0.04;
rCan3Y = 0.8;
rCan3Z = 0.25;
rCan3TopPosition = [rCan3X, rCan3Y, rCan3Z];
rCan3TopRotation = [-pi/2 -pi 0];
gripperTranslation = rCan3TopPosition; 
gripperRotation = rCan3TopRotation; 

%move the arm to hover over the red can position
%can coordinates are as follows
% rCan3X = -0.04;
% rCan3Y = 0.8;
% rCan3Z = 0.25;
%gripperTranslation = set the end effector pose(gripper) in position to
%equivalent to the can
%gripperRotation = set the end effector pose to the orientation of the can relative to the world frame

tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation';
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);
show(UR5e,configSoln);
UR5econfig = [configSoln(3),configSoln(2),configSoln(1),configSoln(4), configSoln(5), configSoln(6)];
trajGoal = packTrajGoal(UR5econfig,trajGoal);
sendGoal(trajAct,trajGoal)

%compute the homogeneous transformation of the gripper via tform (rotation
%and translation
%[configSoln, solnInfo] = compute the inverse kinematics
%Next send the trajectory goal 
%UR5e config = comply from the matlab structure to the ROS structure. 
%note the order of the joints used for the inverse kinematics
%pause(2)= pause for 2 seconds and wait for the gazebo model
%the gazebo robot should now be hovering over the can in a ready postion to
%drop

pause(2); 

%now we are ready to lower the robot arm to "hug" the gripper around the
%can
%update the coordinate of the can to lower the gripper in a ready position
%to grab, the following steps are repeated but note the change of
%coordiantes
% rCan3X = -0.02;
% rCan3Y = 0.8;
% rCan3Z = 0.15;
%gripperTranslation = set the end effector pose(gripper) in position to
%equivalent to the can
%gripperRotation = set the end effector pose to the orientation of the can relative to the world frame

%repeat the inverse kinmatics for the robot arm to lower via the updated
%joint angles
%[configSoln, solnInfo] = compute the inverse kinematics
%Next send the trajectory goal 
%UR5e config = comply from the matlab structure to the ROS structure. 
%note the order of the joints used for the inverse kinematics
%pause(2)= pause for 2 seconds and wait for the gazebo model
%the gazebo robot should now be hugging the can


%Can Position & Orientation
rCan3X = -0.02;
rCan3Y = 0.8;
rCan3Z = 0.15;
rCan3TopPosition = [rCan3X, rCan3Y, rCan3Z];
rCan3TopRotation = [-pi/2 -pi 0];
gripperTranslation = rCan3TopPosition; 
gripperRotation = rCan3TopRotation; 

tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation';
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);
show(UR5e,configSoln);
UR5econfig = [configSoln(3),configSoln(2),configSoln(1),configSoln(4), configSoln(5), configSoln(6)];
trajGoal = packTrajGoal(UR5econfig,trajGoal);
sendGoal(trajAct,trajGoal)
