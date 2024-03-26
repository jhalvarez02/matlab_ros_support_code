% Top-Down Pick
% Task: create a new .m file (script) called pickTopDownCan.m
% Expand code from section B. Have this new script call moveTopDownCan and then
% after that create another function that calls code from notebook
% 09_actions_UR_robot.mlx section 2.4 to close fingers around rCan3.
% Deliverables:
% o pickTopDownCan.m
% Inputs (none)
% Outputs: an array with the [x,y,z,r,p,y] of your end-effector at the time it
% touches the top of the can.
% You will upload/push pickTopDownCan.m to your own github repo. Follow
% the following steps to do so:
% 1. Tell git to state the file pickTopDownCan.m
% git add pickTopDownCan.m
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

tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation';
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);
show(UR5e,configSoln);
UR5econfig = [configSoln(3),configSoln(2),configSoln(1),configSoln(4), configSoln(5), configSoln(6)];
trajGoal = packTrajGoal(UR5econfig,trajGoal);
sendGoal(trajAct,trajGoal)

pause(2); 

%Can Position & Orientation
rCan3X = -0.032; 
rCan3Y = 0.805;
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

%continuation from part B, add following code to close the gripper on the
%can
%update coordinates of can for better precision as follows
% rCan3X = -0.032; 
% rCan3Y = 0.805;
% rCan3Z = 0.15;
% rCan3TopPosition = [rCan3X, rCan3Y, rCan3Z];
% rCan3TopRotation = [-pi/2 -pi 0];
% gripperTranslation = rCan3TopPosition; 
% gripperRotation = rCan3TopRotation; 
%pause(20) pause 20 seconds and wait for the robot to register its current
%postion relative to the world

%set up the gripper ros action client
%grip_Client = set up a rosactionclient for the gripper to send a close
%goal
%gripGoal = set a rosmessage to listen to the the grip_client
%gripPos = set the gripper position to 0.2 to attach to the can
%gripGoal = packGripGoal pack the goal information to to send to the server
%sendGoal(grip_client,gripGoal) = function sends the goal to the action server

pause(20);

%Gripper
grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory')
gripGoal = rosmessage(grip_client);
gripPos=0.2; 
gripGoal = packGripGoal(gripPos,gripGoal);
sendGoal(grip_client,gripGoal);


