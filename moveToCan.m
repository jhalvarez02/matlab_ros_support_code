clc
clear

% Go home ('qr') - Reset the robot to the ready position
goHome('qr');
% Wait for the robot to finish going home
pause(5); 
% Reset the world
resetWorld;
% Wait for the world to be reset
pause(5);

%goHome('qr') to reset the robot to the ready position
%pause(5) pause for 5 seconds to wait for the robot to finish positioning
%resetWorld to reset position of all objects in the world
%pause(5) to wait for the world to reset

grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory')
gripGoal = rosmessage(grip_client);
gripPos=0.8; 
gripGoal = packGripGoal(gripPos,gripGoal);
sendGoal(grip_client,gripGoal);

%grip_Client = set up a rosactionclient for the gripper to send a close or
%open goal
%gripGoal = set a rosmessage to listen to the the grip_client
%gripPos = set the gripper position to 0.8 to fully close the gripper
%gripGoal = packGripGoal pack the goal information to to send to the server
%sendGoal = function sends the goal to the action server

trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory');
trajGoal= rosmessage(trajAct);
trajAct.FeedbackFcn = [];
jointSub = rossubscriber("/joint_states");
jointStateMsg = jointSub.LatestMessage;

%trajAct = set up a rosaction client for the arm to send a position goal
%for the arm
%trajGoal = set a rosmessage to listen to the trajAct
%trajAct.FeedbackFcn = feedback function for the action server for the arm
%jointSub = create a joint subscriber to listen on the joint_states topic
%jointStateMsg = store the lastest message for the subscriber to listen to

UR5e = loadrobot('universalUR5e', DataFormat="row");
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;   
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0])); % 3 is shoulder pan joint
tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0])); % 4 is shoulder lift joint 
tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0])); % 7 is wrist 2 joint 

%UR5e load the UR5e robot in matlab to obtain joint angles
%we need to set the UR5e to the same position as the gazebo model for
%forward kinematics
%to do this we need to set an inverse kinematics solver obtain the joint
%angles
%first set the forward kinematics of the UR5e model to the gazebo model via
%tform
%adjust the UR5e shoulder pan joint{3} to a pi/2 rad rotation in the x
%adjust the UR5e shoulder lift joint{4) to -pi/2 rad rotation in the x 
%adjust the UR5e wrist 2 joint{7} to -pi/2 rad rotation in the x
%ik = set the numerical inverse kinematics solver
%ikWeights = set the ik solver weights
%JointStateMsg = recieve the latest message of the joint angles
%initialIKGuess = set the initial joint guesses to the home configuration
%of the UR5e
%initialIKGuess(1) = jointStateMsg.Position(4); % update configuration in initial guess
% initialIKGuess(2) = jointStateMsg.Position(3);
% initialIKGuess(3) = jointStateMsg.Position(1);
% initialIKGuess(4) = jointStateMsg.Position(5);
% initialIKGuess(5) = jointStateMsg.Position(6);
% initialIKGuess(6) = jointStateMsg.Position(7);
% show(UR5e,initialIKGuess);
%show the UR5e model to verify the position of the robot in matlab

ik = inverseKinematics("RigidBodyTree",UR5e);
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1];
jointStateMsg = receive(jointSub,3);
initialIKGuess = homeConfiguration(UR5e);
jointStateMsg.Name;
initialIKGuess(1) = jointStateMsg.Position(4); % update configuration in initial guess
initialIKGuess(2) = jointStateMsg.Position(3);
initialIKGuess(3) = jointStateMsg.Position(1);
initialIKGuess(4) = jointStateMsg.Position(5);
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);
show(UR5e,initialIKGuess);

%obtain the coordinates of the rCan3 with respect to gazebo model 
%rCan3TopPosition = [rCan3X, rCan3Y, rCan3Z];
%rCan3TopRotation = [-pi/2 -pi 0]
%set the gripper rotation and translation to the coordinates of the can for
%the end effector pose
% gripperTranslation = rCan3TopPosition; 
% gripperRotation = rCan3TopRotation;


rCan3X = -0.04;
rCan3Y = 0.8;
rCan3Z = 0.185;

rCan3TopPosition = [rCan3X, rCan3Y, rCan3Z];
rCan3TopRotation = [-pi/2 -pi 0]

gripperTranslation = rCan3TopPosition; 
gripperRotation = rCan3TopRotation; 

%[configSoln, solnInfo] = compute the inverse kinematics
%Next send the trajectory goal 
%UR5e config = comply from the matlab structure to the ROS structure. 
%note the order of the joints used for the inverse kinematics


tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation';

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);
show(UR5e,configSoln);

UR5econfig = [configSoln(3),configSoln(2),configSoln(1),configSoln(4), configSoln(5), configSoln(6)];

%trajGoal =packGripGoal pack the goal information to to send to the action
%server of the arm
%sendGoal(trajAct,trajGoal), will send the packed information to the action
%server
trajGoal = packTrajGoal(UR5econfig,trajGoal);
sendGoal(trajAct,trajGoal)





