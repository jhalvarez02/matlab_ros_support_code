pickTopDownCan

% Deliverables:
% o pickTopDownLift.m
% Inputs (none)
% Outputs: an array with the [x,y,z,r,p,y] of your end-effector at the time it
% touches the top of the can.
% You will upload/push pickTopDownLift.m to your own github repo. Follow
% the following steps to do so:
% 1. Tell git to state the file pickTo pickTopDownLift pDownCan.m
% git add pickTopDownLift.m
% 2. Commit the changes you have made to this file to history via the commit
% command:
% git commit -m “enter some descriptive message here about changes you
% made. I.e. created moveToCan.m to reach top of can. ”
% 3. Send this commit to your yours repository in the cloud:
% git push yours main


%continuation from part C, add following code to close the gripper on the
%can to pick it up
%update coordinates of can for better precision as follows
% rCan3X = -0.04; 
% rCan3Y = 0.8;
% rCan3Z = 0.30;
% rCan3TopPosition = [rCan3X, rCan3Y, rCan3Z];
% rCan3TopRotation = [-pi/2 -pi 0];
% gripperTranslation = rCan3TopPosition; 
% gripperRotation = rCan3TopRotation; 
%pause(30) pause 30 seconds and wait for the robot to register its current
%postion relative to the world

pause(30)

%Can Position & Orientation
rCan3X = -0.04;
rCan3Y = 0.8;
rCan3Z = 0.30;
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

%compute the homogeneous transformation of the gripper via tform (rotation
%and translation
% tform = eul2tform(gripperRotation); % ie eul2tr call
% tform(1:3,4) = gripperTranslation';
%[configSoln, solnInfo] = compute the inverse kinematics
% [configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);
%Next send the trajectory goal 
%UR5e config = comply from the matlab structure to the ROS structure. 
%note the order of the joints used for the inverse kinematics
%pause(2)= pause for 2 seconds and wait for the gazebo model
%the gazebo robot should now be lifting the can 

