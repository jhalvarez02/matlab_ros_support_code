% gazebo Can model list with cell numbers
    % gCan1 = 20
    % gCan2 = 21
    % gCan3 = 22
    % gCan4 = 23
    % rCan1 = 24
    % rCan2 = 25
    % rcan3 = 26
    % yCan1 = 27
    % yCan2 = 28
    % yCan3 = 29
    % yCan4 = 30
% %Goal is to pick up gCan1 and drop it into the green bin
    clc
    clear
    rosshutdown;
    masterhostIP = "192.168.230.128";
    rosinit(masterhostIP)
    % startup_rvc

    disp('Going home...');
     goHome('qr');    % start robot at the home configuraton
    disp('Resetting the world...');
     resetWorld;      % reset models through the gazebo service

% Pick 1      % retreive gCan1 pose via the gazebo type
    disp('Getting Robot and gCan1 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{20};         % gCan1 is cell {20}  

    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)

    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); %arm will begin to move towards goal

    if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
    goHome('qr');
end

pause(3)

%Pick 2    %retreive rCan1 pose via the gazebo type
    disp('Getting Robot and rCan1 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{24};         % rCan1 is cell {24}  

    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(rCan1)

    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); %arm will begin to move towards goal

    if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
    goHome('qr');
end

pause(3)

%Pick 3  % retreive yCan1 pose via the gazebo type
    disp('Getting Robot and yCan1 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{27};         % yCan1 is cell {27}  

    [mat_R_T_G, mat_R_T_M] =get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(ycan1)

    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); %arm will begin to move towards goal

    if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
    goHome('qr');
end

pause(3)

%Pick 4    % retreive rCan3 pose via the gazebo type
    disp('Getting Robot and rCan3 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{26};         % rCan3 is cell {26}  

    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(rCan3)

    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); %arm will begin to move towards goal

    if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
    goHome('qr');
end

pause(3)

% Pick 5    %retreive gCan2 pose via the gazebo type
    disp('Getting Robot and gCan2 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{21};         % gCan2 is cell {21}  

    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan2)
    mat_R_T_M(3,4)=0.07;
    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); %arm will begin to move towards goal

    if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin_2(strategy,mat_R_T_M,place_pose);
    goHome('qr');
end

pause(3)

%Pick 6    %retreive yCan3 pose via the gazebo type
    disp('Getting Robot and yCan3 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{29};         % yCan3 is cell {29}  

    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(yCan3)

    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); %arm will begin to move towards goal

    if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
    goHome('qr');
end

pause(3)

% Pick 7 %retreive gCan4 pose via the gazebo type
    disp('Getting Robot and gCan4 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{23};         % gCan4 is cell {23}  

    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan4)

    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); %arm will begin to move towards goal



    if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    % goHome2('qr')
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
    goHome('qr');
end

pause(3)

%Pick 8  % retreive yCan2 pose via the gazebo type
    disp('Getting Robot and yCan4 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{28};         % yCan2 is cell {28}  

    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link_2(model_name); % mat_R_T_M is the translation of robot to the model(yCan2)
    mat_R_T_M(3,4)=0.08;
    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); %arm will begin to move towards goal

    if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
    goHome('qr');
end

pause(3)

% Pick 9   % retreive rCan2 pose via the gazebo type
    disp('Getting Robot and gCan2 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{25};         % rCan2 is cell {25}  

    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(rCan2)
    %mat_R_T_M(3,4)=0.07;
%     Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); %arm will begin to move towards goal


    if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
    goHome('qz');
    goHome('qr');
end





