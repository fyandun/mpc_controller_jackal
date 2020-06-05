%%
clear
load('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/cx_global.mat')
load('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/cyaw_global.mat')
load('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/cy_global.mat')

bag = rosbag('/home/fyandun/Documentos/simulation/catkin_ws/src/mpc_controller_warthog/bagFiles/nav_test_1.bag');
bagSelection = select(bag,'Topic','/odometry/filtered');
allMsgs = readMessages(bagSelection);

%%
timeAll = zeros(1,length(allMsgs));
secsAll = zeros(1,length(allMsgs));
nsecsAll = zeros(1,length(allMsgs));
timeZero = zeros(1,length(allMsgs));

pose = zeros(length(allMsgs), 2);
for i=1:length(allMsgs)
   current = allMsgs{i}.Header.Stamp.Sec + allMsgs{i}.Header.Stamp.Nsec*1e-9;
   timeAll(i) = current;
   timeZero(i) = current - timeAll(1);
   
   secsAll(i) = allMsgs{i}.Header.Stamp.Sec;
   nsecsAll(i) = allMsgs{i}.Header.Stamp.Nsec;
   
   pose(i,1) = allMsgs{i}.Pose.Pose.Position.X;
   pose(i,2) = allMsgs{i}.Pose.Pose.Position.Y;
   
       
end


%%
time_cx = zeros(1, length(global_cx));
secs_cx = zeros(1, length(global_cx));
nsecs_cx = zeros(1, length(global_cx));


time_zero = zeros(1, length(global_cx));
for j=1:length(global_cx)
    dist = pdist2([global_cx(j) global_cy(j)], pose);
    [~, index_min_dist_init]= min(dist);
    time_cx(j) = timeAll(index_min_dist_init);
    time_zero(j) = timeZero(index_min_dist_init);
    
    secs_cx(j) = secsAll(index_min_dist_init);
    nsecs_cx(j) = nsecsAll(index_min_dist_init);
end

%%
dist = pdist2([global_cx(1) global_cy(1)], pose);
[~, index_min_dist_init]= min(dist);


%%
dist = pdist2([global_cx(end) global_cy(end)], pose);
[~, index_min_dist_end]= min(dist);

%%
pose_ = pose(index_min_dist_init:index_min_dist_end,:);
time_filtered_ = timeZero(index_min_dist_init:index_min_dist_end);
[~,ia,~] = unique(pose_,'rows');
time_filtered_ = time_filtered_(ia);
pose_filtered_ = pose_(ia,:);

[~,ia,~] = unique(time_filtered_);
time_filtered = time_filtered_(ia);
pose_filtered = pose_filtered_(ia,:);


x_interp = interp1(time_filtered, pose_filtered(:,1), global_cx','linear','extrap');
y_interp = interp1(time_filtered, pose_filtered(:,2), global_cy,'linear','extrap');

% x_interp = interp1(pose_filtered(:,2), pose_filtered(:,1), global_cx,'linear','extrap');
% y_interp = interp1(pose_filtered(:,1), pose_filtered(:,2), global_cy,'linear','extrap');

%%
figure()
plot(pose(:,1), pose(:,2), '.b')
hold on
plot(x_interp, y_interp, 'xr')