%declare name of the bag
experimentbag = rosbag('curved_2020-09-04-23-12-33.bag')
%NED Path
NED = select(experimentbag, "Topic", '/guidance/target');
NEDxy = readMessages(NED,'DataFormat','struct');
NEDxPoints = cellfun(@(m) double(m.X),NEDxy);
NEDyPoints = cellfun(@(m) double(m.Y),NEDxy);
figure
zeta = zeros(length(NEDxPoints));
plot3(NEDyPoints,NEDxPoints,zeta)
hold on
%NED USV
NEDusv = select(experimentbag, "Topic", '/vectornav/ins_2d/NED_pose');
NEDxyusv = readMessages(NEDusv,'DataFormat','struct');
NEDxPointsusv = cellfun(@(m) double(m.X),NEDxyusv);
NEDyPointsusv = cellfun(@(m) double(m.Y),NEDxyusv);
zetausv = zeros(length(NEDxPointsusv));
plot3(NEDyPointsusv,NEDxPointsusv,zetausv)
hold on
%NED UAV
NEDuav = select(experimentbag, "Topic", '/uav_model/pose');
NEDxyuav = readMessages(NEDuav,'DataFormat','struct');
NEDxPointsuav = cellfun(@(m) double(m.Position.X),NEDxyuav);
NEDyPointsuav = cellfun(@(m) double(m.Position.Y),NEDxyuav);
NEDzPointsuav = cellfun(@(m) double(m.Position.Z),NEDxyuav);
plot3(NEDyPointsuav,NEDxPointsuav,-NEDzPointsuav)
legend('Path','USV Trajectory', 'UAV Trajectory', 'Interpreter', 'latex')
xlabel('East [m]', 'Interpreter', 'latex') 
ylabel('North [m]', 'Interpreter', 'latex')
zlabel('Altitude [m]', 'Interpreter', 'latex')

figure
plot(NEDyPoints,NEDxPoints)
hold on
plot(NEDyPointsusv,NEDxPointsusv)
hold on
plot(NEDyPointsuav,NEDxPointsuav)
legend('Path','USV Trajectory', 'UAV Trajectory', 'Interpreter', 'latex')
xlabel('East [m]', 'Interpreter', 'latex') 
ylabel('North [m]', 'Interpreter', 'latex')

%Heading
desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading, 'Data');
start_time = desiredheadingts.get.TimeInfo.Start;
t = desiredheadingts.get.Time - start_time;
desiredheadingdata = desiredheadingts.get.Data;
figure
plot(t,desiredheadingdata)
hold on
%Heading USV
heading = select(experimentbag, "Topic", '/vectornav/ins_2d/NED_pose');
headingts = timeseries(heading, 'Theta');
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
plot(t,headingdata)
hold on
%Heading UAV
heading = select(experimentbag, "Topic", '/uav_model/pose');
headingts = timeseries(heading, 'Orientation.Z');
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
plot(t,headingdata)
legend('$\psi_{d}$', '$\psi_{USV}$', '$\psi_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')

%Cross-track error
desiredheading = select(experimentbag, "Topic", '/guidance/ye');
desiredheadingts = timeseries(desiredheading, 'Data');
t = desiredheadingts.get.Time - start_time;
desiredheadingdata = desiredheadingts.get.Data;
figure
plot(t,desiredheadingdata)
legend('$y_{e}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$y_{e}$ [m]', 'Interpreter', 'latex')

%Speed
desiredheading = select(experimentbag, "Topic", '/guidance/desired_speed');
desiredheadingts = timeseries(desiredheading, 'Data');
start_time = desiredheadingts.get.TimeInfo.Start;
t = desiredheadingts.get.Time - start_time;
desiredheadingdata = desiredheadingts.get.Data;
figure
plot(t,desiredheadingdata)
hold on
%Speed USV
heading = select(experimentbag, "Topic", '/vectornav/ins_2d/local_vel');
headingts = timeseries(heading, 'X');
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
plot(t,headingdata)
hold on
%Speed UAV
heading = select(experimentbag, "Topic", '/uav_model/vel');
headingts = timeseries(heading, 'Linear.X');
t = headingts.get.Time - start_time;
headingdata = headingts.get.Data;
plot(t,headingdata)
legend('$u_{d}$', '$u_{USV}$', '$u_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex')