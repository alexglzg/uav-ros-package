%declare name of the bag
experimentbag = rosbag('curved_2020-09-18-22-58-30.bag')
disp('3DNED')
%NED Path
NED = select(experimentbag, "Topic", '/guidance/target');
NEDxy = readMessages(NED,'DataFormat','struct');
NEDxPoints = cellfun(@(m) double(m.X),NEDxy);
NEDxPoints = NEDxPoints(1:10:end,:);
NEDyPoints = cellfun(@(m) double(m.Y),NEDxy);
NEDyPoints = NEDyPoints(1:10:end,:);
figure
zeta = zeros(length(NEDxPoints));
plot3(NEDyPoints,NEDxPoints,zeta,'k--','LineWidth',1.5)
hold on
%NED USV
NEDusv = select(experimentbag, "Topic", '/vectornav/ins_2d/NED_pose');
NEDxyusv = readMessages(NEDusv,'DataFormat','struct');
NEDxPointsusv = cellfun(@(m) double(m.X),NEDxyusv);
NEDxPointsusv = NEDxPointsusv(1:10:end,:);
NEDyPointsusv = cellfun(@(m) double(m.Y),NEDxyusv);
NEDyPointsusv = NEDyPointsusv(1:10:end,:);
zetausv = zeros(length(NEDxPointsusv));
plot3(NEDyPointsusv,NEDxPointsusv,zetausv,'b-','LineWidth',1.5)
hold on
%NED UAV
NEDuav = select(experimentbag, "Topic", '/uav_model/pose');
NEDxyuav = readMessages(NEDuav,'DataFormat','struct');
NEDxPointsuav = cellfun(@(m) double(m.Position.X),NEDxyuav);
NEDxPointsuav = NEDxPointsuav(1:100:end,:);
NEDyPointsuav = cellfun(@(m) double(m.Position.Y),NEDxyuav);
NEDyPointsuav = NEDyPointsuav(1:100:end,:);
NEDzPointsuav = cellfun(@(m) double(m.Position.Z),NEDxyuav);
NEDzPointsuav = NEDzPointsuav(1:100:end,:);
plot3(NEDyPointsuav,NEDxPointsuav,-NEDzPointsuav, 'r:','LineWidth',1.5)
legend('Path','USV Trajectory', 'UAV Trajectory', 'Interpreter', 'latex')
xlabel('East [m]', 'Interpreter', 'latex') 
ylabel('North [m]', 'Interpreter', 'latex')
zlabel('Altitude [m]', 'Interpreter', 'latex')

disp('2DNED')
figure
plot(NEDyPoints,NEDxPoints, 'k--','LineWidth',1.5)
hold on
plot(NEDyPointsusv,NEDxPointsusv, 'b-','LineWidth',1.5)
hold on
plot(NEDyPointsuav,NEDxPointsuav, 'r:','LineWidth',1.5)
legend('Path','USV Trajectory', 'UAV Trajectory', 'Interpreter', 'latex')
xlabel('East [m]', 'Interpreter', 'latex') 
ylabel('North [m]', 'Interpreter', 'latex')

%Heading
disp('Yaw')
%Heading UAV
uavheadingts = timeseries(NEDuav, 'Orientation.Z');
start_time = uavheadingts.get.TimeInfo.Start;
uavheadingdata = uavheadingts.get.Data;
%Desired heading
desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading, 'Data');
t = desiredheadingts.get.Time - start_time;
desiredheadingdata = desiredheadingts.get.Data;
figure
plot(t,desiredheadingdata,'k--','LineWidth',1.5)
hold on
%Heading USV
usvheadingts = timeseries(NEDusv, 'Theta');
t = usvheadingts.get.Time - start_time;
usvheadingdata = usvheadingts.get.Data;
plot(t,usvheadingdata,'b-','LineWidth',1.5)
hold on
%Heading UAV
t = uavheadingts.get.Time - start_time;
plot(t,uavheadingdata,'r-.','LineWidth',1.5)
legend('$\psi_{d}$', '$\psi_{USV}$', '$\psi_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')

%Cross-track error
disp('CTE')
crosstrack = select(experimentbag, "Topic", '/guidance/ye');
crosstrackts = timeseries(crosstrack, 'Data');
t = crosstrackts.get.Time - start_time;
crosstrackdata = crosstrackts.get.Data;
figure
plot(t,crosstrackdata,'b-','LineWidth',1.5)
legend('$e_{yv}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$e_{yv}$ [m]', 'Interpreter', 'latex')

%Speed
disp('Speed')
desiredspeed = select(experimentbag, "Topic", '/guidance/desired_speed');
desiredspeedts = timeseries(desiredspeed, 'Data');
t = desiredspeedts.get.Time - start_time;
desiredspeeddata = desiredspeedts.get.Data;
figure
plot(t,desiredspeeddata,'k--','LineWidth',1.5)
hold on
%Speed USV
usvspeed = select(experimentbag, "Topic", '/vectornav/ins_2d/local_vel');
usvspeedts = timeseries(usvspeed, 'X');
t = usvspeedts.get.Time - start_time;
usvspeeddata = usvspeedts.get.Data;
plot(t,usvspeeddata,'b-','LineWidth',1.5)
hold on
legend('$u_{d}$', '$u_{USV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex')

%X
usvxts = timeseries(NEDusv, 'X');
t = usvxts.get.Time - start_time;
usvxdata = usvxts.get.Data;
figure
plot(t,usvxdata,'b-','LineWidth',1.5)
hold on
uavxts = timeseries(NEDuav, 'Position.X');
uavxdata = uavxts.get.Data;
t = uavxts.get.Time - start_time;
plot(t,uavxdata,'r-.','LineWidth',1.5)
legend('$x_{USV}$', '$x_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$x$ [m]', 'Interpreter', 'latex')

%Y
usvyts = timeseries(NEDusv, 'Y');
t = usvyts.get.Time - start_time;
usvydata = usvyts.get.Data;
figure
plot(t,usvydata,'b-','LineWidth',1.5)
hold on
uavyts = timeseries(NEDuav, 'Position.Y');
uavydata = uavyts.get.Data;
t = uavyts.get.Time - start_time;
plot(t,uavydata,'r-.','LineWidth',1.5)
legend('$y_{USV}$', '$y_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$y$ [m]', 'Interpreter', 'latex')

%Z
uavzts = timeseries(NEDuav, 'Position.Z');
uavzdata = uavzts.get.Data;
t = uavzts.get.Time - start_time;
t2 = t(1:10:end,:);
zetaref = zeros(length(t2));
figure
plot(t2,zetaref,'b-','LineWidth',1.5)
hold on
plot(t,uavzdata,'r-.','LineWidth',1.5)
legend('$z_{USV}$', '$z_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$z$ [m]', 'Interpreter', 'latex')

%Gains
disp('Gains')
uavk = select(experimentbag, "Topic", '/uav_control/gains');
uavk1ts = timeseries(uavk, 'X');
uavk1data = uavk1ts.get.Data;
t = uavk1ts.get.Time - start_time;
figure
plot(t,uavk1data,'g-','LineWidth',1.5)
hold on
uavk2ts = timeseries(uavk, 'Y');
uavk2data = uavk2ts.get.Data;
t = uavk2ts.get.Time - start_time;
plot(t,uavk2data,'b-.','LineWidth',1.5)
hold on
uavk3ts = timeseries(uavk, 'Z');
uavk3data = uavk3ts.get.Data;
t = uavk3ts.get.Time - start_time;
plot(t,uavk3data,'r--','LineWidth',1.5)
hold on
uavk4ts = timeseries(uavk, 'W');
uavk4data = uavk4ts.get.Data;
t = uavk4ts.get.Time - start_time;
plot(t,uavk4data,'k:','LineWidth',1.5)
hold on
usvk1 = select(experimentbag, "Topic", '/usv_control/asmc/speed_gain');
usvk1ts = timeseries(usvk1, 'Data');
t = usvk1ts.get.Time - start_time;
usvk1data = usvk1ts.get.Data;
plot(t,usvk1data,'c--','LineWidth',1.5)
hold on
usvk2 = select(experimentbag, "Topic", "/usv_control/asmc/heading_gain");
usvk2ts = timeseries(usvk2, 'Data');
t = usvk2ts.get.Time - start_time;
usvk2data = usvk2ts.get.Data;
plot(t,usvk2data,'m:','LineWidth',1.5)
legend('$UAV_1$', '$UAV_2$', '$UAV_3$', '$UAV_4$', '$USV_1$', '$USV_2$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Gain', 'Interpreter', 'latex')

%Control effort uav
disp('UUAV')
uavu = select(experimentbag, "Topic", '/uav_control/input');
uavu1ts = timeseries(uavu, 'X');
uavu1data = uavu1ts.get.Data;
t = uavu1ts.get.Time - start_time;
figure
plot(t,uavu1data,'g-','LineWidth',1.5)
hold on
uavu2ts = timeseries(uavu, 'Y');
uavu2data = uavu2ts.get.Data;
t = uavu2ts.get.Time - start_time;
plot(t,uavu2data,'b-.','LineWidth',1.5)
hold on
uavu3ts = timeseries(uavu, 'Z');
uavu3data = uavu3ts.get.Data;
t = uavu3ts.get.Time - start_time;
plot(t,uavu3data,'r--','LineWidth',1.5)
hold on
uavu4ts = timeseries(uavu, 'W');
uavu4data = uavu4ts.get.Data;
t = uavu4ts.get.Time - start_time;
plot(t,uavu4data,'k:','LineWidth',1.5)
legend('$UAV_1$', '$UAV_2$', '$UAV_3$', '$UAV_4$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Input', 'Interpreter', 'latex')

%Features
disp('Feat')
featuav = select(experimentbag, "Topic", '/uav_control/features');
featxyuav = readMessages(featuav,'DataFormat','struct');
featxPointsuav = cellfun(@(m) double(m.X),featxyuav);
featxPointsuav = featxPointsuav(1:100:end,:);
featyPointsuav = cellfun(@(m) double(m.Y),featxyuav);
featyPointsuav = featyPointsuav(1:100:end,:);
figure
plot(featyPointsuav,featxPointsuav, 'k--','LineWidth',1.5)
legend('Feature Trajectory', 'Interpreter', 'latex')
xlabel('$u$ [pixels]', 'Interpreter', 'latex') 
ylabel('$n$ [pixels]', 'Interpreter', 'latex')

%Pitch-Roll
disp('P-R')
uavheadingts = timeseries(NEDuav, 'Orientation.X');
uavheadingdata = uavheadingts.get.Data;
t = uavheadingts.get.Time - start_time;
figure
plot(t,uavheadingdata,'g-','LineWidth',1.5)
hold on
uavheadingts = timeseries(NEDuav, 'Orientation.Y');
uavheadingdata = uavheadingts.get.Data;
t = uavheadingts.get.Time - start_time;
plot(t,uavheadingdata,'b-','LineWidth',1.5)
legend('$Roll$', '$Pitch$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Attitude [rad]', 'Interpreter', 'latex')

%Thrusters usv
disp('TUSV')
rt = select(experimentbag, "Topic", '/usv_control/controller/right_thruster');
rtts = timeseries(rt, 'Data');
t = rtts.get.Time - start_time;
rtdata = rtts.get.Data;
figure
plot(t,rtdata,'g-','LineWidth',1.5)
hold on
lt = select(experimentbag, "Topic", "/usv_control/controller/left_thruster");
ltts = timeseries(lt, 'Data');
t = ltts.get.Time - start_time;
ltdata = ltts.get.Data;
plot(t,ltdata,'r-','LineWidth',1.5)
legend('$T_{stbd}$', '$T_{port}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Thrust [N]', 'Interpreter', 'latex')

%Thrust usv
disp('UUSV')
tx = select(experimentbag, "Topic", '/usv_control/controller/control_input');
txts = timeseries(tx, 'X');
t = txts.get.Time - start_time;
txdata = txts.get.Data;
figure
plot(t,txdata,'g-','LineWidth',1.5)
hold on
tzts = timeseries(tx, 'Theta');
t = tzts.get.Time - start_time;
tzdata = tzts.get.Data;
plot(t,tzdata,'r-','LineWidth',1.5)
legend('$T_{x}$', '$T_{\psi}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Thrust [N]', 'Interpreter', 'latex')