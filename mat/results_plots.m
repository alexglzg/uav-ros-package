%declare name of the bag
experimentbag = rosbag('pd_curved_2020-10-12-23-12-02.bag')
disp('3DNED')
%NED Path
NED = select(experimentbag, "Topic", '/guidance/target');
NEDxy = readMessages(NED,'DataFormat','struct');
NEDxPoints = cellfun(@(m) double(m.X),NEDxy);
NEDxPoints = NEDxPoints(1:10:end,:);
NEDyPoints = cellfun(@(m) double(m.Y),NEDxy);
NEDyPoints = NEDyPoints(1:10:end,:);
figure
zeta = NEDxPoints*0;
plot3(NEDyPoints,NEDxPoints,zeta,'k--','LineWidth',1.5)
hold on
%NED USV
NEDusv = select(experimentbag, "Topic", '/vectornav/ins_2d/NED_pose');
NEDxyusv = readMessages(NEDusv,'DataFormat','struct');
NEDxPointsusv = cellfun(@(m) double(m.X),NEDxyusv);
NEDxPointsusv = NEDxPointsusv(1:10:end,:);
NEDyPointsusv = cellfun(@(m) double(m.Y),NEDxyusv);
NEDyPointsusv = NEDyPointsusv(1:10:end,:);
zetausv = NEDxPointsusv*0;
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
uavheadingdata = uavheadingdata(1:100:end,:);
%Desired heading
desiredheading = select(experimentbag, "Topic", '/guidance/desired_heading');
desiredheadingts = timeseries(desiredheading, 'Data');
t = desiredheadingts.get.Time - start_time;
t = t(1:10:end,:);
desiredheadingdata = desiredheadingts.get.Data;
desiredheadingdata = desiredheadingdata(1:10:end,:);
figure
plot(t,desiredheadingdata,'k--','LineWidth',1.5)
hold on
%Heading USV
usvheadingts = timeseries(NEDusv, 'Theta');
t = usvheadingts.get.Time - start_time;
t = t(1:10:end,:);
usvheadingdata = usvheadingts.get.Data;
usvheadingdata = usvheadingdata(1:10:end,:);
plot(t,usvheadingdata,'b-','LineWidth',1.5)
hold on
%Heading UAV
t = uavheadingts.get.Time - start_time;
t = t(1:100:end,:);
plot(t,uavheadingdata,'r-.','LineWidth',1.5)
legend('$\psi_{d}$', '$\psi_{USV}$', '$\psi_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\psi$ [rad]', 'Interpreter', 'latex')

%Cross-track error
disp('CTE')
crosstrack = select(experimentbag, "Topic", '/guidance/ye');
crosstrackts = timeseries(crosstrack, 'Data');
t = crosstrackts.get.Time - start_time;
t = t(1:10:end,:);
crosstrackdata = crosstrackts.get.Data;
crosstrackdata = crosstrackdata(1:10:end,:);
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
t = t(1:10:end,:);
desiredspeeddata = desiredspeedts.get.Data;
desiredspeeddata = desiredspeeddata(1:10:end,:);
figure
plot(t,desiredspeeddata,'k--','LineWidth',1.5)
hold on
%Speed USV
usvspeed = select(experimentbag, "Topic", '/vectornav/ins_2d/local_vel');
usvspeedts = timeseries(usvspeed, 'X');
t = usvspeedts.get.Time - start_time;
t = t(1:10:end,:);
usvspeeddata = usvspeedts.get.Data;
usvspeeddata = usvspeeddata(1:10:end,:);
plot(t,usvspeeddata,'b-','LineWidth',1.5)
hold on
legend('$u_{d}$', '$u_{USV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$u$ [m/s]', 'Interpreter', 'latex')

%X
usvxts = timeseries(NEDusv, 'X');
t = usvxts.get.Time - start_time;
t = t(1:10:end,:);
usvxdata = usvxts.get.Data;
usvxdata = usvxdata(1:10:end,:);
figure
plot(t,usvxdata,'b-','LineWidth',1.5)
hold on
uavxts = timeseries(NEDuav, 'Position.X');
uavxdata = uavxts.get.Data;
uavxdata = uavxdata(1:100:end,:);
t = uavxts.get.Time - start_time;
t = t(1:100:end,:);
plot(t,uavxdata,'r-.','LineWidth',1.5)
legend('$x_{USV}$', '$x_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$x$ [m]', 'Interpreter', 'latex')

%Y
usvyts = timeseries(NEDusv, 'Y');
t = usvyts.get.Time - start_time;
t = t(1:10:end,:);
usvydata = usvyts.get.Data;
usvydata = usvydata(1:10:end,:);
figure
plot(t,usvydata,'b-','LineWidth',1.5)
hold on
uavyts = timeseries(NEDuav, 'Position.Y');
uavydata = uavyts.get.Data;
uavydata = uavydata(1:100:end,:);
t = uavyts.get.Time - start_time;
t = t(1:100:end,:);
plot(t,uavydata,'r-.','LineWidth',1.5)
legend('$y_{USV}$', '$y_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$y$ [m]', 'Interpreter', 'latex')

%Z
uavzts = timeseries(NEDuav, 'Position.Z');
uavzdata = uavzts.get.Data;
uavzdata = uavzdata(1:100:end,:);
t = uavzts.get.Time - start_time;
t = t(1:100:end,:);
zetaref = t*0;
figure
plot(t,zetaref,'b-','LineWidth',1.5)
hold on
plot(t,uavzdata,'r-.','LineWidth',1.5)
legend('$z_{USV}$', '$z_{UAV}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$z$ [m]', 'Interpreter', 'latex')

%Control effort uav
disp('UUAV')
uavu = select(experimentbag, "Topic", '/uav_control/input');
uavu1ts = timeseries(uavu, 'X');
uavu1data = uavu1ts.get.Data;
uavu1data = uavu1data(1:100:end,:);
t = uavu1ts.get.Time - start_time;
t = t(1:100:end,:);
figure
plot(t,uavu1data,'g-','LineWidth',1.5)
hold on
uavu2ts = timeseries(uavu, 'Y');
uavu2data = uavu2ts.get.Data;
uavu2data = uavu2data(1:100:end,:);
t = uavu2ts.get.Time - start_time;
t = t(1:100:end,:);
plot(t,uavu2data,'b-.','LineWidth',1.5)
hold on
uavu3ts = timeseries(uavu, 'Z');
uavu3data = uavu3ts.get.Data;
uavu3data = uavu3data(1:100:end,:);
t = uavu3ts.get.Time - start_time;
t = t(1:100:end,:);
plot(t,uavu3data,'r--','LineWidth',1.5)
legend('$UAV_\phi$', '$UAV_\theta$', '$UAV_\psi$','Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('$\tau$', 'Interpreter', 'latex')
uavu4ts = timeseries(uavu, 'W');
uavu4data = uavu4ts.get.Data;
uavu4data = uavu4data(1:100:end,:);
t = uavu4ts.get.Time - start_time;
t = t(1:100:end,:);
figure
plot(t,uavu4data,'k:','LineWidth',1.5)
legend('$UAV_U$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Input $U_1$', 'Interpreter', 'latex')

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
uavheadingdata = uavheadingdata(1:100:end,:);
t = uavheadingts.get.Time - start_time;
t = t(1:100:end,:);
figure
plot(t,uavheadingdata,'g-','LineWidth',1.5)
hold on
uavheadingts = timeseries(NEDuav, 'Orientation.Y');
uavheadingdata = uavheadingts.get.Data;
uavheadingdata = uavheadingdata(1:100:end,:);
t = uavheadingts.get.Time - start_time;
t = t(1:100:end,:);
plot(t,uavheadingdata,'b-','LineWidth',1.5)
legend('$Roll$', '$Pitch$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Attitude [rad]', 'Interpreter', 'latex')

%Thrusters usv
disp('TUSV')
rt = select(experimentbag, "Topic", '/usv_control/controller/right_thruster');
rtts = timeseries(rt, 'Data');
t = rtts.get.Time - start_time;
t = t(1:10:end,:);
rtdata = rtts.get.Data;
rtdata = rtdata(1:10:end,:);
figure
plot(t,rtdata,'g-','LineWidth',1.5)
hold on
lt = select(experimentbag, "Topic", "/usv_control/controller/left_thruster");
ltts = timeseries(lt, 'Data');
t = ltts.get.Time - start_time;
t = t(1:10:end,:);
ltdata = ltts.get.Data;
ltdata = ltdata(1:10:end,:);
plot(t,ltdata,'r-','LineWidth',1.5)
legend('$T_{stbd}$', '$T_{port}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Thrust [N]', 'Interpreter', 'latex')

%Thrust usv
disp('UUSV')
tx = select(experimentbag, "Topic", '/usv_control/controller/control_input');
txts = timeseries(tx, 'X');
t = txts.get.Time - start_time;
t = t(1:10:end,:);
txdata = txts.get.Data;
txdata = txdata(1:10:end,:);
figure
plot(t,txdata,'g-','LineWidth',1.5)
hold on
tzts = timeseries(tx, 'Theta');
t = tzts.get.Time - start_time;
t = t(1:10:end,:);
tzdata = tzts.get.Data;
tzdata = tzdata(1:10:end,:);
plot(t,tzdata,'r-','LineWidth',1.5)
legend('$T_{x}$', '$T_{\psi}$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Thrust [N]', 'Interpreter', 'latex')

%Gains
disp('Gains')
uavk = select(experimentbag, "Topic", '/uav_control/gains');
uavk1ts = timeseries(uavk, 'X');
uavk1data = uavk1ts.get.Data;
uavk1data = uavk1data(1:100:end,:);
t = uavk1ts.get.Time - start_time;
t = t(1:100:end,:);
figure
plot(t,uavk1data,'g-','LineWidth',1.5)
hold on
uavk2ts = timeseries(uavk, 'Y');
uavk2data = uavk2ts.get.Data;
uavk2data = uavk2data(1:100:end,:);
t = uavk2ts.get.Time - start_time;
t = t(1:100:end,:);
plot(t,uavk2data,'b-.','LineWidth',1.5)
hold on
uavk3ts = timeseries(uavk, 'Z');
uavk3data = uavk3ts.get.Data;
uavk3data = uavk3data(1:100:end,:);
t = uavk3ts.get.Time - start_time;
t = t(1:100:end,:);
plot(t,uavk3data,'r--','LineWidth',1.5)
hold on
uavk4ts = timeseries(uavk, 'W');
uavk4data = uavk4ts.get.Data;
uavk4data = uavk4data(1:100:end,:);
t = uavk4ts.get.Time - start_time;
t = t(1:100:end,:);
plot(t,uavk4data,'k:','LineWidth',1.5)
hold on
usvk1 = select(experimentbag, "Topic", '/usv_control/asmc/speed_gain');
usvk1ts = timeseries(usvk1, 'Data');
t = usvk1ts.get.Time - start_time;
t = t(1:10:end,:);
usvk1data = usvk1ts.get.Data;
usvk1data = usvk1data(1:10:end,:);
plot(t,usvk1data,'c--','LineWidth',1.5)
hold on
usvk2 = select(experimentbag, "Topic", "/usv_control/asmc/heading_gain");
usvk2ts = timeseries(usvk2, 'Data');
t = usvk2ts.get.Time - start_time;
t = t(1:10:end,:);
usvk2data = usvk2ts.get.Data;
usvk2data = usvk2data(1:10:end,:);
plot(t,usvk2data,'m:','LineWidth',1.5)
legend('$UAV_1$', '$UAV_2$', '$UAV_3$', '$UAV_4$','$USV_1$', '$USV_2$', 'Interpreter', 'latex')
xlabel('Time [s]', 'Interpreter', 'latex') 
ylabel('Gain', 'Interpreter', 'latex')