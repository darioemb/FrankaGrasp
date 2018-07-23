close all;
figure;
hold on;
grid on;
ptCloud = pcread('/home/dario/FrankaGrasp/src/gpd_bridge/data/ring12.pcd');
pcshow(ptCloud);

% data: sample
% plot3(VarName1(:,1), VarName2(:,1), VarName3(:,1), 'go');

% T = [0, 0.445417, 0.895323, 0.215
% 1, 0, 0, -0.015
% 0, 0.895323, -0.445417, 0.23
% 0, 0, 0, 1]
% 
% x = VarName1(1);
% y = VarName2(1);
% z = VarName3(1);
% 
% plot3(x,y,z,'ro');

% X = [x
%     y
%     z
%     1];
% dx =cos(atan(y/x))*0.014;
% dy= sin(atan(x/y))*0.014;
% dz=0.00;
% T1 = [1 0 0 dx
%     0 1 0 dy
%     0 0 1 dz
%     0 0 0 1];
% newX = T1*X;
% plot3(newX(1),newX(2),newX(3),'bo');


% pose = [0.252942
%     0.873699
%     -0.00987564];

% plot3(pose(1),pose(2),pose(3),'r*');

point=[0.01800000
	0.0419999
	-0.0099999
    1]

dx = -(cos(atan(point(2)/point(1)))*0.014);
dy = -(sin(atan(point(1)/point(2)))*0.014);
dz = -0.0;

T1 = [1 0 0 dx
    0 1 0 dy
    0 0 1 dz
    0 0 0 1];
corr = T1*point;


plot3(point(1),point(2),point(3), 'bo')

plot3(corr(1),corr(2),corr(3),'go')
plot3(point(1)+dx,point(2)+dy, point(3), 'g*')

