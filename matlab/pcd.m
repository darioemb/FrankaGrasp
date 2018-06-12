close all;
clear;
figure;
hold on;
grid on;
ptCloud = pcread('/home/sphero/code/FrankaGrasp/src/agile_grasp_panda/data/ring12.pcd');
% ptCloud=pcread('total.pcd')
pcshow(ptCloud);

version = 'with_piolo';

approach = importdata(strcat(version,'/approach.data'),' ',1);
center = importdata(strcat(version,'/center.data'),' ',1);
surface = importdata(strcat(version,'/surface.data'),' ',1);
axis = importdata(strcat(version,'/axis.data'),' ',1);

% data: surface center
% ap=plot3(approach.data(:,1), approach.data(:,2), approach.data(:,3), 'go');
ce=plot3(center.data(:,1), center.data(:,2), center.data(:,3), 'y*');
su=plot3(surface.data(:,1), surface.data(:,2), surface.data(:,3), 'g*');
% ax=plot3(axis.data(:,1), axis.data(:,2), axis.data(:,3), 'go');

legend([ce,su],["center","surface"]);

index = 0;
check = abs(center.data(1,3)-surface.data(1,3));
size_ = size(center.data);
for i = 1:size_(1)
   if abs(center.data(i,3)-surface.data(i,3))<check
       check=abs(center.data(i,3)-surface.data(i,3));
       index = i;
   end    
end
plot3(center.data(index,1),center.data(index,2),center.data(index,3),'ro');
plot3(surface.data(index,1),surface.data(index,2),surface.data(index,3),'ko');

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
% 
% point=[0.0416059
% 	-0.0445848
% 	0.00211039
%     1]
% 
% dx = -(cos(atan(point(2)/point(1)))*0.014);
% dy = -(sin(atan(point(1)/point(2)))*0.014);
% dz = -0.0;
% 
% T1 = [1 0 0 dx
%     0 1 0 dy
%     0 0 1 dz
%     0 0 0 1];
% corr = T1*point;
% 
% 
% plot3(point(1),point(2),point(3), 'bo')
% 
% plot3(corr(1),corr(2),corr(3),'go')
% plot3(point(1)+dx,point(2)+dy, point(3), 'g*')
