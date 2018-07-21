bag = rosbag('ee_position_duz.bag');
ee_pos_msg = select(bag,'Topic','/vrep_interface/ee_position');
msgStruct = readMessages(ee_pos_msg,'DataFormat', 'struct');    

for i=1:size(msgStruct)
    x(i)=msgStruct{i}.Pose.Position.X;
end
for i=1:size(msgStruct)
    y(i)=msgStruct{i}.Pose.Position.Y;
end
for i=1:size(msgStruct)
    z(i)=msgStruct{i}.Pose.Position.Z;
end

bag1 = rosbag('ee_position_aziz.bag');
ee_pos_msg1 = select(bag1,'Topic','/vrep_interface/ee_position');
msgStruct1 = readMessages(ee_pos_msg1,'DataFormat', 'struct');    

for i=1:size(msgStruct1)
    x1(i)=msgStruct1{i}.Pose.Position.X;
end
for i=1:size(msgStruct1)
    y1(i)=msgStruct1{i}.Pose.Position.Y;
end
for i=1:size(msgStruct1)
    z1(i)=msgStruct1{i}.Pose.Position.Z;
end

bag2 = rosbag('ee_position_kondor.bag');
ee_pos_msg2 = select(bag2,'Topic','/vrep_interface/ee_position');
msgStruct2 = readMessages(ee_pos_msg2,'DataFormat', 'struct');    

for i=1:size(msgStruct2)
    x2(i)=msgStruct2{i}.Pose.Position.X;
end
for i=1:size(msgStruct2)
    y2(i)=msgStruct2{i}.Pose.Position.Y;
end
for i=1:size(msgStruct2)
    z2(i)=msgStruct2{i}.Pose.Position.Z;
end

figure;
hold on;
grid on;
agile = plot3(x1,y1,z1,'b') % agile 
haf = plot3(x,y,z,'r')    % haf
gpd = plot3(x2,y2,z2,'g') % gpd


plot3(0,0,0,'k^');
plot3(0,-0.2500,0,'k^');
plot3(0,0.2500,0,'k^');

legend('agile','haf','gpd','piolo_1','piolo_2','piolo_3')