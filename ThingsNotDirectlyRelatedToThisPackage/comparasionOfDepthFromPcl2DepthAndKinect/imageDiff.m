clear all;
close all;
clc;

pcl2depth=imread('newmyDepthImage_screenshot_01.12.2018.png');
kinect=imread('newKinect.png');
resizedPcl2Depth=imresize(pcl2depth,[785 1046]);



image(kinect);
%image(pcl2depth);
figure();
imshowpair(kinect,resizedPcl2Depth,'falsecolor');


