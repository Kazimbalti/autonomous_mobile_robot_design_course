%%  Probabilistic Roadmam (PRM) examples

clear; close all;
%%  Load Map
load('map_matrix.mat');
map = robotics.OccupancyGrid(mapmatrix,4);

%%  Create a roadmap with 150 nodes.

prm_example = robotics.PRM(map,150);
show(prm_example)


%%  Calculate Collision-Free Path

startLocation = [0.5 0.5];
endLocation = [6 6.2];
path_ = findpath(prm_example,startLocation,endLocation);
show(prm_example)

