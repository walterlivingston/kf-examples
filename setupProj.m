function [currFile, currFolder] = setupProj()
%SETUPPROJ Summary of this function goes here
%   Detailed explanation goes here
    restoredefaultpath;
    
    currFile = mfilename('fullpath');
    currFolder = fileparts(currFile);
    addpath(genpath(currFolder));
end

