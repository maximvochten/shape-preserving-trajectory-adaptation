function [timestamp,pose_data] = importfile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  TESTPOURING = IMPORTFILE(FILENAME) reads data from text file FILENAME
%  for the default selection.  Returns the data as a table.
%
%  TESTPOURING = IMPORTFILE(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  Example:
%  testpouring = importfile("/home/glenn/catkin_ws/src/etasl_invariants_integration/data/demonstrated_trajectories/test_pouring.csv", [1, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 21-May-2021 14:54:47

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [1, Inf];
end

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 8);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["VarName1", "VarName2", "VarName3", "VarName4", "VarName5", "VarName6", "VarName7", "VarName8"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
dataArray = readtable(filename, opts);

timestamp = dataArray{:, 1};
pose_data = [ dataArray{:, 2} dataArray{:, 3} dataArray{:, 4} dataArray{:, 5} dataArray{:, 6} dataArray{:, 7} dataArray{:, 8} ];
end