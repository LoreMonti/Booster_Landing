%RUN_ALL Regenerates every study and every figure of the project from scratch.
%
%   Run from the repository root:  run_all
%   Figures are written to figures/. The .slx files are binary and cannot
%   be read on GitHub, so this script is what makes the results
%   reproducible.

root = fileparts(mfilename('fullpath'));
addpath(root, fullfile(root, 'models'), fullfile(root, 'utils'), fullfile(root, 'studies'));
figDir = fullfile(root, 'figures');
if ~exist(figDir, 'dir'), mkdir(figDir); end

% Step 1: the open-loop plant
study_plant_validation(figDir);
study_openloop(figDir);
