function report = runSACVerification(includePolicy)
%RUNSACVERIFICATION Execute existing regressions in isolated function workspaces.
    if nargin<1, includePolicy=true; end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(root,fullfile(root,'Scripts'),fullfile(root,'tests'));
    visibility=get(groot,'DefaultFigureVisible');
    guard=onCleanup(@() set(groot,'DefaultFigureVisible',visibility));
    set(groot,'DefaultFigureVisible','off');
    names={'testForwardKinematics','testActionUpdate','testRobotCollision', ...
        'testStepFunction','testGoalSampling','testTrainingEnvironment'};
    for i=1:numel(names)
        fprintf('REGRESSION %s\n',names{i});
        runLegacy(fullfile(root,'tests',[names{i} '.m']));
    end
    report=struct('legacyPassed',{names});
    if includePolicy, report.SAC=testSACWorkflow(); end
end

function runLegacy(path)
    run(path);
end
