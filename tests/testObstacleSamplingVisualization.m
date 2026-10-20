%% Offline obstacle sampling inspection; run manually, never during training.
% Options example:
% obstacleSamplingVisualizationOptions = struct('obstacleSamplingMode',"arm_sweep", ...
%     'seed',314160,'reportCollisionLinks',true,'maxRejectedCandidates',20);
% To inspect saved data without generating anything:
% obstacleSamplingVisualizationOptions = struct('episode',episodes{1}, ...
%     'firstCollisionPair',firstCollisionPairs(1));
% Set 'cacheCollisions',false to defer the inspector's offline collision scan.
if ~exist('obstacleSamplingVisualizationOptions','var')
    obstacleSamplingVisualizationOptions = struct();
end
viewerOptions = obstacleSamplingVisualizationOptions;
viewerRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(viewerRoot);
viewerPair = "";
if isfield(viewerOptions,'episode')
    samplingEpisode = viewerOptions.episode;
else
    [samplingEpisode,viewerPair] = generateOneScenario(viewerRoot,viewerOptions);
end
if isfield(viewerOptions,'firstCollisionPair')
    viewerPair = string(viewerOptions.firstCollisionPair);
end
[samplingFigure,samplingGeometry] = environment.visualizeObstacleSampling( ...
    samplingEpisode,FirstCollisionPair=viewerPair, ...
    MaxRejectedCandidates=option(viewerOptions,'maxRejectedCandidates',20), ...
    ShowSegmentAnchors=option(viewerOptions,'showSegmentAnchors',false), ...
    CacheCollisions=option(viewerOptions,'cacheCollisions',true), ...
    ReportCollisionLinks=option(viewerOptions,'reportCollisionLinks',true));

function [episode,pair] = generateOneScenario(projectRoot,options)
    % Reuse the existing reset configuration, seed isolation and diagnostics.
    % It performs the existing interface checks, one reset plus seeded replay,
    % and interpolation inspection. No retries or new sampler are introduced.
    % Local scope preserves any caller environmentDiagnosticOptions variable.
    setupFigures = findall(groot,'Type','figure','Name','Franka Panda - Ready Configuration');
    setupGuard = onCleanup(@() closeSetupFigures(setupFigures));
    environmentDiagnosticOptions = options;
    environmentDiagnosticOptions.nEpisodes = 1;
    environmentDiagnosticOptions.doVisualize = false;
    environmentDiagnosticOptions.recordObstacleSamplingGeometry = true;
    environmentDiagnosticOptions.seed = option(options,'seed',314160);
    % The inspector caches collision pairs itself; avoid a duplicate pair scan.
    environmentDiagnosticOptions.reportCollisionLinks = false;
    environmentDiagnosticOptions.obstacleSamplingMode = ...
        option(options,'obstacleSamplingMode',"arm_sweep");
    run(fullfile(projectRoot,'tests','testEnvironment.m'));
    episode = episodes{1};
    pair = firstCollisionPairs(1);
end

function closeSetupFigures(previous)
    % Keep the generated-scenario workflow to one inspector. Preserve any
    % setup figures already open before this driver called the existing test.
    figures = findall(groot,'Type','figure','Name','Franka Panda - Ready Configuration');
    for h = figures(:).'
        if ~any(h==previous), delete(h); end
    end
end

function value = option(s,name,default)
    if isfield(s,name), value=s.(name); else, value=default; end
end
