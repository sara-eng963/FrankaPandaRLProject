function [fig, geometry] = visualizeObstacleSampling(s, options)
%VISUALIZEOBSTACLESAMPLING Offline, interactive nominal-motion inspector.
% Accepts an episode returned by resetFunction; never supplies an RL action.
% Slider poses are snapped to an offline joint-increment grid so displayed
% collision results belong to the exact displayed configuration.
% Recorded anchors are preferred. Legacy reconstruction is explicitly labeled.
% First sampled collision is not continuous first contact or an RRT route.
% CacheCollisions=false defers the offline scan until its button is pressed.
% Live cache data is available through fig.UserData.CollisionCache.
    arguments
        s (1,1) struct
        options.MaxRejectedCandidates (1,1) double {mustBeInteger,mustBeNonnegative,mustBeFinite} = 20
        options.ShowSegmentAnchors (1,1) logical = false
        options.FirstCollisionPair (1,1) string = ""
        options.CacheCollisions (1,1) logical = true
        options.ReportCollisionLinks (1,1) logical = true
        options.SweepSamples (1,1) double {mustBeInteger,mustBeFinite,mustBePositive} = 61
    end
    assert(options.SweepSamples>=2,'Use at least two sweep samples.');
    d = s.scenarioDiagnostics;
    if options.FirstCollisionPair=="" && isfield(d,'firstCollisionPair')
        options.FirstCollisionPair=string(d.firstCollisionPair);
    end
    p = s.params;
    robot = copy(s.robot); % Isolate this figure's display state.
    workspaceBox = p.workspaceBox;
    hasGoal = isfield(s,'sampledGoalQ') && ~isempty(s.sampledGoalQ) && ...
        ~ismember(d.failureStage,["start","target"]);
    qStart = s.q;
    qGoal = qStart;
    if hasGoal, qGoal=s.sampledGoalQ; end
    assert(isequal(qStart(8:9),qGoal(8:9)),'Expected fixed gripper joints.');
    dq = qGoal(1:7)-qStart(1:7);
    nSteps = 0;
    % A fine display grid makes short motions smooth too; no RL limits change.
    if hasGoal, nSteps=max(200,ceil(max(abs(dq))/p.maxIncrement)); end
    sampleFractions = (0:nSteps).'/max(1,nSteps);
    configurations = repmat(qStart,numel(sampleFractions),1);
    configurations(:,1:7) = qStart(1:7)+sampleFractions*dq;
    geometry = anchorGeometry(s,hasGoal);
    geometry.nominalConfigurations = configurations;
    geometry.sweepPoints = zeros(3,4,0);
    if hasGoal
        geometry.sweepPoints = zeros(3,4,options.SweepSamples);
        for k = 1:options.SweepSamples
            q = qStart;
            q(1:7) = qStart(1:7)+(k-1)/(options.SweepSamples-1)*dq;
            for link = 1:4
                T = getTransform(robot,q,sprintf('panda_link%d',link+3));
                geometry.sweepPoints(:,link,k) = T(1:3,4);
            end
        end
    end
    radius = d.obstacleRadius;
    if d.success, radius=s.obstacle.Radius; end
    cache = struct('checked',false(numel(sampleFractions),1), ...
        'colliding',false(numel(sampleFractions),1), ...
        'pairs',strings(numel(sampleFractions),1),'firstIndex',[]);
    currentIndex = 1;
    busy = false;
    clockHandle = [];
    bg = [0.96 0.97 0.98];
    ink = [0.13 0.17 0.23];
    fig = figure('Name','Panda | Whole-arm sampling inspector', ...
        'NumberTitle','off','Color',bg,'Position',[70 60 1450 880], ...
        'ToolBar','figure','MenuBar','figure');
    ax = axes('Parent',fig,'Position',[0.045 0.23 0.665 0.70], ...
        'Color',[0.99 0.99 1],'XColor',ink,'YColor',ink,'ZColor',ink);
    % Supported fast updates maintain a single robot's existing graphics.
    show(robot,qStart,'Parent',ax,'Frames','off','Visuals','on', ...
        'Collisions','off','PreservePlot',false,'FastUpdate',false);
    hold(ax,'on');
    set(findall(ax,'Type','patch'),'FaceAlpha',1,'FaceColor',[0.64 0.69 0.76], ...
        'EdgeColor','none');
    initialBounds = [xlim(ax);ylim(ax);zlim(ax)];
    layers = struct();
    layers.sweep = hggroup('Parent',ax);
    linkColors = [0.10 0.48 0.75;0.65 0.35 0.75;0.05 0.58 0.48;0.87 0.47 0.10];
    for link = 1:4
        if ~hasGoal, break; end
        pts = reshape(geometry.sweepPoints(:,link,:),3,[]);
        line('Parent',layers.sweep,'XData',pts(1,:),'YData',pts(2,:), ...
            'ZData',pts(3,:),'Color',linkColors(link,:),'LineWidth',1.8);
        text('Parent',layers.sweep,'Position',pts(:,end).', ...
            'String',sprintf('  L%d',link+3),'Color',linkColors(link,:), ...
            'FontWeight','bold','FontSize',10,'Interpreter','none');
    end
    layers.raw = pointLayer(ax,geometry.rawAnchors,70,[0.32 0.36 0.42],'o',false);
    layers.rejected = pointLayer(ax,geometry.rawAnchors(:,~geometry.feasibleMask), ...
        70,[0.82 0.20 0.20],'x',false);
    layers.retained = pointLayer(ax,geometry.rawAnchors(:,geometry.feasibleMask), ...
        42,[0.04 0.58 0.32],'o',true);
    layers.midpoints = pointLayer(ax,geometry.rawAnchors(:,geometry.midpointMask), ...
        65,[0.05 0.53 0.65],'d',false);
    layers.workspace = workspaceLayer(ax,workspaceBox);
    layers.obstacle = hggroup('Parent',ax);
    if d.success
        center = s.obstacle.Pose(1:3,4);
        [sx,sy,sz] = sphere(28);
        h = surf(ax,radius*sx+center(1),radius*sy+center(2),radius*sz+center(3), ...
            'FaceColor',[0.96 0.56 0.12],'FaceAlpha',0.42,'EdgeColor','none');
        h.Parent = layers.obstacle;
        h = scatter3(ax,center(1),center(2),center(3),60,[0.66 0.28 0.04],'filled');
        h.Parent = layers.obstacle;
    end
    layers.ee = hggroup('Parent',ax);
    T = kinematics.forwardKinematics(robot,qStart,s.eeFrame);
    startEE = T(1:3,4);
    h = scatter3(ax,startEE(1),startEE(2),startEE(3),70,[0.10 0.40 0.80],'filled');
    h.Parent = layers.ee;
    distance = NaN;
    if hasGoal
        T = kinematics.forwardKinematics(robot,qGoal,s.eeFrame);
        goalEE = T(1:3,4);
        h = scatter3(ax,goalEE(1),goalEE(2),goalEE(3),70,[0.61 0.28 0.70],'filled');
        h.Parent = layers.ee;
        line('Parent',layers.ee,'XData',[startEE(1) goalEE(1)], ...
            'YData',[startEE(2) goalEE(2)],'ZData',[startEE(3) goalEE(3)], ...
            'Color',[0.30 0.47 0.64],'LineStyle','--','LineWidth',1.2);
        distance = norm(s.target-startEE);
    end
    [layers.candidates,candidateKey] = candidateLayer(ax,d,options.MaxRejectedCandidates);
    boundPoints = [workspaceBox initialBounds reshape(geometry.sweepPoints,3,[])];
    if d.success
        boundPoints = [boundPoints center-radius center+radius];
    end
    axis(ax,'equal');
    xlim(ax,[min(boundPoints(1,:))-.09 max(boundPoints(1,:))+.09]);
    ylim(ax,[min(boundPoints(2,:))-.09 max(boundPoints(2,:))+.09]);
    zlim(ax,[min(boundPoints(3,:))-.07 max(boundPoints(3,:))+.09]);
    set(ax,'XLimMode','manual','YLimMode','manual','ZLimMode','manual', ...
        'GridAlpha',0.08,'MinorGridAlpha',0.03,'FontSize',10, ...
        'Color',[.99 .99 1],'XColor',ink,'YColor',ink,'ZColor',ink);
    grid(ax,'on'); box(ax,'off'); view(ax,135,24);
    xlabel(ax,'X (m)','Color',ink); ylabel(ax,'Y (m)','Color',ink);
    zlabel(ax,'Z (m)','Color',ink);
    title(ax,'Nominal motion | sampled whole-arm geometry','Color',ink,'FontSize',13);
    legend(ax,'off');
    rotate3d(fig,'on');

    panel = uipanel(fig,'Units','normalized','Position',[0.745 0.045 0.24 0.90], ...
        'Title','Scenario & layers','BackgroundColor',bg,'ForegroundColor',ink);
    infoText = ui(panel,'text',[.05 .70 .90 .28],'');
    originText = ui(panel,'text',[.05 .615 .90 .075],char(geometry.source));
    originText.FontSize = 9;
    checkLayer('Sweep traces: L4 / L5 / L6 / L7','sweep',true,.565,ink);
    checkLayer('Raw anchors: open circles','raw',false,.520,[.32 .36 .42]);
    checkLayer('Rejected anchors: red crosses','rejected',false,.475,[.70 .15 .15]);
    checkLayer('Retained anchors: green dots','retained',true,.430,[.02 .45 .22]);
    checkLayer('Mark segment midpoints: diamonds','midpoints',options.ShowSegmentAnchors,.385,[.02 .43 .53]);
    candidateToggle = checkLayer('Rejected candidates (hover for key)','candidates',false,.340,ink);
    candidateToggle.TooltipString = candidateKey;
    checkLayer('Workspace boundary','workspace',true,.295,ink);
    checkLayer('EE: blue start / purple goal & line','ee',true,.250,ink);
    obstacleToggle = checkLayer('Accepted sphere & center','obstacle',d.success,.205,[.66 .28 .04]);
    if ~d.success, obstacleToggle.Enable='off'; end
    cacheButton = ui(panel,'pushbutton',[.05 .135 .90 .055],'Cache collision samples');
    cacheButton.Callback = @buildCollisionCache;
    liveText = ui(panel,'text',[.05 .015 .90 .105],'Current collision status: not cached');
    liveText.FontSize = 9;
    motion = uipanel(fig,'Units','normalized','Position',[.045 .075 .665 .125], ...
        'Title','Nominal joint interpolation (fixed fingers)','BackgroundColor',bg, ...
        'ForegroundColor',ink);
    percentage = ui(motion,'text',[.02 .67 .12 .22],'0.0%');
    slider = ui(motion,'slider',[.15 .70 .82 .16],'');
    set(slider,'Min',0,'Max',1,'Value',0,'SliderStep', ...
        [1/max(1,nSteps) min(1,10/max(1,nSteps))],'Callback',@scrub);
    sliderListener = addlistener(slider,'ContinuousValueChange',@scrub);
    playButton = ui(motion,'pushbutton',[.02 .12 .18 .35],'Play');
    playButton.Callback = @togglePlay;
    resetButton = ui(motion,'pushbutton',[.22 .12 .18 .35],'Reset');
    resetButton.Callback = @resetPose;
    jumpButton = ui(motion,'pushbutton',[.42 .12 .34 .35],'Jump to first collision');
    jumpButton.Callback = @jumpToCollision;
    jumpButton.Enable = 'off';
    sampleText = ui(motion,'text',[.78 .12 .20 .35],'');
    ui(fig,'text',[.045 .015 .67 .045], ...
        'Traces/segments are sampled geometric references, not collision-mesh swept volumes or certified reachability sets.');
    clockHandle = timer('ExecutionMode','fixedSpacing','Period',0.04, ...
        'BusyMode','drop','TimerFcn',@advanceFrame,'ErrorFcn',@(~,~) pausePlayback());
    fig.CloseRequestFcn = @closeInspector;
    fig.DeleteFcn = @cleanupTimer;
    refreshInfo();
    showSample(1);
    if ~hasGoal
        slider.Enable='off'; playButton.Enable='off'; resetButton.Enable='off';
        cacheButton.Enable='off';
        liveText.String='No validated goal: showing stored start/initial pose only.';
    elseif options.CacheCollisions
        buildCollisionCache();
    end
    if isgraphics(fig)
        % Keep the listener alive and make cached data accessible after callbacks.
        fig.UserData = struct('Geometry',geometry,'CollisionCache',cache, ...
            'SliderListener',sliderListener);
    end

    function h = ui(parent,style,position,label)
        h = uicontrol(parent,'Style',style,'Units','normalized','Position',position, ...
            'String',label,'ForegroundColor',ink,'BackgroundColor',bg, ...
            'FontSize',10,'HorizontalAlignment','left');
        if any(strcmp(style,{'pushbutton','slider'}))
            h.BackgroundColor=[.89 .91 .94];
        elseif strcmp(style,'text') && ~isempty(label)
            h.String=textwrap(h,cellstr(label));
        end
    end

    function h = checkLayer(label,name,value,y,color)
        h = ui(panel,'checkbox',[.05 y .92 .038],label);
        h.Value=value; h.ForegroundColor=color;
        set(layers.(name),'Visible',onOff(value));
        h.Callback = @(src,~) set(layers.(name),'Visible',onOff(src.Value));
    end

    function refreshInfo()
        method = d.obstacleSamplingMode;
        if method=="", method="not selected"; end
        outcome = "ACCEPTED";
        if ~d.success, outcome="REJECTED: "+d.failureStage+" (no accepted sphere)"; end
        rrt = sprintf('%d passed / %d rejected',d.rrtAttempts-d.rrtRejects,d.rrtRejects);
        if d.rrtAttempts==0, rrt='not run'; end
        first = 'not cached';
        pair = "";
        if isempty(cache.firstIndex) && all(cache.checked), first='none on this sample grid'; end
        if ~isempty(cache.firstIndex)
            first=sprintf('%.2f%% (discrete sample)',100*sampleFractions(cache.firstIndex));
            if cache.pairs(cache.firstIndex)~="", pair=cache.pairs(cache.firstIndex); end
        end
        if pair=="", pair="not recorded"; end
        lines = {char("Method: "+method),char(outcome), ...
            sprintf('Radius: %.1f cm | start-target: %.1f cm',100*radius,100*distance), ...
            sprintf('Anchors G / R / kept: %d / %d / %d', ...
            d.armAnchorsGenerated,d.armAnchorsRejected,d.armAnchorsRetained), ...
            sprintf('Candidate attempts: %d',d.attempts),['RRT: ' rrt], ...
            ['First collision: ' first],char("Cached pair: "+pair)};
        if options.FirstCollisionPair~=""
            lines{end+1}=char("Saved diagnostic pair: "+options.FirstCollisionPair);
        end
        infoText.String=textwrap(infoText,lines);
    end

    function showSample(index)
        if ~isgraphics(fig), return; end
        currentIndex=max(1,min(numel(sampleFractions),round(index)));
        % Preserve camera position, target, view angle and zoom while updating.
        camera = {ax.CameraPosition,ax.CameraTarget,ax.CameraUpVector,ax.CameraViewAngle};
        show(robot,configurations(currentIndex,:),'Parent',ax,'Frames','off', ...
            'Visuals','on','Collisions','off','PreservePlot',false,'FastUpdate',true);
        set(ax,'CameraPosition',camera{1},'CameraTarget',camera{2}, ...
            'CameraUpVector',camera{3},'CameraViewAngle',camera{4});
        slider.Value=sampleFractions(currentIndex);
        percentage.String=sprintf('%.1f%%',100*slider.Value);
        sampleText.String=sprintf('Sample %d / %d',currentIndex-1,nSteps);
        scope='arm + accepted obstacle';
        if ~d.success, scope='SELF ONLY; no accepted obstacle'; end
        if ~cache.checked(currentIndex)
            liveText.String={['Scope: ' scope],'Current sample: not cached'};
            liveText.ForegroundColor=ink;
        elseif cache.colliding(currentIndex)
            liveText.String={['Scope: ' scope],'Current sample: COLLISION', ...
                char(cache.pairs(currentIndex))};
            liveText.ForegroundColor=[.75 .14 .14];
        else
            liveText.String={['Scope: ' scope],'Current sample: clear (discrete)', ...
                'This does not establish clearance between samples.'};
            liveText.ForegroundColor=[.02 .42 .24];
        end
    end

    function scrub(~,~)
        if busy || ~hasGoal, return; end
        pausePlayback();
        showSample(1+round(slider.Value*nSteps));
        drawnow limitrate;
    end

    function togglePlay(~,~)
        if busy || ~hasGoal, return; end
        if strcmp(clockHandle.Running,'on')
            pausePlayback();
        else
            if currentIndex==numel(sampleFractions), showSample(1); end
            playButton.String='Pause';
            start(clockHandle);
        end
    end

    function advanceFrame(~,~)
        if ~isgraphics(fig), cleanupTimer(); return; end
        if busy, return; end
        % About eight seconds per pass; joint interpolation grid stays exact.
        showSample(currentIndex+max(1,round(nSteps/200)));
        if currentIndex==numel(sampleFractions), pausePlayback(); end
        drawnow limitrate;
    end

    function pausePlayback()
        if ~isempty(clockHandle) && isvalid(clockHandle), stop(clockHandle); end
        if isgraphics(playButton), playButton.String='Play'; end
    end

    function resetPose(~,~)
        if busy, return; end
        pausePlayback(); showSample(1);
    end

    function jumpToCollision(~,~)
        if busy || isempty(cache.firstIndex), return; end
        pausePlayback(); showSample(cache.firstIndex);
    end

    function buildCollisionCache(varargin)
        if busy || ~hasGoal || all(cache.checked), return; end
        busy=true; pausePlayback();
        set([slider playButton resetButton jumpButton cacheButton],'Enable','off');
        guard = onCleanup(@finishCaching);
        names = [string(robot.BodyNames(:)); string(robot.BaseName)];
        for index = 1:numel(sampleFractions)
            if ~isgraphics(fig), return; end
            if cache.checked(index), continue; end
            q = configurations(index,:);
            if options.ReportCollisionLinks
                % Same skipped pairs and Exhaustive=off as the existing helper.
                if d.success
                    [flags,separation] = checkCollision(robot,q,{s.obstacle}, ...
                        'SkippedSelfCollisions',p.skipPairs,'Exhaustive','off');
                else
                    [flags,separation] = checkCollision(robot,q, ...
                        'SkippedSelfCollisions',p.skipPairs,'Exhaustive','off');
                end
                [row,col] = find(isnan(separation),1);
                if ~isempty(row)
                    if col<=robot.NumBodies+1
                        cache.pairs(index)=names(row)+" <-> "+names(col)+" (self)";
                    else
                        cache.pairs(index)=names(row)+" <-> obstacle";
                    end
                end
            elseif d.success
                colliding = collision.checkRobotCollision(robot,q,s.obstacle,p.skipPairs);
                % Attribution not requested: retain only the combined boolean.
                flags = colliding;
            else
                flags = checkCollision(robot,q,'SkippedSelfCollisions',p.skipPairs, ...
                    'Exhaustive','off');
            end
            cache.colliding(index)=any(flags);
            cache.checked(index)=true;
            if any(flags) && isempty(cache.firstIndex), cache.firstIndex=index; end
            if mod(index,20)==0
                liveText.String=sprintf('Offline collision cache: %d / %d',index,numel(sampleFractions));
                drawnow limitrate;
            end
        end
    end

    function finishCaching()
        busy=false;
        if ~isgraphics(fig), return; end
        set([slider playButton resetButton],'Enable','on');
        cacheButton.Enable=onOff(~all(cache.checked));
        jumpButton.Enable=onOff(~isempty(cache.firstIndex));
        refreshInfo(); showSample(currentIndex);
        fig.UserData = struct('Geometry',geometry,'CollisionCache',cache, ...
            'SliderListener',sliderListener);
    end

    function cleanupTimer(varargin)
        if ~isempty(clockHandle) && isvalid(clockHandle)
            stop(clockHandle); delete(clockHandle);
        end
    end

    function closeInspector(~,~)
        cleanupTimer(); delete(fig);
    end
end

function geometry = anchorGeometry(s,hasGoal)
% Use exact recorded arrays when present. Reconstruct legacy episodes only
% with an explicit provenance label and checks against deterministic counts.
    d=s.scenarioDiagnostics; p=s.params;
    geometry=struct('rawAnchors',zeros(3,0),'feasibleMask',false(1,0), ...
        'midpointMask',false(1,0),'source',"No arm anchors generated for this scenario.");
    if ~hasGoal || d.obstacleSamplingMode~="arm_sweep", return; end
    if isfield(d,'armSamplingGeometry')
        recorded=d.armSamplingGeometry;
        geometry.rawAnchors=recorded.rawAnchors;
        geometry.feasibleMask=recorded.feasibleMask;
        geometry.midpointMask=recorded.midpointMask;
        geometry.source="Anchors: exact recorded reset data.";
    else
        fractions=[.4 .5 .6];
        if isfield(p,'armSweepFractions'), fractions=p.armSweepFractions; end
        for k=1:numel(fractions)
            q=s.q; q(1:7)=s.q(1:7)+fractions(k)*(s.sampledGoalQ(1:7)-s.q(1:7));
            points=zeros(3,4);
            for link=1:4
                T=getTransform(s.robot,q,sprintf('panda_link%d',link+3));
                points(:,link)=T(1:3,4);
            end
            geometry.rawAnchors(:,end+(1:7))=[points,(points(:,1:3)+points(:,2:4))/2];
        end
        a=geometry.rawAnchors;
        lo=p.workspaceBox(:,1)+d.obstacleRadius; hi=p.workspaceBox(:,2)-d.obstacleRadius;
        near=min(max(a,lo),hi);
        far=lo+(hi-lo).*(abs(hi-a)>=abs(lo-a));
        geometry.feasibleMask=sqrt(sum((near-a).^2,1))<=p.obstacleOffsetRange(2) & ...
            sqrt(sum((far-a).^2,1))>=p.obstacleOffsetRange(1);
        geometry.midpointMask=repmat([false(1,4) true(1,3)],1,numel(fractions));
        geometry.source="Legacy anchors RECONSTRUCTED from current formulas; not recorded.";
    end
    assert(size(geometry.rawAnchors,2)==d.armAnchorsGenerated && ...
        nnz(geometry.feasibleMask)==d.armAnchorsRetained && ...
        nnz(~geometry.feasibleMask)==d.armAnchorsRejected, ...
        'Anchor counts differ from saved diagnostics; check sampler version.');
end

function group = pointLayer(ax,points,markerSize,color,marker,filled)
    group=hggroup('Parent',ax);
    if isempty(points), return; end
    h=scatter3(ax,points(1,:),points(2,:),points(3,:),markerSize,color,marker,'LineWidth',1.3);
    if filled, h.MarkerFaceColor=color; end
    h.Parent=group;
end

function [group,key] = candidateLayer(ax,d,maximum)
    group=hggroup('Parent',ax);
    key=['Pink: target overlap; red: start collision; purple: goal collision; ' ...
        'gray: workspace nominal point; ochre: RRT rejection.'];
    indices=find(d.candidateCodes~=4);
    if isempty(indices) || maximum==0, return; end
    indices=indices(round(linspace(1,numel(indices),min(maximum,numel(indices)))));
    colors=[.8 .25 .6;.85 .18 .15;.55 .23 .75;0 0 0;.42 .45 .48;.65 .46 .08];
    for code=[1 2 3 5 6]
        rows=indices(d.candidateCodes(indices)==code);
        pts=d.candidateCenters(rows,:).';
        if isempty(pts), continue; end
        h=scatter3(ax,pts(1,:),pts(2,:),pts(3,:),50,colors(code,:),'x','LineWidth',1.5);
        h.Parent=group;
    end
end

function group = workspaceLayer(ax,limits)
    group=hggroup('Parent',ax);
    x=limits(1,:); y=limits(2,:); z=limits(3,:);
    vertices=[x(1) y(1) z(1);x(2) y(1) z(1);x(2) y(2) z(1); ...
        x(1) y(2) z(1);x(1) y(1) z(2);x(2) y(1) z(2); ...
        x(2) y(2) z(2);x(1) y(2) z(2)];
    faces=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
    patch('Parent',group,'Vertices',vertices,'Faces',faces,'FaceColor','none', ...
        'EdgeColor',[.55 .63 .69],'LineWidth',.7);
end

function value = onOff(enabled)
    if enabled, value='on'; else, value='off'; end
end
