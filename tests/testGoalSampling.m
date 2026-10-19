%% Focused Cartesian-first, position-only IK sampler checks.
projectRoot=fileparts(fileparts(mfilename('fullpath')));
addpath(projectRoot);
run(fullfile(projectRoot,'Scripts','robot_setup.m'));
rngBefore=rng;
checkSampler(robot,eeFrame,qReference,jointLimits,skipPairs,cfg);
assert(isequal(rngBefore,rng));
disp('PASS: position-only IK goals, exact checks, bounded failure and RNG reproducibility.');

function checkSampler(robot,eeFrame,qStart,jointLimits,skipPairs,cfg)
    caller=RandStream.getGlobalStream;
    guard=onCleanup(@() RandStream.setGlobalStream(caller));
    RandStream.setGlobalStream(RandStream('mt19937ar','Seed',314159));
    p=struct('workspaceBox',cfg.box,'jointLimits',jointLimits, ...
        'skipPairs',{skipPairs},'maxTargetAttempts',40,'minTargetDistance',0.15, ...
        'maxTargetDistance',0.40,'ikMaxInitialGuesses',3, ...
        'ikPositionTolerance',0.001,'positionTolerance',cfg.posTol);
    p.ikSolver=inverseKinematics('RigidBodyTree',robot,'SolverAlgorithm','LevenbergMarquardt');
    p.ikSolver.SolverParameters=struct('MaxIterations',100,'MaxTime',1, ...
        'SolutionTolerance',1e-6,'AllowRandomRestart',false,'EnforceJointLimits',true);
    TStart=kinematics.forwardKinematics(robot,qStart,eeFrame);
    samples=cell(12,3);
    for pass=1:2
        RandStream.setGlobalStream(RandStream('mt19937ar','Seed',314159));
        for k=1:12
            [xyz,q,d]=environment.sampleGoalConfiguration(robot,eeFrame,qStart,p);
            assert(d.success && isequal(size(xyz),[3 1]) && isequal(size(q),[1 9]));
            assert(d.ikCalls<=p.maxTargetAttempts*p.ikMaxInitialGuesses);
            assert(d.targetAttempts==d.distanceRejects+d.targetIKRejects+1);
            assert(d.ikCalls==d.ikFailures+1 && d.solverTimeLimits==0);
            assert(all(xyz>=cfg.box(:,1) & xyz<=cfg.box(:,2)));
            assert(all(q(1:7)>=jointLimits(:,1).' & q(1:7)<=jointLimits(:,2).'));
            assert(isequal(q(8:9),qStart(8:9)));
            T=kinematics.forwardKinematics(robot,q,eeFrame);
            assert(norm(T(1:3,4)-xyz)<=p.ikPositionTolerance);
            assert(all(T(1:3,4)>=cfg.box(:,1) & T(1:3,4)<=cfg.box(:,2)));
            distance=norm(xyz-TStart(1:3,4));
            assert(distance>=p.minTargetDistance && distance<=p.maxTargetDistance);
            assert(~checkCollision(robot,q,'SkippedSelfCollisions',skipPairs));
            if pass==1
                samples(k,:)={xyz,q,rmfield(d,'seconds')};
            else
                assert(isequal(samples(k,1:2),{xyz,q}));
                assert(isequaln(samples{k,3},rmfield(d,'seconds')));
            end
        end
        if pass==1, firstRng=rng; else, assert(isequal(firstRng,rng)); end
    end
    % Rotating the requested transform has no effect when orientation weights
    % are zero: the feasibility problem contains only the same XYZ constraint.
    desired=eye(4); desired(1:3,4)=samples{1,1};
    rotated=desired; rotated(1:3,1:3)=[0 -1 0;1 0 0;0 0 1];
    qa=p.ikSolver(eeFrame,desired,[0 0 0 1 1 1],qStart);
    qb=p.ikSolver(eeFrame,rotated,[0 0 0 1 1 1],qStart);
    assert(norm(qa-qb)<1e-9,'Orientation affected position-only IK.');

    tooFar=p; tooFar.minTargetDistance=2; tooFar.maxTargetDistance=3;
    tooFar.maxTargetAttempts=3;
    [xyz,q,d]=environment.sampleGoalConfiguration(robot,eeFrame,qStart,tooFar);
    assert(isempty(xyz) && isempty(q) && ~d.success);
    assert(d.targetAttempts==3 && d.distanceRejects==3 && d.ikCalls==0);
    % Force numerical search exhaustion with a distant synthetic test box.
    % This checks bounded termination, not an unreachable-space classifier.
    failed=p; failed.workspaceBox=[3 3.01;3 3.01;3 3.01];
    failed.minTargetDistance=0; failed.maxTargetDistance=Inf;
    failed.maxTargetAttempts=2; failed.ikMaxInitialGuesses=2;
    [xyz,q,d]=environment.sampleGoalConfiguration(robot,eeFrame,qStart,failed);
    assert(~d.success && isempty(xyz) && isempty(q));
    assert(d.targetAttempts==2 && d.targetIKRejects==2 && d.ikCalls==4 && d.ikFailures==4);
    % Static obstacle checks must still reject otherwise valid IK solutions.
    blocked=p; blocked.workspaceBox=[TStart(1:3,4)-1e-5 TStart(1:3,4)+1e-5];
    blocked.minTargetDistance=0; blocked.maxTargetDistance=Inf;
    blocked.maxTargetAttempts=2; blocked.ikMaxInitialGuesses=2;
    sphere=collisionSphere(2); sphere.Pose=trvec2tform(TStart(1:3,4).');
    [xyz,q,d]=environment.sampleGoalConfiguration(robot,eeFrame,qStart,blocked,sphere);
    assert(~d.success && isempty(xyz) && isempty(q) && d.collisionRejects>0);
    assert(d.ikCalls<=4);
end
