
function in = resetPandaSimulink(in, initialState)
% Initialize a Simulink episode using the existing Panda reset logic.

    [obs, s] = environment.resetEpisodeWithRetries(initialState);

    state = [
        s.q(1:7).';
        s.pEE(:);
        s.target(:);
        s.obstacle.Pose(1:3,4);
        s.obstacle.Radius;
        s.stepCount;
        s.failureCode
    ];

    in = setVariable(in, "episodeInitialState", state);
    in = setVariable(in, "episodeInitialObservation", obs);
end
