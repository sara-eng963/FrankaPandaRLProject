%% Compare collision-checking policies

% Check without our custom exclusions
[collisionDefault, D] = checkCollision(robot, qCandidate, ...
    "SkippedSelfCollisions", "parent", ...
    "Exhaustive", "on");

% Check with our custom exclusions
collisionFiltered = checkCollision(robot, qCandidate, ...
    "SkippedSelfCollisions", skipPairs);

fprintf("Parent-only exclusions: %d\n", collisionDefault);
fprintf("Custom exclusions:      %d\n", collisionFiltered);

% Report detected collisions
names = [robot.BodyNames, {robot.Base.Name}];
[row, col] = find(triu(isnan(D), 1));

for k = 1:numel(row)
    fprintf("Collision: %s <-> %s\n", ...
        names{row(k)}, names{col(k)});
end

% Visualize the exact configuration being checked
figure;
show(robot, qCandidate, ...
    "Visuals", "off", ...
    "Collisions", "on");

axis equal;
view(3);
title("Candidate Configuration - Collision Geometry");