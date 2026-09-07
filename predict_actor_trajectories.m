function predictions = predict_actor_trajectories(actors, horizons)

    % predictions(i,j) = predicted state of actor i
    % at future horizon horizons(j)

    numActors = length(actors);
    numHorizons = length(horizons);

    predictions = struct([]);

    for i = 1:numActors

        for j = 1:numHorizons

            h = horizons(j);

            % Constant-acceleration prediction
            futureX = actors(i).x + ...
                      actors(i).vx * h + ...
                      0.5 * actors(i).ax * h^2;

            futureY = actors(i).y + ...
                      actors(i).vy * h + ...
                      0.5 * actors(i).ay * h^2;

            futureVx = actors(i).vx + ...
                       actors(i).ax * h;

            futureVy = actors(i).vy + ...
                       actors(i).ay * h;

            predictions(i,j).id = actors(i).id;
            predictions(i,j).name = actors(i).name;

            predictions(i,j).time = h;

            predictions(i,j).x = futureX;
            predictions(i,j).y = futureY;

            predictions(i,j).vx = futureVx;
            predictions(i,j).vy = futureVy;

            % Uncertainty grows with prediction time
            predictions(i,j).uncertainty = ...
                actors(i).uncertainty + 0.35 * h;

        end

    end

end