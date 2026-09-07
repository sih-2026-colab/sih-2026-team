function actors = update_highway_actors(actors, dt)

    lane3 = 7.0;

    for i = 1:length(actors)

        % Update velocities
        actors(i).vx = actors(i).vx + actors(i).ax * dt;
        actors(i).vy = actors(i).vy + actors(i).ay * dt;

        % Update positions
        actors(i).x = actors(i).x + actors(i).vx * dt;
        actors(i).y = actors(i).y + actors(i).vy * dt;

    end


    % =====================================================
    % CUT-IN VEHICLE
    % Stop its lateral movement after entering lane 3
    % =====================================================

    cutInIndex = 3;

    if actors(cutInIndex).y >= lane3

        actors(cutInIndex).y = lane3;
        actors(cutInIndex).vy = 0;

    end

end