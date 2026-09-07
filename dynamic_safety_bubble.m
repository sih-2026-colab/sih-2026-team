function bubbleRadius = dynamic_safety_bubble(egoV, uncertainty)

    % Base safety radius
    baseRadius = 2.0;

    % Higher speed increases required safety space
    speedFactor = 0.15 * egoV;

    % Higher prediction uncertainty expands the bubble
    uncertaintyFactor = 2.0 * uncertainty;

    bubbleRadius = baseRadius + speedFactor + uncertaintyFactor;

end