%% Test for actor creation
actor = createActor();
assert(isstruct(actor));
assert(strcmp(actor.type, 'vehicle'));
