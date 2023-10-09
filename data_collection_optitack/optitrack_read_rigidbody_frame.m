%% !! run init_optitrack.m first !!
%% Get rigidBody, marker location, and quaternion Q of N rigid bodies

% Information from OptiTrack
frameOfData = theClient.GetLastFrameOfData();

%% rigidbody 1
nRigidBodies = frameOfData.nRigidBodies;
for n=1:nRigidBodies
    rigidBody{n} = frameOfData.RigidBodies(n); % Getting rigid body 1 information
end

%% rigidBody Marker Locations
% for n=1:nRigidBodies
%     nMarkers = rigidBody{n}.nMarkers; % Number of markers per rigid body
%     for i=1:nMarkers
%         X(i,:) = rigidBody{n}.Markers(i).x;
%         Y(i,:) = rigidBody{n}.Markers(i).y;
%     end
%     Markers{n} = [X Y];
%     X = [];
%     Y = [];
% end

%% rigidBody translation and rotation
for n=1:nRigidBodies
    x(n) = rigidBody{n}.x;
    y(n) = rigidBody{n}.y;
    z(n) = rigidBody{n}.z;

    qw(n) = rigidBody{n}.qw;
    qx(n) = rigidBody{n}.qx;
    qy(n) = rigidBody{n}.qy;
    qz(n) = rigidBody{n}.qz;

    Translation(:,n) = double([x(n);y(n);z(n)]);
    
    Opti_Q(:,n) = double([qw(n) qx(n) qy(n) qz(n)]');
    Opti_R(:,:,n) = quaternions2R(Opti_Q(:,n));

    Opti_theta(n) = -atan2(Opti_R(2,1,n),Opti_R(1,1,n)); % Wrapped -180 to 180 degrees
end