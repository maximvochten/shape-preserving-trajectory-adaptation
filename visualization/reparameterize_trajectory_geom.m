function [meas_traj_geom,s,theta] = reparameterize_trajectory_geom(meas_traj_time,dt)

R = meas_traj_time.Obj_frames;
p = meas_traj_time.Obj_location;

N = length(p);
omega = zeros(N-1,3);
for j=1:N-1
    DeltaR = logm(R(:,:,j)'*R(:,:,j+1));
    omega(j,1:3) = [ -DeltaR(2,3) DeltaR(1,3) -DeltaR(1,2) ]/dt;
end
Pdot = diff(p)/dt;

omeganorm = sqrt(sum(omega.^2,2));
vnorm = sqrt(sum(Pdot.^2,2));

theta = [ 0 ; cumsum(omeganorm)*dt ];
s = [ 0 ; cumsum(vnorm)*dt ];

% note: this will fail if you duplicate data somewhere, you can remove it with unique function
[XI, ia, ~] = unique(p,'rows');
meas_traj_geom.Obj_location = interp1(s(ia),unique(p,'rows'),linspace(0,s(end),N));
meas_traj_geom.Obj_frames = interpR(theta,R,linspace(0,theta(end),N));

end