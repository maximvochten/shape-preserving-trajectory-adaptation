function [traj_triggers,trajectories,invariants,sample_triggers] = load_invariant_trajectories(results_folder,nb)

traj_triggers = zeros(nb,1);
trajectories = cell(nb,1);
invariants = cell(nb,1);
sample_triggers = zeros(nb,1);

for i = 1:nb
    trajectory = load([results_folder,['traj',num2str(i-1),'.txt']]);
    invariant = load([results_folder,['invariants',num2str(i-1),'.txt']]);
    
    if i==1
        N1 = size(invariant,1);
    end
    
    traj_triggers(i) = trajectory(1,1);
    trajectories{i}.Obj_location = trajectory(:,11:13);
    N = size(trajectory,1);
    for j=1:N
        trajectories{i}.Obj_frames(:,:,j) = reshape(trajectory(j,2:10),3,3);
    end
    invariants{i} = invariant;
    
    sample_triggers(i) = N1-size(invariant,1)+1;
end
time0 = traj_triggers(1);
traj_triggers = traj_triggers - time0;

