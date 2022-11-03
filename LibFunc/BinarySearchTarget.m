function [Xdes, Ydes, Thetades, Kdes] = BinarySearchTarget(X)
    %% [二分查找]最接近X但小于X的目标点（仅根据X搜索）
%     load('LaneChangingTraj.mat');
    load('DoubleLaneChangingTraj.mat');
    LowPos = 1; HighPos = length(Traj_X);
    while  LowPos < HighPos-1
        MidPos = fix((LowPos + HighPos) / 2);
        if X > Traj_X(MidPos)
            LowPos = MidPos;
        elseif X < Traj_X(MidPos)
            HighPos = MidPos;
        else
            Xdes = Traj_X(MidPos); Ydes = Traj_Y(MidPos); Thetades = Traj_Theta(MidPos); Kdes = Traj_K(MidPos);
            return;
        end
    end
    if X > Traj_X(LowPos+1)
        Xdes = Traj_X(LowPos+1); Ydes = Traj_Y(LowPos+1); Thetades = Traj_Theta(LowPos+1); Kdes = Traj_K(LowPos+1);
    else
        Xdes = Traj_X(LowPos); Ydes = Traj_Y(LowPos); Thetades = Traj_Theta(LowPos); Kdes = Traj_K(LowPos);
    end
end