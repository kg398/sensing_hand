for i=1:10
    temp=table2array(readtable(sprintf("exp1_rec/test_1650898610_traj_kp_shift_replay%d_data.csv",i)));
    pose(i,:,:)=[temp(1:200,15:46)];
end
newmuX=squeeze(mean(mean(pose(:,:,:))));
newsigmaX=squeeze(std(std(pose(:,:,:))));