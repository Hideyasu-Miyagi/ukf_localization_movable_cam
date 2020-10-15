%%
%
% script
%
%
close('all');
n = 1;
sqerr_pos_max = zeros(n, 3);
sqerr_pos_avg = zeros(n, 3);
for i=1:n
    disp('Type A');
    simrecdt_A = simrecdt_arr_A{i};
    err_pos_arr = cell(length(simrecdt_A));
    err_ori_arr = cell(length(simrecdt_A));
    for j=1:length(simrecdt_A)
        err_pos_arr{j} = (simrecdt_A{j}.pose_est(1:3, 4) - simrecdt_A{j}.vclstat_real.pose(1:3, 4));
    end
    sqerr_pos_arr = zeros(1, length(err_pos_arr));
    for j=1:length(err_pos_arr)
        sqerr_pos_arr(j) = err_pos_arr{j}.'*err_pos_arr{j};
    end
    sqerr_pos_max(i, 1) = max(sqerr_pos_arr);
    sqerr_pos_avg(i, 1) = mean(sqerr_pos_arr);
    disp(sprintf('sqerr_pos_max(%d,1):%f', i, sqerr_pos_max(i,1)));
    disp(sprintf('sqerr_pos_avg(%d,1):%f', i, sqerr_pos_avg(i,1)));
    for j=1:length(simrecdt_A)
        err_ori_arr_mtx = logm(simrecdt_A{j}.pose_est(1:3, 1:3) / simrecdt_A{j}.vclstat_real.pose(1:3, 1:3));
        err_ori_arr{j} = [err_ori_arr_mtx(3,2); err_ori_arr_mtx(1,3); err_ori_arr_mtx(2,1)];
    end
    sqerr_ori_arr = zeros(1, length(err_ori_arr));
    for j=1:length(err_ori_arr)
        sqerr_ori_arr(j) = err_ori_arr{j}.'*err_ori_arr{j};
    end
    sqerr_ori_max(i, 1) = max(sqerr_ori_arr);
    sqerr_ori_avg(i, 1) = mean(sqerr_ori_arr);
    disp(sprintf('sqerr_ori_max(%d,1):%f', i, sqerr_ori_max(i,1)));
    disp(sprintf('sqerr_ori_avg(%d,1):%f', i, sqerr_ori_avg(i,1)));
end
for i=1:n
    disp('Type B');
    simrecdt_B = simrecdt_arr_B{i};
    err_pos_arr = cell(length(simrecdt_B));
    err_ori_arr = cell(length(simrecdt_B));
    for j=1:length(simrecdt_B)
        err_pos_arr{j} = (simrecdt_B{j}.pose_est(1:3, 4) - simrecdt_B{j}.vclstat_real.pose(1:3, 4));
    end
    sqerr_pos_arr = zeros(1, length(err_pos_arr));
    for j=1:length(err_pos_arr)
        sqerr_pos_arr(j) = err_pos_arr{j}.'*err_pos_arr{j};
    end
    sqerr_pos_max(i, 2) = max(sqerr_pos_arr);
    sqerr_pos_avg(i, 2) = mean(sqerr_pos_arr);
    disp(sprintf('sqerr_pos_max(%d,2):%f', i, sqerr_pos_max(i,2)));
    disp(sprintf('sqerr_pos_avg(%d,2):%f', i, sqerr_pos_avg(i,2)));
    for j=1:length(simrecdt_B)
        err_ori_arr_mtx = logm(simrecdt_B{j}.pose_est(1:3, 1:3) / simrecdt_B{j}.vclstat_real.pose(1:3, 1:3));
        err_ori_arr{j} = [err_ori_arr_mtx(3,2); err_ori_arr_mtx(1,3); err_ori_arr_mtx(2,1)];
    end
    sqerr_ori_arr = zeros(1, length(err_ori_arr));
    for j=1:length(err_ori_arr)
        sqerr_ori_arr(j) = err_ori_arr{j}.'*err_ori_arr{j};
    end
    sqerr_ori_max(i, 2) = max(sqerr_ori_arr);
    sqerr_ori_avg(i, 2) = mean(sqerr_ori_arr);
    disp(sprintf('sqerr_ori_max(%d,1):%f', i, sqerr_ori_max(i,1)));
    disp(sprintf('sqerr_ori_avg(%d,1):%f', i, sqerr_ori_avg(i,1)));
end
for i=1:n
    disp('Type C');
    simrecdt_C = simrecdt_arr_C{i};
    err_pos_arr = cell(length(simrecdt_C));
    err_ori_arr = cell(length(simrecdt_C));
    for j=1:length(simrecdt_C)
        err_pos_arr{j} = (simrecdt_C{j}.pose_est(1:3, 4) - simrecdt_C{j}.vclstat_real.pose(1:3, 4));
    end
    sqerr_pos_arr = zeros(1, length(err_pos_arr));
    for j=1:length(err_pos_arr)
        sqerr_pos_arr(j) = err_pos_arr{j}.'*err_pos_arr{j};
    end
    sqerr_pos_max(i, 3) = max(sqerr_pos_arr);
    sqerr_pos_avg(i, 3) = mean(sqerr_pos_arr);
    disp(sprintf('sqerr_pos_max(%d,3):%f', i, sqerr_pos_max(i,3)));
    disp(sprintf('sqerr_pos_avg(%d,3):%f', i, sqerr_pos_avg(i,3)));
    for j=1:length(simrecdt_C)
        err_ori_arr_mtx = logm(simrecdt_C{j}.pose_est(1:3, 1:3) / simrecdt_C{j}.vclstat_real.pose(1:3, 1:3));
        err_ori_arr{j} = [err_ori_arr_mtx(3,2); err_ori_arr_mtx(1,3); err_ori_arr_mtx(2,1)];
    end
    sqerr_ori_arr = zeros(1, length(err_ori_arr));
    for j=1:length(err_ori_arr)
        sqerr_ori_arr(j) = err_ori_arr{j}.'*err_ori_arr{j};
    end
    sqerr_ori_max(i, 3) = max(sqerr_ori_arr);
    sqerr_ori_avg(i, 3) = mean(sqerr_ori_arr);
    disp(sprintf('sqerr_ori_max(%d,1):%f', i, sqerr_ori_max(i,1)));
    disp(sprintf('sqerr_ori_avg(%d,1):%f', i, sqerr_ori_avg(i,1)));
end
disp('Done.');
figure(1);
scatter(1:n, sqerr_pos_avg(1:n, 1), [], [0, 0, 1], 'o');
hold on;
scatter(1:n, sqerr_pos_avg(1:n, 2), [], [0, 0.5, 0], 'o');
hold on;
scatter(1:n, sqerr_pos_avg(1:n, 3), [], [1, 0, 0], 'o');
legend('Method A', 'Method B', 'Method C');
ax = gca;
ax.YLabel.String = 'Square error of vehicle position [m^2]';
title('Average square error of the estimate of 3D position')
figure(2);
scatter(1:n, sqerr_pos_max(1:n, 1), [], [0, 0.5, 1], 'x');
hold on;
scatter(1:n, sqerr_pos_max(1:n, 2), [], [0, 1, 0.0], 'x');
hold on;
scatter(1:n, sqerr_pos_max(1:n, 3), [], [1, 0.5, 0], 'x');
legend('Method A', 'Method B', 'Method C');
ax = gca;
ax.YLabel.String = 'Square error of vehicle position [m^2]';
title('Maximum square error of the estimate of 3D position')
figure(3);
scatter(1:n, sqerr_ori_avg(1:n, 1), [], [0, 0, 1], 'o');
hold on;
scatter(1:n, sqerr_ori_avg(1:n, 2), [], [0, 0.5, 0], 'o');
hold on;
scatter(1:n, sqerr_ori_avg(1:n, 3), [], [1, 0, 0], 'o');
legend('Method A', 'Method B', 'Method C');
ax = gca;
ax.YLabel.String = 'Square error of vehicle position [rad^2]';
title('Average square error of the estimate of 3D orientation')
figure(4);
scatter(1:n, sqerr_ori_max(1:n, 1), [], [0, 0.5, 1], 'x');
hold on;
scatter(1:n, sqerr_ori_max(1:n, 2), [], [0, 1, 0.0], 'x');
hold on;
scatter(1:n, sqerr_ori_max(1:n, 3), [], [1, 0.5, 0], 'x');
legend('Method A', 'Method B', 'Method C');
ax = gca;
ax.YLabel.String = 'Square error of vehicle position [rad^2]';
title('Maximum square error of the estimate of 3D orientation')
