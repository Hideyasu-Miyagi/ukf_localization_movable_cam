%% Simulatation program for an IEEJ paper
%  Visualize effect of precise calculation of error covariance matrix related to self-localization
%  2020-05-25 H.Miyagi
%

%% Main function (program entry)
%
function [simrecdt_ret, fh_arr] = IEEJ_paper2020_main(simrecdt, varargin)
    %ppr_check_cov_6x6();
    %return;
    %%
    arg_noldmk = '-';
    arg_noldmk_reuse = '-';
    arg_nocovmul = '-';
    arg_camfixed = '-';
    figureidx = 1;
    flg_figure = false;
    for i=1:(nargin-1)
        if strcmp(varargin(i), 'noldmk')
            disp('noldmk');
            arg_noldmk = 'noldmk';
        elseif strcmp(varargin(i), 'noldmk_reuse')
            disp('noldmk_reuse');
            arg_noldmk_reuse = 'noldmk_reuse';
        elseif strcmp(varargin(i), 'nocovmul')
            disp('nocovmul');
            arg_nocovmul = 'nocovmul';
        elseif strcmp(varargin(i), 'camfixed')
            disp('camfixed');
            arg_camfixed = 'camfixed';
        elseif strcmp(varargin(i), 'figure')
            disp('figure');
            flg_figure = true;
        elseif flg_figure
            figureidx = floor(str2double(varargin(i)));
            flg_figure = false;
        elseif strcmp(varargin(i), 'savefig')
            disp('loop');
            flg_savefig = true;
        end
    end
    if iscell(simrecdt)
        % close all;
        simrecdt_ret = ppr_simulate(22, simrecdt, arg_noldmk, arg_nocovmul, arg_camfixed, arg_noldmk_reuse);    % simulation 
        fh_arr = ppr_plot(simrecdt_ret, figureidx);
    else
        % close all;
        simrecdt_ret = ppr_simulate(22, false, arg_noldmk, arg_nocovmul, arg_camfixed, arg_noldmk_reuse);    % simulation
        fh_arr = ppr_plot(simrecdt_ret, figureidx);
    end
    % input('Simulation done. Enter any key to playback without landmark detection:');
    %%
end

%% Plot pose of vehicle
% 
function fh_arr = ppr_plot(playdt, figure_idx)
    fh_arr = cell(0);
    x_real_arr = [];
    y_real_arr = [];
    z_real_arr = [];
    x_est_arr  = [];
    y_est_arr  = [];
    z_est_arr  = [];
    x_est_upd_arr = [];
    y_est_upd_arr = [];
    z_est_upd_arr = [];
    t_arr = [];
    detP_est_arr    = [];
    detPtrs_est_arr = [];
    detProt_est_arr = [];
    traceP_est_arr    = [];
    tracePtrs_est_arr = [];
    traceProt_est_arr = [];
    x_msr_arr = [];
    y_msr_arr = [];
    z_msr_arr = [];
    sqerr_trs_arr = [];
    sqerr_rot_arr = [];
    for i = 1:length(playdt)
        t_arr = cat(2, t_arr, playdt{i}.t);
        x_real_arr = cat(2, x_real_arr, playdt{i}.vclstat_real.pose(1,4));
        y_real_arr = cat(2, y_real_arr, playdt{i}.vclstat_real.pose(2,4));
        z_real_arr = cat(2, z_real_arr, playdt{i}.vclstat_real.pose(3,4));
        x_est_arr  = cat(2, x_est_arr,  playdt{i}.pose_est(1,4));
        y_est_arr  = cat(2, y_est_arr,  playdt{i}.pose_est(2,4));
        z_est_arr  = cat(2, z_est_arr,  playdt{i}.pose_est(3,4));
        if (isfield(playdt{i}, 'upd_flg') && playdt{i}.upd_flg)
            x_est_upd_arr = cat(2, x_est_upd_arr, playdt{i}.pose_est(1,4));
            y_est_upd_arr = cat(2, y_est_upd_arr, playdt{i}.pose_est(2,4));
            z_est_upd_arr = cat(2, z_est_upd_arr, playdt{i}.pose_est(3,4));
        end
        sqerr_trs = sum((playdt{i}.vclstat_real.pose(1:3,4)-playdt{i}.pose_est(1:3,4)).^2);
        sqerr_rot = sum((ppr_antisym_m2v(logm(playdt{i}.vclstat_real.pose(1:3,1:3)/playdt{i}.pose_est(1:3,1:3)))).^2);
        sqerr_trs_arr = cat(2, sqerr_trs_arr, sqerr_trs);
        sqerr_rot_arr = cat(2, sqerr_rot_arr, sqerr_rot);
        detP_est_arr = cat(2, detP_est_arr, det(playdt{i}.cov_pose_est));
        detPtrs_est_arr = cat(2, detPtrs_est_arr, det(playdt{i}.cov_pose_est(4:6,4:6)));
        detProt_est_arr = cat(2, detProt_est_arr, det(playdt{i}.cov_pose_est(1:3,1:3)));
        traceP_est_arr = cat(2, traceP_est_arr, trace(playdt{i}.cov_pose_est));
        tracePtrs_est_arr = cat(2, tracePtrs_est_arr, trace(playdt{i}.cov_pose_est(4:6,4:6)));
        traceProt_est_arr = cat(2, traceProt_est_arr, trace(playdt{i}.cov_pose_est(1:3,1:3)));
        % disp('(x,y):'); disp(hist{i}.pose_real(1,4)); disp(hist{i}.pose_real(2,4)); 
        if (isfield(playdt{i}, 'pose_msr'))
            x_msr_arr  = cat(2, x_msr_arr,  playdt{i}.pose_msr(1,4));
            y_msr_arr  = cat(2, y_msr_arr,  playdt{i}.pose_msr(2,4));
            z_msr_arr  = cat(2, z_msr_arr,  playdt{i}.pose_msr(3,4));
        end
    end
    fh_arr{1+length(fh_arr)} = figure(figure_idx);
    cla reset;
    %subplot(3, 1, 1);
    hold on;
    
    plot(x_real_arr, y_real_arr,            'Color', [0, 0, 1]);
    plot(x_est_arr,  y_est_arr,             'Color', [1, 0, 0]);
    plot(x_est_upd_arr, y_est_upd_arr, 'o', 'Color', [1, 0, 0]);
    plot(x_msr_arr,  y_msr_arr,        'o', 'Color', [0, 1, 0]); % 'Color', [0, 1, 0]);
    legend('Actual path', 'Path estimated with UKF', 'Position innovated by landmark', 'Position calculated by landmark');
    gca.XLabel.String = 'X [m]';
    gca.YLabel.String = 'Y [m]';
    for i=1:length(playdt)
        % quiver(x_real_arr(i), y_real_arr(i), playdt{i}.vclstat_real.pose(1, 1), playdt{i}.vclstat_real.pose(2,1), 'Color', [0, 0.2, 1]);
        if (isfield(playdt{i}, 'upd_flg') && playdt{i}.upd_flg)
            % text(playdt{i}.pose_est(1,4), playdt{i}.pose_est(2,4), sprintf('%d', i), 'HorizontalAlignment','center', 'VerticalAlignment','bottom', 'Color', [1, 0, 0]);
            % text(playdt{i}.pose_msr(1,4), playdt{i}.pose_msr(2,4), sprintf('%d', i), 'HorizontalAlignment','center', 'VerticalAlignment','bottom', 'Color', [0, 1, 0]);
            % quiver(playdt{i}.pose_est(1,4), playdt{i}.pose_est(2,4), playdt{i}.pose_est(1, 1)*0.1, playdt{i}.pose_est(2,1)*0.1, 'Color', [1, 0, 0.5]);
            % quiver(playdt{i}.pose_msr(1,4), playdt{i}.pose_msr(2,4), playdt{i}.pose_msr(1, 1)*0.1, playdt{i}.pose_msr(2,1)*0.1, 'Color', [0, 1, 0.5]);
        end
    end
    fh_arr{1+length(fh_arr)} = figure(figure_idx+1);
    %subplot(3,1,2);
    hold on;
    plot(t_arr, sqerr_trs_arr, 'Color', [1, 0, 0]);
    plot(t_arr, sqerr_rot_arr, 'Color', [0, 1, 0]);
    text(t_arr(length(t_arr)), sqerr_trs_arr(length(t_arr)), [ 'sqerr avg (trs)', num2str(sum(sqerr_trs_arr)/length(t_arr)) ], 'HorizontalAlignment', 'right', 'Color', [0.5, 0, 0]);
    text(t_arr(length(t_arr)), sqerr_rot_arr(length(t_arr)), [ 'sqerr avg (rot)', num2str(sum(sqerr_rot_arr)/length(t_arr)) ], 'HorizontalAlignment', 'right', 'Color', [0, 0.5, 0]);

    fh_arr{1+length(fh_arr)} = figure(figure_idx+2);
    %subplot(3,1,3);
    hold on;
    %plot(t_arr, log10(detP_est_arr),    'Color', [0, 0, 1]);
    %plot(t_arr, log10(detProt_est_arr), 'Color', [0, 1, 0]);
    %plot(t_arr, log10(detPtrs_est_arr), 'Color', [1, 0, 0]);

    plot(t_arr, traceP_est_arr,    'Color', [0, 0, 1]);
    plot(t_arr, traceProt_est_arr, 'Color', [0, 1, 0]);
    plot(t_arr, tracePtrs_est_arr, 'Color', [1, 0, 0]);

    % figure;
end


%% Do simulation (or just playback simulated data and do UKF)
% input: t_max: time to end simulation or playback
%        playdt : result of past simulation to play back; 
%                 false when simulate
%        flg_noldmk : 1:suppress landmark detection 0:do landmark detection
% output: recdt : result of simulation 
function [recdt] = ppr_simulate(t_max, playdt, varargin)
    global gsim;
    global gvcl;
    global gldmk;
    
    flg_noldmk = false;
    flg_noldmk_reuse = false;
    flg_nocovmul = false;
    flg_camfixed = false;
    arg_camfixed = '-';
    for i=1:(nargin-2)
        if strcmp(varargin(i), 'noldmk')
            disp('noldmk detected');
            flg_noldmk= true;
        elseif strcmp(varargin(i), 'nocovmul')
            disp('nocovmul detected');
            flg_nocovmul = true;
        elseif strcmp(varargin(i), 'camfixed')
            disp('camfixed detected');
            flg_camfixed = true;
            arg_camfixed = 'camfixed';
        elseif strcmp(varargin(i), 'noldmk_reuse')
            disp('noldmk_reuse detected');
            %input('Press enter key ...:');
            flg_noldmk_reuse = true;
        end
    end
    
    ppr_set_constants();
    k = 1;
    t = 0;
  
    recdt = cell(0);
  
    if (iscell(playdt) == 0)
        flg_playback = false;
        vclstat_real = ppr_gen_vclstat_real(t, arg_camfixed);
    else
        flg_playback = true;
        %vclstat_real = playdt{k}.vclstat_real;
        vclstat_real = ppr_retrieve_vclstat_real(playdt, k, arg_camfixed);
    end
    recdt{k}.vclstat_real = vclstat_real;
  
    pose_est = vclstat_real.pose;
    %pose_est = pose_est * [
    %    0.5^0.5, -0.5^0.5, 0, 0;
    %    0.5^0.5,  0.5^0.5, 0, 0;
    %          0,        0, 1, 0;
    %          0,        0, 0, 1 ];
    % cov_pose_est = zeros(6,6);
    cov_pose_est = diag([0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]);
    recdt{k}.t = t;
    recdt{k}.pose_est = pose_est;
    recdt{k}.cov_pose_est = cov_pose_est;
    k = k + 1;
    t = t + gsim.dt;
  
    while (true)
        if t_max < t
            break
        end
        recdt{k}.t = t;
        %disp('k:'); disp(k);
        %disp('t:'); disp(t);
        % [vx, vy, omega]
        
        if (flg_playback == false)
            vclstat_real = ppr_gen_vclstat_real(t, arg_camfixed);
        else
            %vclstat_real = playdt{k}.vclstat_real;
            vclstat_real = ppr_retrieve_vclstat_real(playdt, k, arg_camfixed);
        end
        recdt{k}.vclstat_real = vclstat_real;

        %if (305 <= k)
        %    return;
        %end
        % Measure velocity and angular velocity (=twist_est)
        if (flg_playback == false)
            twist_est = ppr_measure_odom(vclstat_real);
        else
            twist_est = playdt{k}.twist_est;
        end
        recdt{k}.twist_est = twist_est;
        % input('Press enter key..');
      
        % Calculate covariance of twist
        vx_est = twist_est(1,4);
        cov_twist_est = ppr_estimate_cov_twist(vx_est);
      
        % Detect landmarks using camera
        if flg_noldmk
            c2l_info = cell(0);  % empty cell
        else
            if flg_playback == false || flg_noldmk_reuse
                disp('no_ldmk_reuse');
                [c2l_info] = ppr_detect_landmark(vclstat_real);
            else
                c2l_info = playdt{k}.c2l_info;
            end
            recdt{k}.c2l_info = c2l_info;
        end
      
        % Calculate covariance of mtx_b2c and mtx_c2l
        omega_est = -twist_est(1,2);
        steer_est = atan2(gvcl.L * omega_est, vx_est);
        %steer_est = atan2(gvcl.L * (-vclstat_real.twist(1,2)), vclstat_real.twist(1,4));
        %disp('pose_est:'); disp(pose_est);

        upd_flg = false;
        % UKF 
        if (flg_noldmk || isempty(c2l_info))
            [pose_est, cov_pose_est] = ppr_UKF_only_predict(pose_est, cov_pose_est, twist_est, cov_twist_est, 0);
        else
            ldmk_id = c2l_info{1}.id;
            mtx_m2l = [
                eye(3,3),     gldmk.pos{ldmk_id}.'      ;
                zeros(1,3),   1                         ];
            if flg_camfixed
                [mtx_b2c_est, cov_b2c_est] = ppr_estimate_b2c_camfixed();
            else
                [mtx_b2c_est, cov_b2c_est] = ppr_estimate_b2c(steer_est);
            end
            if flg_nocovmul
                cov_b2c_est = zeros(6,6);
            end
            mtx_c2l_msr = c2l_info{1}.mtx;
            cov_c2l_msr = ppr_estimate_cov_c2l(c2l_info{1}.mtx);
            mtx_b2l_est = mtx_b2c_est * mtx_c2l_msr;
            cov_b2l_est = ppr_calc_cov_of_mul(mtx_b2c_est, cov_b2c_est, mtx_c2l_msr, cov_c2l_msr);
            mtx_l2b_est = inv(mtx_b2l_est);
            cov_l2b_est = ppr_calc_cov_of_inv(mtx_b2l_est, cov_b2l_est);
            pose_msr = mtx_m2l / mtx_b2l_est;
            %disp('pose_msr:'); disp(pose_msr);
            %disp('pose_real:'); disp(vclstat_real.pose);
            %disp('mtx_b2c:'); disp(vclstat_real.mtx_b2c);
            %input('Enter any key:');
            if flg_nocovmul
                %cov_pose_msr = ppr_calc_cov_of_mul(mtx_m2l, zeros(6,6), inv(mtx_c2l_msr), ppr_calc_cov_of_inv(mtx_c2l_msr, cov_c2l_msr));
                cov_pose_msr = ppr_calc_cov_of_mul(mtx_m2l, zeros(6,6), mtx_l2b_est, cov_l2b_est);
            else
                cov_pose_msr = ppr_calc_cov_of_mul(mtx_m2l, zeros(6,6), mtx_l2b_est, cov_l2b_est);
            end
            %disp('det(cov_c2l_msr):'); disp(det(cov_c2l_msr));
            %disp('det(cov_b2c_est):'); disp(det(cov_b2c_est));
            %disp('det(cov_b2l_est):'); disp(det(cov_b2l_est));
            %disp('det(cov_l2b_est):'); disp(det(cov_l2b_est));
            %disp('cov_pose_msr:'); disp(cov_pose_msr);
            %disp('det(cov_pose_msr):'); disp(det(cov_pose_msr));
            recdt{k}.pose_msr = pose_msr;
            dbg_err_rot = pose_msr(1:3, 1:3) / vclstat_real.pose(1:3, 1:3);
            dbg_vlog_err_rot = ppr_antisym_m2v((dbg_err_rot));
            if (15*pi/180.0 <= norm(dbg_vlog_err_rot))
                disp('dbg_err_rot:'); disp(dbg_err_rot);
                disp('error angle[deg]:'); disp(norm(dbg_vlog_err_rot)*180.0/pi);
                disp('mtx_b2c:'); disp(vclstat_real.mtx_b2c);
                disp('b2c angle [deg] mtx_b2c:'); disp(norm(ppr_antisym_m2v(logm(vclstat_real.mtx_b2c)))*180.0/pi);
                %input('angle error more than 5deg. Enter any key..');
            end
            if (0.5 <= norm(pose_msr(1:3, 4)-vclstat_real.pose(1:3, 4)))
                disp('pose_msr(1:3, 4):'); disp(pose_msr(1:3, 4));
                disp('pose_real(1:3, 4):');     disp(vclstat_real.pose(1:3, 4));
                disp('cov_pose_msr:');     disp(cov_pose_msr);
                %input('position error more than 0.5m. Enter any key..');
            end
            %input('Enter key:');
            [pose_est, cov_pose_est, pose_prd, cov_pose_prd, G] = ppr_UKF(pose_est, cov_pose_est, twist_est, cov_twist_est, pose_msr, cov_pose_msr, 0);
            upd_flg = true;
            recdt{k}.pose_prd     = pose_prd;
            recdt{k}.cov_pose_prd = cov_pose_prd;
            recdt{k}.G            = G;
        end
        recdt{k}.pose_est     = pose_est;
        recdt{k}.cov_pose_est = cov_pose_est;
        recdt{k}.upd_flg      = upd_flg;

        % Time-evolute vehicle state 
        %if (flg_playback == false)
        %    vclstat_real = ppr_gen_vclstat_real(t, arg_camfixed);
        %else
        %    %vclstat_real = playdt{k}.vclstat_real;
        %    vclstat_real = ppr_retrieve_vclstat_real(playdt, k, arg_camfixed);
        %end
        %recdt{k}.vclstat_real = vclstat_real;
        k = k + 1;
        t = t + gsim.dt;
    end
    global xxx;
    xxx = 3; % recdt{k};
    % disp('xxx'); disp(xxx);
    % input('xxx updated. Press enter key..');
end

%% Set constant values
%
function ppr_set_constants()
    global gpath;
    global gvcl;
    global gldmk;
    global gcam;
    global gsim;
    % simulation
    gsim.dt = 0.04;         % 40ms -> 25Hz
    %gsim.dt = 0.1;         % 40ms -> 25Hz
    %gsim.dt = 1.0;         % 1s -> 1Hz
    
    % path
    gpath.x_line_min = 2.0;
    gpath.x_line_max = 8.0;
    gpath.y_line_min = 0.5;
    gpath.y_line_max = 3.5;
    gpath.y_mid = (gpath.y_line_min + gpath.y_line_max)*0.5;
    gpath.R = 1.5;  % radius: 1.5
    gpath.L_total_path = 2*((gpath.x_line_max-gpath.x_line_min)+pi*gpath.R);
    gpath.v = 1.0;  % m/s
    gpath.slope = 0.05;   % max. slope 5%

    % landmarks
    gldmk.pos = struct([]);
    gldmk.pos{1} = [ 0.25, 2.0, 1.0];
    gldmk.pos{2} = [ 9.75, 2.0, 1.0];
    gldmk.size = 0.2;

    % vehicle
    gvcl.L = 1.0;   % 1.0[m]
    gvcl.W = 0.6;   % 0.6[m]
    gvcl.camera_z      = 1.0;
    gvcl.sigma_steer   = 0.2;   % 0.1 [rad]
    gvcl.sigma_vel     = 0.2;   % 0.3 [m/s]
    gvcl.sigma_osc_rot = 0.01;  % 0.01 [rad]  vehicle oscillation in orientation 
    gvcl.sigma_osc_trs = 0.01;  % 0.01[m]   vehicle oscillation in position
 
    % camera
    gcam.f = 1000; % 310;    % 310 [pixel]
    gcam.fov = 40 * pi / 180.0;  % FOV:40deg
    gcam.cos_halffov = cos(gcam.fov*0.5);

end

%% Calculate pose and twist of vehicle at specific time
%  input: time
%  output: pose, twist
function [pose, twist] = ppr_gen_path(t)
    global gpath;
    L_mod = mod(gpath.v*t, gpath.L_total_path);
    if (L_mod < (gpath.x_line_max-gpath.x_line_min))   %% first straight line
        x = L_mod + gpath.x_line_min;
        y = gpath.y_line_min;
        th = 0;
        omega = 0;
    elseif (L_mod < gpath.x_line_max-gpath.x_line_min + pi*gpath.R)  %% right corner
        L_rem = L_mod - (gpath.x_line_max-gpath.x_line_min);
        phi = -pi*0.5 + L_rem/gpath.R;
        x = gpath.x_line_max + gpath.R * cos(phi);
        y = gpath.y_mid + gpath.R * sin(phi);
        th = pi*0.5+phi;
        omega = gpath.v/gpath.R;
    elseif (L_mod < 2*(gpath.x_line_max-gpath.x_line_min) + pi*gpath.R)  %% upper straight line
        L_rem = L_mod - (gpath.x_line_max-gpath.x_line_min + pi*gpath.R);
        x = -L_rem + gpath.x_line_max;
        y = gpath.y_line_max;
        th = pi;
        omega = 0;
    else %% left corner
        L_rem = L_mod - (2*(gpath.x_line_max-gpath.x_line_min) + pi*gpath.R);
        phi = pi*0.5 + L_rem/gpath.R;
        x = gpath.x_line_min + gpath.R * cos(phi);
        y = gpath.y_mid + gpath.R * sin(phi);
        th = pi*0.5+phi;
        omega = gpath.v/gpath.R;
    end
    
    cos_th = cos(th);
    sin_th = sin(th);
    pose = [
        cos_th   -sin_th    0    x     ;
        sin_th    cos_th    0    y     ;
        0         0         1    0     ;
        0         0         0    1     ];
    v = gpath.v;
    twist = [
        0        -omega     0    v     ;
        omega     0         0    0     ;
        0         0         0    0     ;
        0         0         0    0    ];
end        

%% Generate state of vehicle at specific time
% input: t : time
% output: vclstat_real : actual state of vehicle
function vclstat_real = ppr_gen_vclstat_real(t, varargin)
    global gvcl;
    flg_camfixed = false;
    for i=1:(nargin-1)
        if strcmp(varargin(1), 'camfixed')
            flg_camfixed = true;
        end
    end
    [pose, twist] = ppr_gen_path(t);
    osc_yz = gvcl.sigma_osc_rot*ppr_randu_var1();
    osc_zx = gvcl.sigma_osc_rot*ppr_randu_var1();
    osc_xy = gvcl.sigma_osc_rot*ppr_randu_var1();
    osc_x  = gvcl.sigma_osc_trs*ppr_randu_var1();
    osc_y  = gvcl.sigma_osc_trs*ppr_randu_var1();
    osc_z  = gvcl.sigma_osc_trs*ppr_randu_var1();
    omega = -twist(1,2);
    v = twist(1,4);
    steer = atan2(gvcl.L * omega, v);
    %disp('steer in vclstat_real:'); disp(steer);
    %disp('osc max:'); disp(max([osc_xy, osc_yz, osc_zx]));
    mtx_osc = expm([
        0,        -osc_xy,    osc_zx,    osc_x    ;
        osc_xy,    0,        -osc_yz,    osc_y    ;
       -osc_zx,    osc_yz,    0,         osc_z    ;
        0,         0,         0,         0        ]);
    %disp('mtx_osc'); disp(mtx_osc);
        
    steer_wn = steer + gvcl.sigma_steer * ppr_randu_var1();
    cos_steer_wn = cos(steer_wn);
    sin_steer_wn = sin(steer_wn);
    if flg_camfixed
        mtx_b2c_steer = [
            1, 0, 0, gvcl.L          ;
            0, 1, 0, 0               ;
            0, 0, 1, gvcl.camera_z   ;
            0, 0, 0, 1               ];
    else
        mtx_b2c_steer = [
              cos_steer_wn, -sin_steer_wn,     0,    gvcl.L          ;
              sin_steer_wn,  cos_steer_wn,     0,    0               ;
              0,             0,                1,    gvcl.camera_z   ;
              0,             0,                0,    1               ];
    end
    mtx_b2c = mtx_osc * mtx_b2c_steer;
    % mtx_b2c = mtx_b2c_steer;
    vclstat_real.pose = pose;
    vclstat_real.twist = twist;
    vclstat_real.mtx_b2c = mtx_b2c;
    vclstat_real.mtx_osc = mtx_osc;
end        

%% Retrieve state of vehicle at specific time
% input: k : index
% output: vclstat_real : actual state of vehicle
function vclstat_real = ppr_retrieve_vclstat_real(recdt, k, varargin)
    global gvcl;
    flg_camfixed = false;
    for i=1:(nargin-1)
        if strcmp(varargin(1), 'camfixed')
            flg_camfixed = true;
        end
    end
    vclstat_real = recdt{k}.vclstat_real;
    if flg_camfixed
        mtx_b2c_steer = [
            1, 0, 0, gvcl.L          ;
            0, 1, 0, 0               ;
            0, 0, 1, gvcl.camera_z   ;
            0, 0, 0, 1               ];
        vclstat_real.mtx_b2c = vclstat_real.mtx_osc * mtx_b2c_steer;
    end
end

%% Measure odometry (=twist + noise)
% input: vclstat_real
% output: twist_msr, cov_twist
function twist_msr = ppr_measure_odom(vclstat_real)
    global gvcl;
    global gpath;
    % add noise 
    v = vclstat_real.twist(1,4);
    omega = -vclstat_real.twist(1,2);
    v_msr = v + gvcl.sigma_vel * ppr_randu_var1();
    omega_msr = omega + gvcl.sigma_steer * v / gvcl.L * ppr_randu_var1();
    twist_msr = [
        0            -omega_msr     0    v_msr     ;
        omega_msr     0             0    0         ;
        0             0             0    0         ;
        0             0             0    0         ];
    % cov_twist = diag([
    %     (gpath.slope*gpath.v/gvcl.W)^2 , ...   %% slope can be non-zero
    %     (gpath.slope*gpath.v/gvcl.L)^2 , ...   %% slope can be non-zero
    %     gvcl.sigma_steer^2             , ...   %% error in steering encoder
    %     gvcl.sigma_vel^2               , ...   %% error in wheel encoder
    %     0                              , ...   %% skid
    %     (gpath.slope*gpath.v)^2]);
end

%% Detect landmark by camera attached to vehicle
% input:  vclstat_real (actual state of vehicle)
% output: cell_c2l_info : a cell array of camera-to-landmark information
function [cell_c2l_info] = ppr_detect_landmark(vclstat_real)
    global gldmk;
    global gcam;
    cell_c2l_info = cell(0);

    % calculate pose of camera with regard to world coordinate
    mtx_m2c = vclstat_real.pose * vclstat_real.mtx_b2c ;

    % search for landmarks visible from camera
    i = 1;
    for id = 1:length(gldmk.pos)
        %
        mtx_m2l = [
            eye(3,3)      gldmk.pos{id}.' ;
            zeros(1,3)    1               ];
        
        mtx_c2l = mtx_m2c \ mtx_m2l ;
        %disp('id:'); disp(id);
        %disp('pose:'); disp(vclstat_real.pose);
        %disp('mtx_m2c:'); disp(mtx_m2c);
        %disp('mtx_c2l:'); disp(mtx_c2l);
        d = mtx_c2l(1,4);
        noise_rotvec = [ ppr_randu_var1()*d/gcam.f/gldmk.size,    ppr_randu_var1()*d/gcam.f/gldmk.size, ppr_randu_var1()*d/gcam.f/gldmk.size ];
        noise_trsvec = [ ppr_randu_var1()*d/gcam.f*d/gldmk.size,  ppr_randu_var1()*d/gcam.f,            ppr_randu_var1()*d/gcam.f            ];
        mtx_c2l_noise = expm([
            0                -noise_rotvec(3)  noise_rotvec(2)  noise_trsvec(1) ;
            noise_rotvec(3)   0               -noise_rotvec(1)  noise_trsvec(2) ;
           -noise_rotvec(2)   noise_rotvec(1)  0                noise_trsvec(3) ;
            0                 0                0                0               ]);
        mtx_c2l_wn = mtx_c2l * mtx_c2l_noise;
        % check AOV (angle of view)
        if (norm(mtx_c2l_wn(1:3, 4))*gcam.cos_halffov <= [1 0 0] * mtx_c2l_wn(1:3, 4))
            if (abs(mtx_c2l_wn(1, 4) <= 5))
                % store landmark
                %disp('!MATCHED! id:'); disp(id);
                cell_c2l_info{i}.mtx = mtx_c2l_wn;
                cell_c2l_info{i}.id = id;
                %disp('mtx_c2l:'); disp(mtx_c2l);
                %disp('mtx_c2l_noise:'); disp(mtx_c2l_noise);
                %disp('d:'); disp(d);
                %disp('noise_trsvec:'); disp(noise_trsvec);
                %disp('noise_rotvec:'); disp(noise_rotvec);
                %disp('mtx_c2l_wn:'); disp(mtx_c2l_wn);
                %input('Press ENTER key..');
                %disp('pose_real_reverse_back:');  disp(mtx_m2l/mtx_c2l/vclstat_real.mtx_b2c);
                %disp('pose_real:'); disp(vclstat_real.pose);
                %input('Enter key:');
                i = i + 1;
            end
        end
    end
end


%% Estimate error covariance of twist
% input: vx: forward velocity of vehicle
% output: covariance of twist (linear velocity and angular velocity)
function [cov_twist] = ppr_estimate_cov_twist(vx)
    global gvcl;
    cov_twist = diag([
        0,   0,   (vx/gvcl.L*gvcl.sigma_steer)^2,   gvcl.sigma_vel^2,   0,   0]);
end

%% Estimate error covariance of camera-to-landmark transformation
% input: mtx_c2l : camera-to-landmark transformation
% output: estimated covariance matrix of mtx_c2l
function [cov_c2l] = ppr_estimate_cov_c2l(mtx_c2l)
    global gcam;
    global gldmk;
    d = mtx_c2l(1, 4);
    cov_c2l = diag([
            (d/gcam.f/gldmk.size)^2,  ...
            (d/gcam.f/gldmk.size)^2,  ...
            (d/gcam.f/gldmk.size)^2,  ...
            (d*d/gcam.f/gldmk.size)^2, ...
            (d/gcam.f)^2,  ...
            (d/gcam.f)^2,  ...
            ]);
        %disp('cov_c2l:'); disp(cov_c2l);
end

%% Estimate pose of camera relative to base foot print
% input: steer angle
% output: mtx_b2c:coordinate transform  cov_b2c: error covariance
function [mtx_b2c, cov_b2c] = ppr_estimate_b2c(steer)
    global gvcl;
    %disp('steer in estim:'); disp(steer);
    cos_steer = cos(steer);
    sin_steer = sin(steer);
    mtx_b2c = [
              cos_steer, -sin_steer,     0,    gvcl.L           ;
              sin_steer,  cos_steer,     0,    0                ;
              0,             0,          1,    gvcl.camera_z    ;
              0,             0,          0,    1                ];
    sigma_1rr = diag([gvcl.sigma_osc_rot^2, gvcl.sigma_osc_rot^2, gvcl.sigma_osc_rot^2]);
    sigma_2rr = diag([                   0,                    0, gvcl.sigma_steer^2]);
    sigma_2tt = diag([gvcl.sigma_osc_trs^2, gvcl.sigma_osc_trs^2, gvcl.sigma_osc_trs^2]);
    A = ppr_antisym_v2m([gvcl.L ; 0 ; gvcl.camera_z]);
    cov_b2c = [ sigma_1rr + sigma_2rr      ,   -sigma_1rr*A.'                   ;
                -A*sigma_1rr               ,    sigma_2tt + A * sigma_1rr * A.' ];
end

%% Estimate pose of camera relative to base foot print
% input: steer angle
% output: mtx_b2c:coordinate transform  cov_b2c: error covariance
function [mtx_b2c, cov_b2c] = ppr_estimate_b2c_camfixed()
    global gvcl;
    mtx_b2c = [
              1,  0,     0,    gvcl.L           ;
              0,  1,     0,    0                ;
              0,  0,     1,    gvcl.camera_z    ;
              0,  0,     0,    1                ];
    cov_b2c = diag([gvcl.sigma_osc_rot^2, gvcl.sigma_osc_rot^2, gvcl.sigma_osc_rot^2, ...
        gvcl.sigma_osc_trs^2, gvcl.sigma_osc_trs^2, gvcl.sigma_osc_trs^2]);
end

%% Unscented Kalman Filter
% input : pose_est, cov_pose_est, twist_est, cov_twist_est, pose_msr, cov_pose_msr, kappa
% output : pose_upd, cov_pose_upd
function [pose_upd, cov_pose_upd, pose_prd, P_prd, G] = ppr_UKF(pose_est, cov_pose_est, twist_est, cov_twist_est, pose_msr, cov_pose_msr, kappa)
    global gsim;

    % prepare memory
    n = 6;
    chi_org = cell(2*n+1,1);
    chi_prd = cell(2*n+1,1);
    chi_upd = cell(2*n+1,1);
    w = cat(2, kappa, ones(1,2*n)*0.5) / (n+kappa);
    % disp('w:'); disp(w);
    sqrt_n_p_kappa = (n+kappa)^0.5;
    % calc square root of P
    S = ppr_sqrt_cov(cov_pose_est);
    % disp('S:'); disp(S);
    % calculate sigma points
    chi_org{1} = zeros(n, 1);
    for i = 1:n
        chi_org{1+i} = sqrt_n_p_kappa*S(:,i);
        chi_org{1+n+i} = -chi_org{1+i};
    end
    for i = 1:(2*n+1)
        %fprintf('chi_org{%d}:', i);
        %disp(chi_org{i});
    end
    
    %% prediction step
    % predict chi
    %disp('twist_est'); disp(twist_est);
    twist_est_vec = ppr_twist_m2v(twist_est);
    for i = 1:(2*n+1)
        chi_prd{i} = chi_org{i} + twist_est_vec * gsim.dt;
    end
    % chi -> x -> pose_prd
    x_prd = zeros(6,1);
    for i = 1:(2*n+1)
        x_prd = x_prd + w(i) * chi_prd{i};
    end
    % disp('x_prd:'); disp(x_prd);
    for i = 1:(2*n+1)
        %fprintf('chi_prd{%d}:', i);
        %disp(chi_prd{i});
    end
    pose_prd = pose_est * expm(ppr_twist_v2m(x_prd));
    %disp('pose_prd:'); disp(pose_prd);
    % predict P
    P_prd = cov_twist_est * gsim.dt^2;
    for i = 1:(2*n+1)
        P_prd = P_prd + w(i) * (chi_prd{i} - x_prd) * (chi_prd{i} - x_prd).';
    end
        
    %disp('P_prd:'); disp(P_prd);
    
    %% check if new measurement is obtained
    if pose_msr == false
        pose_upd = pose_prd;
        cov_pose_upd = P_prd;
        return;
    end
        
   %% preparation for innovation step
    S = ppr_sqrt_cov(P_prd);
    chi_prd{1} = zeros(6,1);
    for i = 1:n
        chi_prd{1+i} = sqrt_n_p_kappa*S(:,i);
        chi_prd{1+n+i} = - chi_prd{1+i};
    end
    
    %% innovation step
    % x based on measurement
    %disp('pose_msr:'); disp(pose_msr);
    %disp('pose_prd:'); disp(pose_prd);
    
    x_msr_mtx = logm(pose_prd \ pose_msr);
    x_msr = ppr_twist_m2v(x_msr_mtx);
    %disp('x_msr:'); disp(x_msr);
    %disp('chi_prd{1}:'); disp(chi_prd{1});
    % Kalman gain
    %disp('cov_pose_msr:'); disp(cov_pose_msr);
    cov_pose_msr_relto_pose_prd = ppr_calc_cov_of_mul(inv(pose_prd), zeros(6,6), pose_msr, cov_pose_msr);
    %disp('cov_pose_msr_relto_pose_prd:'); disp(cov_pose_msr_relto_pose_prd);
    %G = P_prd / (P_prd + cov_pose_msr);
    G = P_prd / (P_prd + cov_pose_msr_relto_pose_prd);
    %disp('G:'); disp(G);
    %disp('det(G)^(1/6):'); disp(det(G)^(1/6.0));
    % chi
    for i = 1:(2*n+1)
        chi_upd{i} = chi_prd{i} + G * (x_msr - chi_prd{i});
    end
    %disp('chi_upd{1}:'); disp(chi_upd{1});
    % chi -> x -> pose_upd
    x_upd = w(1) * chi_upd{1};
    for i = 2:(2*n+1)
        x_upd = x_upd + w(i) * chi_upd{i};
    end
    %disp('x_upd:'); disp(x_upd);
    pose_upd = pose_prd * expm(ppr_twist_v2m(x_upd));
    %pose_upd = pose_msr;
    %disp('pose_upd:'); disp(pose_upd);
    P_upd = P_prd - G * P_prd;
    cov_pose_upd = P_upd;
    % disp('P_upd:'); disp(P_upd);
    %input('End of UKF. Enter any key:');
end

%% Unscented Kalman Filter omitting update step
% input : pose_est, cov_pose_est, twist_est, cov_twist_est, kappa
% output : pose_prd, cov_pose_prd
function [pose_prd, P_prd] = ppr_UKF_only_predict(pose_est, cov_pose_est, twist_est, cov_twist_est, kappa)
    [pose_prd, P_prd] = ppr_UKF(pose_est, cov_pose_est, twist_est, cov_twist_est, false, false, kappa);
end


%% Calculate covariance matrix of inverse transformation
% input : mtx: transformation matrix, cov:covariance matrix of mtx
% output: cov_i:convariance matrix of inv(mtx)
function [cov_i] = ppr_calc_cov_of_inv(mtx, cov)
    cov_i = zeros(6,6);
    R = mtx(1:3, 1:3);
    t = mtx(1:3, 4);
    A = ppr_antisym_v2m(t);
    sigma_rr = cov(1:3, 1:3);
    sigma_rt = cov(1:3, 4:6);
    sigma_tt = cov(4:6, 4:6);
    cov_i(1:3, 1:3) = R * sigma_rr * R.';
    cov_i(4:6, 4:6) = R.' * ( ...
        A * R * sigma_rr * R.' * A.' + sigma_tt + A * R * sigma_rt + ...
            sigma_rt.' * R.' * A.' ) * R;
    cov_i(1:3, 4:6) = R * ( sigma_rr * R.' * A.' + sigma_rt ) * R;
    cov_i(4:6, 1:3) = cov_i(1:3, 4:6).';
end

%% Calculate covariance matrix of multiplication of two transformations
% input : mtx1, mtx2:transformation matrices,  cov1, cov2:covariance
% matrices of mtx1 and mtx2 respectively
% output: cov3: covariance matrix of mtx1*mtx2
function [cov3] = ppr_calc_cov_of_mul(mtx1, cov1, mtx2, cov2)
    cov3 = zeros(6,6);
    R1 = mtx1(1:3, 1:3);
    R2 = mtx2(1:3, 1:3);
    t2 = mtx2(1:3, 4);
    A2 = ppr_antisym_v2m(t2);
    sigma_1rr = cov1(1:3, 1:3);
    sigma_1tt = cov1(4:6, 4:6);
    sigma_1rt = cov1(1:3, 4:6);
    sigma_2rr = cov2(1:3, 1:3);
    sigma_2tt = cov2(4:6, 4:6);
    sigma_2rt = cov2(1:3, 4:6);
    sigma_3rr = R2.' * sigma_1rr * R2 + sigma_2rr;
    sigma_3tt = R1 * sigma_2tt * R1.' + sigma_1tt + R1 * A2 * sigma_1rr * A2.' * R1.' ...
              - R1 * A2 * sigma_1rt - sigma_1rt.' * A2.' * R1.';
    sigma_3rt = sigma_2rt * R1.' - R2.' * sigma_1rr * A2.' * R1.' + R2.' * sigma_1rt;
    cov3(1:3, 1:3) = sigma_3rr;
    cov3(1:3, 4:6) = sigma_3rt;
    cov3(4:6, 4:6) = sigma_3tt;
    cov3(4:6, 1:3) = sigma_3rt.';
end

%% Convert 3x3 antisymmetric matrix to 3x1 vector
%
%
function v = ppr_antisym_m2v(M)
    v = [ M(3,2) ; M(1,3) ; M(2,1) ];
end

%% Convert 3x1 vector to 3x3 antisymmetric matrix
%
%
function M = ppr_antisym_v2m(v)
    M = [ 
           0,  -v(3),   v(2)  ;
        v(3),      0,  -v(1)  ;
       -v(2),   v(1),      0  ];
end

%% Convert twist data; 4x4 matrix to 6x1 vector.
%
function v = ppr_twist_m2v(M)
    v = [ M(3,2) ; M(1,3) ; M(2,1) ; M(1,4) ; M(2,4) ; M(3,4) ];
end

%% Convert twist data; 1x6 vector to 4x4 matrix.
%
function M = ppr_twist_v2m(v)
    M = [ 
           0,  -v(3),   v(2),   v(4);
        v(3),      0,  -v(1),   v(5);
       -v(2),   v(1),      0,   v(6);
           0,      0,      0,      0];
end

%% Normalized uniform distribution
%
%
function r = ppr_randu_var1()
    r = (rand()-0.5)*sqrt(12);
end

%% 
%
%
function S = ppr_sqrt_cov(cov)
    S = ppr_sqrt_cov_UDFactor(cov);
end

%%
%
%
function S = ppr_sqrt_cov_chol(cov)
    % cholesky decomposition of P
    S = chol(cov, 'lower');
end

%%
%
%
function S = ppr_sqrt_cov_UDFactor(cov)
    [U, D] = UDFactor(cov);
    % disp('cov:'); disp(cov); disp('U:'); disp(U); disp('D:'); disp(D);
    S = U.' * D.^0.5;
end   


%% Below is copied from https://jp.mathworks.com/matlabcentral/fileexchange/20600-udfactorhttps://jp.mathworks.com/matlabcentral/fileexchange/20600-udfactor
% UDFactor by Dmitry Savransky
% Performs the U-D factorization of a symmetric matrix.
%
function [U D] = UDFactor(P,uflag)
% UDFactor performs the U-D factorization of a symmetric matrix.
% 
% [U D] = UDFactor(P) returns matrices U and D such that U.'*D*U = P
% [U D] = UDFactor(P,uflag) returns matrices U and D such that U*D*U.' = P
% when uflag is set to TRUE.  Setting uflag to FALSE is equivalent to
% running UDFactor with only one argument.
% 
% The alogrithm of UDFactor is similar to the Cholesky decomposition except
% that the matrix is factored into a unitary upper triangular matrix (U)
% and diagonal matrix (D) such that P = U*D*U.' (or U.'*D*U).  Note that
% while this is equivalent to P = (U*D^0.5)*(U*D^0.5).' = S*S.' where S is
% the upper triangular square root of P, no square roots are taken in the
% calculations of U and D.  This makes this factorization ideal for a
% square-root implementation of a Kalman filter (a U-D filter). For more
% details, see Bierman, G. J., Factorization methods for discrete 
% sequential estimation, 1977.
%
% Note: This factorization is only guaranteed to work for symmetric
% matrices.
% 
% Examples:  
%     %create symmetric matrix
%     P = rand(5)*10;, P = triu(P)+triu(P).';
%     %factor
%     [U1,D1] = UDFactor(P);
%     [U2,D2] = UDFactor(P,true);
%     %check factorization
%     P - U1.'*D1*U1
%     P - U2*D2*U2.'
%
% Written by Dmitry Savransky 7 July 2008
%if no flag was set, assume it to be false
if ~exist('uflag','var')
    uflag = false;
end
%size of matrix
n = length(P);
%allocate U
U = zeros(n);
%if uflag UDU' otherwise U'DU
if uflag
    D = zeros(1,n);%D is diagonal, so leave it as a vector
    D(end) = P(end);
    U(:,end) = P(:,end)./P(end);
    for j = n-1:-1:1
        D(j) = P(j,j) - sum(D(j+1:n).*U(j,j+1:n).^2);
        U(j,j) = 1;
        for i=j-1:-1:1
            U(i,j) = (P(i,j) - sum(D(j+1:n).*U(i,j+1:n).*U(j,j+1:n)))/D(j);
        end
    end
else
    D = zeros(n,1);%D is diagonal, so leave it as a vector
    D(1) = P(1);
    U(1,:) = P(:,1)./P(1);
    for j = 2:n
        D(j) = P(j,j) - sum(D(1:j-1).*U(1:j-1,j).^2);
        U(j,j) = 1;
        for i=j+1:n
            U(j,i) = (P(j,i) - sum(D(1:j-1).*U(1:j-1,i).*U(1:j-1,j)))/D(j);
        end
    end
end
D = diag(D);
end



%% Test covariance arithmetics
%
function ppr_check_cov_6x6()
    mtx_arr = cell(100);
    for i = 1:100
        mtx_rot = ppr_antisym_v2m([ppr_randu_var1()*0.1, ppr_randu_var1()*0.1, ppr_randu_var1()*0.1]);
        mtx_trs = [ppr_randu_var1()*0.2,ppr_randu_var1()*0.2,ppr_randu_var1()*0.2];
        mtx = zeros(4,4);
        mtx(1:3, 1:3) = mtx_rot;
        mtx(1:3, 4) = mtx_trs;
        mtx_arr{i} = expm(mtx);
        %disp('mtx'); disp(mtx);
        disp('mtx_arr{i}'); disp(mtx_arr{i});
    end
    cov = zeros(6,6);
    for i = 1:100
        twist = logm(mtx_arr{i});
        vec = ppr_twist_m2v(twist); 
        cov = cov + (vec * vec.')/100.0;
    end
    disp('cov'); disp(cov);
end


%% Test covariance 
%