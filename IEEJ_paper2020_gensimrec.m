%%
% Do IEEJ 2020 simulation
%
% 2020-06-09 H.M

% initialize random number generator
rng('default');
rng(1);
close('all');
n = 1;
simrecdt_arr_A = cell(n);
simrecdt_arr_B = cell(n);
simrecdt_arr_C = cell(n);
fh_arr_A = cell(n);
fh_arr_B = cell(n);
fh_arr_C = cell(n);
for i=1:n
    [simrecdt_arr_A{i}, fh_arr_A{i}] = IEEJ_paper2020_main(false, 'savefig');
    for j=1:length(fh_arr_A{i})
        saveas(fh_arr_A{i}{j}, sprintf('IEEJ_paper2000_A_%d_%d.fig', i, j));
        close(fh_arr_A{i}{j});
    end
end
for i=1:n
    [simrecdt_arr_B{i}, fh_arr_B{i}] = IEEJ_paper2020_main(simrecdt_arr_A{i}, 'noldmk_reuse', 'savefig', 'nocovmul');
    for j=1:length(fh_arr_B{i})
        saveas(fh_arr_B{i}{j}, sprintf('IEEJ_paper2000_B_%d_%d.fig', i, j));
        close(fh_arr_B{i}{j});
    end
end
for i=1:n
    [simrecdt_arr_C{i}, fh_arr_C{i}] = IEEJ_paper2020_main(simrecdt_arr_A{i}, 'noldmk_reuse', 'savefig', 'camfixed');
    for j=1:length(fh_arr_C{i})
        saveas(fh_arr_C{i}{j}, sprintf('IEEJ_paper2000_C_%d_%d.fig', i, j));
        close(fh_arr_C{i}{j});
    end
end
