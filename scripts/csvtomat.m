clear;tic;

matrix_0 = get_mat('output_0.csv');
matrix_1 = get_mat('output_1.csv');

save('matrix.mat','matrix_0','matrix_1')

toc;