% 2.5 ナップサック問題
% https://www.msi.co.jp/nuopt/docs/v23/examples/html/02-05-00.html
clear all
common

% 目的関数
c = [120 130 80 100 250 185]';

% ナップサックの容量の制約
A = [ 10 12  7  9 21 16];
b = [65]';
vartype = "IIIIII";
ctype = "U";

% 非負制約
lb = [0 0 0 0 0 0]';
ub = [];
sense = MAXIMIZE;
param.msglev = GLP_MSG_ALL;
param.itlim = 100;

[xmin, fmin, status, extra] = glpk (c, A, b, lb, ub, ctype, vartype, sense, param);

disp("==== RESULT ====")
disp(["Costs:", num2str(fmin)])
disp("Variables:")
disp(xmin)
