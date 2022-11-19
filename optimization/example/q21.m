% 2.1 配合問題
% https://www.msi.co.jp/nuopt/docs/v23/examples/html/02-01-00.html
clear all
common

% 目的関数
c = [7.3 6.9 7.3 7.5 7.6 6.0 5.8 4.3 4.1]';

% 混合比率の制約
A = [ 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0;
      0.2 0.5 0.3 0.3 0.3 0.6 0.4 0.1 0.1;
      0.3 0.4 0.2 0.4 0.3 0.3 0.5 0.3 0.1;
      0.5 0.1 0.5 0.3 0.4 0.1 0.1 0.6 0.8];
b = [1.0 0.3 0.3 0.4]';
vartype = "CCCCCCCCC";
ctype = "SSSS";

% 非負制約
lb = [0 0 0 0 0 0 0 0 0]';
ub = [];
sense = MINIMIZE;
param.msglev = GLP_MSG_ALL;
param.itlim = 100;

[xmin, fmin, status, extra] = glpk (c, A, b, lb, ub, ctype, vartype, sense, param);

disp("==== RESULT ====")
disp(["Costs:", num2str(fmin)])
disp("Variables:")
disp(xmin)
