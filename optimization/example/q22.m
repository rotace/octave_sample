% 2.2 輸送問題
% https://www.msi.co.jp/nuopt/docs/v23/examples/html/02-02-00.html
clear all
common

% 目的関数
c = [3.4 2.2 2.9 3.4 2.4 2.5]';

% 工場の生産量／店舗の需要量の制約
A = [ 1.0 1.0 1.0 0.0 0.0 0.0;
      0.0 0.0 0.0 1.0 1.0 1.0;
      1.0 0.0 0.0 1.0 0.0 0.0;
      0.0 1.0 0.0 0.0 1.0 0.0;
      0.0 0.0 1.0 0.0 0.0 1.0];
b = [250 450 200 200 200]';
vartype = "CCCCCC";
ctype = "UUSSS";

% 非負制約
lb = [0 0 0 0 0 0]';
ub = [];
sense = MINIMIZE;
param.msglev = GLP_MSG_ALL;
param.itlim = 100;

[xmin, fmin, status, extra] = glpk (c, A, b, lb, ub, ctype, vartype, sense, param);

disp("==== RESULT ====")
disp(["Costs:", num2str(fmin)])
disp("Variables:")
disp(xmin)
