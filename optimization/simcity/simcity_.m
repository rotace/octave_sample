% シムシティ生産計画ソルバー
% https://www.msi.co.jp/nuopt/docs/v23/examples/html/02-25-02.html
% https://www.jstage.jst.go.jp/article/isciesci/61/1/61_14/_pdf/-char/ja
% https://www.jstage.jst.go.jp/article/kikaic1979/71/702/71_702_685/_pdf
clear all
common

function dispResult(xmin)
    [work_max, proc_max, time_max] = size(xmin);
    for w=1:work_max
        for s=1:proc_max
            [~,idx] = max(xmin(w,s,:));
            disp(["[", num2str(w), ",", num2str(s), "]= ", num2str(idx-1)])
        end
    end
end

function dispJobSch(xmin)
    disp("===== Job Schedule =====")
    permute(xmin, [2,3,1])
end

function dispShopSch(xmin)
    disp("===== Shop Schedule =====")
    permute(xmin, [1,3,2])
end


work_max = 3; % 仕事の数
proc_max = 3; % 工程の数
time_max = 30;% 時間の数

% 仕事iのj番目の工程の処理時間
a_times = [
    4 8 5;
    6 4 5;
    5 5 6
];
% 仕事iのj番目の工程の使用機械
a_shops = [
    3 2 1;
    3 2 1;
    3 2 1
];
% 仕事iのj番目の工程の前にk番目の工程が必要かどうか
% （工程順序を表すグラフ／隣接行列）
IS_NEED=1;
a_orders_i = [
    0 IS_NEED 0;
    0 0 IS_NEED;
    0 0 0
];
a_orders = repmat(a_orders_i,1,1,3);
a_orders = permute(a_orders,[3,1,2]);

% 決定変数
A.c = []; % Costs(コスト)
A.d = []; % Delta (仕事 w の工程 p を第 t 期に処理するなら１、そうでなければ0)
A.s = []; % Start (仕事 w の工程 p を第 t 期に開始するなら１、そうでなければ0)
A.f = []; % Finish(仕事 w の工程 p を第 t 期に終了するなら１、そうでなければ0)
b = [];
ctype = "";

% (36) コストの定義
LAST_PROCESS=1
for w=1:work_max
    s = a_shops(w,LAST_PROCESS);
    x.c = 1;
    x.d = zeros(work_max, proc_max, time_max);
    x.s = zeros(work_max, proc_max, time_max);
    x.f = zeros(work_max, proc_max, time_max);

    for t=1:time_max
        x.d(w,s,t) = -t;
    end
    A.c = cat(2,A.c, x.c(:));
    A.d = cat(2,A.d, x.d(:));
    A.s = cat(2,A.s, x.s(:));
    A.f = cat(2,A.f, x.f(:));
    b = cat(2,b,a_times(w,LAST_PROCESS));
    ctype = cat(2,ctype,LOWER_CONST);
end

% (37) 工程処理順序の制約
% TODO: 本制約は仕事wにおける機械sの使用回数が必ず１回であることを前提にしているため、
% 機械を複数回使用する場合、工程処理順序の制約に破綻が生じる
for w=1:work_max
    [pst_process,pre_process]=find(squeeze(a_orders(w,:,:))==IS_NEED);
for i=1:length(pst_process)
        pre_shop = a_shops(w,pre_process(i)); % 前に使用する機械
        pst_shop = a_shops(w,pst_process(i)); % 後に使用する機械
        x.c = 0;
        x.d = zeros(work_max, proc_max, time_max);
        x.s = zeros(work_max, proc_max, time_max);
        x.f = zeros(work_max, proc_max, time_max);

        for t=1:time_max
            x.d(w,pre_shop,t) = -t;
        end
        for t=1:time_max
            x.d(w,pst_shop,t) = +t;
        end
        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,a_times(w,pre_process(i)));
        ctype = cat(2,ctype,LOWER_CONST);
    end
end

% (38) 同時処理禁止の制約
for w=1:work_max
    for i=1:length(a_shops(w,:))
        s = a_shops(w,i);
        p = a_times(w,i);
        for t=1:(time_max-p-1)
            x.c = 0;
            x.d = zeros(work_max, proc_max, time_max);
            x.s = zeros(work_max, proc_max, time_max);
            x.f = zeros(work_max, proc_max, time_max);

            for k=1:work_max
                if k==w
                    % this work
                    x.d(k,s,t) = 100;
                else
                    % other work
                    for j=t:(t+p-1)
                        x.d(k,s,j) = 1;
                    end
                end
            end
            A.c = cat(2,A.c, x.c(:));
            A.d = cat(2,A.d, x.d(:));
            A.s = cat(2,A.s, x.s(:));
            A.f = cat(2,A.f, x.f(:));
            b = cat(2,b,100);
            ctype = cat(2,ctype,UPPER_CONST);
        end
    end
end

% (39) ある機械におけるある仕事の実行回数
count=zeros(work_max, proc_max);
for w=1:work_max
    for i=1:length(a_shops(w,:))
        count(w,a_shops(w,i)) += 1;
    end
end
for w=1:work_max
    for s=1:proc_max
        x.c = 0;
        x.d = zeros(work_max, proc_max, time_max);
        x.s = zeros(work_max, proc_max, time_max);
        x.f = zeros(work_max, proc_max, time_max);

        for t=1:time_max
            x.d(w,s,t) = 1;
        end
        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,count(w,s));
        ctype = cat(2,ctype,EQUAL_CONST);
    end
end

% 実数／整数
vtype.c = repmat("C", 1, 1);
vtype.d = repmat("I", 1, prod([work_max, proc_max, time_max]));
vtype.s = repmat("I", 1, prod([work_max, proc_max, time_max]));
vtype.f = repmat("I", 1, prod([work_max, proc_max, time_max]));

% 目的関数
c.c = 1;
c.d = zeros(work_max, proc_max, time_max)(:);
c.s = zeros(work_max, proc_max, time_max)(:);
c.f = zeros(work_max, proc_max, time_max)(:);

% 非負制約
lb.c = 0;
lb.d = zeros(work_max, proc_max, time_max)(:)';
lb.s = zeros(work_max, proc_max, time_max)(:)';
lb.f = zeros(work_max, proc_max, time_max)(:)';
ub.c = time_max;
ub.d = ones (work_max, proc_max, time_max)(:)';
ub.s = ones (work_max, proc_max, time_max)(:)';
ub.f = ones (work_max, proc_max, time_max)(:)';

% 結合
c = cat(1, c.d, c.c);
A = cat(1, A.d, A.c)';
b = b';
lb = cat(2, lb.d, lb.c);
ub = cat(2, ub.d, ub.c);
ctype = ctype;
vtype = cat(2, vtype.d, vtype.c);

sense = MINIMIZE;
% param.msglev = GLP_MSG_ALL;
param.itlim = 100;

[xmin, fmin, status, extra] = glpk (c, A, b, lb, ub, ctype, vtype, sense, param);

disp("==== RESULT ====")
disp(["Costs:", num2str(fmin)])
disp("Variables:")
Cmax = xmin(end)
xmin = reshape(xmin(1:end-1), work_max, proc_max, time_max);

dispResult(xmin);
dispJobSch(xmin);
dispShopSch(xmin);