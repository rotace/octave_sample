% シムシティ生産計画ソルバー
% https://www.msi.co.jp/nuopt/docs/v23/examples/html/02-25-02.html
% https://www.jstage.jst.go.jp/article/isciesci/61/1/61_14/_pdf/-char/ja
% https://www.jstage.jst.go.jp/article/kikaic1979/71/702/71_702_685/_pdf
clear all
common

function dispResult(xmin)
    [work_max, shop_max, time_max] = size(xmin);
    for w=1:work_max
        for s=1:shop_max
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


work_max = 3;
shop_max = 3;
time_max = 30;

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

% 制約
A = [];
b = [];
ctype = "";

% (36) コストの定義
LAST_PROCESS=1
for w=1:work_max
    s = a_shops(w,LAST_PROCESS);
    x = zeros(work_max, shop_max, time_max);
    for t=1:time_max
        x(w,s,t) = -t; % t x delta
    end
    x = [x(:); 1]; % Cmax
    A = cat(2,A,x);
    b = cat(2,b,a_times(w,LAST_PROCESS));
    ctype = cat(2,ctype,"L");
end

% (37) 工程処理順序の制約
% TODO: 本制約は仕事wにおける機械sの使用回数が必ず１回であることを前提にしているため、
% 機械を複数回使用する場合、工程処理順序の制約に破綻が生じる
for w=1:work_max
    [pst_process,pre_process]=find(squeeze(a_orders(w,:,:))==IS_NEED);
for i=1:length(pst_process)
        pre_shop = a_shops(w,pre_process(i)); % 前に使用する機械
        pst_shop = a_shops(w,pst_process(i)); % 後に使用する機械
        x = zeros(work_max, shop_max, time_max);
        for t=1:time_max
            x(w,pre_shop,t) = -t; % t x delta
        end
        for t=1:time_max
            x(w,pst_shop,t) = +t; % t x delta
        end
        x = [x(:); 0]; % Cmax
        A = cat(2,A,x);
        b = cat(2,b,a_times(w,pre_process(i)));
        ctype = cat(2,ctype,"L");
    end
end

% (38) 同時処理禁止の制約
for w=1:work_max
    for i=1:length(a_shops(w,:))
        s = a_shops(w,i);
        p = a_times(w,i);
        for t=1:(time_max-p-1)
            x = zeros(work_max, shop_max, time_max);
            for k=1:work_max
                if k==w
                    % this work
                    x(k,s,t) = 100;
                else
                    % other work
                    for j=t:(t+p-1)
                        x(k,s,j) = 1;
                    end
                end
            end
            x = [x(:); 0]; % Cmax
            A = cat(2,A,x);
            b = cat(2,b,100);
            ctype = cat(2,ctype,"U");
        end
    end
end

% (39) ある機械におけるある仕事の実行回数
count=zeros(work_max, shop_max);
for w=1:work_max
    for i=1:length(a_shops(w,:))
        count(w,a_shops(w,i)) += 1;
    end
end
for w=1:work_max
    for s=1:shop_max
        x = zeros(work_max, shop_max, time_max);
        for t=1:time_max
            x(w,s,t) = 1;
        end
        x = [x(:); 0]; % Cmax
        A = cat(2,A,x);
        b = cat(2,b,count(w,s));
        ctype = cat(2,ctype,"S");
    end
end

A = A';
b = b';
vartype = repmat("I", 1, size(A)(2));
vartype(end) = "C"; % Cmax

% 目的関数
c = zeros(1, size(A)(2))';
c(end) = 1; % Cmax

% 非負制約
lb = zeros(1, size(A)(2));
ub = ones (1, size(A)(2));
ub(end) = time_max; % Cmax

sense = MINIMIZE;
% param.msglev = GLP_MSG_ALL;
param.itlim = 100;

[xmin, fmin, status, extra] = glpk (c, A, b, lb, ub, ctype, vartype, sense, param);

disp("==== RESULT ====")
disp(["Costs:", num2str(fmin)])
disp("Variables:")
Cmax = xmin(end)
xmin = reshape(xmin(1:end-1), work_max, shop_max, time_max);

dispResult(xmin);
dispJobSch(xmin);
dispShopSch(xmin);