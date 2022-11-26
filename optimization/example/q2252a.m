% 2.25.2 フローショップ問題
% https://www.msi.co.jp/nuopt/docs/v23/examples/html/02-25-02.html
% https://www.jstage.jst.go.jp/article/isciesci/61/1/61_14/_pdf/-char/ja
clear all
common

ret = input("It take a lot of Time. Ok? (y/n)", "s");
if strcmp(ret, "y")
    disp("Calc Start!")
    disp("It takes a lot of time...")
else
    disp("exit.")
    return
end

function dispResult(x)
    [work_max, shop_max, time_max] = size(x);
    for w=1:work_max
        for s=1:shop_max
            [~,idx] = max(x(w,s,:));
            disp(["[", num2str(w), ",", num2str(s), "]= ", num2str(idx-1)])
        end
    end
end

function dispJobSch(x, name="x")
    disp(["===== Job Schedule (",name,") ====="])
    permute(x, [2,3,1])
end

function dispShopSch(x, name="x")
    disp(["===== Shop Schedule (",name,") ====="])
    permute(x, [1,3,2])
end


work_max = 3; % 仕事の数
shop_max = 3; % 機械の数
proc_max = 3; % 工程の数
time_max = 30;% 時間の数

% 仕事iのj番目の工程の処理時間
a_times = [
    5 8 4;
    5 4 6;
    6 5 5
];
% 仕事iのj番目の工程の使用機械
a_shops = [
    1 2 3;
    1 2 3;
    1 2 3
];

% 決定変数
A.c = []; % Costs(コスト)
A.d = []; % Delta (仕事 w の工程 p を第 t 期に処理するなら１、そうでなければ0)
A.s = []; % Start (仕事 w の工程 p を第 t 期に開始するなら１、そうでなければ0)
A.f = []; % Finish(仕事 w の工程 p を第 t 期に終了するなら１、そうでなければ0)
b = [];
ctype = "";

% (29) コストの定義
LAST_PROCESS=proc_max;
for w=1:work_max
    s = a_shops(w,LAST_PROCESS);
    x.c = -1;
    x.d = zeros(work_max, shop_max, time_max);
    x.s = zeros(work_max, shop_max, time_max);
    x.f = zeros(work_max, shop_max, time_max);
    y = 0;

    for t=1:time_max
        x.f(w,s,t) = t;
    end

    A.c = cat(2,A.c, x.c(:));
    A.d = cat(2,A.d, x.d(:));
    A.s = cat(2,A.s, x.s(:));
    A.f = cat(2,A.f, x.f(:));
    b = cat(2,b,y);
    ctype = cat(2,ctype,UPPER_CONST);

end

% (30) 工程処理順序の制約
for w=1:work_max
    for p=1:(proc_max-1)
        pre_shop = a_shops(w,p);
        pst_shop = a_shops(w,p+1);

        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = a_times(w,p);

        for t=1:time_max
            x.s(w,pre_shop,t) = -t;
        end
        for t=1:time_max
            x.s(w,pst_shop,t) = +t;
        end

        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,LOWER_CONST);

    end
end

% (31) 同時処理禁止の制約
for s=1:shop_max
    for t=1:time_max

        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = 1;

        for w=1:work_max
            x.d(w,s,t) = 1;
        end

        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,UPPER_CONST);

    end
end

for w=1:work_max
    for p=1:proc_max
        s=a_shops(w,p);

        % (32) d,s,f の関係式
        for t=1:(time_max-1)

            x.c = 0;
            x.d = zeros(work_max, shop_max, time_max);
            x.s = zeros(work_max, shop_max, time_max);
            x.f = zeros(work_max, shop_max, time_max);
            y = 0;

            x.d(w,s,t+1) = +1;
            x.d(w,s,t  ) = -1;
            x.s(w,s,t+1) = -1;
            x.f(w,s,t  ) = +1;

            A.c = cat(2,A.c, x.c(:));
            A.d = cat(2,A.d, x.d(:));
            A.s = cat(2,A.s, x.s(:));
            A.f = cat(2,A.f, x.f(:));
            b = cat(2,b,y);
            ctype = cat(2,ctype,EQUAL_CONST);
        end

        % (33) ある仕事のある工程の開始は１回以下
        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = 1;

        for t=1:time_max
            x.s(w,s,t) = 1;
        end

        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,UPPER_CONST);

        % (34) ある仕事のある工程の終了は１回以下
        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = 1;

        for t=1:time_max
            x.f(w,s,t) = 1;
        end

        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,UPPER_CONST);

        % (35) ある仕事のある工程の作業時間の関係式
        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = a_times(w,p);

        for t=1:time_max
            x.d(w,s,t) = 1;
        end

        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,EQUAL_CONST);

    end
end

% 初期値 Delta(t=1)=0
for w=1:work_max
    for s=1:shop_max

        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = 0;

        x.d(w,s,1)=1;
        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,EQUAL_CONST);

        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = 0;

        x.s(w,s,1)=1;
        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,EQUAL_CONST);

        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = 0;

        x.f(w,s,1)=1;
        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,EQUAL_CONST);
    end
end

% 終期値 Delta(t=end)=0
for w=1:work_max
    for s=1:shop_max

        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = 0;

        x.d(w,s,end)=1;
        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,EQUAL_CONST);

        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = 0;

        x.s(w,s,end)=1;
        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,EQUAL_CONST);

        x.c = 0;
        x.d = zeros(work_max, shop_max, time_max);
        x.s = zeros(work_max, shop_max, time_max);
        x.f = zeros(work_max, shop_max, time_max);
        y = 0;

        x.f(w,s,end)=1;
        A.c = cat(2,A.c, x.c(:));
        A.d = cat(2,A.d, x.d(:));
        A.s = cat(2,A.s, x.s(:));
        A.f = cat(2,A.f, x.f(:));
        b = cat(2,b,y);
        ctype = cat(2,ctype,EQUAL_CONST);
    end
end

% 実数／整数
vtype.c = repmat("C", 1, 1);
vtype.d = repmat("I", 1, prod([work_max, shop_max, time_max]));
vtype.s = repmat("I", 1, prod([work_max, shop_max, time_max]));
vtype.f = repmat("I", 1, prod([work_max, shop_max, time_max]));

% 目的関数
c.c = 1;
c.d = zeros(work_max, shop_max, time_max)(:);
c.s = zeros(work_max, shop_max, time_max)(:);
c.f = zeros(work_max, shop_max, time_max)(:);

% 非負制約
lb.c = 0;
lb.d = zeros(work_max, shop_max, time_max)(:)';
lb.s = zeros(work_max, shop_max, time_max)(:)';
lb.f = zeros(work_max, shop_max, time_max)(:)';
ub.c = time_max;
ub.d = ones (work_max, shop_max, time_max)(:)';
ub.s = ones (work_max, shop_max, time_max)(:)';
ub.f = ones (work_max, shop_max, time_max)(:)';

% 結合
c = cat(1, c.d, c.c, c.s, c.f);
A = cat(1, A.d, A.c, A.s, A.f)';
b = b';
lb = cat(2, lb.d, lb.c, lb.s, lb.f);
ub = cat(2, ub.d, ub.c, ub.s, ub.f);
ctype = ctype;
vtype = cat(2, vtype.d, vtype.c, vtype.s, vtype.f);

sense = MINIMIZE;
param.msglev = GLP_MSG_ON;
param.itlim = 5;

t=tic;
[xmin, fmin, status, extra] = glpk (c, A, b, lb, ub, ctype, vtype, sense, param);
toc(t)

disp("==== RESULT ====")
disp(["Costs:", num2str(fmin)])
disp("Variables:")

eidx = 0;

sidx = eidx+1;
eidx = sidx+length(x.d(:))-1;
xm.d = xmin(sidx:eidx);

sidx = eidx+1;
eidx = sidx+length(x.c(:))-1;
xm.c = xmin(sidx:eidx);

sidx = eidx+1;
eidx = sidx+length(x.s(:))-1;
xm.s = xmin(sidx:eidx);

sidx = eidx+1;
eidx = sidx+length(x.f(:))-1;
xm.f = xmin(sidx:eidx);

xm.d = reshape(xm.d, work_max, shop_max, time_max);
xm.s = reshape(xm.s, work_max, shop_max, time_max);
xm.f = reshape(xm.f, work_max, shop_max, time_max);

dispResult(xm.d);

dispJobSch(xm.d);
% dispJobSch(xm.s, "start");
% dispJobSch(xm.f, "finish");

dispShopSch(xm.d);
dispShopSch(xm.s, "start");
dispShopSch(xm.f, "finish");