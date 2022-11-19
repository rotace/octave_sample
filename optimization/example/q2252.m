% 2.25.2 フローショップ問題
% https://www.msi.co.jp/nuopt/docs/v23/examples/html/02-25-02.html
% https://www.jstage.jst.go.jp/article/isciesci/61/1/61_14/_pdf/-char/ja
clear all
common

work_max = 3;
mach_max = 3;
time_max = 30;

% 仕事iのj番目の工程の処理時間
pp = [5 8 4;
      5 4 6;
      6 5 5];
% 仕事iのj番目の工程に用いる機械
mu = [1 2 3;
      1 2 3;
      1 2 3];

% 制約
A = [];
b = [];
ctype = "";

% (36) コストの定義
for w=1:work_max
    m = mu(w,end);
    x = zeros(work_max, mach_max, time_max);
    for t=1:time_max
        x(w,m,t) = -t; % t x delta
    end
    x = [x(:); 1]; % Cmax
    A = cat(2,A,x);
    b = cat(2,b,pp(w,end)); % p
    ctype = cat(2,ctype,"L");
end

% (37) 工程処理順序の制約
for w=1:work_max
    len = length(mu(w,:))-1;
    for i=1:len
        m1 = mu(w,i);
        m2 = mu(w,i+1);
        x = zeros(work_max, mach_max, time_max);
        for t=1:time_max
            x(w,m1,t) = -t; % t x delta
        end
        for t=1:time_max
            x(w,m2,t) = +t; % t x delta
        end
        x = [x(:); 0]; % Cmax
        A = cat(2,A,x);
        b = cat(2,b,pp(w,i)); % p
        ctype = cat(2,ctype,"L");
    end
end

% (38) 同時処理禁止の制約
for a=1:work_max
    len = length(mu(a,:));
    for i=1:len
        m = mu(a,i);
        p = pp(a,i);
        for t=1:(time_max-p-1)
            x = zeros(work_max, mach_max, time_max);
            for k=1:work_max
                if k==a
                    % this work
                    x(k,m,t) = 100;
                else
                    % other work
                    for j=t:(t+p-1)
                        x(k,m,j) = 1;
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

% (39) ある機械におけるある仕事の実行回数は１
for w=1:work_max
    for m=1:mach_max
        x = zeros(work_max, mach_max, time_max);
        for t=1:time_max
            x(w,m,t) = 1;
        end
        x = [x(:); 0]; % Cmax
        A = cat(2,A,x);
        b = cat(2,b,1); % 1
        ctype = cat(2,ctype,"S");
    end
end

A = A';
b = b';
vartype = repmat("I", 1, size(A)(2));
vartype(end) = "C";

% 目的関数
c = zeros(1, size(A)(2))';
c(end) = 1;

% 非負制約
lb = zeros(1, size(A)(2));
ub = ones (1, size(A)(2));
ub(end) = time_max;

sense = MINIMIZE;
param.msglev = GLP_MSG_ALL;
param.itlim = 100;

[xmin, fmin, status, extra] = glpk (c, A, b, lb, ub, ctype, vartype, sense, param);

disp("==== RESULT ====")
disp(["Costs:", num2str(fmin)])
disp("Variables:")
Cmax = xmin(end)
xmin = reshape(xmin(1:end-1), work_max, mach_max, time_max);
for w=1:work_max
    for m=1:mach_max
        [~,idx] = max(xmin(w,m,:));
        disp(["[", num2str(w), ",", num2str(m), "]= ", num2str(idx-1)])
    end
end

