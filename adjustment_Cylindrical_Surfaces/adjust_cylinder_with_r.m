% JX = K + V (AV = W + L)
% X(=x_lowercase) = inv(J'*J)*J'*K
function [x_lowercase, X_hat, v, sigma, covariance] = adjust_cylinder_with_r(X_ini, points)
%% ­pºâªñ¦ü­È¼W¶q(x_lowercase)¤Î¥­®t­È(X_hat)
N = zeros(7, 7);
M = zeros(7, 1);

% if length(X_ini)<5
%     kkkkk=0;
% end



a = X_ini(1); b = X_ini(2); c = X_ini(3); d = X_ini(4); e = X_ini(5); f = X_ini(6); r = X_ini(7);
    for i = 1:length(points)
        x = points(i,1);
        y = points(i,2);
        z = points(i,3);
        
        % Partial differential (function-diff)
        % syms x y z a b c d e f r
        % equation sqrt((d - x)^2 + (e - y)^2 + (f - z)^2 - ((a*(d - x) + b*(e - y) + c*(f - z))^2/(a^2 + b^2 + c^2))) = r;
        % constraint E: a^2 + b^2 + c^2 = 1
        % constraint P: a*d + b*e + e*f = 0

        A = -((2*(d - x)*(a*(d - x) + b*(e - y) + c*(f - z)))/(a^2 + b^2 + c^2) - (2*a*(a*(d - x) + b*(e - y) + c*(f - z))^2)/(a^2 + b^2 + c^2)^2)/(2*((d - x)^2 - (a*(d - x) + b*(e - y) + c*(f - z))^2/(a^2 + b^2 + c^2) + (e - y)^2 + (f - z)^2)^(1/2));
        B = -((2*(e - y)*(a*(d - x) + b*(e - y) + c*(f - z)))/(a^2 + b^2 + c^2) - (2*b*(a*(d - x) + b*(e - y) + c*(f - z))^2)/(a^2 + b^2 + c^2)^2)/(2*((d - x)^2 - (a*(d - x) + b*(e - y) + c*(f - z))^2/(a^2 + b^2 + c^2) + (e - y)^2 + (f - z)^2)^(1/2));
        C = -((2*(f - z)*(a*(d - x) + b*(e - y) + c*(f - z)))/(a^2 + b^2 + c^2) - (2*c*(a*(d - x) + b*(e - y) + c*(f - z))^2)/(a^2 + b^2 + c^2)^2)/(2*((d - x)^2 - (a*(d - x) + b*(e - y) + c*(f - z))^2/(a^2 + b^2 + c^2) + (e - y)^2 + (f - z)^2)^(1/2));
        D = -(2*x - 2*d + (2*a*(a*(d - x) + b*(e - y) + c*(f - z)))/(a^2 + b^2 + c^2))/(2*((d - x)^2 - (a*(d - x) + b*(e - y) + c*(f - z))^2/(a^2 + b^2 + c^2) + (e - y)^2 + (f - z)^2)^(1/2));
        E = -(2*y - 2*e + (2*b*(a*(d - x) + b*(e - y) + c*(f - z)))/(a^2 + b^2 + c^2))/(2*((d - x)^2 - (a*(d - x) + b*(e - y) + c*(f - z))^2/(a^2 + b^2 + c^2) + (e - y)^2 + (f - z)^2)^(1/2));
        F = -(2*z - 2*f + (2*c*(a*(d - x) + b*(e - y) + c*(f - z)))/(a^2 + b^2 + c^2))/(2*((d - x)^2 - (a*(d - x) + b*(e - y) + c*(f - z))^2/(a^2 + b^2 + c^2) + (e - y)^2 + (f - z)^2)^(1/2));
        R = -1;
                
        %N matrix
        J = [A, B, C, D, E, F, R];
        n = J'*J;
        N = N + n;
        K = 0 - (sqrt((d - x)^2 + (e - y)^2 + (f - z)^2 - ((a*(d - x) + b*(e - y) + c*(f - z))^2/(a^2 + b^2 + c^2))) - r);
        m = J'*K;
        M = M + m;
    end
    E = [2*a, 2*b, 2*c, 0, 0, 0, 0];
    P = [d, e, f, a, b, c, 0];
    N = N + E'*99999*E;
    N = N + P'*99999*P;
    M = M + E'*(1 -  (a^2 + b^2 + c^2));
    M = M + P'*(- (a*d + b*e + c*f));

    x_lowercase = N\M; %ªñ¦ü­È¼W¶q

    % if rcond(N)< 3.2432e-7
     % rcond(N)
    % end

    X_hat = X_ini + x_lowercase; %¥­®t­È
    
    %% ­pºâ§ï¥¿¼Æ(v)¤Î³æ¦ìÅv¤¤»~®t(sigma)¤Î¤è®t¨ó¤è®t/¨ó¤è®t(covariance)
    v = zeros(length(points), 1);
    a = X_hat(1); b = X_hat(2); c = X_hat(3); d = X_hat(4); e = X_hat(5); f = X_hat(6); r = X_hat(7);
    for j = 1:length(points)
        x = points(j,1);
        y = points(j,2);
        z = points(j,3);
        v(j) = 0 - (sqrt((d - x)^2 + (e - y)^2 + (f - z)^2 - ((a*(d - x) + b*(e - y) + c*(f - z))^2/(a^2 + b^2 + c^2))) - r);
    end
    %³æ¦ìÅv¤¤»~®t
    sigma = sqrt((v'*v)/(length(points)-7));
    %¤è®t¨ó¤è®t/¨ó¤è®t
    covariance = (sigma^2)*inv(N);
end