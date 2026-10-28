function [T, Q, P, vi, v_slipstream, swirls, eff] = ...
    propeller_model_sfcn(tbl, RPM, pitch_delta, V, rho, R)

    assert(isequal(size(RPM), size(pitch_delta)) && isequal(size(RPM), size(V)), ...
        'RPM, pitch_delta, and V must all have the same size.');

    n = numel(RPM);

    % Pre-allocate outputs
    T            = zeros(n,1);
    Q            = zeros(n,1);
    P            = zeros(n,1);
    vi           = zeros(n,1);
    v_slipstream = zeros(n,1);
    eff          = zeros(n,1);

    for i = 1:n
         Ri = R(min(i, numel(R)));       % R scalar or one value per propeller
         [T(i), Q(i), P(i), vi(i), v_slipstream(i), ~, ~, ~, eff(i)] = ...
             propeller_aero(tbl, RPM(i), pitch_delta(i), V(i), rho, 2*Ri);
    end
    swirls = inducedSwirl(Q, V(:)+vi, R, rho);

end