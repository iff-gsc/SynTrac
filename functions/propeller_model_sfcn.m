function [T, Q, P, vi, v_slipstream, eff, ...
          dT_dbeta, dQ_dbeta, dvi_dbeta, ...
          dT_dV, dQ_dV, dvi_dV] = ...
    propeller_model_sfcn(tbl, RPM, pitch_delta, V, rho)

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
    dT_dbeta     = zeros(n,1);
    dQ_dbeta     = zeros(n,1);
    dvi_dbeta    = zeros(n,1);
    dT_dV        = zeros(n,1);
    dQ_dV        = zeros(n,1);
    dvi_dV       = zeros(n,1);

    for i = 1:n
         [T(i), Q(i), P(i), vi(i), v_slipstream(i), ~, ~, ~, eff(i)] = ...
             propeller_aero(tbl, RPM(i), pitch_delta(i), V(i), rho);
 
         [dT_dbeta(i), dQ_dbeta(i), dvi_dbeta(i), dT_dV(i), dQ_dV(i), dvi_dV(i)] = ...
             propeller_derivatives(tbl, RPM(i), pitch_delta(i), V(i), rho);
    end

end