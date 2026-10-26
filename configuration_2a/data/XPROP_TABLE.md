# XROTOR propeller table – content, validity and limits

Applies to `xprop_data.mat` (data set **Old**) and `xprop_data_new.mat`
(data set **New**), both built by `xprop_build_matfile.m` from the XROTOR
text output in this folder. Propeller: D = 2.35 m, 4 blades, 1225 RPM,
zero pitch 54.7° at r/R = 0.7.

## Selecting a data set

```matlab
aircraft = aircraftSyntracCreate();                    % Old (default)
aircraft = aircraftSyntracCreate('PropData', 'New');   % New
```

Both tables use the same grid (31 pitch × 26 J), so the Simulink bus is
identical. Rebuild and review both tables with `configuration_2a/xprop_review_data.m`.

## What the grid contains

Every grid (`Ct_mat`, `Cp_mat`, `dCt_dbeta_mat`, `dCp_dbeta_mat`,
`dCt_dJ_mat`, `dCp_dJ_mat`) is complete: no NaN and no zero padding,
smooth over the full grid. **A complete grid does not mean the values are
valid everywhere.** The nodes fall into these classes:

| Node class | Mask | Origin | May be used for analysis? |
|---|---|---|---|
| XROTOR data | `data_mat` | delivered XROTOR point, never modified | yes, if `valid_mat` |
| Trusted hole | `trusted_mat & filled_mat` | single missing point, spline-filled; J- and pitch-spline agree | yes, if `valid_mat` |
| Untrusted hole | `filled_mat & ~trusted_mat` | larger gap (XROTOR did not converge), minimum-curvature fill | **no** |
| Extrapolated | `~data_mat & ~filled_mat` | beyond zero thrust: windmill/brake placeholder | **no** |

Derivatives are second-order finite differences of the final Ct/Cp grid,
so values and derivatives are always consistent with each other.

## `valid_mat` – where the table is backed by data

A node is valid only if its value **and every node in its derivative
stencils** are trusted (XROTOR data or trusted hole). This means:

- Everything inside `valid_mat` is either XROTOR output or was
  interpolated from a complete neighbourhood of XROTOR points.
  Leave-one-out tests give errors with a 95th percentile of 1.5e-3 (Ct)
  and 3e-3 (Cp) for such fills.
- At the edge of the data and around untrusted holes, one node is trimmed,
  because the derivatives there would depend on non-data values.

| | Old | New |
|---|---|---|
| XROTOR points | 703 | 605 |
| Missing interior points (holes) | 0 | 61 (9 trusted) |
| Valid nodes | 687 | 535 |

## `stall_mat` – stalled side of each pitch row

`stall_mat` marks, for each pitch row, all nodes at or below the J of the
Ct maximum, plus one node of margin. There the blade sections run beyond
CLmax: Ct rises with J, and **dCt/dβ ≈ 0 or negative** (pitch no longer
adds thrust, it only costs power). The ridge in the dβ derivatives at the
stall boundary is physics, not an interpolation artefact. Stalled nodes can
be valid (they are XROTOR data), but XROTOR's 2-D post-stall model makes
them low-confidence.

In data set New, 59 of the 61 missing points lie inside `stall_mat`: XROTOR
did not converge just past the stall break and in deep stall.

## Operating regions

```
 pitch ↑
       |  blade stall (high power)          stall_mat
       |  ─────────────── stall boundary (Ct max per row)
       |  valid operating band              valid_mat & ~stall_mat
       |  ─────────────── zero thrust (Ct = 0), end of data
       |  brake / windmill, no data         placeholder only
       +──────────────────────────────────────────→ J
```

- **Valid operating band:** use without restriction.
- **Blade stall:** XROTOR data, low confidence. Control effectiveness
  dCt/dβ collapses and can change sign.
- **Beyond zero thrust (low pitch, high J):** no delivered data. The grid
  contains a smooth placeholder so that trim, linearization and solvers
  do not break when they pass through this region.

## The placeholder beyond zero thrust

Each pitch row is continued past its last data point with a windmill/brake
shape in four-quadrant scaling,
CT\* = 8 Ct / (π W²), CP\* = 8 Cp / (π² · 0.7 · W²), W² = J² + (0.7π)²,
as a function of the geometric angle of attack at 0.7 R. The shape was
fitted to the 118 Ct < 0 points of the unfiltered XROTOR run of the New
data set, which were removed from the delivered data. It is scaled per row
to the row's own slope, then smoothed in 2-D with the data held fixed.
The same shape is assumed for the Old data set.

- In data set New, the placeholder reproduces XROTOR's own Ct < 0 output
  with a median deviation of 0.005 in Ct and Cp. The previous linear
  extrapolation deviated by 0.06 (Ct) and 0.16 (Cp).
- **This is not validated data.** XROTOR itself is unreliable in the
  windmill and brake states (heavily loaded turbine state, negative
  stall). Do not use these values for performance, handling-qualities or
  controller-design results.

## Known limitations and open items

1. **Missing points (New):** 61 points, mainly in the stall band at pitch
   +5…+15°, were never computed (XROTOR non-convergence). The data
   provider needs to rerun them, e.g. as a continuation from converged
   neighbours or with softer stall settings (CLmax / dCLstall).
2. **Windmill/brake region:** reliable values need a dedicated method,
   e.g. BEM with 360° airfoil polars (Viterna–Corrigan) and a high-induction
   correction, validated against four-quadrant or wind-tunnel data. This
   is not part of the current table.
3. **Runtime interpolation** is bilinear, so evaluated surfaces are
   continuous with kinks at cell borders. The precomputed derivative grids
   are the consistent source for gradients.
