---
name: procedural-bodywork
description: Generate curved car bodywork procedurally from parametric curves (shape-preserving profiles, superellipse sections, lofts and sweeps) as thin watertight shells with openings and convex collision patches that the car build checks can use. Use when designing or changing a car's bodywork, wings, bonnet or roof, or replacing box panels with coachwork.
---

# Procedural bodywork

Design bodywork as a few **curves and numbers**, not hand-placed panels. The
library is [experiments/006_benz_velo/bodywork.py](../../experiments/006_benz_velo/bodywork.py);
the worked example is the Traction Avant's
[body.py](../../experiments/012_citroen_traction_avant_1934/body.py). Bodywork
is car geometry, so the [car-downhill-check](../car-downhill-check/SKILL.md)
workflow still applies: build the chassis stage and pass its gate first, then
add the body and rerun every check.

## Think like a body designer, in three views

A body is fixed by lines read from the side, plan and front views. Write each
as a table of control points along x (forward from the rear axle):

- **Side view:** the roof line (the top), the sill or lower edge (the bottom),
  the belt line (window sills) and the cant rail (roof edge).
- **Plan view:** the half-width at the waist (the widest point).
- **Sections:** the cross-section shape at each station. This is a
  superellipse `|y/a|^n + |z/b|^n = 1` with separate upper and lower halves,
  plus tumblehome (the sides lean inward above the waist).
  - `n = 2` is an ellipse and `n → ∞` a box; 2.5–4.5 reads as coachwork.
  - A flat roof with rounded edges wants `n_up` ≈ 4; upright lower sides want
    `n_dn` ≈ 4.5.

Keep the number of control points small. Every point is a design decision that
someone must be able to justify.

## Principles (why each tool)

- **Profiles:** `Profile` is a shape-preserving (monotone) cubic Hermite
  curve. It passes through every point, is C1, and never overshoots. A
  natural cubic spline bulges past the designer's points, putting dents in a
  roof line.
- **Transitions between features:** use a cubic Bézier (`bezier`) whose inner
  control points lie along the tangents at both ends. The join is then
  tangent-continuous; wings use this from the arch down to the running board.
- **Tessellation from a tolerance:** a chord of length L on curvature κ
  deviates from the curve by the sagitta s ≈ L²κ/8. `Loft` gives each section
  interval the number of points this law requires at its most curved station,
  and adds stations until every section lies within `tol` of its neighbours'
  average. Never pick point counts by eye.
- **Lofts:** `Loft(section, keys, stations, tol)` for bodies that run along x
  (cabin, boot, bonnet). **Keys** are angles each station computes from the
  design lines (the sill edge, belt and cant). Openings are then clean
  rectangles in (station, key) space: `loft.hole(x0, x1, k0, k1)`. Put every
  opening's x limits in the station list so they are stations exactly.
- **Rounded ends:** shrink the section towards a point over the last few
  centimetres, as the Traction's tail does (a quarter-ellipse dome).
- **Sweeps:** `sweep(path, section, tol)` for parts that follow a path (wings,
  running boards, mouldings). The section rides on rotation-minimising frames
  (double reflection), so it never twists about the path. Section size can
  vary along the path: a wing narrows into a running board.
- **Shells, not clay:** panels are thin. `Shell` offsets the surface inward
  by the thickness and closes every boundary (outer edges and openings) into
  a watertight solid. A solid clay body would swallow the seats, engine and
  steering, and fail every clearance check.

## Collision: why convex patches are safe

MuJoCo collides only convex shapes, and a body shell is concave. The old
exporter turned any other mesh into its bounding box, so a curved wing would
collide as a solid block. `Shell.patches(tol)` cuts the built shell into
blocks that stay within `tol` of a plane, and each block collides as its
convex hull (`type: mesh` colliders).

- Each hull **contains** its block of the built mesh and exceeds it by at most
  `tol`. Clearances are therefore measured pessimistically by at most `tol`,
  and a pass is never false.
- The *design* surface bulges past each chord by up to `tol` too. **The part
  is the mesh, not the formula**, so test and render the mesh.
- Blocks are entirely panel or entirely opening, so no hull ever closes a
  window or a wheel opening.

Many hulls make many pairs. The build checks use a conservative
bounding-sphere broad phase (skip a pair only if its spheres are further
apart than both the required gap and the query range), so the exact distances
are unchanged. Expect about 800,000 pairs and roughly 3 minutes for a saloon.

## Workflow

1. **Chassis first.** The chassis stage must already pass its gate (see
   car-downhill-check).
2. **Package before styling.** List what the body must clear and attach to:
   - wheels at full lock with full bump and droop;
   - the steering wheel and column, the seats and the engine;
   - the sills, bulkhead and longerons it is welded to.

   Check the track against the body width. If the body is wider than the
   track (the Traction's rear), the wing is a flare *outboard* of the body
   side, and the tyre sits in an opening cut in the body.
3. **Write the design tables** in the car's `body.py`, and build the shells
   in plain Python first (no Blender). Every shell must pass `check_mesh`:
   watertight, outward, positive volume. A failure is raised, never ignored.
4. **Add them as parts** with `to_blender(..., patches=...)` and `part(...)`.
   - Declare real attachments as joins; the checks require them to touch at
     every pose. The body shell is welded to the sills and the bulkhead.
   - Declare mates only for true sliding or turning fits.
   - Stays and brackets must reach the panel (end them on the crown line).
5. **Run the full validation** (`run.py --stage full`): build checks, planted
   defects, the gate and every scenario. Read each failing pair as a design
   fact about the packaging. Change the design lines, not the tolerances.
6. **Inspect** the Cycles preview and the body-hidden views, and look for
   gaps between panels (bonnet to wing, cowl to scuttle). Grey primer showing
   through usually means structure that a unitary body would paint.

## Tests

`.venv/bin/python experiments/006_benz_velo/test_bodywork.py` checks the
maths against theory. There are 17 checks; they must all pass after any
library change:
- profiles interpolate, never overshoot and are C1;
- an n = 2 superellipse is an ellipse;
- refinement meets the tolerance, with about the sagitta-law point count;
- rotation-minimising frames are orthonormal and do not twist;
- a shell with a window is watertight and outward, with volume = area ×
  thickness;
- patches are planar, cover the panel (tested by MuJoCo point-in-hull) and
  leave the window open;
- MuJoCo places hull colliders where their vertices are and measures
  distances between them correctly.

## Honesty

Bodywork shapes are almost never sourced: say so in the car's README and
report. Keep the design tables in one place, and describe the shapes as
"styled on the general form" unless a drawing or dimension is cited.
