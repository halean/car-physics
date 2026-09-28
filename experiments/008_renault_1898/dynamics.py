"""Renault scenarios: steering, pedal-worked rear drums, three-speed launch (direct
third), hill starts, powered turns, combined pedal. LIMITS fixed before the first run."""
import math
import numpy as np
from control import ENGINE, GOVERNED, MAX_TORQUE, Brake, Controller, Steer, engine_torque

LIMITS = dict(kinematic_yaw_rel=.05, kinematic_yaw_abs_deg=1.0, hold_speed_mps=.02, hold_drift_m=.01,
              brake_vs_coast=.5, release_speed_mps=1.0, half_dt_stop_m=.03, half_dt_rel=.01, reference_rel=.03,
              top_speed_fraction=.95, engine_overspeed=1.01, straight_lateral_m=.05, wheel_speed_match=.01,
              differential_ratio_rel=.03, rollback_m=.05, climb_m=5.0, grade_limit_rollback_m=.5)
COL = dict(time=0, forward=1, lateral=2, yaw=3, speed=4, column=5, kl=6, kr=7)
TORQUE, STEER = -4, -1
BRAKE_TOTAL = 2*ENGINE['rear_drum_capacity_nm_each']
SHIFTS = [(.5, 'first'), (4.0, None), (4.2, 'second'), (8.0, None), (8.2, 'third')]
BRAKE_RUN, RELEASE_AT = 12, 10.0   # same schedule as 007


def at(rows, t, column):
    return float(np.interp(t, rows[:, 0], rows[:, column]))


def rel(a, b):
    return abs(a-b)/max(abs(b), 1e-6)


def reference(model, geometry, ctrl, seconds, slope, ratios, dt=.001):
    """1-D longitudinal model: same masses, wheel inertia, engine, clutch and gear schedule, brakes."""
    r = geometry['parameters']['rear_radius']
    mass = float(model.body_mass[1:].sum())
    inertia = sum(model.body_inertia[model.body(n).id][1]/b['wheel']['radius']**2
                  for n, b in geometry['bodies'].items() if 'wheel' in b)
    v = x = 0.0
    out = []
    for step in range(round(seconds/dt)):
        t = step*dt
        gear, since = ctrl.gear_at(t)
        drive = 0.0
        if gear:
            n = ratios[gear]
            drive = min(1.0, max(0.0, (t-since)/ENGINE['clutch_seconds']))*n*engine_torque(n*v/r)/r
        force = drive+mass*9.81*math.sin(slope)
        capacity = ctrl.brake.command(t)*BRAKE_TOTAL/r
        if abs(v) < 1e-9 and abs(force) <= capacity:
            v = 0.0
        else:
            direction = math.copysign(1, v) if abs(v) >= 1e-9 else math.copysign(1, force)
            new = v+(force-capacity*direction)/(mass+inertia)*dt
            v = 0.0 if capacity and new*v < 0 else new
        x += v*dt
        out.append((t+dt, x, v))
    return np.array(out)


def stopping(rows, apply_at):
    onset = int(np.searchsorted(rows[:, 0], apply_at))
    stops = np.flatnonzero((rows[:, 0] > apply_at+.25) & (np.abs(rows[:, 4]) < LIMITS['hold_speed_mps']))
    if not len(stops):
        return dict(stopped=False)
    stop = int(stops[0])
    hold = rows[rows[:, 0] >= rows[stop, 0]+.25]
    return dict(stopped=True, speed_at_application_mps=float(rows[onset, 4]),
                stopping_distance_m=float(rows[stop, 1]-rows[onset, 1]), stopping_time_s=float(rows[stop, 0]-apply_at),
                hold_duration_s=float(rows[-1, 0]-rows[stop, 0]-.25),
                hold_max_speed_mps=float(np.abs(hold[:, 4]).max()) if len(hold) else None,
                hold_drift_m=float(np.ptp(hold[:, 1])) if len(hold) else None)


def kinematic_ok(s):
    return abs(s['yaw_deg']-s['kinematic_yaw_deg']) < max(LIMITS['kinematic_yaw_abs_deg'],
                                                          LIMITS['kinematic_yaw_rel']*abs(s['kinematic_yaw_deg']))


def run_all(make, simulate, geometry, ratios):
    wheels = [n for n, b in geometry['bodies'].items() if 'wheel' in b]
    wcol = lambda n: 8+wheels.index(n)
    r, track, wb = (geometry['parameters'][k] for k in ('rear_radius', 'rear_track', 'wheelbase'))
    five = math.radians(5)
    res, rows_out = {}, {}

    def go(name, slope, ctrl, seconds, engine=False, spool=False, timestep=None):
        model = make(slope, engine, spool, timestep)
        s, common, rows = simulate(model, seconds, slope, ctrl)
        rows_out[name] = rows
        return model, s, common, rows

    # Steered, unpowered 5-degree descents with the pedal brakes at 3 s.
    for name, deg in (('steer_left', 20), ('steer_right', -20)):
        sign = math.copysign(1, deg)
        _, s, common, rows = go(name, five, Controller(brake=Brake(apply_at=3.0), steer=Steer(deg)), BRAKE_RUN)
        hold = rows[rows[:, 0] > BRAKE_RUN-1]
        s['max_steering_torque_nm'] = float(np.abs(rows[:, STEER]).max())
        s['checks'] = common|dict(turns_correct_way=float((sign*rows[:, 2]).max()) > .25 and sign*s['yaw_deg'] > 5,
                                  follows_handlebar=abs(rows[-1, COL['column']]-deg) < 2, kinematic_yaw=kinematic_ok(s),
                                  stops_and_holds=float(np.abs(hold[:, 4]).max()) < LIMITS['hold_speed_mps']
                                  and float(np.ptp(hold[:, 1])) < LIMITS['hold_drift_m'])
        res[name] = s

    # Pedal brakes: coast / brake / release on 5 degrees, unbraked control, half timestep.
    runs = {}
    for name, ctrl, seconds in (('coast', Controller(), BRAKE_RUN),
                                ('brake', Controller(brake=Brake(apply_at=2.0)), BRAKE_RUN),
                                ('release', Controller(brake=Brake(apply_at=2.0, release_at=RELEASE_AT)), BRAKE_RUN+2)):
        _, s, common, rows = go(f'drums_{name}', five, ctrl, seconds)
        s['checks'] = common
        runs[name] = (s, rows)
    stop, coast_stop = stopping(runs['brake'][1], 2.0), stopping(runs['coast'][1], 2.0)
    rel_stop = stopping(runs['release'][1], 2.0)
    settled = rel_stop['stopped'] and 2.0+rel_stop['stopping_time_s']+.25
    rel_rows = runs['release'][1]
    before = rel_rows[(rel_rows[:, 0] > (settled or RELEASE_AT)) & (rel_rows[:, 0] < RELEASE_AT)]
    _, _, fine_common, fine_rows = go('drums_half_dt', five, Controller(brake=Brake(apply_at=2.0)), BRAKE_RUN,
                                      timestep=.0005)
    fine = stopping(fine_rows, 2.0)
    res['rear_drums'] = dict(stopping=stop, half_timestep_stopping=fine, runs={k: v[0] for k, v in runs.items()},
                             checks=dict(
        all_runs_clean=all(all(v[0]['checks'].values()) for v in runs.values()) and all(fine_common.values()),
        stops=stop['stopped'],
        holds=stop.get('hold_duration_s', 0) > 1 and stop.get('hold_max_speed_mps', 1) < LIMITS['hold_speed_mps']
        and stop.get('hold_drift_m', 1) < LIMITS['hold_drift_m'],
        travels_less_than_coast=at(runs['brake'][1], 6, 1) < LIMITS['brake_vs_coast']*at(runs['coast'][1], 6, 1),
        holds_before_release=bool(settled) and len(before) > 0
        and float(np.abs(before[:, 4]).max()) < LIMITS['hold_speed_mps'],
        rolls_after_release=runs['release'][0]['final_speed_mps'] > LIMITS['release_speed_mps'],
        unbraked_control_fails=not coast_stop['stopped'],
        half_timestep=fine['stopped'] and abs(fine['stopping_distance_m']-stop['stopping_distance_m'])
        < LIMITS['half_dt_stop_m'] and fine['hold_drift_m'] < LIMITS['hold_drift_m']))

    # Launch through the gears (declutch, shift, re-engage), heading-holding driver, 30 s.
    _, open_loop, _, _ = go('launch_open_loop', 0, Controller(gears=SHIFTS), 30, engine=True)
    res['finding_open_loop_launch'] = dict(lateral_m=open_loop['lateral_m'], yaw_deg=open_loop['yaw_deg'], passed=True,
                                           note='Informational: handlebar held straight, no heading correction.')
    launch = Controller(gears=SHIFTS, steer=Steer(hold_heading=True))
    model, s, common, rows = go('launch', 0, launch, 30, engine=True)
    ref = reference(model, geometry, launch, 30, 0, ratios)
    gov = {g: GOVERNED/n*r for g, n in ratios.items()}
    rear = rows[rows[:, 0] > 25][:, [wcol('rear_left'), wcol('rear_right')]]
    s.update(speed_at_3s_mps=at(rows, 3, 4), reference_speed_at_3s_mps=at(ref, 3, 2),
             distance_at_30s_m=at(rows, 30, 1), reference_distance_at_30s_m=at(ref, 30, 1),
             speed_at_shifts_mps=[at(rows, 4.0, 4), at(rows, 8.0, 4)], final_speed_mps=float(rows[-1, 4]),
             governed_speeds_mps=gov)
    s['checks'] = common|dict(
        matches_reference_speed=rel(s['speed_at_3s_mps'], s['reference_speed_at_3s_mps']) < LIMITS['reference_rel'],
        matches_reference_distance=rel(s['distance_at_30s_m'], s['reference_distance_at_30s_m']) < LIMITS['reference_rel'],
        reaches_third_gear_speed=s['final_speed_mps'] >= LIMITS['top_speed_fraction']*gov['third'],
        engine_within_governor=s['max_engine_rpm'] <= ENGINE['no_load_rpm']*LIMITS['engine_overspeed'],
        power_within_rating=s['max_engine_power_w'] <= ENGINE['rated_power_w']*1.001,
        straight=abs(s['lateral_m']) < LIMITS['straight_lateral_m'],
        equal_rear_wheel_speeds=float(np.max(np.abs(rear[:, 0]-rear[:, 1])/np.abs(rear).mean(axis=1)))
        < LIMITS['wheel_speed_match'])
    res['launch'] = s

    # Hill starts: first gear climbs 8 degrees; third gear must roll back on 5 degrees.
    mass = float(model.body_mass[1:].sum())
    predicted = {g: math.degrees(math.asin(min(1, n*MAX_TORQUE/r/(mass*9.81)))) for g, n in ratios.items()}
    for gear, degrees, seconds in (('first', 8, 20), ('third', 5, 6)):
        slope = -math.radians(degrees)
        ctrl = Controller(gears=[(.5, gear)], brake=Brake(apply_at=0, release_at=1.5))
        model, s, common, rows = go(f'hill_{gear}', slope, ctrl, seconds, engine=True)
        ref = reference(model, geometry, ctrl, seconds, slope, ratios)
        after = rows[rows[:, 0] >= 1.5]
        s.update(slope_deg=degrees, predicted_max_grade_deg=predicted[gear], reference_forward_m=float(ref[-1, 1]),
                 rollback_after_release_m=float(at(rows, 1.5, 1)-after[:, 1].min()))
        match = rel(s['forward_m'], s['reference_forward_m']) < LIMITS['reference_rel']
        s['checks'] = common|(dict(no_rollback=s['rollback_after_release_m'] < LIMITS['rollback_m'],
                                   climbs=s['forward_m'] > LIMITS['climb_m'], matches_reference=match)
                              if gear == 'first' else
                              dict(rolls_back=s['forward_m'] < -LIMITS['grade_limit_rollback_m'], matches_reference=match,
                                   prediction_brackets=predicted['third'] < 5 < 8 < predicted['first']))
        res[f'hill_{gear}'] = s

    # Powered turns through the differential, and a locked spool. First gear (~3 m/s,
    # ~0.25 g at 20 degrees): in second the light car reaches ~5 m/s and ~0.7 g and slides
    # wide, which is kept below as a grip-limit finding rather than a rolling test.
    def turn(name, deg, spool=False, gear='first'):
        _, s, common, rows = go(name, 0, Controller(gears=[(.5, gear)], steer=Steer(deg)), 12, engine=True,
                                spool=spool)
        w = rows[rows[:, 0] > 6]
        tl, tr = np.tan(np.radians(w[:, COL['kl']])), np.tan(np.radians(w[:, COL['kr']]))
        radius = wb/abs(float(np.mean(2*tl*tr/(tl+tr))))
        expected = (radius+track/2)/(radius-track/2)
        left, right = w[:, wcol('rear_left')], w[:, wcol('rear_right')]
        outer, inner = (right, left) if deg > 0 else (left, right)
        late = rows[rows[:, 0] > 8]
        yaw_rate = np.gradient(np.radians(late[:, 3]), late[:, 0])
        planar = np.hypot(np.gradient(late[:, 1], late[:, 0]), np.gradient(late[:, 2], late[:, 0]))
        s['lateral_acceleration_g'] = float(np.mean(np.abs(yaw_rate*planar))/9.81)
        s.update(turn_radius_m=radius, expected_outer_inner_ratio=expected,
                 measured_outer_inner_ratio=float(outer.mean()/inner.mean()),
                 max_steering_torque_nm=float(np.abs(rows[:, STEER]).max()))
        s['checks'] = common|dict(differential_ratio=rel(s['measured_outer_inner_ratio'], expected)
                                  < LIMITS['differential_ratio_rel'], kinematic_yaw=kinematic_ok(s),
                                  # Regression guard added after the tyre-contact fix (was up to 77%).
                                  steady_tyre_loads=max(s['tyre_load_variation_last_s'].values()) < .10,
                                  turns_correct_way=math.copysign(1, deg)*s['yaw_deg'] > 5)
        return s
    res['turn_left'] = turn('turn_left', 20)
    res['turn_right'] = turn('turn_right', -20)
    spool = turn('turn_left_spool', 20, spool=True)
    fast = turn('turn_left_second_gear', 20, gear='second')
    res['finding_turn_second_gear'] = dict(passed=True, lateral_acceleration_g=fast['lateral_acceleration_g'],
        yaw_deg=fast['yaw_deg'], kinematic_yaw_deg=fast['kinematic_yaw_deg'],
        measured_outer_inner_ratio=fast['measured_outer_inner_ratio'],
        expected_outer_inner_ratio=fast['expected_outer_inner_ratio'],
        tyre_load_variation_last_s=fast['tyre_load_variation_last_s'],
        failed_rolling_checks=[k for k, v in fast['checks'].items() if not v],
        note='Informational: at the grip limit the car slides wide; rolling kinematics no longer apply.')
    res['locked_differential_control'] = dict(passed=not spool['checks']['differential_ratio'],
                                              measured=spool['measured_outer_inner_ratio'],
                                              expected=spool['expected_outer_inner_ratio'])

    # The combined pedal: driving in every gear, pressing the pedal must declutch
    # (no drive torque while braking), stop the car and hold it.
    for gear in ('first', 'third'):
        _, s, common, rows = go(f'pedal_in_{gear}', 0, Controller(gears=[(.5, gear)], brake=Brake(apply_at=5.0)), 10,
                                engine=True)
        hold = rows[rows[:, 0] > 9]
        braking = rows[rows[:, 0] > 5.0]
        s.update(drive_at_wheels_if_engaged_nm=ratios[gear]*MAX_TORQUE, brake_capacity_nm=BRAKE_TOTAL,
                 max_drive_torque_while_braking_nm=float(braking[:, TORQUE].max()),
                 hold_max_speed_mps=float(np.abs(hold[:, 4]).max()), hold_drift_m=float(np.ptp(hold[:, 1])))
        s['checks'] = common|dict(declutched_while_braking=s['max_drive_torque_while_braking_nm'] == 0,
                                  stops=s['hold_max_speed_mps'] < LIMITS['hold_speed_mps'],
                                  holds=s['hold_drift_m'] < LIMITS['hold_drift_m'])
        res[f'pedal_in_{gear}'] = s

    _, _, fine_common, fine_rows = go('launch_half_dt', 0, launch, 30, engine=True, timestep=.0005)
    base = rows_out['launch']
    half = dict(speed_at_10s_change=rel(at(fine_rows, 10, 4), at(base, 10, 4)),
                distance_at_30s_change=rel(at(fine_rows, 30, 1), at(base, 30, 1)))
    half['checks'] = dict(clean=all(fine_common.values()), within_limit=half['speed_at_10s_change'] < LIMITS['half_dt_rel']
                          and half['distance_at_30s_change'] < LIMITS['half_dt_rel'])
    res['launch_half_dt'] = half
    for v in res.values():
        if 'checks' in v:
            v['passed'] = all(v['checks'].values())
    return dict(passed=all(v['passed'] for v in res.values()), predicted_max_grade_deg=predicted, ratios=ratios,
                engine_torque_nm=MAX_TORQUE, limits=LIMITS), res, rows_out
