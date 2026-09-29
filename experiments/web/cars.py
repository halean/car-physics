"""How each car's real controls map onto the shared driving controls of the web simulator.

Shared controls: a hand throttle (0-1, stays where set), a brake pedal (0-1, momentary), a brake
lever (0-1, latches), a gear selector and steering. Each car maps them onto its own mechanisms,
with the same devices and capacities its validation used (engine.json, control.py):
- a brake device is 'joints' (a drum, band or block on each rear wheel: joint friction, N m each)
  or 'tendon' (a driveline brake acting through the differential: N m at the wheels)
- pedal/lever None: the car has no such control; the UI says so
Names and descriptions come from each experiment's README and sources.
"""

CARS = [
    dict(
        id='panhard_1891', exp='007_panhard_1891', blend='panhard_1891.blend', year=1891,
        name='Panhard et Levassor', short='Panhard 1891',
        gears=[('first', '1st'), ('second', '2nd'), ('third', '3rd')], shift_key='clutch_seconds',
        pedal=dict(label='Brake lever', detail='Rim blocks on the rear tyres, worked by a lever',
                   devices=[dict(kind='joints', capacity_key='rim_brake_capacity_nm_each')]),
        lever=None, lever_note='The rim-block lever is the only brake; it is on the brake control.',
        declutch_on_pedal=False, steering=dict(kind='Tiller', box_ratio=1.0),
        controls='Tiller steering, a clutch and a three-speed sliding-gear lever. The engine runs '
                 'at its governed speed; the brake is a lever pressing blocks on the rear tyres.',
        parity=dict(first='first', brake={}),
    ),
    dict(
        id='benz_velo_1894', exp='006_benz_velo', blend='benz_velo.blend', year=1894,
        name='Benz Velo', short='Benz Velo 1894',
        gears=[('low', 'Low belt'), ('high', 'High belt')], shift_key='belt_shift_seconds',
        pedal=dict(label='Foot brake', detail='A band on the countershaft drum, through the differential',
                   devices=[dict(kind='tendon', capacity_key='brake_capacity_nm_at_wheels')]),
        lever=dict(label='Hand brake', detail='A band on a drum at each rear sprocket',
                   devices=[dict(kind='joints', capacity_key='rear_drum_capacity_nm_each')]),
        declutch_on_pedal=False, steering=dict(kind='Crank handwheel', box_ratio=1.0),
        controls='A crank-handle wheel on a vertical column, a belt-shift lever for the two speeds, '
                 'a foot band brake and a hand brake. The governed engine sets the pace.',
        parity=dict(first='low', brake=dict(kind='band')),
    ),
    dict(
        id='renault_1898', exp='008_renault_1898', blend='renault_1898.blend', year=1898,
        name='Renault Type A', short='Renault 1898',
        gears=[('first', '1st'), ('second', '2nd'), ('third', '3rd, direct')], shift_key='clutch_seconds',
        pedal=dict(label='Combined pedal', detail='Declutches, then brakes the rear drums',
                   devices=[dict(kind='joints', capacity_key='rear_drum_capacity_nm_each')]),
        lever=None, lever_note='The single pedal does both jobs; this model has no separate hand brake.',
        declutch_on_pedal=True, steering=dict(kind='Handlebar', box_ratio=1.0),
        controls='Handlebar steering and a three-speed gearbox with a direct-drive third. '
                 'One pedal declutches and then brakes.',
        parity=dict(first='first', brake={}),
    ),
    dict(
        id='mercedes_1901', exp='009_mercedes_35hp_1901', blend='mercedes_35hp.blend', year=1901,
        name='Mercedes 35 HP', short='Mercedes 1901',
        gears=[('first', '1st'), ('second', '2nd'), ('third', '3rd'), ('fourth', '4th')], shift_key='clutch_seconds',
        pedal=dict(label='Foot brake', detail='A water-cooled drum on the countershaft, through the differential',
                   devices=[dict(kind='tendon', capacity_key='foot_brake_capacity_nm_at_wheels')]),
        lever=dict(label='Hand brake', detail='30 cm drums on the rear wheels',
                   devices=[dict(kind='joints', capacity_key='rear_drum_capacity_nm_each')]),
        declutch_on_pedal=False, steering=dict(kind='Steering wheel', box_ratio=6.0),
        controls='A raked steering wheel through a 6:1 box, a gate-change four-speed lever, a foot brake '
                 'on the countershaft and a hand brake on the rear drums.',
        suspension=dict(front=dict(joints=['front_axle_heave', 'front_axle_roll'], lever=None, sign=1),
                        rear=dict(joints=['rear_axle_swing', 'rear_axle_roll'], lever='radius_rod_m', sign=1)),
        parity=dict(first='first', brake=dict(kind='foot')),
    ),
    dict(
        id='model_t_1909', exp='010_ford_model_t_1909', blend='model_t.blend', year=1909,
        name='Ford Model T', short='Model T 1909',
        gears=[('low', 'Low pedal'), ('high', 'High (lever)')], shift_key='clutch_seconds',
        pedal=dict(label='Brake pedal', detail='The transmission brake, through the differential',
                   devices=[dict(kind='tendon', capacity_key='foot_brake_capacity_nm_at_wheels')]),
        lever=dict(label='Hand lever', detail='Bands on the rear hubs',
                   devices=[dict(kind='joints', capacity_key='rear_drum_capacity_nm_each')]),
        declutch_on_pedal=False, steering=dict(kind='Steering wheel', box_ratio=4.0),
        controls='Three pedals (low, reverse, brake), a hand lever for high gear and the parking brake, '
                 'and a hand throttle on the right of the steering column.',
        suspension=dict(front=dict(joints=['front_axle_swing', 'front_axle_roll'], lever='front_radius_rod_m', sign=-1),
                        rear=dict(joints=['rear_axle_swing', 'rear_axle_roll'], lever='radius_rod_m', sign=1)),
        parity=dict(first='low', brake=dict(kind='foot')),
    ),
    dict(
        id='model_t_shocks', exp='010_ford_model_t_1909', stage='shocks', blend='model_t.blend', year=1909,
        name='Ford Model T with friction shocks', short='Model T + shocks',
        gears=[('low', 'Low pedal'), ('high', 'High (lever)')], shift_key='clutch_seconds',
        pedal=dict(label='Brake pedal', detail='The transmission brake, through the differential',
                   devices=[dict(kind='tendon', capacity_key='foot_brake_capacity_nm_at_wheels')]),
        lever=dict(label='Hand lever', detail='Bands on the rear hubs',
                   devices=[dict(kind='joints', capacity_key='rear_drum_capacity_nm_each')]),
        declutch_on_pedal=False, steering=dict(kind='Steering wheel', box_ratio=4.0),
        controls='As the Model T, plus aftermarket Hartford-type friction shock absorbers: a scissor at '
                 'each corner whose knee is clamped between friction discs.',
        suspension=dict(front=dict(joints=['front_axle_swing', 'front_axle_roll'], lever='front_radius_rod_m', sign=-1),
                        rear=dict(joints=['rear_axle_swing', 'rear_axle_roll'], lever='radius_rod_m', sign=1)),
        shocks=dict(joints=[f'shock_{a}_{s}_b_joint' for a in ('front', 'rear') for s in ('left', 'right')]),
        parity=dict(first='low', brake=dict(kind='foot')),
    ),
]
