# From LunarSphere/aerial-assembly brixzle/src/brixzle/drone_params.py @ a649663
# Changed: payload_max_g 30 -> 35 g (agreed in issue #9: battery headroom); kept masses, thrust, payload, thrust_max_N, t_w and margins, added thrust_margin_N; dropped flight time and dict IO. HoverGains are crazyflow's Mellinger position gains from brixzle/src/brixzle/cf/params.py (STATE2ATTITUDE) and the body inertia from its DYNAMICS, @ a649663.
"""Crazyflie 2.1 Brushless mass and thrust budget, and the hover spring used by the pickup trial.

Masses in grams, forces in newtons. The pickup trial (pickup.py) has no rotor or estimator model:
the drone is one rigid body pulled toward its setpoint by the Mellinger position PD, with its
weight compensated (thrust holds the hover) and its attitude held by a stiff rotational spring.
The position gains are crazyflow's; the attitude spring is an estimate (see HoverGains).
"""

from dataclasses import asdict, dataclass, field
import math

G = 9.81


@dataclass(frozen=True)
class DroneConfig:
    body_mass_g: float = 43.4          # crazyflow cf21B_500 (heavier than the datasheet's 34-37 g)
    deck_mass_g: float = 2.7           # Lighthouse deck
    thrust_source: str = 'datasheet'
    thrust_per_motor_N: dict = field(default_factory=lambda: {'datasheet': 0.2942, 'crazyflow': 0.2})
    tw_min: float = 1.5
    payload_max_g: float = 35.0        # issue #9: 35 g, leaving headroom for the battery

    def __post_init__(self):
        if self.thrust_source not in self.thrust_per_motor_N:
            raise ValueError(f'Unknown thrust_source {self.thrust_source!r}')
        values = [self.body_mass_g, self.deck_mass_g, self.tw_min, self.payload_max_g,
                  *self.thrust_per_motor_N.values()]
        if not all(math.isfinite(v) and v >= 0 for v in values):
            raise ValueError('Drone config values must be finite and non-negative')

    @property
    def mass_g(self):
        return self.body_mass_g + self.deck_mass_g

    def thrust_max_N(self, source=None):
        return 4*self.thrust_per_motor_N[source or self.thrust_source]

    def t_w(self, payload_g=0., source=None):
        return self.thrust_max_N(source)/((self.mass_g + payload_g)*1e-3*G)

    def thrust_margin_N(self, payload_g, source=None):
        """Thrust left over after hovering with this payload (N): max thrust minus total weight."""
        return self.thrust_max_N(source) - (self.mass_g + payload_g)*1e-3*G

    def margins(self, payload_g):
        """Signed margins (>= 0 satisfied) for the hard payload limit and T/W."""
        return {'payload_limit_g': self.payload_max_g - payload_g, 'tw': self.t_w(payload_g) - self.tw_min}

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class HoverGains:
    """Drone hold on its setpoint: force = kp*e + kd*de (N), per axis x, y, z (crazyflow Mellinger).

    ``att_hz`` and ``att_damping`` set the attitude spring: an estimate (not measured or taken from
    the firmware), a 10 Hz, 0.7-damped hold on the drone's inertia ``J``.
    """
    kp: tuple = (0.4, 0.4, 1.25)       # N/m, cf/params.py STATE2ATTITUDE kp
    kd: tuple = (0.2, 0.2, 0.5)        # N s/m, STATE2ATTITUDE kd
    J: tuple = (25e-6, 28e-6, 49e-6)   # kg m^2, cf/params.py DYNAMICS J (diagonal)
    att_hz: float = 10.0               # estimate
    att_damping: float = 0.7           # estimate

    def to_dict(self):
        return asdict(self)


CF21B = DroneConfig()
