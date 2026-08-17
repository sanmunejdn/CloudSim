"""非标设计计算资产（自《非标设计最强自动计算》改写，SI 优先）。"""

from .motor import motor_power, lookup_y_motor
from .reducer import reducer_rated_power
from .load import load_torque_ballscrew
from .inertia import inertia_cylinder
from .gear import gear_spur_helical_shift, gear_rack, gear_high_shift_dims, lookup_gear_material
from .worm import worm_geometry
from .key_strength import key_strength
from .belt_chain import belt_v_length, chain_sprocket_pitch_diameter
from .to_feature_compose import (
	gear_pair_to_feature_compose,
	gear_rack_to_feature_compose,
	worm_to_feature_compose,
)

__all__ = [
	"motor_power",
	"lookup_y_motor",
	"reducer_rated_power",
	"load_torque_ballscrew",
	"inertia_cylinder",
	"gear_spur_helical_shift",
	"gear_rack",
	"gear_high_shift_dims",
	"lookup_gear_material",
	"worm_geometry",
	"key_strength",
	"belt_v_length",
	"chain_sprocket_pitch_diameter",
	"gear_pair_to_feature_compose",
	"gear_rack_to_feature_compose",
	"worm_to_feature_compose",
]
