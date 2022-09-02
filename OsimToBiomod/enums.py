from enum import Enum


class MuscleType(Enum):
    HILL = "hill"
    HILL_THELEN = "hillethelen"
    HILL_DE_GROOTE = "hilldegroote"


class MuscleStateType(Enum):
    DEGROOTE = "degroote"
    DEFAULT = "default"
    BUCHANAN = "buchanan"


class JointType(Enum):
    WELD_JOINT = "WeldJoint"
    CUSTOM_JOINT = 'CustomJoint'


class ForceType(Enum):
    MUSCLE = "Muscle"


class Controller(Enum):
    NONE = None


class Constraint(Enum):
    NONE = None


class ContactGeometry(Enum):
    NONE = None


class Component(Enum):
    NONE = None


class Probe(Enum):
    NONE = None