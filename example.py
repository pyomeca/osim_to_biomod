from OsimToBiomod.converter import Converter
import os
from OsimToBiomod.enums import *

if __name__ == "__main__":
    model_path = "Models/"
    converter = Converter(
        model_path + "Wu_Shoulder_Model_via_points.bioMod",
        model_path + "Wu_Shoulder_Model_via_points.osim",
        ignore_muscle_applied_tag=False,
        ignore_fixed_dof_tag=False,
        ignore_clamped_dof_tag=False,
        mesh_dir="Geometry",
        muscle_type=MuscleType.HILL,
        state_type=MuscleStateType.DEGROOTE,
        print_warnings=True,
        print_general_informations=True,
    )
    converter.convert_file()
