from osim_to_biomod import Converter, MuscleType, MuscleStateType

if __name__ == "__main__":
    model_path = "Models/"
    converter = Converter(
        model_path + "wu_bras_gauche.bioMod",
        model_path + "wu_bras_gauche.osim",
        ignore_muscle_applied_tag=False,
        ignore_fixed_dof_tag=False,
        ignore_clamped_dof_tag=False,
        mesh_dir="Geometry",
        muscle_type=MuscleType.HILL_DE_GROOTE,
        state_type=MuscleStateType.DEGROOTE,
        print_warnings=True,
        print_general_informations=True,
        vtp_polygons_to_triangles=True,
    )
    converter.convert_file()
