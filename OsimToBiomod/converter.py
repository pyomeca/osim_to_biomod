from lxml import etree
import xml.etree.ElementTree as ET
from numpy.linalg import inv
from OsimToBiomod.model_classes import *


class ReadOsim:
    def __init__(self, osim_path, print_warnings: bool = True):
        self.osim_path = osim_path
        self.osim_model = etree.parse(self.osim_path)
        self.model = ET.parse(self.osim_path)
        self.root = self.model.getroot()[0]
        self.print_warnings = print_warnings
        if self.get_file_version() < 40000:
            raise RuntimeError(
                "Osim file version must be superior or equal than '40000' and you have"
                f" : {self.get_file_version()}."
                "To convert the osim file to the newest version please open and save your file in"
                "Opensim 4.0 or latter."
            )

        self.gravity = "0 0 9.81"
        self.ground_elt, self.default_elt, self.credit, self.publications = None, None, None, None
        self.bodyset_elt, self.jointset_elt, self.forceset_elt, self.markerset_elm = None, None, None, None
        self.controllerset_elt, self.constraintset_elt, self.contact_geometryset_elt = None, None, None
        self.componentset_elt, self.probeset_elt = None, None
        self.length_units, self.force_units = None, None

        for element in self.root:
            if element.tag == "gravity":
                self.gravity = element.text
            elif element.tag == "Ground":
                self.ground_elt = element
            elif element.tag == "defaults":
                self.default_elt = element
            elif element.tag == "BodySet":
                self.bodyset_elt = element
            elif element.tag == "JointSet":
                self.jointset_elt = element
            elif element.tag == "ControllerSet":
                self.controllerset_elt = element
            elif element.tag == "ConstraintSet":
                self.constraintset_elt = element
            elif element.tag == "ForceSet":
                self.forceset_elt = element
            elif element.tag == "MarkerSet":
                self.markerset_elt = element
            elif element.tag == "ContactGeometrySet":
                self.contact_geometryset_elt = element
            elif element.tag == "ComponentSet":
                self.componentset_elt = element
            elif element.tag == "ProbeSet":
                self.probeset_elt = element
            elif element.tag == "credits":
                self.credit = element.text
            elif element.tag == "publications":
                self.publications = element.text
            elif element.tag == "length_units":
                self.length_units = element.text
                if self.length_units != "meters":
                    raise RuntimeError("Lengths units must be in meters.")
            elif element.tag == "force_units":
                self.force_units = element.text
            else:
                raise RuntimeError(
                    f"Element {element.tag} not recognize. Please verify your xml file or send an issue"
                    f" in the github repository."
                )

        self.bodies = []
        self.forces = []
        self.joints = []
        self.markers = []
        self.constraint_set = []
        self.controller_set = []
        self.prob_set = []
        self.component_set = []
        self.geometry_set = []
        self.warnings = []
        self.infos = {}
        self._get_infos()

    @staticmethod
    def _is_element_empty(element):
        if element:
            if not element[0].text:
                return True
            else:
                return False
        else:
            return True

    def get_body_set(self):
        bodies = []
        if self._is_element_empty(self.bodyset_elt):
            return None
        else:
            for element in self.bodyset_elt[0]:
                bodies.append(Body().get_body_attrib(element))
            return bodies

    def get_marker_set(self):
        markers = []
        if self._is_element_empty(self.markerset_elt):
            return None
        else:
            for element in self.markerset_elt[0]:
                markers.append(Marker().get_marker_attrib(element))
            return markers

    def get_force_set(self, ignore_muscle_applied_tag=False):
        forces = []
        wrap = []
        if self._is_element_empty(self.forceset_elt):
            return None
        else:
            for element in self.forceset_elt[0]:
                if "Muscle" in element.tag:
                    forces.append(Muscle().get_muscle_attrib(element, ignore_muscle_applied_tag))
                    if forces[-1].wrap:
                        wrap.append(forces[-1].name)
                elif "Force" in element.tag or "Actuator" in element.tag:
                    self.warnings.append(
                        f"Some {element.tag} were present in the original file force set."
                        " Only muscles are supported so they will be ignored."
                    )
            if len(wrap) != 0:
                self.warnings.append(
                    f"Some wrapping objects were present on the muscles :{wrap}"
                    " in the original file force set.\n"
                    " Only via point are supported in biomod so they will be ignored."
                )

            return forces

    def get_joint_set(self, ignore_fixed_dof_tag=False, ignore_clamped_dof_tag=False):
        joints = []
        if self._is_element_empty(self.forceset_elt):
            return None
        else:
            for element in self.jointset_elt[0]:
                joints.append(Joint().get_joint_attrib(element, ignore_fixed_dof_tag, ignore_clamped_dof_tag))
                if joints[-1].function:
                    self.warnings.append(
                        f"Some functions were present for the {joints[-1].name} joint."
                        " This feature is not implemented in biorbd yet so it will be ignored."
                    )
            joints = self._reorder_joints(joints)
            return joints

    @staticmethod
    def add_markers_to_bodies(bodies, markers):
        for b, body in enumerate(bodies):
            try:
                for marker in markers:
                    if body.socket_frame == marker.parent:
                        bodies[b].markers.append(marker)
            except:
                pass
        return bodies

    @staticmethod
    def _reorder_joints(joints: list):
        ordered_joints = [joints[0]]
        joints.pop(0)
        while len(joints) != 0:
            for o, ord_joint in enumerate(ordered_joints):
                idx = []
                for j, joint in enumerate(joints):
                    if joint.parent == ord_joint.child:
                        ordered_joints = ordered_joints + [joint]
                        idx.append(j)
                    elif ord_joint.parent == joint.child:
                        ordered_joints = [joint] + ordered_joints
                        idx.append(j)
                if len(idx) != 0:
                    joints.pop(idx[0])
                elif len(idx) > 1:
                    raise RuntimeError("Two segment can't have the same parent in a biomod.")
        return ordered_joints

    def get_controller_set(self):
        if self._is_element_empty(self.controllerset_elt):
            self.controller_set = None
        else:
            self.warnings.append(
                "Some controllers were present in the original file."
                " This feature is not implemented in biorbd yet so it will be ignored."
            )

    def _get_constraint_set(self):
        if self._is_element_empty(self.constraintset_elt):
            self.constraintset_elt = None
        else:
            self.warnings.append(
                "Some constraints were present in the original file."
                " This feature is not implemented in biorbd yet so it will be ignored."
            )

    def _get_contact_geometry_set(self):
        if self._is_element_empty(self.contact_geometryset_elt):
            self.contact_geometryset_elt = None
        else:
            self.warnings.append(
                "Some contact geometry were present in the original file."
                " This feature is not implemented in biorbd yet so it will be ignored."
            )

    def _get_component_set(self):
        if self._is_element_empty(self.componentset_elt):
            self.componentset_elt = None
        else:
            self.warnings.append(
                "Some additional components were present in the original file."
                " This feature is not implemented in biorbd yet so it will be ignored."
            )

    def _get_probe_set(self):
        if self._is_element_empty(self.probeset_elt):
            self.probeset_elt = None
        else:
            self.warnings.append(
                "Some probes were present in the original file."
                " This feature is not implemented in biorbd yet so it will be ignored."
            )

    def get_warnings(self):
        self._get_probe_set()
        self._get_component_set()
        self._get_contact_geometry_set()
        self._get_constraint_set()
        return self.warnings

    def get_file_version(self):
        return int(self.model.getroot().attrib["Version"])

    def _get_infos(self):
        if self.publications:
            self.infos["Publication"] = self.publications
        if self.credit:
            self.infos["Credit"] = self.credit
        if self.force_units:
            self.infos["Force units"] = self.force_units
        if self.length_units:
            self.infos["Length units"] = self.length_units


class WriteBiomod:
    def __init__(self, biomod_path):
        self.biomod_path = biomod_path
        self.version = str(4)
        self.biomod_file = open(self.biomod_path, "w")

    def write(self, string):
        self.biomod_file.write(string)

    def write_headers(self, gravity, osim_path=None, print_warnings=True, print_info=True, warnings=None, infos=None):
        self.write("version " + self.version + "\n")
        if osim_path:
            self.write("\n// File extracted from " + osim_path + "\n")
        if print_info:
            for info in infos.keys():
                self.write(f"\n//{info} : {infos[info]}\n")
        if print_warnings:
            if warnings:
                self.write(
                    "\n// Biomod not include all Osim features as the optimisation "
                    "is performed on a third part software.\n"
                    "// The original file contained some of these features, "
                    "corresponding warnings are shown in the end of the file.\n"
                )
            else:
                self.write("\n// There are no warnings in this file.\n")
        self.write("\n")
        # Gravity
        self.write(f"\ngravity\t{gravity}\n")

    def write_marker(self, marker):
        self.write(f"\n\tmarker\t{marker.name}")
        self.write(f"\n\t\tparent\t{marker.parent}")
        self.write(f"\n\t\tposition\t{marker.position}")
        self.write(f"\n\tendmarker\n")

    def write_muscle_group(self, group):
        self.write(f"\n// {group[0]} > {group[1]}\n")
        self.write(f"musclegroup {group[0]}_to_{group[1]}\n")
        self.write(f"\tOriginParent\t{group[0]}\n")
        self.write(f"\tInsertionParent\t{group[1]}\n")
        self.write(f"endmusclegroup\n")

    def write_muscle(self, muscle, type, state_type):
        # print muscle data
        self.write(f"\n\tmuscle\t{muscle.name}") if muscle.name else None
        self.write(f"\n\t\ttype\t{type}") if type else None
        self.write(f"\n\t\tstatetype\t{state_type}") if state_type else None
        if muscle.group:
            self.write(f"\n\t\tmusclegroup\t{muscle.group[0]}_to_{muscle.group[1]}")
        self.write(f"\n\t\tOriginPosition\t{muscle.origin}") if muscle.origin else None
        self.write(f"\n\t\tInsertionPosition\t{muscle.insersion}") if muscle.insersion else None
        self.write(f"\n\t\toptimalLength\t{muscle.optimal_length}") if muscle.optimal_length else None
        self.write(f"\n\t\tmaximalForce\t{muscle.maximal_force}") if muscle.maximal_force else None
        self.write(f"\n\t\ttendonSlackLength\t{muscle.tendon_slack_length}") if muscle.tendon_slack_length else None
        self.write(f"\n\t\tpennationAngle\t{muscle.pennation_angle}") if muscle.pennation_angle else None
        self.write(f"\n\t\tPCSA\t{muscle.pcsa}") if muscle.pcsa else None
        self.write(f"\n\t\tmaxVelocity\t{muscle.maximal_velocity}") if muscle.maximal_velocity else None
        self.write(f"\n\tendmuscle\n")

    def write_via_point(self, via_point):
        self.write(f"\n\t\tviapoint\t{via_point.name}")
        self.write(f"\n\t\t\tparent\t{via_point.body}")
        self.write(f"\n\t\t\tmuscle\t{via_point.muscle}")
        self.write(f"\n\t\t\tmusclegroup\t{via_point.muscle_group}")
        self.write(f"\n\t\t\tposition\t{via_point.position}")
        self.write("\n\t\tendviapoint")
        self.write("\n")

    def _write_generic_segment(self, name, parent, rt_in_matrix, frame_offset=None):
        self.write(f"\t// Segment\n")
        self.write(f"\tsegment {name}\n")
        self.write(f"\t\tparent {parent} \n")
        self.write(f"\t\tRTinMatrix\t{rt_in_matrix}\n")
        if rt_in_matrix == 0:
            frame_offset = frame_offset if frame_offset else [[0, 0, 0], [0, 0, 0]]
            for i in range(len(frame_offset)):
                if isinstance(frame_offset[i], (list, np.ndarray)):
                    if isinstance(frame_offset[i], np.ndarray):
                        frame_offset[i] = frame_offset[i].tolist()
                    frame_offset[i] = str(frame_offset[i])[1:-1].replace(",", "\t")
            self.write(f"\t\tRT\t{frame_offset[1]}\txyz\t{frame_offset[0]}\n")
        else:
            frame_offset = frame_offset if frame_offset else OrthoMatrix([0, 0, 0])
            [[r14], [r24], [r34]] = frame_offset.get_translation().tolist()
            [r41, r42, r43, r44] = [0, 0, 0, 1]

            r11, r12, r13 = frame_offset.get_rotation_matrix()[0, :]
            r21, r22, r23 = frame_offset.get_rotation_matrix()[1, :]
            r31, r32, r33 = frame_offset.get_rotation_matrix()[2, :]
            self.write("\t\tRT\n")
            self.write(
                f"\t\t\t{r11}\t\t{r12}\t\t{r13}\t\t{r14}\n"
                f"\t\t\t{r21}\t\t{r22}\t\t{r23}\t\t{r24}\n"
                f"\t\t\t{r31}\t\t{r32}\t\t{r33}\t\t{r34}\n"
                f"\t\t\t{r41}\t\t{r42}\t\t{r43}\t\t{r44}\n"
            )

    def _write_true_segement(self, name, parent_name, frame_offset, com, mass, inertia, mesh_file=None,
                             mesh_scale=None, mesh_color=None, rt=0):
        """
        Segment where is applied inertia.
        """
        [i11, i22, i33, i12, i13, i23] = inertia

        self._write_generic_segment(name, parent_name, frame_offset=frame_offset, rt_in_matrix=rt)
        self.write(f"\t\tmass\t{mass}\n")
        self.write(
            "\t\tinertia\n" f"\t\t\t{i11}\t{i12}\t{i13}\n" f"\t\t\t{i12}\t{i22}\t{i23}\n" f"\t\t\t{i13}\t{i23}\t{i33}\n"
        )
        self.write(f"\t\tcom\t{com}\n")
        if mesh_file:
            self.write(f"\t\tmeshfile\t{mesh_file}\n")
            if mesh_color:
                self.write(f"\t\tmeshcolor\t{mesh_color}\n")
            if mesh_scale:
                self.write(f"\t\tmeshscale\t{mesh_scale}\n")
        self.write(f"\tendsegment\n")

    def _write_virtual_segment(self, name, parent_name, frame_offset, q_range=None, rt=0, trans_dof="", rot_dof="",
                               mesh_file=None,
                               mesh_color=None,
                               mesh_scale=None
                               ):
        """
        This function aims to add virtual segment to convert osim dof in biomod dof.
        """
        self._write_generic_segment(name, parent_name, frame_offset=frame_offset, rt_in_matrix=rt)
        if trans_dof[:2] == "//":
            self.write(f"\t\t//translations {trans_dof[2:]}\n") if trans_dof != "" else True
        else:
            self.write(f"\t\ttranslations {trans_dof}\n") if trans_dof != "" else True
        if rot_dof[:2] == "//":
            self.write(f"\t\t//rotations {rot_dof[2:]}\n") if rot_dof != "" else True
        else:
            self.write(f"\t\trotations {rot_dof}\n") if rot_dof != "" else True

        if q_range:
            if not isinstance(q_range, list):
                q_range = [q_range]
            if q_range.count(None) != 3:
                count = 0
                for q in q_range:
                    if q_range and q[:2] == "//":
                        count += 1

                for q, qrange in enumerate(q_range):
                    if rot_dof[:2] == "//":
                        range_to_write = f"\t\t\t\t//{qrange[2:]}\n"
                    else:
                        range_to_write = f"\t\t\t\t{qrange}\n"
                    if q == 0:
                        if count == len(q_range):
                            self.write(f"\t\t// ranges\n")
                        else:
                            self.write(f"\t\tranges\n")
                    self.write(range_to_write)

        if mesh_file:
            self.write(f"\t\tmeshfile\t{mesh_file}\n")
            if mesh_color:
                self.write(f"\t\tmeshcolor\t{mesh_color}\n")
            if mesh_scale:
                self.write(f"\t\tmeshscale\t{mesh_scale}\n")
        self.write(f"\tendsegment\n\n")

    @staticmethod
    def _get_transformation_parameters(spatial_transform):
        translations = []
        rotations = []
        q_ranges_trans = []
        q_ranges_rot = []
        is_dof_trans = []
        default_value_trans = []
        default_value_rot = []
        is_dof_rot = []
        for transform in spatial_transform:
            q_range = None
            axis = [float(i.replace(",", ".")) for i in transform.axis.split(" ")]
            if transform.coordinate:
                if transform.coordinate.range:
                    q_range = transform.coordinate.range
                    if not transform.coordinate.clamped:
                        q_range = "// " + q_range
                else:
                    q_range = None
                value = transform.coordinate.default_value
                default_value = value if value else 0
                is_dof_tmp = None if transform.coordinate.locked else transform.coordinate.name
            else:
                is_dof_tmp = None
                default_value = 0
            if transform.type == "translation":
                translations.append(axis)
                q_ranges_trans.append(q_range)
                is_dof_trans.append(is_dof_tmp)
                default_value_trans.append(default_value)
            elif transform.type == "rotation":
                rotations.append(axis)
                q_ranges_rot.append(q_range)
                is_dof_rot.append(is_dof_tmp)
                default_value_rot.append(default_value)
            else:
                raise RuntimeError("Transform must be 'rotation' or 'translation'")
        return (
            translations,
            q_ranges_trans,
            is_dof_trans,
            default_value_trans,
            rotations,
            q_ranges_rot,
            is_dof_rot,
            default_value_rot,
        )

    def _write_ortho_segment(
        self, axis, axis_offset, name, parent, rt_in_matrix, frame_offset, q_range=None, trans_dof="", rot_dof=""
    ):

        x = axis[0]
        y = axis[1]
        z = axis[2]
        frame_offset.set_rotation_matrix(np.append(x, np.append(y, z)).reshape(3, 3).T)
        self._write_virtual_segment(
            name,
            parent,
            frame_offset=frame_offset,
            q_range=q_range,
            rt=rt_in_matrix,
            trans_dof=trans_dof,
            rot_dof=rot_dof,
        )
        return axis_offset.dot(frame_offset.get_rotation_matrix())

    def _write_non_ortho_rot_segment(
        self,
        axis,
        axis_offset,
        name,
        parent,
        rt_in_matrix,
        frame_offset,
        spatial_transform,
        q_ranges=None,
        default_values=None,
    ):

        default_values = [0, 0, 0] if not default_values else default_values
        axis_basis = []
        list_rot_dof = ["x", "y", "z"]
        count_dof_rot = 0
        q_range = None
        for i, axe in enumerate(axis):
            if len(axis_basis) == 0:
                axis_basis.append(ortho_norm_basis(axe, i))
                initial_rotation = compute_matrix_rotation([float(default_values[i]), 0, 0])
            elif len(axis_basis) == 1:
                axis_basis.append(inv(axis_basis[i - 1]).dot(ortho_norm_basis(axe, i)))
                initial_rotation = compute_matrix_rotation([0, float(default_values[i]), 0])
            else:
                axis_basis.append(inv(axis_basis[i - 1]).dot(inv(axis_basis[i - 2])).dot(ortho_norm_basis(axe, i)))
                initial_rotation = compute_matrix_rotation([0, 0, float(default_values[i])])

            try:
                coordinate = spatial_transform[i].coordinate
                rot_dof = list_rot_dof[count_dof_rot] if not coordinate.locked else "//" + list_rot_dof[count_dof_rot]
                body_dof = name + "_" + spatial_transform[i].coordinate.name
                q_range = q_ranges[i]
            except:
                body_dof = name + f"_rotation_{i}"
                rot_dof = ""

            frame_offset.set_rotation_matrix(axis_basis[i].dot(initial_rotation))
            count_dof_rot += 1
            self._write_virtual_segment(
                body_dof, parent, frame_offset=frame_offset, q_range=q_range, rt=rt_in_matrix, rot_dof=rot_dof
            )
            axis_offset = axis_offset.dot(frame_offset.get_rotation_matrix())
            parent = body_dof
        return axis_offset, parent

    def write_dof(self, body, dof, mesh_dir=None):
        rotomatrix = OrthoMatrix([0, 0, 0])
        self.write(f"\n// Information about {body.name} segment\n")
        parent = dof.parent_body.split("/")[-1]
        axis_offset = np.identity(3)
        # Parent offset
        body_name = body.name + "_parent_offset"
        offset = [dof.parent_offset_trans, dof.parent_offset_rot]
        self._write_virtual_segment(body_name, parent, frame_offset=offset, rt=0)
        parent = body_name
        # Coordinates
        (
            translations,
            q_ranges_trans,
            is_dof_trans,
            default_value_trans,
            rotations,
            q_ranges_rot,
            is_dof_rot,
            default_value_rot,
        ) = self._get_transformation_parameters(dof.spatial_transform)

        is_dof_trans, is_dof_rot = np.array(is_dof_trans), np.array(is_dof_rot)
        dof_axis = np.array(["x", "y", "z"])
        if len(translations) != 0 or len(rotations) != 0:
            self.write("\t// Segments to define transformation axis.\n")
        # Translations
        if len(translations) != 0:
            body_name = body.name + "_translation"
            if is_ortho_basis(translations):
                trans_axis = ""
                for idx in np.where(is_dof_trans != None)[0]:
                    trans_axis += dof_axis[idx]
                axis_offset = self._write_ortho_segment(
                    axis=translations,
                    axis_offset=axis_offset,
                    name=body_name,
                    parent=parent,
                    rt_in_matrix=1,
                    frame_offset=rotomatrix,
                    q_range=q_ranges_trans,
                    trans_dof=trans_axis,
                )
                parent = body_name
            else:
                raise RuntimeError("Non orthogonal translation vector not implemented yet.")

            # Rotations
        if len(rotations) != 0:
            if is_ortho_basis(rotations):
                rot_axis = ""
                for idx in np.where(is_dof_rot != None)[0]:
                    rot_axis += dof_axis[idx]
                body_name = body.name + "_rotation_transform"
                self.write("// Rotation transform was initially an orthogonal basis\n")
                axis_offset = self._write_ortho_segment(
                    axis=rotations,
                    axis_offset=axis_offset,
                    name=body_name,
                    parent=parent,
                    rt_in_matrix=1,
                    frame_offset=rotomatrix,
                    q_range=q_ranges_rot,
                    rot_dof=rot_axis,
                )
                parent = body_name
            else:
                body_name = body.name
                axis_offset, parent = self._write_non_ortho_rot_segment(
                    rotations,
                    axis_offset,
                    body_name,
                    parent,
                    frame_offset=rotomatrix,
                    rt_in_matrix=1,
                    spatial_transform=dof.spatial_transform,
                    q_ranges=q_ranges_rot,
                    default_values=default_value_rot,
                )

        # segment to cancel axis effects
        self.write("\n    // Segment to cancel transformation bases effect.\n")
        rotomatrix.set_rotation_matrix(inv(axis_offset))
        body_name = body.name + "_reset_axis"
        self._write_virtual_segment(
            body_name,
            parent,
            frame_offset=rotomatrix,
            rt=1,
        )
        parent = body_name

        # True segment
        frame_offset = [dof.child_offset_trans, dof.child_offset_rot]
        mesh_dir = mesh_dir if mesh_dir else "Geometry"
        body.mesh = body.mesh if len(body.mesh) != 0 else [None]
        body.mesh_color = body.mesh_color if len(body.mesh_color) != 0 else [None]
        body.mesh_scale_factor = body.mesh_scale_factor if len(body.mesh_scale_factor) != 0 else [None]
        for i, virt_body in enumerate(body.virtual_body):
            if i > 0:
                body_name = virt_body
                self._write_virtual_segment(
                    body_name,
                    parent,
                    frame_offset=frame_offset,
                    mesh_file=f"{mesh_dir}/{body.mesh[i]}",
                    mesh_color=body.mesh_color[i],
                    mesh_scale=body.mesh_scale_factor[i],
                    rt=0)
                frame_offset = None
                parent = body_name
        self.write("\n    //True segment where are applied inertial values.\n")
        self._write_true_segement(
            body.name,
            parent,
            frame_offset=frame_offset,
            com=body.mass_center,
            mass=body.mass,
            inertia=body.inertia.split(" "),
            mesh_file=f"{mesh_dir}/{body.mesh[0]}",
            mesh_color=body.mesh_color[0],
            mesh_scale=body.mesh_scale_factor[0],
            rt=0,
        )


class Converter:
    def __init__(
        self,
        biomod_path,
        osim_path,
        muscle_type=None,
        state_type=None,
        print_warnings=True,
        print_general_informations=True,
        ignore_clamped_dof_tag=False,
        ignore_fixed_dof_tag=False,
        mesh_dir=None,
        ignore_muscle_applied_tag=False,
    ):
        self.biomod_path = biomod_path
        self.osim_model = ReadOsim(osim_path)
        self.osim_path = osim_path
        self.writer = WriteBiomod(biomod_path)
        self.print_warnings = print_warnings
        self.print_general_informations = print_general_informations
        self.forces = self.osim_model.get_force_set(ignore_muscle_applied_tag)
        self.joints = self.osim_model.get_joint_set(ignore_fixed_dof_tag, ignore_clamped_dof_tag)
        self.bodies = self.osim_model.get_body_set()
        self.markers = self.osim_model.get_marker_set()
        self.infos, self.warnings = self.osim_model.infos, self.osim_model.get_warnings()
        self.bodies = self.osim_model.add_markers_to_bodies(self.bodies, self.markers)
        self.muscle_type = muscle_type if muscle_type else "hilldegroote"
        self.state_type = state_type
        self.mesh_dir = mesh_dir

        if isinstance(self.muscle_type, MuscleType):
            self.muscle_type = self.muscle_type.value
        if isinstance(self.state_type, MuscleStateType):
            self.state_type = self.state_type.value

        if isinstance(muscle_type, str):
            if self.muscle_type not in [e.value for e in MuscleType]:
                raise RuntimeError(f"Muscle of type {self.muscle_type} is not a biorbd muscle.")

        if isinstance(muscle_type, str):
            if self.state_type not in [e.value for e in MuscleStateType]:
                raise RuntimeError(f"Muscle state type {self.state_type} is not a biorbd muscle state type.")
        if self.state_type == "default":
            self.state_type = None

    def convert_file(self):
        # headers
        self.writer.write_headers(
            self.osim_model.gravity,
            self.osim_path,
            self.print_warnings,
            self.print_general_informations,
            self.warnings,
            self.infos,
        )

        # segment
        self.writer.write("\n// SEGMENT DEFINITION\n")
        for dof in self.joints:
            for body in self.bodies:
                if body.socket_frame == dof.child_body:
                    self.writer.write_dof(body, dof, self.mesh_dir)
                    self.writer.write(f"\n\t// Markers\n")
                    for marker in body.markers:
                        self.writer.write_marker(marker)

        muscle_groups = []
        for muscle in self.forces:
            group = muscle.group
            if group not in muscle_groups:
                muscle_groups.append(group)

        self.writer.write("\n// MUSCLE DEFINIION\n")
        muscles = self.forces
        while len(muscles) != 0:
            for muscle_group in muscle_groups:
                idx = []
                self.writer.write_muscle_group(muscle_group)
                for m, muscle in enumerate(muscles):
                    if muscle.group == muscle_group:
                        if not muscle.applied:
                            self.writer.write("\n/*")
                        self.writer.write_muscle(muscle, self.muscle_type, self.state_type)
                        for via_point in muscle.via_point:
                            self.writer.write_via_point(via_point)
                        if not muscle.applied:
                            self.writer.write("*/\n")
                        idx.append(m)
                count = 0
                for i in idx:
                    muscles.pop(i - count)
                    count += 1
            muscle_groups.pop(0)

        if self.print_warnings:
            self.writer.write("\n/*-------------- WARNINGS---------------\n")
            for warning in self.warnings:
                self.writer.write("\n" + warning)
            self.writer.write("*/\n")
        self.writer.biomod_file.close()
        print(f"\nYour file {self.osim_path} has been converted into {self.biomod_path}.")
