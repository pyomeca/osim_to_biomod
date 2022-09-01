from lxml import etree
from OsimToBiomod.utils import *
import xml.etree.ElementTree as ET
from numpy.linalg import inv
import os
from model_classes import *

# TODO :
#   - Add class for joints, bodies, muscles, markers


class ReadOsim:
    def __init__(self, osim_path, print_warnings: bool = True):
        self.osim_path = osim_path
        self.osim_model = etree.parse(self.osim_path)
        self.model = ET.parse(self.osim_path)
        self.root = self.model.getroot()[0]
        self.print_warnings = print_warnings
        if self.get_file_version() < 40000:
            raise RuntimeError("Osim file version must be superior or equal than '40000' and you have"
                               f" : {self.get_file_version()}."
                               "To convert the osim file to the newest version please open and save your file in"
                               "Opensim 4.0 or latter.")

        self.gravity = '0 0 9.81'
        self.ground_elt, self.default_elt, self.credit, self.publications = None, None, None, None
        self.bodyset_elt, self.jointset_elt, self.forceset_elt, self.markerset_elm = None, None, None, None
        self.controllerset_elt, self.constraintset_elt,self.contact_geometryset_elt = None, None, None
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
                raise RuntimeError(f"Element {element.tag} not recognize. Please verify your xml file or send an issue"
                                   f" in the github repository.")

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

    def get_force_set(self):
        forces = []
        wrap = []
        if self._is_element_empty(self.forceset_elt):
            return None
        else:
            for element in self.forceset_elt[0]:
                if "Muscle" in element.tag:
                    forces.append(Muscle().get_muscle_attrib(element))
                    if forces[-1].wrap:
                        wrap.append(forces[-1].name)
                elif "Force" in element.tag:
                    self.warnings.append("Some forces were present in the original file force set."
                                         " Only muscles are supported so their will be ignored.")
                elif "Actuator" in element.tag:
                    self.warnings.append("Some Actuators were present in the original file force set."
                                         " Only muscles are supported so their will be ignored.")
                else:
                    raise RuntimeError(f"Element {element.tag} are not implemented yet. Please raise an issue.")
            if len(wrap) != 0:
                self.warnings.append(f"Some wrapping objects were present on the muscles :{wrap}"
                                     " in the original file force set.\n"
                                     " Only via point are supported in biomod so their will be ignored.")

            return forces

    def get_join_set(self):
        joints = []
        if self._is_element_empty(self.forceset_elt):
            return None
        else:
            for element in self.jointset_elt[0]:
                joints.append(Joint().get_joint_attrib(element))
            joints = [joints[-1]] + joints[:-1]
            joints = self._reorder_joints(joints)
            return joints

    @staticmethod
    def add_markers_to_bodies(bodies, markers):
        for b, body in enumerate(bodies):
            for marker in markers:
                if body.socket_frame == marker.parent:
                    bodies[b].markers.append(marker)
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
            self.warnings.append("Some controllers were present in the original file."
                                 " This feature is not implemented in biorbd yet so it will be ignored.")

    def _get_constraint_set(self):
        if self._is_element_empty(self.constraintset_elt):
            self.constraintset_elt = None
        else:
            self.warnings.append("Some constraints were present in the original file."
                                 " This feature is not implemented in biorbd yet so it will be ignored.")

    def _get_contact_geometry_set(self):
        if self._is_element_empty(self.contact_geometryset_elt):
            self.contact_geometryset_elt = None
        else:
            self.warnings.append("Some contact geometry were present in the original file."
                                 " This feature is not implemented in biorbd yet so it will be ignored.")

    def _get_component_set(self):
        if self._is_element_empty(self.componentset_elt):
            self.componentset_elt = None
        else:
            self.warnings.append("Some additional components were present in the original file."
                                 " This feature is not implemented in biorbd yet so it will be ignored.")

    def _get_probe_set(self):
        if self._is_element_empty(self.probeset_elt):
            self.probeset_elt = None
        else:
            self.warnings.append("Some probes were present in the original file."
                                 " This feature is not implemented in biorbd yet so it will be ignored.")

    def get_warnings(self):
        self._get_probe_set()
        self._get_component_set()
        self._get_contact_geometry_set()
        self._get_constraint_set()
        return self.warnings

    def get_file_version(self):
        return int(self.model.getroot().attrib['Version'])

    def get_infos(self):
        if self.publications:
            self.infos["Publication"] = self.publications
        if self.credit:
            self.infos["Credit"] = self.credit
        if self.force_units:
            self.infos["Force units"] = self.force_units
        if self.length_units:
            self.infos["Length units"] = self.length_units
        return self.infos


class WriteBiomod:
    def __init__(self, biomod_path):
        self.biomod_path = biomod_path
        self.version = str(4)
        self.biomod_file = open(self.biomod_path, 'w')

    def write(self, string):
        self.biomod_file.write(string)

    def write_headers(self, gravity, osim_path=None, print_warnings=True, print_info=True, warnings=None, infos=None):
        self.write('version ' + self.version + '\n')
        if osim_path:
            self.write('\n// File extracted from ' + osim_path)
        if print_info:
            for info in infos.keys():
                self.write(f'\n//{info} : {infos[info]}')
        if print_warnings:
            if warnings:
                self.write("\n// Biomod not include all Osim features as the optimisation "
                           "is performed on a third part software.\n"
                           "//The original file contained some of these features, "
                           "corresponding warnings are showed in the end of the file.")
            else:
                self.write("// There are no warnings in this file.")
        self.write('\n')
        # Gravity
        self.write(f"\ngravity\t{gravity}\n")

    def write_marker(self, marker):
        self.write(f'\n\tmarker\t{marker.name}')
        self.write(f'\n\t\tparent\t{marker.parent}')
        self.write(f'\n\t\tposition\t{marker.position}')
        self.write(f'\n\tendmarker\n')

    def write_muscle_group(self, group):
        self.write(f'\n// {group[0]} > {group[1]}\n')
        self.write(f'musclegroup {group[0]}_to_{group[1]}\n')
        self.write(f'\tOriginParent\t{group[0]}\n')
        self.write(f'\tInsertionParent\t{group[1]}\n')
        self.write(f'endmusclegroup\n')

    def write_muscle(self, muscle):
        # print muscle data
        self.write(f'\n\tmuscle\t{muscle.name}') if muscle.name else None
        self.write(f'\n\t\ttype\t{muscle.type}') if muscle.type else None
        self.write(f'\n\t\tstatetype\t{muscle.state_type}') if muscle.state_type else None
        if muscle.group:
            self.write(f'\n\t\tmusclegroup\t{muscle.group[0]}_to_{muscle.group[1]}')
        self.write(f'\n\t\tOriginPosition\t{muscle.origin}') if muscle.origin else None
        self.write(f'\n\t\tInsertionPosition\t{muscle.insersion}') if muscle.insersion else None
        self.write(f'\n\t\toptimalLength\t{muscle.optimal_length}') if muscle.optimal_length else None
        self.write(f'\n\t\tmaximalForce\t{muscle.maximal_force}') if muscle.maximal_force else None
        self.write(f'\n\t\ttendonSlackLength\t{muscle.tendon_slack_length}') if muscle.tendon_slack_length else None
        self.write(f'\n\t\tpennationAngle\t{muscle.pennation_angle}') if muscle.pennation_angle else None
        self.write(f'\n\t\tPCSA\t{muscle.pcsa}') if muscle.pcsa else None
        self.write(f'\n\t\tmaxVelocity\t{muscle.maximal_velocity}') if muscle.maximal_velocity else None
        self.write(f'\n\tendmuscle\n')

    def write_via_point(self, via_point):
        self.write(f'\n\tviapoint\t{via_point.name}')
        self.write(f'\n\t\tparent\t{via_point.body}')
        self.write(f'\n\t\tmuscle\t{via_point.muscle}')
        self.write(f'\n\t\tmusclegroup\t{via_point.muscle_group}')
        self.write(f'\n\t\tposition\t{via_point.position}')
        self.write('\n\tendviapoint')
        self.write('\n')

    def _write_generic_segment(self, segment, parent, rt_in_matrix, rotation, translation):
        self.write(f'\t// Segment\n')
        self.write(f'\tsegment {segment}\n')
        self.write(f'\t\tparent {parent} \n')
        self.write(f'\t\tRTinMatrix\t{rt_in_matrix}\n')
        if rt_in_matrix == 0:
            self.write(f'\t\tRT\t{rotation}\txyz\t{translation}\n')
        else:
            [[r14], [r24], [r34]] = [float(i) for i in translation.split(" ")]
            [r41, r42, r43, r44] = [0, 0, 0, 1]
            for i in range(3):
                for j in range(3):
                    globals()['r' + str(i + 1) + str(j + 1)] = round(frame_offset.get_rotation_matrix()[i][j], 9)
            self.write('\t\tRT\n')
            self.write(
                f'\t\t\t{r11}\t{r12}\t{r13}\t{r14}\n'
                f'\t\t\t{r21}\t{r22}\t{r23}\t{r24}\n'
                f'\t\t\t{r31}\t{r32}\t{r33}\t{r34}\n'
                f'\t\t\t{r41}\t{r42}\t{r43}\t{r44}\n')

    def _write_true_segement(self):
        """
        Segment where is applied inertia.
        """

    def _write_virtual_segment(self, body, dof, parent_name, rt=0):
        """
        This function aims to add virtual segment to convert osim dof in biomod dof.
        """
        self._write_generic_segment(body.name, parent_name, rt, dof.parent_offset_rot, dof.parent_offset_trans)


    def _get_transformations_parameters(self, spatial_transform):
        translations = []
        rotations = []
        q_ranges_trans = []
        q_ranges_rot = []
        is_dof_trans = []
        default_value_trans = []
        default_value_rot = []
        default_value = None
        q_range = None
        is_dof_rot = []
        for transform in spatial_transform:
            axis = [float(i.replace(',', '.')) for i in transform.axis.split(" ")]
            if transform.coordinate:
                if transform.coordinate.range:
                    q_range = transform.coordinate.range
                    if transform.coordinate.clamped:
                        q_range = '//' + q_range
                else:
                    q_range = None
                value = transform.coordinate.default_value
                default_value = value if value else 0
                is_dof_tmp = transform.coordinate.name
            else:
                is_dof_tmp = None
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
        return translations, q_ranges_trans, is_dof_trans, default_value_trans,\
               rotations, q_ranges_rot, is_dof_rot, default_value_rot

    def _write_ortho_trans_segment(self, translations, body, dof, rotomatrix, is_dof, axis_offset):
        x = translations[0]
        y = translations[1]
        z = translations[2]
        rotomatrix.set_rotation_matrix(np.append(x, np.append(y, z)).reshape(3, 3).T)
        self._write_virtual_segment(body, dof, rotomatrix, is_dof, rt=1)
        return axis_offset.dot(rotomatrix.get_rotation_matrix())

    def _write_ortho_rot_segment(self, rotations, body, dof, rotomatrix, is_dof, axis_offset, parent):
        x = rotations[0]
        y = rotations[1]
        z = rotations[2]
        rotomatrix.set_rotation_matrix(np.append(x, np.append(y, z)).reshape(3, 3).T)
        body_name = body.name + '_rotation_transform'
        rot_dof = "xyz"
        self.write("// Rotation transform was initially an orthogonal basis\n")
        self._write_virtual_segment(body, body_name, parent, rotomatrix, rt_in_matrix=1,
                                    _dof_total_rot=rot_dof, true_segment=False, _is_dof='True')
        return axis_offset.dot(rotomatrix.get_rotation_matrix())

    def _write_non_ortho_rot_segment(self, rotations, rotomatrix, default_value, is_dof, q_ranges, dof, body, axis_offset):
        axis_basis = []
        list_rot_dof = ['x', 'y', 'z']
        count_dof_rot = 0
        q_range = None
        dof_names = []
        parent = None
        for coordinate in dof:
            dof_names.append(coordinate.name)
        for i, rotation in enumerate(rotations):
            initial_rotation = compute_matrix_rotation([0, default_value[i], 0])
            if is_dof[i]:
                q_range = q_ranges[i]
            if len(axis_basis) == 0:
                axis_basis.append(ortho_norm_basis(rotation, i))
            elif len(axis_basis) == 1:
                axis_basis.append(inv(axis_basis[i - 1]).dot(ortho_norm_basis(rotation, i)))
            else:
                axis_basis.append(
                    inv(axis_basis[i - 1]).dot(inv(axis_basis[i - 2])).dot(ortho_norm_basis(rotation, i)))

            if is_dof[i] in dof_names:
                dof_rot = list_rot_dof[count_dof_rot]
                activate_dof = True
                body_dof = body.name + '_' + is_dof[i]
            else:
                body_dof = body.name + '_' + rotation
                activate_dof = False
                dof_rot = ''

            rotomatrix.set_rotation_matrix(axis_basis[i].dot(initial_rotation))
            count_dof_rot += 1
            self._write_virtual_segment(body, body_dof, parent, rotomatrix, rt_in_matrix=1,
                                  _dof_total_rot=dof_rot, true_segment=False, _is_dof=activate_dof,
                                  _range_q=q_range
                                  )
            axis_offset = axis_offset.dot(rotomatrix.get_rotation_matrix())
            parent = body_dof
        return axis_offset, parent

    def write_dof(self, body, dof):
        rotomatrix = OrthoMatrix([0, 0, 0])
        self.write(f'\n// Information about {body.name} segment\n')
        parent = dof.parent_body.split("/")[-1]
        axis_offset = np.identity(3)

        # Parent offset
        self._write_virtual_segment(body, dof, body.name + "_parent_offset", rt=0)
        parent = body.name + "_parent_offset"

        # Coordinates
        self.write("\t// Segments to define transformation axis.\n")
        body_trans = body + '_translation'
        translations, q_ranges_trans, is_dof_trans, default_value_trans, \
        rotations, q_ranges_rot, is_dof_rot,\
        default_value_rot = self._get_transformations_parameters(dof.spatial_transform)

        # Translations
        if is_ortho_basis(translations):
            axis_offset = self._write_ortho_trans_segment(translations, body, dof, rotomatrix, is_dof_trans, axis_offset)
            parent = body_trans
        else:
            raise RuntimeError("Non orthogonal translation vector not implemented yet.")

        # Rotations
        if is_ortho_basis(rotations):
            axis_offset = self._write_ortho_rot_segment(rotations, body, dof, rotomatrix, is_dof_trans, axis_offset)
            parent = body.name + '_rotation_transform'
        else:
            axis_offset = self._write_non_ortho_rot_segment(rotations,
                                                            rotomatrix,
                                                            default_value_rot,
                                                            is_dof_rot,
                                                            q_ranges_rot,
                                                            dof,
                                                            body,
                                                            axis_offset)

        # segment to cancel axis effects
        self.write("\n    // Segment to cancel transformation bases effect.\n")
        rotomatrix.set_rotation_matrix(inv(axis_offset))
        self._write_virtual_segment(body, body + '_reset_axis', parent, rotomatrix, rt_in_matrix=1, true_segment=False)
        parent = body.name + '_reset_axis'

        # True segment
        self.write("\n    //True segment where are applied inertial values.\n")
        self._write_true_segement(body, body, parent, physical_offset[1], rt_in_matrix=0, true_segment=True)
        parent = body


class Converter:
    def __init__(self, biomod_path, osim_path, print_warnings=True, print_general_informations=True):
        self.biomod_path = biomod_path
        self.osim_model = ReadOsim(osim_path)
        self.osim_path = osim_path
        self.writer = WriteBiomod(biomod_path)
        self.print_warnings = print_warnings
        self.print_general_informations = print_general_informations
        self.infos, self.warnings = self.osim_model.infos, self.osim_model.get_warnings()
        self.forces = self.osim_model.get_force_set()
        self.joints = self.osim_model.get_join_set()
        self.bodies = self.osim_model.get_body_set()
        self.markers = self.osim_model.get_marker_set()
        self.bodies = self.osim_model.add_markers_to_bodies(self.bodies, self.markers)

    def convert_file(self):
        # headers
        self.writer.write_headers(self.osim_model.gravity,
                                  self.osim_path,
                                  self.print_warnings,
                                  self.print_general_informations,
                                  self.warnings,
                                  self.infos,
        )

        # segment
        # self.writer.write('\n// SEGMENT DEFINITION\n')
        # for dof in self.joints:
        #     for body in self.bodies:
        #         if body.socket_frame == dof.child_body:
        #             self.writer.write_dof(body, dof)
        #             self.writer.write(f"\n\t// Markers\n")
        #             for marker in body.markers:
        #                 self.writer.write_marker(marker)
        #
        muscle_groups = []
        for muscle in self.forces:
            group = muscle.group
            if group not in muscle_groups:
                muscle_groups.append(group)

        self.writer.write('\n// MUSCLE DEFINIION\n')
        muscles = self.forces
        while len(muscles) != 0:
            for muscle_group in muscle_groups:
                idx = []
                self.writer.write_muscle_group(muscle_group)
                for m, muscle in enumerate(muscles):
                    if muscle.group == muscle_group:
                        self.writer.write_muscle(muscle)
                        for via_point in muscle.via_point:
                            self.writer.write_via_point(via_point)
                        idx.append(m)
                    for i in idx:
                        muscles.pop(i)

        if self.print_warnings:
            self.writer.write("\n*/\n")
            for warning in self.warnings:
                self.writer.write(warning)
            self.writer.write("\n/*\n")
        self.writer.biomod_file.close()
        print(f"\nYour file {self.osim_path} has been converted into {self.biomod_path}.")


if __name__ == '__main__':
    model_path = os.path.dirname(os.getcwd()) + "/Models/"
    model = ReadOsim(model_path + "Wu_Shoulder_Model_via_points.osim")
    # model.BodySet().get_markers('thorax')

    converter = Converter(model_path + "wu_converted_test.bioMod", model_path + "Wu_Shoulder_Model_via_points.osim")
    converter.convert_file()
