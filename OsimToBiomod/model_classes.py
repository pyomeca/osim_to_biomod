from utils import *


class Body:
    def __init__(self):
        self.name = None
        self.mass = None
        self.inertia = None
        self.mass_center = None
        self.mesh = None
        self.wrap = None
        self.socket_frame = None
        self.markers = []

    def get_body_attrib(self, element):
        self.name = (element.attrib["name"]).split("/")[-1]
        self.mass = find(element, "mass")
        self.inertia = find(element, "inertia")
        self.mass_center = find(element, "mass_center")
        geometry = element.find("FrameGeometry")
        if geometry:
            self.socket_frame = geometry.find("socket_frame").text.split("/")[-1]
            if self.socket_frame == "..":
                self.socket_frame = self.name

        if element.find("WrapObjectSet"):
            self.wrap = True if len(element.find("WrapObjectSet").text) != 0 else False

        if element.find("attached_geometry"):
            mesh_list = element.find("attached_geometry").findall("Mesh")
            if mesh_list:
                if len(mesh_list) > 1:
                    self.mesh = "// Several mesh are not allowed in biomod for one segment."
                elif len(mesh_list) == 0:
                    self.mesh = element.find("attached_geometry").find("Mesh").find("mesh_file").text
        return self


class Joint:
    def __init__(self):
        self.parent = None
        self.child = None
        self.type = None
        self.name = None
        self.coordinates = []
        self.parent_offset_trans = []
        self.parent_offset_rot = []
        self.child_offset_trans = []
        self.child_offset_rot = []
        self.child_body = None
        self.parent_body = None
        self.spatial_transform = []

    def get_joint_attrib(self, element):
        self.type = element.tag
        self.name = (element.attrib["name"]).split("/")[-1]
        self.parent = find(element, "socket_parent_frame").split("/")[-1]
        self.child = find(element, "socket_child_frame").split("/")[-1]

        if element.find("coordinates"):
            for coordinate in element.find("coordinates").findall("Coordinate"):
                self.coordinates.append(Coordinate().get_coordinate_attrib(coordinate))

        if element.find("SpatialTransform"):
            for i, transform in enumerate(element.find("SpatialTransform").findall("TransformAxis")):
                spat_transform = SpatialTransform().get_transform_attrib(transform)
                if i < 3:
                    spat_transform.type = "rotation"
                else:
                    spat_transform.type = "translation"
                for coordinate in self.coordinates:
                    if coordinate.name == spat_transform.coordinate_name:
                        spat_transform.coordinate = coordinate
                self.spatial_transform.append(spat_transform)

        for frame in element.find("frames").findall("PhysicalOffsetFrame"):
            if self.parent == frame.attrib["name"]:
                self.parent_body = frame.find("socket_parent").text.split("/")[-1]
                self.parent_offset_rot = frame.find("orientation").text
                self.parent_offset_trans = frame.find("translation").text
            elif self.child == frame.attrib["name"]:
                self.child_body = frame.find("socket_parent").text.split("/")[-1]
                offset_rot = frame.find("orientation").text
                offset_trans = frame.find("translation").text
                offset_trans = [float(i) for i in offset_trans.split(" ")]
                offset_rot = [float(i) for i in offset_rot.split(" ")]
                self.child_offset_trans, self.child_offset_rot = self._convert_offset_child(offset_rot, offset_trans)
        return self

    @staticmethod
    def _convert_offset_child(offset_child_rot, offset_child_trans):
        R = compute_matrix_rotation(offset_child_rot).T
        new_translation = -np.dot(R.T, offset_child_trans)
        new_rotation = -rot2eul(R)
        new_rotation_str = ''
        new_translation_str = ''
        for i in range(3):
            if i == 0:
                pass
            else:
                new_rotation_str += " "
                new_translation_str += " "
            new_rotation_str += str(new_rotation[i])
            new_translation_str += str(new_translation[i])
        return new_translation, new_rotation


class Coordinate:
    def __init__(self):
        self.name = None
        self.type = None
        self.default_value = None
        self.range = []
        self.clamped = True

    def get_coordinate_attrib(self, element):
        self.name = (element.attrib["name"]).split("/")[-1]
        self.default_value = find(element, "default_value")
        self.range = find(element, "range")
        clamped = find(element, "clamped")
        self.clamped = clamped == 'true' if clamped else None
        return self


class SpatialTransform:
    def __init__(self):
        self.name = None
        self.type = None
        self.coordinate_name = None
        self.coordinate = []
        self.axis = None
        self.function = False

    def get_transform_attrib(self, element):
        self.name = (element.attrib["name"]).split("/")[-1]
        self.coordinate_name = find(element, "coordinates")
        self.axis = find(element, "axis")
        for elt in element[0]:
            if "Function" in elt.tag and len(elt.text) != 0:
                self.function = True
        return self


class Muscle:
    def __init__(self):
        self.name = None
        self.via_point = []
        self.type = None
        self.origin = None
        self.insersion = None
        self.optimal_length = None
        self.maximal_force = None
        self.tendon_slack_length = None
        self.pennation_angle = None
        self.applied = None
        self.pcsa = None
        self.maximal_velocity = None
        self.wrap = False
        self.group = None
        self.state_type = None

    def get_muscle_attrib(self, element):
        self.name = (element.attrib["name"]).split("/")[-1]
        self.maximal_force = find(element, "max_isometric_force")
        self.optimal_length = find(element, "optimal_fiber_length")
        self.tendon_slack_length = find(element, "tendon_slack_length")
        self.pennation_angle = find(element, "pennation_angle_at_optimal")
        self.maximal_velocity = find(element, "max_contraction_velocity")

        if element.find("appliesForce"):
            self.applied = element.find("appliesForce").text == 'true'

        for path_point_elt in element.find("GeometryPath").find("PathPointSet")[0].findall("PathPoint"):
            via_point = PathPoint().get_path_point_attrib(path_point_elt)
            via_point.muscle = self.name
            self.via_point.append(via_point)
        self.group = [self.via_point[0].body, self.via_point[-1].body]
        for i in range(len(self.via_point)):
            self.via_point[i].muscle_group = f"{self.group[0]}_to_{self.group[1]}"

        if element.find("GeometryPath").find("PathWrapSet"):
            wrap = element.find("GeometryPath").find("PathWrapSet")[0].text
            n_wrap = 0 if not wrap else len(wrap)
            self.wrap = True if n_wrap != 0 else False

        self.insersion = self.via_point[-1].position
        self.origin = self.via_point[0].position

        return self


class PathPoint:
    def __init__(self):
        self.name = None
        self.muscle = None
        self.body = None
        self.muscle_group = None
        self.position = None

    def get_path_point_attrib(self, element):
        self.name = element.attrib["name"]
        self.body = find(element, "socket_parent_frame").split("/")[-1]
        self.position = find(element, "location")
        return self


class Marker:
    def __init__(self):
        self.name = None
        self.parent = None
        self.position = None
        self.fixed = True

    def get_marker_attrib(self, element):
        self.name = (element.attrib["name"]).split("/")[-1]
        self.position = find(element, "location")
        self.parent = find(element, "socket_parent_frame").split("/")[-1]
        fixed = find(element, "fixed")
        self.fixed = fixed == 'true' if fixed else None
        return self

