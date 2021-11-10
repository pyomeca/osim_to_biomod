from lxml import etree
from OsimToBiomod.utils import *
from numpy.linalg import inv
import os


# TODO :
#   - Add class for joints, bodies, muscles, markers

class Converter:

    def __init__(self, path, origin_file):
        self.path = path
        self.origin_file = origin_file
        self.version = str(4)

        self.data_origin = etree.parse(self.origin_file)
        self.root = self.data_origin.getroot()

        self.file = open(self.path, 'w')
        self.file.write('version ' + self.version + '\n')
        self.file.write('\n// File extracted from ' + self.origin_file)
        self.file.write('\n')

    def get_frames_offsets(self, _joint, _joint_type):
        offset_data = []
        if _joint == 'None':
            return [[], []]

        joint_of_interest = go_to(self.root, _joint_type, 'name', _joint)
        for i_att in joint_of_interest.getchildren():
            info = {}
            if i_att.tag =='frames':
                for j in i_att.getchildren():
                    for j_info in j.getchildren():
                        if j_info.tag in ['socket_parent', 'translation', 'orientation']:
                            info.update({j_info.tag:j_info.text})
                    offset_data.append(info.copy())

        # Todo : add check that first is parent and second is child
        offset_parent = [[float(i) for i in offset_data[0]['translation'].split(' ')],
                         [float(i) for i in offset_data[0]['orientation'].split(' ')]]
        offset_child = [[float(i) for i in offset_data[1]['translation'].split(' ')],
                        [-float(i) for i in offset_data[1]['orientation'].split(' ')]]

        if any([item for sublist in offset_child for item in sublist]):
            R = compute_matrix_rotation(offset_child[1]).T
            new_translation = -np.dot(R.T, offset_child[0])
            new_rotation = -rot2eul(R)
            offset_child = [new_translation, new_rotation]

        return [offset_parent, offset_child]


    def matrix_inertia(self, _body):
        _ref = new_text(go_to(go_to(self.root, 'Body', 'name', _body), 'inertia'))
        if _ref != 'None':
            _inertia_str = _ref
            _inertia = [float(s) for s in _inertia_str.split(' ')]
            return _inertia
        else:
            return 'None'

    def muscle_list(self):
        _list = []
        for _muscle in self.data_origin.xpath(
                '/OpenSimDocument/Model/ForceSet/objects/Thelen2003Muscle'):
            _list.append(_muscle.get("name"))
        return _list

    def list_pathpoint_muscle(self, _muscle):
        # return list of viapoint for each muscle
        _viapoint = []
        # TODO warning for other type of pathpoint
        index_pathpoint = index_go_to(go_to(self.root, 'Thelen2003Muscle', 'name', _muscle), 'PathPoint')
        _list_index = list(index_pathpoint)
        _tronc_list_index = _list_index[:len(_list_index) - 2]
        _tronc_index = ''.join(_tronc_list_index)
        index_root = index_go_to(self.root, 'Thelen2003Muscle', 'name', _muscle)
        index_tronc_total = index_root + _tronc_index
        i = 0
        while True:
            try:
                child = eval('self.root' + index_tronc_total + str(i) + ']')
                _viapoint.append(child.get("name"))
                i += 1
            except:  # Exception as e:   print('Error', e)
                break
        return _viapoint

    def list_markers_body(self, _body):
        # return list of transformation for each body
        markers = []
        index_markers = index_go_to(self.root, 'Marker')
        if index_markers is None:
            return []
        else:
            _list_index = list(index_markers)
            _tronc_list_index = _list_index[:len(_list_index) - 2]
            _tronc_index = ''.join(_tronc_list_index)
            i = 0
            while True:
                try:
                    child = eval('self.root' + _tronc_index + str(i) + ']').get('name')
                    which_body = new_text(go_to(go_to(self.root, 'Marker', 'name', child), 'socket_parent_frame'))[
                                 9:]
                    if which_body == _body:
                        markers.append(child) if child is not None else True
                    i += 1
                except:
                    break
            return markers

    def dof_of_joint(self, _joint, _joint_type):
        dof = []
        _index_dof = index_go_to(go_to(self.root, _joint_type, 'name', _joint), 'Coordinate')
        if _index_dof is None:
            return []
        else:
            _list_index = list(_index_dof)
            _tronc_list_index = _list_index[:len(_list_index) - 2]
            _tronc_index = ''.join(_tronc_list_index)
            _index_root = index_go_to(self.root, _joint_type, 'name', _joint)
            _index_tronc_total = _index_root + _tronc_index
            i = 0
            while True:
                try:
                    child = eval('self.root' + _index_tronc_total + str(i) + ']')
                    if child.get('name') is not None:
                        dof.append(child.get("name"))
                    i += 1
                except:
                    break
        return dof

    @staticmethod
    def parent_child(_child, list_joint):
        # return parent of a child
        # suppose that a parent can only have one child
        for _joint in list_joint:
            if _joint[2] == _child:
                return _joint[1]
        else:
            return 'None'

    @staticmethod
    def joint_body(_body, list_joint):
        # return the joint to which the body is child
        for _joint in list_joint:
            if _joint[2] == _body:
                return _joint[0], _joint[3]
        else:
            return 'None', 'None'

    def transform_of_joint(self, _joint, _joint_type):
        _translation = []
        _rotation = []
        if _joint == 'None':
            return [[], []]
        _index_transform = index_go_to(go_to(self.root, _joint_type, 'name', _joint), 'TransformAxis')
        if _index_transform is None:
            return [[], []]
        else:
            _list_index = list(_index_transform)
            _tronc_list_index = _list_index[:len(_list_index) - 2]
            _tronc_index = ''.join(_tronc_list_index)
            _index_root = index_go_to(self.root, _joint_type, 'name', _joint)
            if not _index_root:
                pass
            _index_tronc_total = _index_root + _tronc_index
            i = 0
            while True:
                try:
                    child = eval('self.root' + _index_tronc_total + str(i) + ']')
                    if child.get('name') is not None:
                        _translation.append(child.get("name")) \
                            if child.get('name').find('translation') == 0 else True
                        _rotation.append(child.get("name")) \
                            if child.get('name').find('rotation') == 0 else True
                    i += 1
                except:  # Exception as e:  print('Error', e)
                    break
        return [_translation, _rotation]

    def get_body_pathpoint(self, _pathpoint, muscle):
        return new_text(
            go_to(go_to(go_to(self.root, 'Thelen2003Muscle', 'name', muscle), 'PathPoint', 'name', _pathpoint),
                  'socket_parent_frame'))[9:]
        # while True:
        #     try:
        #         if index_go_to(go_to(self.root, 'PathPoint', 'name', _pathpoint),
        #                        'socket_parent_frame') is not None or '':
        #             _ref = new_text(go_to(
        #                 go_to(self.root, 'PathPoint', 'name', _pathpoint), 'socket_parent_frame'))
        #             return _ref[9:]
        #         if index_go_to(go_to(self.root, 'ConditionalPathPoint', 'name', _pathpoint),
        #                        'socket_parent_frame') is not None or '':
        #             _ref = new_text(go_to(
        #                 go_to(self.root, 'ConditionalPathPoint', 'name', _pathpoint), 'socket_parent_frame'))
        #             return _ref[9:]
        #         if index_go_to(go_to(self.root, 'MovingPathPoint', 'name', _pathpoint),
        #                        'socket_parent_frame') is not None or '':
        #             _ref = new_text(
        #                 go_to(go_to(self.root, 'MovingPathPoint', 'name', _pathpoint), 'socket_parent_frame'))
        #             return _ref[9:]
        #         else:
        #             return 'None'
        #     except Exception as e:
        #         break

    def get_pos(self, _pathpoint, muscle):
        return new_text(go_to(go_to(go_to(self.root, 'Thelen2003Muscle', 'name', muscle), 'PathPoint', 'name', _pathpoint),
                       'location'))
        # while True:
        #     try:
        #         if index_go_to(go_to(self.root, 'PathPoint', 'name', _pathpoint), 'location') != '':
        #             return new_text(go_to(go_to(self.root, 'PathPoint', 'name', _pathpoint), 'location'))
        #         elif index_go_to(go_to(self.root, 'ConditionalPathPoint', 'name', _pathpoint), 'location') != '':
        #             return new_text(go_to(go_to(self.root, 'ConditionalPathPoint', 'name', _pathpoint), 'location'))
        #         elif index_go_to(go_to(self.root, 'MovingPathPoint', 'name', _pathpoint), 'location') != '':
        #             return new_text(go_to(go_to(self.root, 'MovingPathPoint', 'name', _pathpoint), 'location'))
        #         else:
        #             return 'None'
        #     except Exception as e:
        #         break

    @staticmethod
    def muscle_group_reference(_muscle, ref_group):
        for el in ref_group:
            if _muscle == el[0]:
                return el[1]
        else:
            return 'None'

    # TODO change spaces into \t
    def printing_segment(
            self,
            _body,
            _name,
            parent_name,
            frame_offset,
            rt_in_matrix=0,
            true_segment=False,
            _dof_total_trans='',
            _dof_total_rot='',
            _range_q=None,
            _is_dof='False'
    ):
        if rt_in_matrix not in [0, 1]:
            raise RuntimeError('rt_in_matrix can be only set to 1 or 0.')
        if rt_in_matrix == 1:
            [[r14], [r24], [r34]] = frame_offset.get_translation().tolist()
            [r41, r42, r43, r44] = [0, 0, 0, 1]
            for i in range(3):
                for j in range(3):
                    globals()['r' + str(i + 1) + str(j + 1)] = round(frame_offset.get_rotation_matrix()[i][j], 9)
                    # globals()['r' + str(i + 1) + str(j + 1)] = frame_offset.get_rotation_matrix()[i][j]
        range_q = ''
        if _range_q is not None and len(_range_q) != 0:
            if isinstance(_range_q, list):
                _range_q = np.array(_range_q)
            if len(_range_q.shape) == 1:
                _range_q = _range_q.reshape(1, 2)
            for i in range(_range_q.shape[0]):
                range_q += f'\t{_range_q[i, 0]}\t{_range_q[i, 1]}\n'

        # for i in _range_q:
        #     range_q_text += f'               {i[0]}\t{i[1]}\n'
        [i11, i22, i33, i12, i13, i23] = self.matrix_inertia(_body)
        mass = new_text(go_to(go_to(self.root, 'Body', 'name', _body), 'mass'))
        com = new_text(go_to(go_to(self.root, 'Body', 'name', _body), 'mass_center'))
        path_mesh_file = new_text(go_to(go_to(self.root, 'Body', 'name', _body), 'mesh_file'))
        # TODO add mesh files

        # writing data
        self.write('    // Segment\n')
        self.write('    segment {}\n'.format(_name)) if _name != 'None' else self.write('')
        self.write('        parent {} \n'.format(parent_name)) if (
                    parent_name != 'None' or parent_name != 'ground') else self.write('')
        self.write('        RTinMatrix    {}\n'.format(rt_in_matrix)) if rt_in_matrix != 'None' else self.write(
            '')
        if rt_in_matrix == 0:
            self.write('        RT\t{}\txyz\t{}\n'.
                       format(' '.join(map(str, frame_offset[1])), ' '.join(map(str, frame_offset[0]))))
        else:
            self.write('        RT\n')
            self.write(
                '            {}    {}    {}    {}\n'
                '            {}    {}    {}    {}\n'
                '            {}    {}    {}    {}\n'
                '            {}    {}    {}    {}\n'
                    .format(r11, r12, r13, r14,
                            r21, r22, r23, r24,
                            r31, r32, r33, r34,
                            r41, r42, r43, r44))

        self.write(f'        translations {_dof_total_trans}\n'
                   )if _dof_total_trans != '' else True

        self.write(f'        rotations {_dof_total_rot}\n') if _is_dof == 'True' and _dof_total_rot != '' else True

        if _range_q is not None and len(_range_q) != 0:
            self.write('        ranges\t'
                       '{}'.format(range_q))
        self.write('        mass {}\n'.format(mass)) if true_segment is True else True
        self.write('        inertia\n'
                   '            {}    {}    {}\n'
                   '            {}    {}    {}\n'
                   '            {}    {}    {}\n'
                   .format(i11, i12, i13,
                           i12, i22, i23,
                           i13, i23, i33)) if true_segment is True else True
        self.write('        com    {}\n'.format(com)) if true_segment is True else True
        self.write('        meshfile Geometry/{}\n'
                   .format(path_mesh_file)) if path_mesh_file != 'None' and true_segment else True
        self.write('    endsegment\n')

    def __getattr__(self, attr):
        print(f'Error : {attr} is not an attribute of this class')

    def get_path(self):
        return self.path

    def write(self, string):
        self.file = open(self.path, 'a')
        self.file.write(string)
        self.file.close()

    def get_origin_file(self):
        return self.originfile

    def credits(self):
        return self.data_origin.xpath(
            '/OpenSimDocument/Model/credits')[0].text

    def publications(self):
        return self.data_origin.xpath(
            '/OpenSimDocument/Model/publications')[0].text

    def body_list(self):
        _list = []
        for body in self.data_origin.xpath(
                '/OpenSimDocument/Model/BodySet/objects/Body'):
            _list.append(body.get("name"))
        return _list

    def get_q_range(self, _is_dof, _joint_type, _joint):
        range_q = new_text(go_to(go_to(go_to(self.root, _joint_type, 'name', _joint), 'Coordinate', 'name', _is_dof),
                                 'range'))
        range_value = []
        for r in range_q.split(' '):
            if r != '' and r != 'None':
                range_value.append(float(r))
        return range_value

    def update_q_range(self, _range_q, _Transform_function):
        new_range = [-1, 1]
        if _Transform_function[0] == 'LinearFunction':
            new_range = [i_r * float(_Transform_function[1][0]) + float(_Transform_function[1][1]) for i_r in _range_q]
        elif _Transform_function[0] == 'SimmSpline':
            y_value = [float(i) for i in ' '.join(_Transform_function[1][1].split(' ')).split()]
            new_range = [min(y_value), max(y_value)]
        elif _Transform_function[0] == 'MultiplierFunction':
            # y_value = [float(i)*float(_Transform_function[1]) for i in ' '.join(_Transform_function[2][1].split(' ')).split()]
            # new_range = [min(y_value), max(y_value)]
            new_range = [-1, 1]

        return new_range

    def extract_information_dof(self, _list_transform, _type_transf, _joint_type, _joint, range_q):
        dof_chain_loc = ''
        _dof_total_transf = ''
        list_trans_dof = ['x', 'y', 'z']
        for i_transf in _list_transform:
            if i_transf.find(_type_transf) == 0:
                dof_information = '\n'

                axis_str = new_text(go_to(
                    go_to(
                        go_to(self.root, _joint_type, 'name', _joint),
                        'TransformAxis', 'name', i_transf)
                    , 'axis'))

                axis = [float(s) for s in axis_str.split(' ')]

                is_dof = new_text(go_to(
                    go_to(
                        go_to(self.root, _joint_type, 'name', _joint),
                        'TransformAxis', 'name', i_transf),
                    'coordinates'))

                Transform_function = []

                dof_information += f'//dof axis: {_joint} at {i_transf} on {axis_str}\n'

                for i_att in go_to(go_to(self.root, _joint_type, 'name', _joint), 'TransformAxis',
                                   'name', i_transf).getchildren():

                    if i_att.tag == 'coordinates':
                        dof_information += f'//coordinates that serve as the independent variables: {i_att.text}\n'

                    if i_att.tag in ['LinearFunction', 'SimmSpline', 'MultiplierFunction']:
                        Transform_function.append(i_att.tag)

                        if i_att.tag == 'LinearFunction':
                            linear_func = ' '.join(i_att[0].text.split(' ')).split()
                            Transform_function.append(linear_func)
                            if linear_func != ['1', '0']:
                                constraints_in_model = True

                        elif i_att.tag == 'SimmSpline':
                            Transform_function.append([i_att[0].text, i_att[1].text])
                            constraints_in_model = True

                        elif i_att.tag == 'MultiplierFunction':
                            for iparam in i_att.getchildren():
                                if iparam.tag == 'scale':
                                    scale = iparam.text
                                if iparam.tag == 'function':
                                    simmsp = [iparam[0][0].text, iparam[0][1].text]
                            Transform_function.append(scale)
                            Transform_function.append(simmsp)
                            Transform_function = [1]
                            constraints_in_model = True

                dof_information += f'//Transform function: \n\t//Function type: {Transform_function[0]}\n' \
                    f'\t//Parameters: {Transform_function[1:]}\n'

                if not Transform_function:
                    raise (f'Transform function for {_joint} at {axis_str} is unknown,'
                           f' only LinearFunction,  SimmSpline and MultiplierFunction are implemented')

                if is_dof in self.dof_of_joint(_joint, _joint_type):
                    # _dof_total_transf += list_trans_dof[axis.index(1.0)]
                    range_q_value = self.get_q_range(is_dof, _joint_type, _joint)
                    range_q_value_updated = self.update_q_range(range_q_value, Transform_function)
                    range_q.append(range_q_value_updated)
                    if 1.0 in axis:
                        _dof_total_transf += list_trans_dof[axis.index(1.0)]
                    else:
                        _dof_total_transf += list_trans_dof[axis.index(-1.0)]
                dof_chain_loc += dof_information

        return dof_chain_loc, range_q, _dof_total_transf

        #        # Credits
        #        self.write('\n// CREDITS')
        #        _credits = print_credits()
        #        self.write('\n'+_credits+'\n')
        #
        #         # Publications
        #        self.write('\n// PUBLICATIONS\n')
        #        _publications = print_publications()
        #        self.write('\n'+_publications+'\n')

    def main(self):
        # list of joints with parent and child
        list_joint = []
        joint_type = ["WeldJoint", "CustomJoint"]

        # Gravity
        gravity_vector = new_text(go_to(self.root, 'gravity'))
        self.write(f"\ngravity\t{gravity_vector}\n")

        for joint in joint_type:
            index_joints = index_go_to(self.root, joint)
            if index_joints is not None:
                list_index = list(index_joints)
                tronc_list_index = list_index[:len(list_index) - 2]
                tronc_index = ''.join(tronc_list_index)
                # check these if/else rows
                if joint == 'WeldJoint':
                    i = 0
                else:
                    i = int(list_index[len(list_index) - 2])
                while True:
                    try:
                        new_joint = eval('self.root' + tronc_index + str(i) + ']').get('name')
                        if new_text(go_to(self.root, joint, 'name', new_joint)) != 'None':
                            _parent_joint = new_text(
                                go_to(go_to(self.root, joint, 'name', new_joint), 'socket_parent_frame'))[:-7]
                            _child_joint = new_text(
                                go_to(go_to(self.root, joint, 'name', new_joint), 'socket_child_frame'))[:-7]
                            list_joint.append([new_joint, _parent_joint, _child_joint, joint])
                        i += 1
                    except:
                        break

        # Segment definition
        self.write('\n// SEGMENT DEFINITION\n')
        # Division of body in segment depending of transformation
        for body in self.body_list():
            rotomatrix = OrthoMatrix([0, 0, 0])
            self.write(f'\n// Information about {body} segment\n')
            parent = self.parent_child(body, list_joint)
            joint, joint_type = self.joint_body(body, list_joint)
            physical_offset = self.get_frames_offsets(joint, joint_type)
            list_transform = self.transform_of_joint(joint, joint_type)
            # Add default values for list_transform
            axis_offset = np.identity(3)
            # segment data

            self.printing_segment(
                body, body + '_parent_offset', parent, physical_offset[0], rt_in_matrix=0, true_segment=False
                                  )

            parent = body + '_parent_offset'
            # If there are no transformation (e.g. welded joint)
            if list_transform[0] == []:
                if list_transform[1] == []:
                    pass

            # For other joint (e.g. custom)
            else:
                self.write("\n    // Segments to define transformation axis.\n")
                body_trans = body + '_translation'
                dof_total_trans = ''
                j = 0
                list_trans_dof = ['x', 'y', 'z']
                transform_translation = []
                is_dof = []
                for translation in list_transform[0]:
                    if translation.find('translation') == 0:
                        axis_str = new_text(go_to(
                            go_to(go_to(self.root, joint_type, 'name', joint), 'TransformAxis', 'name', translation),
                            'axis'))
                        axis = [float(s) for s in axis_str.split(' ')]
                        transform_translation.append(axis)
                        current_dof = new_text(
                            go_to(go_to(go_to(self.root, joint_type, 'name', joint), 'TransformAxis', 'name', translation),
                                  'coordinates'))
                        is_dof.append(current_dof)
                        if current_dof in self.dof_of_joint(joint, joint_type):
                            dof_total_trans += list_trans_dof[j]
                        j += 1

                # Check if transformation basis is orthonormal
                if is_ortho_basis(transform_translation):
                    x = transform_translation[0]
                    y = transform_translation[1]
                    z = transform_translation[2]
                    rotomatrix.set_rotation_matrix(np.append(x, np.append(y, z)).reshape(3, 3).T)
                    # axis offset
                    q_range = []
                    for i in range(len(is_dof)):
                        if is_dof[i] != 'None' and is_dof[i]:
                            q_range.append(self.get_q_range(is_dof[i], joint_type, joint))
                    self.printing_segment(body, body_trans, parent, rotomatrix, rt_in_matrix=1, true_segment=False,
                                          _dof_total_trans=dof_total_trans,
                                          _range_q=q_range
                                          )
                    axis_offset = axis_offset.dot(rotomatrix.get_rotation_matrix())
                    parent = body_trans
                else:
                    raise RuntimeError("Translation vector no orthogonal non implemented yet")

            if list_transform[1] != []:
                list_rot_dof = ['x', 'y', 'z']
                count_dof_rot = 0
                transform_rotation = []
                is_dof = []
                default_value = []
                for rotation in list_transform[1]:
                    if rotation.find('rotation') == 0:
                        axis_str = new_text(
                            go_to(go_to(go_to(self.root, joint_type, 'name', joint), 'TransformAxis', 'name', rotation),
                                  'axis'))
                        axis = [float(s) for s in axis_str.split(' ')]
                        transform_rotation.append(axis)
                        is_dof.append(new_text(
                            go_to(go_to(go_to(self.root, joint_type, 'name', joint), 'TransformAxis', 'name', rotation),
                                  'coordinates')))

                        if is_dof[-1] and is_dof[-1] != 'None' and is_dof[-1]:
                            default_value.append(float(new_text(
                                go_to(
                                    go_to(
                                        go_to(self.root, joint_type, 'name', joint
                                              ),
                                        'Coordinate', 'name', is_dof[-1]
                                    ), 'default_value'
                                )
                            )))

                        else:
                            default_value.append(0)

                if is_ortho_basis(transform_rotation):
                    if np.linalg.norm(default_value) != 0:
                        raise RuntimeError("Default value for orthogonal basis not implemented yet.")
                    x = transform_rotation[0]
                    y = transform_rotation[1]
                    z = transform_rotation[2]
                    rotomatrix.set_rotation_matrix(np.append(x, np.append(y, z)).reshape(3, 3).T)
                    body_name = body + '_rotation_transform'
                    rot_dof = "xyz"
                    q_range = []
                    for i in range(len(is_dof)):
                        if is_dof[i] != 'None' and is_dof[i]:
                            q_range.append(self.get_q_range(is_dof[i], joint_type, joint))
                    self.write("// Rotation transform was initially an orthogonal basis\n")
                    self.printing_segment(body, body_name, parent, rotomatrix, rt_in_matrix=1,
                                          _dof_total_rot=rot_dof, true_segment=False, _is_dof='True'
                                          , _range_q=q_range
                                          )
                    axis_offset = axis_offset.dot(rotomatrix.get_rotation_matrix())
                    parent = body_name

                else:
                    axis_basis = []
                    q_range = None
                    for i in range(len(transform_rotation)):
                        rotation = list_transform[1][i]
                        if len(axis_basis) == 0:
                            axis_basis.append(ortho_norm_basis(transform_rotation[i], i))
                            initial_rotation = compute_matrix_rotation([default_value[i], 0, 0])
                            if is_dof[i] != 'None' and is_dof[i]:
                                q_range = self.get_q_range(is_dof[i], joint_type, joint)

                        elif len(axis_basis) == 1:
                            axis_basis.append(inv(axis_basis[i - 1]).dot(ortho_norm_basis(transform_rotation[i], i)))
                            initial_rotation = compute_matrix_rotation([0, default_value[i], 0])
                            if is_dof[i] != 'None' and is_dof[i]:
                                q_range = self.get_q_range(is_dof[i], joint_type, joint)
                        else:
                            axis_basis.append(inv(axis_basis[i - 1]).dot(inv(axis_basis[i - 2])).dot(ortho_norm_basis(transform_rotation[i], i)))
                            initial_rotation = compute_matrix_rotation([0, 0, default_value[i]])
                            if is_dof[i] != 'None' and is_dof[i]:
                                q_range = self.get_q_range(is_dof[i], joint_type, joint)

                        if is_dof[i] in self.dof_of_joint(joint, joint_type):
                            dof_rot = list_rot_dof[count_dof_rot]
                            activate_dof = 'True'
                            body_dof = body + '_' + is_dof[i]
                        else:
                            body_dof = body + '_' + rotation
                            activate_dof = 'None'
                            dof_rot = ''

                        rotomatrix.set_rotation_matrix(axis_basis[i].dot(initial_rotation))
                        count_dof_rot += 1
                        self.printing_segment(body, body_dof, parent, rotomatrix, rt_in_matrix=1,
                                              _dof_total_rot=dof_rot, true_segment=False, _is_dof=activate_dof,
                                              _range_q=q_range
                                              )
                        axis_offset = axis_offset.dot(rotomatrix.get_rotation_matrix())
                        parent = body_dof

                #### segment to cancel axis effects
                self.write("\n    // Segment to cancel transformation bases effect.\n")
                rotomatrix.set_rotation_matrix(inv(axis_offset))
                self.printing_segment(body, body + '_reset_axis', parent, rotomatrix, rt_in_matrix=1, true_segment=False)
                parent = body + '_reset_axis'

            self.write("\n    //True segment where are applied inertial values.\n")
            self.printing_segment(body, body, parent, physical_offset[1], rt_in_matrix=0, true_segment=True)
            parent = body

            # Markers
            _list_markers = self.list_markers_body(body)
            if _list_markers is not []:
                self.write('\n    // Markers')
                for marker in _list_markers:
                    position = new_text(go_to(go_to(self.root, 'Marker', 'name', marker), 'location'))
                    self.write('\n    marker    {}'.format(marker))
                    self.write('\n        parent    {}'.format(parent))
                    self.write('\n        position    {}'.format(position))
                    self.write('\n    endmarker\n')
            late_body = body

            ###########    Coupled Coordinates constraints
            constraints_in_model = False
            constraints_output = '\n//COUPLED COORDINATES\n\n'
            cc_constraints = index_go_to(self.root, 'CoordinateCouplerConstraint')
            if cc_constraints is not None:

                constraints_in_model = True
                list_index = list(cc_constraints)
                cc_index = ''.join(list_index[:len(list_index) - 2])
                i = 0
                while True:
                    try:
                        new_ccc = eval('self.root' + cc_index + str(i) + ']').get('name')
                        constraints_output += f'\n//name: {new_ccc}\n'
                        qx = new_text(go_to(go_to(self.root, 'CoordinateCouplerConstraint', 'name', new_ccc),
                                            'independent_coordinate_names'))
                        qy = new_text(go_to(go_to(self.root, 'CoordinateCouplerConstraint', 'name', new_ccc),
                                            'dependent_coordinate_name'))
                        type_coupling = go_to(go_to(self.root, 'CoordinateCouplerConstraint', 'name', new_ccc),
                                              'coupled_coordinates_function')[0].tag
                        constraints_output += f'\t//independent q: {qx}\n' \
                                              f'\t//dependent q: {qy}\n' \
                                              f'\t//coupling type: {type_coupling}\n'
                        i += 1
                    except:
                        break

        # Muscle definition
        self.write('\n// MUSCLE DEFINIION\n')
        sort_muscle = []
        muscle_ref_group = []
        for muscle in self.muscle_list():
            viapoint = self.list_pathpoint_muscle(muscle)
            bodies_viapoint = []
            for pathpoint in viapoint:
                bodies_viapoint.append(self.get_body_pathpoint(pathpoint, muscle))

            # it is supposed that viapoints are organized in order
            # from the parent body to the child body
            body_start = bodies_viapoint[0]
            body_end = bodies_viapoint[len(bodies_viapoint) - 1]
            sort_muscle.append([body_start, body_end])
            muscle_ref_group.append([muscle, body_start + '_to_' + body_end])

        # selecting muscle group
        group_muscle = []
        for ext_muscle in sort_muscle:
            if ext_muscle not in group_muscle:
                group_muscle.append(ext_muscle)

        for muscle_group in group_muscle:
            self.write('\n// {} > {}\n'.format(muscle_group[0], muscle_group[1]))
            self.write('musclegroup {}\n'.format(muscle_group[0] + '_to_' + muscle_group[1]))
            self.write('    OriginParent        {}\n'.format(muscle_group[0]))
            self.write('    InsertionParent        {}\n'.format(muscle_group[1]))
            self.write('endmusclegroup\n')

            count = 0
            for muscle in self.muscle_list():
                if muscle_ref_group[count][1] == muscle_group[0] + '_to_' + muscle_group[1]:
                    m_ref = muscle_ref_group[count][1]
                    muscle_type = 'hillthelen'
                    state_type = 'buchanan'
                    list_pathpoint = self.list_pathpoint_muscle(muscle)
                    start_point = list_pathpoint.pop(0)
                    end_point = list_pathpoint.pop()
                    start_pos = self.get_pos(start_point, muscle)
                    insert_pos = self.get_pos(end_point, muscle)
                    opt_length = new_text(
                        go_to(go_to(self.root, 'Thelen2003Muscle', 'name', muscle), 'optimal_fiber_length'))
                    max_force = new_text(
                        go_to(go_to(self.root, 'Thelen2003Muscle', 'name', muscle), 'max_isometric_force'))
                    tendon_slack_length = new_text(
                        go_to(go_to(self.root, 'Thelen2003Muscle', 'name', muscle), 'tendon_slack_length'))
                    pennation_angle = new_text(
                        go_to(go_to(self.root, 'Thelen2003Muscle', 'name', muscle), 'pennation_angle_at_optimal'))
                    pcsa = new_text(go_to(go_to(self.root, 'Thelen2003Muscle', 'name', muscle), 'pcsa'))
                    max_velocity = new_text(
                        go_to(go_to(self.root, 'Thelen2003Muscle', 'name', muscle), 'max_contraction_velocity'))

                    # print muscle data
                    self.write('\n    muscle    {}'.format(muscle))
                    self.write('\n        Type    {}'.format(muscle_type)) if muscle_type != 'None' else self.write('')
                    self.write('\n        statetype    {}'.format(state_type)) if state_type != 'None' else self.write(
                        '')
                    self.write('\n        musclegroup    {}'.format(m_ref)) if m_ref != 'None' else self.write('')
                    self.write(
                        '\n        OriginPosition    {}'.format(start_pos)) if start_pos != 'None' else self.write('')
                    self.write(
                        '\n        InsertionPosition    {}'.format(insert_pos)) if insert_pos != 'None' else self.write(
                        '')
                    self.write(
                        '\n        optimalLength    {}'.format(opt_length)) if opt_length != 'None' else self.write('')
                    self.write('\n        maximalForce    {}'.format(max_force)) if max_force != 'None' else self.write(
                        '')
                    self.write('\n        tendonSlackLength    {}'.format(
                        tendon_slack_length)) if tendon_slack_length != 'None' else self.write('')
                    self.write('\n        pennationAngle    {}'.format(
                        pennation_angle)) if pennation_angle != 'None' else self.write('')
                    self.write('\n        PCSA    {}'.format(pcsa)) if pcsa != 'None' else self.write('')
                    self.write(
                        '\n        maxVelocity    {}'.format(max_velocity)) if max_velocity != 'None' else self.write(
                        '')
                    self.write('\n    endmuscle\n')

                    # viapoint
                    for viapoint in list_pathpoint:
                        # viapoint data
                        parent_viapoint = self.get_body_pathpoint(viapoint, muscle)
                        viapoint_pos = self.get_pos(viapoint, muscle)
                        # print viapoint data
                        self.write('\n        viapoint    {}'.format(viapoint))
                        self.write('\n            parent    {}'.format(
                            parent_viapoint)) if parent_viapoint != 'None' else self.write('')
                        self.write('\n            muscle    {}'.format(muscle))
                        self.write('\n            musclegroup    {}'.format(m_ref)) if m_ref != 'None' else self.write(
                            '')
                        self.write('\n            position    {}'.format(
                            viapoint_pos)) if viapoint_pos != 'None' else self.write('')
                        self.write('\n        endviapoint')
                    self.write('\n')
                count += 1

        ###########    Additional data as comment about the model's constraints
        if constraints_in_model:
            self.write(constraints_output)
            self.write(dof_chain)
            self.write_insert(3, '\n\nWARNING\n'
                                 '// The original model has some constrained DOF, thus it can not be '
                                 'directly used for kinematics or dynamics analysis.\n'
                                 '//If used in optimization, constraints should be added to the nlp to account'
                                 ' for the reduced number of DOF\n'
                                 '// Check end of file for possible constraints in the osim model\n\n')
        self.file.close()
        print(f"\nYour file {self.origin_file} has been converted into {self.path}.")


if __name__ == '__main__':
    model_path = os.path.dirname(os.getcwd()) + "/Models/"
    converter = Converter(model_path + "wu_converted.bioMod", model_path + "Wu_Shoulder_Model_via_points.osim")
    converter.main()
