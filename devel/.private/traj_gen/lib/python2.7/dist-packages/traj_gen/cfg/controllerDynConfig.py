## *********************************************************
##
## File autogenerated for the traj_gen package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [{'upper': 'GROUP_CONTROL', 'lower': 'group_control', 'srcline': 124, 'name': 'group_control', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT::GROUP_CONTROL', 'field': 'DEFAULT::group_control', 'state': True, 'parentclass': 'DEFAULT', 'groups': [], 'parameters': [{'srcline': 35, 'description': 'enable take off', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/fechec/feng_ws/src/traj_gen/cfg/controllerDyn.cfg', 'name': 'enable_take_off', 'edit_method': '', 'default': False, 'level': 1, 'min': False, 'type': 'bool'}, {'srcline': 36, 'description': 'enable landing', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/home/fechec/feng_ws/src/traj_gen/cfg/controllerDyn.cfg', 'name': 'enable_landing', 'edit_method': '', 'default': False, 'level': 1, 'min': False, 'type': 'bool'}], 'type': '', 'id': 1}], 'parameters': [{'srcline': 291, 'description': 'test mode', 'max': 6, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'test_mode', 'edit_method': "{'enum_description': 'An enum to chose test mode', 'enum': [{'srcline': 22, 'description': 'Step z test', 'srcfile': '/home/fechec/feng_ws/src/traj_gen/cfg/controllerDyn.cfg', 'cconsttype': 'const int', 'value': 0, 'ctype': 'int', 'type': 'int', 'name': 'TEST_STEP_Z'}, {'srcline': 23, 'description': 'Step z test', 'srcfile': '/home/fechec/feng_ws/src/traj_gen/cfg/controllerDyn.cfg', 'cconsttype': 'const int', 'value': 1, 'ctype': 'int', 'type': 'int', 'name': 'TEST_STEP_PSI'}, {'srcline': 24, 'description': 'Step z test', 'srcfile': '/home/fechec/feng_ws/src/traj_gen/cfg/controllerDyn.cfg', 'cconsttype': 'const int', 'value': 2, 'ctype': 'int', 'type': 'int', 'name': 'TEST_STEP_X'}, {'srcline': 25, 'description': 'Step z test', 'srcfile': '/home/fechec/feng_ws/src/traj_gen/cfg/controllerDyn.cfg', 'cconsttype': 'const int', 'value': 3, 'ctype': 'int', 'type': 'int', 'name': 'TEST_STEP_Y'}, {'srcline': 26, 'description': 'Circular tracking test', 'srcfile': '/home/fechec/feng_ws/src/traj_gen/cfg/controllerDyn.cfg', 'cconsttype': 'const int', 'value': 4, 'ctype': 'int', 'type': 'int', 'name': 'TEST_TRACKING_CIRCLE'}, {'srcline': 27, 'description': 'Figure 8 test', 'srcfile': '/home/fechec/feng_ws/src/traj_gen/cfg/controllerDyn.cfg', 'cconsttype': 'const int', 'value': 5, 'ctype': 'int', 'type': 'int', 'name': 'TEST_TRACKING_8'}, {'srcline': 28, 'description': 'Trajectory tracking test', 'srcfile': '/home/fechec/feng_ws/src/traj_gen/cfg/controllerDyn.cfg', 'cconsttype': 'const int', 'value': 6, 'ctype': 'int', 'type': 'int', 'name': 'TEST_TRACKING_TRAJ'}]}", 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

controllerDyn_ENABLE_CTRL = 1
controllerDyn_TEST_STEP_Z = 0
controllerDyn_TEST_STEP_PSI = 1
controllerDyn_TEST_STEP_X = 2
controllerDyn_TEST_STEP_Y = 3
controllerDyn_TEST_TRACKING_CIRCLE = 4
controllerDyn_TEST_TRACKING_8 = 5
controllerDyn_TEST_TRACKING_TRAJ = 6
