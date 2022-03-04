# -*- coding: utf-8 -*-

import os
import errno
import traceback

import numpy as np
import six  # compatibility between python 2 + 3 = six
import matplotlib.pyplot as plt

try:
    import sim as sim
except Exception as e:
   
    raise e

import tensorflow as tf

from tensorflow.python.platform import flags
from tensorflow.python.platform import gfile
from tensorflow.python.ops import data_flow_ops
from ply import write_xyz_rgb_as_ply
from PIL import Image

try:
    from tqdm import tqdm
except ImportError:

    def tqdm(*args, **kwargs):
        if args:
            return args[0]
        return kwargs.get('iterable', None)

from depth_image_encoding import ClipFloatValues
from depth_image_encoding import FloatArrayToRgbImage
from depth_image_encoding import FloatArrayToRawRGB
from skimage.transform import resize
from skimage import img_as_ubyte
from skimage import img_as_uint
from skimage.color import grey2rgb

try:
    import eigen  # https://github.com/jrl-umi3218/Eigen3ToPython
    import sva  # https://github.com/jrl-umi3218/SpaceVecAlg
except ImportError:
    print('eigen and sva python modules are not available. To install run the script at:'
          'https://github.com/ahundt/robotics_setup/blob/master/robotics_tasks.sh'
          'or follow the instructions at https://github.com/jrl-umi3218/Eigen3ToPython'
          'and https://github.com/jrl-umi3218/SpaceVecAlg. '
          'When you build the modules make sure python bindings are enabled.')

tf.flags.DEFINE_string('csimVisualizeDepthFormat', 'csim_depth_encoded_rgb')
tf.flags.DEFINE_string('csimVisualizeRGBFormat', 'csim_rgb')

# the following line is needed for tf versions before 1.5
# flags.FLAGS._parse_flags()
FLAGS = flags.FLAGS


def depth_image_to_point_cloud(depth, intrinsics_matrix, dtype=np.float32, verbose=0):
   
    fy = intrinsics_matrix[1, 1]
    fx = intrinsics_matrix[0, 0]
    # center of image y coordinate
    center_y = intrinsics_matrix[2, 1]
    # center of image x coordinate
    center_x = intrinsics_matrix[2, 0]
    depth = np.squeeze(depth)
    y_range, x_range = depth.shape

    y, x = np.meshgrid(np.arange(y_range),
                       np.arange(x_range),
                       indexing='ij')
    assert y.size == x.size and y.size == depth.size
    x = x.flatten()
    y = y.flatten()
    depth = depth.flatten()

    X = (x - center_x) * depth / fx
    Y = (y - center_y) * depth / fy

    assert X.size == Y.size and X.size == depth.size
    assert X.shape == Y.shape and X.shape == depth.shape

    if verbose > 0:
        print('X np: ', X.shape)
        print('Y np: ', Y.shape)
        print('depth np: ', depth.shape)
    XYZ = np.column_stack([X, Y, depth])
    assert XYZ.shape == (y_range * x_range, 3)
    if verbose > 0:
        print('XYZ pre reshape np: ', XYZ.shape)
    XYZ = XYZ.reshape((y_range, x_range, 3))

    return XYZ.astype(dtype)


def csimPrint(client_id, message):
   
    sim.simxAddStatusbarMessage(client_id, message, sim.simx_opmode_oneshot)
    print(message)


def create_dummy(client_id, display_name, transform=None, parent_handle=-1, debug=FLAGS.csimDebugMode, operation_mode=sim.simx_opmode_blocking):
    
    if transform is None:
        transform = np.array([0., 0., 0., 0., 0., 0., 1.])
    # 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
    empty_buffer = bytearray()
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'createDummy_function',
        [parent_handle],
        transform,
        [display_name],
        empty_buffer,
        operation_mode)
    if res == sim.simx_return_ok:
        # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
        if debug is not None and 'print_transform' in debug:
            print ('Dummy name:', display_name, ' handle: ', ret_ints[0], ' transform: ', transform)
    else:
        print('create_dummy remote function call failed.')
        print(''.join(traceback.format_stack()))
        return -1
    return ret_ints[0]


def setPose(client_id, display_name, transform=None, parent_handle=-1):
    
    if transform is None:
        transform = np.array([0., 0., 0., 0., 0., 0., 1.])
    # 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
    empty_buffer = bytearray()
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'createDummy_function',
        [parent_handle],
        transform,
        [display_name],
        empty_buffer,
        sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
        print ('SetPose object name:', display_name, ' handle: ', ret_ints[0], ' transform: ', transform)
    else:
        print('setPose remote function call failed.')
        print(''.join(traceback.format_stack()))
        return -1
    return ret_ints[0]


def set_vision_sensor_image(client_id, display_name, image, convert=None, scale_factor=256000.0, operation_mode=sim.simx_opmode_oneshot_wait):
    
    strings = [display_name]
    parent_handle = -1

    # TODO(ahundt) support is_greyscale True again
    is_greyscale = 0
    csim_conversion = False
    if convert is not None:
        csim_conversion = 'sim' in convert

        if 'depth_encoded_rgb' in convert:
            image = np.array(FloatArrayToRgbImage(image, scale_factor=scale_factor, drop_blue=False), dtype=np.uint8)
        elif 'depth_rgb' in convert:

            image = img_as_uint(image)

        elif not csim_conversion:
            raise ValueError('set_vision_sensor_image() convert parameter must be one of `depth_encoded_rgb`, `depth_rgb`, or None'
                             'with the optional addition of the word `sim` to rotate 180, flip left right, then invert colors.')

    if csim_conversion:
        # rotate 180 degrees, flip left over right, then invert the colors
        image = np.array(256 - np.fliplr(np.rot90(image, 2)), dtype=np.uint8)

    if np.issubdtype(image.dtype, np.integer):
        is_float = 0
        floats = []
        color_buffer = bytearray(image.flatten().tobytes())
        color_size = image.size
        num_floats = 0
    else:
        is_float = 1
        floats = [image]
        color_buffer = bytearray()
        num_floats = image.size
        color_size = 0

    cloud_handle = -1
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'setVisionSensorImage_function',
        [parent_handle, num_floats, is_greyscale, color_size],  # int params
        np.append(floats, []),  # float params
        strings,  # string params
        # byte buffer params
        color_buffer,
        operation_mode)
    if res == sim.simx_return_ok:
        print ('point cloud handle: ', ret_ints[0])  # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
        # set the transform for the point cloud
        return ret_ints[0]
    else:
        print('insertPointCloud_function remote function call failed.')
        print(''.join(traceback.format_stack()))
        return res


def create_point_cloud(client_id, display_name, transform=None, point_cloud=None, depth_image=None, color_image=None,
                       camera_intrinsics_matrix=None, parent_handle=-1, clear=True,
                       max_voxel_size=0.01, max_point_count_per_voxel=10, point_size=10, options=8,
                       rgb_sensor_display_name=None, depth_sensor_display_name=None, convert_depth=FLAGS.csimVisualizeDepthFormat,
                       convert_rgb=FLAGS.csimVisualizeRGBFormat, save_ply_path=None, rgb_display_mode='vision_sensor'):
    
    if transform is None:
        transform = np.array([0., 0., 0., 0., 0., 0., 1.])

    if point_cloud is None:
        point_cloud = depth_image_to_point_cloud(depth_image, camera_intrinsics_matrix)
        point_cloud = point_cloud.reshape([point_cloud.size/3, 3])

    # show the depth sensor image
    if depth_sensor_display_name is not None and depth_image is not None:
        # matplotlib.image.imsave(display_name + depth_sensor_display_name + '_norotfliplr.png', depth_image)
        # rotate 180, flip left over right then invert the image colors for display in CoppeliaSim
        # depth_image = np.fliplr(np.rot90(depth_image, 2))
        # matplotlib.image.imsave(display_name + depth_sensor_display_name + '_rot90fliplr.png', depth_image)
        set_vision_sensor_image(client_id, depth_sensor_display_name, depth_image, convert=convert_depth)

    if rgb_sensor_display_name is not None and color_image is not None and rgb_display_mode == 'vision_sensor':
          set_vision_sensor_image(client_id, rgb_sensor_display_name, color_image, convert=convert_rgb)

    # Save out Point cloud
    if save_ply_path is not None:
        write_xyz_rgb_as_ply(point_cloud, color_image, save_ply_path)

    # color_buffer is initially empty
    color_buffer = bytearray()
    strings = [display_name]
    if rgb_sensor_display_name is not None and rgb_display_mode == 'point_cloud':
        strings = [display_name, rgb_sensor_display_name]

    transform_entries = 7
    if clear:
        clear = 1
    else:
        clear = 0

    cloud_handle = -1
    # Create the point cloud if it does not exist, or retrieve the handle if it does
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'createPointCloud_function',
        # int params
        [parent_handle, transform_entries, point_cloud.size, cloud_handle, clear, max_point_count_per_voxel, options, point_size],
        # float params
        [max_voxel_size],
        # string params
        strings,
        # byte buffer params
        color_buffer,
        sim.simx_opmode_blocking)

    setPose(client_id, display_name, transform, parent_handle)

    if res == sim.simx_return_ok:
        cloud_handle = ret_ints[0]

        # convert the rgb values to a string
        color_size = 0
        if color_image is not None:
            # see simInsertPointsIntoPointCloud() in sim documentation
            # 3 indicates the cloud should be in the parent frame, and color is enabled
            # bit 2 is 1 so each point is colored
            simInsertPointsIntoPointCloudOptions = 3
            # color_buffer = bytearray(np.fliplr(np.rot90(color_image, 3)).flatten().tobytes())
            color_buffer = bytearray(color_image.flatten().tobytes())
            color_size = color_image.size
        else:
            simInsertPointsIntoPointCloudOptions = 1

        # Actually transfer the point cloud
        res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
            client_id,
            'remoteApiCommandServer',
            sim.sim_scripttype_childscript,
            'insertPointCloud_function',
            [parent_handle, transform_entries, point_cloud.size, cloud_handle, color_size, simInsertPointsIntoPointCloudOptions],
            np.append(point_cloud, []),
            strings,
            color_buffer,
            sim.simx_opmode_blocking)

        if res == sim.simx_return_ok:
            print ('point cloud handle: ', ret_ints[0])  # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
            # set the transform for the point cloud
            return ret_ints[0]
        else:
            print('insertPointCloud_function remote function call failed.')
            print(''.join(traceback.format_stack()))
            return res

    else:
        print('createPointCloud_function remote function call failed')
        print(''.join(traceback.format_stack()))
        return res


def drawLines(client_id, display_name, lines, parent_handle=-1, transform=None, debug=FLAGS.csimDebugMode, operation_mode=sim.simx_opmode_blocking):
    
    empty_buffer = bytearray()
    res, ret_ints, ret_floats, ret_strings, ret_buffer = sim.simxCallScriptFunction(
        client_id,
        'remoteApiCommandServer',
        sim.sim_scripttype_childscript,
        'addDrawingObject_function',
        [parent_handle, int(lines.size/6)],
        # np.append(transform, lines),
        lines,
        [display_name],
        empty_buffer,
        operation_mode)
    if res == sim.simx_return_ok:
        # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
        if debug is not None and 'print_drawLines' in debug:
            print ('drawLines name:', display_name, ' handle: ', ret_ints[0], ' transform: ', transform)

        if transform is not None:
            # set the transform for the point cloud
            setPose(client_id, display_name, transform, parent_handle)
    else:
        print('drawLines remote function call failed.')
        print(''.join(traceback.format_stack()))
        return -1
    return ret_ints[0]


def restore_cropped(cropped_image, crop_size, crop_offset, full_size):
  
    cropped_image = np.squeeze(cropped_image)
    restored = np.zeros((full_size[0], full_size[1]), dtype=cropped_image.dtype)
    scaled_crop = resize(cropped_image, (crop_size[0], crop_size[1]))
    restored[crop_offset[0]:crop_offset[0]+crop_size[0],
             crop_offset[1]:crop_offset[1]+crop_size[1]] = scaled_crop

    return restored
