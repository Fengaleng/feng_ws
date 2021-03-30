# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from target_detection/target_dect_msg.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class target_dect_msg(genpy.Message):
  _md5sum = "2a847c4ada344c2340bcd2fb12cdd132"
  _type = "target_detection/target_dect_msg"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

# references
geometry_msgs/Vector3 position_ref
geometry_msgs/Vector3 velocity_ref
geometry_msgs/Vector3 accel_ref
geometry_msgs/Vector3 jerk_ref
geometry_msgs/Vector3 snap_ref
float64[3] yaw_ref
bool launch_flag
float64 speed
bool collide_state

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z"""
  __slots__ = ['header','position_ref','velocity_ref','accel_ref','jerk_ref','snap_ref','yaw_ref','launch_flag','speed','collide_state']
  _slot_types = ['std_msgs/Header','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','geometry_msgs/Vector3','float64[3]','bool','float64','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,position_ref,velocity_ref,accel_ref,jerk_ref,snap_ref,yaw_ref,launch_flag,speed,collide_state

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(target_dect_msg, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.position_ref is None:
        self.position_ref = geometry_msgs.msg.Vector3()
      if self.velocity_ref is None:
        self.velocity_ref = geometry_msgs.msg.Vector3()
      if self.accel_ref is None:
        self.accel_ref = geometry_msgs.msg.Vector3()
      if self.jerk_ref is None:
        self.jerk_ref = geometry_msgs.msg.Vector3()
      if self.snap_ref is None:
        self.snap_ref = geometry_msgs.msg.Vector3()
      if self.yaw_ref is None:
        self.yaw_ref = [0.] * 3
      if self.launch_flag is None:
        self.launch_flag = False
      if self.speed is None:
        self.speed = 0.
      if self.collide_state is None:
        self.collide_state = False
    else:
      self.header = std_msgs.msg.Header()
      self.position_ref = geometry_msgs.msg.Vector3()
      self.velocity_ref = geometry_msgs.msg.Vector3()
      self.accel_ref = geometry_msgs.msg.Vector3()
      self.jerk_ref = geometry_msgs.msg.Vector3()
      self.snap_ref = geometry_msgs.msg.Vector3()
      self.yaw_ref = [0.] * 3
      self.launch_flag = False
      self.speed = 0.
      self.collide_state = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_15d().pack(_x.position_ref.x, _x.position_ref.y, _x.position_ref.z, _x.velocity_ref.x, _x.velocity_ref.y, _x.velocity_ref.z, _x.accel_ref.x, _x.accel_ref.y, _x.accel_ref.z, _x.jerk_ref.x, _x.jerk_ref.y, _x.jerk_ref.z, _x.snap_ref.x, _x.snap_ref.y, _x.snap_ref.z))
      buff.write(_get_struct_3d().pack(*self.yaw_ref))
      _x = self
      buff.write(_get_struct_BdB().pack(_x.launch_flag, _x.speed, _x.collide_state))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.position_ref is None:
        self.position_ref = geometry_msgs.msg.Vector3()
      if self.velocity_ref is None:
        self.velocity_ref = geometry_msgs.msg.Vector3()
      if self.accel_ref is None:
        self.accel_ref = geometry_msgs.msg.Vector3()
      if self.jerk_ref is None:
        self.jerk_ref = geometry_msgs.msg.Vector3()
      if self.snap_ref is None:
        self.snap_ref = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 120
      (_x.position_ref.x, _x.position_ref.y, _x.position_ref.z, _x.velocity_ref.x, _x.velocity_ref.y, _x.velocity_ref.z, _x.accel_ref.x, _x.accel_ref.y, _x.accel_ref.z, _x.jerk_ref.x, _x.jerk_ref.y, _x.jerk_ref.z, _x.snap_ref.x, _x.snap_ref.y, _x.snap_ref.z,) = _get_struct_15d().unpack(str[start:end])
      start = end
      end += 24
      self.yaw_ref = _get_struct_3d().unpack(str[start:end])
      _x = self
      start = end
      end += 10
      (_x.launch_flag, _x.speed, _x.collide_state,) = _get_struct_BdB().unpack(str[start:end])
      self.launch_flag = bool(self.launch_flag)
      self.collide_state = bool(self.collide_state)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_15d().pack(_x.position_ref.x, _x.position_ref.y, _x.position_ref.z, _x.velocity_ref.x, _x.velocity_ref.y, _x.velocity_ref.z, _x.accel_ref.x, _x.accel_ref.y, _x.accel_ref.z, _x.jerk_ref.x, _x.jerk_ref.y, _x.jerk_ref.z, _x.snap_ref.x, _x.snap_ref.y, _x.snap_ref.z))
      buff.write(self.yaw_ref.tostring())
      _x = self
      buff.write(_get_struct_BdB().pack(_x.launch_flag, _x.speed, _x.collide_state))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.position_ref is None:
        self.position_ref = geometry_msgs.msg.Vector3()
      if self.velocity_ref is None:
        self.velocity_ref = geometry_msgs.msg.Vector3()
      if self.accel_ref is None:
        self.accel_ref = geometry_msgs.msg.Vector3()
      if self.jerk_ref is None:
        self.jerk_ref = geometry_msgs.msg.Vector3()
      if self.snap_ref is None:
        self.snap_ref = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 120
      (_x.position_ref.x, _x.position_ref.y, _x.position_ref.z, _x.velocity_ref.x, _x.velocity_ref.y, _x.velocity_ref.z, _x.accel_ref.x, _x.accel_ref.y, _x.accel_ref.z, _x.jerk_ref.x, _x.jerk_ref.y, _x.jerk_ref.z, _x.snap_ref.x, _x.snap_ref.y, _x.snap_ref.z,) = _get_struct_15d().unpack(str[start:end])
      start = end
      end += 24
      self.yaw_ref = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      _x = self
      start = end
      end += 10
      (_x.launch_flag, _x.speed, _x.collide_state,) = _get_struct_BdB().unpack(str[start:end])
      self.launch_flag = bool(self.launch_flag)
      self.collide_state = bool(self.collide_state)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_BdB = None
def _get_struct_BdB():
    global _struct_BdB
    if _struct_BdB is None:
        _struct_BdB = struct.Struct("<BdB")
    return _struct_BdB
_struct_15d = None
def _get_struct_15d():
    global _struct_15d
    if _struct_15d is None:
        _struct_15d = struct.Struct("<15d")
    return _struct_15d
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
