# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from navplay/imu.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class imu(genpy.Message):
  _md5sum = "08a3930f07de2d670d1b2b4f3b2dd41c"
  _type = "navplay/imu"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64 gpst
float64[3] acce
float64[3] gyro"""
  __slots__ = ['gpst','acce','gyro']
  _slot_types = ['float64','float64[3]','float64[3]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       gpst,acce,gyro

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(imu, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.gpst is None:
        self.gpst = 0.
      if self.acce is None:
        self.acce = [0.] * 3
      if self.gyro is None:
        self.gyro = [0.] * 3
    else:
      self.gpst = 0.
      self.acce = [0.] * 3
      self.gyro = [0.] * 3

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
      _x = self.gpst
      buff.write(_get_struct_d().pack(_x))
      buff.write(_get_struct_3d().pack(*self.acce))
      buff.write(_get_struct_3d().pack(*self.gyro))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 8
      (self.gpst,) = _get_struct_d().unpack(str[start:end])
      start = end
      end += 24
      self.acce = _get_struct_3d().unpack(str[start:end])
      start = end
      end += 24
      self.gyro = _get_struct_3d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.gpst
      buff.write(_get_struct_d().pack(_x))
      buff.write(self.acce.tostring())
      buff.write(self.gyro.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 8
      (self.gpst,) = _get_struct_d().unpack(str[start:end])
      start = end
      end += 24
      self.acce = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      start = end
      end += 24
      self.gyro = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=3)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d
