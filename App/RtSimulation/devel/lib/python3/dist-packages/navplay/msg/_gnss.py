# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from navplay/gnss.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class gnss(genpy.Message):
  _md5sum = "d8b3bc143a1901908ef5f7a6d985bd0b"
  _type = "navplay/gnss"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float64 gpst
float64 lat
float64 lon
float32 h
float32[3] pos_std
uint8 mode
uint8 ns"""
  __slots__ = ['gpst','lat','lon','h','pos_std','mode','ns']
  _slot_types = ['float64','float64','float64','float32','float32[3]','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       gpst,lat,lon,h,pos_std,mode,ns

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(gnss, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.gpst is None:
        self.gpst = 0.
      if self.lat is None:
        self.lat = 0.
      if self.lon is None:
        self.lon = 0.
      if self.h is None:
        self.h = 0.
      if self.pos_std is None:
        self.pos_std = [0.] * 3
      if self.mode is None:
        self.mode = 0
      if self.ns is None:
        self.ns = 0
    else:
      self.gpst = 0.
      self.lat = 0.
      self.lon = 0.
      self.h = 0.
      self.pos_std = [0.] * 3
      self.mode = 0
      self.ns = 0

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
      buff.write(_get_struct_3df().pack(_x.gpst, _x.lat, _x.lon, _x.h))
      buff.write(_get_struct_3f().pack(*self.pos_std))
      _x = self
      buff.write(_get_struct_2B().pack(_x.mode, _x.ns))
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
      _x = self
      start = end
      end += 28
      (_x.gpst, _x.lat, _x.lon, _x.h,) = _get_struct_3df().unpack(str[start:end])
      start = end
      end += 12
      self.pos_std = _get_struct_3f().unpack(str[start:end])
      _x = self
      start = end
      end += 2
      (_x.mode, _x.ns,) = _get_struct_2B().unpack(str[start:end])
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
      _x = self
      buff.write(_get_struct_3df().pack(_x.gpst, _x.lat, _x.lon, _x.h))
      buff.write(self.pos_std.tostring())
      _x = self
      buff.write(_get_struct_2B().pack(_x.mode, _x.ns))
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
      _x = self
      start = end
      end += 28
      (_x.gpst, _x.lat, _x.lon, _x.h,) = _get_struct_3df().unpack(str[start:end])
      start = end
      end += 12
      self.pos_std = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      _x = self
      start = end
      end += 2
      (_x.mode, _x.ns,) = _get_struct_2B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2B = None
def _get_struct_2B():
    global _struct_2B
    if _struct_2B is None:
        _struct_2B = struct.Struct("<2B")
    return _struct_2B
_struct_3df = None
def _get_struct_3df():
    global _struct_3df
    if _struct_3df is None:
        _struct_3df = struct.Struct("<3df")
    return _struct_3df
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f