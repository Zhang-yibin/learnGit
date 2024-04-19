# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/Rune.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Rune(type):
    """Metaclass of message 'Rune'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('auto_aim_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'auto_aim_interfaces.msg.Rune')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rune
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rune
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rune
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rune
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rune

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Rune(metaclass=Metaclass_Rune):
    """Message class 'Rune'."""

    __slots__ = [
        '_header',
        '_rune_mode',
        '_r_center',
        '_target_center',
        '_real_angle',
        '_radian',
        '_rows',
        '_cols',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'rune_mode': 'int32',
        'r_center': 'geometry_msgs/Point',
        'target_center': 'geometry_msgs/Point',
        'real_angle': 'float',
        'radian': 'float',
        'rows': 'int32',
        'cols': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.rune_mode = kwargs.get('rune_mode', int())
        from geometry_msgs.msg import Point
        self.r_center = kwargs.get('r_center', Point())
        from geometry_msgs.msg import Point
        self.target_center = kwargs.get('target_center', Point())
        self.real_angle = kwargs.get('real_angle', float())
        self.radian = kwargs.get('radian', float())
        self.rows = kwargs.get('rows', int())
        self.cols = kwargs.get('cols', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.rune_mode != other.rune_mode:
            return False
        if self.r_center != other.r_center:
            return False
        if self.target_center != other.target_center:
            return False
        if self.real_angle != other.real_angle:
            return False
        if self.radian != other.radian:
            return False
        if self.rows != other.rows:
            return False
        if self.cols != other.cols:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def rune_mode(self):
        """Message field 'rune_mode'."""
        return self._rune_mode

    @rune_mode.setter
    def rune_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rune_mode' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rune_mode' field must be an integer in [-2147483648, 2147483647]"
        self._rune_mode = value

    @builtins.property
    def r_center(self):
        """Message field 'r_center'."""
        return self._r_center

    @r_center.setter
    def r_center(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'r_center' field must be a sub message of type 'Point'"
        self._r_center = value

    @builtins.property
    def target_center(self):
        """Message field 'target_center'."""
        return self._target_center

    @target_center.setter
    def target_center(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'target_center' field must be a sub message of type 'Point'"
        self._target_center = value

    @builtins.property
    def real_angle(self):
        """Message field 'real_angle'."""
        return self._real_angle

    @real_angle.setter
    def real_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'real_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'real_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._real_angle = value

    @builtins.property
    def radian(self):
        """Message field 'radian'."""
        return self._radian

    @radian.setter
    def radian(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'radian' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'radian' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._radian = value

    @builtins.property
    def rows(self):
        """Message field 'rows'."""
        return self._rows

    @rows.setter
    def rows(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rows' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rows' field must be an integer in [-2147483648, 2147483647]"
        self._rows = value

    @builtins.property
    def cols(self):
        """Message field 'cols'."""
        return self._cols

    @cols.setter
    def cols(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cols' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'cols' field must be an integer in [-2147483648, 2147483647]"
        self._cols = value
