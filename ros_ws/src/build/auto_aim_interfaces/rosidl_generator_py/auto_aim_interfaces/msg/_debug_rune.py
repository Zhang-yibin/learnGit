# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/DebugRune.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DebugRune(type):
    """Metaclass of message 'DebugRune'."""

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
                'auto_aim_interfaces.msg.DebugRune')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__debug_rune
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__debug_rune
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__debug_rune
            cls._TYPE_SUPPORT = module.type_support_msg__msg__debug_rune
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__debug_rune

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


class DebugRune(metaclass=Metaclass_DebugRune):
    """Message class 'DebugRune'."""

    __slots__ = [
        '_header',
        '_spin_speed',
        '_filter_speed',
        '_c_function',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'spin_speed': 'float',
        'filter_speed': 'double',
        'c_function': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.spin_speed = kwargs.get('spin_speed', float())
        self.filter_speed = kwargs.get('filter_speed', float())
        self.c_function = kwargs.get('c_function', float())

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
        if self.spin_speed != other.spin_speed:
            return False
        if self.filter_speed != other.filter_speed:
            return False
        if self.c_function != other.c_function:
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
    def spin_speed(self):
        """Message field 'spin_speed'."""
        return self._spin_speed

    @spin_speed.setter
    def spin_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'spin_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'spin_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._spin_speed = value

    @builtins.property
    def filter_speed(self):
        """Message field 'filter_speed'."""
        return self._filter_speed

    @filter_speed.setter
    def filter_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'filter_speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'filter_speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._filter_speed = value

    @builtins.property
    def c_function(self):
        """Message field 'c_function'."""
        return self._c_function

    @c_function.setter
    def c_function(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'c_function' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'c_function' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._c_function = value
