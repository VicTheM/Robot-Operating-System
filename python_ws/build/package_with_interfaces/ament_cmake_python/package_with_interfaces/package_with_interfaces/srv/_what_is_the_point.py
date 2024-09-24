# generated from rosidl_generator_py/resource/_idl.py.em
# with input from package_with_interfaces:srv/WhatIsThePoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WhatIsThePoint_Request(type):
    """Metaclass of message 'WhatIsThePoint_Request'."""

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
            module = import_type_support('package_with_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'package_with_interfaces.srv.WhatIsThePoint_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__what_is_the_point__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__what_is_the_point__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__what_is_the_point__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__what_is_the_point__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__what_is_the_point__request

            from package_with_interfaces.msg import AmazingQuote
            if AmazingQuote.__class__._TYPE_SUPPORT is None:
                AmazingQuote.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WhatIsThePoint_Request(metaclass=Metaclass_WhatIsThePoint_Request):
    """Message class 'WhatIsThePoint_Request'."""

    __slots__ = [
        '_quote',
    ]

    _fields_and_field_types = {
        'quote': 'package_with_interfaces/AmazingQuote',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['package_with_interfaces', 'msg'], 'AmazingQuote'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from package_with_interfaces.msg import AmazingQuote
        self.quote = kwargs.get('quote', AmazingQuote())

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
        if self.quote != other.quote:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def quote(self):
        """Message field 'quote'."""
        return self._quote

    @quote.setter
    def quote(self, value):
        if __debug__:
            from package_with_interfaces.msg import AmazingQuote
            assert \
                isinstance(value, AmazingQuote), \
                "The 'quote' field must be a sub message of type 'AmazingQuote'"
        self._quote = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_WhatIsThePoint_Response(type):
    """Metaclass of message 'WhatIsThePoint_Response'."""

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
            module = import_type_support('package_with_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'package_with_interfaces.srv.WhatIsThePoint_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__what_is_the_point__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__what_is_the_point__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__what_is_the_point__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__what_is_the_point__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__what_is_the_point__response

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WhatIsThePoint_Response(metaclass=Metaclass_WhatIsThePoint_Response):
    """Message class 'WhatIsThePoint_Response'."""

    __slots__ = [
        '_point',
    ]

    _fields_and_field_types = {
        'point': 'geometry_msgs/Point',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Point
        self.point = kwargs.get('point', Point())

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
        if self.point != other.point:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def point(self):
        """Message field 'point'."""
        return self._point

    @point.setter
    def point(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'point' field must be a sub message of type 'Point'"
        self._point = value


class Metaclass_WhatIsThePoint(type):
    """Metaclass of service 'WhatIsThePoint'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('package_with_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'package_with_interfaces.srv.WhatIsThePoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__what_is_the_point

            from package_with_interfaces.srv import _what_is_the_point
            if _what_is_the_point.Metaclass_WhatIsThePoint_Request._TYPE_SUPPORT is None:
                _what_is_the_point.Metaclass_WhatIsThePoint_Request.__import_type_support__()
            if _what_is_the_point.Metaclass_WhatIsThePoint_Response._TYPE_SUPPORT is None:
                _what_is_the_point.Metaclass_WhatIsThePoint_Response.__import_type_support__()


class WhatIsThePoint(metaclass=Metaclass_WhatIsThePoint):
    from package_with_interfaces.srv._what_is_the_point import WhatIsThePoint_Request as Request
    from package_with_interfaces.srv._what_is_the_point import WhatIsThePoint_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
