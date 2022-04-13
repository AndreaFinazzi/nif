#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ================================================================================ #
from __future__ import absolute_import
from __future__ import print_function
__author__ = 'Henry A Leopold'


# ================================================================================ #
# Imports
# -------------------------------------------------------------------------------- #
# Builtins
import re
import copy
import os
from functools import wraps
from collections import OrderedDict
from collections import Callable
import json
import yaml

# ================================================================================ #
# Class
# -------------------------------------------------------------------------------- #

class AttrDict(dict):
    """ General attribute dictionary with some recursion for handling nested dictionaries.

    Args:
        dict ([type]): [description]

    """

    __setattr__ = dict.__setitem__
    __getattr__ = dict.__getitem__
    __delattr__ = dict.__delitem__

    def add(self, key, value):
        self.__setattr__(key, value)

    def remove(self, key):
        self.__delattr__(key)

    def filter(self, fn):
        d = self.__class__()
        for key, value in self.items():
            if fn(key, value):
                d.add(key, value)

    def copy(self):
        return self.__copy__()

    def __copy__(self):
        return self.__deepcopy__()

    def __deepcopy__(self):
        return self.__class__(copy.deepcopy(dict(self)))

    def __getnewargs__(self):
        return tuple(self.items())

    @classmethod
    def from_dict(cls, d):
        new_dict = cls()
        for key, value in d.items():
            if type(value) is dict:
                new_dict.add(key, cls.from_dict(value))
            else:
                new_dict.add(key, value)
        return new_dict
