#!/usr/bin/env python3

"""
# Pretty printing of Lanelet2 data (for gdb, usable with e.g. CLion)

## Installation
$ gedit ~/.gdbinit # edit file and paste everything between <code> and </code>:
<code>
python
sys.path.insert(0, "/path/to/res/folder")
import sys
import os
import lanelet_gdb
lanelet_gdb.register_printers()
</code>
Finally, correct the absolute path in the sys.path.insert line to the
actual location.

Similar plugin for Eigen objects: https://github.com/nspo/drake_gdb
"""

import gdb


class LaneletPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        return "Lanelet"

    def children(self):
        return self._iterator(self.val)

    class _iterator:
        def __init__(self, val):
            self.val = val
            self.count = 0

        def __iter__(self):
            return self

        def __next__(self):
            self.count = self.count + 1

            data = self.val["constData_"]["_M_ptr"].dereference()

            datadict = {
                1: ("id", data["id"]),
                2: ("attributes", data["attributes"]),
                3: ("leftBound", data["leftBound_"]),
                4: ("rightBound", data["rightBound_"]),
                5: ("regulatoryElements", data["regulatoryElements_"]),
                6: ("centerline", data["centerline_"]),
                7: ("inverted", self.val["inverted_"]),
            }

            try:
                return datadict[self.count]
            except KeyError:
                raise StopIteration


class Point3dPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        parr = self.val["constData_"]["_M_ptr"].dereference()["point"]['m_storage']['m_data']['array']
        return "Point3d (x={}, y={}, z={})".format(parr[0], parr[1], parr[2])

    def children(self):
        return self._iterator(self.val)

    class _iterator:
        def __init__(self, val):
            self.val = val
            self.count = 0

        def __iter__(self):
            return self

        def __next__(self):
            self.count = self.count + 1

            data = self.val["constData_"]["_M_ptr"].dereference()
            p = data["point"]
            parr = p['m_storage']['m_data']['array']

            datadict = {
                1: ("x", parr[0]),
                2: ("y", parr[1]),
                3: ("z", parr[2]),
                4: ("id", data["id"]),
                5: ("attributes", data["attributes"]),
                6: ("point2d_ (x/y)", data["point2d_"]),
            }

            try:
                return datadict[self.count]
            except KeyError:
                raise StopIteration


class LineString3dPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        return "LineString3d"

    def children(self):
        return self._iterator(self.val)

    class _iterator:
        def __init__(self, val):
            self.val = val
            self.count = 0

        def __iter__(self):
            return self

        def __next__(self):
            self.count = self.count + 1

            data = self.val["constData_"]["_M_ptr"].dereference()
            datadict = {
                1: ("id", data["id"]),
                2: ("inverted", self.val["inverted_"]),
                3: ("attributes", data["attributes"]),
                4: ("points", data["points_"]),
            }

            try:
                return datadict[self.count]
            except KeyError:
                raise StopIteration


class AttributePrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        return "Attribute"

    def children(self):
        return self._iterator(self.val)

    class _iterator:
        def __init__(self, val):
            self.val = val
            self.count = 0

        def __iter__(self):
            return self

        def __next__(self):
            self.count = self.count + 1

            datadict = {
                1: ("cache_", self.val["cache_"]),
                2: ("value_", self.val["value_"]),
            }

            try:
                return datadict[self.count]
            except KeyError:
                raise StopIteration


class RegulatoryElementPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        return "RegulatoryElement"

    def children(self):
        return self._iterator(self.val)

    class _iterator:
        def __init__(self, val):
            self.val = val
            self.count = 0

        def __iter__(self):
            return self

        def __next__(self):
            self.count = self.count + 1
            data = self.val["constData_"]["_M_ptr"].dereference()
            datadict = {
                1: ("id", data["id"]),
                2: ("attributes", data["attributes"]),
                3: ("parameters", data["parameters"]),
            }

            try:
                return datadict[self.count]
            except KeyError:
                raise StopIteration


def lookup_type(val):
    type_and_mod = str(val.type).split(" ")  # type and modifiers

    if 'lanelet::Lanelet' in type_and_mod or 'lanelet::ConstLanelet' in type_and_mod:
        return LaneletPrinter(val)
    elif 'lanelet::Point3d' in type_and_mod or 'lanelet::ConstPoint3d' in type_and_mod or \
            'lanelet::Point2d' in type_and_mod or 'lanelet::ConstPoint2d' in type_and_mod:
        return Point3dPrinter(val)
    elif 'lanelet::LineString3d' in type_and_mod or 'lanelet::ConstLineString3d' in type_and_mod or \
            'lanelet::LineString2d' in type_and_mod or 'lanelet::ConstLineString2d' in type_and_mod or \
            'lanelet::Polygon3d' in type_and_mod or 'lanelet::ConstPolygon3d' in type_and_mod or \
            'lanelet::Polygon2d' in type_and_mod or 'lanelet::ConstPolygon2d' in type_and_mod:
        return LineString3dPrinter(val)
    elif 'lanelet::Attribute' in type_and_mod:
        return AttributePrinter(val)
    elif 'lanelet::RegulatoryElement' in type_and_mod:
        return RegulatoryElementPrinter(val)
    return None


def register_printers():
    # as lambda so additional parameters are possible, e.g. for_clion for special CLion formatting
    gdb.pretty_printers.append(lambda val: lookup_type(val))
