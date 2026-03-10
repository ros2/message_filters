# Copyright 2026, Open Source Robotics Foundation, Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import unittest

from message_filters import Cache
from message_filters.delta_filter import (
    CachedComparisonHandler,
    ComparisonFilter,
    DeltaCompare,
    DeltaFilter,
    PathDeltaFilter,
    SimpleFilter,
)

from rclpy.type_support import MsgT

from std_msgs.msg import Bool, ColorRGBA, Int32, String


class TestDeltaCompareClasses(unittest.TestCase):

    def test_string_delta_compare_success(self):
        delta_compare = DeltaCompare(
            field_getters=[lambda message: message.data]
        )
        msg_1 = String(data='a')
        msg_2 = String(data='b')
        assert delta_compare.message_fits(msg_1)
        assert delta_compare.message_fits(msg_2)

    def test_string_delta_compare_failure(self):
        delta_compare = DeltaCompare(
            field_getters=[lambda message: message.data]
        )
        msg_1 = String(data='a')
        msg_2 = String(data='a')
        assert delta_compare.message_fits(msg_1)
        assert not delta_compare.message_fits(msg_2)

    def test_bool_delta_compare_success(self):
        delta_compare = DeltaCompare(
            field_getters=[lambda message: message.data]
        )
        msg_1 = Bool(data=True)
        msg_2 = Bool(data=False)
        assert delta_compare.message_fits(msg_1)
        assert delta_compare.message_fits(msg_2)

    def test_bool_delta_compare_failure(self):
        delta_compare = DeltaCompare(
            field_getters=[lambda message: message.data]
        )
        msg_1 = Bool(data=True)
        msg_2 = Bool(data=True)
        assert delta_compare.message_fits(msg_1)
        assert not delta_compare.message_fits(msg_2)

    def test_int32_delta_compare_success(self):
        delta_compare = DeltaCompare(
            field_getters=[lambda message: message.data]
        )
        msg_1 = Int32(data=10)
        msg_2 = Int32(data=11)
        assert delta_compare.message_fits(msg_1)
        assert delta_compare.message_fits(msg_2)

    def test_int32_delta_compare_failure(self):
        delta_compare = DeltaCompare(
            field_getters=[lambda message: message.data]
        )
        msg_1 = Int32(data=10)
        msg_2 = Int32(data=10)
        assert delta_compare.message_fits(msg_1)
        assert not delta_compare.message_fits(msg_2)


class TestComparisonFilter(unittest.TestCase):

    class ComparisonHandlerMock(CachedComparisonHandler):

        def __init__(self):
            super().__init__()
            self._compare_result: bool = True

        def message_fits(self, message: MsgT) -> bool:
            return self._compare_result

        def set_compare_to(self, res: bool):
            self._compare_result = res

    def test_comparison_filter_compare_messages(self):
        comparison_handler = self.ComparisonHandlerMock()

        comparison_filter = ComparisonFilter(
            comparison_handler=comparison_handler,
            message_filter=SimpleFilter()
        )

        cache_filter = Cache(f=comparison_filter, allow_headerless=True)

        self.assertEqual(cache_filter.cache_msgs, [])

        msg = Int32(data=10)
        comparison_filter.add(msg)

        self.assertEqual(cache_filter.cache_msgs, [msg])
        comparison_handler.set_compare_to(False)

        comparison_filter.add(msg)
        self.assertEqual(cache_filter.cache_msgs, [msg])


class TestDeltaFilter(unittest.TestCase):

    def test_delta_filter_str(self):
        delta_filter = DeltaFilter(
            field_getters=[lambda msg: msg.data]
        )
        cache_filter = Cache(f=delta_filter, allow_headerless=True)
        self.assertEqual(cache_filter.cache_msgs, [])

        msg_1 = String(data='a')
        msg_2 = String(data='b')

        delta_filter.add(msg_1)
        self.assertEqual(cache_filter.cache_msgs, [msg_1])

        delta_filter.add(msg_2)
        self.assertEqual(cache_filter.cache_msgs, [msg_2])

    def test_multiple_field_messages(self):
        delta_filter = DeltaFilter(
            field_getters=[
                lambda msg: msg.r,
                lambda msg: msg.g,
                lambda msg: msg.b,
                lambda msg: msg.a,
            ]
        )
        cache_filter = Cache(f=delta_filter, allow_headerless=True)
        self.assertEqual(cache_filter.cache_msgs, [])

        msg_1 = ColorRGBA(
            r=0,
            g=0,
            b=0,
            a=0
        )
        msg_2 = ColorRGBA(
            r=1,
            g=0,
            b=0,
            a=0
        )

        delta_filter.add(msg_1)
        self.assertEqual(cache_filter.cache_msgs, [msg_1])

        delta_filter.add(msg_2)
        self.assertEqual(cache_filter.cache_msgs, [msg_2])


class TestPathDeltaFilter(unittest.TestCase):

    def test_path_delta_filter(self):
        path_delta_filter = PathDeltaFilter(
            field_path_list=[
                'r',
                'g',
                'b',
                'a',
            ]
        )
        cache_filter = Cache(f=path_delta_filter, allow_headerless=True)
        self.assertEqual(cache_filter.cache_msgs, [])

        msg_1 = ColorRGBA(
            r=0,
            g=0,
            b=0,
            a=0
        )
        msg_2 = ColorRGBA(
            r=1,
            g=0,
            b=0,
            a=0
        )

        path_delta_filter.add(msg_1)
        self.assertEqual(cache_filter.cache_msgs, [msg_1])

        path_delta_filter.add(msg_2)
        self.assertEqual(cache_filter.cache_msgs, [msg_2])


if __name__ == '__main__':
    suite = unittest.TestSuite()

    # Test DeltaCompare comparison handler with basic data types
    suite.addTest(TestDeltaCompareClasses('test_string_delta_compare_success'))
    suite.addTest(TestDeltaCompareClasses('test_string_delta_compare_failure'))
    suite.addTest(TestDeltaCompareClasses('test_bool_delta_compare_success'))
    suite.addTest(TestDeltaCompareClasses('test_bool_delta_compare_failure'))
    suite.addTest(TestDeltaCompareClasses('test_int32_delta_compare_success'))
    suite.addTest(TestDeltaCompareClasses('test_int32_delta_compare_failure'))

    # Test ComparisonFilter
    suite.addTest(TestComparisonFilter('test_comparison_filter_compare_messages'))

    # Test DeltaFilter
    suite.addTest(TestDeltaFilter('test_delta_filter_str'))
    suite.addTest(TestDeltaFilter('test_multiple_field_messages'))

    # Test PathDeltaFilter
    suite.addTest(TestPathDeltaFilter('test_path_delta_filter'))

    unittest.TextTestRunner(verbosity=2).run(suite)
