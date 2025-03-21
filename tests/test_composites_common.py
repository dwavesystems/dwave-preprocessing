# Copyright 2025 D-Wave
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
from unittest import mock

import dimod
from parameterized import parameterized_class

from dwave.preprocessing.composites import (
    ClipComposite,
    ConnectedComponentsComposite,
    FixVariablesComposite,
    ScaleComposite,
    SpinReversalTransformComposite,
)


@parameterized_class([
    dict(composite_cls=ClipComposite),
    dict(composite_cls=ConnectedComponentsComposite),
    dict(composite_cls=FixVariablesComposite),
    dict(composite_cls=ScaleComposite),
    dict(composite_cls=SpinReversalTransformComposite),
])
class TestScoped(unittest.TestCase):
    """Test all composites defined in dwave-preprocessing are properly scoped,
    i.e. they propagate `close()` to samplers and they implement the context
    manager protocol.
    """

    def get_composite(self) -> tuple[dimod.Sampler, dimod.Composite]:
        params = getattr(self, 'params', None)
        if params is None:
            params = {}

        sampler = getattr(self, 'sampler_cls', dimod.ExactSolver)()
        sampler.close = mock.MagicMock()

        composite = self.composite_cls(sampler, **params)

        return sampler, composite

    def test_close_propagation(self):
        sampler, composite = self.get_composite()

        composite.close()

        sampler.close.assert_called_once()

    def test_context_manager(self):
        sampler, composite = self.get_composite()

        with composite:
            ...

        sampler.close.assert_called_once()
