# Copyright 2022 D-Wave Systems Inc.
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import unittest

import dimod
import numpy as np

from dwave.preprocessing import Presolver


class TestPresolver(unittest.TestCase):
    def test_cqm(self):
        cqm = dimod.CQM()

        i, j = dimod.Integers('ij')

        cqm.add_variables('INTEGER', 'ij')
        cqm.add_constraint(i <= 5)
        cqm.add_constraint(i >= -5)
        cqm.add_constraint(j == 105)

        presolver = Presolver(cqm)

        presolver.load_default_presolvers()
        presolver.apply()

        np.testing.assert_array_equal(presolver.restore_samples([[0], [1]]), [[0, 105], [1, 105]])

    def test_move(self):
        # todo
        pass

    def test_self_loop(self):
        i = dimod.Integer("i")
        cqm = dimod.ConstrainedQuadraticModel()
        cqm.add_constraint(i * i <= 0)

        presolver = Presolver(cqm)
        presolver.load_default_presolvers()
        presolver.apply()