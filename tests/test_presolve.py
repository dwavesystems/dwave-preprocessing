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
    def test_copy_model(self):
        cqm = dimod.CQM()

        i, j = dimod.Integers('ij')

        cqm.add_variables('INTEGER', 'ij')
        cqm.add_constraint(i <= 5)
        cqm.add_constraint(i >= -5)
        cqm.add_constraint(j == 105)

        presolver = Presolver(cqm)

        presolver.load_default_presolvers()
        presolver.apply()

        model = presolver.copy_model()
        self.assertEqual(len(model.variables), 1)

        # and again, since it's just a copy
        model = presolver.copy_model()
        self.assertEqual(len(model.variables), 1)

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

        samplearray, labels = presolver.restore_samples([[0], [1]])
        np.testing.assert_array_equal(samplearray, [[0, 105], [1, 105]])
        self.assertEqual(labels, 'ij')

    def test_detach_model(self):
        cqm = dimod.CQM()

        i, j = dimod.Integers('ij')

        cqm.add_variables('INTEGER', 'ij')
        cqm.add_constraint(i <= 5)
        cqm.add_constraint(i >= -5)
        cqm.add_constraint(j == 105)

        presolver = Presolver(cqm)

        presolver.load_default_presolvers()
        presolver.apply()

        model = presolver.detach_model()
        self.assertEqual(len(model.variables), 1)

        with self.assertRaises(RuntimeError):
            presolver.apply()

    def test_move(self):
        cqm = dimod.CQM()

        i, j = dimod.Integers('ij')

        cqm.add_variables('INTEGER', 'ij')
        cqm.add_constraint(i <= 5)
        cqm.add_constraint(i >= -5)
        cqm.add_constraint(j == 105)

        presolver = Presolver(cqm, move=True)

        self.assertTrue(cqm.is_equal(dimod.CQM()))

    def test_self_loop(self):
        i = dimod.Integer("i")
        cqm = dimod.ConstrainedQuadraticModel()
        cqm.add_constraint(i * i <= 0)

        presolver = Presolver(cqm)
        presolver.load_default_presolvers()
        presolver.apply()
