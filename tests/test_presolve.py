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

from dwave.preprocessing import Presolver, InfeasibleModelError


class TestExperiment(unittest.TestCase):
    def test_parallel(self):
        cqm = dimod.CQM()
        for _ in range(100):
            cqm.add_constraint(dimod.BQM("BINARY") == 1)

        presolver = Presolver(cqm)
        self.assertLessEqual(presolver.apply_parallel(), 55)  # should take 100s


class TestPresolver(unittest.TestCase):
    def test_bug0(self):
        random = np.random.RandomState(0)
        cqm = dimod.ConstrainedQuadraticModel()

        dimod_vars = [
            dimod.Binary("a"),
            dimod.Spin("b"),
            dimod.Integer("c", lower_bound=-5000, upper_bound=5000),
            dimod.Real("z", lower_bound=-5000, upper_bound=5000),
        ]

        discrete_labels = "defghi"
        discrete = [dimod.Binary(c) for c in discrete_labels]
        dimod_vars.extend(discrete)

        cqm.add_discrete(discrete_labels)

        obj = dimod.QM()
        obj.add_variable(dimod.INTEGER, "c", lower_bound=-5000, upper_bound=5000)
        obj += sum(random.uniform() * u for u in dimod_vars)
        cqm.set_objective(obj.copy())

        presolver = Presolver(cqm, move=False)
        presolver.load_default_presolvers()
        presolver.apply()
        presolver.clear_model()

        with self.assertRaises(ValueError):
            presolver.restore_samples(np.array([[]]))

    def test_bug1(self):
        cqm = dimod.ConstrainedQuadraticModel()
        cqm.set_objective(dimod.Spin("f"))

        presolver = Presolver(cqm, move=False)
        presolver.load_default_presolvers()
        presolver.apply()
        presolver.clear_model()

        with self.assertRaises(ValueError):
            # wrong number of variables
            presolver.restore_samples(np.array([[]]))

        presolver.restore_samples(np.array([[0]]))

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

        presolver.load_default_presolvers()
        presolver.apply()

        self.assertEqual(presolver.copy_model().num_variables(), 1)

        samplearray, labels = presolver.restore_samples([[0], [1]])
        np.testing.assert_array_equal(samplearray, [[0, 105], [1, 105]])
        self.assertEqual(labels, 'ij')

    def test_no_variable_constraints(self):
        with self.subTest("feasible"):
            cqm = dimod.ConstrainedQuadraticModel()
            i = dimod.Integer('i')

            cqm.add_constraint(i <= 1)
            cqm.add_constraint(i >= -1)
            cqm.add_constraint(i == 0)
            cqm.fix_variable('i', 0)

            presolver = Presolver(cqm)
            presolver.load_default_presolvers()
            presolver.apply()

        with self.subTest("infeas <="):
            cqm = dimod.ConstrainedQuadraticModel()
            i = dimod.Integer('i')
            cqm.add_constraint(i <= 1)
            cqm.fix_variable('i', 2)

            presolver = Presolver(cqm)
            presolver.load_default_presolvers()
            with self.assertRaises(InfeasibleModelError):
                presolver.apply()

        with self.subTest("infeas =="):
            cqm = dimod.ConstrainedQuadraticModel()
            i = dimod.Integer('i')
            cqm.add_constraint(i == 1)
            cqm.fix_variable('i', 2)

            presolver = Presolver(cqm)
            presolver.load_default_presolvers()
            with self.assertRaises(InfeasibleModelError):
                presolver.apply()

        with self.subTest("infeas >="):
            cqm = dimod.ConstrainedQuadraticModel()
            i = dimod.Integer('i')
            cqm.add_constraint(i >= 1)
            cqm.fix_variable('i', -1)

            presolver = Presolver(cqm)
            presolver.load_default_presolvers()
            with self.assertRaises(InfeasibleModelError):
                presolver.apply()

    def test_self_loop(self):
        i = dimod.Integer("i")
        cqm = dimod.ConstrainedQuadraticModel()
        cqm.add_constraint(i * i <= 0)

        presolver = Presolver(cqm)
        presolver.load_default_presolvers()
        presolver.apply()

        reduced = presolver.detach_model()

        samples = [[reduced.lower_bound(v) for v in reduced.variables],
                   [reduced.upper_bound(v) for v in reduced.variables]]

        samplearray, labels = presolver.restore_samples(samples)

        self.assertEqual(samplearray.shape, (2, 1))
        self.assertEqual(labels, 'i')

    def test_variable_removal(self):
        v0, v1, v2, v3 = dimod.Binaries('wxyz')

        cqm = dimod.ConstrainedQuadraticModel()
        cqm.set_objective(v0 + v1 + v2 + v3)
        cqm.add_constraint(-3 + 7*v0 >= 3)
        cqm.add_constraint(-24 + 5*v0 + 12*v3 + 12*v1 + 19*v2 >= -4)
        cqm.add_constraint(6 - 12*v0 - 12*v2 + 25*v0*v2 >= 3)
        cqm.add_constraint(-5 + 10*v3 >= -1)

        presolver = Presolver(cqm)
        presolver.load_default_presolvers()
        presolver.apply()
