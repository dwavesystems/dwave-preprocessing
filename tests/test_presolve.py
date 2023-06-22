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

import concurrent.futures
import os.path
import unittest

import dimod
import numpy as np

from dwave.preprocessing import Presolver, Feasibility, InvalidModelError

try:
    NUM_CPUS = len(os.sched_getaffinity(0))
except AttributeError:
    # windows
    NUM_CPUS = os.cpu_count()


class TestApply(unittest.TestCase):
    # Developer note: most of the testing for normalization/presolve is done at the C++
    # level. These tests are mostly testing the Cython/Python interface.

    def test_invalid_model(self):
        cqm = dimod.CQM()
        i = cqm.add_variable("INTEGER")
        cqm.objective.set_linear(i, float("nan"))

        presolver = Presolver(cqm)

        with self.assertRaises(InvalidModelError):
            presolver.apply()


class TestNormalization(unittest.TestCase):
    # Developer note: most of the testing for normalization is done at the C++
    # level. These tests are mostly testing the Cython/Python interface.

    @unittest.skipIf(NUM_CPUS < 4, "insufficient CPUs available")
    def test_concurrency(self):
        # obviously this is difficult to test fully, but we can at least run some
        # smoke tests to try to detect deadlocks and corrupted models

        cqm = dimod.ConstrainedQuadraticModel()
        cqm.add_variables("BINARY", 1000)
        for _ in range(1000):
            # these will be flipped, so if we're modifying the same model in
            # several threads at the same time we should see some issues
            cqm.add_constraint(((v, 1) for v in range(1000)), '>=', 1)

        presolver = Presolver(cqm)

        num_threads = 4

        with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
            concurrent.futures.wait(
                [executor.submit(presolver.normalize) for i in range(num_threads)])

        cqm = presolver.detach_model()

        self.assertEqual(cqm.num_variables(), 1000)
        self.assertEqual(cqm.num_constraints(), 1000)

        linear = {v: -1 for v in range(1000)}
        for comp in cqm.constraints.values():
            self.assertEqual(comp.rhs, -1)
            self.assertEqual(comp.sense, dimod.sym.Sense.Le)
            self.assertEqual(comp.lhs.linear, linear)

    def test_empty(self):
        presolver = Presolver(dimod.ConstrainedQuadraticModel())

        self.assertFalse(presolver.normalize())  # no changes made

    def test_ge(self):
        cqm = dimod.ConstrainedQuadraticModel()
        i, j = dimod.Integers("ij")
        c = cqm.add_constraint(i + j >= 5)

        presolver = Presolver(cqm)

        self.assertTrue(presolver.normalize())  # constraint will be flipped
        self.assertFalse(presolver.normalize())  # no changes, already normalized

    def test_invalid_model(self):
        cqm = dimod.CQM()
        i = cqm.add_variable("INTEGER")
        cqm.objective.set_linear(i, float("nan"))

        presolver = Presolver(cqm)

        with self.assertRaises(InvalidModelError):
            presolver.normalize()


class TestPresolve(unittest.TestCase):
    # Developer note: most of the testing for presolve is done at the C++
    # level. These tests are mostly testing the Cython/Python interface.

    @unittest.skipIf(NUM_CPUS < 4, "insufficient CPUs available")
    def test_concurrency(self):
        # obviously this is difficult to test fully, but we can at least run some
        # smoke tests to try to detect deadlocks and corrupted models

        cqm = dimod.ConstrainedQuadraticModel()
        cqm.add_variables("INTEGER", 100)
        for i in range(100):
            # these will be presolved, so if we're modifying the same model in
            # several threads at the same time we should see some issues
            cqm.add_constraint(((v, 1) for v in range(i, 100)), '==', 1)

        presolver = Presolver(cqm)
        presolver.load_default_presolvers()
        presolver.normalize()

        num_threads = 4

        with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
            jobs = [executor.submit(presolver.presolve) for i in range(num_threads)]

        # presolve should only make any changes once
        self.assertEqual(sum(future.result() for future in jobs), 1)

        cqm = presolver.detach_model()

        self.assertEqual(cqm.num_variables(), 0)
        self.assertEqual(cqm.num_constraints(), 0)

    def test_empty(self):
        presolver = Presolver(dimod.ConstrainedQuadraticModel())

        self.assertFalse(presolver.normalize())  # no changes made
        self.assertFalse(presolver.presolve())  # no changes made

    def test_normalized(self):
        cqm = dimod.ConstrainedQuadraticModel()
        i, j = dimod.Integers("ij")
        c = cqm.add_constraint(i + j >= 100)

        presolver = Presolver(cqm)

        self.assertTrue(presolver.normalize())  # constraint will be flipped
        self.assertFalse(presolver.presolve())  # no changes made

    def test_not_normalized(self):
        cqm = dimod.ConstrainedQuadraticModel()
        i, j = dimod.Integers("ij")
        c = cqm.add_constraint(i + j >= 100)

        presolver = Presolver(cqm)

        with self.assertRaises(TypeError):
            self.assertFalse(presolver.presolve())


# Todo: reorganize these tests into individual classes for specific methods
# like the above
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

    def test_bug56(self):
        # https://github.com/dwavesystems/dwave-preprocessing/issues/56
        for fname in ['bug56.0.cqm', 'bug56.1.cqm']:
            with self.subTest(fname):
                with open(os.path.join(os.path.dirname(__file__), 'data', fname), 'rb') as f:
                    cqm = dimod.CQM.from_file(f)

                presolver = Presolver(cqm)
                presolver.load_default_presolvers()
                presolver.apply()

                cqm = presolver.detach_model()

                for v in cqm.variables:
                    self.assertNotEqual(cqm.lower_bound(v), cqm.upper_bound(v))

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

        with self.assertRaises(InvalidModelError):
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
            presolver.apply()
            self.assertIs(presolver.feasibility(), Feasibility.Infeasible)

        with self.subTest("infeas =="):
            cqm = dimod.ConstrainedQuadraticModel()
            i = dimod.Integer('i')
            cqm.add_constraint(i == 1)
            cqm.fix_variable('i', 2)

            presolver = Presolver(cqm)
            presolver.load_default_presolvers()
            presolver.apply()
            self.assertIs(presolver.feasibility(), Feasibility.Infeasible)

        with self.subTest("infeas >="):
            cqm = dimod.ConstrainedQuadraticModel()
            i = dimod.Integer('i')
            cqm.add_constraint(i >= 1)
            cqm.fix_variable('i', -1)

            presolver = Presolver(cqm)
            presolver.load_default_presolvers()
            presolver.apply()
            self.assertIs(presolver.feasibility(), Feasibility.Infeasible)

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

    def test_zero_biases(self):
        i, j = dimod.Integers('ij')
        cqm = dimod.ConstrainedQuadraticModel()

        cqm.add_constraint(0*i >= -5)

        cqm.set_lower_bound('i', -5)
        cqm.set_upper_bound('i', +5)

        presolver = Presolver(cqm)
        presolver.load_default_presolvers()
        presolver.apply()

        cqm = presolver.detach_model()

        self.assertEqual(cqm.lower_bound(0), -5)
        self.assertEqual(cqm.upper_bound(0), +5)
        self.assertEqual(cqm.num_constraints(), 0)

    def test_domain_propagation(self):
        i, j, k, l, m, n, o, p = dimod.Reals('ijklmnop')

        cqm = dimod.ConstrainedQuadraticModel()
        cqm.set_objective(i + j + k + l + m + n + o + p)
        cqm.add_constraint(i + 2 * j + k + l <= 100)
        cqm.add_constraint(2 * i + j + 2 * k + 5 * l <= 200)
        cqm.add_constraint(m - 2 * n >= 100)
        cqm.add_constraint(-m + 2 * n >= 100)
        cqm.add_constraint(o - 0.5 * p == 0)
        cqm.add_constraint(0.5 * o - p == 0)

        cqm.set_lower_bound('o', 0)
        cqm.set_upper_bound('o', 1)
        cqm.set_lower_bound('p', 0)
        cqm.set_upper_bound('p', 1)

        presolver = Presolver(cqm)
        presolver.load_default_presolvers()
        presolver.apply()
        updated_cqm = presolver.copy_model()

        self.assertEqual(updated_cqm.lower_bound(0), 0)
        self.assertEqual(updated_cqm.upper_bound(0), 100)
        self.assertEqual(updated_cqm.lower_bound(1), 0)
        self.assertEqual(updated_cqm.upper_bound(1), 50)
        self.assertEqual(updated_cqm.lower_bound(2), 0)
        self.assertEqual(updated_cqm.upper_bound(2), 100)
        self.assertEqual(updated_cqm.lower_bound(3), 0)
        self.assertEqual(updated_cqm.upper_bound(3), 40)
        self.assertEqual(updated_cqm.lower_bound(4), 100 + 99 * 200)
        self.assertEqual(updated_cqm.upper_bound(4), 1e30)
        self.assertEqual(updated_cqm.lower_bound(5), 100 + 99 * 100)
        self.assertEqual(updated_cqm.upper_bound(5), 1e30)
        self.assertEqual(updated_cqm.lower_bound(6), 0)
        self.assertEqual(updated_cqm.upper_bound(6), 0.5**31)
        self.assertEqual(updated_cqm.lower_bound(7), 0)
        self.assertEqual(updated_cqm.upper_bound(7), 0.5**30)
