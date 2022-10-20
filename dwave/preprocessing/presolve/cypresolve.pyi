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

import dimod
import numpy as np


class cyPresolver:
    variables: dimod.variables.Variables

    def __init__(self, cqm: dimod.ConstrainedQuadraticModel, move: bool = ...): ...
    def apply(self) -> None: ...
    def load_default_presolvers(self) -> None: ...
    def clear_cqm(self) -> None: ...
    def restore_samples(self, samples_like: dimod.typing.SamplesLike) -> np.ndarray: ...
