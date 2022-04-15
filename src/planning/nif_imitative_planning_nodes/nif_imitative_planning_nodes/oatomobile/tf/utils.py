# Copyright 2020 The OATomobile Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Utilities for nested data structures involving NumPy and TensorFlow 2.X."""

import tensorflow as tf
import tree

from oatomobile.tf import types


def add_batch_dim(nest: types.NestedArray) -> types.NestedTensor:
  """Adds a batch dimension to each leaf of a nested structure of Tensors."""
  return tree.map_structure(lambda x: tf.expand_dims(x, axis=0), nest)


def squeeze_batch_dim(nest: types.NestedTensor) -> types.NestedTensor:
  """Squeezes out a batch dimension from each leaf of a nested structure."""
  return tree.map_structure(lambda x: tf.squeeze(x, axis=0), nest)
