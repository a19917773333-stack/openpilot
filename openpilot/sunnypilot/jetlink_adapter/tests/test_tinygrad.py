"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

jetlink's small-model reset on this fork's model states.

Jetlink 0.8.7 changed what a model state keeps its history in: stock modeld
and a modeld_v2 native bundle use state_pairs plus one packed host buffer,
while a modeld_v2 legacy bundle keeps queues and numpy_inputs on its adapter.
prepare_reset has to clear exactly those, in place, so a fallback never
allocates or compiles on the failure frame.
"""
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from tinygrad import Tensor

from jetlink.openpilot.warp import prepare_reset

from openpilot.common.test import OpenpilotTestCase

ROOT = Path(__file__).resolve().parents[4]


def _queue(shape=(4, 8)) -> Tensor:
  return Tensor.zeros(*shape, device='CPU').contiguous().realize()


class TestPrepareReset(OpenpilotTestCase):
  def test_stock_modeld_resets_state_pairs_and_its_packed_input(self):
    packed = np.ones(16, dtype=np.float32)
    model = SimpleNamespace(
      input_queues={name: _queue() for name in ('img_q', 'big_img_q', 'desire_q')},
      state_pairs={'img_q': 'next_img_q', 'big_img_q': 'next_big_img_q', 'desire_q': 'next_desire_q'},
      packed_input=packed,
      prev_desire=np.ones(8),
    )
    identities = {name: id(queue) for name, queue in model.input_queues.items()}
    reset = prepare_reset(model)

    for queue in model.input_queues.values():
      queue.assign(7).realize()
    model.prev_desire.fill(1)
    packed.fill(1)
    reset()

    self.assertEqual({name: id(queue) for name, queue in model.input_queues.items()}, identities)
    for queue in model.input_queues.values():
      np.testing.assert_array_equal(queue.numpy(), 0)
    np.testing.assert_array_equal(model.prev_desire, 0)
    np.testing.assert_array_equal(packed, 0)

  def test_modeld_v2_legacy_bundle_resets_its_adapter(self):
    packed = np.ones(16, dtype=np.float32)
    adapter = SimpleNamespace(
      is_native=False,
      input_queues={name: _queue() for name in ('img_q', 'big_img_q', 'desire_q')},
      numpy_inputs={'desire': packed[:8], 'prev_desired_curv': packed[8:]},
    )
    model = SimpleNamespace(adapter=adapter, prev_desire=np.ones(8))
    reset = prepare_reset(model)

    for queue in adapter.input_queues.values():
      queue.assign(7).realize()
    model.prev_desire.fill(1)
    packed.fill(1)
    reset()

    for queue in adapter.input_queues.values():
      np.testing.assert_array_equal(queue.numpy(), 0)
    np.testing.assert_array_equal(model.prev_desire, 0)
    np.testing.assert_array_equal(packed, 0)

  def test_modeld_v2_native_bundle_resets_its_adapter(self):
    packed = np.ones(16, dtype=np.float32)
    adapter = SimpleNamespace(
      is_native=True,
      input_queues={name: _queue() for name in ('img_q', 'big_img_q', 'desire_q')},
      state_pairs={'img_q': 'next_img_q', 'big_img_q': 'next_big_img_q', 'desire_q': 'next_desire_q'},
      packed_input=packed,
    )
    model = SimpleNamespace(adapter=adapter, prev_desire=np.ones(8))
    reset = prepare_reset(model)

    for queue in adapter.input_queues.values():
      queue.assign(7).realize()
    model.prev_desire.fill(1)
    packed.fill(1)
    reset()

    for queue in adapter.input_queues.values():
      np.testing.assert_array_equal(queue.numpy(), 0)
    np.testing.assert_array_equal(model.prev_desire, 0)
    np.testing.assert_array_equal(packed, 0)

  def test_it_reads_what_openpilots_model_states_have(self):
    stock = _assigned_attrs(ROOT / 'openpilot/selfdrive/modeld/modeld.py')
    self.assertLessEqual({'input_queues', 'state_pairs', 'packed_input', 'prev_desire'}, stock)

    v2 = _assigned_attrs(ROOT / 'openpilot/sunnypilot/modeld_v2/modeld.py')
    self.assertLessEqual({'adapter', 'numpy_inputs', 'prev_desire'}, v2)


def _assigned_attrs(path: Path) -> set[str]:
  tree = ast.parse(path.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ModelState')
  return {n.attr for n in ast.walk(cls) if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store)
          and isinstance(n.value, ast.Name) and n.value.id == 'self'}


if __name__ == '__main__':
  unittest.main()
