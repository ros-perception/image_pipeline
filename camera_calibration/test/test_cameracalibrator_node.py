# Copyright (c) 2026, Igor Stadnyk
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following disclaimer
#     in the documentation and/or other materials provided with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

from camera_calibration.nodes import cameracalibrator
import pytest


@pytest.fixture(autouse=True)
def shutdown_rclpy_context():
    cameracalibrator.rclpy.try_shutdown()
    yield
    cameracalibrator.rclpy.try_shutdown()


def _run_main(monkeypatch, shutdown_during_spin):
    class FakeCalibrationNode:

        def __init__(self, *args, **kwargs):
            pass

        def spin(self):
            if shutdown_during_spin:
                cameracalibrator.rclpy.shutdown()

    monkeypatch.setattr(cameracalibrator, 'OpenCVCalibrationNode', FakeCalibrationNode)
    monkeypatch.setattr(sys, 'argv', ['cameracalibrator'])

    cameracalibrator.main()

    assert not cameracalibrator.rclpy.ok()


def test_main_does_not_shutdown_inactive_context(monkeypatch):
    _run_main(monkeypatch, shutdown_during_spin=True)


def test_main_shuts_down_active_context(monkeypatch):
    _run_main(monkeypatch, shutdown_during_spin=False)
