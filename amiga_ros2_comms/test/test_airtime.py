#!/usr/bin/env python3
"""Unit tests for the LoRa time-on-air model. No ROS, no serial port."""

import pytest

from amiga_ros2_comms.lora.airtime import (
    DWELL_LIMIT_SEC,
    RadioConfig,
    airtime_sec,
    low_data_rate_optimize,
    max_payload_for_dwell,
    symbol_time_sec,
)

SF7_125K = RadioConfig(spreading_factor=7, bandwidth_hz=125_000)
SF10_125K = RadioConfig(spreading_factor=10, bandwidth_hz=125_000)
SF12_125K = RadioConfig(spreading_factor=12, bandwidth_hz=125_000)


def test_symbol_time_matches_the_definition():
    assert symbol_time_sec(SF7_125K) == pytest.approx(1.024e-3)
    assert symbol_time_sec(SF12_125K) == pytest.approx(32.768e-3)


def test_airtime_matches_the_published_reference_case():
    # SF7/BW125/CR4-5, 8-symbol preamble, explicit header, CRC on, 10-byte
    # payload is the figure quoted everywhere as 41.2 ms. If this drifts, the
    # formula is wrong, not the calculator.
    assert airtime_sec(10, SF7_125K) == pytest.approx(41.216e-3, rel=1e-6)


def test_low_data_rate_optimize_engages_only_where_it_must():
    # Mandatory once a symbol exceeds 16 ms, which at 125 kHz is SF11 and SF12.
    assert not low_data_rate_optimize(SF10_125K)
    assert not low_data_rate_optimize(RadioConfig(spreading_factor=11, bandwidth_hz=250_000))
    assert low_data_rate_optimize(SF12_125K)


def test_airtime_grows_with_payload_and_with_spreading_factor():
    assert airtime_sec(0, SF7_125K) < airtime_sec(50, SF7_125K) < airtime_sec(200, SF7_125K)
    assert airtime_sec(50, SF7_125K) < airtime_sec(50, SF10_125K) < airtime_sec(50, SF12_125K)


def test_the_dwell_limit_is_what_actually_caps_payload_size():
    """The constraint the frame contract flagged, now computed rather than guessed.

    200 bytes is comfortably legal at SF7 and wildly illegal at SF10, which is
    why max_payload_bytes cannot be pinned down until the firmware's spreading
    factor is known.
    """
    assert airtime_sec(200, SF7_125K) < DWELL_LIMIT_SEC
    assert airtime_sec(200, SF10_125K) > 4 * DWELL_LIMIT_SEC


@pytest.mark.parametrize("radio", [SF7_125K, SF10_125K, SF12_125K])
def test_max_payload_for_dwell_is_the_actual_boundary(radio):
    budget = max_payload_for_dwell(radio)
    if budget > 0:
        assert airtime_sec(budget, radio) <= DWELL_LIMIT_SEC
    if budget < 255:
        assert airtime_sec(budget + 1, radio) > DWELL_LIMIT_SEC


def test_sf7_leaves_room_for_the_bridge_default():
    # The bridge ships max_payload_bytes=200. That has to be legal at the
    # spreading factor we are assuming, or the default is a bug.
    assert max_payload_for_dwell(SF7_125K) >= 200


def test_config_rejects_settings_no_modem_would_accept():
    with pytest.raises(ValueError):
        RadioConfig(spreading_factor=13)
    with pytest.raises(ValueError):
        RadioConfig(coding_rate=4)  # the 4/N denominator, not N-4
    with pytest.raises(ValueError):
        RadioConfig(bandwidth_hz=0)
