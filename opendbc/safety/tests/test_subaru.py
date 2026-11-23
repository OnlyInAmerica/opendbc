#!/usr/bin/env python3
import enum
import unittest

from opendbc.car.subaru.values import SubaruSafetyFlags
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from functools import partial


class SubaruMsg(enum.IntEnum):
  Brake_Status      = 0x13c
  CruiseControl     = 0x240
  Throttle          = 0x40
  Steering_Torque   = 0x119
  Wheel_Speeds      = 0x13a
  Cruise_Status_2   = 0x27
  Throttle_Hybrid   = 0x168
  Brake_Hybrid      = 0x226
  ES_LKAS           = 0x122
  ES_Distance       = 0x221
  ES_DashStatus     = 0x321
  ES_LKAS_State     = 0x322
  ES_Infotainment   = 0x323


SUBARU_MAIN_BUS = 0
SUBARU_ALT_BUS  = 1
SUBARU_CAM_BUS  = 2


def lkas_tx_msgs(alt_bus):
  return [[SubaruMsg.ES_LKAS,           SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Distance,       alt_bus],
          [SubaruMsg.ES_DashStatus,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_LKAS_State,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Infotainment,   SUBARU_MAIN_BUS]]


def fwd_blacklisted_addr():
  return {SUBARU_CAM_BUS: [SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]}


class TestSubaruSafetyBase(common.CarSafetyTest):
  DBC = "subaru_global_2017_generated"
  SAFETY_MODEL = CarParams.SafetyModel.subaru

  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr()

  MAX_RT_DELTA = 940

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 50

  ALT_MAIN_BUS = SUBARU_MAIN_BUS

  INACTIVE_GAS = 1818

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("Steering_Torque", 0, values)

  def _speed_msg(self, speed):
    values = {s: speed for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_safety("Brake_Status", self.ALT_MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, values)


class TestSubaruStockLongitudinalSafetyBase(TestSubaruSafetyBase):
  def _cancel_msg(self, cancel, cruise_throttle=0):
    values = {"Cruise_Cancel": cancel, "Cruise_Throttle": cruise_throttle}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def test_cancel_message(self):
    # test that we can only send the cancel message (ES_Distance) with inactive throttle (1818) and Cruise_Cancel=1
    for cancel in [True, False]:
      self._generic_limit_safety_check(partial(self._cancel_msg, cancel), self.INACTIVE_GAS, self.INACTIVE_GAS, 0, 2**12, 1, self.INACTIVE_GAS, cancel)


class TestSubaruTorqueSafetyBase(TestSubaruSafetyBase, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE_LOOKUP = [0], [2047]

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 7
  MAX_INVALID_STEERING_FRAMES = 1
  STEER_STEP = 2

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Request": steer_req}
    return self.packer.make_can_msg_safety("ES_LKAS", SUBARU_MAIN_BUS, values)


class TestSubaruGen1TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS)


class TestSubaruGen1HybridTorqueSafety(TestSubaruTorqueSafetyBase):
  # Hybrids report driver gas, driver brake and the ACC state on the alt bus.
  # There is no ES_Distance, so no cancel message is allowed
  DBC = "subaru_global_2020_hybrid_generated"
  SAFETY_PARAM = SubaruSafetyFlags.HYBRID

  TX_MSGS = [m for m in lkas_tx_msgs(SUBARU_MAIN_BUS) if m[0] != SubaruMsg.ES_Distance]

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_safety("Brake_Hybrid", SUBARU_ALT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle_Hybrid", SUBARU_ALT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("Cruise_Status_2", SUBARU_ALT_BUS, values)

  def setUp(self):
    super().setUp()
    self.global_packer = CANPackerSafety("subaru_global_2017_generated")

  def _state(self):
    return self.safety.get_brake_pressed_prev(), self.safety.get_gas_pressed_prev(), self.safety.get_cruise_engaged_prev()

  def test_non_hybrid_signals_ignored(self):
    # Brake_Status and Throttle are also on the main bus of hybrids, but never show a driver press there
    for pressed in (True, False):
      self._rx(self._user_brake_msg(pressed))
      self._rx(self._user_gas_msg(pressed))
      self._rx(self._pcm_status_msg(pressed))
      for bus in (SUBARU_MAIN_BUS, SUBARU_ALT_BUS):
        self._rx(self.global_packer.make_can_msg_safety("Brake_Status", bus, {"Brake": not pressed}))
        self._rx(self.global_packer.make_can_msg_safety("Throttle", bus, {"Throttle_Pedal": not pressed}))
        self._rx(self.global_packer.make_can_msg_safety("CruiseControl", bus, {"Cruise_Activated": not pressed}))
        self.assertEqual((pressed, pressed, pressed), self._state())

  def test_hybrid_signals_bus(self):
    # the hybrid powertrain messages are only read from the alt bus
    for bus in (SUBARU_MAIN_BUS, SUBARU_CAM_BUS):
      self._rx(self.packer.make_can_msg_safety("Brake_Hybrid", bus, {"Brake": 1}))
      self._rx(self.packer.make_can_msg_safety("Throttle_Hybrid", bus, {"Throttle_Pedal": 1}))
      self._rx(self.packer.make_can_msg_safety("Cruise_Status_2", bus, {"Cruise_Activated": 1}))
      self.assertEqual((False, False, False), self._state())

  def test_cancel_message_blocked(self):
    values = {"Cruise_Cancel": 1, "Cruise_Throttle": self.INACTIVE_GAS}
    for bus in (SUBARU_MAIN_BUS, SUBARU_ALT_BUS, SUBARU_CAM_BUS):
      self.assertFalse(self._tx(self.global_packer.make_can_msg_safety("ES_Distance", bus, values)))


class TestSubaruGen2TorqueSafetyBase(TestSubaruTorqueSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS

  MAX_RATE_UP = 40
  MAX_RATE_DOWN = 40
  MAX_TORQUE_LOOKUP = [0], [1000]


class TestSubaruGen2TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  SAFETY_PARAM = SubaruSafetyFlags.GEN2

  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS)


if __name__ == "__main__":
  unittest.main()
