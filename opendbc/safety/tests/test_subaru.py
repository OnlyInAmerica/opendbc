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
  Brake_Switch      = 0x147
  BodyInfo          = 0x3ac


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
  # There is no ES_Distance, ACC is cancelled through the brake bits sent to the camera
  DBC = "subaru_global_2020_hybrid_generated"
  SAFETY_PARAM = SubaruSafetyFlags.HYBRID
  MAX_TORQUE_LOOKUP = [0], [3200]

  # Brake_Switch and BodyInfo are blocked from the camera and resent by openpilot, with the brake bit set to cancel ACC
  HYBRID_CAM_MSGS = (SubaruMsg.Brake_Switch, SubaruMsg.BodyInfo)
  TX_MSGS = [m for m in lkas_tx_msgs(SUBARU_MAIN_BUS) if m[0] != SubaruMsg.ES_Distance] + [[m, SUBARU_CAM_BUS] for m in HYBRID_CAM_MSGS]
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment),
                             SUBARU_CAM_BUS: HYBRID_CAM_MSGS}
  FWD_BLACKLISTED_ADDRS = {SUBARU_CAM_BUS: fwd_blacklisted_addr()[SUBARU_CAM_BUS], SUBARU_MAIN_BUS: list(HYBRID_CAM_MSGS)}

  def _user_brake_msg(self, brake):
    self._brake = brake
    return self._throttle_hybrid_msg()

  def _user_gas_msg(self, gas):
    self._gas = gas
    return self._throttle_hybrid_msg()

  def _throttle_hybrid_msg(self, bus=SUBARU_ALT_BUS):
    # driver gas and the brake pedal switch are in the same message
    values = {"Brake": 3 if self._brake else 0, "Throttle_Pedal": self._gas}
    return self.packer.make_can_msg_safety("Throttle_Hybrid", bus, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("Cruise_Status_2", SUBARU_ALT_BUS, values)

  def setUp(self):
    self._brake = False
    self._gas = 0
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
      self._rx(self.packer.make_can_msg_safety("Throttle_Hybrid", bus, {"Brake": 3, "Throttle_Pedal": 1}))
      self._rx(self.packer.make_can_msg_safety("Cruise_Status_2", bus, {"Cruise_Activated": 1}))
      self.assertEqual((False, False, False), self._state())

  def test_brake_hybrid_ignored(self):
    # Brake_Hybrid has no checksum or counter, the brake pedal switch in Throttle_Hybrid is used instead
    self._rx(self.packer.make_can_msg_safety("Brake_Hybrid", SUBARU_ALT_BUS, {"Brake": 1}))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    for value in (1, 2, 3):
      self._rx(self.packer.make_can_msg_safety("Throttle_Hybrid", SUBARU_ALT_BUS, {"Brake": value}))
      self.assertTrue(self.safety.get_brake_pressed_prev())
      self._rx(self.packer.make_can_msg_safety("Throttle_Hybrid", SUBARU_ALT_BUS, {"Brake": 0}))
      self.assertFalse(self.safety.get_brake_pressed_prev())

  def test_es_distance_blocked(self):
    values = {"Cruise_Cancel": 1, "Cruise_Throttle": self.INACTIVE_GAS}
    for bus in (SUBARU_MAIN_BUS, SUBARU_ALT_BUS, SUBARU_CAM_BUS):
      self.assertFalse(self._tx(self.global_packer.make_can_msg_safety("ES_Distance", bus, values)))

  # (message, length, byte and mask of the driver brake bit, signal values of a car frame)
  RESENT_MSGS = (
    ("Brake_Switch", 3, 1, 0x20, {"Brake_Late": 1, "Signal1": 0x2aa}),
    ("BodyInfo", 8, 6, 0x40, {"Signal1": 0xa5a5a, "DOOR_OPEN_FR": 1, "Signal2": 0x15555, "LOWBEAM": 1, "WIPERS": 1}),
  )

  def _resent_msgs(self):
    for name, length, brake_byte, brake_mask, values in self.RESENT_MSGS:
      addr = SubaruMsg[name]
      for counter in (0, 5, 15):
        for car_brake in (False, True):
          brake_sig = "Brake" if name == "Brake_Switch" else "BRAKE"
          _, dat, _ = self.packer.make_can_msg(name, SUBARU_MAIN_BUS, {**values, "COUNTER": counter, brake_sig: car_brake})
          self.assertEqual(length, len(dat))
          self.assertEqual(car_brake, bool(dat[brake_byte] & brake_mask))
          yield addr, bytearray(dat), brake_byte, brake_mask

  @staticmethod
  def _checksum(addr, dat):
    return (sum(dat[1:]) + (addr & 0xFF) + (addr >> 8)) & 0xFF

  def _resend(self, addr, dat, fix_checksum=True):
    dat = bytearray(dat)
    if fix_checksum:
      dat[0] = self._checksum(addr, dat)
    return self._tx(common.make_msg(SUBARU_CAM_BUS, addr, len(dat), bytes(dat)))

  def test_resend_unchanged(self):
    for addr, dat, _, _ in self._resent_msgs():
      # nothing can be sent before the car's frame is seen
      self._reset_safety_hooks()
      self.assertFalse(self._resend(addr, dat))
      self.assertTrue(self._rx(common.make_msg(SUBARU_MAIN_BUS, addr, len(dat), bytes(dat))))
      self.assertTrue(self._resend(addr, dat))

  def test_resend_brake_bit(self):
    # the driver brake bit can be set to cancel ACC, with or without controls allowed. It can never be cleared
    for addr, dat, brake_byte, brake_mask in self._resent_msgs():
      for controls_allowed in (True, False):
        self._reset_safety_hooks()
        self._rx(common.make_msg(SUBARU_MAIN_BUS, addr, len(dat), bytes(dat)))
        self.safety.set_controls_allowed(controls_allowed)

        car_brake = bool(dat[brake_byte] & brake_mask)
        pressed, released = bytearray(dat), bytearray(dat)
        pressed[brake_byte] |= brake_mask
        released[brake_byte] &= ~brake_mask & 0xFF
        self.assertTrue(self._resend(addr, pressed))
        self.assertEqual(not car_brake, self._resend(addr, released))

  def test_resend_other_bits(self):
    # every other bit must match the car's frame, including the counter
    for addr, dat, brake_byte, brake_mask in self._resent_msgs():
      self._reset_safety_hooks()
      self._rx(common.make_msg(SUBARU_MAIN_BUS, addr, len(dat), bytes(dat)))
      for bit in range(8, len(dat) * 8):
        byte, mask = bit // 8, 1 << (bit % 8)
        if (byte, mask) == (brake_byte, brake_mask):
          continue
        for set_brake in (False, True):
          changed = bytearray(dat)
          changed[byte] ^= mask
          if set_brake:
            changed[brake_byte] |= brake_mask
          self.assertFalse(self._resend(addr, changed), (hex(addr), bit))
      self.assertTrue(self._resend(addr, dat))

  def test_resend_checksum(self):
    for addr, dat, brake_byte, brake_mask in self._resent_msgs():
      self._reset_safety_hooks()
      self._rx(common.make_msg(SUBARU_MAIN_BUS, addr, len(dat), bytes(dat)))
      pressed = bytearray(dat)
      pressed[brake_byte] |= brake_mask
      pressed[0] = self._checksum(addr, pressed)
      for frame in (dat, pressed):
        for checksum in range(256):
          changed = bytearray(frame)
          changed[0] = checksum
          self.assertEqual(checksum == frame[0], self._resend(addr, changed, fix_checksum=False))

  def test_resend_latest_frame_only(self):
    frames = list(self._resent_msgs())
    for (addr, dat, _, _), (addr_new, dat_new, _, _) in zip(frames, frames[1:], strict=False):
      if addr != addr_new or dat[1] & 0xF == dat_new[1] & 0xF:
        continue
      self._reset_safety_hooks()
      self._rx(common.make_msg(SUBARU_MAIN_BUS, addr, len(dat), bytes(dat)))
      self._rx(common.make_msg(SUBARU_MAIN_BUS, addr, len(dat_new), bytes(dat_new)))
      self.assertFalse(self._resend(addr, dat))
      self.assertTrue(self._resend(addr, dat_new))

  def test_resend_car_frame_bus(self):
    # the car's frames are only taken from the main bus, and a frame of one message never allows the other
    for addr, dat, _, _ in self._resent_msgs():
      self._reset_safety_hooks()
      for bus in (SUBARU_ALT_BUS, SUBARU_CAM_BUS):
        self._rx(common.make_msg(bus, addr, len(dat), bytes(dat)))
      self.safety.set_relay_malfunction(False)
      self.assertFalse(self._resend(addr, dat))

    (addr_a, dat_a, _, _), (addr_b, dat_b, _, _) = (next(m for m in self._resent_msgs() if m[0] == a) for a in self.HYBRID_CAM_MSGS)
    self._reset_safety_hooks()
    self._rx(common.make_msg(SUBARU_MAIN_BUS, addr_a, len(dat_a), bytes(dat_a)))
    self.assertFalse(self._resend(addr_b, dat_b))
    self._reset_safety_hooks()
    self._rx(common.make_msg(SUBARU_MAIN_BUS, addr_b, len(dat_b), bytes(dat_b)))
    self.assertFalse(self._resend(addr_a, dat_a))


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
