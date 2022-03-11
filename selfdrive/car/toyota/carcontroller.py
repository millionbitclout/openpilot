from cereal import car, log
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_interceptor_command, make_can_msg
from selfdrive.car.toyota.toyotacan import create_steer_command, create_ui_command, create_ui_command_off,\
                                           create_accel_command, create_acc_cancel_command, \
                                           create_fcw_command, create_lta_steer_command, create_acc_set_speed
from selfdrive.car.toyota.values import CAR, STATIC_DSU_MSGS, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        MIN_ACC_SPEED, PEDAL_TRANSITION, CarControllerParams, CruiseButtons
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from common.params import Params
import cereal.messaging as messaging
from selfdrive.controls.lib.speed_limit_controller import SpeedLimitController
VisualAlert = car.CarControl.HUDControl.VisualAlert
SpeedLimitControlState = log.LongitudinalPlan.SpeedLimitControlState


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.steer_rate_limited = False
    self.signal_last = 0.

    self.packer = CANPacker(dbc_name)

    self.sm = messaging.SubMaster(['liveMapData', 'controlsState', 'longitudinalPlan'])
    self.speed_limit_control_enabled = Params().get_bool("SpeedLimitControl")

    self.speed_limit_osm = 0
    self.speed_limit_offset_osm = 0
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.resume_count = 0
    self.t_interval = 7
    self.sl_force_active_timer = 0
    self.slc_state = 0
    self.slc_active_stock = False
    self.final_speed_kph_prev = 0

  def update(self, enabled, active, CS, frame, actuators, pcm_cancel_cmd, hud_alert,
             left_line, right_line, lead, left_lane_depart, right_lane_depart):

    # *** compute control surfaces ***

    # gas and brake
    if CS.CP.enableGasInterceptor and enabled and CS.out.cruiseState.enabled:
      MAX_INTERCEPTOR_GAS = 0.5
      # RAV4 has very sensitive gas pedal
      if CS.CP.carFingerprint in [CAR.RAV4, CAR.RAV4H, CAR.HIGHLANDER, CAR.HIGHLANDERH]:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.15, 0.3, 0.0])
      elif CS.CP.carFingerprint in [CAR.COROLLA]:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.3, 0.4, 0.0])
      else:
        PEDAL_SCALE = interp(CS.out.vEgo, [0.0, MIN_ACC_SPEED, MIN_ACC_SPEED + PEDAL_TRANSITION], [0.4, 0.5, 0.0])
      # offset for creep and windbrake
      pedal_offset = interp(CS.out.vEgo, [0.0, 2.3, MIN_ACC_SPEED + PEDAL_TRANSITION], [-.4, 0.0, 0.2])
      pedal_command = PEDAL_SCALE * (actuators.accel + pedal_offset)
      interceptor_gas_cmd = clip(pedal_command, 0., MAX_INTERCEPTOR_GAS)
    else:
      interceptor_gas_cmd = 0.
    pcm_accel_cmd = 0 if not (enabled and CS.out.cruiseState.enabled) else clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

    # steer torque
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, CarControllerParams)
    self.steer_rate_limited = False

    cur_time = frame * DT_CTRL
    if CS.leftBlinkerOn or CS.rightBlinkerOn:
      self.signal_last = cur_time

    # Cut steering while we're in a known fault state (2s)
    if enabled and not CS.steer_not_allowed and CS.lkasEnabled and ((CS.automaticLaneChange and not CS.belowLaneChangeSpeed) or ((not ((cur_time - self.signal_last) < 1) or not CS.belowLaneChangeSpeed) and not (CS.leftBlinkerOn or CS.rightBlinkerOn))):
      self.steer_rate_limited = new_steer != apply_steer
      apply_steer_req = 1
    else:
      apply_steer = 0
      apply_steer_req = 0

    # TODO: probably can delete this. CS.pcm_acc_status uses a different signal
    # than CS.cruiseState.enabled. confirm they're not meaningfully different
    #if not enabled and CS.pcm_acc_status:
      #pcm_cancel_cmd = 1

    # on entering standstill, send standstill request
    if CS.out.standstill and not self.last_standstill and CS.CP.carFingerprint not in NO_STOP_TIMER_CAR:
      self.standstill_req = True
    if CS.pcm_acc_status != 8:
      # pcm entered standstill or it's disabled
      self.standstill_req = False

    self.last_steer = apply_steer
    self.last_standstill = CS.out.standstill

    can_sends = []

    #*** control msgs ***
    #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

    # toyota can trace shows this message at 42Hz, with counter adding alternatively 1 and 2;
    # sending it at 100Hz seem to allow a higher rate limit, as the rate limit seems imposed
    # on consecutive messages
    can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))
    if frame % 2 == 0 and CS.CP.carFingerprint in TSS2_CAR:
      can_sends.append(create_lta_steer_command(self.packer, 0, 0, frame // 2))

    # LTA mode. Set ret.steerControlType = car.CarParams.SteerControlType.angle and whitelist 0x191 in the panda
    # if frame % 2 == 0:
    #   can_sends.append(create_steer_command(self.packer, 0, 0, frame // 2))
    #   can_sends.append(create_lta_steer_command(self.packer, actuators.steeringAngleDeg, apply_steer_req, frame // 2))

    # we can spam can to cancel the system even if we are using lat only control
    if (frame % 3 == 0 and CS.CP.openpilotLongitudinalControl) or pcm_cancel_cmd:
      lead = lead or CS.out.vEgo < 12.    # at low speed we always assume the lead is present do ACC can be engaged

      # Lexus IS uses a different cancellation message
      if pcm_cancel_cmd and CS.CP.carFingerprint in [CAR.LEXUS_IS, CAR.LEXUS_RC]:
        can_sends.append(create_acc_cancel_command(self.packer))
      elif CS.CP.openpilotLongitudinalControl:
        can_sends.append(create_accel_command(self.packer, pcm_accel_cmd, pcm_cancel_cmd, self.standstill_req, lead, CS.acc_type))
      else:
        can_sends.append(create_accel_command(self.packer, 0, pcm_cancel_cmd, False, lead, CS.acc_type))

    if frame % 2 == 0 and CS.CP.enableGasInterceptor:
      # send exactly zero if gas cmd is zero. Interceptor will send the max between read value and gas cmd.
      # This prevents unexpected pedal range rescaling
      can_sends.append(create_gas_interceptor_command(self.packer, interceptor_gas_cmd, frame // 2))

    speed_limit_control_enabled = Params().get_bool("SpeedLimitControl")
    last_speed_limit_sign_tap = Params().get_bool("LastSpeedLimitSignTap")

    speed_limit = self.get_speed_limit_osm()
    speed_limit_offset = self.get_speed_limit_offset_osm()
    speed_limit_offsetted = int(speed_limit + speed_limit_offset)

    if last_speed_limit_sign_tap:
      self.sl_force_active_timer = cur_time

    sl_force_active = speed_limit_control_enabled and (cur_time < self.sl_force_active_timer + 2.0)
    sl_inactive = not sl_force_active and (not speed_limit_control_enabled or (True if self.get_slc_state() == 0 else False))
    sl_temp_inactive = not sl_force_active and (speed_limit_control_enabled and (True if self.get_slc_state() == 1 else False))
    slc_active = not sl_inactive and not sl_temp_inactive

    self.slc_active_stock = slc_active

    if not CS.CP.openpilotLongitudinalControl:
      if not CS.out.cruiseState.standstill and CS.acc_active and self.speed_limit_control_enabled:
        cruise_set_point = self.get_cruise_speed(CS)
        if cruise_set_point != 0:
          can_sends.append(create_acc_set_speed(self.packer, cruise_set_point))

    # ui mesg is at 100Hz but we send asap if:
    # - there is something to display
    # - there is something to stop displaying
    fcw_alert = hud_alert == VisualAlert.fcw
    steer_alert = hud_alert in [VisualAlert.steerRequired, VisualAlert.ldw]

    send_ui = False
    if ((fcw_alert or steer_alert) and not self.alert_active) or \
       (not (fcw_alert or steer_alert) and self.alert_active):
      send_ui = True
      self.alert_active = not self.alert_active
    elif pcm_cancel_cmd:
      # forcing the pcm to disengage causes a bad fault sound so play a good sound instead
      send_ui = True

    if (frame % 100 == 0 or send_ui):
      if(CS.lkasEnabled):
        can_sends.append(create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, left_line, right_line, left_lane_depart, right_lane_depart, CS.lkasEnabled and not apply_steer_req))
      else:
        can_sends.append(create_ui_command_off(self.packer))

    if frame % 100 == 0 and CS.CP.enableDsu:
      can_sends.append(create_fcw_command(self.packer, fcw_alert))

    #*** static msgs ***

    for (addr, cars, bus, fr_step, vl) in STATIC_DSU_MSGS:
      if frame % fr_step == 0 and CS.CP.enableDsu and CS.CP.carFingerprint in cars:
        can_sends.append(make_can_msg(addr, vl, bus))

    return can_sends

  def get_speed_limit_osm(self):
    is_metric = Params().get_bool("IsMetric")
    self.sm.update(0)
    speed_limit_osm = float(self.sm['longitudinalPlan'].speedLimit if self.sm['longitudinalPlan'].speedLimit is not None else 0.0) * (CV.MS_TO_MPH if not is_metric else CV.MS_TO_KPH)
    return speed_limit_osm

  def get_speed_limit_offset_osm(self):
    speed_limit_perc_offset = Params().get_bool("SpeedLimitPercOffset")
    speed_limit_value_offset = int(Params().get("SpeedLimitValueOffset"))
    is_metric = Params().get_bool("IsMetric")
    if speed_limit_perc_offset:
      self.sm.update(0)
      speed_limit_offset = float(self.sm['longitudinalPlan'].speedLimitOffset) * (CV.MS_TO_MPH if not is_metric else CV.MS_TO_KPH)
    else:
      speed_limit_offset = float(speed_limit_value_offset)
    return speed_limit_offset

  def get_slc_state(self):
    self.sm.update(0)
    self.slc_state = self.sm['longitudinalPlan'].speedLimitControlState
    return self.slc_state

  def get_cruise_buttons_status(self, CS):
    if not CS.cruise_active or CS.cruise_buttons != CruiseButtons.SET_ACC:
      self.timer = 40
    elif self.timer:
      self.timer -= 1
    else:
      return 1
    return 0

  def get_target_speed(self, v_cruise_kph_prev):
    speed_limit_perc_offset = Params().get_bool("SpeedLimitPercOffset")
    speed_limit_value_offset = int(Params().get("SpeedLimitValueOffset"))
    is_metric = Params().get_bool("IsMetric")
    v_cruise_kph = v_cruise_kph_prev
    self.sm.update(0)
    if Params().get_bool("SpeedLimitControl") and (float(self.sm['longitudinalPlan'].speedLimit if self.sm['longitudinalPlan'].speedLimit is not None else 0.0) != 0.0):
      self.sm.update(0)
      target_v_cruise_kph = float(self.sm['longitudinalPlan'].speedLimit if self.sm['longitudinalPlan'].speedLimit is not None else 0.0) * CV.MS_TO_KPH
      if speed_limit_perc_offset:
        self.sm.update(0)
        v_cruise_kph = target_v_cruise_kph + float(float(self.sm['longitudinalPlan'].speedLimitOffset) * CV.MS_TO_KPH)
      else:
        v_cruise_kph = target_v_cruise_kph + (float(speed_limit_value_offset * CV.MPH_TO_KPH) if not is_metric else speed_limit_value_offset)
      if not self.slc_active_stock:
        v_cruise_kph = v_cruise_kph_prev

    print('v_cruise_kph={}'.format(v_cruise_kph))

    return v_cruise_kph

  def get_button_type(self, button_type):
    self.type_status = "type_" + str(button_type)
    self.button_picker = getattr(self, self.type_status, lambda:"default")
    return self.button_picker()

  def reset_button(self):
    if self.button_type != 3:
      self.button_type = 0

  def type_default(self):
    self.button_type = 0
    return None

  def type_0(self):
    self.button_count = 0
    self.target_speed = self.init_speed
    speed_diff = round(self.target_speed - self.v_set_dis)
    if speed_diff > 0:
      self.button_type = 1
    elif speed_diff < 0:
      self.button_type = 2
    return CruiseButtons.SET_ACC

  def type_1(self):
    cruise_button = CruiseButtons.ACCEL_ACC
    self.button_count += 1
    if self.target_speed == self.v_set_dis:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_2(self):
    cruise_button = CruiseButtons.DECEL_ACC
    self.button_count += 1
    if self.target_speed == self.v_set_dis:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_3(self):
    cruise_button = CruiseButtons.SET_ACC
    self.button_count += 1
    if self.button_count > self.t_interval:
      self.button_type = 0
    return cruise_button

  def get_curve_speed(self, target_speed_kph):
    v_cruise_kph_prev = self.sm['controlsState'].vCruise
    if Params().get_bool("TurnVisionControl"):
      self.sm.update(0)
      vision_v_cruise_kph = float(float(self.sm['longitudinalPlan'].visionTurnSpeed) * CV.MS_TO_KPH)
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
      vision_v_cruise_kph = min(target_speed_kph, vision_v_cruise_kph)
    else:
      vision_v_cruise_kph = 255
    if Params().get_bool("TurnSpeedControl"):
      self.sm.update(0)
      map_v_cruise_kph = float(float(self.sm['longitudinalPlan'].turnSpeed) * CV.MS_TO_KPH)
      if int(map_v_cruise_kph) == 0.0:
        map_v_cruise_kph = 255
      map_v_cruise_kph = min(target_speed_kph, map_v_cruise_kph)
    else:
      map_v_cruise_kph = 255
    print('vision_v_cruise_kph={}  map_v_cruise_kph={}'.format(vision_v_cruise_kph, map_v_cruise_kph))
    return min(target_speed_kph, vision_v_cruise_kph, map_v_cruise_kph)

  def get_button_control(self, CS, final_speed):
    is_metric = Params().get_bool("IsMetric")
    self.sm.update(0)
    v_cruise_kph_max = self.sm['controlsState'].vCruise
    self.init_speed = round(min(final_speed, v_cruise_kph_max) * CV.KPH_TO_MPH) if not is_metric else round(min(final_speed, v_cruise_kph_max))
    self.v_set_dis = round(CS.out.cruiseState.speed * CV.MS_TO_MPH) if not is_metric else round(CS.out.cruiseState.speed * CV.MS_TO_KPH)
    cruise_button = self.get_button_type(self.button_type)
    return cruise_button

  def get_cruise_speed(self, CS):
    cruise_set_point = 0
    if not self.get_cruise_buttons_status(CS):
      pass
    elif CS.cruise_active:
      self.sm.update(0)
      v_cruise_kph_prev = self.sm['controlsState'].vCruise
      set_speed_kph = self.get_target_speed(v_cruise_kph_prev)
      if Params().get_bool("SpeedLimitControl"):
        target_speed_kph = set_speed_kph
      else:
        target_speed_kph = min(v_cruise_kph_prev, set_speed_kph)
      if Params().get_bool("TurnVisionControl") or Params().get_bool("TurnSpeedControl"):
        self.final_speed_kph = self.get_curve_speed(target_speed_kph)
      else:
        self.final_speed_kph = target_speed_kph

      print('self.final_speed_kph={}  v_cruise_kph_prev={}'.format(self.final_speed_kph, v_cruise_kph_prev))

      if self.final_speed_kph_prev != self.final_speed_kph:
        cruise_set_point = self.final_speed_kph
        self.button_count += 1
      if self.button_count > 5:
        self.button_count = 0
        cruise_set_point = 0
        self.final_speed_kph_prev = self.final_speed_kph
    return cruise_set_point