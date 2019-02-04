
"use strict";

let navdata_vision_raw = require('./navdata_vision_raw.js');
let navdata_wifi = require('./navdata_wifi.js');
let navdata_phys_measures = require('./navdata_phys_measures.js');
let navdata_time = require('./navdata_time.js');
let navdata_vision = require('./navdata_vision.js');
let matrix33 = require('./matrix33.js');
let navdata_magneto = require('./navdata_magneto.js');
let navdata_vision_perf = require('./navdata_vision_perf.js');
let navdata_vision_detect = require('./navdata_vision_detect.js');
let navdata_games = require('./navdata_games.js');
let Navdata = require('./Navdata.js');
let navdata_altitude = require('./navdata_altitude.js');
let navdata_demo = require('./navdata_demo.js');
let navdata_hdvideo_stream = require('./navdata_hdvideo_stream.js');
let navdata_euler_angles = require('./navdata_euler_angles.js');
let navdata_pressure_raw = require('./navdata_pressure_raw.js');
let navdata_trims = require('./navdata_trims.js');
let navdata_video_stream = require('./navdata_video_stream.js');
let vector21 = require('./vector21.js');
let navdata_vision_of = require('./navdata_vision_of.js');
let navdata_rc_references = require('./navdata_rc_references.js');
let vector31 = require('./vector31.js');
let navdata_kalman_pressure = require('./navdata_kalman_pressure.js');
let navdata_watchdog = require('./navdata_watchdog.js');
let navdata_trackers_send = require('./navdata_trackers_send.js');
let navdata_adc_data_frame = require('./navdata_adc_data_frame.js');
let navdata_raw_measures = require('./navdata_raw_measures.js');
let navdata_references = require('./navdata_references.js');
let navdata_zimmu_3000 = require('./navdata_zimmu_3000.js');
let navdata_gyros_offsets = require('./navdata_gyros_offsets.js');
let navdata_pwm = require('./navdata_pwm.js');
let navdata_wind_speed = require('./navdata_wind_speed.js');

module.exports = {
  navdata_vision_raw: navdata_vision_raw,
  navdata_wifi: navdata_wifi,
  navdata_phys_measures: navdata_phys_measures,
  navdata_time: navdata_time,
  navdata_vision: navdata_vision,
  matrix33: matrix33,
  navdata_magneto: navdata_magneto,
  navdata_vision_perf: navdata_vision_perf,
  navdata_vision_detect: navdata_vision_detect,
  navdata_games: navdata_games,
  Navdata: Navdata,
  navdata_altitude: navdata_altitude,
  navdata_demo: navdata_demo,
  navdata_hdvideo_stream: navdata_hdvideo_stream,
  navdata_euler_angles: navdata_euler_angles,
  navdata_pressure_raw: navdata_pressure_raw,
  navdata_trims: navdata_trims,
  navdata_video_stream: navdata_video_stream,
  vector21: vector21,
  navdata_vision_of: navdata_vision_of,
  navdata_rc_references: navdata_rc_references,
  vector31: vector31,
  navdata_kalman_pressure: navdata_kalman_pressure,
  navdata_watchdog: navdata_watchdog,
  navdata_trackers_send: navdata_trackers_send,
  navdata_adc_data_frame: navdata_adc_data_frame,
  navdata_raw_measures: navdata_raw_measures,
  navdata_references: navdata_references,
  navdata_zimmu_3000: navdata_zimmu_3000,
  navdata_gyros_offsets: navdata_gyros_offsets,
  navdata_pwm: navdata_pwm,
  navdata_wind_speed: navdata_wind_speed,
};
