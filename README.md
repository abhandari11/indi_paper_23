
# INDI_paper_23
Simulation Tools and Results supplementing the published articles on INDI based control law on quad-plane UAV. 

## Running the Simulations
To run the quad-plane simulations with INDI based control architecture, one can follow these steps:
	* Open 'config.m' and make sure 'session_v0' is the selected vehicle. 
	* Open 'setup.m' file and Run. 
	* It will open the Simulink model 'quadplane_sim()' with 'vms/quadplane_controlA()' as embedded control law. 
	* Modify the trajectory commands as needed. 
	* Hit 'Run'.
	* General results will be exported to the workspace, or can be accessed with Scopes. 

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/logo.png" alt="Logo" width="250">

# Simulink/C++ Platform for Aeronautics and Autonomy Research and Operations (SPAARO)
SPAARO, when coupled with Bolder Flight control systems, enables engineers to quickly research, develop, and deploy control laws, autonomy algorithms, and flight software.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Overview
<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/spaaro.jpg" alt="SPAARO" width="400">

SPAARO and Bolder Flight control systems handle low-level processor startup, timing/scheduling, peripheral drivers, and real-time filtering and estimation. An input / output plane is presented around the flight software, enabling developers to focus on development of control laws, autonomy algorithms, and high-level planning, guidance, and control algorithms. Bolder Flight Systems hardware and software is developed by former NASA and DoD researchers and engineers with a focus on data quality, reliability, and determinism. It is an ideal platform for conducting world-class research and can be rapidly deployed on off-the-shelf or custom commercial flight control systems, enabling businesses to focus on their differentiating technologies and bring products to market at an astonishing speed.

SPAARO supports fixed-wing, multi-rotor, helicopter, and V/STOL vehicles. Software can be developed in Simulink or C++. A Simulink simulation is available for developing and validating algorithms prior to flight. Flight data is converted to MATLAB format for analysis, which can be opened by [MATLAB](https://www.mathworks.com/products/matlab.html), [Octave](https://www.gnu.org/software/octave/index), and [SciPy](https://www.scipy.org/). [MAVLink](https://mavlink.io/) is fully supported for real-time telemetry, in-flight-tunable parameters, flight plans, fences, and rally points. All Bolder Flight control systems are designed and assembled in the United States.

# Flight Control Systems

## FMU-R
The Research Flight Management Unit (FMU-R) is designed to provide unsurpassed data quality, determinism, and flexibility. FMU-R is ideally suited for early-stage R&D and features a plethora of ports for integrating new peripherals. FMU-R has the option of using a low-cost integrated IMU or adding a VectorNav VN-100, VN-200, or VN-300 IMU/INS. FMU-R can be used stand-alone or a BeagleBone Black or BeagleBone AI can be added for additional compute power; high bandwidth serial and USB connections are available for sharing data between the FMU-R and BeagleBone. FMU-R is designed around a consumer temperature range of 0C to +50C.

An integrated static pressure sensor is available or an external air data sensor can be added to collect differential and static pressure data from a pitot-tube. Multiple pressure ranges are available and sensors can be chained to accommodate 5 or 7 hole probes. Breakout boards are available to convert the JST-GH PWM or SBUS connectors to standard servo headers and have screw terminals for supplying servo rail power. VectorNav, SBUS, and PWM breakout boards can be mounted to the FMU-R for a compact, integrated solution.

### FMU-R v1.x

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/fmu-r-v1.png" alt="FMU-R v1" width="400">

FMU-R v1.x consists of:
   * 50 Hz hard real-time frame rate.
   * Cortex M4F processor, 180 MHz CPU frequency and a single precision hardware floating point unit.
   * Integrated voltage regulation with a +6.5V to +36V input range. Clean +5V output up to 2A is available for powering the FMU-R, BeagleBone, and peripherals.
   * Integrated 9 axis IMU and static pressure sensor.
   * SBUS input for integrating pilot commands.
   * 16 channels of SBUS output and 8 channels of PWM output.
   * Two I2C buses, two UARTs, and one SPI bus for connecting external sensors, such as air data, GNSS receivers, telemetry, and additional IMUs.
   * Two GPIOs for analog input, digital I/O, or additional PWM channels.
   * Integrated measurement of input voltage, regulated voltage, and servo rail voltages (up to +9.9V).
   * Two UARTs from the BeagleBone are brought out to convenient connectors for interfacing with external sensors.

The FMU-R v1.x schematic is [available here](./docs/fmu_r_v1_schematic.pdf).

### FMU-R v2.x

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/fmu-r-v2.png" alt="FMU-R v2" width="400">

FMU-R v2.x consists of:
   * 100 Hz hard real-time frame rate.
   * Cortex M7 processor, 528 MHz CPU frequency and double precision hardware floating point unit.
   * Integrated 9 axis IMU and static pressure sensor.
   * SBUS input for integrating pilot commands.
   * 16 channels of SBUS output and 8 channels of PWM output.
   * 8 channels of analog input.
   * One CAN 2.0/FD bus, one I2C bus, four UARTs, and one SPI bus for connecting external sensors, such as air data, GNSS receivers, telemetry, and additional IMUs.
   * Measurement of battery voltage and current from an external power module.
   * Two UARTs from the BeagleBone are brought out to convenient connectors for interfacing with external sensors.

The FMU-R v2.x schematic is [available here](./docs/fmu_r_v2_schematic.pdf).

### GNSS Receiver
[uBlox](https://www.u-blox.com/) 8 and 9 series GNSS receivers are supported via the UBX communication protocol. Bolder Flight Systems manufactures a small, low-cost GNSS receiver using the [SAM-M8Q module](https://www.u-blox.com/en/product/sam-m8q-module). If better position accuracy is required, we recommend the [ZED-F9P dual frequency module](https://www.u-blox.com/en/product/zed-f9p-module). [ArduSimple](https://www.ardusimple.com/product/simplertk2blite/) manufactures a small ZED-F9P receiver, which we use frequently with the FMU-R.

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/sam-m8q.png" alt="SAM-M8Q GNSS Receiver" width="150">

### Air Data Sensor
Bolder Flight Systems developed an air data sensor, which uses AMS5915 pressure transducers to measure static and differential pressure. Several pressure ranges are available and can be customized to the vehicle's airspeed range. Additionally, customized sensors can be built to support multi-hole probes for angle of attack and angle of sideslip measurement.

<img src="https://github.com/bolderflight/spaaro/blob/main/docs/img/swift.png" alt="Air Data Sensor" width="200">

### VectorNav IMU/INS
VectorNav [VN-100](https://www.vectornav.com/products/vn-100), [VN-200](https://www.vectornav.com/products/vn-200), and [VN-300](https://www.vectornav.com/products/vn-300) IMU and INS sensors can be added to the FMU-R. These sensors are temperature calibrated and feature integrated navigation filter algorithms. The VN-200 and VN-300 include integrated GNSS receivers. The VN-300 includes dual GNSS receivers, which can be used to estimate the vehicle heading more accurately than magnetometer based approaches.

### PWM and SBUS Breakouts
Boards are available to breakout the JST-GH connectors to standard servo connectors. 8 channels are available on each board and the SBUS boards can be daisy-chained for 16 total output channels. Servo power is bused and can be provided by a connected ESC, BEC, or via screw terminals. Servo rail voltage is measured up to +9.9V.

# Setting up the Development Environment
Follow the [build tools guide](https://github.com/bolderflight/build-tools) for setting up your development environment. Additionally, if you plan on using the SPAARO Simulink simulation, you'll need:
   * MATLAB Simulink
   * Aerospace Blockset
   * Control System Toolbox (trimming and linearization)
   * Simulink Control Design (trimming and linearization)

If you plan on autocoding flight software, you will also need:
   * MATLAB Coder
   * Simulink Coder
   * Embedded Coder

# Aircraft Configuration
The aircraft sensors, real-time filtering and estimation, and effectors are configured with a configuration file, */flight_code/flight/config.cc*. Within that file, you'll find a bool *DEBUG* and a struct *config*. 

The software will output messages over the FMU-R micro USB. If *DEBUG* is set to *true*, the software will wait for a serial monitor to be opened before it starts booting, ensuring that you will receive all messages, which is useful for debugging any issues. If *DEBUG* is set to false, the software will immediately start booting on power-up, which is the typical configuration for flight.

The *config* struct has top-level items for *sensor*, *nav*, *effector*, and *telem*, which will be described in detail in the following sections. An example, complete aircraft configuration is:

```C++
AircraftConfig config = {
  .sensor = {
    .pitot_static_installed = true,
    .imu = {
      .dev = IMU_CS,
      .frame_rate = FRAME_RATE_HZ,
      .bus = &IMU_SPI_BUS,
      .accel_bias_mps2 = {0, 0, 0},
      .mag_bias_ut = {0, 0, 0},
      .accel_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      .mag_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
      .rotation = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
    },
    .gnss = {
      .sampling_period_ms = 200,  // 5 Hz
      .baud = 921600,
      .bus = &Serial3
    },
    .static_pres = {
      .dev = 0x10,
      .transducer = bfs::AMS5915_1200_B,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_I2C_BUS
    },
    .diff_pres = {
      .dev = 0x11,
      .transducer = bfs::AMS5915_0010_D,
      .sampling_period_ms = FRAME_PERIOD_MS,
      .bus = &PRES_I2C_BUS
    }
  .nav = {
    .accel_cutoff_hz = 20,
    .gyro_cutoff_hz = 20,
    .mag_cutoff_hz = 10,
    .static_pres_cutoff_hz = 10,
    .diff_pres_cutoff_hz = 10
  },
  .telem = {
    .aircraft_type = bfs::FIXED_WING,
    .bus = &Serial4,
    .baud = 57600
  }
};
```

## Sensors
*.sensor* configures the aircraft sensors.

### Pitot-Static Installed
The first configurable item is whether an air data sensor is installed and should be used for static and differential pressure sensing. This is simply a boolean *true* (air data sensor installed), *false* (air data sensor not installed).

```C++
/* Pitot static sensor not installed */
.sensor = {
  .pitot_static_installed = false,
}
```

### IMU
The *.imu* struct configures the integrated IMU. First, the communication bus information and frame rate is specified, which are defined in */flight_code/include/flight/hardware_defs.h* and should not be modified.

Next, the accelerometer bias, magnetometer bias, accelerometer scale factor, and magnetometer scale factor can be configured. These are defined such that:

```
y = c * x + b
```

Where *y* is the corrected sensor output, *c* is the scale factor matrix, *x* is the uncorrected sensor output, and *b* is the bias vector. An ideal sensor would have a bias vector of zeros and an identity scale factor matrix.

We typically estimate accelerometer bias and scale factor by collecting data on the FMU-R with each axis aligned with gravity in a positive and negative sense. We then fit the bias and scale factor such that we get a magnitude in each axis of 9.80665.

The magnetometer is typically calibrated in vehicle with the electronics powered and the motors off since we usually only use the magnetometer to initialize the aircraft heading for the navigation filters. The aircraft is rotated in a sphere for all three axes and in post-processing we estimate the bias and scale factors necessary to fit the magnetometer data to a sphere.

A rotation matrix can be defined to rotate the IMU into the vehicle frame. The rotation matrix is defined such that:

```
y = c * x
```

Where *x* is the sensor data, *c* is the rotation matrix, and *y* is the sensor data rotated into the vehicle frame.

An example IMU configuration is:

```C++
.imu = {
  .dev = IMU_CS,
  .frame_rate = FRAME_RATE_HZ,
  .bus = &IMU_SPI_BUS,
  .accel_bias_mps2 = {0, 0, 0},
  .mag_bias_ut = {0, 0, 0},
  .accel_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
  .mag_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
  .rotation = {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
},
```

### GNSS
The *.gnss* struct configures the GNSS receiver. The configurable items are the sampling period, in ms, the baud rate, and the serial bus the receiver is connected to. Below is an example configuration of a receiver connected to FMU-UART3, with a baud rate of 921600, and an update rate of 5 Hz.

```C++
.gnss = {
  .sampling_period_ms = 200,  // 5 Hz
  .baud = 921600,
  .bus = &Serial3
},
```

### Static Pressure
The *.static_pres* struct configures the static pressure sensor. If an air data sensor is not used, all of the items are defined in */flight_code/include/flight/hardware_defs.h* and should not be modified.

```C++
.static_pres = {
  .dev = PRES_CS,
  .sampling_period_ms = FRAME_PERIOD_MS,
  .bus = &PRES_SPI_BUS
},
```

If an air data sensor is used, configurable items include the I2C bus, I2C address, and transducer type. Typically the AMS1200B is used as a static pressure sensor.

```C++
.static_pres = {
  .dev = 0x10,
  .transducer = bfs::AMS5915_1200_B,
  .sampling_period_ms = FRAME_PERIOD_MS,
  .bus = &PRES_I2C_BUS
},
```

### Differential Pressure
The *.diff_pres* struct configures the differential pressure sensor. Configurable items include the I2C bus, I2C address, and transducer type. Typically the AMS1200B is used as a static pressure sensor.

```C++
.diff_pres = {
  .dev = 0x11,
  .transducer = bfs::AMS5915_0010_D,
  .sampling_period_ms = FRAME_PERIOD_MS,
  .bus = &PRES_I2C_BUS
},
```

## Navigation Filter
The *.nav* struct configures the navigation filter. 

Data flow from the sensor is:
1. Anti-alias filtering is applied, if available, based on the FMU sample rate.
2. Bias and scale factor corrections are applied. For the accelerometer and magnetometer, these are defined in the sensor configuration. For the gyro and differential pressure sensor, biases are estimated on startup.
3. The IMU is rotated into the vehicle frame.

This process yields the sensor data that is output. In the navigation filter:
1. An Extended Kalman Filter (EKF) uses IMU and GNSS data to estimate vehicle position, velocity, attitude, and accelerometer and gyro biases.
2. Accelerometer and gyro biases are removed.
3. Digital low pass filters are applied to the IMU and pressure transducer data.
4. This data is used to estimate derived quantities, such as pressure altitude, airspeed, NED position, etc.

Configurable items include the accelerometer, gyro, mag, static pressure, and differential pressure digital low pass filter cutoff frequencies. An example *nav* configuration is:

```C++
.nav = {
  .accel_cutoff_hz = 20,
  .gyro_cutoff_hz = 20,
  .mag_cutoff_hz = 10,
  .static_pres_cutoff_hz = 10,
  .diff_pres_cutoff_hz = 10
},
```

## Telemetry
The *.telem* struct configures the telemetry radio modem. The configurable items are the aircraft type, serial bus, and the baud rate.

The aircraft type can be:

| Aircraft Type | Description |
| --- | --- |
| bfs::FIXED_WING | Fixed-wing aircraft |
| bfs::HELICOPTER | Helicopter |
| bfs::MULTIROTOR | Multirotor |
| bfs::VTOL | VTOL aircraft |

An example configuration for a fixed-wing aircraft with the telemetry radio modem connected to FMU-UART4, and a baud rate of 57600 is below.

```C++
.telem = {
  .aircraft_type = bfs::FIXED_WING,
  .bus = &Serial4,
  .baud = 57600
}
```

# Software Overview
On boot, SPAARO initializes the:
1. Messaging bus, which provides status, warning, and error messages over the FMU micro USB.
2. System, which includes:
   * Initializing I2C and SPI buses
   * Setting up the analog to digital converters for monitoring system voltages
3. Sensors, which includes:
   * IMU: establish communications, configure the IMU, and estimate gyro biases.
   * GNSS: establish communications.
   * Inceptors: establish communications.
   * Pressure transducers: establish communications, configure the pressure transducers, and estimate differential pressure biases.
   * Battery monitoring (FMU-R v2.x): measures battery voltage and current, estimates battery capacity remaining and remaining flight time.
   * Analog: measures analog inputs, converts to engineering units.
4. Effectors, which establishes communications over SBUS and PWM protocols.
5. Telemetry, which establishing communications with the radio modem.
6. Datalog, which checks for an SD card present and creates a datalog file.

After a succesful boot, a low priority loop is established to write datalog entries from a buffer to the SD card. An interrupt is attached to the IMU data ready pin to trigger the main flight software loop at the desired frame rate.

The main flight software loop consists of:
1. Reading system data: system time, frame duration, and input, regulated, and servo voltages.
2. Reading sensor data, correcting scale factors and biases, and rotating sensor data into the vehicle frame.
3. Running the navigation filter to filter the sensor data and estimate the aircraft states.
4. Run the control software.
5. Convert effector commands from engineering units to PWM and SBUS values.
6. Add data to the datalog buffer.
7. Send updated telemetry data. Check for updated in-flight-tunable parameters, flight plans, fences, and rally points.

A timer to send commands to the effectors is started by the main flight software loop and triggers at 90% of the frame duration. On this trigger, the effector commands are sent to the effectors. This approach provides a fixed latency between sensing and actuation for developing robust control laws.

This process continues until the system is powered down.

# Developing Software
Software for SPAARO can be developed in C++ or autocoded from Simulink. The input plane has the following data available:

   * System Data:
      * int32_t frame_time_us: time the previous frame took to complete, us. Useful for analyzing CPU load.
      * float input_volt (*FMU-R v1.x*): the input voltage to the voltage regulator.
      * float reg_volt (*FMU-R v1.x*): the regulated voltage.
      * float pwm_volt (*FMU-R v1.x*): the PWM servo rail voltage.
      * float sbus_volt (*FMU-R v1.x*): the SBUS servo rail voltage.
      * int64_t sys_time_us: the time since boot, us.
   * Sensor Data:
      * bool pitot_static_installed: whether a pitot-static probe and air data sensor are installed.
      * Inceptor Data:
         * bool new_data: whether new data was received by the SBUS receiver.
         * bool lost_frame: whether a frame of SBUS data was lost by the receiver.
         * bool failsafe: whether the SBUS receiver has entered failsafe mode - this typically occurs if many frames of data are lost in a row.
         * bool ch17 | ch18: some SBUS transmitters and receivers support two boolean outputs, CH 17 and CH 18, which are available here.
         * int16_t ch[16]: SBUS channel values. SBUS is 11 bits with a range of 0 - 2048. Some SBUS receivers, such as FrSky, use a default range of 172 - 1811, unless an extended range is configured.
      * IMU Data:
         * bool new_imu_data: whether new data was received from the accelerometer and gyro.
         * bool new_mag_data: whether new data was received from the magnetometer.
         * bool imu_healthy: whether the accelerometer and gyro are healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * bool mag_healthy: whether the magnetometer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * float die_temp_c: the IMU die temperature, C.
         * float accel_mps2[3]: the accelerometer data, with bias and scale factor corrected, and rotated into the vehicle frame, m/s/s [x y z].
         * float gyro_radps[3]: the gyro data, with bias corrected, and rotated into the vehicle frame, rad/s [x y z].
         * float mag_ut[3]: the magnetometer data, with bias and scale factor corrected, and rotated into the vehicle frame, uT [x y z].
      * GNSS Data:
         * bool new_data: whether new data was received by the GNSS receiver.
         * bool healthy: whether the GNSS receiver is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * int8_t fix: the GNSS fix type:
            * 1: No fix
            * 2: 2D fix
            * 3: 3D fix
            * 4: 3D fix with differential GNSS
            * 5: 3D fix, RTK with floating integer ambiguity
            * 6: 3D fix, RTK with fixed integer ambiguity
         * int8_t num_sats: the number of satellites used in the GNSS solution.
         * int16_t week: GNSS week number.
         * int32_t tow_ms: GNSS time of week, ms.
         * float alt_wgs84_m: Altitude above the WGS84 ellipsoid, m.
         * float alt_msl_m: Altitude above Mean Sea Level (MSL), m.
         * float hdop: horizontal dilution of precision.
         * float vdop: vertical dilution of precision.
         * float track_rad: ground track, rad.
         * float spd_mps: ground speed, m/s.
         * float horz_acc_m: estimated horizontal position accuracy, m.
         * float vert_acc_m: estimated vertical position accuracy, m.
         * float vel_acc_mps: estimated velocity accuracy, m/s.
         * float track_acc_rad: estimated track accuracy, rad.
         * float ned_vel_mps[3]: north east down velocity, m/s [North East Down].
         * double lat_rad: latitude, rad.
         * double lon_rad: longitude, rad.
      * Static Pressure Data:
         * bool new_data: whether new data was received from the pressure transducer.
         * bool healthy: whether the pressure transducer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * float pres_pa: the measured pressure, Pa.
         * float die_temp_c: the pressure transducer die temperature, C.
      * Differential Pressure Data:
         * bool new_data: whether new data was received from the pressure transducer.
         * bool healthy: whether the pressure transducer is healthy. Unhealthy is defined as missing 5 frames of data in a row at the expected rate.
         * float pres_pa: the measured pressure, Pa.
         * float die_temp_c: the pressure transducer die temperature, C.
      * ADC Data:
         * float volt[2(*FMU-R v1.x*)/8(*FMU-R v2.x*)]: voltages measured by the FMU analog to digital converters
      * Power Module Data (*FMU-R v2.x*):
         * float voltage_v: voltage measured on the power port voltage pin. Note that this is not the battery pack voltage, typically this value needs to be scaled by the power module volts / volt value and is power module specific.
         * float current_v: voltage measured on the power port current pin. Typically this is scaled by the power module mA / volt value and is power module specific.
   * Navigation Filter Data:
      * bool nav_initialized: whether the navigation filter has been initialized. Do not use navigation filter data before it has been initialized. Requires a good GNSS solution to complete the initialization process.
      * float pitch_rad: pitch angle, rad.
      * float roll_rad: roll angle, rad.
      * float heading_rad: heading angle relative to true north, rad.
      * float alt_wgs84_m: altitude above the WGS84 ellipsoid, m.
      * float home_alt_wgs84_m: home location (i.e. origin of the NED position) above the WGS84 ellipsoid, m.
      * float alt_msl_m: altitude above Mean Sea Level (MSL), m.
      * float alt_rel_m: altitude above where the navigation filter was initialized, m.
      * float static_pres_pa: filtered static pressure, Pa.
      * float diff_pres_pa: filtered differential pressure, Pa.
      * float alt_pres_m: pressure altitude, m.
      * float ias_mps: indicated airspeed, m/s.
      * float gnd_spd_mps: ground speed, m/s.
      * float gnd_track_rad: ground track, rad.
      * float flight_path_rad: flight path angle, rad.
      * float accel_bias_mps2[3]: accelerometer bias estimate from the EKF, m/s/s [x y z].
      * float gyro_bias_radps[3]: gyro bias estimate from the EKF, rad/s [x y z].
      * float accel_mps2[3]: IMU acceleterometer data with the EKF estimated biases removed and digital low pass filtereing applied, m/s/s [x y z].
      * float gyro_radps[3]: IMU gyro data with the EKF estimated biases removed and digital low pass filtereing applied, rad/s [x y z].
      * float mag_ut[3]: IMU magnetometer data with digital low pass filtering applied, uT [x y z].
      * float ned_pos_m[3]: North east down position relative to where the navigation filter was initialized, m [north east down].
      * float ned_vel_mps[3]: North east down ground velocity, m/s [north east down].
      * double lat_rad: latitude, rad.
      * double lon_rad: longitude, rad.
      * double home_lat_rad: home location (i.e. origin of the NED position) latitude, rad.
      * double home_lon_rad: home location (i.e. origin of the NED position) longitude, rad.
   * Telemetry Data:
      * bool waypoints_updated: whether the flight plan waypoints have been updated.
      * bool fence_updated: whether the fence has been updated.
      * bool rally_points_updated: whether the rally points have been updated.
      * int16_t current_waypoint: the index of the current waypoint.
      * int16_t num_waypoints: the number of waypoints in the current flight plan.
      * int16_t num_fence_items: the number of fence items.
      * int16_t num_rally_points: the number of rally points.
      * std::array<float, NUM_TELEM_PARAMS> param: an array of in-flight-tunable parameters sent from the ground station. NUM_TELEM_PARAMS defines the number of parameters available, typically 24. These parameters can be used for anything that might be adjusted in flight, such as controlling gains, selecting excitation waveforms, etc.
      * std::array<bfs::MissionItem, NUM_FLIGHT_PLAN_POINTS> flight_plan: an array storing all of the waypoints in the flight plan. NUM_FLIGHT_PLAN_POINTS defines the maximum number of waypoints that can be stored, num_waypoints is the number of waypoints currently stored, and current_waypoint is the 0-based index of the current waypoint.
      * std::array<bfs::MissionItem, NUM_FENCE_POINTS> fence: an array storing all of the fence items. NUM_FENCE_POINTS defines the maximum number of fence items that can be stored, num_fence_items is the number of fence items currently stored.
      * std::array<bfs::MissionItem, NUM_RALLY_POINTS> rally: an array storing all of the rally points. NUM_RALLY_POINTS defines the maximum number of rally points that can be stored, num_rally_points is the number of rally points currently stored.
   * Analog Data (*FMU-R v2.x*):
      * std::array<float, NUM_AIN_PINS> volt: measured voltages from the analog to digital converters
      * std::array<float, NUM_AIN_PINS> val: voltages converted to engineering units.
   * Battery Data (*FMU-R v2.x*):
      * float voltage_v: battery voltage
      * float current_ma: battery current, mA.
      * float consumed_mah: battery capacity consumed, mAh.
      * float remaining_prcnt: battery capacity remaining, % (i.e. 75 for 75%).
      * float remaining_time_s: estimated flight time remaining, s.

Mission Items are defined as:

   * bool autocontinue: hether to automatically continue to the next MissionItem
   * uint8_t frame: the [coordinate frame](https://mavlink.io/en/messages/common.html#MAV_FRAME) of the MissionItem
   * uint16_t cmd: the [command](https://mavlink.io/en/messages/common.html#mav_commands) associated with the MissionItem
   * float param1: command dependent parameter
   * float param2: command dependent parameter
   * float param3: command dependent parameter
   * float param4: command dependent parameter
   * int32_t x: typically latitude represented as 1e7 degrees
   * int32_t y: typically longitude represented as 1e7 degrees
   * float z: typically altitude, but can be dependent on the command and frame

The output plane is defined as:

   * VMS Data:
      * bool motors_enabled: whether the motors are enabled and can turn. This is not a command, rather just feedback provided from the VMS about whether the motors are "hot" and is used in telemetry and for operator situation awareness.
      * bool waypoint_reached: whether the current waypoint has been reached. This is used to indicate to the ground station that the active waypoint should be advanced to the next in the flight plan.
      * int8_t mode: the current aircraft mode:
         * 0: manual flight mode.
         * 1: stability augmented flight mode.
         * 2: attitude feedback flight mode.
         * 3: autonomous flight mode.
         * 4: test point / research flight mode.
      * float throttle_cmd_prcnt: the throttle command given as a %, this is used for telemetry and situational awareness.
      * std::array<float, NUM_AUX_VAR> aux: aux variables - these are undefined and can be used by the developer to output data for logging. Useful for logging internal control law states, research variables, or other values of interest. NUM_AUX_VAR defines the number of channels available, currently 24.
      * SbusCmd:
         * bool ch17: output command to SBUS CH 17.
         * bool ch18: output command to SBUS CH 18.
         * float cmd[16]: angle or PLA commands to SBUS channels. This is used to drive the simulation.
         * int16_t cnt[16]: raw SBUS counts to SBUS channels. This is sent to the aircraft effectors. Typically a polynomial evaluation would be used to convert from an angle command (i.e. an aileron deflection) to raw SBUS output.
      * PwmCmd:
         * float cmd[8]: angle or PLA commands to PWM channels. This is used to drive the simulation.
         * int16_t cnt[8]: raw PWM counts to PWM channels. This is sent to the aircraft effectors. Typically a polynomial evaluation would be used to convert from an angle command (i.e. an aileron deflection) to raw PWM output.
      * Analog:
         * float val[2(*FMU-R v1.x*)/8(*FMU-R v2.x*)]: ADC voltages converted to engineering units (i.e. POT voltage to control surface deflection).
      * Battery (*FMU-R v2.x*):
         * float voltage_v: battery pack voltage.
         * float current_ma: battery pack current draw, mA.
         * float consumed_mah: battery pack capacity consumed, mAh.
         * float remaining_prcnt: battery pack capacity remaining, %.
         * float remaining_time_s: estimated flight time remaining, s.

## C++
C++ software should be developed in */flight_code/flight/control.cc*. [Filters](https://github.com/bolderflight/filter), [control algorithm](https://github.com/bolderflight/control) templates, and [excitations](https://github.com/bolderflight/excitation/) are available to ease the development effort. An init function, *ControlInit*, is provided and is run once as the system boots. The *ControlRun* function is run every frame.

## Cite

Please cite the LAGER SPAARO framework as follows
```
    @Misc{lager_spaaro,
        author       = {Brian Taylor and Jordan D. Larson and Tuan D. Luong and Aabhash Bhandari and Ryan W. Thomas},
        howpublished = {Web page},
        title        = {{lager_spaaro}: {S}imulink/C++ {P}latform for {A}eronautics and {A}utonomy {R}esearch and {O}perations},
        year         = {2023},
        url          = {https://github.com/drjdlarson/lager_spaaro},
    }
```

<!-- # Simulation

# Analyzing Data -->
