import struct
import ctypes
import json


class Packet(ctypes.LittleEndianStructure):
    """The base packet class for API version F1 22"""
    _pack_ = 1

    def to_dict(self):
        return {field[0]: self._convert_to_python_type(getattr(self, field[0])) for field in self._fields_}

    def _convert_to_python_type(self, value):
        if isinstance(value, (ctypes.Array, ctypes.Structure)):
            return value.to_dict() if hasattr(value, 'to_dict') else str(value)
        return value

    def __repr__(self):
        return json.dumps(self.to_dict(), indent=4)


class PacketHeader(Packet):
    _fields_ = [
        ("packet_format", ctypes.c_int16),  # 2023
        ("game_year", ctypes.c_uint8),  # Game year - last two digits e.g. 23
        ("game_major_version", ctypes.c_uint8),  # Game major version - "X.00"
        ("game_minor_version", ctypes.c_uint8),  # Game minor version - "1.XX"
        ("packet_version", ctypes.c_uint8),  # Version of this packet type, all start from 1
        ("packet_id", ctypes.c_uint8),  # Identifier for the packet type
        ("session_uid", ctypes.c_uint64),  # unique identifier for the session
        ("session_time", ctypes.c_float),  # Session timestamp
        ("frame_identifier", ctypes.c_uint32),  # Identifier for the frame the data was retrieved on
        ("overall_frame_identifier", ctypes.c_uint32),  # Overall identifier for the frame the data was retrieved
        # on, doesn't go back after flashbacks
        ("player_car_index", ctypes.c_uint8),  # Index of player's car in the array
        ("secondary_player_car_index", ctypes.c_uint8),  # Index of secondary player's car in the array (splitscreen)
        # 255 if no second player
    ]


class LapData(Packet):
    _fields_ = [
        ("last_lap_time_in_ms", ctypes.c_uint32),  # Last lap time in milliseconds
        ("current_lap_time_in_ms", ctypes.c_uint32),  # Current time around the lap in milliseconds
        ("sector_1_time_in_ms", ctypes.c_uint16),  # Sector 1 time in milliseconds
        ("sector_1_time_minutes", ctypes.c_uint8),  # Sector 1 whole minute part
        ("sector_2_time_in_ms", ctypes.c_uint16),  # Sector 2 time in milliseconds
        ("sector_2_time_minutes", ctypes.c_uint8),  # Sector 2 whole minute part
        ("delta_to_car_in_front_in_ms", ctypes.c_uint16),  # Time delta to car in front milliseconds part
        ("delta_to_car_in_front_minutes", ctypes.c_uint8),  # Time delta to car in front whole minute part
        ("delta_to_race_leader_in_ms", ctypes.c_uint16),  # Time delta to race leader milliseconds part
        ("delta_to_race_leader_minutes", ctypes.c_uint8),  # Time delta to race leader whole minute part
        ("lap_distance", ctypes.c_float),  # Distance vehicle is around current lap in metres – could
        # be negative if line hasn’t been crossed yet
        ("total_distance", ctypes.c_float),  # Total distance travelled in session in metres – could
        # be negative if line hasn’t been crossed yet
        ("safety_car_delta", ctypes.c_float),  # Delta in seconds for safety car
        ("car_position", ctypes.c_uint8),  # Car race position
        ("current_lap_num", ctypes.c_uint8),  # Current lap number
        ("pit_status", ctypes.c_uint8),  # 0 = none, 1 = pitting, 2 = in pit area
        ("num_pit_stops", ctypes.c_uint8),  # Number of pit stops taken in this race
        ("sector", ctypes.c_uint8),  # 0 = sector1, 1 = sector2, 2 = sector3
        ("current_lap_invalid", ctypes.c_uint8),
        # Current lap invalid - 0 = valid, 1 = invalid
        ("penalties", ctypes.c_uint8),
        # Accumulated time penalties in seconds to be added
        ("total_warnings", ctypes.c_uint8),  # Accumulated number of warnings issued
        ("corner_cutting_warnings", ctypes.c_uint8),  # Accumulated number of corner cutting warnings issued
        ("num_unserved_drive_through_pens", ctypes.c_uint8),  # Num drive through pens left to serve
        ("num_unserved_stop_go_pens", ctypes.c_uint8),  # Num stop go pens left to serve
        ("grid_position", ctypes.c_uint8),  # Grid position the vehicle started the race in
        ("driver_status", ctypes.c_uint8),  # Status of driver - 0 = in garage, 1 = flying lap # 2 = in lap,
        # 3 = out lap, 4 = on track
        ("result_status", ctypes.c_uint8),  # Result status - 0 = invalid, 1 = inactive, 2 = active
        # 3 = finished, 4 = didnotfinish, 5 = disqualified
        # 6 = not classified, 7 = retired
        ("pit_lane_timer_active", ctypes.c_uint8),  # Pit lane timing, 0 = inactive, 1 = active
        ("pit_lane_time_in_lane_in_ms", ctypes.c_uint16),  # If active, the current time spent in the pit lane in ms
        ("pit_stop_timer_in_ms", ctypes.c_uint16),  # Time of the actual pit stop in ms
        ("pit_stop_should_serve_pen", ctypes.c_uint8),  # Whether the car should serve a penalty at this stop
        ("speed_trap_fastest_speed", ctypes.c_float),  # Fastest speed through speed trap for this car in kmph
        ("speed_trap_fastest_lap", ctypes.c_uint8),  # Lap no the fastest speed was achieved, 255 = not set
    ]


class PacketLapData(Packet):
    _fields_ = [
        ("header", PacketHeader),  # Header
        ("lap_data", LapData * 22),  # Lap data for all cars on track
        ("time_trial_pb_car_idx", ctypes.c_uint8),  # Index of Personal Best car in time trial (255 if invalid)
        ("time_trial_rival_car_idx", ctypes.c_uint8),  # Index of Rival car in time trial (255 if invalid)
    ]


class CarMotionData(Packet):
    _fields_ = [
        ("world_position_x", ctypes.c_float),  # World space X position
        ("world_position_y", ctypes.c_float),  # World space Y position
        ("world_position_z", ctypes.c_float),  # World space Z position
        ("world_velocity_x", ctypes.c_float),  # Velocity in world space X
        ("world_velocity_y", ctypes.c_float),  # Velocity in world space Y
        ("world_velocity_z", ctypes.c_float),  # Velocity in world space Z
        ("world_forward_dir_x", ctypes.c_int16),  # World space forward X direction (normalised)
        ("world_forward_dir_y", ctypes.c_int16),  # World space forward Y direction (normalised)
        ("world_forward_dir_z", ctypes.c_int16),  # World space forward Z direction (normalised)
        ("world_right_dir_x", ctypes.c_int16),  # World space right X direction (normalised)
        ("world_right_dir_y", ctypes.c_int16),  # World space right Y direction (normalised)
        ("world_right_dir_z", ctypes.c_int16),  # World space right Z direction (normalised)
        ("g_force_lateral", ctypes.c_float),  # Lateral G-Force component
        ("g_force_longitudinal", ctypes.c_float),  # Longitudinal G-Force component
        ("g_force_vertical", ctypes.c_float),  # Vertical G-Force component
        ("yaw", ctypes.c_float),  # Yaw angle in radians
        ("pitch", ctypes.c_float),  # Pitch angle in radians
        ("roll", ctypes.c_float),  # Roll angle in radians
    ]


class PacketMotionData(Packet):
    _fields_ = [
        ("header", PacketHeader),  # Header
        ("car_motion_data", CarMotionData * 22)  # Data for car motion of all cars on track
    ]


class PacketMotionExData(Packet):
    _fields_ = [
        ("header", PacketHeader),  # Header
        ("m_suspensionPosition", ctypes.c_float * 4),  # Note: All wheel arrays have the following order:
        ("m_suspensionVelocity", ctypes.c_float * 4),  # RL, RR, FL, FR
        ("m_suspensionAcceleration", ctypes.c_float * 4),  # RL, RR, FL, FR
        ("m_wheelSpeed", ctypes.c_float * 4),  # Speed of each wheel
        ("m_wheelSlipRatio", ctypes.c_float * 4),  # Slip ratio for each wheel
        ("m_wheelSlipAngle", ctypes.c_float * 4),  # Slip angles for each wheel
        ("m_wheelLatForce", ctypes.c_float * 4),  # Lateral forces for each wheel
        ("m_wheelLongForce", ctypes.c_float * 4),  # Longitudinal forces for each wheel
        ("m_heightOfCOGAboveGround", ctypes.c_float),  # Height of centre of gravity above ground
        ("m_localVelocityX", ctypes.c_float),  # Velocity in local space – metres/s
        ("m_localVelocityY", ctypes.c_float),  # Velocity in local space
        ("m_localVelocityZ", ctypes.c_float),  # Velocity in local space
        ("m_angularVelocityX", ctypes.c_float),  # Angular velocity x-component – radians/s
        ("m_angularVelocityY", ctypes.c_float),  # Angular velocity y-component
        ("m_angularVelocityZ", ctypes.c_float),  # Angular velocity z-component
        ("m_angularAccelerationX", ctypes.c_float),  # Angular acceleration x-component – radians/s/s
        ("m_angularAccelerationY", ctypes.c_float),  # Angular acceleration y-component
        ("m_angularAccelerationZ", ctypes.c_float),  # Angular acceleration z-component
        ("m_frontWheelsAngle", ctypes.c_float),  # Current front wheels angle in radians
        ("m_wheelVertForce", ctypes.c_float),  # Vertical forces for each wheel
        ("front_aero_height", ctypes.c_float),  # Front plank edge height above road surface
        ("rear_aero_height", ctypes.c_float),  # Rear plank edge height above road surface
        ("front_roll_angle", ctypes.c_float),  # Roll angle of the front suspension
        ("rear_roll_angle", ctypes.c_float),   # Roll angle of the rear suspension
        ("chassis_yaw", ctypes.c_float),       # Yaw angle of the chassis relative to the direction
    ]


class CarTelemetryData(Packet):
    _fields_ = [
        ("speed", ctypes.c_uint16),  # Speed of car in kilometres per hour
        ("throttle", ctypes.c_float),  # Amount of throttle applied (0.0 to 1.0)
        ("steer", ctypes.c_float),  # Steering (-1.0 (full lock left) to 1.0 (full lock right))
        ("brake", ctypes.c_float),  # Amount of brake applied (0.0 to 1.0)
        ("clutch", ctypes.c_uint8),  # Amount of clutch applied (0 to 100)
        ("gear", ctypes.c_int8),  # Gear selected (1-8, N=0, R=-1)
        ("engine_rpm", ctypes.c_uint16),  # Engine RPM
        ("drs", ctypes.c_uint8),  # 0 = off, 1 = on
        ("rev_lights_percent", ctypes.c_uint8),  # Rev lights indicator (percentage)
        ("rev_lights_bit_value", ctypes.c_uint16),  # Rev lights (bit 0 = leftmost LED, bit 14 = rightmost LED)
        ("brakes_temperature", ctypes.c_uint16 * 4),  # Brakes temperature (celsius)
        ("tyres_surface_temperature", ctypes.c_uint8 * 4),  # Tyres surface temperature (celsius)
        ("tyres_inner_temperature", ctypes.c_uint8 * 4),  # Tyres inner temperature (celsius)
        ("engine_temperature", ctypes.c_uint16),  # Engine temperature (celsius)
        ("tyres_pressure", ctypes.c_float * 4),  # Tyres pressure (PSI)
        ("surface_type", ctypes.c_uint8 * 4),  # Driving surface, see appendices
    ]


class PacketCarTelemetryData(Packet):
    _fields_ = [
        ("header", PacketHeader),  # Header
        ("car_telemetry_data", CarTelemetryData * 22),
        ("mfd_panel_index", ctypes.c_uint8),  # Index of MFD panel open - 255 = MFD closed
        # Single player, race – 0 = Car setup, 1 = Pits
        # 2 = Damage, 3 =  Engine, 4 = Temperatures
        # May vary depending on game mode
        ("mfd_panel_index_secondary_player", ctypes.c_uint8),  # See above
        ("suggested_gear", ctypes.c_int8),  # Suggested gear for the player (1-8), 0 if no gear suggested
    ]


class CarStatusData(Packet):

    _fields_ = [
        ("traction_control", ctypes.c_uint8),  # Traction control - 0 = off, 1 = medium, 2 = full
        ("anti_lock_brakes", ctypes.c_uint8),  # 0 (off) - 1 (on)
        ("fuel_mix", ctypes.c_uint8),  # Fuel mix - 0 = lean, 1 = standard, 2 = rich, 3 = max
        ("front_brake_bias", ctypes.c_uint8),  # Front brake bias (percentage)
        ("pit_limiter_status", ctypes.c_uint8),  # Pit limiter status - 0 = off, 1 = on
        ("fuel_in_tank", ctypes.c_float),  # Current fuel mass
        ("fuel_capacity", ctypes.c_float),  # Fuel capacity
        ("fuel_remaining_laps", ctypes.c_float),  # Fuel remaining in terms of laps (value on MFD)
        ("max_rpm", ctypes.c_uint16),  # Cars max RPM, point of rev limiter
        ("idle_rpm", ctypes.c_uint16),  # Cars idle RPM
        ("max_gears", ctypes.c_uint8),  # Maximum number of gears
        ("drs_allowed", ctypes.c_uint8),  # 0 = not allowed, 1 = allowed
        ("drs_activation_distance", ctypes.c_uint16),  # 0 = DRS not available, non-zero - DRS will be available
        # in [X] metres
        ("actual_tyre_compound", ctypes.c_uint8),  # F1 Modern - 16 = C5, 17 = C4, 18 = C3, 19 = C2, 20 = C1
        # 21 = C0, 7 = inter, 8 = wet
        # F1 Classic - 9 = dry, 10 = wet
        # F2 – 11 = super soft, 12 = soft, 13 = medium, 14 = hard
        # 15 = wet
        ("visual_tyre_compound", ctypes.c_uint8),  # F1 visual (can be different from actual compound)
        # 16 = soft, 17 = medium, 18 = hard, 7 = inter, 8 = wet
        # F1 Classic – same as above
        # F2 ‘19, 15 = wet, 19 – super soft, 20 = soft
        # 21 = medium , 22 = hard
        ("tyres_age_laps", ctypes.c_uint8),  # Age in laps of the current set of tyres
        ("vehicle_fia_flags", ctypes.c_int8),  # -1 = invalid/unknown, 0 = none, 1 = green 2 = blue, 3 = yellow, 4 = red
        ("engine_power_ice", ctypes.c_float),  # Engine power output of ICE (W)
        ("engine_power_mguk", ctypes.c_float),  # Engine power output of MGU-K (W)
        ("ers_store_energy", ctypes.c_float),  # ERS energy store in Joules
        ("ers_deploy_mode", ctypes.c_uint8),  # ERS deployment mode, 0 = none, 1 = medium 2 = hotlap, 3 = overtake
        ("ers_harvested_this_lap_mguk", ctypes.c_float),  # ERS energy harvested this lap by MGU-K
        ("ers_harvested_this_lap_mguh", ctypes.c_float),  # ERS energy harvested this lap by MGU-H
        ("ers_deployed_this_lap", ctypes.c_float),  # ERS energy deployed this lap
        (
            "network_paused",
            ctypes.c_uint8,
        ),  # Whether the car is paused in a network game
    ]


class PacketCarStatusData(Packet):
    """
    struct PacketCarStatusData
    {
        PacketHeader m_header; // Header
        CarStatusData m_carStatusData[22];
    };
    """

    _fields_ = [
        ("header", PacketHeader),  # Header
        ("car_status_data", CarStatusData * 22),
    ]


class TimeTrialDataSet(Packet):
    """
    The time trial data gives extra information only relevant to time trial game mode. This packet will not be sent in other game modes. 
    Frequency: 1 per second
    Size: 101 bytes
    Version: 1
    struct TimeTrialDataSet
    {
        uint8   m_carIdx;              // Index of the car this data relates to
        uint8   m_teamId;              // Team id - see appendix
        uint32  m_lapTimeInMS;         // Lap time in milliseconds
        uint32  m_sector1TimeInMS;     // Sector 1 time in milliseconds
        uint32  m_sector2TimeInMS;     // Sector 2 time in milliseconds
        uint32  m_sector3TimeInMS;     // Sector 3 time in milliseconds
        uint8   m_tractionControl;     // 0 = off, 1 = medium, 2 = full
        uint8   m_gearboxAssist;       // 1 = manual, 2 = manual & suggested gear, 3 = auto
        uint8   m_antiLockBrakes;      // 0 (off) - 1 (on)
        uint8   m_equalCarPerformance; // 0 = Realistic, 1 = Equal
        uint8   m_customSetup;         // 0 = No, 1 = Yes
        uint8   m_valid;               // 0 = invalid, 1 = valid
    };
    """
    _fields_ = [
        ('car_idx', ctypes.c_uint8),               # Index of the car this data relates to
        ('team_id', ctypes.c_uint8),               # Team id - see appendix
        ('lap_time_in_ms', ctypes.c_uint32),       # Lap time in milliseconds
        ('sector1_time_in_ms', ctypes.c_uint32),   # Sector 1 time in milliseconds
        ('sector2_time_in_ms', ctypes.c_uint32),   # Sector 2 time in milliseconds
        ('sector3_time_in_ms', ctypes.c_uint32),   # Sector 3 time in milliseconds
        ('traction_control', ctypes.c_uint8),      # 0 = off, 1 = medium, 2 = full
        ('gearbox_assist', ctypes.c_uint8),        # 1 = manual, 2 = manual & suggested gear, 3 = auto
        ('anti_lock_brakes', ctypes.c_uint8),      # 0 (off) - 1 (on)
        ('equal_car_performance', ctypes.c_uint8),  # 0 = Realistic, 1 = Equal
        ('custom_setup', ctypes.c_uint8),          # 0 = No, 1 = Yes
        ('valid', ctypes.c_uint8),                 # 0 = invalid, 1 = valid
    ]


class PacketTimeTrialData(Packet):
    """
    struct PacketTimeTrialData
    {
        PacketHeader        m_header;                    // Header

        TimeTrialDataSet    m_playerSessionBestDataSet;  // Player session best data set
        TimeTrialDataSet    m_personalBestDataSet;       // Personal best data set
        TimeTrialDataSet    m_rivalDataSet;              // Rival data set
    };
    """
    _fields_ = [
        ("header", PacketHeader),  # Header
        ('player_session_best_data_set', TimeTrialDataSet),  # Player session best data set
        ('personal_best_data_set', TimeTrialDataSet),       # Personal best data set
        ('rival_data_set', TimeTrialDataSet),               # Rival data set
    ]


HEADER_FIELD_TO_PACKET_TYPE = {
    (2024, 1, 0): PacketMotionData,
    # (2024, 1, 1): PacketSessionData,
    (2024, 1, 2): PacketLapData,
    # (2024, 1, 3): PacketEventData,
    # (2024, 1, 4): PacketParticipantsData,
    # (2024, 1, 5): PacketCarSetupData,
    # (2024, 1, 6): PacketCarTelemetryData,
    # (2024, 1, 7): PacketCarStatusData,
    # (2024, 1, 8): PacketFinalClassificationData,
    # (2024, 1, 9): PacketLobbyInfoData,
    # (2024, 1, 10): PacketCarDamageData,
    # (2024, 1, 11): PacketSessionHistoryData,
    # (2024, 1, 12): PacketTyreSetsData,
    (2024, 1, 13): PacketMotionExData,
    (2024, 1, 14): PacketTimeTrialData,
}
