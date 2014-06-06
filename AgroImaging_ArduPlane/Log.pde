// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if LOGGING_ENABLED == ENABLED

// Code to Write and Read packets from DataFlash.log memory
// Code to interact with the user to dump or erase logs

// These are function definitions so the Menu can be constructed before the functions
// are defined below. Order matters to the compiler.
static int8_t   dump_log(uint8_t argc,                  const Menu::arg *argv);
static int8_t   erase_logs(uint8_t argc,                const Menu::arg *argv);
static int8_t   select_logs(uint8_t argc,               const Menu::arg *argv);

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Coommon for implementation details
static const struct Menu::command log_menu_commands[] PROGMEM = {
    {"dump",        dump_log},
    {"erase",       erase_logs},
    {"enable",      select_logs},
    {"disable",     select_logs}
};

// A Macro to create the Menu
MENU2(log_menu, "Log", log_menu_commands, print_log_menu);

static bool
print_log_menu(void)
{
    cliSerial->println_P(PSTR("logs enabled: "));

    if (0 == g.log_bitmask) {
        cliSerial->println_P(PSTR("none"));
    }else{
        // Macro to make the following code a bit easier on the eye.
        // Pass it the capitalised name of the log option, as defined
        // in defines.h but without the LOG_ prefix.  It will check for
        // the bit being set and print the name of the log option to suit.
 #define PLOG(_s) if (g.log_bitmask & MASK_LOG_ ## _s) cliSerial->printf_P(PSTR(" %S"), PSTR(# _s))
        PLOG(ATTITUDE_FAST);
        PLOG(ATTITUDE_MED);
        PLOG(GPS);
        PLOG(PM);
        PLOG(CTUN);
        PLOG(NTUN);
        PLOG(MODE);
        PLOG(IMU);
        PLOG(CMD);
        PLOG(CURRENT);
        PLOG(COMPASS);
        PLOG(TECS);
        PLOG(CAMERA);
        PLOG(RC);
        PLOG(SONAR);
 #undef PLOG
    }

    cliSerial->println();

    DataFlash.ListAvailableLogs(cliSerial);
    return(true);
}

static int8_t
dump_log(uint8_t argc, const Menu::arg *argv)
{
    int16_t dump_log;
    uint16_t dump_log_start;
    uint16_t dump_log_end;
    uint16_t last_log_num;

    // check that the requested log number can be read
    dump_log = argv[1].i;
    last_log_num = DataFlash.find_last_log();

    if (dump_log == -2) {
        DataFlash.DumpPageInfo(cliSerial);
        return(-1);
    } else if (dump_log <= 0) {
        cliSerial->printf_P(PSTR("dumping all\n"));
        Log_Read(0, 1, 0);
        return(-1);
    } else if ((argc != 2)
               || ((uint16_t)dump_log > last_log_num))
    {
        cliSerial->printf_P(PSTR("bad log number\n"));
        return(-1);
    }

    DataFlash.get_log_boundaries(dump_log, dump_log_start, dump_log_end);
    Log_Read((uint16_t)dump_log, dump_log_start, dump_log_end);
    return 0;
}

static void do_erase_logs(void)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Erasing logs"));
    DataFlash.EraseAll();
    gcs_send_text_P(SEVERITY_LOW, PSTR("Log erase complete"));
}

static int8_t
erase_logs(uint8_t argc, const Menu::arg *argv)
{
    in_mavlink_delay = true;
    do_erase_logs();
    in_mavlink_delay = false;
    return 0;
}

static int8_t
select_logs(uint8_t argc, const Menu::arg *argv)
{
    uint32_t bits;

    if (argc != 2) {
        cliSerial->printf_P(PSTR("missing log type\n"));
        return(-1);
    }

    bits = 0;

    // Macro to make the following code a bit easier on the eye.
    // Pass it the capitalised name of the log option, as defined
    // in defines.h but without the LOG_ prefix.  It will check for
    // that name as the argument to the command, and set the bit in
    // bits accordingly.
    //
    if (!strcasecmp_P(argv[1].str, PSTR("all"))) {
        bits = 0xFFFFFFFFUL;
    } else {
 #define TARG(_s)        if (!strcasecmp_P(argv[1].str, PSTR(# _s))) bits |= MASK_LOG_ ## _s
        TARG(ATTITUDE_FAST);
        TARG(ATTITUDE_MED);
        TARG(GPS);
        TARG(PM);
        TARG(CTUN);
        TARG(NTUN);
        TARG(MODE);
        TARG(IMU);
        TARG(CMD);
        TARG(CURRENT);
        TARG(COMPASS);
        TARG(TECS);
        TARG(CAMERA);
        TARG(RC);
        TARG(SONAR);
 #undef TARG
    }

    if (!strcasecmp_P(argv[0].str, PSTR("enable"))) {
        g.log_bitmask.set_and_save(g.log_bitmask | bits);
    }else{
        g.log_bitmask.set_and_save(g.log_bitmask & ~bits);
    }
    return(0);
}

static int8_t
process_logs(uint8_t argc, const Menu::arg *argv)
{
    log_menu.run();
    return 0;
}

struct PACKED log_Attitude {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    uint16_t error_rp;
    uint16_t error_yaw;
};

// Write an attitude packet
static void Log_Write_Attitude(void)
{
    struct log_Attitude pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
        time_ms : hal.scheduler->millis(),
        roll  : (int16_t)ahrs.roll_sensor,
        pitch : (int16_t)ahrs.pitch_sensor,
        yaw   : (uint16_t)ahrs.yaw_sensor,
        error_rp  : (uint16_t)(ahrs.get_error_rp() * 100),
        error_yaw : (uint16_t)(ahrs.get_error_yaw() * 100)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

#if AP_AHRS_NAVEKF_AVAILABLE
    DataFlash.Log_Write_EKF(ahrs);
    DataFlash.Log_Write_AHRS2(ahrs);
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    sitl.Log_Write_SIMSTATE(DataFlash);
#endif
}


struct PACKED log_Performance {
    LOG_PACKET_HEADER;
    uint32_t loop_time;
    uint16_t main_loop_count;
    uint32_t g_dt_max;
    int16_t  gyro_drift_x;
    int16_t  gyro_drift_y;
    int16_t  gyro_drift_z;
    uint8_t  i2c_lockup_count;
    uint16_t ins_error_count;
};

// Write a performance monitoring packet. Total length : 19 bytes
static void Log_Write_Performance()
{
    struct log_Performance pkt = {
        LOG_PACKET_HEADER_INIT(LOG_PERFORMANCE_MSG),
        loop_time       : millis() - perf_mon_timer,
        main_loop_count : mainLoop_count,
        g_dt_max        : G_Dt_max,
        gyro_drift_x    : (int16_t)(ahrs.get_gyro_drift().x * 1000),
        gyro_drift_y    : (int16_t)(ahrs.get_gyro_drift().y * 1000),
        gyro_drift_z    : (int16_t)(ahrs.get_gyro_drift().z * 1000),
        i2c_lockup_count: hal.i2c->lockup_count(),
        ins_error_count  : ins.error_count()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write a mission command. Total length : 36 bytes
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd)
{
    mavlink_mission_item_t mav_cmd = {};
    AP_Mission::mission_cmd_to_mavlink(cmd,mav_cmd);
    DataFlash.Log_Write_MavCmd(mission.num_commands(),mav_cmd);
}

struct PACKED log_Camera {
    LOG_PACKET_HEADER;
    uint32_t gps_time;
    uint16_t gps_week;
    int32_t  latitude;
    int32_t  longitude;
    int32_t  altitude;
    int16_t  roll;
    int16_t  pitch;
    uint16_t yaw;
};

// Write a Camera packet. Total length : 26 bytes
static void Log_Write_Camera()
{
#if CAMERA == ENABLED
    struct log_Camera pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CAMERA_MSG),
        gps_time    : gps.time_week_ms(),
        gps_week    : gps.time_week(),
        latitude    : current_loc.lat,
        longitude   : current_loc.lng,
        altitude    : current_loc.alt,
        roll        : (int16_t)ahrs.roll_sensor,
        pitch       : (int16_t)ahrs.pitch_sensor,
        yaw         : (uint16_t)ahrs.yaw_sensor
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#endif
}

struct PACKED log_Startup {
    LOG_PACKET_HEADER;
    uint8_t startup_type;
    uint16_t command_total;
};

static void Log_Write_Startup(uint8_t type)
{
    struct log_Startup pkt = {
        LOG_PACKET_HEADER_INIT(LOG_STARTUP_MSG),
        startup_type    : type,
        command_total   : mission.num_commands()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    // write all commands to the dataflash as well
    AP_Mission::Mission_Command cmd;
    for (uint16_t i = 0; i < mission.num_commands(); i++) {
        if (mission.read_cmd_from_storage(i,cmd)) {
            Log_Write_Cmd(cmd);
        }
    }
}

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t nav_roll_cd;
    int16_t roll;
    int16_t nav_pitch_cd;
    int16_t pitch;
    int16_t throttle_out;
    int16_t rudder_out;
    float   accel_y;
};

// Write a control tuning packet. Total length : 22 bytes
static void Log_Write_Control_Tuning()
{
    Vector3f accel = ins.get_accel();
    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CTUN_MSG),
        time_ms         : hal.scheduler->millis(),
        nav_roll_cd     : (int16_t)nav_roll_cd,
        roll            : (int16_t)ahrs.roll_sensor,
        nav_pitch_cd    : (int16_t)nav_pitch_cd,
        pitch           : (int16_t)ahrs.pitch_sensor,
        throttle_out    : (int16_t)channel_throttle->servo_out,
        rudder_out      : (int16_t)channel_rudder->servo_out,
        accel_y         : accel.y
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

// Write a TECS tuning packet
static void Log_Write_TECS_Tuning(void)
{
    SpdHgt_Controller->log_data(DataFlash, LOG_TECS_MSG);
}

struct PACKED log_Nav_Tuning {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint16_t yaw;
    uint32_t wp_distance;
    uint16_t target_bearing_cd;
    uint16_t nav_bearing_cd;
    int16_t altitude_error_cm;
    int16_t airspeed_cm;
    float   altitude;
    uint32_t groundspeed_cm;
};

// Write a navigation tuning packe
static void Log_Write_Nav_Tuning()
{
    struct log_Nav_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_NTUN_MSG),
        time_ms             : hal.scheduler->millis(),
        yaw                 : (uint16_t)ahrs.yaw_sensor,
        wp_distance         : wp_distance,
        target_bearing_cd   : (uint16_t)nav_controller->target_bearing_cd(),
        nav_bearing_cd      : (uint16_t)nav_controller->nav_bearing_cd(),
        altitude_error_cm   : (int16_t)altitude_error_cm,
        airspeed_cm         : (int16_t)airspeed.get_airspeed_cm(),
        altitude            : barometer.get_altitude(),
        groundspeed_cm      : (uint32_t)(gps.ground_speed()*100)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Mode {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t mode;
    uint8_t mode_num;
};

// Write a mode packet. Total length : 5 bytes
static void Log_Write_Mode(uint8_t mode)
{
    struct log_Mode pkt = {
        LOG_PACKET_HEADER_INIT(LOG_MODE_MSG),
        time_ms  : hal.scheduler->millis(),
        mode     : mode,
        mode_num : mode
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Sonar {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float distance;
    float voltage;
    float baro_alt;
    float groundspeed;
    uint8_t throttle;
};

// Write a sonar packet
static void Log_Write_Sonar()
{
    struct log_Sonar pkt = {
        LOG_PACKET_HEADER_INIT(LOG_SONAR_MSG),
        timestamp   : hal.scheduler->millis(),
        distance    : sonar.distance_cm(),
        voltage     : sonar.voltage(),
        baro_alt    : barometer.get_altitude(),
        groundspeed : gps.ground_speed(),
        throttle    : (uint8_t)(100 * channel_throttle->norm_output())
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Current {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t throttle_in;
    int16_t battery_voltage;
    int16_t current_amps;
    uint16_t board_voltage;
    float   current_total;
};

struct PACKED log_Arm_Disarm {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    uint8_t  arm_state;
    uint16_t arm_checks;
};

static void Log_Write_Current()
{
    struct log_Current pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CURRENT_MSG),
        time_ms                 : hal.scheduler->millis(),
        throttle_in             : channel_throttle->control_in,
        battery_voltage         : (int16_t)(battery.voltage() * 100.0),
        current_amps            : (int16_t)(battery.current_amps() * 100.0),
        board_voltage           : (uint16_t)(hal.analogin->board_voltage()*1000),
        current_total           : battery.current_total_mah()
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));

    // also write power status
    DataFlash.Log_Write_Power();
}

static void Log_Arm_Disarm() {
    struct log_Arm_Disarm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ARM_DISARM_MSG),
        time_ms                 : hal.scheduler->millis(),
        arm_state               : arming.is_armed(),
        arm_checks              : arming.get_enabled_checks()      
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_Compass {
    LOG_PACKET_HEADER;
    uint32_t time_ms;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
};

// Write a Compass packet. Total length : 15 bytes
static void Log_Write_Compass()
{
    const Vector3f &mag_offsets = compass.get_offsets();
    const Vector3f &mag = compass.get_field();
    struct log_Compass pkt = {
        LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
        time_ms         : hal.scheduler->millis(),
        mag_x           : (int16_t)mag.x,
        mag_y           : (int16_t)mag.y,
        mag_z           : (int16_t)mag.z,
        offset_x        : (int16_t)mag_offsets.x,
        offset_y        : (int16_t)mag_offsets.y,
        offset_z        : (int16_t)mag_offsets.z
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
#if COMPASS_MAX_INSTANCES > 1
    if (compass.get_count() > 1) {
        const Vector3f &mag2_offsets = compass.get_offsets(1);
        const Vector3f &mag2 = compass.get_field(1);
        struct log_Compass pkt2 = {
            LOG_PACKET_HEADER_INIT(LOG_COMPASS2_MSG),
            time_ms         : hal.scheduler->millis(),
            mag_x           : (int16_t)mag2.x,
            mag_y           : (int16_t)mag2.y,
            mag_z           : (int16_t)mag2.z,
            offset_x        : (int16_t)mag2_offsets.x,
            offset_y        : (int16_t)mag2_offsets.y,
            offset_z        : (int16_t)mag2_offsets.z
        };
        DataFlash.WriteBlock(&pkt2, sizeof(pkt2));
    }
#endif
}

static void Log_Write_GPS(uint8_t instance)
{
    DataFlash.Log_Write_GPS(gps, instance, current_loc.alt - ahrs.get_home().alt);
}

static void Log_Write_IMU() 
{
    DataFlash.Log_Write_IMU(ins);
}

static void Log_Write_RC(void)
{
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
}

static void Log_Write_Baro(void)
{
    DataFlash.Log_Write_Baro(barometer);
}

struct PACKED log_AIRSPEED {
    LOG_PACKET_HEADER;
    uint32_t timestamp;
    float   airspeed;
    float   diffpressure;
    int16_t temperature;
};

// Write a AIRSPEED packet
static void Log_Write_Airspeed(void)
{
    float temperature;
    if (!airspeed.get_temperature(temperature)) {
        temperature = 0;
    }
    struct log_AIRSPEED pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AIRSPEED_MSG),
        timestamp     : hal.scheduler->millis(),
        airspeed      : airspeed.get_raw_airspeed(),
        diffpressure  : airspeed.get_differential_pressure(),
        temperature   : (int16_t)(temperature * 100.0f)
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}


// AA241X - start

struct PACKED log_AA241X_MF {
    LOG_PACKET_HEADER;
    // Medium Frequency Variables "II hhh hh BB HHH B hH" 
    uint32_t cpu_time_ms;  // I
    uint32_t gps_time_ms;  // I
    
    int16_t x_position;  // h
    int16_t y_position;  // h
    int16_t z_position;  // h
    
    int16_t Ground_speed;  //h
    int16_t Ground_course;  //h
    
    uint8_t GPS_Fix;      // B
    uint8_t GPS_NumSats;  // B
    
    uint16_t Battery_Current;  // H
    uint16_t Battery_Voltage;   // H
    uint16_t Battery_Energy_Consumed;    // H
    
    uint8_t Control_Mode;  // B
    
    int16_t Z_Velocity;  // h
    uint16_t airspeed;   // H

};


static void Log_Write_AA241X_MF(void)
{
  const Location &loc = gps.location(0);
    struct log_AA241X_MF pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AA241X_MF_MSG),
        cpu_time_ms   :  CPU_time_ms,  
        gps_time_ms   :  GPS_time_ms,  
        
        x_position    :  X_position*100,
        y_position    :  Y_position*100,
        z_position    :  Z_position_GPS*100,
        
        Ground_speed  :  ground_speed*100,
        Ground_course :  ground_course*100,
        
        GPS_Fix       :  gps.status(0),      
        GPS_NumSats   :  gps.num_sats(0),  
        
        Battery_Current  :   battery_current*100, 
        Battery_Voltage  :   battery_voltage*100,   
        Battery_Energy_Consumed  :  battery_energy_consumed,    
        
        Control_Mode   :  control_mode,   
        
        Z_Velocity        :  Z_velocity*100,
        airspeed          :  Air_speed*100,

    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AA241X_HF {
    LOG_PACKET_HEADER;   
    // High Frequency Variables  "H h hhh fff BBBB"
    uint16_t cpu_time_ms;  // H
    
    int16_t Z_Position_Baro; // h
    
    int16_t Roll;  // h
    int16_t Pitch; // h
    int16_t Yaw;   // h
    
    float Roll_rate;     // f
    float Pitch_rate;    // f
    float Yaw_rate;      // f
    
    uint8_t Roll_Servo_Out_PWM;      // B
    uint8_t Pitch_Servo_Out_PWM;     // B
    uint8_t Throttle_Servo_Out_PWM;  // B
    uint8_t Rudder_Servo_Out_PWM;    // B
};

static void Log_Write_AA241X_HF(void)
{
    struct log_AA241X_HF pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AA241X_HF_MSG),
        // High Frequency Variables 
        cpu_time_ms   :  CPU_time_ms,  
        
        Z_Position_Baro   :  Z_position_Baro*100.0, 
        
        Roll   :  ToDeg(roll)*100,
        Pitch  :  ToDeg(pitch)*100,
        Yaw    :  ToDeg(yaw)*100,
        
        Roll_rate   : ToDeg(roll_rate),
        Pitch_rate  : ToDeg(pitch_rate),
        Yaw_rate    : ToDeg(yaw_rate),
        
        Roll_Servo_Out_PWM       :   200.0*((channel_roll->radio_out  - PWM_MIN))/((float)(PWM_MAX - PWM_MIN)),      
        Pitch_Servo_Out_PWM      :   200.0*((channel_pitch->radio_out  - PWM_MIN))/((float)(PWM_MAX - PWM_MIN)),
        Throttle_Servo_Out_PWM   :   200.0*((channel_throttle->radio_out - PWM_MIN))/((float)(PWM_MAX - PWM_MIN)),
        Rudder_Servo_Out_PWM     :   200.0*((channel_rudder->radio_out - PWM_MIN))/((float)(PWM_MAX - PWM_MIN)), 
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}

struct PACKED log_AA241X_SCR {
    LOG_PACKET_HEADER;   
    // Score Variables  "I BHI ffffffff fB"
    uint32_t cpu_time_ms;  // I
    
    uint8_t In_mission;  // B
    uint16_t Mission_energy_consumed;  // H
    uint32_t cpu_time_sight_ms;  // I
    
    float X_Person0_estimate;  // f
    float Y_Person0_estimate;  // f
    float X_Person1_estimate;  // f
    float Y_Person1_estimate;  // f
    float X_Person2_estimate;  // f
    float Y_Person2_estimate;  // f
    float X_Person3_estimate;  // f
    float Y_Person3_estimate;  // f
    
    float score;    // f
    uint8_t PersonDistributionIndex;  // B
       
};

static void Log_Write_AA241X_SCR(void)
{
    struct log_AA241X_SCR pkt = {
        LOG_PACKET_HEADER_INIT(LOG_AA241X_SCR_MSG),
        // High Frequency Variables 
        cpu_time_ms   :  CPU_time_ms,  
        
        In_mission    : in_mission,
        Mission_energy_consumed    :  mission_energy_consumed,
        cpu_time_sight_ms     :    CPU_time_sight_ms,
        
        X_Person0_estimate  :  X_person_estimate[0],
        Y_Person0_estimate  :  Y_person_estimate[0],
        X_Person1_estimate  :  X_person_estimate[1],
        Y_Person1_estimate  :  Y_person_estimate[1],
        X_Person2_estimate  :  X_person_estimate[2],
        Y_Person2_estimate  :  Y_person_estimate[2],
        X_Person3_estimate  :  X_person_estimate[3],
        Y_Person3_estimate  :  Y_person_estimate[3],
        
        score     :  Score,
        PersonDistributionIndex   :  personDistributionIndex,
    };
    DataFlash.WriteBlock(&pkt, sizeof(pkt));
}



// AA241X - end

static const struct LogStructure log_structure[] PROGMEM = {
    LOG_COMMON_STRUCTURES,
    { LOG_ATTITUDE_MSG, sizeof(log_Attitude),       
      "ATT", "IccCCC",        "TimeMS,Roll,Pitch,Yaw,ErrorRP,ErrorYaw" },
    { LOG_PERFORMANCE_MSG, sizeof(log_Performance), 
      "PM",  "IHIhhhBH", "LTime,MLC,gDt,GDx,GDy,GDz,I2CErr,INSErr" },
    { LOG_CAMERA_MSG, sizeof(log_Camera),                 
      "CAM", "IHLLeccC",   "GPSTime,GPSWeek,Lat,Lng,Alt,Roll,Pitch,Yaw" },
    { LOG_STARTUP_MSG, sizeof(log_Startup),         
      "STRT", "BH",         "SType,CTot" },
    { LOG_CTUN_MSG, sizeof(log_Control_Tuning),     
      "CTUN", "Icccchhf",    "TimeMS,NavRoll,Roll,NavPitch,Pitch,ThrOut,RdrOut,AccY" },
    { LOG_NTUN_MSG, sizeof(log_Nav_Tuning),         
      "NTUN", "ICICCccfI",   "TimeMS,Yaw,WpDist,TargBrg,NavBrg,AltErr,Arspd,Alt,GSpdCM" },
    { LOG_SONAR_MSG, sizeof(log_Sonar),             
      "SONR", "IffffB",   "TimeMS,DistCM,Volt,BaroAlt,GSpd,Thr" },
    { LOG_MODE_MSG, sizeof(log_Mode),             
      "MODE", "IMB",         "TimeMS,Mode,ModeNum" },
    { LOG_CURRENT_MSG, sizeof(log_Current),             
      "CURR", "IhhhHf",      "TimeMS,Thr,Volt,Curr,Vcc,CurrTot" },
    { LOG_COMPASS_MSG, sizeof(log_Compass),             
      "MAG", "Ihhhhhh",   "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ" },
    { LOG_COMPASS2_MSG, sizeof(log_Compass),             
      "MAG2", "Ihhhhhh",   "TimeMS,MagX,MagY,MagZ,OfsX,OfsY,OfsZ" },
    { LOG_ARM_DISARM_MSG, sizeof(log_Arm_Disarm),
      "ARM", "IHB", "TimeMS,ArmState,ArmChecks" },
    { LOG_AIRSPEED_MSG, sizeof(log_AIRSPEED),
      "ARSP",  "Iffc",     "TimeMS,Airspeed,DiffPress,Temp" },
    { LOG_AA241X_MF_MSG, sizeof(log_AA241X_MF),
      "MF",  "IIhhhhhBBHHHBhH",     "CP_t,GP_t,X_p,Y_p,Z_pG,Gd_S,Gd_C,Fix,NS,B_c,B_v,B_e,c_m,Z_v,AS" },
    { LOG_AA241X_HF_MSG, sizeof(log_AA241X_HF),
      "HF",  "HhhhhfffBBBB",     "CP_t,Z_pB,fi,theta,psi,p,q,r,CH1,CH2,CH3,CH4" },
    { LOG_AA241X_SCR_MSG, sizeof(log_AA241X_SCR),
      "SCR",  "IBHIfffffffffB",     "CP_t,In_Mis,Mis_En,T_sit,X0,Y0,X1,Y1,X2,Y2,X4,Y3,Scre,D_Ind " },
    TECS_LOG_FORMAT(LOG_TECS_MSG)
};

// Read the DataFlash.log memory : Packet Parser
static void Log_Read(uint16_t log_num, int16_t start_page, int16_t end_page)
{
    cliSerial->printf_P(PSTR("\n" FIRMWARE_STRING
                             "\nFree RAM: %u\n"),
                        (unsigned)hal.util->available_memory());

    cliSerial->println_P(PSTR(HAL_BOARD_NAME));

	DataFlash.LogReadProcess(log_num, start_page, end_page, 
                             print_flight_mode,
                             cliSerial);
}

// start a new log
static void start_logging() 
{
    DataFlash.StartNewLog();
    DataFlash.Log_Write_Message_P(PSTR(FIRMWARE_STRING));
#if defined(PX4_GIT_VERSION) && defined(NUTTX_GIT_VERSION)
    DataFlash.Log_Write_Message_P(PSTR("PX4: " PX4_GIT_VERSION " NuttX: " NUTTX_GIT_VERSION));
#endif

    // write system identifier as well if available
    char sysid[40];
    if (hal.util->get_system_id(sysid)) {
        DataFlash.Log_Write_Message(sysid);
    }
}

#else // LOGGING_ENABLED

// dummy functions
static void Log_Write_Startup(uint8_t type) {}
static void Log_Write_Current() {}
static void Log_Write_Nav_Tuning() {}
static void Log_Write_TECS_Tuning() {}
static void Log_Write_Performance() {}
static void Log_Write_Cmd(const AP_Mission::Mission_Command &cmd) {}
static void Log_Write_Attitude() {}
static void Log_Write_Control_Tuning() {}
static void Log_Write_Camera() {}
static void Log_Write_Mode(uint8_t mode) {}
static void Log_Write_Compass() {}
static void Log_Write_GPS(uint8_t instance) {}
static void Log_Write_IMU() {}
static void Log_Write_RC() {}
static void Log_Write_Airspeed(void) {}
static void Log_Write_Baro(void) {}

static int8_t process_logs(uint8_t argc, const Menu::arg *argv) {
    return 0;
}


#endif // LOGGING_ENABLED
