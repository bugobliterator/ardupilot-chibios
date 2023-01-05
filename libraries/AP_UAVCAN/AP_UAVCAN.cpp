/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Eugene Shamaev, Siddharth Bharat Purohit
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS
#include "AP_UAVCAN.h"
#include <GCS_MAVLink/GCS.h>

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>

#include <AP_Arming/AP_Arming.h>
#include <AP_GPS/AP_GPS_UAVCAN.h>
#include <AP_Compass/AP_Compass_UAVCAN.h>
#include <AP_Baro/AP_Baro_UAVCAN.h>
#include <AP_BattMonitor/AP_BattMonitor_UAVCAN.h>
#include <AP_Airspeed/AP_Airspeed_UAVCAN.h>
#include <AP_OpticalFlow/AP_OpticalFlow_HereFlow.h>
#include <AP_RangeFinder/AP_RangeFinder_UAVCAN.h>
#include <AP_EFI/AP_EFI_DroneCAN.h>
#include <AP_Proximity/AP_Proximity_DroneCAN.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ADSB/AP_ADSB.h>
#include "AP_UAVCAN_DNA_Server.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_OpenDroneID/AP_OpenDroneID.h>
#include <string.h>

#define LED_DELAY_US 50000

extern const AP_HAL::HAL& hal;

// setup default pool size
#ifndef UAVCAN_NODE_POOL_SIZE
#if HAL_CANFD_SUPPORTED
#define UAVCAN_NODE_POOL_SIZE 16384
#else
#define UAVCAN_NODE_POOL_SIZE 8192
#endif
#endif

#if HAL_CANFD_SUPPORTED
#define UAVCAN_STACK_SIZE     8192
#else
#define UAVCAN_STACK_SIZE     4096
#endif

#define debug_uavcan(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "UAVCAN", fmt, ##args); } while (0)

// Translation of all messages from UAVCAN structures into AP structures is done
// in AP_UAVCAN and not in corresponding drivers.
// The overhead of including definitions of DSDL is very high and it is best to
// concentrate in one place.

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_UAVCAN::var_info[] = {
    // @Param: NODE
    // @DisplayName: UAVCAN node that is used for this network
    // @Description: UAVCAN node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE", 1, AP_UAVCAN, _dronecan_node, 10),

    // @Param: SRV_BM
    // @DisplayName: Output channels to be transmitted as servo over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a servo command over UAVCAN
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15, 15: Servo 16, 16: Servo 17, 17: Servo 18, 18: Servo 19, 19: Servo 20, 20: Servo 21, 21: Servo 22, 22: Servo 23, 23: Servo 24, 24: Servo 25, 25: Servo 26, 26: Servo 27, 27: Servo 28, 28: Servo 29, 29: Servo 30, 30: Servo 31, 31: Servo 32

    // @User: Advanced
    AP_GROUPINFO("SRV_BM", 2, AP_UAVCAN, _servo_bm, 0),

    // @Param: ESC_BM
    // @DisplayName: Output channels to be transmitted as ESC over UAVCAN
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over UAVCAN
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32
    // @User: Advanced
    AP_GROUPINFO("ESC_BM", 3, AP_UAVCAN, _esc_bm, 0),

    // @Param: SRV_RT
    // @DisplayName: Servo output rate
    // @Description: Maximum transmit rate for servo outputs
    // @Range: 1 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("SRV_RT", 4, AP_UAVCAN, _servo_rate_hz, 50),

    // @Param: OPTION
    // @DisplayName: UAVCAN options
    // @Description: Option flags
    // @Bitmask: 0:ClearDNADatabase,1:IgnoreDNANodeConflicts,2:EnableCanfd,3:IgnoreDNANodeUnhealthy,4:SendServoAsPWM
    // @User: Advanced
    AP_GROUPINFO("OPTION", 5, AP_UAVCAN, _options, 0),
    
    // @Param: NTF_RT
    // @DisplayName: Notify State rate
    // @Description: Maximum transmit rate for Notify State Message
    // @Range: 1 200
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("NTF_RT", 6, AP_UAVCAN, _notify_state_hz, 20),

    // @Param: ESC_OF
    // @DisplayName: ESC Output channels offset
    // @Description: Offset for ESC numbering in DroneCAN ESC RawCommand messages. This allows for more efficient packing of ESC command messages. If your ESCs are on servo functions 5 to 8 and you set this parameter to 4 then the ESC RawCommand will be sent with the first 4 slots filled. This can be used for more efficint usage of CAN bandwidth
    // @Range: 0 18
    // @User: Advanced
    AP_GROUPINFO("ESC_OF", 7, AP_UAVCAN, _esc_offset, 0),

    // @Param: POOL
    // @DisplayName: CAN pool size
    // @Description: Amount of memory in bytes to allocate for the DroneCAN memory pool. More memory is needed for higher CAN bus loads
    // @Range: 1024 16384
    // @User: Advanced
    AP_GROUPINFO("POOL", 8, AP_UAVCAN, _pool_size, UAVCAN_NODE_POOL_SIZE),
    
    AP_GROUPEND
};

// this is the timeout in milliseconds for periodic message types. We
// set this to 1 to minimise resend of stale msgs
#define CAN_PERIODIC_TX_TIMEOUT_MS 2

AP_UAVCAN::AP_UAVCAN(const int driver_index) :
_driver_index(driver_index),
canard_iface(driver_index),
_dna_server(*this)
{
    AP_Param::setup_object_defaults(this, var_info);

    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        _SRV_conf[i].esc_pending = false;
        _SRV_conf[i].servo_pending = false;
    }

    debug_uavcan(AP_CANManager::LOG_INFO, "AP_UAVCAN constructed\n\r");
}

AP_UAVCAN::~AP_UAVCAN()
{
}

AP_UAVCAN *AP_UAVCAN::get_uavcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_UAVCAN) {
        return nullptr;
    }
    return static_cast<AP_UAVCAN*>(AP::can().get_driver(driver_index));
}

bool AP_UAVCAN::add_interface(AP_HAL::CANIface* can_iface)
{
    if (!canard_iface.add_interface(can_iface)) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: can't add UAVCAN interface\n\r");
        return false;   
    }
    return true;
}

void AP_UAVCAN::init(uint8_t driver_index, bool enable_filters)
{
    if (driver_index != _driver_index) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: init called with wrong driver_index");
        return;
    }
    if (_initialized) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: init called more than once\n\r");
        return;
    }

    node_info_rsp.name.len = snprintf((char*)node_info_rsp.name.data, sizeof(node_info_rsp.name.data), "org.ardupilot:%u", driver_index);

    node_info_rsp.software_version.major = AP_UAVCAN_SW_VERS_MAJOR;
    node_info_rsp.software_version.minor = AP_UAVCAN_SW_VERS_MINOR;
    node_info_rsp.hardware_version.major = AP_UAVCAN_HW_VERS_MAJOR;
    node_info_rsp.hardware_version.minor = AP_UAVCAN_HW_VERS_MINOR;

#if HAL_CANFD_SUPPORTED
    if (option_is_set(Options::CANFD_ENABLED)) {
        // _node->enableCanFd();
    }
#endif

    uint8_t uid_len = sizeof(uavcan_protocol_HardwareVersion::unique_id);
    uint8_t unique_id[sizeof(uavcan_protocol_HardwareVersion::unique_id)];

    mem_pool = new uint32_t[_pool_size/sizeof(uint32_t)];
    if (mem_pool == nullptr) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: Failed to allocate memory pool\n\r");
        return;
    }
    canard_iface.init(mem_pool, (_pool_size/sizeof(uint32_t))*sizeof(uint32_t), _dronecan_node);

    if (!hal.util->get_system_id_unformatted(unique_id, uid_len)) {
        return;
    }
    unique_id[uid_len - 1] += _dronecan_node;
    memcpy(node_info_rsp.hardware_version.unique_id, unique_id, uid_len);

    //Start Servers
    if (!_dna_server.init(unique_id, uid_len, _dronecan_node)) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: Failed to start DNA Server\n\r");
        return;
    }

    // Roundup all subscribers from supported drivers
    AP_GPS_UAVCAN::subscribe_msgs(this);
    AP_Compass_UAVCAN::subscribe_msgs(this);

#if AP_BARO_UAVCAN_ENABLED
    AP_Baro_UAVCAN::subscribe_msgs(this);
#endif
    AP_BattMonitor_UAVCAN::subscribe_msgs(this);
#if AP_AIRSPEED_UAVCAN_ENABLED
    AP_Airspeed_UAVCAN::subscribe_msgs(this);
#endif
#if AP_OPTICALFLOW_HEREFLOW_ENABLED
    AP_OpticalFlow_HereFlow::subscribe_msgs(this);
#endif
#if AP_RANGEFINDER_UAVCAN_ENABLED
    AP_RangeFinder_UAVCAN::subscribe_msgs(this);
#endif
#if HAL_EFI_DRONECAN_ENABLED
    AP_EFI_DroneCAN::subscribe_msgs(this);
#endif

#if AP_PROXIMITY_DRONECAN_ENABLED
    AP_Proximity_DroneCAN::subscribe_msgs(this);
#endif

    act_out_array.set_timeout_ms(2);
    act_out_array.set_priority(CANARD_TRANSFER_PRIORITY_HIGH);

    esc_raw.set_timeout_ms(2);
    esc_raw.set_priority(CANARD_TRANSFER_PRIORITY_HIGH);

    rgb_led.set_timeout_ms(20);
    rgb_led.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    buzzer.set_timeout_ms(20);
    buzzer.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    safety_state.set_timeout_ms(20);
    safety_state.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    arming_status.set_timeout_ms(20);
    arming_status.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    rtcm_stream.set_timeout_ms(20);
    rtcm_stream.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    notify_state.set_timeout_ms(20);
    notify_state.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    param_save_client.set_timeout_ms(20);
    param_save_client.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    param_get_set_client.set_timeout_ms(20);
    param_get_set_client.set_priority(CANARD_TRANSFER_PRIORITY_LOW);

    node_status.set_priority(CANARD_TRANSFER_PRIORITY_LOWEST);
    node_status.set_timeout_ms(1000);

    node_info_server.set_timeout_ms(20);

    _led_conf.devices_count = 0;

    // setup node status
    node_status_msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status_msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status_msg.sub_mode = 0;

    // Spin node for device discovery
    for (uint8_t i = 0; i < 5; i++) {
        send_node_status();
        canard_iface.process(1000);
    }

    snprintf(_thread_name, sizeof(_thread_name), "uavcan_%u", driver_index);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_UAVCAN::loop, void), _thread_name, UAVCAN_STACK_SIZE, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_uavcan(AP_CANManager::LOG_ERROR, "UAVCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;
    debug_uavcan(AP_CANManager::LOG_INFO, "UAVCAN: init done\n\r");
}

void AP_UAVCAN::loop(void)
{
    while (true) {
        if (!_initialized) {
            hal.scheduler->delay_microseconds(1000);
            continue;
        }

        canard_iface.process(1);

        if (_SRV_armed) {
            bool sent_servos = false;

            if (_servo_bm > 0) {
                // if we have any Servos in bitmask
                uint32_t now = AP_HAL::native_micros();
                const uint32_t servo_period_us = 1000000UL / unsigned(_servo_rate_hz.get());
                if (now - _SRV_last_send_us >= servo_period_us) {
                    _SRV_last_send_us = now;
                    SRV_send_actuator();
                    sent_servos = true;
                    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
                        _SRV_conf[i].servo_pending = false;
                    }
                }
            }

            // if we have any ESC's in bitmask
            if (_esc_bm > 0 && !sent_servos) {
                SRV_send_esc();
            }

            for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
                _SRV_conf[i].esc_pending = false;
            }
        }

        led_out_send();
        buzzer_send();
        rtcm_stream_send();
        safety_state_send();
        notify_state_send();
        send_parameter_request();
        send_parameter_save_request();
        send_node_status();
        _dna_server.verify_nodes();
#if AP_OPENDRONEID_ENABLED
        AP::opendroneid().dronecan_send(this);
#endif
        logging();
    }
}


void AP_UAVCAN::send_node_status(void)
{
    const uint32_t now = AP_HAL::native_millis();
    if (now - _node_status_last_send_ms < 1000) {
        return;
    }
    _node_status_last_send_ms = now;
    node_status_msg.uptime_sec = now / 1000;
    node_status.broadcast(node_status_msg);
}

void AP_UAVCAN::handle_node_info_request(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req)
{
    node_info_rsp.status = node_status_msg;
    node_info_rsp.status.uptime_sec = AP_HAL::native_millis() / 1000;

    node_info_server.respond(transfer, node_info_rsp);
}


///// SRV output /////

void AP_UAVCAN::SRV_send_actuator(void)
{
    uint8_t starting_servo = 0;
    bool repeat_send;

    WITH_SEMAPHORE(SRV_sem);

    do {
        repeat_send = false;
        uavcan_equipment_actuator_ArrayCommand msg;

        uint8_t i;
        // UAVCAN can hold maximum of 15 commands in one frame
        for (i = 0; starting_servo < UAVCAN_SRV_NUMBER && i < 15; starting_servo++) {
            uavcan_equipment_actuator_Command cmd;

            /*
             * Servo output uses a range of 1000-2000 PWM for scaling.
             * This converts output PWM from [1000:2000] range to [-1:1] range that
             * is passed to servo as unitless type via UAVCAN.
             * This approach allows for MIN/TRIM/MAX values to be used fully on
             * autopilot side and for servo it should have the setup to provide maximum
             * physically possible throws at [-1:1] limits.
             */

            if (_SRV_conf[starting_servo].servo_pending && ((((uint32_t) 1) << starting_servo) & _servo_bm)) {
                cmd.actuator_id = starting_servo + 1;

                if (option_is_set(Options::USE_ACTUATOR_PWM)) {
                    cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM;
                    cmd.command_value = _SRV_conf[starting_servo].pulse;
                } else {
                    cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS;
                    cmd.command_value = constrain_float(((float) _SRV_conf[starting_servo].pulse - 1000.0) / 500.0 - 1.0, -1.0, 1.0);
                }

                msg.commands.data[i] = cmd;

                i++;
            }
        }
        msg.commands.len = i;
        if (i > 0) {
            if (act_out_array.broadcast(msg) > 0) {
                _srv_send_count++;
            } else {
                _fail_send_count++;
            }

            if (i == 15) {
                repeat_send = true;
            }
        }
    } while (repeat_send);
}

void AP_UAVCAN::SRV_send_esc(void)
{
    static const int cmd_max = ((1<<13)-1);
    uavcan_equipment_esc_RawCommand esc_msg;

    uint8_t active_esc_num = 0, max_esc_num = 0;
    uint8_t k = 0;

    WITH_SEMAPHORE(SRV_sem);

    // esc offset allows for efficient packing of higher ESC numbers in RawCommand
    const uint8_t esc_offset = constrain_int16(_esc_offset.get(), 0, UAVCAN_SRV_NUMBER);

    // find out how many esc we have enabled and if they are active at all
    for (uint8_t i = esc_offset; i < UAVCAN_SRV_NUMBER; i++) {
        if ((((uint32_t) 1) << i) & _esc_bm) {
            max_esc_num = i + 1;
            if (_SRV_conf[i].esc_pending) {
                active_esc_num++;
            }
        }
    }

    // if at least one is active (update) we need to send to all
    if (active_esc_num > 0) {
        k = 0;

        for (uint8_t i = esc_offset; i < max_esc_num && k < 20; i++) {
            if ((((uint32_t) 1) << i) & _esc_bm) {
                // TODO: ESC negative scaling for reverse thrust and reverse rotation
                float scaled = cmd_max * (hal.rcout->scale_esc_to_unity(_SRV_conf[i].pulse) + 1.0) / 2.0;

                scaled = constrain_float(scaled, 0, cmd_max);

                esc_msg.cmd.data[k] = static_cast<int>(scaled);
            } else {
                esc_msg.cmd.data[k] = static_cast<unsigned>(0);
            }

            k++;
        }
        esc_msg.cmd.len = k;

        if (esc_raw.broadcast(esc_msg)) {
            _esc_send_count++;
        } else {
            _fail_send_count++;
        }
    }
}

void AP_UAVCAN::SRV_push_servos()
{
    WITH_SEMAPHORE(SRV_sem);

    for (uint8_t i = 0; i < UAVCAN_SRV_NUMBER; i++) {
        // Check if this channels has any function assigned
        if (SRV_Channels::channel_function(i) >= SRV_Channel::k_none) {
            _SRV_conf[i].pulse = SRV_Channels::srv_channel(i)->get_output_pwm();
            _SRV_conf[i].esc_pending = true;
            _SRV_conf[i].servo_pending = true;
        }
    }

    _SRV_armed = hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;
}


///// LED /////

void AP_UAVCAN::led_out_send()
{
    uint64_t now = AP_HAL::native_micros64();

    if ((now - _led_conf.last_update) < LED_DELAY_US) {
        return;
    }

    uavcan_equipment_indication_LightsCommand msg;
    {
        WITH_SEMAPHORE(_led_out_sem);

        if (_led_conf.devices_count == 0) {
            return;
        }

        msg.commands.len = _led_conf.devices_count;
        for (uint8_t i = 0; i < _led_conf.devices_count; i++) {
            msg.commands.data[i].light_id =_led_conf.devices[i].led_index;
            msg.commands.data[i].color.red = _led_conf.devices[i].red >> 3;
            msg.commands.data[i].color.green = _led_conf.devices[i].green >> 2;
            msg.commands.data[i].color.blue = _led_conf.devices[i].blue >> 3;
        }
    }

    rgb_led.broadcast(msg);
    _led_conf.last_update = now;
}

bool AP_UAVCAN::led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (_led_conf.devices_count >= AP_UAVCAN_MAX_LED_DEVICES) {
        return false;
    }

    WITH_SEMAPHORE(_led_out_sem);

    // check if a device instance exists. if so, break so the instance index is remembered
    uint8_t instance = 0;
    for (; instance < _led_conf.devices_count; instance++) {
        if (_led_conf.devices[instance].led_index == led_index) {
            break;
        }
    }

    // load into the correct instance.
    // if an existing instance was found in above for loop search,
    // then instance value is < _led_conf.devices_count.
    // otherwise a new one was just found so we increment the count.
    // Either way, the correct instance is the current value of instance
    _led_conf.devices[instance].led_index = led_index;
    _led_conf.devices[instance].red = red;
    _led_conf.devices[instance].green = green;
    _led_conf.devices[instance].blue = blue;

    if (instance == _led_conf.devices_count) {
        _led_conf.devices_count++;
    }

    return true;
}

// buzzer send
void AP_UAVCAN::buzzer_send()
{
    uavcan_equipment_indication_BeepCommand msg;
    WITH_SEMAPHORE(_buzzer.sem);
    uint8_t mask = (1U << _driver_index);
    if ((_buzzer.pending_mask & mask) == 0) {
        return;
    }
    _buzzer.pending_mask &= ~mask;
    msg.frequency = _buzzer.frequency;
    msg.duration = _buzzer.duration;
    buzzer.broadcast(msg);
}

// buzzer support
void AP_UAVCAN::set_buzzer_tone(float frequency, float duration_s)
{
    WITH_SEMAPHORE(_buzzer.sem);
    _buzzer.frequency = frequency;
    _buzzer.duration = duration_s;
    _buzzer.pending_mask = 0xFF;
}

// notify state send
void AP_UAVCAN::notify_state_send()
{
    uint32_t now = AP_HAL::native_millis();

    if (_notify_state_hz == 0 || (now - _last_notify_state_ms) < uint32_t(1000 / _notify_state_hz)) {
        return;
    }

    ardupilot_indication_NotifyState msg;
    msg.vehicle_state = 0;
    if (AP_Notify::flags.initialising) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_INITIALISING;
    }
    if (AP_Notify::flags.armed) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_ARMED;
    }
    if (AP_Notify::flags.flying) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FLYING;
    }
    if (AP_Notify::flags.compass_cal_running) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_MAGCAL_RUN;
    }
    if (AP_Notify::flags.ekf_bad) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_EKF_BAD;
    }
    if (AP_Notify::flags.esc_calibration) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_ESC_CALIBRATION;
    }
    if (AP_Notify::flags.failsafe_battery) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FAILSAFE_BATT;
    }
    if (AP_Notify::flags.failsafe_gcs) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FAILSAFE_GCS;
    }
    if (AP_Notify::flags.failsafe_radio) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FAILSAFE_RADIO;
    }
    if (AP_Notify::flags.firmware_update) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_FW_UPDATE;
    }
    if (AP_Notify::flags.gps_fusion) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_GPS_FUSION;
    }
    if (AP_Notify::flags.gps_glitching) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_GPS_GLITCH;
    }
    if (AP_Notify::flags.have_pos_abs) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_POS_ABS_AVAIL;
    }
    if (AP_Notify::flags.leak_detected) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_LEAK_DET;
    }
    if (AP_Notify::flags.parachute_release) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_CHUTE_RELEASED;
    }
    if (AP_Notify::flags.powering_off) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_POWERING_OFF;
    }
    if (AP_Notify::flags.pre_arm_check) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_PREARM;
    }
    if (AP_Notify::flags.pre_arm_gps_check) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_PREARM_GPS;
    }
    if (AP_Notify::flags.save_trim) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_SAVE_TRIM;
    }
    if (AP_Notify::flags.vehicle_lost) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_LOST;
    }
    if (AP_Notify::flags.video_recording) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_VIDEO_RECORDING;
    }
    if (AP_Notify::flags.waiting_for_throw) {
        msg.vehicle_state |= 1 << ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_STATE_THROW_READY;
    }

    msg.aux_data_type = ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_YAW_EARTH_CENTIDEGREES;
    uint16_t yaw_cd = (uint16_t)(360.0f - degrees(AP::ahrs().get_yaw()))*100.0f;
    const uint8_t *data = (uint8_t *)&yaw_cd;
    for (uint8_t i=0; i<2; i++) {
        msg.aux_data.data[i] = data[i];
    }
    msg.aux_data.len = 2;
    notify_state.broadcast(msg);
    _last_notify_state_ms = AP_HAL::native_millis();
}

void AP_UAVCAN::rtcm_stream_send()
{
    WITH_SEMAPHORE(_rtcm_stream.sem);
    if (_rtcm_stream.buf == nullptr ||
        _rtcm_stream.buf->available() == 0) {
        // nothing to send
        return;
    }
    uint32_t now = AP_HAL::native_millis();
    if (now - _rtcm_stream.last_send_ms < 20) {
        // don't send more than 50 per second
        return;
    }
    _rtcm_stream.last_send_ms = now;
    uavcan_equipment_gnss_RTCMStream msg;
    uint32_t len = _rtcm_stream.buf->available();
    if (len > 128) {
        len = 128;
    }
    msg.protocol_id = UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_PROTOCOL_ID_RTCM3;
    for (uint8_t i=0; i<len; i++) {
        uint8_t b;
        if (!_rtcm_stream.buf->read_byte(&b)) {
            return;
        }
        msg.data.data[i] = b;
    }
    msg.data.len = len;
    rtcm_stream.broadcast(msg);
}

// SafetyState send
void AP_UAVCAN::safety_state_send()
{
    uint32_t now = AP_HAL::native_millis();
    if (now - _last_safety_state_ms < 500) {
        // update at 2Hz
        return;
    }
    _last_safety_state_ms = now;

    { // handle SafetyState
        ardupilot_indication_SafetyState safety_msg;
        switch (hal.util->safety_switch_state()) {
        case AP_HAL::Util::SAFETY_ARMED:
            safety_msg.status = ARDUPILOT_INDICATION_SAFETYSTATE_STATUS_SAFETY_OFF;
            break;
        case AP_HAL::Util::SAFETY_DISARMED:
            safety_msg.status = ARDUPILOT_INDICATION_SAFETYSTATE_STATUS_SAFETY_ON;
            break;
        default:
            // nothing to send
            break;
        }
        safety_state.broadcast(safety_msg);
    }

    { // handle ArmingStatus
        uavcan_equipment_safety_ArmingStatus arming_msg;
        arming_msg.status = hal.util->get_soft_armed() ? UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED :
                                                      UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_DISARMED;
        arming_status.broadcast(arming_msg);
    }
}

/*
 send RTCMStream packet on all active UAVCAN drivers
*/
void AP_UAVCAN::send_RTCMStream(const uint8_t *data, uint32_t len)
{
    WITH_SEMAPHORE(_rtcm_stream.sem);
    if (_rtcm_stream.buf == nullptr) {
        // give enough space for a full round from a NTRIP server with all
        // constellations
        _rtcm_stream.buf = new ByteBuffer(2400);
    }
    if (_rtcm_stream.buf == nullptr) {
        return;
    }
    _rtcm_stream.buf->write(data, len);
}

/*
  handle Button message
 */
void AP_UAVCAN::handle_button(const CanardRxTransfer& transfer, const ardupilot_indication_Button& msg)
{
    switch (msg.button) {
    case ARDUPILOT_INDICATION_BUTTON_BUTTON_SAFETY: {
        AP_BoardConfig *brdconfig = AP_BoardConfig::get_singleton();
        if (brdconfig && brdconfig->safety_button_handle_pressed(msg.press_time)) {
            AP_HAL::Util::safety_state state = hal.util->safety_switch_state();
            if (state == AP_HAL::Util::SAFETY_ARMED) {
                hal.rcout->force_safety_on();
            } else {
                hal.rcout->force_safety_off();
            }
        }
        break;
    }
    }
}

/*
  handle traffic report
 */
void AP_UAVCAN::handle_traffic_report(const CanardRxTransfer& transfer, const ardupilot_equipment_trafficmonitor_TrafficReport& msg)
{
#if HAL_ADSB_ENABLED
    AP_ADSB *adsb = AP::ADSB();
    if (!adsb || !adsb->enabled()) {
        // ADSB not enabled
        return;
    }

    AP_ADSB::adsb_vehicle_t vehicle;
    mavlink_adsb_vehicle_t &pkt = vehicle.info;

    pkt.ICAO_address = msg.icao_address;
    pkt.tslc = msg.tslc;
    pkt.lat = msg.latitude_deg_1e7;
    pkt.lon = msg.longitude_deg_1e7;
    pkt.altitude = msg.alt_m * 1000;
    pkt.heading = degrees(msg.heading) * 100;
    pkt.hor_velocity = norm(msg.velocity[0], msg.velocity[1]) * 100;
    pkt.ver_velocity = -msg.velocity[2] * 100;
    pkt.squawk = msg.squawk;
    for (uint8_t i=0; i<9; i++) {
        pkt.callsign[i] = msg.callsign[i];
    }
    pkt.emitter_type = msg.traffic_type;

    if (msg.alt_type == ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_PRESSURE_AMSL) {
        pkt.flags |= ADSB_FLAGS_VALID_ALTITUDE;
        pkt.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
    } else if (msg.alt_type == ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_WGS84) {
        pkt.flags |= ADSB_FLAGS_VALID_ALTITUDE;
        pkt.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
    }

    if (msg.lat_lon_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_COORDS;
    }
    if (msg.heading_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_HEADING;
    }
    if (msg.velocity_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_VELOCITY;
    }
    if (msg.callsign_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_CALLSIGN;
    }
    if (msg.ident_valid) {
        pkt.flags |= ADSB_FLAGS_VALID_SQUAWK;
    }
    if (msg.simulated_report) {
        pkt.flags |= ADSB_FLAGS_SIMULATED;
    }
    if (msg.vertical_velocity_valid) {
        pkt.flags |= ADSB_FLAGS_VERTICAL_VELOCITY_VALID;
    }
    if (msg.baro_valid) {
        pkt.flags |= ADSB_FLAGS_BARO_VALID;
    }

    vehicle.last_update_ms = AP_HAL::native_millis() - (vehicle.info.tslc * 1000);
    adsb->handle_adsb_vehicle(vehicle);
#endif
}

/*
  handle actuator status message
 */
void AP_UAVCAN::handle_actuator_status(const CanardRxTransfer& transfer, const uavcan_equipment_actuator_Status& msg)
{
    // log as CSRV message
    AP::logger().Write_ServoStatus(AP_HAL::native_micros64(),
                                   msg.actuator_id,
                                   msg.position,
                                   msg.force,
                                   msg.speed,
                                   msg.power_rating_pct);
}

/*
  handle ESC status message
 */
void AP_UAVCAN::handle_ESC_status(const CanardRxTransfer& transfer, const uavcan_equipment_esc_Status& msg)
{
#if HAL_WITH_ESC_TELEM
    const uint8_t esc_offset = constrain_int16(_esc_offset.get(), 0, UAVCAN_SRV_NUMBER);
    const uint8_t esc_index = msg.esc_index + esc_offset;

    if (!is_esc_data_index_valid(esc_index)) {
        return;
    }

    TelemetryData t {
        .temperature_cdeg = int16_t((KELVIN_TO_C(msg.temperature)) * 100),
        .voltage = msg.voltage,
        .current = msg.current,
    };

    update_rpm(esc_index, msg.rpm);
    update_telem_data(esc_index, t,
        (isnanf(msg.current) ? 0 : AP_ESC_Telem_Backend::TelemetryType::CURRENT)
            | (isnanf(msg.voltage) ? 0 : AP_ESC_Telem_Backend::TelemetryType::VOLTAGE)
            | (isnanf(msg.temperature) ? 0 : AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE));
#endif
}

bool AP_UAVCAN::is_esc_data_index_valid(const uint8_t index) {
    if (index > UAVCAN_SRV_NUMBER) {
        // printf("UAVCAN: invalid esc index: %d. max index allowed: %d\n\r", index, UAVCAN_SRV_NUMBER);
        return false;
    }
    return true;
}

/*
  handle LogMessage debug
 */
void AP_UAVCAN::handle_debug(const CanardRxTransfer& transfer, const uavcan_protocol_debug_LogMessage& msg)
{
#if HAL_LOGGING_ENABLED
    if (AP::can().get_log_level() != AP_CANManager::LOG_NONE) {
        // log to onboard log and mavlink
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CAN[%u] %s", transfer.source_node_id, msg.text.data);
    } else {
        // only log to onboard log
        AP::logger().Write_MessageF("CAN[%u] %s", transfer.source_node_id, msg.text.data);
    }
#endif
}

void AP_UAVCAN::send_parameter_request()
{
    WITH_SEMAPHORE(_param_sem);
    if (param_request_sent) {
        return;
    }
    param_get_set_client.request(param_request_node_id, param_getset_req);
    param_request_sent = true;
}

bool AP_UAVCAN::set_parameter_on_node(uint8_t node_id, const char *name, float value, ParamGetSetFloatCb *cb)
{
    WITH_SEMAPHORE(_param_sem);
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        //busy
        return false;
    }
    param_getset_req.index = 0;
    param_getset_req.name.len = strncpy_noterm((char*)param_getset_req.name.data, name, sizeof(param_getset_req.name.data)-1);
    param_getset_req.value.real_value = value;
    param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
    param_float_cb = cb;
    param_request_sent = false;
    param_request_node_id = node_id;
    return true;
}

bool AP_UAVCAN::set_parameter_on_node(uint8_t node_id, const char *name, int32_t value, ParamGetSetIntCb *cb)
{
    WITH_SEMAPHORE(_param_sem);
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        //busy
        return false;
    }
    param_getset_req.index = 0;
    param_getset_req.name.len = strncpy_noterm((char*)param_getset_req.name.data, name, sizeof(param_getset_req.name.data)-1);
    param_getset_req.value.integer_value = value;
    param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
    param_int_cb = cb;
    param_request_sent = false;
    param_request_node_id = node_id;
    return true;
}

bool AP_UAVCAN::get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetFloatCb *cb)
{
    WITH_SEMAPHORE(_param_sem);
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        //busy
        return false;
    }
    param_getset_req.index = 0;
    param_getset_req.name.len = strncpy_noterm((char*)param_getset_req.name.data, name, sizeof(param_getset_req.name.data));
    param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
    param_float_cb = cb;
    param_request_sent = false;
    param_request_node_id = node_id;
    return true;
}

bool AP_UAVCAN::get_parameter_on_node(uint8_t node_id, const char *name, ParamGetSetIntCb *cb)
{
    WITH_SEMAPHORE(_param_sem);
    if (param_int_cb != nullptr ||
        param_float_cb != nullptr) {
        //busy
        return false;
    }
    param_getset_req.index = 0;
    param_getset_req.name.len = strncpy_noterm((char*)param_getset_req.name.data, name, sizeof(param_getset_req.name.data));
    param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
    param_int_cb = cb;
    param_request_sent = false;
    param_request_node_id = node_id;
    return true;
}

void AP_UAVCAN::handle_param_get_set_response(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetResponse& rsp)
{
    WITH_SEMAPHORE(_param_sem);
    if (!param_int_cb &&
        !param_float_cb) {
        return;
    }
    if ((rsp.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) && param_int_cb) {
        int32_t val = rsp.value.integer_value;
        if ((*param_int_cb)(this, transfer.source_node_id, (const char*)rsp.name.data, val)) {
            // we want the parameter to be set with val
            param_getset_req.index = 0;
            memcpy(param_getset_req.name.data, rsp.name.data, rsp.name.len);
            param_getset_req.value.integer_value = val;
            param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            param_request_sent = false;
            param_request_node_id = transfer.source_node_id;
            return;
        }
    } else if ((rsp.value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) && param_float_cb) {
        float val = rsp.value.real_value;
        if ((*param_float_cb)(this, transfer.source_node_id, (const char*)rsp.name.data, val)) {
            // we want the parameter to be set with val
            param_getset_req.index = 0;
            memcpy(param_getset_req.name.data, rsp.name.data, rsp.name.len);
            param_getset_req.value.real_value = val;
            param_getset_req.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            param_request_sent = false;
            param_request_node_id = transfer.source_node_id;
            return;
        }
    }
    param_int_cb = nullptr;
    param_float_cb = nullptr;
}


void AP_UAVCAN::send_parameter_save_request()
{
    WITH_SEMAPHORE(_param_save_sem);
    if (param_save_request_sent) {
        return;
    }
    param_save_client.request(param_save_request_node_id, param_save_req);
    param_save_request_sent = true;
}

bool AP_UAVCAN::save_parameters_on_node(uint8_t node_id, ParamSaveCb *cb)
{
    WITH_SEMAPHORE(_param_save_sem);
    if (save_param_cb != nullptr) {
        //busy
        return false;
    }

    param_save_req.opcode = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE;
    param_save_request_sent = false;
    param_save_request_node_id = node_id;
    save_param_cb = cb;
    return true;
}

// handle parameter save request response
void AP_UAVCAN::handle_param_save_response(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeResponse& rsp)
{
    WITH_SEMAPHORE(_param_save_sem);
    if (!save_param_cb) {
        return;
    }
    (*save_param_cb)(this, transfer.source_node_id, rsp.ok);
    save_param_cb = nullptr;
}

// Send Reboot command
// Note: Do not call this from outside UAVCAN thread context,
// THIS IS NOT A THREAD SAFE API!
void AP_UAVCAN::send_reboot_request(uint8_t node_id)
{
    // uavcan_protocol_RestartNode_Request request;
    // request.magic_number = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;
    // CubeFramework::Client<uavcan::protocol::RestartNode> client(*_node);
    // client.setCallback([](const uavcan::ServiceCallResult<uavcan::protocol::RestartNode>& call_result){});

    // client.call(node_id, request);
}

// check if a option is set and if it is then reset it to 0.
// return true if it was set
bool AP_UAVCAN::check_and_reset_option(Options option)
{
    bool ret = option_is_set(option);
    if (ret) {
        _options.set_and_save(int16_t(_options.get() & ~uint16_t(option)));
    }
    return ret;
}

// handle prearm check
bool AP_UAVCAN::prearm_check(char* fail_msg, uint8_t fail_msg_len) const
{
    // forward this to DNA_Server
    return _dna_server.prearm_check(fail_msg, fail_msg_len);
}

/*
  periodic logging
 */
void AP_UAVCAN::logging(void)
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_log_ms < 1000) {
        return;
    }
    last_log_ms = now_ms;
    if (HAL_NUM_CAN_IFACES <= _driver_index) {
        // no interface?
        return;
    }
    const auto *iface = hal.can[_driver_index];
    if (iface == nullptr) {
        return;
    }
    const auto *stats = iface->get_statistics();
    if (stats == nullptr) {
        // statistics not implemented on this interface
        return;
    }
    const auto &s = *stats;
    AP::logger().WriteStreaming("CANS",
                                "TimeUS,I,T,Trq,Trej,Tov,Tto,Tab,R,Rov,Rer,Bo,Etx,Stx,Ftx",
                                "s#-------------",
                                "F--------------",
                                "QBIIIIIIIIIIIII",
                                AP_HAL::micros64(),
                                _driver_index,
                                s.tx_success,
                                s.tx_requests,
                                s.tx_rejected,
                                s.tx_overflow,
                                s.tx_timedout,
                                s.tx_abort,
                                s.rx_received,
                                s.rx_overflow,
                                s.rx_errors,
                                s.num_busoff_err,
                                _esc_send_count,
                                _srv_send_count,
                                _fail_send_count);
#endif // HAL_LOGGING_ENABLED
}

#endif // HAL_NUM_CAN_IFACES
