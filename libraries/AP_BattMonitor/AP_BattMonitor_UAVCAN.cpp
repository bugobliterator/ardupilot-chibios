#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#define LOG_TAG "BattMon"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_UAVCAN::var_info[] = {

    // @Param: CURR_MULT
    // @DisplayName: Scales reported power monitor current
    // @Description: Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications
    // @Range: .1 10
    // @User: Advanced
    AP_GROUPINFO("CURR_MULT", 30, AP_BattMonitor_UAVCAN, _curr_mult, 1.0),

    // Param indexes must be between 30 and 39 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_UAVCAN::AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _type(type)
{
    AP_Param::setup_object_defaults(this,var_info);
    _state.var_info = var_info;

    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto *battinfo_cb = Canard::allocate_arg_callback(ap_uavcan, &handle_battery_info_trampoline);
    if (battinfo_cb == nullptr) {
        AP_BoardConfig::allocation_error("battinfo_cb");
    }
    auto *battinfo_sub = new Canard::Subscriber<uavcan_equipment_power_BatteryInfo_cxx_iface>{*battinfo_cb, ap_uavcan->get_driver_index()};
    if (battinfo_sub == nullptr) {
        AP_BoardConfig::allocation_error("battinfo_sub");
    }

    auto *battinfo_aux_cb = Canard::allocate_arg_callback(ap_uavcan, &handle_battery_info_aux_trampoline);
    if (battinfo_aux_cb == nullptr) {
        AP_BoardConfig::allocation_error("battinfo_aux_cb");
    }
    auto *battinfo_aux_sub = new Canard::Subscriber<ardupilot_equipment_power_BatteryInfoAux_cxx_iface>{*battinfo_aux_cb, ap_uavcan->get_driver_index()};
    if (battinfo_aux_sub == nullptr) {
        AP_BoardConfig::allocation_error("battinfo_aux_sub");
    }

    auto *mppt_stream_cb = Canard::allocate_arg_callback(ap_uavcan, &handle_mppt_stream_trampoline);
    if (mppt_stream_cb == nullptr) {
        AP_BoardConfig::allocation_error("mppt_stream_cb");
    }
    auto *mppt_stream_sub = new Canard::Subscriber<mppt_Stream_cxx_iface>{*mppt_stream_cb, ap_uavcan->get_driver_index()};
    if (mppt_stream_sub == nullptr) {
        AP_BoardConfig::allocation_error("mppt_stream_sub");
    }
}

AP_BattMonitor_UAVCAN* AP_BattMonitor_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t battery_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < AP::battery()._num_instances; i++) {
        if (AP::battery().drivers[i] == nullptr ||
            AP::battery().get_type(i) != AP_BattMonitor::Type::UAVCAN_BatteryInfo) {
            continue;
        }
        AP_BattMonitor_UAVCAN* driver = (AP_BattMonitor_UAVCAN*)AP::battery().drivers[i];
        if (driver->_ap_uavcan == ap_uavcan && driver->_node_id == node_id && match_battery_id(i, battery_id)) {
            return driver;
        }
    }
    // find empty uavcan driver
    for (uint8_t i = 0; i < AP::battery()._num_instances; i++) {
        if (AP::battery().drivers[i] != nullptr &&
            AP::battery().get_type(i) == AP_BattMonitor::Type::UAVCAN_BatteryInfo &&
            match_battery_id(i, battery_id)) {

            AP_BattMonitor_UAVCAN* batmon = (AP_BattMonitor_UAVCAN*)AP::battery().drivers[i];
            if(batmon->_ap_uavcan != nullptr || batmon->_node_id != 0) {
                continue;
            }
            batmon->_ap_uavcan = ap_uavcan;
            batmon->_node_id = node_id;
            batmon->_instance = i;
            batmon->init();
            AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Registered BattMonitor Node %d on Bus %d\n",
                            node_id,
                            ap_uavcan->get_driver_index());
            return batmon;
        }
    }
    return nullptr;
}

void AP_BattMonitor_UAVCAN::handle_battery_info(const uavcan_equipment_power_BatteryInfo &msg)
{
    update_interim_state(msg.voltage, msg.current, msg.temperature, msg.state_of_charge_pct); 

    WITH_SEMAPHORE(_sem_battmon);
    _remaining_capacity_wh = msg.remaining_capacity_wh;
    _full_charge_capacity_wh = msg.full_charge_capacity_wh;
}

void AP_BattMonitor_UAVCAN::update_interim_state(const float voltage, const float current, const float temperature_K, const uint8_t soc)
{
    WITH_SEMAPHORE(_sem_battmon);

    _interim_state.voltage = voltage;
    _interim_state.current_amps = _curr_mult * current;
    _soc = soc;

    if (!isnanf(temperature_K) && temperature_K > 0) {
        // Temperature reported from battery in kelvin and stored internally in Celsius.
        _interim_state.temperature = KELVIN_TO_C(temperature_K);
        _interim_state.temperature_time = AP_HAL::millis();
    }

    const uint32_t tnow = AP_HAL::micros();

    if (!_has_battery_info_aux || _mppt.is_detected) {
        const uint32_t dt_us = tnow - _interim_state.last_time_micros;

        // update total current drawn since startup
        update_consumed(_interim_state, dt_us);
    }

    // record time
    _interim_state.last_time_micros = tnow;
    _interim_state.healthy = true;
}

void AP_BattMonitor_UAVCAN::handle_battery_info_aux(const ardupilot_equipment_power_BatteryInfoAux &msg)
{
    WITH_SEMAPHORE(_sem_battmon);
    uint8_t cell_count = MIN(ARRAY_SIZE(_interim_state.cell_voltages.cells), msg.voltage_cell.len);
    float remaining_capacity_ah = _remaining_capacity_wh / msg.nominal_voltage;
    float full_charge_capacity_ah = _full_charge_capacity_wh / msg.nominal_voltage;

    _cycle_count = msg.cycle_count;
    for (uint8_t i = 0; i < cell_count; i++) {
        _interim_state.cell_voltages.cells[i] = msg.voltage_cell.data[i] * 1000;
    }
    _interim_state.is_powering_off = msg.is_powering_off;
    _interim_state.consumed_mah = (full_charge_capacity_ah - remaining_capacity_ah) * 1000;
    _interim_state.consumed_wh = _full_charge_capacity_wh - _remaining_capacity_wh;
    _interim_state.time_remaining =  is_zero(_interim_state.current_amps) ? 0 : (remaining_capacity_ah / _interim_state.current_amps * 3600);
    _interim_state.has_time_remaining = true;

    _has_cell_voltages = true;
    _has_time_remaining = true;
    _has_consumed_energy = true;
    _has_battery_info_aux = true;
}

void AP_BattMonitor_UAVCAN::handle_mppt_stream(const mppt_Stream &msg)
{
    const bool use_input_value = (uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::MPPT_Use_Input_Value)) != 0;
    const float voltage = use_input_value ? msg.input_voltage : msg.output_voltage;
    const float current = use_input_value ? msg.input_current : msg.output_current;

    // use an invalid soc so we use the library calculated one
    const uint8_t soc = 127;

    // convert C to Kelvin
    const float temperature_K = isnanf(msg.temperature) ? 0 : C_TO_KELVIN(msg.temperature);

    update_interim_state(voltage, current, temperature_K, soc); 

    if (!_mppt.is_detected) {
        // this is the first time the mppt message has been received
        // so set powered up state
        _mppt.is_detected = true;
        mppt_set_bootup_powered_state();
    }

    mppt_check_and_report_faults(msg.fault_flags);
}

void AP_BattMonitor_UAVCAN::handle_battery_info_trampoline(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const uavcan_equipment_power_BatteryInfo &msg)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, transfer.source_node_id, msg.battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info(msg);
}

void AP_BattMonitor_UAVCAN::handle_battery_info_aux_trampoline(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const ardupilot_equipment_power_BatteryInfoAux &msg)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, transfer.source_node_id, msg.battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info_aux(msg);
}

void AP_BattMonitor_UAVCAN::handle_mppt_stream_trampoline(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const mppt_Stream &msg)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, transfer.source_node_id, transfer.source_node_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_mppt_stream(msg);
}

// read - read the voltage and current
void AP_BattMonitor_UAVCAN::read()
{
    uint32_t tnow = AP_HAL::micros();

    // timeout after 5 seconds
    if ((tnow - _interim_state.last_time_micros) > AP_BATTMONITOR_UAVCAN_TIMEOUT_MICROS) {
        _interim_state.healthy = false;
    }
    // Copy over relevant states over to main state
    WITH_SEMAPHORE(_sem_battmon);
    _state.temperature = _interim_state.temperature;
    _state.temperature_time = _interim_state.temperature_time;
    _state.voltage = _interim_state.voltage;
    _state.current_amps = _interim_state.current_amps;
    _state.consumed_mah = _interim_state.consumed_mah;
    _state.consumed_wh = _interim_state.consumed_wh;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.healthy = _interim_state.healthy;
    _state.time_remaining = _interim_state.time_remaining;
    _state.has_time_remaining = _interim_state.has_time_remaining;
    _state.is_powering_off = _interim_state.is_powering_off;
    memcpy(_state.cell_voltages.cells, _interim_state.cell_voltages.cells, sizeof(_state.cell_voltages));

    _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;

    // check if MPPT should be powered on/off depending upon arming state
    if (_mppt.is_detected) {
        mppt_set_armed_powered_state();
    }
}

/// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
bool AP_BattMonitor_UAVCAN::capacity_remaining_pct(uint8_t &percentage) const
{
    if ((uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::Ignore_UAVCAN_SoC)) ||
        _mppt.is_detected ||
        _soc == 127) {
        // a UAVCAN battery monitor may not be able to supply a state of charge. If it can't then
        // the user can set the option to use current integration in the backend instead.
        // SOC of 127 is used as an invalid SOC flag ie system configuration errors or SOC estimation unavailable
        return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
    }

    // the monitor must have current readings in order to estimate consumed_mah and be healthy
    if (!has_current() || !_state.healthy) {
        return false;
    }

    percentage = _soc;
    return true;
}

/// get_cycle_count - return true if cycle count can be provided and fills in cycles argument
bool AP_BattMonitor_UAVCAN::get_cycle_count(uint16_t &cycles) const
{
    if (_has_battery_info_aux) {
        cycles = _cycle_count;
        return true;
    }
    return false;
}

// request MPPT board to power on/off at boot as specified by BATT_OPTIONS
void AP_BattMonitor_UAVCAN::mppt_set_bootup_powered_state()
{
    const uint32_t options = uint32_t(_params._options.get());
    const bool on_at_boot = (options & uint32_t(AP_BattMonitor_Params::Options::MPPT_Power_On_At_Boot)) != 0;
    const bool off_at_boot = (options & uint32_t(AP_BattMonitor_Params::Options::MPPT_Power_Off_At_Boot)) != 0;

    if (on_at_boot) {
        mppt_set_powered_state(true, true);
    } else if (off_at_boot) {
        mppt_set_powered_state(false, true);
    }
}

// request MPPT board to power on/off depending upon vehicle arming state as specified by BATT_OPTIONS
void AP_BattMonitor_UAVCAN::mppt_set_armed_powered_state()
{
    // check if vehicle armed state has changed
    const bool vehicle_armed = hal.util->get_soft_armed();
    if (vehicle_armed == _mppt.vehicle_armed_last) {
        return;
    }
    _mppt.vehicle_armed_last = vehicle_armed;

    // check options for arming state change events
    const uint32_t options = uint32_t(_params._options.get());
    const bool power_on_at_arm = (options & uint32_t(AP_BattMonitor_Params::Options::MPPT_Power_On_At_Arm)) != 0;
    const bool power_off_at_disarm = (options & uint32_t(AP_BattMonitor_Params::Options::MPPT_Power_Off_At_Disarm)) != 0;

    if (vehicle_armed && power_on_at_arm) {
        mppt_set_powered_state(true, false);
    } else if (!vehicle_armed && power_off_at_disarm) {
        mppt_set_powered_state(false, false);
    }
}

// request MPPT board to power on or off
// power_on should be true to power on the MPPT, false to power off
// force should be true to force sending the state change request to the MPPT
void AP_BattMonitor_UAVCAN::mppt_set_powered_state(bool power_on, bool force)
{
    if (_ap_uavcan == nullptr || !_mppt.is_detected) {
        return;
    }

    // return immediately if already desired state and not forced
    if ((_mppt.powered_state == power_on) && !force) {
        return;
    }
    _mppt.powered_state = power_on;
    _mppt.powered_state_changed = true;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: powering %s", (unsigned)_instance+1, _mppt.powered_state ? "ON" : "OFF");

    mppt_OutputEnableRequest request;
    request.enable = _mppt.powered_state;
    request.disable = !request.enable;

    if (mppt_outputenable_client == nullptr) {
        mppt_outputenable_client = new Canard::Client<mppt_OutputEnable_cxx_iface>{_ap_uavcan->get_canard_iface(), mppt_outputenable_res_cb};
        if (mppt_outputenable_client == nullptr) {
            return;
        }
    }
    mppt_outputenable_client->request(_node_id, request);
}

// report changes in MPPT faults
void AP_BattMonitor_UAVCAN::mppt_check_and_report_faults(uint8_t fault_flags)
{
    // return immediately if no changes
    if (_mppt.fault_flags == fault_flags) {
        return;
    }
    _mppt.fault_flags = fault_flags;

    // handle recovery
    if (_mppt.fault_flags == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: OK", (unsigned)_instance+1);
        return;
    }

    // send battery faults via text messages
    for (uint8_t fault_bit=0x01; fault_bit <= 0x08; fault_bit <<= 1) {
        // this loop is to generate multiple messages if there are multiple concurrent faults, but also run once if there are no faults
        if ((fault_bit & fault_flags) != 0) {
            const MPPT_FaultFlags err = (MPPT_FaultFlags)fault_bit;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: %s", (unsigned)_instance+1, mppt_fault_string(err));
        }
    }
}

// returns string description of MPPT fault bit. Only handles single bit faults
const char* AP_BattMonitor_UAVCAN::mppt_fault_string(MPPT_FaultFlags fault)
{
    switch (fault) {
        case MPPT_FaultFlags::OVER_VOLTAGE:
            return "over voltage";
        case MPPT_FaultFlags::UNDER_VOLTAGE:
            return "under voltage";
        case MPPT_FaultFlags::OVER_CURRENT:
            return "over current";
        case MPPT_FaultFlags::OVER_TEMPERATURE:
            return "over temp";
    }
    return "unknown";
}

// return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
uint32_t AP_BattMonitor_UAVCAN::get_mavlink_fault_bitmask() const
{
    // return immediately if not mppt or no faults
    if (!_mppt.is_detected || (_mppt.fault_flags == 0)) {
        return 0;
    }

    // convert mppt fault bitmask to mavlink fault bitmask
    uint32_t mav_fault_bitmask = 0;
    if ((_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_VOLTAGE) || (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::UNDER_VOLTAGE)) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE;
    }
    if (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_CURRENT) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_OVER_CURRENT;
    }
    if (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_TEMPERATURE) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_OVER_TEMPERATURE;
    }
    return mav_fault_bitmask;
}

#endif
