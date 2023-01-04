#include "AP_Airspeed_UAVCAN.h"

#if AP_AIRSPEED_UAVCAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "AirSpeed"

AP_Airspeed_UAVCAN::DetectedModules AP_Airspeed_UAVCAN::_detected_modules[];
HAL_Semaphore AP_Airspeed_UAVCAN::_sem_registry;

// constructor
AP_Airspeed_UAVCAN::AP_Airspeed_UAVCAN(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{}

void AP_Airspeed_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }
#if AP_TEST_DRONECAN_DRIVERS
    get_uavcan_backend(ap_uavcan, 125); // add entry for test driver
#endif
    auto *airspeed_cb = Canard::allocate_arg_callback(ap_uavcan, &handle_airspeed);
    if (airspeed_cb == nullptr) {
        AP_BoardConfig::allocation_error("airspeed_cb");
    }
    auto *airspeed_sub = new Canard::Subscriber<uavcan_equipment_air_data_RawAirData_cxx_iface>{*airspeed_cb, ap_uavcan->get_driver_index()};
    if (airspeed_sub == nullptr) {
        AP_BoardConfig::allocation_error("airspeed_sub");
    }

#if AP_AIRSPEED_HYGROMETER_ENABLE
    auto *hygrometer_cb = Canard::allocate_arg_callback(ap_uavcan, &handle_hygrometer);
    if (hygrometer_cb == nullptr) {
        AP_BoardConfig::allocation_error("hygrometer_cb");
    }
    auto *hygrometer_sub = new Canard::Subscriber<dronecan_sensors_hygrometer_Hygrometer_cxx_iface>{*hygrometer_cb, ap_uavcan->get_driver_index()};
    if (hygrometer_sub == nullptr) {
        AP_BoardConfig::allocation_error("hygrometer_sub");
    }
#endif
}

AP_Airspeed_Backend* AP_Airspeed_UAVCAN::probe(AP_Airspeed &_frontend, uint8_t _instance, uint32_t previous_devid)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_UAVCAN* backend = nullptr;

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            const auto bus_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                            _detected_modules[i].ap_uavcan->get_driver_index(),
                                                            _detected_modules[i].node_id, 0);
            if (previous_devid != 0 && previous_devid != bus_id) {
                // match with previous ID only
                continue;
            }
            backend = new AP_Airspeed_UAVCAN(_frontend, _instance);
            if (backend == nullptr) {
                AP::can().log_text(AP_CANManager::LOG_INFO,
                                   LOG_TAG,
                                   "Failed register UAVCAN Airspeed Node %d on Bus %d\n",
                                   _detected_modules[i].node_id,
                                   _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                AP::can().log_text(AP_CANManager::LOG_INFO,
                                   LOG_TAG,
                                   "Registered UAVCAN Airspeed Node %d on Bus %d\n",
                                   _detected_modules[i].node_id,
                                   _detected_modules[i].ap_uavcan->get_driver_index());
                backend->set_bus_id(bus_id);
#if AP_TEST_DRONECAN_DRIVERS
                // send something to the UAVCAN node to initialise the driver
                hal.scheduler->register_timer_process(FUNCTOR_BIND(backend, &AP_Airspeed_UAVCAN::update_test_sensor, void));
#endif
            }
            break;
        }
    }

    return backend;
}

AP_Airspeed_UAVCAN* AP_Airspeed_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }

    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
        }
    }

    bool detected = false;
    for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
        if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
            // detected
            detected = true;
            break;
        }
    }

    if (!detected) {
        for (uint8_t i = 0; i < AIRSPEED_MAX_SENSORS; i++) {
            if (_detected_modules[i].ap_uavcan == nullptr) {
                _detected_modules[i].ap_uavcan = ap_uavcan;
                _detected_modules[i].node_id = node_id;
                break;
            }
        }
    }

    return nullptr;
}

void AP_Airspeed_UAVCAN::handle_airspeed(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_RawAirData &msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_UAVCAN* driver = get_uavcan_backend(ap_uavcan, transfer.source_node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_airspeed);
        driver->_pressure = msg.differential_pressure;
        if (!isnan(msg.static_air_temperature) &&
            msg.static_air_temperature > 0) {
            driver->_temperature = KELVIN_TO_C(msg.static_air_temperature);
            driver->_have_temperature = true;
        }
        driver->_last_sample_time_ms = AP_HAL::millis();
    }
}

#if AP_AIRSPEED_HYGROMETER_ENABLE
void AP_Airspeed_UAVCAN::handle_hygrometer(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const dronecan_sensors_hygrometer_Hygrometer &msg)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Airspeed_UAVCAN* driver = get_uavcan_backend(ap_uavcan, transfer.source_node_id);

    if (driver != nullptr) {
        WITH_SEMAPHORE(driver->_sem_airspeed);
        driver->_hygrometer.temperature = KELVIN_TO_C(msg.temperature);
        driver->_hygrometer.humidity = msg.humidity;
        driver->_hygrometer.last_sample_ms = AP_HAL::millis();
    }
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE

bool AP_Airspeed_UAVCAN::init()
{
    // always returns true
    return true;
}

bool AP_Airspeed_UAVCAN::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(_sem_airspeed);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    pressure = _pressure;

    return true;
}

bool AP_Airspeed_UAVCAN::get_temperature(float &temperature)
{
    if (!_have_temperature) {
        return false;
    }
    WITH_SEMAPHORE(_sem_airspeed);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    temperature = _temperature;

    return true;
}

#if AP_AIRSPEED_HYGROMETER_ENABLE
/*
  return hygrometer data if available
 */
bool AP_Airspeed_UAVCAN::get_hygrometer(uint32_t &last_sample_ms, float &temperature, float &humidity)
{
    if (_hygrometer.last_sample_ms == 0) {
        return false;
    }
    WITH_SEMAPHORE(_sem_airspeed);
    last_sample_ms = _hygrometer.last_sample_ms;
    temperature = _hygrometer.temperature;
    humidity = _hygrometer.humidity;
    return true;
}
#endif // AP_AIRSPEED_HYGROMETER_ENABLE

#if AP_TEST_DRONECAN_DRIVERS
void AP_Airspeed_UAVCAN::update_test_sensor() {
    uavcan_equipment_air_data_RawAirData msg {};
    msg.differential_pressure = AP::sitl()->state.airspeed_raw_pressure[get_instance()];

    // this was mostly swiped from SIM_Airspeed_DLVR:
    const float sim_alt = AP::sitl()->state.altitude;

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(sim_alt * 0.001f, sigma, delta, theta);

    // To Do: Add a sensor board temperature offset parameter
    msg.static_air_temperature = SSL_AIR_TEMPERATURE * theta + 25.0;

    static Canard::Publisher<uavcan_equipment_air_data_RawAirData_cxx_iface> raw_air_pub{CanardInterface::get_test_iface()};
    raw_air_pub.broadcast(msg);
}
#endif

#endif // AP_AIRSPEED_UAVCAN_ENABLED
