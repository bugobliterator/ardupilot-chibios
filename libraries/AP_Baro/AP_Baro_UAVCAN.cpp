#include "AP_Baro_UAVCAN.h"

#if AP_BARO_UAVCAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_Baro_SITL.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "Baro"

AP_Baro_UAVCAN::DetectedModules AP_Baro_UAVCAN::_detected_modules[];
HAL_Semaphore AP_Baro_UAVCAN::_sem_registry;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_UAVCAN::AP_Baro_UAVCAN(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{}

void AP_Baro_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }
#if AP_TEST_DRONECAN_DRIVERS
    get_uavcan_backend(ap_uavcan, 125, true); // add entry for test driver
#endif
    auto *pressure_cb = Canard::allocate_arg_callback(ap_uavcan, &handle_pressure);
    if (pressure_cb == nullptr) {
        AP_BoardConfig::allocation_error("pressure_cb");
    }
    auto *pressure_sub = new Canard::Subscriber<uavcan_equipment_air_data_StaticPressure_cxx_iface>{*pressure_cb, ap_uavcan->get_driver_index()};
    if (pressure_sub == nullptr) {
        AP_BoardConfig::allocation_error("pressure_sub");
    }

    auto *temperature_cb = Canard::allocate_arg_callback(ap_uavcan, &handle_temperature);
    if (temperature_cb == nullptr) {
        AP_BoardConfig::allocation_error("temperature_cb");
    }
    auto *temperature_sub = new Canard::Subscriber<uavcan_equipment_air_data_StaticTemperature_cxx_iface>{*temperature_cb, ap_uavcan->get_driver_index()};
    if (temperature_sub == nullptr) {
        AP_BoardConfig::allocation_error("temperature_sub");
    }
}

AP_Baro_Backend* AP_Baro_UAVCAN::probe(AP_Baro &baro)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Baro_UAVCAN* backend = nullptr;
    for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_uavcan != nullptr) {
            backend = new AP_Baro_UAVCAN(baro);
            if (backend == nullptr) {
                AP::can().log_text(AP_CANManager::LOG_ERROR,
                            LOG_TAG,
                            "Failed register UAVCAN Baro Node %d on Bus %d\n",
                            _detected_modules[i].node_id,
                            _detected_modules[i].ap_uavcan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                backend->_pressure = 0;
                backend->_pressure_count = 0;
                backend->_ap_uavcan = _detected_modules[i].ap_uavcan;
                backend->_node_id = _detected_modules[i].node_id;

                backend->_instance = backend->_frontend.register_sensor();
                backend->set_bus_id(backend->_instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                                                    _detected_modules[i].ap_uavcan->get_driver_index(),
                                                                                    backend->_node_id, 0));

                AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Registered UAVCAN Baro Node %d on Bus %d\n",
                            _detected_modules[i].node_id,
                            _detected_modules[i].ap_uavcan->get_driver_index());
#if AP_TEST_DRONECAN_DRIVERS
                // send something to the UAVCAN node to initialise the driver
                hal.scheduler->register_timer_process(FUNCTOR_BIND(backend, &AP_Baro_UAVCAN::update_test_sensor, void));
#endif
            }
            break;
        }
    }
    return backend;
}

AP_Baro_UAVCAN* AP_Baro_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, bool create_new)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_uavcan == ap_uavcan && 
            _detected_modules[i].node_id == node_id) {
            return _detected_modules[i].driver;
        }
    }
    
    if (create_new) {
        bool already_detected = false;
        //Check if there's an empty spot for possible registeration
        for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
            if (_detected_modules[i].ap_uavcan == ap_uavcan && _detected_modules[i].node_id == node_id) {
                //Already Detected
                already_detected = true;
                break;
            }
        }
        if (!already_detected) {
            for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
                if (_detected_modules[i].ap_uavcan == nullptr) {
                    _detected_modules[i].ap_uavcan = ap_uavcan;
                    _detected_modules[i].node_id = node_id;
                    break;
                }
            }
        }
    }

    return nullptr;
}


void AP_Baro_UAVCAN::_update_and_wrap_accumulator(float *accum, float val, uint8_t *count, const uint8_t max_count)
{
    *accum += val;
    *count += 1;
    if (*count == max_count) {
        *count = max_count / 2;
        *accum = *accum / 2;
    }
}

void AP_Baro_UAVCAN::handle_pressure(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticPressure &msg)
{
    AP_Baro_UAVCAN* driver;
    {
        WITH_SEMAPHORE(_sem_registry);
        driver = get_uavcan_backend(ap_uavcan, transfer.source_node_id, true);
        if (driver == nullptr) {
            return;
        }
    }
    {
        WITH_SEMAPHORE(driver->_sem_baro);
        _update_and_wrap_accumulator(&driver->_pressure, msg.static_pressure, &driver->_pressure_count, 32);
        driver->new_pressure = true;
    }
}

void AP_Baro_UAVCAN::handle_temperature(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticTemperature &msg)
{
    AP_Baro_UAVCAN* driver;
    {
        WITH_SEMAPHORE(_sem_registry);
        driver = get_uavcan_backend(ap_uavcan, transfer.source_node_id, false);
        if (driver == nullptr) {
            return;
        }
    }
    {
        WITH_SEMAPHORE(driver->_sem_baro);
        driver->_temperature = KELVIN_TO_C(msg.static_temperature);
    }
}

// Read the sensor
void AP_Baro_UAVCAN::update(void)
{
    float pressure = 0;

    WITH_SEMAPHORE(_sem_baro);
    if (new_pressure) {
        if (_pressure_count != 0) {
            pressure = _pressure / _pressure_count;
            _pressure_count = 0;
            _pressure = 0;
        }
        _copy_to_frontend(_instance, pressure, _temperature);


        _frontend.set_external_temperature(_temperature);
        new_pressure = false;
    }
}

#if AP_TEST_DRONECAN_DRIVERS
void AP_Baro_UAVCAN::update_test_sensor() {
    const uint32_t now = AP_HAL::millis();
    if ((now - _test_sensor_last_update_ms) < 10) {
        return;
    }
    _test_sensor_last_update_ms = now;

    float sim_alt = AP::sitl()->state.altitude;

    sim_alt += AP::sitl()->baro[_instance].drift * now * 0.001f;
    sim_alt += AP::sitl()->baro[_instance].noise * rand_float();

#if !APM_BUILD_TYPE(APM_BUILD_ArduSub)
    float sigma, delta, theta;

    AP_Baro::SimpleAtmosphere(sim_alt * 0.001f, sigma, delta, theta);
    float p = SSL_AIR_PRESSURE * delta;
    float T = SSL_AIR_TEMPERATURE * theta;

    AP_Baro_SITL::temperature_adjustment(p, T);
#else
    float rho, delta, theta;
    AP_Baro::SimpleUnderWaterAtmosphere(-sim_alt * 0.001f, rho, delta, theta);
    float p = SSL_AIR_PRESSURE * delta;
    float T = SSL_AIR_TEMPERATURE * theta;
#endif

    // add in correction for wind effects
    p += AP_Baro_SITL::wind_pressure_correction(_instance);

    static Canard::Publisher<uavcan_equipment_air_data_StaticPressure_cxx_iface> press_pub{CanardInterface::get_test_iface()};
    static Canard::Publisher<uavcan_equipment_air_data_StaticTemperature_cxx_iface> temp_pub{CanardInterface::get_test_iface()};
    uavcan_equipment_air_data_StaticPressure press_msg {};
    press_msg.static_pressure = p;
    press_pub.broadcast(press_msg);
    uavcan_equipment_air_data_StaticTemperature temp_msg {};
    temp_msg.static_temperature = T;
    temp_pub.broadcast(temp_msg);
}
#endif

#endif // AP_BARO_UAVCAN_ENABLED
