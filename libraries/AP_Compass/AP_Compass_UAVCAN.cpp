/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Compass_UAVCAN.h"

#if AP_COMPASS_UAVCAN_ENABLED

#include <AP_HAL/AP_HAL.h>

#include <AP_CANManager/AP_CANManager.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

#define LOG_TAG "COMPASS"

AP_Compass_UAVCAN::DetectedModules AP_Compass_UAVCAN::_detected_modules[];
HAL_Semaphore AP_Compass_UAVCAN::_sem_registry;

AP_Compass_UAVCAN::AP_Compass_UAVCAN(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id, uint32_t devid)
    : _ap_uavcan(ap_uavcan)
    , _node_id(node_id)
    , _sensor_id(sensor_id)
    , _devid(devid)
{
}

void AP_Compass_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }
#if AP_TEST_DRONECAN_DRIVERS
    get_uavcan_backend(ap_uavcan, 125, ap_uavcan->get_driver_index()); // add entry for test driver
#endif
    auto *mag_cb = Canard::allocate_arg_callback(ap_uavcan, &handle_magnetic_field);
    if (mag_cb == nullptr) {
        AP_BoardConfig::allocation_error("fix2_cb");
    }
    auto *mag_sub = new Canard::Subscriber<uavcan_equipment_ahrs_MagneticFieldStrength_cxx_iface>{*mag_cb, ap_uavcan->get_driver_index()};
    if (mag_sub == nullptr) {
        AP_BoardConfig::allocation_error("mag_sub");
    }

    auto *mag2_cb = Canard::allocate_arg_callback(ap_uavcan, &handle_magnetic_field_2);
    if (mag2_cb == nullptr) {
        AP_BoardConfig::allocation_error("mag2_cb");
    }
    auto *mag2_sub = new Canard::Subscriber<uavcan_equipment_ahrs_MagneticFieldStrength2_cxx_iface>{*mag2_cb, ap_uavcan->get_driver_index()};
    if (mag2_sub == nullptr) {
        AP_BoardConfig::allocation_error("mag2_sub");
    }
}

AP_Compass_Backend* AP_Compass_UAVCAN::probe(uint8_t index)
{
    AP_Compass_UAVCAN* driver = nullptr;
    if (!_detected_modules[index].driver && _detected_modules[index].ap_uavcan) {
        WITH_SEMAPHORE(_sem_registry);
        // Register new Compass mode to a backend
        driver = new AP_Compass_UAVCAN(_detected_modules[index].ap_uavcan, _detected_modules[index].node_id, _detected_modules[index].sensor_id, _detected_modules[index].devid);
        if (driver) {
            if (!driver->init()) {
                delete driver;
                return nullptr;
            }
            _detected_modules[index].driver = driver;
            AP::can().log_text(AP_CANManager::LOG_INFO,
                                LOG_TAG,
                                "Found Mag Node %d on Bus %d Sensor ID %d\n",
                                _detected_modules[index].node_id,
                                _detected_modules[index].ap_uavcan->get_driver_index(),
                                _detected_modules[index].sensor_id);
#if AP_TEST_DRONECAN_DRIVERS
            // Scroll through the registered compasses, and set the offsets
            if (driver->_compass.get_offsets(index).is_zero()) {
                driver->_compass.set_offsets(index, AP::sitl()->mag_ofs[index]);
            }

            // we want to simulate a calibrated compass by default, so set
            // scale to 1
            AP_Param::set_default_by_name("COMPASS_SCALE", 1);
            AP_Param::set_default_by_name("COMPASS_SCALE2", 1);
            AP_Param::set_default_by_name("COMPASS_SCALE3", 1);
            driver->save_dev_id(index);
            driver->set_rotation(index, ROTATION_NONE);

            // make first compass external
            driver->set_external(index, true);
            // send something to the UAVCAN node to initialise the driver
            hal.scheduler->register_timer_process(FUNCTOR_BIND(driver, &AP_Compass_UAVCAN::update_test_sensor, void));
#endif
        }
    }
    return driver;
}

bool AP_Compass_UAVCAN::init()
{
    // Adding 1 is necessary to allow backward compatibilty, where this field was set as 1 by default
    if (!register_compass(_devid, _instance)) {
        return false;
    }

    set_dev_id(_instance, _devid);
    set_external(_instance, true);

    AP::can().log_text(AP_CANManager::LOG_INFO, LOG_TAG,  "AP_Compass_UAVCAN loaded\n\r");
    return true;
}

AP_Compass_UAVCAN* AP_Compass_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i=0; i<COMPASS_MAX_BACKEND; i++) {
        if (_detected_modules[i].driver &&
            _detected_modules[i].ap_uavcan == ap_uavcan &&
            _detected_modules[i].node_id == node_id &&
            _detected_modules[i].sensor_id == sensor_id) {
            return _detected_modules[i].driver;
        }
    }

    bool already_detected = false;
    // Check if there's an empty spot for possible registeration
    for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
        if (_detected_modules[i].ap_uavcan == ap_uavcan && 
            _detected_modules[i].node_id == node_id &&
            _detected_modules[i].sensor_id == sensor_id) {
            // Already Detected
            already_detected = true;
            break;
        }
    }
    if (!already_detected) {
        for (uint8_t i = 0; i < COMPASS_MAX_BACKEND; i++) {
            if (nullptr == _detected_modules[i].ap_uavcan) {
                _detected_modules[i].ap_uavcan = ap_uavcan;
                _detected_modules[i].node_id = node_id;
                _detected_modules[i].sensor_id = sensor_id;
                _detected_modules[i].devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                 ap_uavcan->get_driver_index(),
                                                 node_id,
                                                 sensor_id + 1); // we use sensor_id as devtype
                break;
            }
        }
    }

    struct DetectedModules tempslot;
    // Sort based on the node_id, larger values first
    // we do this, so that we have repeatable compass
    // registration, especially in cases of extraneous
    // CAN compass is connected.
    for (uint8_t i = 1; i < COMPASS_MAX_BACKEND; i++) {
        for (uint8_t j = i; j > 0; j--) {
            if (_detected_modules[j].node_id > _detected_modules[j-1].node_id) {
                tempslot = _detected_modules[j];
                _detected_modules[j] = _detected_modules[j-1];
                _detected_modules[j-1] = tempslot;
            }
        }
    }
    return nullptr;
}

void AP_Compass_UAVCAN::handle_mag_msg(const Vector3f &mag)
{
    Vector3f raw_field = mag * 1000.0;

    accumulate_sample(raw_field, _instance);
}

void AP_Compass_UAVCAN::handle_magnetic_field(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength& msg)
{
    WITH_SEMAPHORE(_sem_registry);

    Vector3f mag_vector;
    AP_Compass_UAVCAN* driver = get_uavcan_backend(ap_uavcan, transfer.source_node_id, 0);
    if (driver != nullptr) {
        mag_vector[0] = msg.magnetic_field_ga[0];
        mag_vector[1] = msg.magnetic_field_ga[1];
        mag_vector[2] = msg.magnetic_field_ga[2];
        driver->handle_mag_msg(mag_vector);
    }
}

void AP_Compass_UAVCAN::handle_magnetic_field_2(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength2 &msg)
{
    WITH_SEMAPHORE(_sem_registry);

    Vector3f mag_vector;
    uint8_t sensor_id = msg.sensor_id;
    AP_Compass_UAVCAN* driver = get_uavcan_backend(ap_uavcan, transfer.source_node_id, sensor_id);
    if (driver != nullptr) {
        mag_vector[0] = msg.magnetic_field_ga[0];
        mag_vector[1] = msg.magnetic_field_ga[1];
        mag_vector[2] = msg.magnetic_field_ga[2];
        driver->handle_mag_msg(mag_vector);
    }
}

void AP_Compass_UAVCAN::read(void)
{
    drain_accumulated_samples(_instance);
}

#if AP_TEST_DRONECAN_DRIVERS
void AP_Compass_UAVCAN::_setup_eliptical_correcion(uint8_t i)
{
    Vector3f diag = AP::sitl()->mag_diag[i].get();
    if (diag.is_zero()) {
        diag = {1,1,1};
    }
    const Vector3f &diagonals = diag;
    const Vector3f &offdiagonals = AP::sitl()->mag_offdiag[i];
    
    if (diagonals == _last_dia && offdiagonals == _last_odi) {
        return;
    }
    
    _eliptical_corr = Matrix3f(diagonals.x,    offdiagonals.x, offdiagonals.y,
                               offdiagonals.x, diagonals.y,    offdiagonals.z,
                               offdiagonals.y, offdiagonals.z, diagonals.z);
    if (!_eliptical_corr.invert()) {
        _eliptical_corr.identity();
    }
    _last_dia = diag;
    _last_odi = offdiagonals;
}

void AP_Compass_UAVCAN::update_test_sensor() {

    // Sampled at 100Hz
    uint32_t now = AP_HAL::millis();
    if ((now - _test_sensor_last_update_ms) < 10) {
        return;
    }
    _test_sensor_last_update_ms = now;

    // calculate sensor noise and add to 'truth' field in body frame
    // units are milli-Gauss
    Vector3f noise = rand_vec3f() * AP::sitl()->mag_noise;
    Vector3f new_mag_data = AP::sitl()->state.bodyMagField + noise;

    _setup_eliptical_correcion(_instance);
    Vector3f f = (_eliptical_corr * new_mag_data) - AP::sitl()->mag_ofs[_instance].get();
    // rotate compass
    f.rotate_inverse((enum Rotation)AP::sitl()->mag_orient[_instance].get());
    f.rotate(get_board_orientation());
    // scale the compass to simulate sensor scale factor errors
    f *= AP::sitl()->mag_scaling[_instance];

    static Canard::Publisher<uavcan_equipment_ahrs_MagneticFieldStrength_cxx_iface> mag_pub{CanardInterface::get_test_iface()};
    uavcan_equipment_ahrs_MagneticFieldStrength mag_msg {};
    mag_msg.magnetic_field_ga[0] = f.x/1000.0f;
    mag_msg.magnetic_field_ga[1] = f.y/1000.0f;
    mag_msg.magnetic_field_ga[2] = f.z/1000.0f;
    mag_msg.magnetic_field_covariance.len = 0;
    mag_pub.broadcast(mag_msg);
    static Canard::Publisher<uavcan_equipment_ahrs_MagneticFieldStrength2_cxx_iface> mag2_pub{CanardInterface::get_test_iface()};
    uavcan_equipment_ahrs_MagneticFieldStrength2 mag2_msg;
    mag2_msg.magnetic_field_ga[0] = f.x/1000.0f;
    mag2_msg.magnetic_field_ga[1] = f.y/1000.0f;
    mag2_msg.magnetic_field_ga[2] = f.z/1000.0f;
    mag2_msg.sensor_id = _instance;
    mag2_msg.magnetic_field_covariance.len = 0;
    mag2_pub.broadcast(mag2_msg);
}
#endif
#endif  // AP_COMPASS_UAVCAN_ENABLED
