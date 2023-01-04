#pragma once

#include "AP_Compass.h"

#if AP_COMPASS_UAVCAN_ENABLED

#include "AP_Compass_Backend.h"

#include <AP_UAVCAN/AP_UAVCAN.h>

class AP_Compass_UAVCAN : public AP_Compass_Backend {
public:
    AP_Compass_UAVCAN(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id, uint32_t devid);

    void        read(void) override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);
    static AP_Compass_Backend* probe(uint8_t index);
    static uint32_t get_detected_devid(uint8_t index) { return _detected_modules[index].devid; }
    static void handle_magnetic_field(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength& msg);
    static void handle_magnetic_field_2(AP_UAVCAN *ap_uavcan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_MagneticFieldStrength2 &msg);
#if AP_TEST_DRONECAN_DRIVERS
    void update_test_sensor();
    void _setup_eliptical_correcion(uint8_t i);
#endif

private:
    bool init();

    // callback for DroneCAN messages
    void handle_mag_msg(const Vector3f &mag);

    static AP_Compass_UAVCAN* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t sensor_id);

    uint8_t  _instance;

    AP_UAVCAN* _ap_uavcan;
    uint8_t _node_id;
    uint8_t _sensor_id;
    uint32_t _devid;

    // Module Detection Registry
    static struct DetectedModules {
        AP_UAVCAN* ap_uavcan;
        uint8_t node_id;
        uint8_t sensor_id;
        AP_Compass_UAVCAN *driver;
        uint32_t devid;
    } _detected_modules[COMPASS_MAX_BACKEND];

    static HAL_Semaphore _sem_registry;
#if AP_TEST_DRONECAN_DRIVERS
    Matrix3f _eliptical_corr;
    Vector3f _last_dia;
    Vector3f _last_odi;
    uint32_t _test_sensor_last_update_ms;
#endif
};

#endif  // AP_COMPASS_UAVCAN_ENABLED
