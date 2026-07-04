#include "vl53l1x.h"
#include "vl53l1_api.h"
#include "vl53l1_platform.h"

extern void VL53L1_IncTick(void);

static VL53L1_Dev_t dev;
static volatile bool g_dataReady = false;

void VL53L1X_NotifyDataReady(void)
{
    g_dataReady = true;
}

bool VL53L1X_DataReady(void)
{
    return g_dataReady;
}

int8_t VL53L1X_Init(void *i2c_handle)
{
    VL53L1_Error status;

    VL53L1_PlatformInit();

    dev.I2cDevAddr  = 0x52;
    dev.I2cHandle   = i2c_handle;

    status = VL53L1_DataInit(&dev);
    if (status != VL53L1_ERROR_NONE) return status;

    status = VL53L1_StaticInit(&dev);
    if (status != VL53L1_ERROR_NONE) return status;

    status = VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_LONG);
    if (status != VL53L1_ERROR_NONE) return status;

    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 200000);
    if (status != VL53L1_ERROR_NONE) return status;

    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev, 500);
    if (status != VL53L1_ERROR_NONE) return status;

    status = VL53L1_StartMeasurement(&dev);
    if (status != VL53L1_ERROR_NONE) return status;

    return 0;
}

int8_t VL53L1X_GetDistance(uint16_t *distance_mm)
{
    VL53L1_Error status;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;

    VL53L1_IncTick();

    if (g_dataReady) {
        g_dataReady = false;
    } else {
        status = VL53L1_GetMeasurementDataReady(&dev, &dataReady);
        if (status != VL53L1_ERROR_NONE) return status;
        if (!dataReady) return 1;
    }

    status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
    if (status != VL53L1_ERROR_NONE) return status;

    *distance_mm = rangingData.RangeMilliMeter;

    VL53L1_ClearInterruptAndStartMeasurement(&dev);

    return 0;
}
