#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/jiffies.h>

#define hts221_AVGH_4	0x00	 // Internal average on 4 samples
#define hts221_AVGH_8	0x01	 // Internal average on 8 samples
#define hts221_AVGH_16	0x02	 // Internal average on 16 samples
#define hts221_AVGH_32	0x03	 // Internal average on 32 samples
#define hts221_AVGH_64	0x04	 // Internal average on 64 samples
#define hts221_AVGH_128	0x05	 // Internal average on 128 samples
#define hts221_AVGH_256	0x06	 // Internal average on 256 samples
#define hts221_AVGH_512	0x07	 // Internal average on 512 samples

#define hts221_AVGT_2	0x00	// Internal average on 2 samples
#define hts221_AVGT_4	0x08	// Internal average on 4 samples
#define hts221_AVGT_8	0x10	// Internal average on 8 samples
#define hts221_AVGT_16	0x18	// Internal average on 16 samples
#define hts221_AVGT_32	0x20	// Internal average on 32 samples
#define hts221_AVGT_64	0x28	// Internal average on 64 samples
#define hts221_AVGT_128	0x30	// Internal average on 128 samples
#define hts221_AVGT_256	0x38	// Internal average on 256 samples

#define hts221_ODR_ONE_SHOT	0x00	// Output Data Rate: one shot
#define hts221_ODR_1HZ		0x01	// Output Data Rate: 1Hz
#define hts221_ODR_7HZ		0x02	// Output Data Rate: 7Hz
#define hts221_ODR_12_5HZ	0x03	// Output Data Rate: 12.5Hz

/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0xBC
* 7:0 This read-only register contains the device identifier for hts221.
* \endcode
*/
#define hts221_WHO_AM_I_REG          (uint8_t)0x0F

/**
* @brief Device Identification value.
*/
#define hts221_WHO_AM_I_VAL         (uint8_t)0xBC


/**
* @brief Humidity and temperature average mode register.
* \code
* Read/write
* Default value: 0x1B
* 7:6 Reserved.
* 5:3 AVGT2-AVGT1-AVGT0: Select the temperature internal average.
*
*      AVGT2 | AVGT1 | AVGT0 | Nr. Internal Average
*   ----------------------------------------------------
*       0    |   0   |   0   |    2
*       0    |   0   |   1   |    4
*       0    |   1   |   0   |    8
*       0    |   1   |   1   |    16
*       1    |   0   |   0   |    32
*       1    |   0   |   1   |    64
*       1    |   1   |   0   |    128
*       1    |   1   |   1   |    256
*
* 2:0 AVGH2-AVGH1-AVGH0: Select humidity internal average.
*      AVGH2 | AVGH1 |  AVGH0 | Nr. Internal Average
*   ------------------------------------------------------
*       0    |   0   |   0   |    4 
*       0    |   0   |   1   |    8  
*       0    |   1   |   0   |    16
*       0    |   1   |   1   |    32
*       1    |   0   |   0   |    64
*       1    |   0   |   1   |    128
*       1    |   1   |   0   |    256
*       1    |   1   |   1   |    512
*
* \endcode
*/
#define hts221_AV_CONF_REG        (uint8_t)0x10

#define hts221_AVGT_BIT           hts221_BIT(3)
#define hts221_AVGH_BIT           hts221_BIT(0)

#define hts221_AVGH_MASK          (uint8_t)0x07
#define hts221_AVGT_MASK          (uint8_t)0x38

/**
* @brief Control register 1.
* \code
* Read/write
* Default value: 0x00
* 7 PD: power down control. 0 - power down mode; 1 - active mode.
* 6:3 Reserved.
* 2 BDU: block data update. 0 - continuous update; 1 - output registers not updated until MSB and LSB reading.
* 1:0 ODR1, ODR0: output data rate selection.
*
*   ODR1  | ODR0  | Humidity output data-rate(Hz)  | Pressure output data-rate(Hz)
*   ----------------------------------------------------------------------------------
*     0   |   0   |         one shot               |         one shot
*     0   |   1   |            1                   |            1
*     1   |   0   |            7                   |            7
*     1   |   1   |           12.5                 |           12.5
*
* \endcode
*/
#define hts221_CTRL_REG1      (uint8_t)0x20

#define hts221_PD_BIT          hts221_BIT(7)
#define hts221_BDU_BIT         hts221_BIT(2)
#define hts221_ODR_BIT         hts221_BIT(0)

#define hts221_PD_MASK        (uint8_t)0x80
#define hts221_BDU_MASK       (uint8_t)0x04
#define hts221_ODR_MASK       (uint8_t)0x03

/**
* @brief Control register 2.
* \code
* Read/write
* Default value: 0x00
* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-cleared upon completation.
* 6:2 Reserved.
* 1 HEATHER: 0: heater enable; 1: heater disable.
* 0 ONE_SHOT: 0: waiting for start of conversion; 1: start for a new dataset. Self-cleared upon completation.
* \endcode
*/
#define hts221_CTRL_REG2      (uint8_t)0x21

#define hts221_BOOT_BIT        hts221_BIT(7)
#define hts221_HEATHER_BIT     hts221_BIT(1)
#define hts221_ONESHOT_BIT     hts221_BIT(0)

#define hts221_BOOT_MASK      (uint8_t)0x80
#define hts221_HEATHER_MASK   (uint8_t)0x02
#define hts221_ONE_SHOT_MASK  (uint8_t)0x01

/**
* @brief Control register 3.
* \code
* Read/write
* Default value: 0x00
* 7 DRDY_H_L: Interrupt edge. 0: active high, 1: active low.
* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: push-pull; 1: open drain.
* 5:3 Reserved.
* 2 DRDY: interrupt config. 0: disable, 1: enable.
* \endcode
*/
#define hts221_CTRL_REG3      (uint8_t)0x22

#define hts221_DRDY_H_L_BIT    hts221_BIT(7)
#define hts221_PP_OD_BIT       hts221_BIT(6)
#define hts221_DRDY_BIT        hts221_BIT(2)

#define hts221_DRDY_H_L_MASK  (uint8_t)0x80
#define hts221_PP_OD_MASK     (uint8_t)0x40
#define hts221_DRDY_MASK      (uint8_t)0x04

/**
* @brief  Status register.
* \code
* Read
* Default value: 0x00
* 7:2 Reserved.
* 1 H_DA: Humidity data available. 0: new data for humidity is not yet available; 1: new data for humidity is available.
* 0 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
* \endcode
*/
#define hts221_STATUS_REG    (uint8_t)0x27

#define hts221_H_DA_BIT       hts221_BIT(1)
#define hts221_T_DA_BIT       hts221_BIT(0)

#define hts221_HDA_MASK      (uint8_t)0x02
#define hts221_TDA_MASK      (uint8_t)0x01

/**
* @brief  Humidity data (LSB).
* \code
* Read
* Default value: 0x00.
* HOUT7 - HOUT0: Humidity data LSB (2's complement).
* \endcode
*/
#define hts221_HR_OUT_L_REG        (uint8_t)0x28

/**
* @brief  Humidity data (MSB).
* \code
* Read
* Default value: 0x00.
* HOUT15 - HOUT8: Humidity data MSB (2's complement).
* \endcode
*/
#define hts221_HR_OUT_H_REG        (uint8_t)0x29


/**
* @brief  Temperature data (LSB).
* \code
* Read
* Default value: 0x00.
* TOUT7 - TOUT0: temperature data LSB.
* \endcode
*/
#define hts221_TEMP_OUT_L_REG         (uint8_t)0x2A

/**
* @brief  Temperature data (MSB).
* \code
* Read
* Default value: 0x00.
* TOUT15 - TOUT8: temperature data MSB.
* \endcode
*/
#define hts221_TEMP_OUT_H_REG         (uint8_t)0x2B

/**
* @brief  Calibration registers.
* \code
* Read
* \endcode
*/
#define hts221_H0_RH_X2        (uint8_t)0x30
#define hts221_H1_RH_X2        (uint8_t)0x31
#define hts221_T0_DEGC_X8      (uint8_t)0x32
#define hts221_T1_DEGC_X8      (uint8_t)0x33
#define hts221_T0_T1_DEGC_H2   (uint8_t)0x35
#define hts221_H0_T0_OUT_L     (uint8_t)0x36
#define hts221_H0_T0_OUT_H     (uint8_t)0x37
#define hts221_H1_T0_OUT_L     (uint8_t)0x3A
#define hts221_H1_T0_OUT_H     (uint8_t)0x3B
#define hts221_T0_OUT_L        (uint8_t)0x3C
#define hts221_T0_OUT_H        (uint8_t)0x3D
#define hts221_T1_OUT_L        (uint8_t)0x3E
#define hts221_T1_OUT_H        (uint8_t)0x3F

/**
* struct hts221 - hts221 device specific data
* @hwmon_dev: device registered with hwmon
* @lock: mutex to protect measurement values
* @valid: only 0 before first measurement is taken
* @last_update: time of last update (jiffies)
* @temperature:
* @humidity:
*/
struct hts221
{
	struct device *hwmon_dev;
	struct mutex lock;
	char valid;
	int temperature;
	int humidity;
	unsigned char heater;

	/*
	u8 H0_rH_x2;
	u8 H1_rH_x2;
	u16 T0_degC;
	u16 T1_degC;
	s16 H0_T0_out;
	s16 H1_T0_out;
	s16 T0_out;
	s16 T1_out;
	*/

	int16_t H0_T0_out, H1_T0_out, H_T_out;
	int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
	int16_t H0_rh, H1_rh;
	int16_t T0_degC, T1_degC;
};

/**
 * hts221_update_measurements() - get updated measurements from device
 * @dev: device
 *
 * Returns 0 on success, else negative errno.
 */
static int hts221_update_measurements(struct device *dev)
{
	int32_t tmp;

	int ret = 0;
	struct hts221 *hts221 = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);
	
	mutex_lock(&hts221->lock);
	
	

	/*
	ret = i2c_smbus_read_word_swapped(client, hts221_TRIG_T_MEASUREMENT_HM);
	if (ret < 0) goto out;
	hts221->temperature = hts221_temp_ticks_to_millicelsius(ret);
	ret = i2c_smbus_read_word_swapped(client, hts221_TRIG_RH_MEASUREMENT_HM);
	if (ret < 0) goto out;
	*/
	/*hts221->humidity = hts221_rh_ticks_to_per_cent_mille(ret);
	hts221->valid = 1;*/

	hts221->H_T_out = (uint16_t)i2c_smbus_read_byte_data(client, hts221_HR_OUT_L_REG) | (uint16_t)((i2c_smbus_read_byte_data(client, hts221_HR_OUT_H_REG)) << 8);
	hts221->T_out = (uint16_t)i2c_smbus_read_byte_data(client, hts221_TEMP_OUT_L_REG) | (uint16_t)((i2c_smbus_read_byte_data(client, hts221_TEMP_OUT_H_REG)) << 8);

	tmp = ((uint32_t)(hts221->H_T_out - hts221->H0_T0_out)) * ((uint32_t)(hts221->H1_rh - hts221->H0_rh)*10);
	hts221->humidity = tmp/(hts221->H1_T0_out - hts221->H0_T0_out)  + hts221->H0_rh*10;

	tmp = ((int32_t)(hts221->T_out - hts221->T0_out)) * ((int32_t)(hts221->T1_degC - hts221->T0_degC)*10);
	hts221->temperature = tmp/(hts221->T1_out - hts221->T0_out) + hts221->T0_degC*10;

	if(hts221->humidity > 1000) hts221->humidity = 1000;

//out:
	mutex_unlock(&hts221->lock);

	return ret >= 0 ? 0 : ret;
}

/**
* @brief  Set ONE_SHOT bit to start a new conversion (ODR mode has to be 00).
*         Once the measurement is done, ONE_SHOT bit is self-cleared.
* @param  None.
* @retval Error code [HTS221_OK, HTS221_ERROR].
*/
/*
static int HTS221_StartOneShotMeasurement(void)
{
  uint8_t tmp;
  
  if(HTS221_ReadReg(HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;
  
  tmp |= HTS221_ONE_SHOT_MASK;
  
  if(HTS221_WriteReg(HTS221_CTRL_REG2, 1, &tmp))
    return HTS221_ERROR;
  
  return HTS221_OK;
  
}
*/

/**
 * hts221_show_temperature() - show temperature measurement value in sysfs
 * @dev: device
 * @attr: device attribute
 * @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to temp sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t hts221_show_temperature(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hts221 *hts221 = dev_get_drvdata(dev);
	int ret;

	ret = hts221_update_measurements(dev);
	if (ret < 0) return ret;
	return sprintf(buf, "%d\n", hts221->temperature);
}

/**
 * hts221_show_humidity() - show humidity measurement value in sysfs
 * @dev: device
 * @attr: device attribute
 * @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to humidity sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t hts221_show_humidity(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hts221 *hts221 = dev_get_drvdata(dev);
	int ret;

	ret = hts221_update_measurements(dev);
	if (ret < 0) return ret;
	return sprintf(buf, "%d\n", hts221->humidity);
}

/**
* hts221_set_heater() - return value in sysfs
* @dev: device
* @attr: device attribute
* @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
*
* Will be called on write access to heater sysfs attribute.
* Returns number of bytes written into buffer, negative errno on error.
*/
static ssize_t hts221_set_heater(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long val;
	struct hts221 *hts221 = dev_get_drvdata(dev);

	err = kstrtoul(buf, 10, &val);
	if (err) return err;

	hts221->heater = (unsigned char)val;

	return count;
}

/**
* hts221_show_heater() - return value in sysfs
* @dev: device
* @attr: device attribute
* @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
*
* Will be called on read access to heater sysfs attribute.
* Returns number of bytes written into buffer, negative errno on error.
*/
static ssize_t hts221_show_heater(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hts221 *hts221 = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", hts221->heater);
}

/**
* hts221_show_memory() - return value in sysfs
* @dev: device
* @attr: device attribute
* @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
*
* Will be called on read access to heater sysfs attribute.
* Returns number of bytes written into buffer, negative errno on error.
*/
static ssize_t hts221_show_memory(struct device *dev, struct device_attribute *attr, char *buf)
{

	u8 byteArray[256];
	char* start = buf;
	int i, p;
	int err = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct hts221 *hts221 = i2c_get_clientdata(client);

	//memset(&byteArray, 0, 256);

	mutex_lock(&hts221->lock);

	for (i = 0; i <= 0xFF; i++)
	{
		if ((byteArray[i] = i2c_smbus_read_byte_data(client, i)) < 0) err = 1;
	}

	if(!err)
	{
		*buf++ = ' ';
		*buf++ = ' ';
		*buf++ = ' ';
		for (i = 0; i <= 0xF; i++) buf += sprintf(buf, "  %x", i);
		buf += sprintf(buf, "\n");
		for (i = 0; i <= 0xF; i++)
		{
			buf += sprintf(buf, "%02x: ", i);
			for (p = 0; p <= 0xF; p++) buf += sprintf(buf, "%02x ", byteArray[0x10 * i + p]);
			buf += sprintf(buf, "\n");
		}
	}
	mutex_unlock(&hts221->lock);
	return buf - start;
}

static ssize_t hts221_show_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	char* start = buf;
	struct i2c_client *client = to_i2c_client(dev);
	struct hts221 *hts221 = i2c_get_clientdata(client);

	buf += sprintf(buf,"H0_rh=%x\n", hts221->H0_rh);
	buf += sprintf(buf,"H1_rh=%x\n", hts221->H1_rh);
	buf += sprintf(buf,"T0_degC=%x\n", hts221->T0_degC);
	buf += sprintf(buf,"T1_degC=%x\n", hts221->T1_degC);
	buf += sprintf(buf,"H0_T0_out=%d (0x%x)\n", hts221->H0_T0_out, hts221->H0_T0_out);
	buf += sprintf(buf,"H1_T0_out=%d (0x%x)\n", hts221->H1_T0_out, hts221->H1_T0_out);
	buf += sprintf(buf,"T0_out=%d (0x%x)\n", hts221->T0_out, hts221->T0_out);
	buf += sprintf(buf,"T1_out=%d (0x%x)\n", hts221->T1_out, hts221->T1_out);

	return buf - start;
}

/* sysfs attributes */
static SENSOR_DEVICE_ATTR(temp, S_IRUGO, hts221_show_temperature, NULL, 0);
static SENSOR_DEVICE_ATTR(humidity, S_IRUGO, hts221_show_humidity, NULL, 0);
static SENSOR_DEVICE_ATTR(heater, S_IWUSR | S_IRUGO, hts221_show_heater, hts221_set_heater, 0);
static SENSOR_DEVICE_ATTR(memory, S_IRUGO, hts221_show_memory, NULL, 0);
static SENSOR_DEVICE_ATTR(calibration, S_IRUGO, hts221_show_calibration, NULL, 0);

static struct attribute *hts221_attributes[] =
{
	&sensor_dev_attr_temp.dev_attr.attr,
	&sensor_dev_attr_humidity.dev_attr.attr,
	&sensor_dev_attr_heater.dev_attr.attr,
	&sensor_dev_attr_memory.dev_attr.attr,
	&sensor_dev_attr_calibration.dev_attr.attr,
	NULL
};

static const struct attribute_group hts221_attr_group =
{
	.attrs = hts221_attributes,
};

static int hts221_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct hts221 *hts221;
	int err;
	int ret;
	u8 var;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
	{
		dev_err(&client->dev,"adapter does not support SMBus word transactions\n");
		return -ENODEV;
	}

	hts221 = devm_kzalloc(&client->dev, sizeof(*hts221), GFP_KERNEL);
	if (!hts221) return -ENOMEM;

	i2c_set_clientdata(client, hts221);

	mutex_init(&hts221->lock);

	err = sysfs_create_group(&client->dev.kobj, &hts221_attr_group);
	if (err)
	{
		dev_dbg(&client->dev, "could not create sysfs files\n");
		return err;
	}
	hts221->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(hts221->hwmon_dev))
	{
		dev_dbg(&client->dev, "unable to register hwmon device\n");
		err = PTR_ERR(hts221->hwmon_dev);
		goto fail_remove_sysfs;
	}

	mutex_init(&hts221->lock);

	// configure hts221
	mutex_lock(&hts221->lock);

	// check if hts221 connected device is
	ret = i2c_smbus_read_byte_data(client, hts221_WHO_AM_I_REG);
	if (ret != 0xBC)
	{
		dev_dbg(&client->dev, "Wrong WHO_AM_I responce!\n");
		return -ENODEV;
	}

	// read calibration data
	/*
	hts221->H0_rH_x2 = i2c_smbus_read_byte_data(client, hts221_H0_RH_X2);
	hts221->H1_rH_x2 = i2c_smbus_read_byte_data(client, hts221_H1_RH_X2);
	hts221->T0_degC = i2c_smbus_read_byte_data(client, hts221_T0_DEGC_X8) + ((var & 0x3)<<8);
	hts221->T1_degC = i2c_smbus_read_byte_data(client, hts221_T1_DEGC_X8) + ((var & 0xC)<<6);
	hts221->H0_T0_out = i2c_smbus_read_byte_data(client, hts221_H0_T0_OUT_L) + (i2c_smbus_read_byte_data(client, hts221_H0_T0_OUT_H) << 8);
	hts221->H1_T0_out = i2c_smbus_read_byte_data(client, hts221_H1_T0_OUT_L) + (i2c_smbus_read_byte_data(client, hts221_H1_T0_OUT_H) << 8);
	hts221->T0_out = i2c_smbus_read_byte_data(client, hts221_T0_OUT_L) + (i2c_smbus_read_byte_data(client, hts221_T0_OUT_H) << 8);
	hts221->T1_out = i2c_smbus_read_byte_data(client, hts221_T1_OUT_L) + (i2c_smbus_read_byte_data(client, hts221_T1_OUT_H) << 8);
	*/

  	hts221->H0_T0_out = (uint16_t)i2c_smbus_read_byte_data(client, hts221_H0_T0_OUT_L) | (uint16_t)(i2c_smbus_read_byte_data(client, hts221_H0_T0_OUT_H) << 8);
	hts221->H1_T0_out = (uint16_t)i2c_smbus_read_byte_data(client, hts221_H1_T0_OUT_L) | (uint16_t)(i2c_smbus_read_byte_data(client, hts221_H1_T0_OUT_H) << 8);
	hts221->T0_out = (uint16_t)i2c_smbus_read_byte_data(client, hts221_T0_OUT_L) | (uint16_t)((i2c_smbus_read_byte_data(client, hts221_T0_OUT_H)) << 8);
	hts221->T1_out = (uint16_t)i2c_smbus_read_byte_data(client, hts221_T1_OUT_L) | (uint16_t)((i2c_smbus_read_byte_data(client, hts221_T1_OUT_H)) << 8);
	var = i2c_smbus_read_byte_data(client, hts221_T0_T1_DEGC_H2);
	hts221->T0_degC = (((uint16_t)i2c_smbus_read_byte_data(client, hts221_T0_DEGC_X8) | (((uint16_t)(var & 0x3)) << 8))) >> 3;
	hts221->T1_degC = (((uint16_t)i2c_smbus_read_byte_data(client, hts221_T1_DEGC_X8) | (((uint16_t)(var & 0xC)) << 6))) >> 3;
	
	hts221->H0_rh = (uint16_t)i2c_smbus_read_byte_data(client, hts221_H0_RH_X2);
	hts221->H1_rh = (uint16_t)i2c_smbus_read_byte_data(client, hts221_H1_RH_X2);
	
	// set sample rate
	//ret = i2c_smbus_write_byte_data(client, hts221_CTRL_REG1, hts221_PD_MASK | hts221_ODR_12_5HZ);	// 12.5 Hz
	ret = i2c_smbus_write_byte_data(client, hts221_CTRL_REG1, hts221_PD_MASK | hts221_ODR_1HZ);	// 1 Hz
	if (ret < 0) return -ENODEV;

	// set resolution
	// Internal average on 512 samples Humidity, Internal average on 256 samples Temperature
	ret = i2c_smbus_write_byte_data(client, hts221_AV_CONF_REG, hts221_AVGT_256 | hts221_AVGH_512);
	if (ret < 0) return -ENODEV;

	mutex_unlock(&hts221->lock);

	dev_info(&client->dev, "initialized\n");

	return 0;

fail_remove_sysfs:
	//sysfs_remove_group(&client->dev.kobj, &hts221_attr_group);
	return err;
}

/**
 * hts221_remove() - remove device
 * @client: I2C client device
 */
static int hts221_remove(struct i2c_client *client)
{
	struct hts221 *hts221 = i2c_get_clientdata(client);

	hwmon_device_unregister(hts221->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &hts221_attr_group);

	return 0;
}

/* Device ID table */
static const struct i2c_device_id hts221_id[] =
{
	{ "hts221", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hts221_id);

static struct i2c_driver hts221_driver =
{
	.driver.name = "hts221",
	.probe       = hts221_probe,
	.remove	     = hts221_remove,
	.id_table    = hts221_id,
};

module_i2c_driver(hts221_driver);

MODULE_AUTHOR("Ing. Jan Fíla, ml <jfila@jfila.cz>");
MODULE_DESCRIPTION("HTS221 driver");
MODULE_LICENSE("GPL");
