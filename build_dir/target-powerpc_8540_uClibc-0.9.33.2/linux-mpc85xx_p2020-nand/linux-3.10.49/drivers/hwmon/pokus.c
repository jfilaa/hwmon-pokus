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

/**
* struct pokus - pokus device specific data
* @hwmon_dev: device registered with hwmon
* @lock: mutex to protect measurement values
* @valid: only 0 before first measurement is taken
* @last_update: time of last update (jiffies)
* @number1:
* @number2:
*/
struct pokus
{
        struct device *hwmon_dev;
        struct mutex lock;
        char valid;
        unsigned long last_update;
        int number1;
        int number2;
};

/**
* pokus_show_number1() - return value in sysfs
* @dev: device
* @attr: device attribute
* @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
*
* Will be called on read access to number1 sysfs attribute.
* Returns number of bytes written into buffer, negative errno on error.
*/
static ssize_t pokus_show_number1(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct pokus *pokus = dev_get_drvdata(dev);
        /*int ret;

        ret = pokus_update_measurements(dev);
        if (ret < 0) return ret;*/
        return sprintf(buf, "%d\n", pokus->number1++);
}

/**
* pokus_show_number2() - return value in sysfs
* @dev: device
* @attr: device attribute
* @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
*
* Will be called on read access to number1 sysfs attribute.
* Returns number of bytes written into buffer, negative errno on error.
*/
static ssize_t pokus_show_number2(struct device *dev, struct device_attribute *attr, char *buf)
{
        struct pokus *pokus = dev_get_drvdata(dev);
        /*int ret;

        ret = pokus_update_measurements(dev);
        if (ret < 0) return ret;*/
        return sprintf(buf, "%d\n", pokus->number2++);
}

/* sysfs attributes */
static SENSOR_DEVICE_ATTR(number1, S_IRUGO, pokus_show_number1,
        NULL, 0);
static SENSOR_DEVICE_ATTR(number2, S_IRUGO, pokus_show_number2,
        NULL, 0);

static struct attribute *pokus_attributes[] =
{
        &sensor_dev_attr_number1.dev_attr.attr,
        &sensor_dev_attr_number2.dev_attr.attr,
        NULL
};

static const struct attribute_group pokus_attr_group = {
        .attrs = pokus_attributes,
};

static int pokus_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
        struct pokus *pokus;
	int err;

        if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
        {
                dev_err(&client->dev, "adapter does not support SMBus word transactions\n");
                return -ENODEV;
        }

        pokus = devm_kzalloc(&client->dev, sizeof(*pokus), GFP_KERNEL);
        if (!pokus) return -ENOMEM;

        i2c_set_clientdata(client, pokus);

        mutex_init(&pokus->lock);

	err = sysfs_create_group(&client->dev.kobj, &pokus_attr_group);
        if (err) {
                dev_dbg(&client->dev, "could not create sysfs files\n");
                return err;
        }
        pokus->hwmon_dev = hwmon_device_register(&client->dev);
        if (IS_ERR(pokus->hwmon_dev)) {
                dev_dbg(&client->dev, "unable to register hwmon device\n");
                err = PTR_ERR(pokus->hwmon_dev);
                goto fail_remove_sysfs;
        }

        dev_info(&client->dev, "initialized\n");

        return 0;

fail_remove_sysfs:
        sysfs_remove_group(&client->dev.kobj, &pokus_attr_group);
        return err;

}

/**
 * pokus_remove() - remove device
 * @client: I2C client device
 */
static int pokus_remove(struct i2c_client *client)
{
        struct pokus *pokus = i2c_get_clientdata(client);

        hwmon_device_unregister(pokus->hwmon_dev);
        sysfs_remove_group(&client->dev.kobj, &pokus_attr_group);

        return 0;
}

/* Device ID table */
static const struct i2c_device_id pokus_id[] =
{
        { "pokus", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, pokus_id);

static struct i2c_driver pokus_driver =
{
        .driver.name = "pokus",
        .probe       = pokus_probe,
	.remove	     = pokus_remove,
        .id_table    = pokus_id,
};

module_i2c_driver(pokus_driver);

MODULE_AUTHOR("Ing. Jan Fíla, ml <jfila@jfila.cz>");
MODULE_DESCRIPTION("Pokus driver");
MODULE_LICENSE("GPL");
