/***************************************************************
本驱动由正点原子的ft5426驱动修改而来
 ***************************************************************/
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

/* CST128寄存器相关宏定义 */
#define CST128_DEVIDE_MODE_REG	0x00	// 模式寄存器
#define CST128_TD_STATUS_REG    	0x02  	// 状态寄存器
#define CST128_TOUCH_DATA_REG   	0x03   	// 触摸数据读取的起始寄存器
#define CST128_ID_G_MODE_REG    	0xA4  	// 中断模式寄存器

#define MAX_SUPPORT_POINTS		5    	// cst128最大支持5点触摸

#define TOUCH_EVENT_DOWN      	0x00    // 按下
#define TOUCH_EVENT_UP           	0x01    // 抬起
#define TOUCH_EVENT_ON         	0x02    // 接触
#define TOUCH_EVENT_RESERVED  	0x03    // 保留


struct hyn_cst128_dev {
    struct i2c_client *client;
    struct input_dev *input;
    int reset_gpio;
    int irq_gpio;
};

static int hyn_cst128_ts_write(struct hyn_cst128_dev *cst128,
            u8 addr, u8 *buf, u16 len)
{
    struct i2c_client *client = cst128->client;
    struct i2c_msg msg;
    u8 send_buf[6] = {0};
    int ret;

    send_buf[0] = addr;
    memcpy(&send_buf[1], buf, len);

    msg.flags = 0;                  //i2c写
    msg.addr = client->addr;
    msg.buf = send_buf;
    msg.len = len + 1;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (1 == ret)
        return 0;
    else {
        dev_err(&client->dev, "%s: write error, addr=0x%x len=%d.\n",
                    __func__, addr, len);
        return -1;
    }
}

static int hyn_cst128_ts_read(struct hyn_cst128_dev *cst128,
            u8 addr, u8 *buf, u16 len)
{
    struct i2c_client *client = cst128->client;
    struct i2c_msg msg[2];
    int ret;

    msg[0].flags = 0;             	// i2c写
    msg[0].addr = client->addr;
    msg[0].buf = &addr;
    msg[0].len = 1;              	// 1个字节

    msg[1].flags = I2C_M_RD;    	//i2c读
    msg[1].addr = client->addr;
    msg[1].buf = buf;
    msg[1].len = len;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (2 == ret)
        return 0;
    else {
        dev_err(&client->dev, "%s: read error, addr=0x%x len=%d.\n",
                    __func__, addr, len);
        return -1;
    }
}

static int hyn_cst128_ts_reset(struct hyn_cst128_dev *cst128)
{
    struct i2c_client *client = cst128->client;
    int ret;

    /* 从设备树中获取复位管脚 */
    cst128->reset_gpio = of_get_named_gpio(client->dev.of_node, "reset-gpios", 0);
    if (!gpio_is_valid(cst128->reset_gpio)) {
        dev_err(&client->dev, "Failed to get ts reset gpio\n");
        return cst128->reset_gpio;
    }

    /* 申请使用管脚 */
    ret = devm_gpio_request_one(&client->dev, cst128->reset_gpio,
                GPIOF_OUT_INIT_HIGH, "cst128 reset");
    if (ret < 0)
        return ret;

    msleep(20);
    gpio_set_value_cansleep(cst128->reset_gpio, 0);    	// 拉低复位引脚
    msleep(5);
    gpio_set_value_cansleep(cst128->reset_gpio, 1);   	// 拉高复位引脚，结束复位

    return 0;
}

static irqreturn_t hyn_cst128_ts_isr(int irq, void *dev_id)
{
    struct hyn_cst128_dev *cst128 = dev_id;
    u8 rdbuf[30] = {0};
    int i, type, x, y, id;
    bool down;
    int ret;

    /* 读取CST128触摸点坐标从0x02寄存器开始，连续读取29个寄存器 */
    ret = hyn_cst128_ts_read(cst128, CST128_TD_STATUS_REG, rdbuf, 29);
    if (ret)
        goto out;

    for (i = 0; i < MAX_SUPPORT_POINTS; i++) {

        u8 *buf = &rdbuf[i * 6 + 1];

        /* 以第一个触摸点为例，寄存器TOUCH1_XH(地址0x03)，各bit位描述如下：
         * bit7:6  Event flag  0:按下 1:释放 2:接触 3:没有事件
         * bit5:4  保留
         * bit3:0  X轴触摸点的11~8位
         */
        type = buf[0] >> 6;                     // 获取触摸点的Event Flag
        if (type == TOUCH_EVENT_RESERVED)
            continue;

        y = ((buf[2] << 8) | buf[3]) & 0x0fff;
        x = (((buf[0] << 8) | buf[1]) & 0x0fff);
        x = 480 - x;

        /* 以第一个触摸点为例，寄存器TOUCH1_YH(地址0x05)，各bit位描述如下：
         * bit7:4  Touch ID  触摸ID，表示是哪个触摸点
         * bit3:0  Y轴触摸点的11~8位。
         */
        id = (buf[2] >> 4) & 0x0f;
        down = type != TOUCH_EVENT_UP;

        input_mt_slot(cst128->input, id);
        input_mt_report_slot_state(cst128->input, MT_TOOL_FINGER, down);

        if (!down)
            continue;

        input_report_abs(cst128->input, ABS_MT_POSITION_X, x);
        input_report_abs(cst128->input, ABS_MT_POSITION_Y, y);
    }

    input_mt_report_pointer_emulation(cst128->input, true);
    input_sync(cst128->input);

out:
    return IRQ_HANDLED;
}

static int hyn_cst128_ts_irq(struct hyn_cst128_dev *cst128)
{
    struct i2c_client *client = cst128->client;
    int ret;

    /* 从设备树中获取中断管脚 */
    cst128->irq_gpio = of_get_named_gpio(client->dev.of_node, "irq-gpios", 0);
    if (!gpio_is_valid(cst128->irq_gpio)) {
        dev_err(&client->dev, "Failed to get ts interrupt gpio\n");
        return cst128->irq_gpio;
    }

    /* 申请使用管脚 */
    ret = devm_gpio_request_one(&client->dev, cst128->irq_gpio,
                GPIOF_IN, "cst128 interrupt");
    if (ret < 0)
        return ret;

    /* 注册中断服务函数 */
    ret = devm_request_threaded_irq(&client->dev, gpio_to_irq(cst128->irq_gpio),
                NULL, hyn_cst128_ts_isr, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                client->name, cst128);
    if (ret) {
        dev_err(&client->dev, "Failed to request touchscreen IRQ.\n");
        return ret;
    }

    return 0;
}

static int hyn_cst128_ts_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct hyn_cst128_dev *cst128;
    struct input_dev *input;
    u8 data;
    int ret;

    /* 实例化一个struct hyn_cst128_dev对象 */
    cst128 = devm_kzalloc(&client->dev, sizeof(struct hyn_cst128_dev), GFP_KERNEL);
    if (!cst128) {
        dev_err(&client->dev, "Failed to allocate cst128 driver data.\n");
        return -ENOMEM;
    }

    cst128->client = client;

    /* 复位CST128触摸芯片 */
    ret = hyn_cst128_ts_reset(cst128);
    if (ret)
        return ret;

    msleep(5);

    /* 初始化CST128 */
    data = 0;
    hyn_cst128_ts_write(cst128, CST128_DEVIDE_MODE_REG, &data, 1);
    data = 1;
    hyn_cst128_ts_write(cst128, CST128_ID_G_MODE_REG, &data, 1);

    /* 申请、注册中断服务函数 */
    ret = hyn_cst128_ts_irq(cst128);
    if (ret)
        return ret;

    /* 注册input设备 */
    input = devm_input_allocate_device(&client->dev);
    if (!input) {
        dev_err(&client->dev, "Failed to allocate input device.\n");
        return -ENOMEM;
    }

    cst128->input = input;
    input->name = "CST128 TouchScreen";
    input->id.bustype = BUS_I2C;

    input_set_abs_params(input, ABS_MT_POSITION_X,
                0, 480, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y,
                0, 800, 0, 0);

    ret = input_mt_init_slots(input, MAX_SUPPORT_POINTS, INPUT_MT_DIRECT);
    if (ret) {
        dev_err(&client->dev, "Failed to init MT slots.\n");
        return ret;
    }

    ret = input_register_device(input);
    if (ret)
        return ret;

    i2c_set_clientdata(client, cst128);
    return 0;
}

static int hyn_cst128_ts_remove(struct i2c_client *client)
{
    struct hyn_cst128_dev *cst128 = i2c_get_clientdata(client);
    input_unregister_device(cst128->input);
    return 0;
}

static const struct of_device_id hyn_cst128_of_match[] = {
    { .compatible = "hyn,cst128a", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hyn_cst128_of_match);

static struct i2c_driver hyn_cst128_ts_driver = {
    .driver = {
        .owner     		= THIS_MODULE,
        .name          	= "hyn_cst128a",
        .of_match_table	= of_match_ptr(hyn_cst128_of_match),
    },
    .probe    = hyn_cst128_ts_probe,
    .remove   = hyn_cst128_ts_remove,
};

module_i2c_driver(hyn_cst128_ts_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leefei");
MODULE_INFO(intree, "Y");

