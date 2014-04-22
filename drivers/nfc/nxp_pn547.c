#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
//#include <linux/pn544.h>
#include <linux/pn547.h>
#include <linux/clk.h>
#include <linux/fs.h>

#include <linux/notifier.h>
#include <linux/fb.h>

#define PN547_DRIVER_NAME	"pn547"
//#define ZTEMT_NFC_CONFIG_FB
#define NXP_PN547_DEBUG

#define DRIVER_DESC	"NFC driver for PN547"

#define MAX_BUFFER_SIZE		512
#define PN547_MSG_MAX_SIZE	0x21 /* at normal HCI mode */

/* Timing restrictions (ms) */
#define PN547_RESETVEN_TIME	35 /* 7 */

//struct PN547_nfc_platform_data PN547_nfc_platform_data;

enum pn547_irq {
	PN547_NONE,
	PN547_INT,
};

struct pn547_dev	{
	wait_queue_head_t	read_wq;
	struct mutex read_mutex;
	struct i2c_client	*client;
	struct miscdevice	miscdev;
	struct work_struct clk_work;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	enum pn547_irq read_irq;
	
	int updata_gpio;
	int ven_gpio;
	int irq_gpio;
	int (*request_resources) (void);
	void (*free_resources) (void);
	void (*enable) (int fw);
	int (*test) (void);
	void (*disable) (void);
	int (*irq_status) (void);
#ifdef ZTEMT_NFC_CONFIG_FB
    struct notifier_block fb_notif;
#endif
};

static struct pn547_dev *pn547_dev;
//the clk is defined for nfc pll-clk in 19.2M
struct clk* nfc_rf_clk;

#define PN547_NFC_SW_UPDATE   (pn547_dev->updata_gpio) //#define PN547_NFC_SW_UPDATE 13
#define PN547_NFC_nVEN        (pn547_dev->ven_gpio) //#define PN547_NFC_nVEN      85
#define PN547_NFC_GPIO_INT    (pn547_dev->irq_gpio)//#define PN547_NFC_GPIO_INT  68


static int pn547_nfc_request_resources(void)
{
	printk("[%s] pn547_nfc_request_resources start.\n", __func__);
	
	if(gpio_request(PN547_NFC_SW_UPDATE, "NFC_SW_UPDATE")) {
		pr_err("failed request NFC_SW_UPDATE.\n");
		return -1;
	}
	gpio_tlmm_config(GPIO_CFG(PN547_NFC_SW_UPDATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	gpio_set_value(PN547_NFC_SW_UPDATE,0);
	
	if(gpio_request(PN547_NFC_nVEN, "NFC_VEN")) {
		pr_err("failed request NFC_VEN.\n");
		return -1;
	}
	gpio_direction_output(PN547_NFC_nVEN, 1);
	gpio_set_value_cansleep(PN547_NFC_nVEN,1);
	
	if(gpio_request(PN547_NFC_GPIO_INT, "NFC_IRQ")) {
		pr_err("failed request NFC_IRQ.\n");
		goto err_rst_gpio_req;
	}	
	gpio_tlmm_config(GPIO_CFG(PN547_NFC_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);


	return 0;
err_rst_gpio_req:
	gpio_free(PN547_NFC_SW_UPDATE);
	gpio_free(PN547_NFC_nVEN);
	return -1;
}
static void pn547_nfc_free_resources(void)
{
	/* Release the HW resources */
	printk("%s:goto pn547_nfc_free_resources\n",__func__);
	//pmapp_clock_vote("NNFC", PMAPP_CLOCK_ID_A1, PMAPP_CLOCK_VOTE_OFF);
	gpio_tlmm_config(GPIO_CFG(PN547_NFC_SW_UPDATE, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(PN547_NFC_GPIO_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	gpio_free(PN547_NFC_SW_UPDATE);
	gpio_free(PN547_NFC_nVEN);
	gpio_free(PN547_NFC_GPIO_INT);
}
static void pn547_nfc_enable(int fw)
{
	printk("%s:goto pn547_nfc_enable\n",__func__);
	/* Turn the device on */
    if (HCI_MODE == fw )
    {
		printk("%s:enable HCI_MODE pn547\n",__func__);
        gpio_set_value(PN547_NFC_SW_UPDATE, 0);
        gpio_set_value_cansleep(PN547_NFC_nVEN,0);
    }
    else // FW_MODE
    {
        
		printk("%s:enable FW_MODE pn547\n",__func__);
		gpio_set_value_cansleep(PN547_NFC_nVEN, 0);/* 1 */
        gpio_set_value(PN547_NFC_SW_UPDATE, 1); /* 1 */
        msleep(10);
        gpio_set_value_cansleep(PN547_NFC_nVEN, 1);/* 0 */
        msleep(50);
        gpio_set_value_cansleep(PN547_NFC_nVEN, 0);/* 1 */
        msleep(10);
    }
}
static int pn547_nfc_test(void)
{
	/*
	 * Put the device into the FW update mode
	 * and then back to the normal mode.
	 * Check the behavior and return one on success,
	 * zero on failure.
	 */
	return 0;
}

static void pn547_nfc_disable(void)
{
	/* turn the power off */
 	gpio_set_value(PN547_NFC_SW_UPDATE, 0);
	gpio_set_value_cansleep(PN547_NFC_nVEN, 1);
}

static int pn547_pt_irq_status(void)
{
  return (gpio_get_value(PN547_NFC_GPIO_INT) != 0);
}


static int pn547_dev_enable(struct pn547_dev *dev, int mode)
{
#ifdef NXP_PN547_DEBUG
    printk("%s: mode: %d\n", __func__, mode);
#endif

	dev->read_irq = PN547_NONE;
	if (dev->enable)
		dev->enable(mode);
	usleep_range(10000, 15000);
	//usleep_range(20000, 35000);
	return 0;
}

static void pn547_dev_disable(struct pn547_dev *dev)
{
	//struct i2c_client *client = dev->client;
	
	if (dev->disable)
		dev->disable();

	msleep(PN547_RESETVEN_TIME);

	dev->read_irq = PN547_NONE;

#ifdef NXP_PN547_DEBUG
	//dev_dbg(&client->dev, "%s: Now in OFF-mode\n", __func__);//delete by chengdongsheng 
    printk(KERN_DEBUG "%s: Now in OFF-mode\n", __func__);
#endif
}

static int pn547_irq_status(struct pn547_dev *dev)
{
	if ( dev->irq_status ) {
		return 0;
	} else {
		return dev->irq_status();
	}
}

static void pn547_disable_irq(struct pn547_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->irq_enabled_lock, flags);
	if (dev->irq_enabled) {
		disable_irq_nosync(dev->client->irq);
		dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&dev->irq_enabled_lock, flags);
}


static int pn547_dev_open(struct inode *inode, struct file *filp)
{
	filp->private_data = pn547_dev;
	
	printk("%s : (imajor) %d, (iminor) %d\n", __func__, imajor(inode), iminor(inode));
	
	return pn547_dev_enable(pn547_dev, HCI_MODE);
}

static ssize_t pn547_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	//ztemt changed  by chengdongsheng 2012.12.20 avoid the filp->private_data was null
	//struct pn547_dev *dev = filp->private_data;	
	struct pn547_dev *dev = pn547_dev;
	//ztemt end
	struct i2c_client *client = dev->client;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

#ifdef NXP_PN547_DEBUG
    printk(KERN_DEBUG "%s: reading %zu bytes.\n", __func__, count);
#endif
	mutex_lock(&dev->read_mutex);

	if ( !pn547_irq_status(dev) ) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		dev->irq_enabled = true;
		enable_irq(dev->client->irq);
		ret = wait_event_interruptible(dev->read_wq, 
			(dev->read_irq == PN547_INT));
		pn547_disable_irq(dev);

		if (ret)
			goto fail;
	}
	/* Read data */
	ret = i2c_master_recv(dev->client, tmp, count);
	dev->read_irq = PN547_NONE;
	mutex_unlock(&dev->read_mutex);

	if (ret < 0) {
		dev_err(&client->dev, "%s: i2c_master_recv returned %d\n", 
				__func__, ret);
		return ret;
	}
	if (ret > count) {
		dev_err(&client->dev, "%s: received too many bytes from i2c (%d)\n",
				__func__, ret);
		return -EIO;
	}
#ifdef NXP_PN547_DEBUG
	print_hex_dump(KERN_DEBUG, " read: ", DUMP_PREFIX_NONE, 16, 1, tmp, ret, false);
#endif
	if (copy_to_user(buf, tmp, ret)) {
		dev_err(&client->dev, "%s : failed to copy to user space\n", 
				__func__);
		return -EFAULT;
	}
	return ret;

fail:
	mutex_unlock(&dev->read_mutex);
	return ret;
}

static ssize_t pn547_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	//ztem changed  by chengdongsheng 2012.12.20 avoid the filp->private_data was null
	//struct pn547_dev *dev = filp->private_data;
	struct pn547_dev *dev = pn547_dev;
	//ztemt end
	struct i2c_client *client = dev->client;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		dev_err(&client->dev, "%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

#ifdef NXP_PN547_DEBUG
	dev_dbg(&client->dev, "%s : writing %zu bytes.\n", __func__, count);
	print_hex_dump(KERN_DEBUG, "write: ", DUMP_PREFIX_NONE, 16, 1, tmp, count, false);
#endif
	/* Write data */
	ret = i2c_master_send(client, tmp, count);
	if (ret != count) {
		dev_err(&client->dev, "%s : addr is 0x%x, i2c_master_send returned %d\n", __func__, client->addr, ret);
		ret = -EIO;
	}

	return ret;
}

static long pn547_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{	
	//ztemt changed  by chengdongsheng 2012.12.20 avoid the filp->private_data was null
	//struct pn547_dev *dev = filp->private_data;
	struct pn547_dev *dev = pn547_dev;	
	//ztemt end
	struct i2c_client *client = dev->client;
	unsigned int val;
	int r = 0;

#ifdef NXP_PN547_DEBUG
	dev_dbg(&client->dev, "%s: cmd: 0x%x,PN547_SET_PWR:0x%x\n", __func__, cmd, PN547_SET_PWR);
#endif
	switch (cmd) {
		case PN547_SET_PWR:
			val = arg;
#ifdef NXP_PN547_DEBUG
			dev_dbg(&client->dev, "%s:  PN547_SET_PWR: %d\n", __func__, val);
#endif
			switch (val) {
				case 0: // power off
					pn547_dev_disable(dev);
					break;
				case 1: // power on
					r = pn547_dev_enable(dev, HCI_MODE);
					if (r < 0)
						goto out;
					break;
				case 2: // reset and power on with firmware download enabled
					r = pn547_dev_enable(dev, FW_MODE);
					if (r < 0)
						goto out;
					break;
				default:
					r = -ENOIOCTLCMD;
					goto out;
					break;
			}
			break;
		default:
			dev_err(&client->dev, "%s bad ioctl %u\n", __func__, cmd);
			return -EINVAL;
	}
out:
	return r;
}

static irqreturn_t pn547_dev_irq_handler(int irq, void *dev_id)
{
	struct pn547_dev *dev = dev_id;

	pn547_disable_irq(dev);

	//dev_dbg(&dev->client->dev, "IRQ\n");

	dev->read_irq = PN547_INT;
	
	/* Wake up waiting readers */
	wake_up(&dev->read_wq);

	return IRQ_HANDLED;
}

static const struct file_operations pn547_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.open	= pn547_dev_open,
	.read	= pn547_dev_read,
	.write	= pn547_dev_write,
	.unlocked_ioctl = pn547_dev_ioctl,
};

#ifdef NXP_PN547_STANDBY_MODE
static void set_standby_mode(struct pn547_dev *pn547dev)
{
	char standby_commands[] = {0x09, 0x9B, 0x82, 0x3F, 0x00, 0x9E, 0xAA, 0x01, 0x9B, 0x03};
	int ret = -1;

	pn547dev = pn547_dev;
	if(pn547dev == NULL){
		printk(KERN_ERR "[%s]dev is NULL.\n", __func__);
		return;
	}
	
	//Reset and write standy mode commands
	ret = pn547_dev_enable(pn547dev, HCI_MODE);
	if (ret < 0){
		printk(KERN_ERR "[%s]enable nfc failed.\n", __func__);
		return;
	}
	
	pn547_dev_disable(pn547dev);

	ret = pn547_dev_enable(pn547dev, HCI_MODE);
	if (ret < 0){
		printk(KERN_ERR "[%s]enable nfc failed.\n", __func__);
		return;
	}

	ret = i2c_master_send(pn547dev->client , standby_commands, sizeof(standby_commands));
	if (ret != sizeof(standby_commands)) {
		printk(KERN_ERR "[%s]i2c_master_send failed. returned %d\n", __func__, ret);
		return;
	}
	pn547_dev_disable(pn547dev);
	
    printk(KERN_DEBUG "[%s]Reset and write standby commands successfully.write bytes:%d\n", __func__, ret);
}
#endif
#ifdef ZTEMT_NFC_CONFIG_FB //NFC_PM_CONFIG
static int nxp_pn547_suspend(struct device *dev)
{
	
#ifdef NXP_PN547_DEBUG
	printk("[dsc]%s:goto suspend\n",__FUNCTION__);
	printk("[dsc] PN547_REMOVE clk 19.2M\n");
#endif
	if (nfc_rf_clk != NULL) {
		//clk_disable_unprepare(nfc_rf_clk);//clk_disable_unprepare
		clk_disable(nfc_rf_clk);//clk_disable_unprepare
		pr_err("have disable NFC_CLK.\n");
	} else {
			pr_err("%s:nfc_rf_clk is null.we can't disable it \n",__FUNCTION__);
	}
	
	return 0 ;
	
}

static int nxp_pn547_resume(struct device *dev)
{
#ifdef NXP_PN547_DEBUG
	printk("[dsc]%s:goto resume\n",__FUNCTION__);
	printk("[dsc] enable clk 19.2M\n");
#endif
	if (nfc_rf_clk != NULL) {
		if (clk_enable(nfc_rf_clk))
			pr_err("failed enable NFC_CLK.\n");
	} else {
			pr_err("%s:nfc_rf_clk is null.we can't enable it \n",__FUNCTION__);
	}
	return 0;
}

static int PN547_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
    int rc;
    
    struct device *dev = &pn547_dev->client->dev;

	if (evdata && evdata->data && event == FB_EVENT_BLANK && pn547_dev && dev) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			rc = nxp_pn547_resume(dev);
        	if (rc < 0) {
        		dev_err(dev, "%s: Error on wake\n", __func__);
        	}
            dev_info(dev, "[dsc] %s: Wake!\n", __func__);
		}
		else if (*blank == FB_BLANK_POWERDOWN) {
			rc = nxp_pn547_suspend(dev);
        	if (rc < 0) {
        		dev_err(dev, "%s: Error on sleep\n", __func__);
        	}
            dev_info(dev, "[dsc] %s: Sleep!\n", __func__);
		}
	}

	return 0;
}
#endif

static __devinit int pn547_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int irq_gpio = -1;
	int updata_gpio = -1;
	int ven_gpio = -1;
	int irq;
	int addr;
	int ret;

	dev_dbg(&client->dev, "IRQ: %d\n", client->irq);

	if(client->dev.of_node)
	{
		irq_gpio = of_get_named_gpio_flags(client->dev.of_node, "nxp,irq-gpio", 0, NULL);
		updata_gpio = of_get_named_gpio_flags(client->dev.of_node, "nxp,updata-gpio", 0, NULL);
		ven_gpio = of_get_named_gpio_flags(client->dev.of_node, "nxp,ven-gpio", 0, NULL);
		printk("pn547,--irq_gpio---:%d\n", irq_gpio);
		printk("pn547,--updata_gpio---:%d\n", updata_gpio);
		printk("pn547,--ven_gpio---:%d\n", ven_gpio);
	}

	irq = client->irq;
	addr = client->addr;

	if (pn547_dev != NULL) {
		dev_warn(&client->dev, "only one PN547 supported.\n");
		return -EBUSY;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}
	
	pn547_dev = kzalloc(sizeof(struct pn547_dev), GFP_KERNEL);
	if (pn547_dev == NULL) {
		dev_err(&client->dev, "failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}
	
	pn547_dev->client = client;
	pn547_dev->irq_gpio = irq_gpio;
	pn547_dev->updata_gpio = updata_gpio;
	pn547_dev->ven_gpio = ven_gpio;
	pn547_dev->request_resources = pn547_nfc_request_resources;
	pn547_dev->free_resources = pn547_nfc_free_resources;
	pn547_dev->enable = pn547_nfc_enable;
	pn547_dev->disable = pn547_nfc_disable;
	pn547_dev->test = pn547_nfc_test;
	pn547_dev->irq_status = pn547_pt_irq_status;
	/* init mutex and queues */
	pn547_dev->read_irq = PN547_NONE;
	init_waitqueue_head(&pn547_dev->read_wq);
	mutex_init(&pn547_dev->read_mutex);
	spin_lock_init(&pn547_dev->irq_enabled_lock);
	
	i2c_set_clientdata(client, pn547_dev);

	if (!pn547_dev->request_resources) {
		dev_err(&client->dev, "request_resources() missing\n");
		ret = -EINVAL;
		goto err_request_resources;
	}

	ret = pn547_dev->request_resources();
	if (ret < 0) {
		dev_err(&client->dev, "Cannot get platform resources\n");
		goto err_request_resources;
	}

	//enable clk 19.2M 
	printk("[dsc] enable clk 19.2M\n");
	nfc_rf_clk = clk_get(&client->dev, "nfc_rf_clk");
	if (nfc_rf_clk != NULL) {
		if (clk_prepare_enable(nfc_rf_clk))
			pr_err("failed request NFC_CLK.\n");
	} else {
			pr_err("%s:nfc_rf_clk is null\n",__FUNCTION__);
	}
	
	pn547_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn547_dev_irq_handler,
			IRQF_TRIGGER_HIGH, client->name, pn547_dev);
	if(ret){
		pr_err("faile to get the irq for nfc-c.\n");
	}
	/*device_init_wakeup(&client->dev,1);
	ret=irq_set_irq_wake(client->irq,1);
	if(ret){
		pr_err("faile to enable the irq for nfc-c to wake.\n");
	}*/
	
	printk("--client->irq:%d,client->name:%s\n", client->irq,client->name);
	
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq;
	}
	
	printk("pn547_dev->client->irq:%d\n",pn547_dev->client->irq);
	pn547_disable_irq(pn547_dev);	

	pn547_dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	pn547_dev->miscdev.name = PN547_DRIVER_NAME;
	pn547_dev->miscdev.fops = &pn547_dev_fops;
	pn547_dev->miscdev.parent = &client->dev;
	ret = misc_register(&pn547_dev->miscdev);
	if (ret){
		dev_err(&client->dev, "%s : misc_register failed\n", __func__);
		goto err_misc_register;
	}
#ifdef ZTEMT_NFC_CONFIG_FB
    pn547_dev->fb_notif.notifier_call = PN547_fb_notifier_callback;

    ret = fb_register_client(&pn547_dev->fb_notif);
    if (ret)
        pr_err("%s:Unable to register fb_notifier: %d\n", __func__, ret);
#endif
	printk("%s: dev: %p, client %p:dev_name%s\n",__func__, pn547_dev, client,dev_name(&client->dev));
   
#ifdef NXP_PN547_STANDBY_MODE
	set_standby_mode(pn547_dev);
#endif

	return 0;

err_misc_register:
	free_irq(client->irq, pn547_dev);
err_request_irq:
	if (pn547_dev->free_resources)
		pn547_dev->free_resources();
err_request_resources:
	mutex_destroy(&pn547_dev->read_mutex);
	kfree(pn547_dev);
err_exit:
	return ret;
}

static __devexit int pn547_remove(struct i2c_client *client)
{
	struct pn547_dev *dev = i2c_get_clientdata(client);
	//int ret;

	dev_dbg(&client->dev, "%s\n", __func__);
	printk("[dsc] PN547_REMOVE clk 19.2M\n");
	if (nfc_rf_clk != NULL) {
		clk_disable_unprepare(nfc_rf_clk);
		pr_err("have disable NFC_CLK.\n");
	} else {
			pr_err("%s:nfc_rf_clk is null.we can't disable it \n",__FUNCTION__);
	}
	/*ret=irq_set_irq_wake(client->irq,0);
	if(ret){
		pr_err("%s:faile to disable irq wake. \n",__FUNCTION__);
	}*/
	misc_deregister(&dev->miscdev);
	if (dev->disable)
		dev->disable();
	dev->read_irq = PN547_NONE;
	free_irq(client->irq, dev);
	if (dev->free_resources)
		dev->free_resources();
	mutex_destroy(&dev->read_mutex);
	kfree(dev);

	pn547_dev = NULL;
	
	return 0;
}

static struct of_device_id nxp_match_table[] = {
	{ .compatible = "nxp,i2c_adapter",},
	{ },
};

static const struct i2c_device_id pn547_id[] = {
	{ PN547_DRIVER_NAME, 0 },
	{ }
};

#ifdef ZTEMT_NFC_CONFIG_FB
static SIMPLE_DEV_PM_OPS(nxp_pn547_pm_ops,nxp_pn547_suspend, nxp_pn547_resume);
#endif
static struct i2c_driver pn547_driver = {
	.id_table	= pn547_id,
	.probe		= pn547_probe,
	.remove		= pn547_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= PN547_DRIVER_NAME,
#ifdef ZTEMT_NFC_CONFIG_FB //NFC_PM_CONFIG
		.pm = &nxp_pn547_pm_ops,
#endif
		.of_match_table = nxp_match_table,
	},
};

module_i2c_driver(pn547_driver);

MODULE_DESCRIPTION("I2C_TEST_NXP");
MODULE_LICENSE("GPL");
