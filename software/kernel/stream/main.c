#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/math64.h>

#include "smi_kernel.h"

#define DRIVER_NAME "mychardev"

#define SOAPY_SDR_HAS_TIME   (1 << 2)
#define SOAPY_SDR_TIME_ERROR   (-6)

/* TODO que faire avec ca */
#define FS (1000000000/(4*1000000))


/* TODO mettre dans un context */
uint64_t lower_bound_in_nano;
uint64_t upper_bound_in_nano;

static int mychardev_open(struct inode *inode, struct file *file);
static int mychardev_release(struct inode *inode, struct file *file);
static long mychardev_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t mychardev_read(struct file *file, char __user *buf, size_t count, loff_t *offset);
static ssize_t mychardev_write(struct file *file, const char __user *buf, size_t count, loff_t *offset);

static const struct file_operations mychardev_fops = {
    .owner      = THIS_MODULE,
    .open       = mychardev_open,
    .release    = mychardev_release,
    .unlocked_ioctl = mychardev_ioctl,
    .read       = mychardev_read,
    .write       = mychardev_write,
};

struct mychar_device_data {
    struct cdev cdev;

        struct kfifo rx_fifo;
        struct md meta_data;
        struct mutex read_lock;
        wait_queue_head_t readable;
        bool streaming;

        struct stream_config stream_config;
        struct setup_stream setup_stream;
};

static int dev_major = 0;
static struct class *mychardev_class = NULL;
static struct mychar_device_data mychardev_data;
static struct mychar_device_data *inst = &mychardev_data;

static int mychardev_uevent(struct device *dev, struct kobj_uevent_env *env)
{
    add_uevent_var(env, "DEVMODE=%#o", 0666);
    return 0;
}

static int __init mychardev_init(void)
{
	int err;
	dev_t dev;

	err = alloc_chrdev_region(&dev, 0, 1, "mychardev");

	dev_major = MAJOR(dev);

	mychardev_class = class_create(THIS_MODULE, "mychardev");
	mychardev_class->dev_uevent = mychardev_uevent;

	cdev_init(&mychardev_data.cdev, &mychardev_fops);
	mychardev_data.cdev.owner = THIS_MODULE;

	cdev_add(&mychardev_data.cdev, MKDEV(dev_major, 0), 1);

	device_create(mychardev_class, NULL, MKDEV(dev_major, 0), NULL, "mychardev");

	init_waitqueue_head(&inst->readable);
	mutex_init(&inst->read_lock);

	return 0;
}

static void __exit mychardev_exit(void)
{

	device_destroy(mychardev_class, MKDEV(dev_major, 0));

	class_unregister(mychardev_class);
	class_destroy(mychardev_class);

	unregister_chrdev_region(MKDEV(dev_major, 0), MINORMASK);
}

static bool is_open = false;

int bam(uint32_t *samples, int nb_samples, uint64_t ts_first_sample) {

	if (is_open == false) return 0;

        if (inst->streaming) {

                if (lower_bound_in_nano == 0) {
                        lower_bound_in_nano = ts_first_sample;

                        if (inst->stream_config.num_elements == 0) {
                                upper_bound_in_nano = U64_MAX;
                        } else {
                                upper_bound_in_nano = lower_bound_in_nano + inst->stream_config.num_elements * FS;
                        }
                }

                if (kfifo_avail(&inst->rx_fifo) < sizeof(uint32_t) * nb_samples) {
                        //printk("LAZY BOY %d", kfifo_avail(&inst->rx_fifo));
                } else if (kfifo_len(&inst->rx_fifo) == 0) {
                        inst->meta_data.timestamp = ts_first_sample;
                }

                if (ts_first_sample > upper_bound_in_nano || (ts_first_sample + nb_samples*FS) < lower_bound_in_nano) {
                        //printk("discard from %lld to %lld", ts_first_sample, ts_first_sample + nb_samples*FS);
                } else {

                        uint64_t start = div_u64((max(lower_bound_in_nano, ts_first_sample) - ts_first_sample), FS);
                        uint64_t stop = div_u64((min(upper_bound_in_nano, ts_first_sample + nb_samples*FS) - ts_first_sample), FS);

                        uint8_t *samples2 = (uint8_t*)samples;

                        //printk("start %lld stop %lld %lld %lld", start, stop, upper_bound_in_nano, min(upper_bound_in_nano, ts_first_sample + nb_samples*FS));
                        //printk("%x %x %x %x %x %x %x %x", samples2[start], samples2[start+1], samples2[start+2], samples2[start + 3],
                        //              samples2[start+4], samples2[start+5], samples2[start+6], samples2[start + 7]);

                        int n = kfifo_in(&inst->rx_fifo, &samples[start], (stop - start) * sizeof(uint32_t));
                        wake_up_interruptible(&inst->readable);
                }
        }

        return 0; 
}

static int mychardev_open(struct inode *inode, struct file *file)
{
	int ret;

	printk("MYCHARDEV: Device open\n");

	/* TODO allocation should be done in activate_rx */

	ret = kfifo_alloc(&inst->rx_fifo, 1024*1024/2, GFP_KERNEL);

	if (ret) {
		printk(KERN_ERR "error kfifo_alloc\n");
		return ret;
	}

	inst->streaming = false;
	inst->meta_data.timestamp = 0;

	is_open = true;

	return 0;
}

static int mychardev_release(struct inode *inode, struct file *file)
{
	kfifo_free(&inst->rx_fifo);
        
    printk("MYCHARDEV: Device close\n");
    is_open = false;
    return 0;
}

extern int my_test(const char*, size_t);
extern int bcm2835_smi_start(void);

static long mychardev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        long ret = 0;
        static bool check_at = false;

        switch (cmd) {
                case SETUP_STREAM_TX: 
					/* nothing to do */
					printk("SETUP_STREAM_TX");
					break;
                case SETUP_STREAM_RX: 
					/* nothing to do */
					printk("SETUP_STREAM_RX");
					break;
					/* nothing to do */
					//if (copy_from_user(&inst->setup_stream, (void *)arg, sizeof(struct setup_stream))) {
					//        printk("setup stream copy failed.");
					//} else {
					//        printk("setup stream is_rx %d.", inst->setup_stream.is_rx);
					//	if (inst->setup_stream.is_rx) {
					//		bcm2835_smi_start();
					//	}
					//}
		
                case ACTIVATE_STREAM_TX: 
					/* nothing to do */
					printk("ACTIVATE_STREAM_RX");
					break;

                case ACTIVATE_STREAM_RX: 

                        if (copy_from_user(&inst->stream_config, (void *)arg, sizeof(struct stream_config))) {
                                printk("stream config copy failed.");
                        } else {

                                /* if activation_time equals 0 (aka stream now) the following boundaries
                                 * will be recomputed upon acquisition of first samples */

                                lower_bound_in_nano = inst->stream_config.activation_time;

                                if (inst->stream_config.num_elements == 0) {
                                        upper_bound_in_nano = U64_MAX;
                                } else {
                                        upper_bound_in_nano = lower_bound_in_nano + inst->stream_config.num_elements * FS;
                                }

                                check_at = inst->stream_config.flags == SOAPY_SDR_HAS_TIME;

                                kfifo_reset(&inst->rx_fifo);

                                inst->streaming = true;
                        }
                        break;
                case DEACTIVATE_STREAM_TX: 
						/* TODO stop DMA transfer and cleanup */
						printk("DEACTIVATE_STREAM_TX");
                        break;
                case DEACTIVATE_STREAM_RX: 
						/* TODO stop DMA transfer and cleanup */
						printk("DEACTIVATE_STREAM_TX");
                        break;
                case CLOSE_STREAM_TX: 
					printk("CLOSE_STREAM_TX");
                        break;
                case CLOSE_STREAM_RX: 
					printk("CLOSE_STREAM_RX");
                        break;

                case GET_METADATA:

                        if (kfifo_is_empty(&inst->rx_fifo)) {
                                ret = wait_event_interruptible(inst->readable, !kfifo_is_empty(&inst->rx_fifo));

                                if (ret == -ERESTARTSYS) { 
                                        pr_err("interrupted\n");
                                        return -EINTR;
                                }
                        }

                        //printk("check_at %d stream_config.activation_time %lld md.timestamp %lld", check_at, inst->stream_config.activation_time, inst->meta_data.timestamp);
                        if (check_at && inst->stream_config.activation_time < inst->meta_data.timestamp) {

                                ret = SOAPY_SDR_TIME_ERROR;
                                printk("TOO LATE %ld", ret);

                        } else {

                                check_at = false;

                                if (copy_to_user((void *)arg, &inst->meta_data, sizeof(struct md))) {
                                        printk("settings copy failed.");
                                }
                        }
                        break;
                default: 
                        printk("invalid ioctl cmd: %d", cmd);
                        ret = -ENOTTY;
                        break;

        }
        return ret;
}


static ssize_t mychardev_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
        my_test(buf, count);
        return count;
}

static ssize_t mychardev_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
        int ret = 0;
        unsigned int copied;

        if (kfifo_len(&inst->rx_fifo) < count) { 

                if (file->f_flags & O_NONBLOCK) {
                        return -EAGAIN;
                } else {

                        ret = wait_event_interruptible(inst->readable, kfifo_len(&inst->rx_fifo) >= count);
                        if (ret == -ERESTARTSYS) { 
                                pr_err("interrupted\n");
                                return -EINTR;
                        }
                }
        }

        if (mutex_lock_interruptible(&inst->read_lock)) {
                return -EINTR;
        }

        ret = kfifo_to_user(&inst->rx_fifo, buf, count, &copied);
        //printk("kfb %x %x %x %x %x %x %x %x", buf[0], buf[1], buf[2], buf[3],
        //                              buf[4], buf[5], buf[6], buf[7]);

        mutex_unlock(&inst->read_lock);

        if (ret == 0) {
                inst->meta_data.timestamp += copied;
        }

        return ret ? ret : copied;
}

EXPORT_SYMBOL(bam);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oleg Kutkov <elenbert@gmail.com>");

module_init(mychardev_init);
module_exit(mychardev_exit);
