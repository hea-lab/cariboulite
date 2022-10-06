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
#include "../dev/bcm2835_smi.h"

#define DRIVER_NAME          "mychardev"

#define SOAPY_SDR_HAS_TIME   (1 << 2)
#define SOAPY_SDR_TIME_ERROR (-6)

/* TODO  this should come from configuration */
#define FS (1000000000/(4*1000000))

struct my_tx_ctxt {
    uint64_t lb_in_ns;
    uint64_t ub_in_ns;
};

extern int bcm2835_smi_tx_start(const char*, size_t);
extern int bcm2835_smi_rx_start(void);
extern int bcm2835_smi_rx_stop(void);
extern int bcm2835_smi_tx_stop(void);
extern int bcm2835_smi_reg_cb(int (*f)(uint32_t *samples, int nb_samples, uint64_t ts_first_samples));

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

    struct stream_config stream_config;
    struct my_tx_ctxt tx_ctxt;
};

static int dev_major = 0;
static struct class *mychardev_class = NULL;
static struct mychar_device_data mychardev_data = {0};
static struct mychar_device_data *inst = &mychardev_data;

int process_rx_samples_dummy(uint32_t *samples, int nb_samples, uint64_t ts_first_sample) {
    return 0;
}

int process_rx_samples(uint32_t *samples, int nb_samples, uint64_t ts_first_sample) {

    if (inst->tx_ctxt.lb_in_ns == 0) {
        inst->tx_ctxt.lb_in_ns = ts_first_sample;

        if (inst->stream_config.num_elements == 0) {
            inst->tx_ctxt.ub_in_ns = U64_MAX;
        } else {
            inst->tx_ctxt.ub_in_ns = inst->tx_ctxt.lb_in_ns + inst->stream_config.num_elements * FS;
        }
    }

    if (kfifo_avail(&inst->rx_fifo) < sizeof(uint32_t) * nb_samples) {
//        printk("consumer is too slow %d", kfifo_avail(&inst->rx_fifo));
        return 1;
    } else if (kfifo_len(&inst->rx_fifo) == 0) {
        inst->meta_data.timestamp = ts_first_sample;
    }

    if (ts_first_sample > inst->tx_ctxt.ub_in_ns || (ts_first_sample + nb_samples*FS) < inst->tx_ctxt.lb_in_ns) {
        printk("discard from %lld to %lld", ts_first_sample, ts_first_sample + nb_samples*FS);
    } else {

	//printk("lb_in_ns %lld ub_in_ns %lld %lld", inst->tx_ctxt.lb_in_ns, inst->tx_ctxt.ub_in_ns, ts_first_sample);

        uint64_t start = div_u64((max(inst->tx_ctxt.lb_in_ns, ts_first_sample) - ts_first_sample), FS);
        uint64_t stop = div_u64((min(inst->tx_ctxt.ub_in_ns, ts_first_sample + nb_samples*FS) - ts_first_sample), FS);

	//printk("start %lld stop %lld", start, stop);

        kfifo_in(&inst->rx_fifo, &samples[start], (stop - start) * sizeof(uint32_t));
        wake_up_interruptible(&inst->readable);
    }

    return 0; 
}


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

    int ret = kfifo_alloc(&inst->rx_fifo, 1024*1024/2, GFP_KERNEL);

    if (ret) {
        printk(KERN_ERR "error kfifo_alloc\n");
        return ret;
    }

    return 0;
}

static void __exit mychardev_exit(void)
{
    kfifo_free(&inst->rx_fifo);

    device_destroy(mychardev_class, MKDEV(dev_major, 0));

    class_unregister(mychardev_class);
    class_destroy(mychardev_class);

    unregister_chrdev_region(MKDEV(dev_major, 0), MINORMASK);
}

static int mychardev_open(struct inode *inode, struct file *file)
{
    printk("MYCHARDEV: Device open\n");
    return 0;
}

static int mychardev_release(struct inode *inode, struct file *file)
{
    printk("MYCHARDEV: Device close\n");
    return 0;
}

static long mychardev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;
    static bool check_first_sample = false;

    switch (cmd) {
        case SETUP_STREAM_TX: 
            /* nothing to do */
            printk("SETUP_STREAM_TX");
            break;

        case SETUP_STREAM_RX: 
            /* nothing to do */
            printk("SETUP_STREAM_RX");
            break;

        case ACTIVATE_STREAM_TX: 
            /* nothing to do */
            printk("ACTIVATE_STREAM_TX");
            break;

        case ACTIVATE_STREAM_RX: 
            printk("ACTIVATE_STREAM_RX");

            /* FIXME que se passe t il quand on active 2 fois sans deactivate */

            if (copy_from_user(&inst->stream_config, (void *)arg, sizeof(struct stream_config))) {
                printk("stream config copy failed.");
            } else {

                /* if activation_time equals 0 (aka stream now) the following boundaries
                 * will be recomputed upon acquisition of first samples */

                inst->tx_ctxt.lb_in_ns = inst->stream_config.activation_time;

                printk("ACTIVATE_STREAM_RX : lb_in_ns %llu", inst->tx_ctxt.lb_in_ns);

                if (inst->stream_config.num_elements == 0) {
                    inst->tx_ctxt.ub_in_ns = U64_MAX;
                } else {
                    inst->tx_ctxt.ub_in_ns = inst->tx_ctxt.lb_in_ns + inst->stream_config.num_elements * FS;
                }

                check_first_sample = inst->stream_config.flags == SOAPY_SDR_HAS_TIME;

                kfifo_reset(&inst->rx_fifo);
                bcm2835_smi_reg_cb(process_rx_samples);
                bcm2835_smi_rx_start();
            }
            break;
        case DEACTIVATE_STREAM_TX: 
            printk("DEACTIVATE_STREAM_TX");
            bcm2835_smi_tx_stop();
            break;
        case DEACTIVATE_STREAM_RX: 
            /* TODO stop DMA transfer and cleanup */
            printk("DEACTIVATE_STREAM_RX");
            bcm2835_smi_reg_cb(process_rx_samples_dummy);
            bcm2835_smi_rx_stop();
            break;
        case CLOSE_STREAM_TX: 
            printk("CLOSE_STREAM_TX");
            break;
        case CLOSE_STREAM_RX: 
            bcm2835_smi_reg_cb(process_rx_samples_dummy);
            printk("CLOSE_STREAM_RX");
            //bcm2835_smi_rx_stop();
            break;

        case GET_METADATA:

            if (kfifo_is_empty(&inst->rx_fifo)) {
                ret = wait_event_interruptible(inst->readable, !kfifo_is_empty(&inst->rx_fifo));

                if (ret == -ERESTARTSYS) { 
                    pr_err("interrupted\n");
                    return -EINTR;
                }
            }

            if (check_first_sample && inst->stream_config.activation_time < inst->meta_data.timestamp) {
                ret = SOAPY_SDR_TIME_ERROR;
                printk("TOO LATE %ld", ret);

            } else {

                check_first_sample = false;

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
    bcm2835_smi_tx_start(buf, count);
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

    mutex_unlock(&inst->read_lock);

    if (ret == 0) {
        inst->meta_data.timestamp += copied;
    }

    return ret ? ret : copied;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oleg Kutkov <elenbert@gmail.com>");

module_init(mychardev_init);
module_exit(mychardev_exit);
