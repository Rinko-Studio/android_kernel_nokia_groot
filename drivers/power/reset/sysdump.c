
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/rtc.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <asm-generic/kdebug.h>
#include <linux/slab.h>
#include <linux/kdebug.h>


#define SYSDUMP_MAGIC_VOLUP  (0x766f7570) // v-l-u-p
#define SYSDUMP_MAGIC_VOLDN  (0X766f646e) // v-l-d-n

void debug_check_crash_key(unsigned int code, int value)
{
	static unsigned int volup_p;
	static unsigned int voldown_p;
	static unsigned int loopcount;
	static unsigned long vol_pressed;

#if 0
	/* Must be deleted later */
	pr_info("Test %s:key code(%d) value(%d),(up:%d,down:%d),lpct(%d),vop(%ld)\n", __func__,
		code, value, volup_p, voldown_p, loopcount, vol_pressed);
#endif

	/*  Enter Force Upload
	 *  hold the volume down and volume up
	 *  and then press power key twice
	 */
	if (value) {
		if (code == KEY_VOLUMEUP)
			volup_p = SYSDUMP_MAGIC_VOLUP;
		if (code == KEY_VOLUMEDOWN)
			voldown_p = SYSDUMP_MAGIC_VOLDN;

		if ((volup_p == SYSDUMP_MAGIC_VOLUP) && (voldown_p == SYSDUMP_MAGIC_VOLDN)) {
			if (!vol_pressed)
				vol_pressed = jiffies;

			if (code == KEY_POWER) {
				pr_info("%s: Crash key count : %d,vol_pressed:%ld\n", __func__,
					++loopcount, vol_pressed);
				if (time_before(jiffies, vol_pressed + 5 * HZ)) {
					if (loopcount == 2)
						panic("Crash Key");
				} else {
					pr_info("%s: exceed 5s(%u) between power key and volup/voldn key\n",
						__func__, jiffies_to_msecs(jiffies - vol_pressed));
					volup_p = 0;
					voldown_p = 0;
					loopcount = 0;
					vol_pressed = 0;
				}
			}
		}
	} else {
		if (code == KEY_VOLUMEUP) {
			volup_p = 0;
			loopcount = 0;
			vol_pressed = 0;
		}
		if (code == KEY_VOLUMEDOWN) {
			voldown_p = 0;
			loopcount = 0;
			vol_pressed = 0;
		}
	}
}
static int sysdump_connect(struct input_handler *handler,
			 struct input_dev *dev,
			 const struct input_device_id *id)
{
	struct input_handle *sysdump_handle;
	int error;

	sysdump_handle = (struct input_handle *)kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!sysdump_handle)
		return -ENOMEM;

	sysdump_handle->dev = dev;
	sysdump_handle->handler = handler;
	sysdump_handle->name = "sysdump";

	error = input_register_handle(sysdump_handle);
	if (error) {
		pr_err("Failed to register input sysrq handler, error %d\n",
			error);
		goto err_free;
	}

	error = input_open_device(sysdump_handle);
	if (error) {
		pr_err("Failed to open input device, error %d\n", error);
		goto err_unregister;
	}

	return 0;

 err_unregister:
	input_unregister_handle(sysdump_handle);
 err_free:
	kfree(sysdump_handle);
	return error;
}

static void sysdump_event(struct input_handle *handle,
	unsigned int type, unsigned int code, int value)
{
	if (type == EV_KEY && code != BTN_TOUCH)
		debug_check_crash_key(code, value);
}

static void sysdump_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
}

static const struct input_device_id sysdump_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{},
};

static struct input_handler sysdump_handler = {
	.event = sysdump_event,
	.connect	= sysdump_connect,
	.disconnect	= sysdump_disconnect,
	.name = "sysdump_crashkey",
	.id_table	= sysdump_ids,
};

int sysdump_sysctl_init(void)
{
	if (input_register_handler(&sysdump_handler))
		pr_err("regist sysdump_handler failed.\n");

	return 0;
}

void sysdump_sysctl_exit(void)
{
	input_unregister_handler(&sysdump_handler);
}

late_initcall_sync(sysdump_sysctl_init);
module_exit(sysdump_sysctl_exit);

MODULE_AUTHOR("Heylay.huang");
MODULE_DESCRIPTION("comb key trigger dump");
MODULE_LICENSE("GPL");
