/* ************************************************************************
 *       Filename:  hq_drv2624.c
 *    Description:  
 *        Version:  1.0
 *        Created:  03/25/20 18:55:42
 *       Revision:  none
 *       Compiler:  gcc
 *         Author:  YOUR NAME (), 
 *        Company:  
 * ************************************************************************/

#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include "drv2624.h"
#include <linux/hqsysfs.h>
//#include "parse_rtp.h"
//#define HOLD_RAM
#define DRV2624_PATH "/mnt/vendor/persist/DRV2624_DATA.txt"
#define LED_BRIGHTNESS_FAST 255
static struct regmap_config drv2624_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static int dev_run_diagnostics(struct drv2624_data *pDRV2624);
static inline int drv2624_calculate_voltage(unsigned int voltage);
static int drv2624_reg_write(struct drv2624_data *pDRV2624,
					unsigned char reg, unsigned char val);
static int drv2624_set_go_bit(struct drv2624_data *pDRV2624, unsigned char val);
static int drv2624_stop(struct drv2624_data *pDRV2624);
static void drv2624_set_mode_reg(struct drv2624_data *pDRV2624, char WorkMode);
/**
 * RW Functions for DRV2624 registers through I2C
 * drv2624_reg_read, drv2624_reg_write,
 * drv2624_bulk_read, drv2624_bulk_write
 **/
static int drv2624_reg_read(struct drv2624_data *pDRV2624, unsigned char reg)
{
	unsigned int val;
	int nResult;

	mutex_lock(&pDRV2624->dev_lock);
	nResult = regmap_read(pDRV2624->mpRegmap, reg, &val);
	mutex_unlock(&pDRV2624->dev_lock);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s I2C error %d\n", __func__, nResult);
		return nResult;
	} else {
		/*dev_dbg(pDRV2624->dev, "%s, Reg[0x%x]=0x%x\n", __func__, reg,
			val);*/
		return val;
	}
}

static int drv2624_reg_write(struct drv2624_data *pDRV2624,
					unsigned char reg, unsigned char val)
{
	int nResult;

	mutex_lock(&pDRV2624->dev_lock);
	nResult = regmap_write(pDRV2624->mpRegmap, reg, val);
	mutex_unlock(&pDRV2624->dev_lock);
	if (nResult < 0)
		dev_err(pDRV2624->dev, "%s reg=0x%x, value=0%x error %d\n",
			__func__, reg, val, nResult);
	//dev_dbg(pDRV2624->dev, "%s, Reg[0x%x]=0x%x\n", __func__, reg, val);
	return nResult;
}

#if 0
static int drv2624_bulk_read(struct drv2624_data *pDRV2624,
					unsigned char reg, unsigned char *buf,
					unsigned int count)
{
	int nResult;
	mutex_lock(&pDRV2624->dev_lock);
	nResult = regmap_bulk_read(pDRV2624->mpRegmap, reg, buf, count);
	mutex_unlock(&pDRV2624->dev_lock);
	if (nResult < 0)
		dev_err(pDRV2624->dev, "%s reg=0%x, count=%d error %d\n",
			__func__, reg, count, nResult);
	return nResult;
}
#endif

static int drv2624_bulk_write(struct drv2624_data *pDRV2624,
					unsigned char reg, const u8 *buf,
					unsigned int count)
{
	int nResult, i;

	mutex_lock(&pDRV2624->dev_lock);
	nResult = regmap_bulk_write(pDRV2624->mpRegmap, reg, buf, count);
	mutex_unlock(&pDRV2624->dev_lock);
	if (nResult < 0)
		dev_err(pDRV2624->dev, "%s reg=0%x, count=%d error %d\n",
			__func__, reg, count, nResult);
	for (i = 0; i < count; i++)
		dev_dbg(pDRV2624->dev, "%s, Reg[0x%x]=0x%x\n", __func__,
			reg + i, buf[i]);
	return nResult;
}

static int drv2624_set_bits(struct drv2624_data *pDRV2624,
			    unsigned char reg, unsigned char mask,
			    unsigned char val)
{
	int nResult;

	mutex_lock(&pDRV2624->dev_lock);
	nResult = regmap_update_bits(pDRV2624->mpRegmap, reg, mask, val);
	mutex_unlock(&pDRV2624->dev_lock);
	if (nResult < 0)
		dev_err(pDRV2624->dev,
			"%s reg=%x, mask=0x%x, value=0x%x error %d\n",
			__func__, reg, mask, val, nResult);
	dev_dbg(pDRV2624->dev, "%s, Reg[0x%x]:M=0x%x, V=0x%x\n", __func__,
		reg, mask, val);
	return nResult;
}

static int drv2624_set_waveform(struct drv2624_data *pDRV2624,
					struct drv2624_waveform_sequencer *pSequencer)
{
	int nResult = 0;
	int i = 0;
	unsigned char loop[2] = { 0 };
	unsigned char effects[DRV2624_SEQUENCER_SIZE] = { 0 };
	unsigned char len = 0;

	dev_dbg(pDRV2624->dev, "%s:enter\n", __func__);
	pDRV2624->msWaveformSequencer.msWaveform[0].mnLoop = 0;
	pDRV2624->msWaveformSequencer.msWaveform[1].mnEffect = 0;
	pDRV2624->msWaveformSequencer.msWaveform[1].mnLoop = 0;
	for (i = 0; i < DRV2624_SEQUENCER_SIZE; i++) {
		if (pSequencer->msWaveform[i].mnEffect != 0) {
			len++;
			if (i < 4)
				loop[0] |= (pSequencer->msWaveform[i].mnLoop << (2 * i));
			else
				loop[1] |= (pSequencer->msWaveform[i].mnLoop << (2 * (i - 4)));
			effects[i] = pSequencer->msWaveform[i].mnEffect;
		} else
			break;
	}
	dev_dbg(pDRV2624->dev, "%s:len =%d, effects[0] = %d\n", __func__, len,
		effects[0]);
	if (len == 1) {
		nResult =
			drv2624_reg_write(pDRV2624, DRV2624_REG_SEQUENCER_1, effects[0]);
		drv2624_reg_write(pDRV2624, DRV2624_REG_SEQ_LOOP_1, loop[0]);
		nResult = drv2624_reg_write(pDRV2624, 0x10, 0);
	} else
		nResult = drv2624_bulk_write(pDRV2624, DRV2624_REG_SEQUENCER_1, effects, len);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "sequence error\n");
		goto end;
	}
	if (len > 1) {
		if ((len - 1) <= 4)
			drv2624_reg_write(pDRV2624, DRV2624_REG_SEQ_LOOP_1, loop[0]);

		else
			drv2624_bulk_write(pDRV2624, DRV2624_REG_SEQ_LOOP_1, loop, 2);
	}
end:
	return nResult;
}

static int fw_chksum(const struct firmware *fw)
{
	int sum = 0;
	int i = 0;
	int size = fw->size;
	const unsigned char *pBuf = fw->data;

	for (i = 0; i < size; i++) {
		if ((i > 11) && (i < 16)) {
		} else
			sum += pBuf[i];
	}
	return sum;
}
#if 0
static int drv2624_get_effect_timems(struct drv2624_data *pDRV2624,
							unsigned char effect)
{
	unsigned char *fw = &pDRV2624->mnFwRam[0];
	u16 header_address, tmp;
	u16 address = 0;
	unsigned char effect_repeats = 0;
	unsigned int effect_size = 0;
	int i = 0;
	unsigned int ticks = 0;
	unsigned int playback_interval = 0;
	effect++;
	header_address = (effect - 1) * 3 + 1;
	tmp = fw[header_address];
	address = tmp << 8 | fw[header_address + 1];
	effect_repeats = (fw[header_address + 2] & 0xe0) >> 5;
	effect_size = fw[header_address + 2] & 0x1f;
	for (i = 0; i < effect_size / 2; i++) {
		ticks += fw[address + (i * 2) + 1];
	}
	playback_interval =
	    (pDRV2624->msWaveformSetting.mnInterval == INTERVAL_5MS) ? 5 : 1;
	return ticks * (effect_repeats + 1) * playback_interval;
}
#endif

/* drv2624_firmware_load:
* This function is called by the
* request_firmware_nowait function as soon
* as the firmware has been loaded from the file.
* The firmware structure contains the data and$
* the size of the firmware loaded.
*
* @fw: pointer to firmware file to be dowloaded
* @context: pointer variable to drv2624 data
*/
static void drv2624_firmware_load(const struct firmware *fw, void *context)
{
	struct drv2624_data *pDRV2624 = context;
	int size = 0, fwsize = 0, i = 0;
	const unsigned char *pBuf = NULL;

	mutex_lock(&pDRV2624->lock);
	if (fw != NULL) {
		pBuf = fw->data;
		size = fw->size;
		if (size > 1024) {
			dev_err(pDRV2624->dev,
				"%s, ERROR!! firmware size %d too big\n",
				__func__, size);
		} else {
			memcpy(&(pDRV2624->fw_header), pBuf,
				sizeof(struct drv2624_fw_header));
			if ((pDRV2624->fw_header.fw_magic !=
				DRV2624_MAGIC) || (pDRV2624->fw_header.fw_size != size)
				|| (pDRV2624->fw_header.fw_chksum != fw_chksum(fw))) {
				dev_dbg(pDRV2624->dev,
					"%s, ERROR!! firmware not right:Magic=0x%x, Size=%d, chksum=0x%x\n",
					__func__, pDRV2624->fw_header.fw_magic,
					pDRV2624->fw_header.fw_size,
					pDRV2624->fw_header.fw_chksum);
			} else {
				dev_info(pDRV2624->dev, "%s, firmware good\n", __func__);
				pDRV2624->effects_count = pDRV2624->fw_header.fw_effCount;
				pBuf += sizeof(struct drv2624_fw_header);
				dev_dbg(pDRV2624->dev,
					"%s: pDRV2624->fw_header.fw_effCount =%d\n",
					__func__,
					pDRV2624->fw_header.fw_effCount);

				drv2624_reg_write(pDRV2624, DRV2624_REG_RAM_ADDR_UPPER, 0);
				drv2624_reg_write(pDRV2624, DRV2624_REG_RAM_ADDR_LOWER, 0);
				fwsize = size - sizeof(struct drv2624_fw_header);
				dev_dbg(pDRV2624->dev, "%s, firmwar fwsize = %d\n", __func__, fwsize);
				for (i = 0; i < fwsize; i++) {
					dev_dbg(pDRV2624->dev,
						"%s, firmware bytes pBuf[%03d]=0x%02x\n",
						__func__, i, pBuf[i]);
					drv2624_reg_write(pDRV2624, DRV2624_REG_RAM_DATA, pBuf[i]);
				}
#if 0
				memset(&pDRV2624->mnFwRam[0], 0, DRV2624_RAM_SIZE);
				memcpy(&pDRV2624->mnFwRam[0], pBuf, fwsize);
				for (i = 0; i < pDRV2624->fw_header.fw_effCount; i++)
					pDRV2624->mnEffectTimems[i] =
							drv2624_get_effect_timems(pDRV2624, i);
#endif
			}
		}
	} else
		dev_err(pDRV2624->dev, "%s, ERROR!! firmware not found\n", __func__);
	release_firmware(fw);
	mutex_unlock(&pDRV2624->lock);
}
/**
 *
 * bRTP = NO == 0; Enable all interrupt of DRV2624
 * bRTP = 1 == 1; Only Enable critical interrupt,  PROCESS_DONE and PRG_ERROR
 *
 **/
static int drv2624_enableIRQ(struct drv2624_data *pDRV2624, unsigned char bRTP)
{
	int nResult = 0;
	unsigned char mask = INT_ENABLE_CRITICAL;
	if (!pDRV2624->mbIRQUsed)
		goto end;
	if (pDRV2624->mbIRQEnabled)
		goto end;
	if (bRTP == 0)
		mask = INT_ENABLE_ALL;
	nResult = drv2624_reg_read(pDRV2624, DRV2624_REG_STATUS);
	if (nResult < 0)
		goto end;
	nResult = drv2624_reg_write(pDRV2624, DRV2624_REG_INT_ENABLE, mask);
	if (nResult < 0)
		goto end;
	enable_irq(pDRV2624->mnIRQ);
	pDRV2624->mbIRQEnabled = true;
end:	return nResult;
}

static void drv2624_disableIRQ(struct drv2624_data *pDRV2624)
{
	dev_dbg(pDRV2624->dev, "%s:entter\n", __func__);
	if (pDRV2624->mbIRQUsed) {
		if (pDRV2624->mbIRQEnabled) {
			disable_irq_nosync(pDRV2624->mnIRQ);
			drv2624_reg_write(pDRV2624, DRV2624_REG_INT_ENABLE,
					  INT_MASK_ALL);
			pDRV2624->mbIRQEnabled = false;
		}
	}
}

static int drv2624_set_go_bit(struct drv2624_data *pDRV2624, unsigned char val)
{
	int nResult = 0, value = 0;
	int retry = GO_BIT_MAX_RETRY_CNT;

	val &= DRV2624_GO_BIT_MASK;
	nResult = drv2624_reg_write(pDRV2624, DRV2624_REG_GO, val);
	if (nResult < 0)
		goto end;

	mdelay(GO_BIT_CHECK_INTERVAL);
	value = drv2624_reg_read(pDRV2624, DRV2624_REG_GO);
	if (value < 0) {
		nResult = value;
		goto end;
	}
	//dev_dbg(pDRV2624->dev, "%s, go value = %d\n", __func__, value);
	while (((value & DRV2624_GO_BIT_MASK) != val) && (retry > 0)) {
		value = drv2624_reg_read(pDRV2624, DRV2624_REG_GO);
		dev_dbg(pDRV2624->dev, "%s, GO bit %d\n", __func__, value);
		mdelay(GO_BIT_CHECK_INTERVAL);
		retry--;
	}
	dev_dbg(pDRV2624->dev, "%s: pull go bit success!\n", __func__);
end:
	return nResult;
}

static inline int drv2624_change_mode(struct drv2624_data *pDRV2624,
				      drv2624_mode_t work_mode)
{
	pDRV2624->mnWorkMode = work_mode;
	drv2624_set_mode_reg(pDRV2624, work_mode);
	return drv2624_set_bits(pDRV2624, DRV2624_REG_MODE, WORKMODE_MASK,
				work_mode);
}

static inline void drv2624_set_stopflag(struct drv2624_data *pDRV2624)
{
	pDRV2624->mnVibratorPlaying = NO;
	dev_dbg(pDRV2624->dev, "%s: mnVibratorPlaying=%d\n", __func__,
			 pDRV2624->mnVibratorPlaying);
}

static int drv2624_get_diag_result(struct drv2624_data *pDRV2624,
				   unsigned char nStatus)
{
	int Re = 0, nResult = 0;
	pDRV2624->mDiagResult.mnResult = nStatus;
	if ((nStatus & DIAG_MASK) != DIAG_SUCCESS)
		dev_err(pDRV2624->dev, "%s: Diagnostic fail\n", __func__);

	else {
		nResult = drv2624_reg_read(pDRV2624, DRV2624_REG_DIAG_Z);
		if (nResult < 0)
			goto end;
		pDRV2624->mDiagResult.mnDiagZ = nResult;
		nResult = drv2624_reg_read(pDRV2624, DRV2624_REG_DIAG_K);
		if (nResult < 0)
			goto end;
		pDRV2624->mDiagResult.mnDiagK = nResult;
		Re = ((478 * pDRV2624->mDiagResult.mnDiagZ) /
			       (4 * pDRV2624->mDiagResult.mnDiagK + 719));
		dev_dbg(pDRV2624->dev,
			"%s: ZResult=0x%x, CurrentK=0x%x, Re = %d ohm\n",
			__func__, pDRV2624->mDiagResult.mnDiagZ,
			pDRV2624->mDiagResult.mnDiagK, Re);
	}
end:	return nResult;
}

/**
 * No need to stop in Waveform Sequencer Mode.
 * 1. Disable irq
 * 2. Cancel hrimer
 * 3. Set GO bit as STOP
 * 4. Set stop flag in drv2624_data struct
 *
 **/
static int drv2624_stop(struct drv2624_data *pDRV2624)
{
	int nResult = 0;

#if 0
	dev_dbg(pDRV2624->dev, "%s enter!\n", __func__);
	if (pDRV2624->mnWorkMode == MODE_WAVEFORM_SEQUENCER) {
		dev_dbg(pDRV2624->dev, "In sequence play, ignore stop\n");
		return 0;
	}
#endif

	dev_dbg(pDRV2624->dev, "%s, mnVibratorPlaying=%d, mnWorkMode=%d\n",
							__func__, pDRV2624->mnVibratorPlaying, pDRV2624->mnWorkMode);
	if (pDRV2624->mnVibratorPlaying == YES) {
		if (pDRV2624->mbIRQUsed)
			drv2624_disableIRQ(pDRV2624);
		if (hrtimer_active(&pDRV2624->haptics_timer))
			hrtimer_cancel(&pDRV2624->haptics_timer);
		nResult = drv2624_set_go_bit(pDRV2624, STOP);
		drv2624_set_stopflag(pDRV2624);
	}
	//debugging reset mode, play ram mode continuious, cause problems
	//drv2624_change_mode(pDRV2624, DRV2624_RTP_MODE);

	return nResult;
}
static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct drv2624_data *pDRV2624 =
	    container_of(timer, struct drv2624_data, haptics_timer);
	dev_dbg(pDRV2624->dev, "%s\n", __func__);
	schedule_work(&pDRV2624->vibrator_work);
	return HRTIMER_NORESTART;
}

/**
 * 1. Do work due to pDRV2624->mnWorkMode set before.
 * 2. For WORK_EFFECTSEQUENCER, WORK_CALIBRATION and WORK_DIAGNOSTIC
 *    check the GO bit until the process in DRV2624 has completed.
 * 3. For WORK_VIBRATOR, Stop DRV2624 directly.
 **/
static void vibrator_work_routine(struct work_struct *work)
{
	struct drv2624_data *pDRV2624 =
	    container_of(work, struct drv2624_data, vibrator_work);
	unsigned char status;
	int nResult = 0;

	mutex_lock(&pDRV2624->lock);
	dev_dbg(pDRV2624->dev, "%s, after mnWorkMode=0x%x\n",
							__func__, pDRV2624->mnWorkMode);
	if (pDRV2624->mbIRQUsed) {
		pDRV2624->mnIntStatus =
			drv2624_reg_read(pDRV2624, DRV2624_REG_STATUS);
		if (nResult < 0)
			goto err;
		drv2624_disableIRQ(pDRV2624);
		status = pDRV2624->mnIntStatus;
		dev_dbg(pDRV2624->dev, "%s, status=0x%x\n", __func__,
			pDRV2624->mnIntStatus);
		if (status & OVERCURRENT_MASK)
			dev_err(pDRV2624->dev,
				"ERROR, Over Current detected!!\n");
		if (status & OVERTEMPRATURE_MASK)
			dev_err(pDRV2624->dev,
				"ERROR, Over Temperature detected!!\n");
		if (status & ULVO_MASK)
			dev_err(pDRV2624->dev, "ERROR, VDD drop observed!!\n");
		if (status & PRG_ERR_MASK)
			dev_err(pDRV2624->dev, "ERROR, PRG error!!\n");
	}
	if (pDRV2624->mnWorkMode == DRV2624_RTP_MODE) {
		drv2624_stop(pDRV2624);
	} else if (pDRV2624->mnWorkMode == DRV2624_RAM_MODE) {
		dev_dbg(pDRV2624->dev, "%s: read go bit\n", __func__);
		status = drv2624_reg_read(pDRV2624, DRV2624_REG_GO);
		if ((status < 0) || (status == STOP)
				|| !pDRV2624->mnVibratorPlaying) {
			dev_err(pDRV2624->dev, "%s: status error = %d\n",
				__func__, status);
			//RAM haptic already stoped, just update flag
			drv2624_set_stopflag(pDRV2624);
		} else {
			if (!hrtimer_active(&pDRV2624->haptics_timer)) {
					dev_dbg(pDRV2624->dev, "will check GO bit after %d ms\n",
						GO_BIT_CHECK_INTERVAL);
					hrtimer_start(&pDRV2624->haptics_timer,
							ns_to_ktime((u64)GO_BIT_CHECK_INTERVAL * NSEC_PER_MSEC),
							HRTIMER_MODE_REL);
			}
		}
	}
err:
	mutex_unlock(&pDRV2624->lock);
}
/**
 * Play Waveform sequence stored in DRV2624_REG_SEQUENCER_1
 *
 **/
static int drv2624_playEffect(struct drv2624_data *pDRV2624)
{
	int nResult = 0;

	dev_dbg(pDRV2624->dev, "%s;enter\n", __func__);
	nResult = drv2624_change_mode(pDRV2624, DRV2624_WAVE_SEQ_MODE);
	if (nResult < 0)
		goto end;
	nResult = drv2624_set_go_bit(pDRV2624, GO);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s:enter go bit fail\n", __func__);
		goto end;
	}
	dev_dbg(pDRV2624->dev, "effects start\n");
	pDRV2624->mnVibratorPlaying = YES;
	/*if (pDRV2624->mbIRQUsed) {
		schedule_work(&pDRV2624->vibrator_work);
	}*/
end:
	return nResult;
}
static void haptics_playback_work_routine(struct work_struct *work)
{
	int nResult = 0;
	struct drv2624_data *pDRV2624 =
			container_of(work, struct drv2624_data, haptics_playback_work);

	mutex_lock(&pDRV2624->lock);
	nResult = drv2624_stop(pDRV2624);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s: stop failed!\n", __func__);
		goto end;
	}
	dev_dbg(pDRV2624->dev, "%s:work mode = %d\n", __func__, pDRV2624->mnWorkMode);
	if (pDRV2624->mnWorkMode == DRV2624_RAM_MODE) {
		drv2624_set_waveform(pDRV2624, &pDRV2624->msWaveformSequencer);
		nResult = drv2624_playEffect(pDRV2624);
		if ((nResult >= 0) && pDRV2624->mbIRQUsed) {
			drv2624_enableIRQ(pDRV2624, NO);
			goto end;
		}
	} else if (pDRV2624->mnWorkMode == DRV2624_RTP_MODE) {
		dev_dbg(pDRV2624->dev, "%s enter effect.length(%d) \n",
			__func__, pDRV2624->play.length);
		if (pDRV2624->play.length <= 0) {
			goto end;
		}
		nResult = drv2624_set_go_bit(pDRV2624, GO);
		if (nResult < 0)
			goto end;
		pDRV2624->mnVibratorPlaying = YES;
		if (pDRV2624->mbIRQUsed) {
			nResult = drv2624_enableIRQ(pDRV2624, YES);
		}

		if (pDRV2624->play.length != 0) {
			hrtimer_start(&pDRV2624->haptics_timer,
				ns_to_ktime((u64)pDRV2624->play.length
					* NSEC_PER_MSEC),
					HRTIMER_MODE_REL);
		}
	}
end:
	mutex_unlock(&pDRV2624->lock);
	dev_dbg(pDRV2624->dev, "%s: exit\n", __func__);
}

/* updata f0 */
static int dev_update_f0(struct drv2624_data *pDRV2624)
{
	int nResult, msb, lsb;

	dev_dbg(pDRV2624->dev, "%s: enter!\n", __func__);
	msb = drv2624_reg_read(pDRV2624, DRV2624_REG_RUNING_PERIOD_H);
	if (msb < 0)
		goto end;
	lsb = drv2624_reg_read(pDRV2624, DRV2624_REG_RUNING_PERIOD_L);
	if (lsb < 0)
		goto end;
	pDRV2624->mAutoCalResult.f0_data_msb = msb;
	pDRV2624->mAutoCalResult.f0_data_lsb = lsb;
	nResult =
	    drv2624_reg_write(pDRV2624, DRV2624_REG_OL_PERIOD_H, msb);
	if (nResult < 0)
		goto end;
	nResult =
	    drv2624_reg_write(pDRV2624, DRV2624_REG_OL_PERIOD_L, lsb);
	if (nResult < 0)
		goto end;

	dev_dbg(pDRV2624->dev, "%s: F0 is updated MSB:%d LSB:%d\n",
		__func__, msb, lsb);
	return 0;
end:
	dev_dbg(pDRV2624->dev, "%s: Failed to update f0 !\n", __func__);
	return -1;
}

static int dev_auto_calibrate(struct drv2624_data *pDRV2624)
{
	int nResult = 0;
	dev_info(pDRV2624->dev, "%s enter!\n", __func__);

	/* Set MODE register to Auto Level Calibration Routine
	 * and choose Trigger Function Internal */
	nResult = drv2624_reg_write(pDRV2624, DRV2624_REG_MODE,
						DRV2624_CALIBRATION_MODE_CFG);
	if (nResult < 0)
		goto end;
	nResult = drv2624_change_mode(pDRV2624, MODE_CALIBRATION);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s: change mode  Done nResult = %d\n",
			__func__, nResult);
		goto end;
	}
	nResult =
		drv2624_set_bits(pDRV2624, AUTO_CAL_TIME_REG, AUTO_CAL_TIME_MASK,
					AUTO_CAL_TIME_AUTO_TRIGGER);
	if (nResult < 0) {
		dev_err(pDRV2624->dev, "%s: set bits Done nResult = %d\n",
			__func__, nResult);
		goto end;
	}
	nResult = drv2624_set_go_bit(pDRV2624, GO);
	if (nResult < 0) {
		dev_err(pDRV2624->dev,
			"%s: calibrate go bit Done nResult = %d\n", __func__,
			nResult);
		goto end;
	}
	mdelay(2000);
	//pDRV2624->mnVibratorPlaying = YES;
	return nResult;
end:
	dev_err(pDRV2624->dev, "%s: Calibtion Done nResult = %d\n", __func__,
		nResult);
	return nResult;
}

static int drv2624_get_calibration_result(struct drv2624_data *pDRV2624)
{
	int nResult, cal_bemf, cal_comp, cal_gain;

	dev_dbg(pDRV2624->dev, "%s: enter!\n", __func__);
	nResult = drv2624_reg_read(pDRV2624, DRV2624_REG_STATUS);
	if (nResult < 0)
		goto end;
	pDRV2624->mnIntStatus = nResult;
	cal_comp = drv2624_reg_read(pDRV2624, DRV2624_REG_CAL_COMP);
	if (cal_comp < 0)
		goto end;
	pDRV2624->mAutoCalResult.mnCalComp = cal_comp;
	cal_bemf = drv2624_reg_read(pDRV2624, DRV2624_REG_CAL_BEMF);
	if (cal_bemf < 0)
		goto end;
	pDRV2624->mAutoCalResult.mnCalBemf = cal_bemf;
	cal_gain =
		drv2624_reg_read(pDRV2624,
				DRV2624_REG_LOOP_CONTROL) & BEMFGAIN_MASK;
	if (cal_gain < 0)
		goto end;
	pDRV2624->mAutoCalResult.mnCalGain = cal_gain;
	/* updata f0*/
	dev_update_f0(pDRV2624);

end:
	dev_dbg(pDRV2624->dev, "%s: nResult = %d\n", __func__, nResult);
	return nResult;
}

static int dev_run_diagnostics(struct drv2624_data *pDRV2624)
{
	int nResult = 0, value = 0;

	dev_info(pDRV2624->dev, "%s\n", __func__);
	nResult = drv2624_change_mode(pDRV2624, DRV2624_DIAG_MODE);
	if (nResult < 0)
		goto end;
	nResult = drv2624_set_go_bit(pDRV2624, GO);
	if (nResult < 0)
		goto end;
	dev_dbg(pDRV2624->dev, "%s: Diag start\n", __func__);
	pDRV2624->mnVibratorPlaying = YES;
	mdelay(2000);
	value = drv2624_reg_read(pDRV2624, DRV2624_REG_STATUS);
	pDRV2624->mnIntStatus_Diag = value;
	if (value < 0) {
		return value;
	}
	drv2624_get_diag_result(pDRV2624, value);

end:
	return nResult;
}

static void drv2624_set_basic_reg(struct drv2624_data *pDRV2624)
{
	drv2624_reg_write(pDRV2624, 0x09, 0x07);
	drv2624_reg_write(pDRV2624, 0xa, 0x92);
	drv2624_reg_write(pDRV2624, 0xb, 0x8d);
	drv2624_reg_write(pDRV2624, 0xd, 0x20);
	drv2624_reg_write(pDRV2624, 0xe, 0x61);		//default gain value
	drv2624_reg_write(pDRV2624, 0x1a, 0x00);
	drv2624_reg_write(pDRV2624, 0x28, 0x11);
	drv2624_reg_write(pDRV2624, 0x29, 0x08);
	drv2624_reg_write(pDRV2624, 0x2b, 0x00);
	drv2624_reg_write(pDRV2624, 0x23, 0x37);
	drv2624_reg_write(pDRV2624, 0x27, 0x13);
}

static void drv2624_set_mode_reg(struct drv2624_data *pDRV2624, char WorkMode)
{
	if (WorkMode == DRV2624_RAM_MODE) {
		drv2624_reg_write(pDRV2624, 0x23, 0x27);
		drv2624_reg_write(pDRV2624, 0x27, 0x93);
	} else {
		drv2624_reg_write(pDRV2624, 0x23, 0x37);
		drv2624_reg_write(pDRV2624, 0x27, 0x13);
	}
}

static void drv2624_init(struct drv2624_data *pDRV2624)
{
	struct drv2624_platform_data *pDrv2624Platdata = &pDRV2624->msPlatData;
	struct actuator_data actuator = pDrv2624Platdata->msActuator;
	unsigned char value_temp = 0;
	unsigned char mask_temp = 0;
	struct drv2624_wave_setting wavesetting;
	unsigned char value = 0;
	drv2624_set_bits(pDRV2624, DRV2624_REG_MODE, PINFUNC_MASK,
			 (PINFUNC_INT << PINFUNC_SHIFT));

	if ((actuator.mnActuatorType == ERM) || (actuator.mnActuatorType ==
						 LRA)) {
		mask_temp |= ACTUATOR_MASK;
		value_temp |= (actuator.mnActuatorType << ACTUATOR_SHIFT);
	}

	if ((pDrv2624Platdata->mnLoop == CLOSE_LOOP)
		|| (pDrv2624Platdata->mnLoop == OPEN_LOOP)) {
		mask_temp |= LOOP_MASK;
		value_temp |= (pDrv2624Platdata->mnLoop << LOOP_SHIFT);
	}
	drv2624_set_bits(pDRV2624, DRV2624_REG_CONTROL1,
			 mask_temp | AUTOBRK_OK_MASK,
			 value_temp | AUTOBRK_OK_ENABLE);
	value_temp = 0;
	if (actuator.mnActuatorType == ERM)
		value_temp = LIB_ERM;
	else if (actuator.mnActuatorType == LRA)
		value_temp = LIB_LRA;

	if (value_temp != 0)
		drv2624_set_bits(pDRV2624, DRV2624_REG_CONTROL2, LIB_MASK,
				 value_temp << LIB_SHIFT);
	if (actuator.mnRatedVoltage != 0)
		drv2624_reg_write(pDRV2624, DRV2624_REG_RATED_VOLTAGE,
				  actuator.mnRatedVoltage);
	else
		dev_err(pDRV2624->dev, "%s, ERROR Rated ZERO\n", __func__);
	if (actuator.mnOverDriveClampVoltage != 0)
		drv2624_reg_write(pDRV2624, DRV2624_REG_OVERDRIVE_CLAMP,
					actuator.mnOverDriveClampVoltage);
	else
		dev_err(pDRV2624->dev, "%s, ERROR OverDriveVol ZERO\n", __func__);

	if (actuator.mnActuatorType == LRA) {
		unsigned char DriveTime =
			5 * (1000 - actuator.mnLRAFreq) / actuator.mnLRAFreq;
		unsigned short openLoopPeriod =
			(unsigned short)((unsigned int)1000000000 /
					(24619 * actuator.mnLRAFreq));
		if (actuator.mnLRAFreq < 125)
			DriveTime |= (MINFREQ_SEL_45HZ << MINFREQ_SEL_SHIFT);
		drv2624_set_bits(pDRV2624, DRV2624_REG_DRIVE_TIME,
				 DRIVE_TIME_MASK | MINFREQ_SEL_MASK, DriveTime);
		drv2624_set_bits(pDRV2624, DRV2624_REG_OL_PERIOD_H, 0x03,
				 (openLoopPeriod & 0x0300) >> 8);
		drv2624_reg_write(pDRV2624, DRV2624_REG_OL_PERIOD_L,
				  (openLoopPeriod & 0x00ff));
		dev_info(pDRV2624->dev, "%s, LRA = %d, DriveTime=0x%x, Shape:sine\n",
			 __func__, actuator.mnLRAFreq, DriveTime);
	}
	drv2624_set_basic_reg(pDRV2624);
	value = drv2624_reg_read(pDRV2624, DRV2624_REG_CONTROL2);
	wavesetting.mnLoop =
	    drv2624_reg_read(pDRV2624, DRV2624_REG_MAIN_LOOP) & 0x07;
	wavesetting.mnInterval = ((value & INTERVAL_MASK) >> INTERVAL_SHIFT);
	wavesetting.mnScale = (value & SCALE_MASK);
	memcpy(&pDRV2624->msWaveformSetting, &wavesetting,
	       sizeof(struct drv2624_wave_setting));
}

static irqreturn_t drv2624_irq_handler(int irq, void *dev_id)
{
	struct drv2624_data *pDRV2624 = (struct drv2624_data *)dev_id;

	dev_dbg(pDRV2624->dev, "%s: enter\n", __func__);
	pDRV2624->mnWorkMode |= WORK_IRQ;
	schedule_work(&pDRV2624->vibrator_work);
	return IRQ_HANDLED;
}

static int drv2624_parse_dt(struct device *dev, struct drv2624_data *pDRV2624)
{
	struct device_node *np = dev->of_node;
	struct drv2624_platform_data *pPlatData = &pDRV2624->msPlatData;
	int rc = 0, nResult = 0;
	unsigned int value;

	pPlatData->mnGpioNRST = of_get_named_gpio(np, "reset-gpio", 0);
	if (pPlatData->mnGpioNRST < 0) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,reset-gpio", np->full_name, pPlatData->mnGpioNRST);
		return -EINVAL;
	} else
		dev_dbg(pDRV2624->dev, "reset-gpio=%d\n",
			pPlatData->mnGpioNRST);
	rc = of_property_read_u32(np, "ti,smart-loop", &value);
	if (rc) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,smart-loop", np->full_name, rc);
		return -EINVAL;
	} else {
		pPlatData->mnLoop = value & 0x01;
		dev_dbg(pDRV2624->dev, "ti,smart-loop=%d\n", pPlatData->mnLoop);
	}
	rc = of_property_read_u32(np, "ti,actuator", &value);
	if (rc) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,actuator", np->full_name, rc);
		return -EINVAL;
	} else {
		pPlatData->msActuator.mnActuatorType = value & 0x01;
		dev_dbg(pDRV2624->dev, "ti,actuator=%d\n",
			pPlatData->msActuator.mnActuatorType);
	}
	rc = of_property_read_u32(np, "ti,rated-voltage", &value);
	if (rc) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,rated-voltage", np->full_name, rc);
		return -EINVAL;
	} else {
		pPlatData->msActuator.mnRatedVoltage =
		    drv2624_calculate_voltage(value);
		dev_dbg(pDRV2624->dev, "ti,rated-voltage=0x%x\n",
			pPlatData->msActuator.mnRatedVoltage);
	}
	rc = of_property_read_u32(np, "ti,odclamp-voltage", &value);
	if (rc) {
		dev_err(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,odclamp-voltage", np->full_name, rc);
		return -EINVAL;
	} else {
		pPlatData->msActuator.mnOverDriveClampVoltage =
		    drv2624_calculate_voltage(value);
		dev_dbg(pDRV2624->dev, "ti,odclamp-voltage=0x%x\n",
			pPlatData->msActuator.mnOverDriveClampVoltage);
	}
	if (pPlatData->msActuator.mnActuatorType == LRA) {
		rc = of_property_read_u32(np, "ti,lra-frequency", &value);
		if (rc) {
			dev_err(pDRV2624->dev,
				"Looking up %s property in node %s failed %d\n",
				"ti,lra-frequency", np->full_name, rc);
			return -EINVAL;
		} else {
			if ((value >= 45) && (value <= 300)) {
				pPlatData->msActuator.mnLRAFreq = value;
				dev_dbg(pDRV2624->dev, "ti,lra-frequency=%d\n",
					pPlatData->msActuator.mnLRAFreq);
			} else {
				dev_err(pDRV2624->dev,
					"ERROR, ti,lra-frequency=%d, out of range\n",
					pPlatData->msActuator.mnLRAFreq);
				return -EINVAL;
			}
		}
	}
	pPlatData->mnGpioINT = of_get_named_gpio(np, "ti,irq-gpio", 0);
	if (pPlatData->mnGpioINT < 0) {
		dev_dbg(pDRV2624->dev,
			"Looking up %s property in node %s failed %d\n",
			"ti,irq-gpio", np->full_name, pPlatData->mnGpioINT);
		//return -EINVAL;
	} else
		dev_dbg(pDRV2624->dev, "ti,irq-gpio=%d\n",
			pPlatData->mnGpioINT);
	return nResult;
}

static int choose_wav_seq(struct drv2624_data *pDRV2624,int control_time)
{
	int nResult = 0;

	if (control_time > 0 && control_time < 20) {
		pDRV2624->msWaveformSequencer.msWaveform[0].mnEffect = 1;
		nResult = drv2624_change_mode(pDRV2624, DRV2624_RAM_MODE);
	} else if (control_time >= 20 && control_time < 30) {
		pDRV2624->msWaveformSequencer.msWaveform[0].mnEffect = 2;
		nResult = drv2624_change_mode(pDRV2624, DRV2624_RAM_MODE);
	} else if (control_time >= 30 && control_time < 60) {
		pDRV2624->msWaveformSequencer.msWaveform[0].mnEffect = 3;
		nResult = drv2624_change_mode(pDRV2624, DRV2624_RAM_MODE);
	} else {
		pDRV2624->play.length = control_time;
		nResult = drv2624_change_mode(pDRV2624, DRV2624_RTP_MODE);
		if (nResult < 0) {
			pr_err("%s: nResult = %d\n", __func__, nResult);
		}
	}

	//pr_info("%s:  mnEffect = %d\n", __func__, pDRV2624->msWaveformSequencer.msWaveform[0].mnEffect);
	return nResult;
}
/* *
 * vibrator_save_cali_data()
 *
 * save calibration data to
 * /persist/DRV2624_DATA.txt
 *
 * vibrator_read_cali_dta()
 *
 * read the file to set
 * CalComp,mnCalBemf,mnCalGain
 * f0_data_msb,f0_data_lsb
 *
 * */
int vibrator_save_cali_data(struct drv2624_data *pDRV2624)
{
	unsigned char buf[5] = {0};

    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos;

	buf[0] = pDRV2624->mAutoCalResult.mnCalComp;
	buf[1] = pDRV2624->mAutoCalResult.mnCalBemf;
	buf[2] = pDRV2624->mAutoCalResult.mnCalGain;
	buf[3] = pDRV2624->mAutoCalResult.f0_data_msb;
	buf[4] = pDRV2624->mAutoCalResult.f0_data_lsb;

    pr_err("%s enter! %s 0x%x 0x%x 0x%x 0x%x 0x%x\n",__func__,
			DRV2624_PATH, buf[0], buf[1], buf[2],
			buf[3],buf[4]);

    fp = filp_open(DRV2624_PATH, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(fp)){
		pr_err("%s:create file error \n",__func__);
		return -1;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(fp, buf, sizeof(buf), &pos);
    filp_close(fp,NULL);
    set_fs(fs);
    return 0;
}

int vibrator_read_cali_data(struct drv2624_data *pDRV2624)
{
	unsigned char buf[5] = {0};
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos;
	pos = 0;

    fp = filp_open(DRV2624_PATH, O_RDONLY, 0);
    if (IS_ERR(fp)){
		pr_err("%s: read filed\n", __func__);
		return -1;
    }

	fs = get_fs();
    set_fs(KERNEL_DS);
    pr_info("%s enter! path = %s\n",__func__, DRV2624_PATH);
    vfs_read(fp, buf, sizeof(buf), &pos);
    pr_info("%s: read: 0x%x 0x%x 0x%x 0x%x 0x%x\n", __func__,
			buf[0], buf[1], buf[2], buf[3], buf[4]);
    filp_close(fp,NULL);
    set_fs(fs);
	pDRV2624->mAutoCalResult.mnCalComp = buf[0];
	pDRV2624->mAutoCalResult.mnCalBemf = buf[1];
	pDRV2624->mAutoCalResult.mnCalGain = buf[2];
	pDRV2624->mAutoCalResult.f0_data_msb = buf[3];
	pDRV2624->mAutoCalResult.f0_data_lsb = buf[4];

    return 0;
}

static int drv2624_calibration_fetch(struct drv2624_data *pDRV2624)
{
	dev_dbg(pDRV2624->dev, "%s: enter!\n", __func__);
	drv2624_reg_write(pDRV2624, DRV2624_REG_CAL_COMP,
				pDRV2624->mAutoCalResult.mnCalComp);
	drv2624_reg_write(pDRV2624, DRV2624_REG_CAL_BEMF,
				pDRV2624->mAutoCalResult.mnCalBemf);
	drv2624_set_bits(pDRV2624, DRV2624_REG_LOOP_CONTROL, BEMFGAIN_MASK,
				pDRV2624->mAutoCalResult.mnCalGain);
	drv2624_reg_write(pDRV2624, DRV2624_REG_OL_PERIOD_H, pDRV2624->mAutoCalResult.f0_data_msb);
	drv2624_reg_write(pDRV2624, DRV2624_REG_OL_PERIOD_L, pDRV2624->mAutoCalResult.f0_data_lsb);
	return 0;
}
/* *
 * add led device nodes
 * state
 * activate
 * duration
 * gain
 * cali
 * save_cali
 *
 * */
static void android_vibrator_enable(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	pr_err("%s:enter!\n", __func__);
}

static ssize_t gain_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	int nResult = 0;

	pr_info("%s:enter!\n", __func__);
	nResult = drv2624_reg_read(pDRV2624, DRV2624_REG_RTP_INPUT);
	pr_info("%s:gain value = %d\n", __func__, nResult);
	return snprintf(buf, 30, "gain value = %d\n", nResult);
}
static ssize_t gain_store(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
    int value = 0;
	int rc;

	rc = kstrtouint(buf, 0, &value);
	dev_dbg(pDRV2624->dev, "%s: value = %d\n",__func__, value);
	if (rc < 0) {
		pr_err("%s: kstrtouint filed\n", __func__);
		return count;
	} else {
		if (value > 0 && value <= 127)
			drv2624_reg_write(pDRV2624, DRV2624_REG_RTP_INPUT, value);
	}
		return count;
}
static ssize_t state_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	int nResult;

	nResult = drv2624_reg_read(pDRV2624, 0x0c);
	pr_err("%s:enter!\n", __func__);
	return snprintf(buf, 30, "haptics state = %d\n", nResult);
}
static ssize_t state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	pr_err("%s:enter!\n", __func__);
	return count;
}
static ssize_t activate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);

	drv2624_stop(pDRV2624);
	pr_err("%s:enter!\n", __func__);
	return 0;
}
static ssize_t activate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	dev_dbg(pDRV2624->dev,"%s:enter val = %d mnWorkMode %d\n", __func__, val, pDRV2624->mnWorkMode);

	mutex_lock(&pDRV2624->lock);
	if (1 == val) {
		if (pDRV2624->mnWorkMode == DRV2624_RTP_MODE) {
			if (hrtimer_active(&pDRV2624->haptics_timer))
				hrtimer_cancel(&pDRV2624->haptics_timer);
			schedule_work(&pDRV2624->haptics_playback_work);
		} else if (pDRV2624->mnWorkMode == DRV2624_RAM_MODE) {
			schedule_work(&pDRV2624->haptics_playback_work);
		}
	} else {
		hrtimer_cancel(&pDRV2624->haptics_timer);
		drv2624_stop(pDRV2624);
	}
	mutex_unlock(&pDRV2624->lock);

	return count;
}

static ssize_t duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	int nResult  = 0;

	nResult = pDRV2624->play.length;
	pr_err("%s:enter! play_length = %d\n", __func__, nResult);
	return snprintf(buf, 30, "duration value = %d\n", nResult);
}

static ssize_t duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	int rc = 0;
	int value;

	mutex_lock(&pDRV2624->lock);
	rc = kstrtouint(buf, 0, &value);
	if(rc < 0) {
		mutex_unlock(&pDRV2624->lock);
		pr_err("%s: kstrtouint filed\n", __func__);
		return rc;
	}
	if (value > MAX_TIMEOUT)
		value = MAX_TIMEOUT;

	if (value < 0){
		mutex_unlock(&pDRV2624->lock);
		pr_err("%s: duration data error\n", __func__);
		return count;
	} else {
		choose_wav_seq(pDRV2624, value);
	}
	mutex_unlock(&pDRV2624->lock);
	dev_dbg(pDRV2624->dev,"%s:enter! duration value = %d\n", __func__, value);

	return count;
}

static ssize_t name_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;

	len += snprintf(buf+len, PAGE_SIZE-len, "DRV2624\n");
	return len;
}

/*show calibrtion*/
static ssize_t cali_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	dev_err(pDRV2624->dev,
		"%s: Cal_Result: 0x%x, CalComp: 0x%x, CalBemf: 0x%x,\
		CalGain: 0x%x, f0_data_msb:0x%x,f0_data_lsb:0x%x\n",
		__func__, pDRV2624->mnIntStatus,
		pDRV2624->mAutoCalResult.mnCalComp,
		pDRV2624->mAutoCalResult.mnCalBemf,
		pDRV2624->mAutoCalResult.mnCalGain,
		pDRV2624->mAutoCalResult.f0_data_msb,
		pDRV2624->mAutoCalResult.f0_data_lsb);
# if 0
	nResult = drv2624_get_calibration_result(pDRV2624);
	if (nResult < 0)
		goto end;


	return snprintf(buf, 100, "%x %x %x %x %x %x\n",
			pDRV2624->mnIntStatus,
			pDRV2624->mAutoCalResult.mnCalComp,
			pDRV2624->mAutoCalResult.mnCalBemf,
			pDRV2624->mAutoCalResult.mnCalGain,
			pDRV2624->mAutoCalResult.f0_data_msb,
			pDRV2624->mAutoCalResult.f0_data_lsb);
#endif

	if(pDRV2624->mnIntStatus == 8)
		return snprintf(buf, 50, "PASS cali_status = %d\n", pDRV2624->mnIntStatus);
	else if (pDRV2624->mnIntStatus_Diag == 8)
		return snprintf(buf, 50, "PASS diag_status = %d\n", pDRV2624->mnIntStatus_Diag);
	else
		return snprintf(buf, 100, "FAILED cali_status = %d,diag_status = %d\n",
			pDRV2624->mnIntStatus, pDRV2624->mnIntStatus_Diag);
}
/**
 * store calibration
 *
 **/
static ssize_t cali_store (
		struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	int nResult = 0;

	if ((buf[0] == 'C') && (buf[1] == 'A') &&
			(buf[2] == 'L') && (buf[3] == 'I')) {
		nResult = dev_auto_calibrate(pDRV2624);
		if (nResult<0) {
			pr_err("%s: run calibrate err nResult=%d\n",__func__, nResult);
			return count;
		}
		nResult = drv2624_get_calibration_result(pDRV2624);
		if (nResult<0) {
			pr_err("%s: get calibration result err nResult=%d\n",__func__, nResult);
			return count;
		}
		nResult = vibrator_save_cali_data(pDRV2624);
		if (nResult<0) {
			pr_err("%s: save cali data err nResult=%d\n",__func__, nResult);
			return count;
		}
	} else if ((buf[0] == 'D') && (buf[1] == 'I')
			&&(buf[2] == 'A') && (buf[3] == 'G')) {
		nResult = dev_run_diagnostics(pDRV2624);
		if (nResult<0) {
			pr_err("%s: run diag err nResult=%d\n",__func__, nResult);
			return count;
		}
	}
	return count;
}

static ssize_t cali_save_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	int nResult = 0;

	pr_err("drv2624 %s: enter!\n" , __func__);
	nResult = vibrator_read_cali_data(pDRV2624);
	if (nResult<0) {
		pr_err("%s: read cali data err nResult=%d\n",__func__, nResult);
		return count;
	}
	nResult = drv2624_calibration_fetch(pDRV2624);
	if (nResult<0) {
		pr_err("%s: calibration fetch err nResult=%d\n",__func__, nResult);
		return count;
	}
	return count;
}

static ssize_t
cali_save_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct drv2624_data *pDRV2624 = container_of(led_cdev, struct drv2624_data, led_dev);
	int nResult;
	ssize_t len = 0;

	nResult = vibrator_read_cali_data(pDRV2624);
	if (nResult<0) {
		len += snprintf(buf+len, PAGE_SIZE-len, "read cali data err nResult=%d\n", nResult);
		return len;
	}
	nResult = drv2624_calibration_fetch(pDRV2624);
	if (nResult<0) {
		len += snprintf(buf+len, PAGE_SIZE-len, "calibration fetch err nResult=%d\n", nResult);
		return len;
	}

	dev_err(pDRV2624->dev,
		"%s: CalComp: 0x%x, CalBemf: 0x%x,\
		CalGain: 0x%x, f0_data_msb:0x%x,f0_data_lsb:0x%x\n",
		__func__, pDRV2624->mAutoCalResult.mnCalComp,
		pDRV2624->mAutoCalResult.mnCalBemf,
		pDRV2624->mAutoCalResult.mnCalGain,
		pDRV2624->mAutoCalResult.f0_data_msb,
		pDRV2624->mAutoCalResult.f0_data_lsb);

	return snprintf(buf, 100, "%x %x %x %x %x\n",
			pDRV2624->mAutoCalResult.mnCalComp,
			pDRV2624->mAutoCalResult.mnCalBemf,
			pDRV2624->mAutoCalResult.mnCalGain,
			pDRV2624->mAutoCalResult.f0_data_msb,
			pDRV2624->mAutoCalResult.f0_data_lsb);
}
static DEVICE_ATTR(state, 0644, state_show, state_store);
static DEVICE_ATTR(activate, 0644, activate_show, activate_store);
static DEVICE_ATTR(duration, 0644, duration_show, duration_store);
static DEVICE_ATTR(cali, 0644, cali_show, cali_store);
static DEVICE_ATTR(name, 0644, name_show, NULL);
static DEVICE_ATTR(gain, 0644, gain_show, gain_store);
static DEVICE_ATTR(cali_lra, 0644, cali_save_show, cali_save_store);
static struct attribute *android_led_dev_fs_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_activate.attr,
	&dev_attr_duration.attr,
	&dev_attr_cali.attr,
	&dev_attr_name.attr,
	&dev_attr_gain.attr,
	&dev_attr_cali_lra.attr,
	NULL,
};
static struct attribute_group android_led_dev_fs_attr_group = {
	.attrs = android_led_dev_fs_attrs,
};

static const struct attribute_group *android_led_dev_fs_attr_groups[] = {
	&android_led_dev_fs_attr_group,
	NULL,
};

static int android_hal_stub_init(struct drv2624_data *pDRV2624)
{
	int ret;

    pDRV2624->led_dev.name = "vibrator";
    pDRV2624->led_dev.max_brightness = LED_FULL;
    pDRV2624->led_dev.brightness_set = android_vibrator_enable;
    pDRV2624->led_dev.flags = LED_BRIGHTNESS_FAST;
    pDRV2624->led_dev.groups = android_led_dev_fs_attr_groups;
	pDRV2624->play.length= MAX_TIMEOUT;
	INIT_WORK(&pDRV2624->haptics_playback_work,
			haptics_playback_work_routine);
	INIT_WORK(&pDRV2624->vibrator_work, vibrator_work_routine);
	hrtimer_init(&pDRV2624->haptics_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	pDRV2624->haptics_timer.function = vibrator_timer_func;
    ret = led_classdev_register(&pDRV2624->client->dev, &pDRV2624->led_dev);
    if (ret) {
    	pr_err("Failed to create led classdev: %d\n", ret);
		return ret;
	}
	return 0;
}
/**
 * Rated and Overdriver Voltages:
 * Calculated using the formula r = voltage(V) * 255 / 5.6
 * where r is what will be written to the register
 * and v is the rated or overdriver voltage of the actuator
 **/
static inline int drv2624_calculate_voltage(unsigned int voltage)
{
	return (voltage * 255 / 5600);
}

static int drv2624_hw_reset(struct drv2624_data *pDRV2624)
{
	int nResult = 0;

	dev_dbg(pDRV2624->dev, "%s: enter! \n", __func__);
	gpio_direction_output(pDRV2624->msPlatData.mnGpioNRST, 0);
	mdelay(5);
	gpio_direction_output(pDRV2624->msPlatData.mnGpioNRST, 1);
	mdelay(2);
	return nResult;
}

static int drv2624_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int nResult = 0;
	struct drv2624_data *pDRV2624;

	dev_info(&client->dev, "%s enter\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s:I2C check failed\n", __func__);
		return -ENODEV;
	}
	pDRV2624 =
	    devm_kzalloc(&client->dev, sizeof(struct drv2624_data), GFP_KERNEL);
	if (pDRV2624 == NULL) {
		dev_err(&client->dev, "%s:no memory\n", __func__);
		return -ENOMEM;
	}
	pDRV2624->dev = &client->dev;
	pDRV2624->client = client;
	i2c_set_clientdata(client, pDRV2624);
	dev_set_drvdata(&client->dev, pDRV2624);
	pDRV2624->mpRegmap = devm_regmap_init_i2c(client, &drv2624_i2c_regmap);
	if (IS_ERR(pDRV2624->mpRegmap)) {
		nResult = PTR_ERR(pDRV2624->mpRegmap);
		dev_err(pDRV2624->dev, "%s:Failed to allocate register map: %d\n",
			__func__, nResult);
		goto free_mem;
	}
	if (client->dev.of_node) {
		dev_dbg(pDRV2624->dev, "of node parse\n");
		nResult = drv2624_parse_dt(&client->dev, pDRV2624);
		if (nResult < 0) {
			dev_err(pDRV2624->dev, "%s: parse_dt failed %d\n",
				__func__, nResult);
			goto free_gpio;
		}
	} else if (client->dev.platform_data) {
		dev_dbg(pDRV2624->dev, "platform data parse\n");
		memcpy(&pDRV2624->msPlatData, client->dev.platform_data,
			sizeof(struct drv2624_platform_data));
	} else {
		dev_err(pDRV2624->dev, "%s: ERROR no platform data\n",
			__func__);
		goto free_gpio;
	}

	if (gpio_is_valid(pDRV2624->msPlatData.mnGpioNRST)) {
		nResult =
		    gpio_request(pDRV2624->msPlatData.mnGpioNRST,
				 "DRV2624-NRST");
		if (nResult < 0) {
			dev_err(pDRV2624->dev,
				"%s: GPIO %d request NRST error\n", __func__,
				pDRV2624->msPlatData.mnGpioNRST);
			goto free_gpio;
		}
		drv2624_hw_reset(pDRV2624);
	}
	mutex_init(&pDRV2624->dev_lock);
	mutex_init(&pDRV2624->lock);
	nResult = drv2624_reg_read(pDRV2624, DRV2624_REG_ID);
	if (nResult != 0x3) {
		goto exit_gpio_request_failed1;
	} else {
		dev_info(pDRV2624->dev, "%s, ID status (0x%x)\n", __func__, nResult);
		pDRV2624->mnDeviceID = nResult;
	}
	if ((pDRV2624->mnDeviceID & 0xf0) != DRV2624_ID) {
		dev_err(pDRV2624->dev, "%s, device_id(0x%x) fail\n", __func__,
			pDRV2624->mnDeviceID);
		goto destroy_mutex;
	}
	drv2624_init(pDRV2624);
	if (gpio_is_valid(pDRV2624->msPlatData.mnGpioINT)) {
		nResult = gpio_request(pDRV2624->msPlatData.mnGpioINT, "DRV2624-IRQ");
		if (nResult < 0) {
			dev_err(pDRV2624->dev,
				"%s: GPIO %d request INT error\n", __func__,
				pDRV2624->msPlatData.mnGpioINT);
			goto destroy_mutex;
		}
		gpio_direction_input(pDRV2624->msPlatData.mnGpioINT);
		pDRV2624->mnIRQ = gpio_to_irq(pDRV2624->msPlatData.mnGpioINT);
		dev_dbg(pDRV2624->dev, "irq = %d \n", pDRV2624->mnIRQ);
		nResult =
			request_threaded_irq(pDRV2624->mnIRQ, drv2624_irq_handler,
					NULL,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					client->name, pDRV2624);
		if (nResult < 0) {
			dev_err(pDRV2624->dev, "request_irq failed, %d\n", nResult);
			goto destroy_mutex;
		}
		disable_irq_nosync(pDRV2624->mnIRQ);
		pDRV2624->mbIRQEnabled = false;
		pDRV2624->mbIRQUsed = true;
	} else
		pDRV2624->mbIRQUsed = false;

	nResult = android_hal_stub_init(pDRV2624);
	nResult = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
						"drv2624.bin", &(client->dev),
						GFP_KERNEL, pDRV2624,
						drv2624_firmware_load);
	if (nResult != 0) {
		dev_err(&client->dev,
			"%s: %u: nResult = %d:request drv2624_firmware_laod!\n",
			__func__, __LINE__, nResult);
	}

	//register hw info
	hq_register_hw_info(HWID_VIBRATOR, "DRV2624");

	dev_info(pDRV2624->dev, "drv2624 probe succeeded\n");
	return 0;
destroy_mutex:mutex_destroy(&pDRV2624->dev_lock);
free_gpio:
	if (gpio_is_valid(pDRV2624->msPlatData.mnGpioINT))
		gpio_free(pDRV2624->msPlatData.mnGpioINT);
	if (gpio_is_valid(pDRV2624->msPlatData.mnGpioNRST))
		gpio_free(pDRV2624->msPlatData.mnGpioNRST);
free_mem:
	if (NULL != pDRV2624)
		kfree(pDRV2624);
exit_gpio_request_failed1:if (gpio_is_valid
	    (pDRV2624->msPlatData.mnGpioNRST))
		gpio_free(pDRV2624->msPlatData.mnGpioNRST);
	mutex_destroy(&pDRV2624->dev_lock);
	return nResult;
}

static int drv2624_i2c_remove(struct i2c_client *client)
{
	struct drv2624_data *pDRV2624 = i2c_get_clientdata(client);

	if (pDRV2624->msPlatData.mnGpioNRST)
		gpio_free(pDRV2624->msPlatData.mnGpioNRST);
	if (pDRV2624->msPlatData.mnGpioINT)
		gpio_free(pDRV2624->msPlatData.mnGpioINT);
	mutex_destroy(&pDRV2624->lock);
	mutex_destroy(&pDRV2624->dev_lock);
	return 0;
}
static const struct i2c_device_id drv2624_i2c_id[] = {
	{"drv2624", 0},
	{}
};

#ifdef CONFIG_PM_SLEEP
static int __maybe_unused drv2624_suspend(struct device *dev)
{
	struct drv2624_data *pDRV2624 = dev_get_drvdata(dev);
	dev_dbg(pDRV2624->dev, "%s enter!\n", __func__);
	mutex_lock(&pDRV2624->lock);
	if (hrtimer_active(&pDRV2624->haptics_timer)
	    || pDRV2624->mnVibratorPlaying) {
		drv2624_stop(pDRV2624);
	}

	/* set device to standby mode */
	drv2624_set_bits(pDRV2624, DRV2624_REG_CONTROL1,
			 DRV2624_AUTO_BRK_INTO_STBY_MASK,
			 DRV2624_REMOVE_STBY_MODE);
	mutex_unlock(&pDRV2624->lock);
	return 0;
}

static int __maybe_unused drv2624_resume(struct device *dev)
{
	struct drv2624_data *pDRV2624 = dev_get_drvdata(dev);

	dev_dbg(pDRV2624->dev, "%s enter!\n", __func__);
	mutex_lock(&pDRV2624->lock);

	drv2624_set_bits(pDRV2624, DRV2624_REG_CONTROL1,
			 DRV2624_AUTO_BRK_INTO_STBY_MASK,
			 DRV2624_STBY_MODE_WITH_AUTO_BRAKE);
	mutex_unlock(&pDRV2624->lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(drv2624_pm_ops, drv2624_suspend, drv2624_resume);

#endif /*  */
MODULE_DEVICE_TABLE(i2c, drv2624_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id drv2624_of_match[] = {
	{.compatible = "ti,drv2624"},
	{},
};

MODULE_DEVICE_TABLE(of, drv2624_of_match);

#endif /*  */
static struct i2c_driver drv2624_i2c_driver = {
	.driver = {
		   .name = "drv2624",
		   .owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		   .pm = &drv2624_pm_ops,
#endif /*  */
#if defined(CONFIG_OF)
		   .of_match_table = of_match_ptr(drv2624_of_match),

#endif /*  */
		   },
	.probe = drv2624_i2c_probe,
	.remove = drv2624_i2c_remove,
	.id_table = drv2624_i2c_id,
};

module_i2c_driver(drv2624_i2c_driver);
MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("DRV2624 I2C Smart Haptics driver");
MODULE_LICENSE("GPL");