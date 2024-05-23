// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2021 Silicon Solution ISRAEL LTD
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation files (the
 * “Software”), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons
 * to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 **************************************************************************/

#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/if_vlan.h>
#include <linux/version.h>
#include <linux/bitrev.h>

/*! The default network device name used */
#define LMS_DEFAULT_NETDEV_NAME_PREFIX	"seth"

/*! SPI clock speed */
#define LMS_SPI_CLK_SPEED_MIN 250000
#define LMS_SPI_CLK_SPEED_MAX 50000000
#define LMS_SPI_DEFAULT_CLK_SPEED 8000000

/*! module param to override device tree SPI clock speed */
static int lms_spi_clkspeed;
module_param(lms_spi_clkspeed, int, 0);
MODULE_PARM_DESC(lms_spi_clkspeed, "Override SPI bus clock speed (Hz).");

/*! module param to override idle CG5317 polling timeout */
static int lms_spi_idle_poll_ms = 200;
module_param(lms_spi_idle_poll_ms, int, 0);
MODULE_PARM_DESC(lms_spi_idle_poll_ms, "Override idle SPI CG5317 polling timeout (ms). Default is 200. Use 0 to disable idle polling.");

/*! module param to override SPI lsb first convertion in HW */
static bool lms_force_spi_lsb_first_in_sw = false;
module_param(lms_force_spi_lsb_first_in_sw, bool, 0);
MODULE_PARM_DESC(lms_force_spi_lsb_first_in_sw, "Force lsb first over SPI to happen in SW even if HW supports it.");

static DEFINE_MUTEX(lms_global_lock);
static struct device *lms_root_device;
static u32 lms_probe_counter;

/*! tx timeout */
#define LMS_SPI_TX_TIMEOUT (1 * HZ)

#define LMS_READ_ONLY_PERMISSION 0440
#define LMS_WRITE_ONLY_PERMISSION 0220
#define LMS_READ_WRITE_PERMISSION 0660

/*! Driver name and version */
#define LMS_DRV_NAME "lms_eth2spi"
#define LMS_SPI_DRV_VERSION "0.0.8"

/*! Minimal and maximal MTU */
#define LMS_FRM_MIN_MTU (ETH_ZLEN - ETH_HLEN)
#define LMS_FRM_MAX_MTU ETH_DATA_LEN

#define LMS_MIN_PACKET_LEN             (LMS_FRM_MIN_MTU + ETH_HLEN)
#define LMS_MAX_PACKET_LEN             (LMS_FRM_MAX_MTU + VLAN_ETH_HLEN)

#define LMS_BYTES_IN_DWORDS (sizeof(u32) / sizeof(u8))
#define LMS_REG_ONE_GET_OCCUPANCY_IN_BYTES(x) ((u16)(((x) & 0xFFFF) * LMS_BYTES_IN_DWORDS))
#define LMS_REG_TWO_GET_NUM_PACKETS_IN_FIFO(x) ((u8)(((x) & 0xFF0000) >> 16))

/* Give priority to RX */
#define LMS_NUM_TX_BEFORE_RX	(4)
#define LMS_NUM_RX_BEFORE_TX	(12)

/* Registers numberings */
/*! The main status register, with a lot of flags */
#define LMS_READ_STAT_REG_ZERO 16
/*! The TX occupancy register */
#define LMS_READ_STAT_REG_ONE 17
/*! The RX occupancy register */
#define LMS_READ_STAT_REG_TWO 18
#define LMS_READ_STAT_REG_THREE 19

#define LMS_CG2H_MAILBOX_REG_ZERO 20
#define LMS_CG2H_MAILBOX_REG_ONE 21
#define LMS_CG2H_MAILBOX_REG_TWO 22
#define LMS_CG2H_MAILBOX_REG_THREE 23

#define LMS_H2CG_MAILBOX_REG_ZERO 24
#define LMS_H2CG_MAILBOX_REG_ONE 25
#define LMS_H2CG_MAILBOX_REG_TWO 26
#define LMS_H2CG_MAILBOX_REG_THREE 27

#define LMS_OUI_0 0x00 /*!< Lumissil OUI */
#define LMS_OUI_1 0x16 /*!< Lumissil OUI */
#define LMS_OUI_2 0xe8 /*!< Lumissil OUI */

/*! The TX FIFO occupancy size
 * Note: The real HW TX_FIFO is slightly larger, this value defines the
 *       Flow Control limit of the SPI driver.
 */
#define LMS_TX_MAX_OCCUPANCY_BYTES 12000
/*! Max number of bytes added to TX FIFO for each packet in CG BootROM state */
#define LMS_BOOTROM_MAX_OCCUPANCY_ADDITION_PER_PACKET 4
/*! Max number of bytes added to TX FIFO for each packet in CG FW state */
#define LMS_FW_MAX_OCCUPANCY_ADDITION_PER_PACKET 8


/*! rx irq name */
#define LMS_RX_IRQ_NAME "rx_irq"

/*! status0 irq name */
#define LMS_STAT0_IRQ_NAME "gp_irq"

/* Status0 register flags */
#define LMS_CG2H_MAIL_ZERO 0
#define LMS_CG2H_MAIL_ONE 1
#define LMS_CG2H_MAIL_TWO 2
#define LMS_CG2H_MAIL_THREE 3
#define LMS_H2CG_MAIL_ZERO 4
#define LMS_H2CG_MAIL_ONE 5
#define LMS_H2CG_MAIL_TWO 6
#define LMS_H2CG_MAIL_THREE 7
#define LMS_FLOW_CONTROL 8
#define LMS_OVERFLOW_TX 9
#define LMS_OVERFLOW_RX 10
#define LMS_EXCEPTION_ONE 11
#define LMS_EXCEPTION_TWO 12
#define LMS_FIRST_CONST_ZERO 13
#define LMS_LAST_CONST_ZERO 27
#define LMS_SANITY_ONE 28
#define LMS_SANITY_TWO 29
#define LMS_SANITY_THREE 30
#define LMS_SANITY_FOUR 31

#define LMS_BUSY_WAIT_SLEEP_LEN 5
#define LMS_MAX_BUSY_WAIT_SLEEP_TIMES 100
#define LMS_TX_OCCU_FIFO_MAX_RETRY_COUNT 400

#define LMS_FAST_RX_COMPLETION_THRESHOLD (4)

#define LMS_MAX_WAIT_STATUS_ZERO_MS 1400
#define LMS_STATUS_ZERO_CHECK_RETRY_DELAY_MS 50
#define LMS_STATUS_ZERO_CHECK_MAX_RETRIES (LMS_MAX_WAIT_STATUS_ZERO_MS / LMS_STATUS_ZERO_CHECK_RETRY_DELAY_MS)
#define LMS_WAIT_CHECK_STATUS_ZERO 100

/* Commands */
#define LMS_READ_SIZE_COMMAND 8
#define LMS_READ_DATA_COMMAND 12
#define LMS_WRITE_SIZE_COMMAND 2
#define LMS_WRITE_DATA_COMMAND 3

#define LMS_MAX_HEADER_ETH_LEN 36
#define LMS_MAX_RX_BUFF_LEN (ETH_DATA_LEN + LMS_MAX_HEADER_ETH_LEN)

/*! Size of the internal TX queue */
#define LMS_TX_QUEUE_SIZE 1000
/*! Size of the internal RX FIFO */
#define LMS_RX_FIFO_SIZE (0x4000)

struct lms_tx_queue {
	struct sk_buff *array[LMS_TX_QUEUE_SIZE];
	u16 head;
	u16 tail;
	atomic_t count;
};

struct lms_CG2H_mailbox {
	u32 val;
	time64_t last_update_time;
	/* The lock is meant to synchronize access to the mailbox */
	struct mutex lock;
};

struct lms_get_CG2H_mailbox {
	u32 val;
	time64_t last_update_time;
};

struct lms_H2CG_user_write {
	atomic_t flag;
	atomic_t val;
};

/*! struct used to pass lms_spi struct pointer
 *  to sysfs handling functions
 */
struct lms_attr_wrapper {
	struct kobj_attribute dev_attr;
	struct lms_spi *lms;
};

enum lms_spi_state {
	SPI_OK,
	SPI_DISABLED,
	SPI_ERR,
};

#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
/* When this option is used, this driver will combine spi messages that use
 * a tx and then rx transfer into a single spi transfer.
 * Meaning that if an SPI message is of the following: send X bytes -> read Y bytes response,
 * then the driver will create an SPI transfer with a buffer of size X + Y and send it in a
 * 'full duplex' mode.
 * transfer.len = X + Y
 * transfer.rx_buf =  -------------------------------------------
 * transfer.tx_buf =  | X bytes = cmd | Y bytes = uninitialized |
 *                    -------------------------------------------
 * after sending the message, the response will be at offset Y inside the buffer.
 *
 * The rational: when sending an SPI message built from multiple SPI transfers to the SPI
 * subsystem, there will be small breaks in the clock between every transfer.
 * In order to slightly increase the throughput of the driver this option should be considered.
 *
 * **note** : The SPI controller used by the SPI subsystem must support full duplex transfers.
 */
struct lms_spi_tx_rx_frame {
	u8 cmd;
	u8 buf[LMS_MAX_RX_BUFF_LEN];
} __packed;
#endif

/*! The main struct of the lms_eth2spi driver */
struct lms_spi {
	u32 probe_idx;
	struct net_device *net_dev;
	struct spi_device *spi_dev;
	struct task_struct *spi_thread;
	/*! A flag that indicates if spi is available or not */
	enum lms_spi_state spi_state;

	/*! Our TX queue */
	struct lms_tx_queue queue;
	/*! A buffer we use to save spi output */
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	struct lms_spi_tx_rx_frame *fdplx_rx;
	struct lms_spi_tx_rx_frame *fdplx_tx;
#else
	u8 *spi_buffer;
#endif
	struct workqueue_struct	*rx_completion_wq;
	struct work_struct rx_completion_work;
	struct kfifo rx_completion_fifo;
	bool sw_lsb_first;

	atomic_t rx_intr_req;
	atomic_t stat_intr_req;
	atomic_t spi_modem_status;
	u16 tx_occupancy;
	u8 rx_packets_waiting;
	int stat_gpio;
	int rx_gpio;
	int stat_irq;
	int rx_irq;

	atomic_t H2CG_status_mailbox_zero;
	atomic_t H2CG_status_mailbox_one;
	atomic_t H2CG_status_mailbox_two;
	atomic_t H2CG_status_mailbox_three;

	struct lms_H2CG_user_write H2CG_user_write_mailbox_zero;
	struct lms_H2CG_user_write H2CG_user_write_mailbox_one;
	struct lms_H2CG_user_write H2CG_user_write_mailbox_two;
	struct lms_H2CG_user_write H2CG_user_write_mailbox_three;

	atomic_t overflow_counter_tx;
	atomic_t overflow_counter_rx;
	atomic_t unexpected_behaviour_counter;

	struct lms_CG2H_mailbox CG2H_mailbox_zero;
	struct lms_CG2H_mailbox CG2H_mailbox_one;
	struct lms_CG2H_mailbox CG2H_mailbox_two;
	struct lms_CG2H_mailbox CG2H_mailbox_three;

	struct lms_attr_wrapper cg2h_read_zero_attr_wrapper;
	struct lms_attr_wrapper cg2h_timestamp_zero_attr_wrapper;
	struct lms_attr_wrapper cg2h_read_one_attr_wrapper;
	struct lms_attr_wrapper cg2h_timestamp_one_attr_wrapper;
	struct lms_attr_wrapper cg2h_read_two_attr_wrapper;
	struct lms_attr_wrapper cg2h_timestamp_two_attr_wrapper;
	struct lms_attr_wrapper cg2h_read_three_attr_wrapper;
	struct lms_attr_wrapper cg2h_timestamp_three_attr_wrapper;

	struct lms_attr_wrapper h2cg_write_zero_attr_wrapper;
	struct lms_attr_wrapper h2cg_status_zero_attr_wrapper;
	struct lms_attr_wrapper h2cg_write_one_attr_wrapper;
	struct lms_attr_wrapper h2cg_status_one_attr_wrapper;
	struct lms_attr_wrapper h2cg_write_two_attr_wrapper;
	struct lms_attr_wrapper h2cg_status_two_attr_wrapper;
	struct lms_attr_wrapper h2cg_write_three_attr_wrapper;
	struct lms_attr_wrapper h2cg_status_three_attr_wrapper;

	struct lms_attr_wrapper statistics_wrapper;
	struct lms_attr_wrapper modem_status_wrapper;
	struct lms_attr_wrapper legacy_modem_status_wrapper;
	bool legacy_modem_status_owner;

	struct kobject *lms_eth2spi;
	struct kobject *mailboxes;
	struct kobject *h2cg_mailboxes;
	struct kobject *cg2h_mailboxes;
	struct kobject *mailbox_h2cg_0;
	struct kobject *mailbox_h2cg_1;
	struct kobject *mailbox_h2cg_2;
	struct kobject *mailbox_h2cg_3;
	struct kobject *mailbox_cg2h_0;
	struct kobject *mailbox_cg2h_1;
	struct kobject *mailbox_cg2h_2;
	struct kobject *mailbox_cg2h_3;
};

static void bitrev8_buf(u8 *buf, size_t size)
{
	u32 *start;
	u32 *end;

	while ((((uintptr_t)buf & 0x03) != 0) && size) {
		*buf = __constant_bitrev8(*buf);
		buf++;
		size--;
	}

	if (!size)
		return;

	start = (u32*)buf;
	end = (u32 *)(buf + (size & (~0x03)));
	while (start < end) {
		*start = bitrev8x4(*start);
		start++;
	}

	size &= 0x03;
	if (!size)
		return;

	buf = (u8 *)start;
	while (size) {
		*buf = __constant_bitrev8(*buf);
		buf++;
		size--;
	}

	/*
	u32 i;
	for (i = 0; i < size; i++)
	{
		buf[i] = __constant_bitrev8(buf[i]);
	}
	*/
}

static int
lms_tx_enqueue(struct lms_tx_queue *queue, struct sk_buff *skb)
{
	if (atomic_read(&queue->count) == LMS_TX_QUEUE_SIZE)
		return -1;

	queue->array[queue->tail] = skb;
	queue->tail++;
	if (queue->tail == LMS_TX_QUEUE_SIZE)
		queue->tail = 0;

	atomic_inc(&queue->count);
	return 0;
}

static struct sk_buff *
lms_tx_dequeue(struct lms_tx_queue *queue)
{
	struct sk_buff *ret = NULL;

	if (atomic_read(&queue->count) == 0)
		return NULL;
	ret = queue->array[queue->head];
	queue->head++;
	if (queue->head == LMS_TX_QUEUE_SIZE)
		queue->head = 0;

	atomic_dec(&queue->count);
	return ret;
}

static int
lms_init_tx_queue(struct lms_tx_queue *queue)
{
	queue->head = 0;
	queue->tail = 0;
	atomic_set(&queue->count, 0);
	return 0;
}

static void
lms_destroy_tx_queue(struct lms_spi *lms)
{
	while ((atomic_read(&lms->queue.count)) > 0) {
		dev_kfree_skb(lms->queue.array[lms->queue.head]);
		lms->queue.head++;
		if (lms->queue.head == LMS_TX_QUEUE_SIZE)
			lms->queue.head = 0;
		atomic_dec(&lms->queue.count);
	}
}

static void *
netdev_to_lms(const struct net_device *dev)
{
	struct lms_spi **lms = netdev_priv(dev);

	if (!lms)
		return NULL;
	return *lms;
}

static void
lms_set_CG2H_mailbox(struct lms_CG2H_mailbox *mailbox, u32 new_val)
{
	mutex_lock(&mailbox->lock);
	mailbox->val = new_val;
	mailbox->last_update_time = ktime_get_real_seconds();
	mutex_unlock(&mailbox->lock);
}

static void
lms_get_CG2H_mailbox(struct lms_CG2H_mailbox *mailbox,
		     struct lms_get_CG2H_mailbox *out)
{
	mutex_lock(&mailbox->lock);
	out->val = mailbox->val;
	out->last_update_time = mailbox->last_update_time;
	mutex_unlock(&mailbox->lock);
}

static int
lms_spi_read_register(struct lms_spi *lms, u8 reg, u32 *result)
{
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	struct spi_transfer transfer = {
		.tx_buf = lms->fdplx_tx,
		.rx_buf = lms->fdplx_rx,
		.len = (sizeof(reg) + sizeof(*result)),
	};
#else
	struct spi_transfer transfer[2] = {
		{ .tx_buf = &reg, .len = sizeof(reg) },
		{ .rx_buf = result, .len = sizeof(*result) }};
#endif
	struct spi_message msg;
	int ret = 0;

	if (lms->sw_lsb_first)
		reg = __constant_bitrev8(reg);

	spi_message_init(&msg);
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	lms->fdplx_tx->cmd = reg;
	spi_message_add_tail(&transfer, &msg);
#else
	spi_message_add_tail(&transfer[0], &msg);
	spi_message_add_tail(&transfer[1], &msg);
#endif

	ret = spi_sync(lms->spi_dev, &msg);
	if (ret) {
		netdev_err(lms->net_dev, "spi read register error: %d\n", ret);
		return ret;
	}

#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	memcpy(result, lms->fdplx_rx->buf, sizeof(*result));
#endif

	*result = le32_to_cpu(*result);
	if (lms->sw_lsb_first)
		bitrev8_buf((u8 *)result, sizeof(*result));

	return ret;
}

static inline int
lms_spi_next_packet_size(struct lms_spi *lms, u16 *packet_size)
{
	u8 cmd_to_send;
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	struct spi_transfer transfer = {
		.tx_buf = lms->fdplx_tx,
		.rx_buf = lms->fdplx_rx,
		.len = (sizeof(cmd_to_send) + sizeof(*packet_size)),
	};
#else
	struct spi_transfer transfer[2] = {
		{ .tx_buf = &cmd_to_send, .len = sizeof(cmd_to_send) },
		{ .rx_buf = packet_size, .len = sizeof(*packet_size) }};
#endif
	struct spi_message msg;
	int ret = 0;

	if (!lms->sw_lsb_first)
		cmd_to_send = LMS_READ_SIZE_COMMAND;
	else
		cmd_to_send = __constant_bitrev8(LMS_READ_SIZE_COMMAND);

	spi_message_init(&msg);
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	lms->fdplx_tx->cmd = cmd_to_send;
	spi_message_add_tail(&transfer, &msg);
#else
	spi_message_add_tail(&transfer[0], &msg);
	spi_message_add_tail(&transfer[1], &msg);
#endif
	ret = spi_sync(lms->spi_dev, &msg);
	if (ret) {
		netdev_err(lms->net_dev, "spi read packet size error: %d\n", ret);
		return ret;
	}

#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	memcpy(packet_size, lms->fdplx_rx->buf, sizeof(*packet_size));
#endif

	*packet_size = le16_to_cpu(*packet_size);
	if (lms->sw_lsb_first)
		bitrev8_buf((u8 *)packet_size, sizeof(*packet_size));

	return ret;
}

static int
lms_spi_read_data(struct lms_spi *lms, u16 *size)
{
	u8 cmd_to_send;
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	struct spi_transfer transfer = {
		.tx_buf = lms->fdplx_tx,
		.rx_buf = lms->fdplx_rx,
		.len = 0 };
#else
	struct spi_transfer transfer[2] = {
		{ .tx_buf = &cmd_to_send, .len = sizeof(cmd_to_send) },
		{ .rx_buf = lms->spi_buffer, .len = 0 } };
#endif
	struct spi_message msg;
	int ret = 0;

	ret = lms_spi_next_packet_size(lms, size);
	if (ret) {
		netdev_err(lms->net_dev, "spi read data READ_SIZE error: %d\n",
			   ret);
		lms->net_dev->stats.rx_errors++;
		return ret;
	}

	if (*size > LMS_MAX_RX_BUFF_LEN) {
		netdev_err(lms->net_dev, "size of rx too big: %d\n", *size);
		lms->net_dev->stats.rx_errors++;
		lms->net_dev->stats.rx_length_errors++;
		return -1;
	}

	if (*size < ETH_HLEN) {
		netdev_err(lms->net_dev, "size of rx too small: %d\n", *size);
		lms->net_dev->stats.rx_errors++;
		lms->net_dev->stats.rx_length_errors++;
		return -1;
	}

	/* Send a READ_DATA command */
	if (!lms->sw_lsb_first)
		cmd_to_send = LMS_READ_DATA_COMMAND;
	else
		cmd_to_send = __constant_bitrev8(LMS_READ_DATA_COMMAND);

	spi_message_init(&msg);

#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	lms->fdplx_tx->cmd = cmd_to_send;
	transfer.len = *size + 1;
	spi_message_add_tail(&transfer, &msg);
#else
	transfer[1].len = *size;
	spi_message_add_tail(&transfer[0], &msg);
	spi_message_add_tail(&transfer[1], &msg);
#endif

	ret = spi_sync(lms->spi_dev, &msg);
	if (ret) {
		netdev_err(lms->net_dev, "spi read data READ_DATA error: %d\n",
			   ret);
		lms->net_dev->stats.rx_errors++;
		return ret;
	}

#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	if (lms->sw_lsb_first)
		bitrev8_buf(lms->fdplx_rx->buf, *size);
#else
	if (lms->sw_lsb_first)
		bitrev8_buf(lms->spi_buffer, *size);
#endif

	lms->net_dev->stats.rx_bytes += *size;
	lms->net_dev->stats.rx_packets++;
	return ret;
}

static int
lms_spi_write_immediate(struct lms_spi *lms, u8 reg, u32 value)
{
	u8 tx_reg_dest;
	u32 tx_data;
	struct spi_transfer transfer[2];
	struct spi_message msg;
	int ret;

	memset(&transfer, 0, sizeof(transfer));
	spi_message_init(&msg);

	/* Send the data */
	tx_reg_dest = reg;
	tx_data = cpu_to_le32(value);

	if (lms->sw_lsb_first) {
		tx_reg_dest = __constant_bitrev8(tx_reg_dest);
		bitrev8_buf((u8 *)&tx_data, sizeof(tx_data));
	}

	transfer[0].tx_buf = &tx_reg_dest;
	transfer[0].len = sizeof(tx_reg_dest);
	transfer[1].tx_buf = &tx_data;
	transfer[1].len = sizeof(tx_data);

	spi_message_add_tail(&transfer[0], &msg);
	spi_message_add_tail(&transfer[1], &msg);
	ret = spi_sync(lms->spi_dev, &msg);
	if (ret)
		netdev_err(lms->net_dev, "spi write immediate error: %d\n", ret);

	return ret;
}

static int
lms_update_modem_status_flag(struct lms_spi *lms)
{
	int ret;
	u32 val = 0;

	ret = lms_spi_read_register(lms, LMS_CG2H_MAILBOX_REG_ZERO, &val);
	if (!ret) {
		lms_set_CG2H_mailbox(&lms->CG2H_mailbox_zero, val);
		if (val & BIT(0))
			atomic_set(&lms->spi_modem_status, 1);
		else if (val & BIT(1))
			atomic_set(&lms->spi_modem_status, 0);
	}

	return ret;
}

static int
lms_check_stat_zero(struct lms_spi *lms, bool print)
{
	u32 stat_zero = 0;
	u32 val = 0;
	int i;
	int ret = 0;

	atomic_set(&lms->stat_intr_req, 0);

	/* Check status 0 register */
	ret = lms_spi_read_register(lms, LMS_READ_STAT_REG_ZERO, &stat_zero);
	if (ret) {
		if (print)
			netdev_err(lms->net_dev, "failed to read status0 register");
		return -1;
	}

	/* Sanity checks */
	for (i = LMS_FIRST_CONST_ZERO; i <= LMS_LAST_CONST_ZERO; i++) {
		if (stat_zero & BIT(i)) {
			if (print)
				netdev_err(lms->net_dev,
					   "spi check stat 0 error: should be 0, got 1 instead in bit num %d\n",
					   i);
			return -1;
		}
	}

	if ((stat_zero & BIT(LMS_SANITY_ONE)) ||
	    !(stat_zero & BIT(LMS_SANITY_TWO)) ||
	    (stat_zero & BIT(LMS_SANITY_THREE)) ||
	    !(stat_zero & BIT(LMS_SANITY_FOUR))) {
		if (print)
			netdev_err(lms->net_dev, "spi check stat sanity bits error: should be 1010, got %li%li%li%li (0x%08X)\n",
				   (stat_zero & BIT(LMS_SANITY_FOUR)) >> LMS_SANITY_FOUR,
				   (stat_zero & BIT(LMS_SANITY_THREE)) >> LMS_SANITY_THREE,
				   (stat_zero & BIT(LMS_SANITY_TWO)) >> LMS_SANITY_TWO,
				   (stat_zero & BIT(LMS_SANITY_ONE)) >> LMS_SANITY_ONE,
				   stat_zero);
		return -1;
	}

	if (stat_zero & BIT(LMS_CG2H_MAIL_ZERO))
		lms_update_modem_status_flag(lms);

	if (stat_zero & BIT(LMS_CG2H_MAIL_ONE)) {
		ret = lms_spi_read_register(lms, LMS_CG2H_MAILBOX_REG_ONE, &val);
		if (!ret)
			lms_set_CG2H_mailbox(&lms->CG2H_mailbox_one, val);
	}

	if (stat_zero & BIT(LMS_CG2H_MAIL_TWO)) {
		ret = lms_spi_read_register(lms, LMS_CG2H_MAILBOX_REG_TWO, &val);
		if (!ret)
			lms_set_CG2H_mailbox(&lms->CG2H_mailbox_two, val);
	}

	if (stat_zero & BIT(LMS_CG2H_MAIL_THREE)) {
		ret = lms_spi_read_register(lms, LMS_CG2H_MAILBOX_REG_THREE, &val);
		if (!ret)
			lms_set_CG2H_mailbox(&lms->CG2H_mailbox_three, val);
	}

	if (stat_zero & BIT(LMS_H2CG_MAIL_ZERO))
		atomic_set(&lms->H2CG_status_mailbox_zero, 1);
	else
		atomic_set(&lms->H2CG_status_mailbox_zero, 0);

	if (stat_zero & BIT(LMS_H2CG_MAIL_ONE))
		atomic_set(&lms->H2CG_status_mailbox_one, 1);
	else
		atomic_set(&lms->H2CG_status_mailbox_one, 0);

	if (stat_zero & BIT(LMS_H2CG_MAIL_TWO))
		atomic_set(&lms->H2CG_status_mailbox_two, 1);
	else
		atomic_set(&lms->H2CG_status_mailbox_two, 0);

	if (stat_zero & BIT(LMS_H2CG_MAIL_THREE))
		atomic_set(&lms->H2CG_status_mailbox_three, 1);
	else
		atomic_set(&lms->H2CG_status_mailbox_three, 0);

	if (stat_zero & BIT(LMS_OVERFLOW_TX))
		atomic_inc(&lms->overflow_counter_tx);

	if (stat_zero & BIT(LMS_OVERFLOW_RX))
		atomic_inc(&lms->overflow_counter_rx);

	if ((stat_zero & BIT(LMS_EXCEPTION_ONE)) || (stat_zero & BIT(LMS_EXCEPTION_TWO)))
		atomic_inc(&lms->unexpected_behaviour_counter);

	return 0;
}

static int
lms_check_stat_zero_w_retries(struct lms_spi *lms, int retries)
{
	int ret = 0;
	int attempts = 0;

retry:
	ret = lms_check_stat_zero(lms, false);
	if (ret) {
		if (attempts++ < retries) {
			msleep(LMS_STATUS_ZERO_CHECK_RETRY_DELAY_MS);
			goto retry;
		}
	}

	return ret;
}

static int lms_spi_receive(struct lms_spi *lms, u8 max_num_packets);

static int
lms_spi_write_data(struct lms_spi *lms, u8 *buffer, u16 size)
{
	u8 tx_write_size[3];
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	struct spi_transfer transfer[2];
#else
	u8 cmd_to_send;
	struct spi_transfer transfer[3];
#endif
	struct spi_message msg;
	int ret;
	int counter = 0;
	u32 reg_val = 0;
	u16 size_in_fifo = size;

	if (atomic_read(&lms->spi_modem_status))
		/* CG in Bootrom */
		size_in_fifo += LMS_BOOTROM_MAX_OCCUPANCY_ADDITION_PER_PACKET;
	else
		/* CG running FW code */
		size_in_fifo += LMS_FW_MAX_OCCUPANCY_ADDITION_PER_PACKET;

	memset(&transfer, 0, sizeof(transfer));

	/* if tx_occupancy not enough update it from firmware */
	if (lms->tx_occupancy >= LMS_TX_MAX_OCCUPANCY_BYTES ||
	    size_in_fifo > (LMS_TX_MAX_OCCUPANCY_BYTES - lms->tx_occupancy)) {
		lms_spi_read_register(lms, LMS_READ_STAT_REG_ONE, &reg_val);
		/* This is to get number of bytes currently in use inside the TX FIFO */
		lms->tx_occupancy = LMS_REG_ONE_GET_OCCUPANCY_IN_BYTES(reg_val);

		/* if tx_occupancy still not enough busy wait for it to clear */
		while (lms->tx_occupancy >= LMS_TX_MAX_OCCUPANCY_BYTES ||
		       size_in_fifo > (LMS_TX_MAX_OCCUPANCY_BYTES - lms->tx_occupancy)) {
			if (counter >= LMS_TX_OCCU_FIFO_MAX_RETRY_COUNT) {
				netdev_err(lms->net_dev, "failed to write tx packet, size %d, timeout\n",
					   size);
				lms->net_dev->stats.tx_dropped++;
				return -1;
			}

			if (atomic_read(&lms->rx_intr_req)) {
				ret = lms_spi_receive(lms, 1);
				if (ret) {
					netdev_err(lms->net_dev, "failed to receive data from modem\n");
					return ret;
				}
			} else {
				usleep_range(2000, 4000);
			}

			lms_spi_read_register(lms, LMS_READ_STAT_REG_ONE, &reg_val);
			/* This is to get number of bytes currently in use inside the TX FIFO */
			lms->tx_occupancy = LMS_REG_ONE_GET_OCCUPANCY_IN_BYTES(reg_val);
			counter++;
		}
	}

	lms->tx_occupancy += size_in_fifo;

	/* Send a WR_SIZE command */
	tx_write_size[0] = LMS_WRITE_SIZE_COMMAND;
	tx_write_size[1] = (u8)(size & 0xFF);
	tx_write_size[2] = (u8)(size >> 8);
	if (lms->sw_lsb_first)
		bitrev8_buf(tx_write_size, sizeof(tx_write_size));

	transfer[0].tx_buf = tx_write_size;
	transfer[0].len = sizeof(tx_write_size);
	transfer[0].cs_change = true;

	spi_message_init(&msg);
	spi_message_add_tail(&transfer[0], &msg);

	/* Send a WR_DATA command */
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	lms->fdplx_tx->cmd = LMS_WRITE_DATA_COMMAND;
	memcpy(lms->fdplx_tx->buf, buffer, size);
	transfer[1].tx_buf = lms->fdplx_tx;
	transfer[1].len = size + 1;
	if (lms->sw_lsb_first)
		bitrev8_buf((u8 *)lms->fdplx_tx, size + 1);
	spi_message_add_tail(&transfer[1], &msg);
#else
	memcpy(lms->spi_buffer, buffer, size);
	cmd_to_send = LMS_WRITE_DATA_COMMAND;
	transfer[1].tx_buf = &cmd_to_send;
	transfer[1].len = sizeof(cmd_to_send);
	transfer[2].tx_buf = lms->spi_buffer;
	transfer[2].len = size;
	if (lms->sw_lsb_first) {
		cmd_to_send = __constant_bitrev8(cmd_to_send);
		bitrev8_buf(lms->spi_buffer, size);
	}
	spi_message_add_tail(&transfer[1], &msg);
	spi_message_add_tail(&transfer[2], &msg);
#endif

	ret = spi_sync(lms->spi_dev, &msg);
	if (ret) {
		netdev_err(lms->net_dev, "spi write data WRITE_DATA error: %d\n", ret);
		lms->net_dev->stats.tx_errors++;
		return ret;
	}

	lms->net_dev->stats.tx_bytes += size;
	lms->net_dev->stats.tx_packets++;
	return ret;
}

static int
lms_spi_set_mailbox(struct lms_spi *lms, u8 reg, u32 value)
{
	int ret;
	atomic_t *status_flag;
	int counter = 0;
	/* Check if the register we got is
	 *one of the H2CG mailboxes
	 */
	switch (reg) {
	case LMS_H2CG_MAILBOX_REG_ZERO:
		status_flag = &lms->H2CG_status_mailbox_zero;
		break;
	case LMS_H2CG_MAILBOX_REG_ONE:
		status_flag = &lms->H2CG_status_mailbox_one;
		break;
	case LMS_H2CG_MAILBOX_REG_TWO:
		status_flag = &lms->H2CG_status_mailbox_two;
		break;
	case LMS_H2CG_MAILBOX_REG_THREE:
		status_flag = &lms->H2CG_status_mailbox_three;
		break;
	default:
		netdev_err(lms->net_dev, "spi write immediate error: Illegal register provided: %u, should be between %u and %u\n",
			   reg, LMS_H2CG_MAILBOX_REG_ZERO,
			   LMS_H2CG_MAILBOX_REG_THREE);
		return -1;
	}

	/* Busy wait until the register we
	 * want to send to is free
	 */
	while (atomic_read(status_flag)) {
		if (counter >= LMS_MAX_BUSY_WAIT_SLEEP_TIMES) {
			netdev_err(lms->net_dev, "failed to read reg %d, timeout, waited %d ms\n",
				   reg, (counter * LMS_BUSY_WAIT_SLEEP_LEN));
		}
		msleep(LMS_BUSY_WAIT_SLEEP_LEN);
		counter++;
		lms_check_stat_zero(lms, false);
	}

	ret = lms_spi_write_immediate(lms, reg, value);
	if (!ret) {
		netdev_err(lms->net_dev, "spi write immediate error: %d\n", ret);
		return ret;
	}

	atomic_set(status_flag, 1);

	return ret;
}

static int
lms_spi_receive(struct lms_spi *lms, u8 max_num_packets)
{
	u16 data_size;
	int ret = 0;
	unsigned int res, retry_count;
	u32 result;
	u8 i = 0;
	int gpio_val = 1;
	bool fast_rx_completion = false;

	/* Receiving packets as long as the RX irq is up */
	while (gpio_val) {
		if (!lms->rx_packets_waiting) {
			/* Reading the RX FIFO occupancy register */
			lms_spi_read_register(lms, LMS_READ_STAT_REG_TWO, &result);
			/* This is to get the number of packets waiting in the RX FIFO */
			lms->rx_packets_waiting = LMS_REG_TWO_GET_NUM_PACKETS_IN_FIFO(result);

			if (!lms->rx_packets_waiting)
				break;
			else if (lms->rx_packets_waiting <= LMS_FAST_RX_COMPLETION_THRESHOLD)
				fast_rx_completion = true;
		}

		/* Receive all the packets waiting in the RX FIFO */
		for (; i < max_num_packets && lms->rx_packets_waiting; i++) {
			lms->rx_packets_waiting--;
			ret = lms_spi_read_data(lms, &data_size);
			if (ret) {
				netdev_err(lms->net_dev, "%s error: failed to read data %d\n",
					   __func__, ret);
				return ret;
			}

			retry_count = 0;
retry:
			res = kfifo_avail(&lms->rx_completion_fifo);
			if (res < data_size + sizeof(data_size)) {
				netdev_err(lms->net_dev, "%s rx completion fifo full\n", __func__);

				if (++retry_count > 3) {
					netdev_err(lms->net_dev, "%s packet dropped\n", __func__);
					return -ENOMEM;
				}

				msleep(LMS_BUSY_WAIT_SLEEP_LEN);
				goto retry;
			} else {
				kfifo_in(&lms->rx_completion_fifo,
					 &data_size, sizeof(data_size));
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
				kfifo_in(&lms->rx_completion_fifo,
					 lms->fdplx_rx->buf, data_size);
#else
				kfifo_in(&lms->rx_completion_fifo,
					 lms->spi_buffer, data_size);
#endif
				if (fast_rx_completion ||
				    kfifo_avail(&lms->rx_completion_fifo) < (LMS_RX_FIFO_SIZE / 2))
					queue_work(lms->rx_completion_wq, &lms->rx_completion_work);
			}
		}

		if (!fast_rx_completion)
			queue_work(lms->rx_completion_wq, &lms->rx_completion_work);

		if (i < max_num_packets)
			gpio_val = gpiod_get_value(gpio_to_desc(lms->rx_gpio));
		else
			return 0;
	}

	/* dec and not set to 0, for the case we get rx irq after the
	 * while and before the dec
	 */
	if (!lms->rx_packets_waiting)
		atomic_dec(&lms->rx_intr_req);
	else
		atomic_set(&lms->rx_intr_req, 1);

	return ret;
}

static int
lms_check_mailboxes_flags(struct lms_spi *lms)
{
	if (atomic_read(&lms->H2CG_user_write_mailbox_zero.flag))
		return 1;
	if (atomic_read(&lms->H2CG_user_write_mailbox_one.flag))
		return 1;
	if (atomic_read(&lms->H2CG_user_write_mailbox_two.flag))
		return 1;
	if (atomic_read(&lms->H2CG_user_write_mailbox_three.flag))
		return 1;
	return 0;
}

static void
lms_remove_mailboxes_sysfs(struct lms_spi *lms)
{
	sysfs_remove_file(lms->mailbox_cg2h_0,
			  &lms->cg2h_read_zero_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_cg2h_0,
			  &lms->cg2h_timestamp_zero_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_cg2h_1,
			  &lms->cg2h_read_one_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_cg2h_1,
			  &lms->cg2h_timestamp_one_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_cg2h_2,
			  &lms->cg2h_read_two_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_cg2h_2,
			  &lms->cg2h_timestamp_two_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_cg2h_3,
			  &lms->cg2h_read_three_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_cg2h_3,
			  &lms->cg2h_timestamp_three_attr_wrapper.dev_attr.attr);

	sysfs_remove_file(lms->mailbox_h2cg_0,
			  &lms->h2cg_write_zero_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_h2cg_0,
			  &lms->h2cg_status_zero_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_h2cg_1,
			  &lms->h2cg_write_one_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_h2cg_1,
			  &lms->h2cg_status_one_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_h2cg_2,
			  &lms->h2cg_write_two_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_h2cg_2,
			  &lms->h2cg_status_two_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_h2cg_3,
			  &lms->h2cg_write_three_attr_wrapper.dev_attr.attr);
	sysfs_remove_file(lms->mailbox_h2cg_3,
			  &lms->h2cg_status_three_attr_wrapper.dev_attr.attr);

	kobject_put(lms->mailboxes);
	kobject_put(lms->h2cg_mailboxes);
	kobject_put(lms->cg2h_mailboxes);
	kobject_put(lms->mailbox_h2cg_0);
	kobject_put(lms->mailbox_h2cg_1);
	kobject_put(lms->mailbox_h2cg_2);
	kobject_put(lms->mailbox_h2cg_3);
	kobject_put(lms->mailbox_cg2h_0);
	kobject_put(lms->mailbox_cg2h_1);
	kobject_put(lms->mailbox_cg2h_2);
	kobject_put(lms->mailbox_cg2h_3);
}

static void lms_rx_completion_handler(struct work_struct *work)
{
	struct lms_spi *lms;
	struct sk_buff *skb = NULL;
	unsigned int res;
	int ret;
	u16 packet_size = 0;

	lms = container_of(work, struct lms_spi, rx_completion_work);

	while (!kfifo_is_empty(&lms->rx_completion_fifo)) {
		if (kfifo_len(&lms->rx_completion_fifo) <= sizeof(packet_size))
			return;

		res = kfifo_out_peek(&lms->rx_completion_fifo,
				     &packet_size, sizeof(packet_size));
		BUG_ON(sizeof(packet_size) != res);

		skb = netdev_alloc_skb_ip_align(lms->net_dev, packet_size);
		if (!skb)
			return;

		/* skip the 2 packet_size bytes */
		kfifo_skip(&lms->rx_completion_fifo);
		kfifo_skip(&lms->rx_completion_fifo);

		res = kfifo_out(&lms->rx_completion_fifo,
				skb->data, packet_size);
		BUG_ON(packet_size != res);
		skb_put(skb, packet_size);
		skb->dev = lms->net_dev;
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb->protocol = eth_type_trans(skb, skb->dev);

#if KERNEL_VERSION(5, 18, 0) <=  LINUX_VERSION_CODE
		ret = netif_rx(skb);
#else
		ret = netif_rx_ni(skb);
#endif
		if (ret == NET_RX_DROP) {
			netdev_dbg(lms->net_dev, "rx packet of type %d was dropped",
				   skb->protocol);
			lms->net_dev->stats.rx_dropped++;
		}
	}
}

static int
lms_spi_thread(void *data)
{
	struct lms_spi *lms = data;
	u32 pkt_len;
	struct sk_buff *skb;
	int ret = 0;
	struct lms_H2CG_user_write *lms_write_mbx;
	u32 i, tx_queue_size;
	long spi_retest_timeout;
	long last_spi_transaction_time = jiffies;

	if (lms_spi_idle_poll_ms)
		spi_retest_timeout = msecs_to_jiffies(lms_spi_idle_poll_ms);
	else
		spi_retest_timeout = MAX_SCHEDULE_TIMEOUT;

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (!atomic_read(&lms->rx_intr_req) &&
		    (atomic_read(&lms->queue.count) == 0) &&
		    (!atomic_read(&lms->stat_intr_req)) &&
		    (!lms_check_mailboxes_flags(lms)) &&
		    lms->spi_state == SPI_OK)
			schedule_timeout(spi_retest_timeout);

		set_current_state(TASK_RUNNING);

		if (lms->spi_state == SPI_DISABLED) {
			ret = lms_check_stat_zero(lms, false);
			if (ret) {
				msleep(LMS_WAIT_CHECK_STATUS_ZERO);
				continue;
			}
			netdev_info(lms->net_dev, "SPI interface working - Turning netif carrier on\n");
			lms_update_modem_status_flag(lms);
			lms->spi_state = SPI_OK;
			netif_carrier_on(lms->net_dev);
			netif_wake_queue(lms->net_dev);
			last_spi_transaction_time = jiffies;
			/* Force a check of the RX FIFO */
			lms->rx_packets_waiting = 0;
			atomic_set(&lms->rx_intr_req, 1);
			/* Force a re-read of the TX FIFO Occupancy */
			lms->tx_occupancy = LMS_TX_MAX_OCCUPANCY_BYTES;
			continue;
		}

		/* Check status 0 */
		if (atomic_read(&lms->stat_intr_req)) {
			/* using two tries to reduce
			 * the chance of turning carrier off
			 */
			ret = lms_check_stat_zero_w_retries(lms, 2);
			if (!ret)
				goto check_ok;

			lms->spi_state = SPI_ERR;
			ret = lms_check_stat_zero_w_retries(lms,
							    LMS_STATUS_ZERO_CHECK_MAX_RETRIES);
			if (!ret) {
				lms_update_modem_status_flag(lms);
				lms->spi_state = SPI_OK;
			} else {
				lms->spi_state = SPI_DISABLED;
				netif_stop_queue(lms->net_dev);
				netif_carrier_off(lms->net_dev);
				/* If we failed to read status 0 twice
				 * we assume SPI has stopped working
				 * we turn carrier off until it will work again
				 */
				netdev_err(lms->net_dev, "Status reg 0 sanity failed during operation - Turning carrier off\n");
				lms->rx_packets_waiting = 0;
				continue;
			}

check_ok:
			last_spi_transaction_time = jiffies;
		}

		/* Check RX */
		if (atomic_read(&lms->rx_intr_req)) {
			ret = lms_spi_receive(lms, LMS_NUM_RX_BEFORE_TX);
			if (ret) {
				netdev_err(lms->net_dev, "failed to receive data from modem\n");
				atomic_set(&lms->stat_intr_req, 1);
			} else {
				last_spi_transaction_time = jiffies;
			}
		}

		/* Check if something is in TX queue and transmit it */
		tx_queue_size = atomic_read(&lms->queue.count);
		for (i = 0; i < tx_queue_size && i < LMS_NUM_TX_BEFORE_RX; ++i) {
			/* Remove the skb from the queue */
			netif_tx_lock_bh(lms->net_dev);
			skb = lms_tx_dequeue(&lms->queue);
			netif_tx_unlock_bh(lms->net_dev);

			if (netif_queue_stopped(lms->net_dev))
				netif_wake_queue(lms->net_dev);

			pkt_len = skb->len;
			/* Transmit a single message */
			ret = lms_spi_write_data(lms, (u8 *)skb->data, pkt_len);
			if (ret) {
				lms->net_dev->stats.tx_errors++;
				atomic_set(&lms->stat_intr_req, 1);
				dev_kfree_skb(skb);
				break;
			}
			dev_kfree_skb(skb);
		}

		/* Check H2CG user write */
		lms_write_mbx = &lms->H2CG_user_write_mailbox_zero;
		if (atomic_read(&lms_write_mbx->flag)) {
			ret = lms_spi_set_mailbox(lms, LMS_H2CG_MAILBOX_REG_ZERO,
						  atomic_read(&lms_write_mbx->val));
			if (ret)
				netdev_err(lms->net_dev, "failed to set mailbox0\n");

			atomic_set(&lms_write_mbx->flag, 0);
		}

		lms_write_mbx = &lms->H2CG_user_write_mailbox_one;
		if (atomic_read(&lms_write_mbx->flag)) {
			ret = lms_spi_set_mailbox(lms, LMS_H2CG_MAILBOX_REG_ONE,
						  atomic_read(&lms_write_mbx->val));
			if (ret)
				netdev_err(lms->net_dev, "failed to set mailbox1\n");

			atomic_set(&lms_write_mbx->flag, 0);
		}

		lms_write_mbx = &lms->H2CG_user_write_mailbox_two;
		if (atomic_read(&lms_write_mbx->flag)) {
			ret = lms_spi_set_mailbox(lms, LMS_H2CG_MAILBOX_REG_TWO,
						  atomic_read(&lms_write_mbx->val));
			if (ret)
				netdev_err(lms->net_dev, "failed to set mailbox2\n");

			atomic_set(&lms_write_mbx->flag, 0);
		}

		lms_write_mbx = &lms->H2CG_user_write_mailbox_three;
		if (atomic_read(&lms_write_mbx->flag)) {
			ret = lms_spi_set_mailbox(lms, LMS_H2CG_MAILBOX_REG_THREE,
						  atomic_read(&lms_write_mbx->val));
			if (ret)
				netdev_err(lms->net_dev, "failed to set mailbox3\n");

			atomic_set(&lms_write_mbx->flag, 0);
		}

		if (jiffies - last_spi_transaction_time >= spi_retest_timeout) {
			last_spi_transaction_time = jiffies;
			atomic_set(&lms->stat_intr_req, 1);
		}
	}

	return 0;
}

static irqreturn_t
lms_spi_intr_handler(int irq, void *data)
{
	struct lms_spi *lms = data;
	struct net_device *dev = lms->net_dev;
	struct task_struct *spi_thread = lms->spi_thread;

	if (irq == lms->rx_irq)
		atomic_inc(&lms->rx_intr_req);
	else if (irq == lms->stat_irq)
		atomic_set(&lms->stat_intr_req, 1);
	else
		netdev_err(dev, "%s: unknown irq %d", LMS_DRV_NAME, irq);

	/* Wake up the SPI thread */
	if (spi_thread)
		wake_up_process(spi_thread);

	return IRQ_HANDLED;
}

netdev_tx_t
lms_spi_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct lms_spi *lms = netdev_to_lms(dev);
	int ret = NETDEV_TX_OK;
	unsigned int padding = 0;

	if (skb->len < LMS_MIN_PACKET_LEN) {
		padding = LMS_MIN_PACKET_LEN - skb->len;

		if (skb_tailroom(skb) < padding) {
			struct sk_buff *skb_tmp;

			skb_tmp = skb_copy_expand(skb, 0, padding, GFP_ATOMIC);
			if (!skb_tmp)
				return NETDEV_TX_BUSY;

			dev_kfree_skb(skb);
			skb = skb_tmp;
		}

		skb_put_zero(skb, padding);
	}

	/* Insert into the queue */
	ret = lms_tx_enqueue(&lms->queue, skb);
	if (ret == -1) {
		netdev_err(dev, "%s error: lms_tx_enqueue failed\n",
			   __func__);
		return NETDEV_TX_BUSY;
	}

	if (lms->spi_thread)
		wake_up_process(lms->spi_thread);

	return NETDEV_TX_OK;
}

#if KERNEL_VERSION(5, 6, 0) <= LINUX_VERSION_CODE
void lms_spi_netdev_tx_timeout(struct net_device *dev, unsigned int txqueue)
#else
void lms_spi_netdev_tx_timeout(struct net_device *dev)
#endif
{
	struct lms_spi *lms = netdev_to_lms(dev);
	struct tm formatted_time;

	lms->net_dev->stats.tx_errors++;
	time64_to_tm(ktime_get_real_seconds(), 0, &formatted_time);
	netdev_info(lms->net_dev, "Transmit timeout at: %02d:%02d:%04ld-%02d:%02d:%02d UTC\n",
		    formatted_time.tm_mday,
		    formatted_time.tm_mon + 1,
		    formatted_time.tm_year + 1900,
		    formatted_time.tm_hour,
		    formatted_time.tm_min,
		    formatted_time.tm_sec);
	netif_trans_update(dev);
	netif_wake_queue(dev);
}

static int lms_create_sysfs_file(struct lms_spi *lms, const char *name,
				 struct kobject *dir,
				 struct lms_attr_wrapper *attr_wrapper,
				 ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr,
						 char *buf),
				 ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,
						  const char *buf, size_t count))
{
	attr_wrapper->lms = lms;
	attr_wrapper->dev_attr.attr.name = name;
	if (!store)
		attr_wrapper->dev_attr.attr.mode = LMS_READ_ONLY_PERMISSION;
	else if (!show)
		attr_wrapper->dev_attr.attr.mode = LMS_WRITE_ONLY_PERMISSION;
	else
		attr_wrapper->dev_attr.attr.mode = LMS_READ_WRITE_PERMISSION;
	attr_wrapper->dev_attr.show = show;
	attr_wrapper->dev_attr.store = store;
	sysfs_attr_init(&attr_wrapper->dev_attr.attr);

	if (sysfs_create_file(dir, &attr_wrapper->dev_attr.attr)) {
		/* TODO */
		return -ENOMEM;
	}

	return 0;
}

static ssize_t
lms_cg2h_read_mailbox0_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_get_CG2H_mailbox mail;
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	lms_get_CG2H_mailbox(&this_wrapper->lms->CG2H_mailbox_zero, &mail);
	return sprintf(buf, "%d", mail.val);
}

static ssize_t
lms_cg2h_timestamp_mailbox0_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_get_CG2H_mailbox mail;
	struct tm formatted_time;
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	lms_get_CG2H_mailbox(&this_wrapper->lms->CG2H_mailbox_zero, &mail);
	time64_to_tm(mail.last_update_time, 0, &formatted_time);
	return sprintf(buf, "%02d:%02d:%04ld-%02d:%02d:%02d UTC\n",
		       formatted_time.tm_mday,
		       formatted_time.tm_mon + 1,
		       formatted_time.tm_year + 1900,
		       formatted_time.tm_hour,
		       formatted_time.tm_min,
		       formatted_time.tm_sec);
}

static ssize_t
lms_cg2h_read_mailbox1_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_get_CG2H_mailbox mail;
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	lms_get_CG2H_mailbox(&this_wrapper->lms->CG2H_mailbox_one, &mail);
	return sprintf(buf, "%d", mail.val);
}

static ssize_t
lms_cg2h_timestamp_mailbox1_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_get_CG2H_mailbox mail;
	struct tm formatted_time;
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	lms_get_CG2H_mailbox(&this_wrapper->lms->CG2H_mailbox_one, &mail);
	time64_to_tm(mail.last_update_time, 0, &formatted_time);
	return sprintf(buf, "%02d:%02d:%04ld-%02d:%02d:%02d UTC\n",
		       formatted_time.tm_mday,
		       formatted_time.tm_mon + 1,
		       formatted_time.tm_year + 1900,
		       formatted_time.tm_hour,
		       formatted_time.tm_min,
		       formatted_time.tm_sec);
}

static ssize_t
lms_cg2h_read_mailbox2_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_get_CG2H_mailbox mail;
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	lms_get_CG2H_mailbox(&this_wrapper->lms->CG2H_mailbox_two, &mail);
	return sprintf(buf, "%d", mail.val);
}

static ssize_t
lms_cg2h_timestamp_mailbox2_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_get_CG2H_mailbox mail;
	struct tm formatted_time;
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	lms_get_CG2H_mailbox(&this_wrapper->lms->CG2H_mailbox_two, &mail);
	time64_to_tm(mail.last_update_time, 0, &formatted_time);
	return sprintf(buf, "%02d:%02d:%04ld-%02d:%02d:%02d UTC\n",
		       formatted_time.tm_mday,
		       formatted_time.tm_mon + 1,
		       formatted_time.tm_year + 1900,
		       formatted_time.tm_hour,
		       formatted_time.tm_min,
		       formatted_time.tm_sec);
}

static ssize_t
lms_cg2h_read_mailbox3_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_get_CG2H_mailbox mail;
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	lms_get_CG2H_mailbox(&this_wrapper->lms->CG2H_mailbox_three, &mail);
	return sprintf(buf, "%d", mail.val);
}

static ssize_t
lms_cg2h_timestamp_mailbox3_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_get_CG2H_mailbox mail;
	struct tm formatted_time;
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	lms_get_CG2H_mailbox(&this_wrapper->lms->CG2H_mailbox_three, &mail);
	time64_to_tm(mail.last_update_time, 0, &formatted_time);
	return sprintf(buf, "%02d:%02d:%04ld-%02d:%02d:%02d UTC\n",
		       formatted_time.tm_mday,
		       formatted_time.tm_mon + 1,
		       formatted_time.tm_year + 1900,
		       formatted_time.tm_hour,
		       formatted_time.tm_min,
		       formatted_time.tm_sec);
}

static ssize_t
lms_h2cg_write_mailbox0_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);
	u32 val;
	u8 flag = (u8)atomic_read(&this_wrapper->lms->H2CG_user_write_mailbox_zero.flag);

	if (flag) {
		netdev_err(this_wrapper->lms->net_dev, "mailbox0 busy\n");
		return -1;
	}
	sscanf(buf, "%d", &val);
	atomic_set(&this_wrapper->lms->H2CG_user_write_mailbox_zero.flag, 1);
	atomic_set(&this_wrapper->lms->H2CG_user_write_mailbox_zero.val, val);

	/* Wake up the SPI thread */
	if (this_wrapper->lms->spi_thread)
		wake_up_process(this_wrapper->lms->spi_thread);

	return count;
}

static ssize_t
lms_h2cg_status_mailbox0_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	return sprintf(buf, "%d", atomic_read(&this_wrapper->lms->H2CG_status_mailbox_zero));
}

static ssize_t
lms_h2cg_write_mailbox1_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);
	u32 val;
	u8 flag = (u8)atomic_read(&this_wrapper->lms->H2CG_user_write_mailbox_one.flag);

	if (flag) {
		netdev_err(this_wrapper->lms->net_dev, "mailbox1 busy\n");
		return -1;
	}
	sscanf(buf, "%d", &val);
	atomic_set(&this_wrapper->lms->H2CG_user_write_mailbox_one.flag, 1);
	atomic_set(&this_wrapper->lms->H2CG_user_write_mailbox_one.val, val);

	/* Wake up the SPI thread */
	if (this_wrapper->lms->spi_thread)
		wake_up_process(this_wrapper->lms->spi_thread);

	return count;
}

static ssize_t
lms_h2cg_status_mailbox1_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	return sprintf(buf, "%d", atomic_read(&this_wrapper->lms->H2CG_status_mailbox_one));
}

static ssize_t
lms_h2cg_write_mailbox2_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
			      size_t count)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);
	u32 val;
	u8 flag = (u8)atomic_read(&this_wrapper->lms->H2CG_user_write_mailbox_two.flag);

	if (flag) {
		netdev_err(this_wrapper->lms->net_dev, "mailbox2 busy\n");
		return -1;
	}
	sscanf(buf, "%d", &val);
	atomic_set(&this_wrapper->lms->H2CG_user_write_mailbox_two.flag, 1);
	atomic_set(&this_wrapper->lms->H2CG_user_write_mailbox_two.val, val);

	/* Wake up the SPI thread */
	if (this_wrapper->lms->spi_thread)
		wake_up_process(this_wrapper->lms->spi_thread);

	return count;
}

static ssize_t
lms_h2cg_status_mailbox2_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	return sprintf(buf, "%d", atomic_read(&this_wrapper->lms->H2CG_status_mailbox_two));
}

static ssize_t
lms_h2cg_write_mailbox3_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);
	u32 val;
	u8 flag = (u8)atomic_read(&this_wrapper->lms->H2CG_user_write_mailbox_three.flag);

	if (flag) {
		netdev_err(this_wrapper->lms->net_dev, "mailbox3 busy\n");
		return -1;
	}
	sscanf(buf, "%d", &val);
	atomic_set(&this_wrapper->lms->H2CG_user_write_mailbox_three.flag, 1);
	atomic_set(&this_wrapper->lms->H2CG_user_write_mailbox_three.val, val);

	/* Wake up the SPI thread */
	if (this_wrapper->lms->spi_thread)
		wake_up_process(this_wrapper->lms->spi_thread);

	return count;
}

static ssize_t
lms_h2cg_status_mailbox3_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	return sprintf(buf, "%d", atomic_read(&this_wrapper->lms->H2CG_status_mailbox_three));
}

static int
lms_init_mailboxes_cg2h_0(struct kobject *mailbox_cg2h_0, struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_0/read */
	ret = lms_create_sysfs_file(lms, "read", lms->mailbox_cg2h_0,
				    &lms->cg2h_read_zero_attr_wrapper,
				    lms_cg2h_read_mailbox0_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'read' in mailbox_cg2h_0");
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_0/timestamp */
	ret = lms_create_sysfs_file(lms, "timestamp", lms->mailbox_cg2h_0,
				    &lms->cg2h_timestamp_zero_attr_wrapper,
				    lms_cg2h_timestamp_mailbox0_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'timestamp' in mailbox_cg2h_0");
		sysfs_remove_file(lms->mailbox_cg2h_0,
				  &lms->cg2h_read_zero_attr_wrapper.dev_attr.attr);
	}

	return ret;
}

static int
lms_init_mailboxes_cg2h_1(struct kobject *mailbox_cg2h_1, struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_1/read */
	ret = lms_create_sysfs_file(lms, "read", lms->mailbox_cg2h_1,
				    &lms->cg2h_read_one_attr_wrapper,
				    lms_cg2h_read_mailbox1_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'read' in mailbox_cg2h_1");
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_1/timestamp */
	ret = lms_create_sysfs_file(lms, "timestamp", lms->mailbox_cg2h_1,
				    &lms->cg2h_timestamp_one_attr_wrapper,
				    lms_cg2h_timestamp_mailbox1_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'timestamp' in mailbox_cg2h_1");
		sysfs_remove_file(lms->mailbox_cg2h_1,
				  &lms->cg2h_read_one_attr_wrapper.dev_attr.attr);
	}

	return ret;
}

static int
lms_init_mailboxes_cg2h_2(struct kobject *mailbox_cg2h_2, struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_2/read */
	ret = lms_create_sysfs_file(lms, "read", lms->mailbox_cg2h_2,
				    &lms->cg2h_read_two_attr_wrapper,
				    lms_cg2h_read_mailbox2_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'read' in mailbox_cg2h_2");
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_2/timestamp */
	ret = lms_create_sysfs_file(lms, "timestamp", lms->mailbox_cg2h_2,
				    &lms->cg2h_timestamp_two_attr_wrapper,
				    lms_cg2h_timestamp_mailbox2_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'timestamp' in mailbox_cg2h_2");
		sysfs_remove_file(lms->mailbox_cg2h_2,
				  &lms->cg2h_read_two_attr_wrapper.dev_attr.attr);
	}

	return ret;
}

static int
lms_init_mailboxes_cg2h_3(struct kobject *mailbox_cg2h_3, struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_3/read */
	ret = lms_create_sysfs_file(lms, "read", lms->mailbox_cg2h_3,
				    &lms->cg2h_read_three_attr_wrapper,
				    lms_cg2h_read_mailbox3_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'read' in mailbox_cg2h_3");
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_3/timestamp */
	ret = lms_create_sysfs_file(lms, "timestamp", lms->mailbox_cg2h_3,
				    &lms->cg2h_timestamp_three_attr_wrapper,
				    lms_cg2h_timestamp_mailbox3_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'timestamp' in mailbox_cg2h_3");
		sysfs_remove_file(lms->mailbox_cg2h_3,
				  &lms->cg2h_read_three_attr_wrapper.dev_attr.attr);
	}

	return ret;
}

static int
lms_init_cg2h_mailboxes(struct lms_spi *lms)
{
	int ret = 0;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/ */
	lms->cg2h_mailboxes = kobject_create_and_add("cg2h_mailboxes", lms->mailboxes);
	if (!lms->cg2h_mailboxes) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create cg2h_mailboxes kobject\n",
			   __func__);
		return -1;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_0/ */
	lms->mailbox_cg2h_0 = kobject_create_and_add("mailbox_cg2h_0", lms->cg2h_mailboxes);
	if (!lms->mailbox_cg2h_0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create mailbox_cg2h_0 kobject\n",
			   __func__);
		return -1;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_1/ */
	lms->mailbox_cg2h_1 = kobject_create_and_add("mailbox_cg2h_1", lms->cg2h_mailboxes);
	if (!lms->mailbox_cg2h_1) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create mailbox_cg2h_1 kobject\n",
			   __func__);
		return -1;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_2/ */
	lms->mailbox_cg2h_2 = kobject_create_and_add("mailbox_cg2h_2", lms->cg2h_mailboxes);
	if (!lms->mailbox_cg2h_2) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create mailbox_cg2h_2 kobject\n",
			   __func__);
		return -1;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_3/ */
	lms->mailbox_cg2h_3 = kobject_create_and_add("mailbox_cg2h_3", lms->cg2h_mailboxes);
	if (!lms->mailbox_cg2h_3) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create mailbox_cg2h_3 kobject\n",
			   __func__);
		return -1;
	}

	/* Create:
	 * /sys/devices/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_0/cg2h_read_zero
	 * /sys/devices/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_0/cg2h_timestamp_zero
	 */
	ret = lms_init_mailboxes_cg2h_0(lms->mailbox_cg2h_0, lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to init CG2H 0 mailboxes\n",
			   __func__);
		return ret;
	}

	/* Create:
	 * /sys/devices/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_1/cg2h_read_one
	 * /sys/devices/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_1/cg2h_timestamp_one
	 */
	ret = lms_init_mailboxes_cg2h_1(lms->mailbox_cg2h_1, lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to init CG2H 1 mailboxes\n",
			   __func__);
		return ret;
	}

	/* Create:
	 * /sys/devices/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_2/cg2h_read_two
	 * /sys/devices/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_2/cg2h_timestamp_two
	 */
	ret = lms_init_mailboxes_cg2h_2(lms->mailbox_cg2h_2, lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to init CG2H 2 mailboxes\n",
			   __func__);
		return ret;
	}

	/* Create:
	 * /sys/devices/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_3/cg2h_read_three
	 * /sys/devices/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_cg2h_3/cg2h_timestamp_three
	 */
	ret = lms_init_mailboxes_cg2h_3(lms->mailbox_cg2h_3, lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to init CG2H 3 mailboxes\n",
			   __func__);
		return ret;
	}

	return ret;
}

static int
lms_init_h2cg_mailboxes0(struct kobject *mailbox_h2cg_0, struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_0/write */
	ret = lms_create_sysfs_file(lms, "write", lms->mailbox_h2cg_0,
				    &lms->h2cg_write_zero_attr_wrapper,
				    NULL, lms_h2cg_write_mailbox0_store);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'write' in mailbox_h2cg_0");
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_0/status */
	ret = lms_create_sysfs_file(lms, "status", lms->mailbox_h2cg_0,
				    &lms->h2cg_status_zero_attr_wrapper,
				    lms_h2cg_status_mailbox0_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'status' in mailbox_h2cg_0");
		sysfs_remove_file(lms->mailbox_h2cg_0,
				  &lms->h2cg_write_zero_attr_wrapper.dev_attr.attr);
	}

	return ret;
}

static int
lms_init_h2cg_mailboxes1(struct kobject *mailbox_h2cg_1, struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_h2cg_1/write */
	ret = lms_create_sysfs_file(lms, "write", lms->mailbox_h2cg_1,
				    &lms->h2cg_write_one_attr_wrapper,
				    NULL, lms_h2cg_write_mailbox1_store);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'write' in mailbox_h2cg_1");
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_1/status */
	ret = lms_create_sysfs_file(lms, "status", lms->mailbox_h2cg_1,
				    &lms->h2cg_status_one_attr_wrapper,
				    lms_h2cg_status_mailbox1_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'status' in mailbox_h2cg_1");
		sysfs_remove_file(lms->mailbox_h2cg_1,
				  &lms->h2cg_write_one_attr_wrapper.dev_attr.attr);
	}

	return ret;
}

static int
lms_init_h2cg_mailboxes2(struct kobject *mailbox_h2cg_2, struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_h2cg_2/write */
	ret = lms_create_sysfs_file(lms, "write", lms->mailbox_h2cg_2,
				    &lms->h2cg_write_two_attr_wrapper,
				    NULL, lms_h2cg_write_mailbox2_store);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'write' in mailbox_h2cg_2");
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_2/status */
	ret = lms_create_sysfs_file(lms, "status", lms->mailbox_h2cg_2,
				    &lms->h2cg_status_two_attr_wrapper,
				    lms_h2cg_status_mailbox2_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'status' in mailbox_h2cg_2");
		sysfs_remove_file(lms->mailbox_h2cg_2,
				  &lms->h2cg_write_two_attr_wrapper.dev_attr.attr);
	}

	return ret;
}

static int
lms_init_h2cg_mailboxes3(struct kobject *mailbox_h2cg_3, struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/mailbox_h2cg_3/write */
	ret = lms_create_sysfs_file(lms, "write", lms->mailbox_h2cg_3,
				    &lms->h2cg_write_three_attr_wrapper,
				    NULL, lms_h2cg_write_mailbox3_store);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'write' in mailbox_h2cg_3");
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_3/status */
	ret = lms_create_sysfs_file(lms, "status", lms->mailbox_h2cg_3,
				    &lms->h2cg_status_three_attr_wrapper,
				    lms_h2cg_status_mailbox3_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create sysfs file 'status' in mailbox_h2cg_3");
		sysfs_remove_file(lms->mailbox_h2cg_3,
				  &lms->h2cg_write_three_attr_wrapper.dev_attr.attr);
	}

	return ret;
}

static int
lms_init_h2cg_mailboxes(struct lms_spi *lms)
{
	int ret = 0;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/ */
	lms->h2cg_mailboxes = kobject_create_and_add("h2cg_mailboxes", lms->mailboxes);
	if (!lms->h2cg_mailboxes) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create h2cg_mailboxes kobject\n",
			   __func__);
		return -1;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_0/ */
	lms->mailbox_h2cg_0 = kobject_create_and_add("mailbox_h2cg_0", lms->h2cg_mailboxes);
	if (!lms->mailbox_h2cg_0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create mailbox_h2cg_0 kobject\n",
			   __func__);
		return -1;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_1/ */
	lms->mailbox_h2cg_1 = kobject_create_and_add("mailbox_h2cg_1", lms->h2cg_mailboxes);
	if (!lms->mailbox_h2cg_1) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create mailbox_h2cg_1 kobject\n",
			   __func__);
		return -1;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_2/ */
	lms->mailbox_h2cg_2 = kobject_create_and_add("mailbox_h2cg_2", lms->h2cg_mailboxes);
	if (!lms->mailbox_h2cg_2) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create mailbox_h2cg_2 kobject\n",
			   __func__);
		return -1;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_3/ */
	lms->mailbox_h2cg_3 = kobject_create_and_add("mailbox_h2cg_3", lms->h2cg_mailboxes);
	if (!lms->mailbox_h2cg_3) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create mailbox_h2cg_2 kobject\n",
			   __func__);
		return -1;
	}

	/* Create:
	 * /sys/devices/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_0/h2cg_read_zero
	 * /sys/devices/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_0/h2cg_timestamp_zero
	 */
	ret = lms_init_h2cg_mailboxes0(lms->mailbox_h2cg_0, lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to init H2CG 0 mailboxes\n",
			   __func__);
		return ret;
	}

	/* Create:
	 * /sys/devices/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_1/h2cg_read_one
	 * /sys/devices/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_1/h2cg_timestamp_one
	 */
	ret = lms_init_h2cg_mailboxes1(lms->mailbox_h2cg_1, lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to init H2CG 1 mailboxes\n",
			   __func__);
		return ret;
	}

	/* Create:
	 * /sys/devices/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_2/h2cg_read_two
	 * /sys/devices/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_2/h2cg_timestamp_two
	 */
	ret = lms_init_h2cg_mailboxes2(lms->mailbox_h2cg_2, lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to init H2CG 2 mailboxes\n",
			   __func__);
		return ret;
	}

	/* Create:
	 * /sys/devices/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_3/h2cg_read_three
	 * /sys/devices/<netdev_name>/Mailboxes/h2cg_mailboxes/mailbox_h2cg_3/h2cg_timestamp_three
	 */
	ret = lms_init_h2cg_mailboxes3(lms->mailbox_h2cg_3, lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to init H2CG 3 mailboxes\n",
			   __func__);
		return ret;
	}

	return ret;
}

static int
lms_init_sysfs_mailboxes(struct lms_spi *lms)
{
	int ret = 0;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/ */
	lms->mailboxes = kobject_create_and_add("Mailboxes", lms->lms_eth2spi);
	if (!lms->mailboxes) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create Mailboxes kobject\n",
			   __func__);
		return -ENOMEM;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/cg2h_mailboxes/ */
	ret = lms_init_cg2h_mailboxes(lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create CG2H mailboxes\n",
			   __func__);
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes/h2cg_mailboxes/ */
	ret = lms_init_h2cg_mailboxes(lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "%s sysfs error: failed to create H2CG mailboxes\n",
			   __func__);
		return ret;
	}

	return ret;
}

static ssize_t
lms_statistics_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);

	return sprintf(buf,
		       "overflow_counter_rx=%d\noverflow_counter_tx=%d\nunexpected_behaviour_counter=%d\n",
		       atomic_read(&this_wrapper->lms->overflow_counter_rx),
		       atomic_read(&this_wrapper->lms->overflow_counter_tx),
		       atomic_read(&this_wrapper->lms->unexpected_behaviour_counter));
}

static int
lms_init_sysfs_statistics(struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/statistics */
	ret = lms_create_sysfs_file(lms, "statistics", lms->lms_eth2spi,
				    &lms->statistics_wrapper,
				    lms_statistics_show, NULL);

	return ret;
}

static ssize_t
lms_modem_status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct lms_attr_wrapper *this_wrapper =
		container_of(attr, struct lms_attr_wrapper, dev_attr);
	int status = 0;

	if (this_wrapper->lms->spi_state == SPI_DISABLED) {
		int ret = gpiod_get_value(gpio_to_desc(this_wrapper->lms->stat_gpio));

		status = (ret == 1);
	} else if (this_wrapper->lms->spi_state == SPI_OK) {
		status = atomic_read(&this_wrapper->lms->spi_modem_status);
	}

	if (status)
		return sprintf(buf, "1");

	return 0;
}

static ssize_t
lms_legacy_modem_status_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	if (lms_probe_counter != 1)
		return -EBUSY;
	else
		return lms_modem_status_show(kobj, attr, buf);
}

static int
lms_init_sysfs_notifications_flow(struct lms_spi *lms)
{
	int ret;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/modem_status */
	ret = lms_create_sysfs_file(lms, "modem_status", lms->lms_eth2spi,
				    &lms->modem_status_wrapper,
				    lms_modem_status_show, NULL);
	if (ret) {
		netdev_err(lms->net_dev, "Cannot create modem_status sysfs file");
		return ret;
	}

	mutex_lock(&lms_global_lock);
	lms_probe_counter++;
	if (lms_probe_counter == 1)
		lms->legacy_modem_status_owner = true;
	mutex_unlock(&lms_global_lock);

	/* Create legacy /sys/devices/lms_eth2spi/modem_status */
	if (lms->legacy_modem_status_owner) {
		ret = lms_create_sysfs_file(lms, "modem_status", &lms_root_device->kobj,
					    &lms->legacy_modem_status_wrapper,
					    lms_legacy_modem_status_show, NULL);
		if (ret) {
			netdev_err(lms->net_dev, "Cannot create legacy modem_status sysfs file");
			sysfs_remove_file(lms->lms_eth2spi,
					  &lms->modem_status_wrapper.dev_attr.attr);
		}
	}

	return ret;
}

static int
lms_init_notif_sysfs(struct lms_spi *lms)
{
	int ret = 0;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/ */
	lms->lms_eth2spi = kobject_create_and_add(lms->net_dev->name, &lms_root_device->kobj);
	if (!lms->lms_eth2spi) {
		dev_err(&lms->spi_dev->dev, "sysfs error: failed to create Mailboxes kobject");
		return -ENOMEM;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/modem_status
	 */
	ret = lms_init_sysfs_notifications_flow(lms);
	if (ret < 0) {
		dev_err(&lms->spi_dev->dev, "lms_init_sysfs sysfs error: failed to create notifications files\n");
		return ret;
	}
	return ret;
}

static int
lms_spi_netdev_close(struct net_device *dev)
{
	struct lms_spi *lms = netdev_to_lms(dev);

	netif_stop_queue(dev);
	lms_remove_mailboxes_sysfs(lms);
	sysfs_remove_file(lms->lms_eth2spi,
			  &lms->statistics_wrapper.dev_attr.attr);
	kthread_stop(lms->spi_thread);
	lms->spi_thread = NULL;
	cancel_work_sync(&lms->rx_completion_work);
	destroy_workqueue(lms->rx_completion_wq);
	lms->rx_completion_wq = NULL;
	kfifo_free(&lms->rx_completion_fifo);

	return 0;
}

int lms_spi_netdev_open(struct net_device *dev)
{
	struct lms_spi *lms = netdev_to_lms(dev);
	int ret = 0;

	if (!lms)
		return -EINVAL;

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/Mailboxes */
	ret = lms_init_sysfs_mailboxes(lms);
	if (ret < 0) {
		netdev_err(lms->net_dev, "lms_init_sysfs sysfs error: failed to create mailboxes\n");
		return ret;
	}

	/* Create /sys/devices/lms_eth2spi/<netdev_name>/statistics */
	ret = lms_init_sysfs_statistics(lms);
	if (ret < 0) {
		lms_remove_mailboxes_sysfs(lms);
		netdev_err(lms->net_dev, "lms_init_sysfs sysfs error: failed to create statistics file\n");
		return ret;
	}

	ret = kfifo_alloc(&lms->rx_completion_fifo, LMS_RX_FIFO_SIZE, GFP_KERNEL);
	if (ret < 0) {
		lms_remove_mailboxes_sysfs(lms);
		sysfs_remove_file(lms->lms_eth2spi,
				  &lms->statistics_wrapper.dev_attr.attr);
		netdev_err(lms->net_dev, "kfifo_alloc error\n");
		return ret;
	}

	lms->rx_completion_wq = create_singlethread_workqueue("lms_rx_completion_wq");
	if (!lms->rx_completion_wq) {
		kfifo_free(&lms->rx_completion_fifo);
		lms_remove_mailboxes_sysfs(lms);
		sysfs_remove_file(lms->lms_eth2spi,
				  &lms->statistics_wrapper.dev_attr.attr);
		netdev_err(dev, "%s: Unable to create the rx completion workqueue!\n",
			   LMS_DRV_NAME);
		return ret;
	}

	INIT_WORK(&lms->rx_completion_work, lms_rx_completion_handler);

	lms->spi_thread = kthread_run(lms_spi_thread,
				      lms, "%s", dev->name);

	if (IS_ERR(lms->spi_thread)) {
		destroy_workqueue(lms->rx_completion_wq);
		kfifo_free(&lms->rx_completion_fifo);
		lms_remove_mailboxes_sysfs(lms);
		sysfs_remove_file(lms->lms_eth2spi,
				  &lms->statistics_wrapper.dev_attr.attr);
		netdev_err(dev, "%s: Unable to start the SPI thread!\n",
			   LMS_DRV_NAME);
		return PTR_ERR(lms->spi_thread);
	}

	if (lms->spi_state == SPI_OK && netif_queue_stopped(lms->net_dev))
		netif_wake_queue(lms->net_dev);

	return 0;
}

static int
lms_CG2H_mailboxs_init(struct lms_spi *lms)
{
	lms->CG2H_mailbox_zero.val = -1;
	lms->CG2H_mailbox_zero.last_update_time = 0;
	mutex_init(&lms->CG2H_mailbox_zero.lock);

	lms->CG2H_mailbox_one.val = -1;
	lms->CG2H_mailbox_one.last_update_time = 0;
	mutex_init(&lms->CG2H_mailbox_one.lock);

	lms->CG2H_mailbox_two.val = -1;
	lms->CG2H_mailbox_two.last_update_time = 0;
	mutex_init(&lms->CG2H_mailbox_two.lock);

	lms->CG2H_mailbox_three.val = -1;
	lms->CG2H_mailbox_three.last_update_time = 0;
	mutex_init(&lms->CG2H_mailbox_three.lock);

	return 0;
}

static void
lms_CG2H_mailboxs_uninit(struct lms_spi *lms)
{
	mutex_destroy(&lms->CG2H_mailbox_zero.lock);
	mutex_destroy(&lms->CG2H_mailbox_one.lock);
	mutex_destroy(&lms->CG2H_mailbox_two.lock);
	mutex_destroy(&lms->CG2H_mailbox_three.lock);
}

static const struct net_device_ops lms_spi_netdev_ops = {
	.ndo_open = lms_spi_netdev_open,
	.ndo_stop = lms_spi_netdev_close,
	.ndo_start_xmit = lms_spi_netdev_xmit,
	.ndo_tx_timeout = lms_spi_netdev_tx_timeout,
	.ndo_set_mac_address = eth_mac_addr,
};

static void
lms_remove_notif_sysfs(struct lms_spi *lms)
{
	sysfs_remove_file(lms->lms_eth2spi,
			  &lms->modem_status_wrapper.dev_attr.attr);
	kobject_put(lms->lms_eth2spi);

	mutex_lock(&lms_global_lock);
	lms_probe_counter--;
	mutex_unlock(&lms_global_lock);
	if (lms->legacy_modem_status_owner)
		sysfs_remove_file(&lms_root_device->kobj,
				  &lms->legacy_modem_status_wrapper.dev_attr.attr);
}

static int
lms_spi_netdev_setup(struct lms_spi *lms, struct net_device *dev)
{
	char net_dev_name[IFNAMSIZ];
	struct net_device *netdev = NULL;
	const char *iface_prefix;
	u32 iface_idx;
	bool is_default_idx = false;
	int ret;

	ret = of_property_read_string(lms->spi_dev->dev.of_node, "iface-name-prefix", &iface_prefix);
	if (ret)
		iface_prefix = LMS_DEFAULT_NETDEV_NAME_PREFIX;

	ret = of_property_read_u32(lms->spi_dev->dev.of_node, "iface-idx", &iface_idx);
	if (ret) {
		iface_idx = 0;
		is_default_idx = true;
	}

	snprintf(net_dev_name, sizeof(net_dev_name), "%s%u", iface_prefix, iface_idx);

	dev->netdev_ops = &lms_spi_netdev_ops;
	netdev = dev_get_by_name(&init_net, net_dev_name);
	if (is_default_idx == true)
	{
		while (netdev) {
			dev_put(netdev);
			if (++iface_idx == 10) {
				netdev_err(dev, "could not find free netdev name");
				return -1;
			}
			snprintf(net_dev_name, sizeof(net_dev_name), "%s%u", iface_prefix, iface_idx);
			netdev = dev_get_by_name(&init_net, net_dev_name);
		}
	}
	else if (netdev)
	{
		netdev_err(dev, "could not allocate net dev name: %s", net_dev_name);
		return -1;
	}

	strcpy(dev->name, net_dev_name);
	dev->watchdog_timeo = LMS_SPI_TX_TIMEOUT;
	dev->priv_flags &= ~IFF_TX_SKB_SHARING;
	dev->tx_queue_len = 100;

	/* MTU range: 46 - 1500 */
	dev->min_mtu = LMS_FRM_MIN_MTU;
	dev->max_mtu = LMS_FRM_MAX_MTU;

	if (lms->spi_state != SPI_OK) {
		netif_stop_queue(dev);
		netif_carrier_off(dev);
	}

	return 0;
}

static const struct of_device_id lms_spi_of_match[] = {
	{ .compatible = "cg,cg5317" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lms_spi_of_match);

static void free_lms(struct lms_spi *lms)
{
	if (lms) {
#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
		if (lms->fdplx_rx)
			kfree(lms->fdplx_rx);
		if (lms->fdplx_tx)
			kfree(lms->fdplx_tx);
#else
		if (lms->spi_buffer)
			kfree(lms->spi_buffer);
#endif
		kfree(lms);
	}
}

static struct lms_spi *alloc_lms(void)
{
	struct lms_spi *lms = NULL;

	lms = kmalloc(sizeof(*lms), GFP_KERNEL);
	if (!lms)
		goto err;

	memset(lms, 0, sizeof(*lms));

#ifdef CONFIG_LMS_FULL_DPLX_TX_RX_SPI
	lms->fdplx_rx = kmalloc(sizeof(*lms->fdplx_rx), GFP_KERNEL | GFP_DMA);
	lms->fdplx_tx = kmalloc(sizeof(*lms->fdplx_tx), GFP_KERNEL | GFP_DMA);
	if (!lms->fdplx_rx || !lms->fdplx_tx)
		goto err;
#else
	lms->spi_buffer = kmalloc(LMS_MAX_RX_BUFF_LEN, GFP_KERNEL | GFP_DMA);
	if (!lms->spi_buffer)
		goto err;
#endif

	return lms;
err:
	free_lms(lms);
	return NULL;
}

int lms_spi_probe(struct spi_device *spi)
{
	struct lms_spi *lms = NULL;
	struct lms_spi **lms_p;
	struct net_device *lms_spi_devs = NULL;
	int ret = 0, irq;
#if  KERNEL_VERSION(5, 13, 0) > LINUX_VERSION_CODE
	const char *mac;
#else
	u8 mac[ETH_ALEN] = { 0 };
#endif
	struct lms_get_CG2H_mailbox mailbox_output;

	lms = alloc_lms();
	if (!lms)
		return -ENOMEM;

	spi_set_drvdata(spi, lms);

	lms->spi_dev = spi;

	if (!spi->dev.of_node) {
		dev_err(&spi->dev, "Missing device tree\n");
		free_lms(lms);
		return -EINVAL;
	}

	if (lms_spi_clkspeed)
		spi->max_speed_hz = lms_spi_clkspeed;
	else if (!spi->max_speed_hz)
		spi->max_speed_hz = LMS_SPI_DEFAULT_CLK_SPEED;

	if (spi->max_speed_hz < LMS_SPI_CLK_SPEED_MIN ||
	    spi->max_speed_hz > LMS_SPI_CLK_SPEED_MAX) {
		dev_err(&spi->dev, "Invalid clkspeed: %d\n", spi->max_speed_hz);
		free_lms(lms);
		return -EINVAL;
	}

	dev_info(&spi->dev, "SPI controller min possibled speed  : %uHz\n",
		 spi->controller->min_speed_hz);
	dev_info(&spi->dev, "SPI controller max possible speed   : %uHz\n",
		 spi->controller->max_speed_hz);
	dev_info(&spi->dev, "SPI lms driver configured max speed : %uHz\n",
		 spi->max_speed_hz);

	if (lms_force_spi_lsb_first_in_sw) {
		spi->mode &= ~SPI_LSB_FIRST;
	} else {
		if (!(spi->mode & SPI_LSB_FIRST) &&
		    of_property_read_bool(spi->dev.of_node, "spi-lsb-first"))
			/* In case the driver was executed previously with the 'force_lsb_first_in_sw' flag
			 * the spi->mode would not contain the SPI_LSB_FIRST flag as it was removed on last
			 * execution of the driver. The device tree node is not re-read on each probe. */
			spi->mode |= SPI_LSB_FIRST;
	}

	if (spi_setup(spi) < 0) {
		dev_err(&spi->dev, "Unable to setup SPI device\n");
		free_lms(lms);
		return -EFAULT;
	}

	lms->sw_lsb_first = false;
	if (!(spi->mode & SPI_LSB_FIRST)) {
		/* The SPI controller driver will send/receive data over SPI in MSB first mode (perhaps
		 * due to lack of support for LSB first). But the CG5317 works in LSB first mode -->
		 * therefore this driver will perform all bit reverses internally in software */
		lms->sw_lsb_first = true;
	}

	dev_info(&spi->dev, "Translation to 'LSB first' happens in %s\n",
		 (lms->sw_lsb_first) ? "software" : "hardware");

	atomic_set(&lms->rx_intr_req, 0);
	atomic_set(&lms->stat_intr_req, 0);
	atomic_set(&lms->spi_modem_status, 0);

	lms->spi_thread = NULL;
	lms->rx_completion_wq = NULL;
	/* Force a re-read of the TX FIFO Occupancy */
	lms->tx_occupancy = LMS_TX_MAX_OCCUPANCY_BYTES;
	lms->rx_packets_waiting = 0;

	/* Set gpio vars to -1 as a way to verify
	 * both will be set at the end
	 */
	lms->rx_gpio = -1;
	lms->stat_gpio = -1;
	atomic_set(&lms->H2CG_status_mailbox_zero, 0);
	atomic_set(&lms->H2CG_status_mailbox_one, 0);
	atomic_set(&lms->H2CG_status_mailbox_two, 0);
	atomic_set(&lms->H2CG_status_mailbox_three, 0);
	atomic_set(&lms->overflow_counter_tx, 0);
	atomic_set(&lms->overflow_counter_rx, 0);
	atomic_set(&lms->unexpected_behaviour_counter, 0);

	/* get rx irq number */
	ret = of_irq_get_byname(spi->dev.of_node, LMS_RX_IRQ_NAME);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: of_irq_get_byname %s failed err=%d\n",
			LMS_DRV_NAME, LMS_RX_IRQ_NAME, ret);
		free_lms(lms);
		return ret;
	}
	lms->rx_irq = ret;

	/* get stat0 irq number */
	ret = of_irq_get_byname(spi->dev.of_node, LMS_STAT0_IRQ_NAME);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: of_irq_get_byname %s failed err=%d\n",
			LMS_DRV_NAME, LMS_STAT0_IRQ_NAME, ret);
		free_lms(lms);
		return ret;
	}
	lms->stat_irq = ret;

	/* get gpio number from device tree */
	/* get first gpio number */
	ret = of_get_named_gpio(spi->dev.of_node, "interrupts-extended", 0);

	if (ret < 0) {
		dev_err(&spi->dev, "%s: of_get_named_gpio 0 failed err=%d\n",
			LMS_DRV_NAME, ret);
		free_lms(lms);
		return ret;
	}
	/* match gpio to irq */
	irq = gpio_to_irq(ret);
	if (lms->rx_gpio == -1 && irq == lms->rx_irq) {
		lms->rx_gpio = ret;
	} else if ((lms->stat_gpio == -1) && (irq == lms->stat_irq)) {
		lms->stat_gpio = ret;
	} else {
		dev_err(&spi->dev, "%s: irq %d of gpio %d does not fit rx or stat irq\n",
			LMS_DRV_NAME, irq, ret);
		return -1;
	}

	/* get second gpio number */
	ret = of_get_named_gpio(spi->dev.of_node, "interrupts-extended", 1);

	if (ret < 0) {
		dev_err(&spi->dev, "%s: of_get_named_gpio 1 failed err=%d\n",
			LMS_DRV_NAME, ret);
		free_lms(lms);
		return ret;
	}
	/* match gpio to irq */
	irq = gpio_to_irq(ret);
	if (lms->rx_gpio == -1 && irq == lms->rx_irq) {
		lms->rx_gpio = ret;
	} else if ((lms->stat_gpio == -1) && (irq == lms->rx_irq)) {
		lms->stat_gpio = ret;
	} else {
		dev_err(&spi->dev, "%s: irq %d of gpio %d does not fit rx or stat irq\n",
			LMS_DRV_NAME, irq, ret);
		return -1;
	}

	/* catch gpios and irqs */
	ret = gpio_request(lms->rx_gpio, LMS_DRV_NAME);
	if (ret) {
		dev_err(&spi->dev, "%s: unable to get gpio %d (irqval=%d)!\n",
			LMS_DRV_NAME, lms->rx_gpio, ret);
		free_lms(lms);
		return ret;
	}

	ret = gpiod_get_value(gpio_to_desc(lms->rx_gpio));
	if (ret == 1)
		atomic_set(&lms->rx_intr_req, 1);

	ret = request_irq(lms->rx_irq, lms_spi_intr_handler, 0,
			  LMS_DRV_NAME, lms);
	if (ret) {
		dev_err(&spi->dev, "%s: unable to get IRQ %d of gpio %d (irqval=%d)!\n",
			LMS_DRV_NAME, lms->rx_irq, lms->rx_gpio, ret);
		gpio_free(lms->rx_gpio);
		free_lms(lms);
		return ret;
	}

	enable_irq_wake(lms->rx_irq);

	ret = gpio_request(lms->stat_gpio, LMS_DRV_NAME);
	if (ret) {
		dev_err(&spi->dev, "%s: unable to get gpio %d (irqval=%d)!\n",
			LMS_DRV_NAME, lms->stat_gpio, ret);
		free_irq(lms->rx_irq, lms);
		gpio_free(lms->rx_gpio);
		free_lms(lms);
		return ret;
	}

	ret = request_irq(lms->stat_irq, lms_spi_intr_handler, 0,
			  LMS_DRV_NAME, lms);
	if (ret) {
		dev_err(&spi->dev, "%s: unable to get IRQ %d of gpio %d (irqval=%d)!\n",
			LMS_DRV_NAME, lms->stat_irq, lms->stat_gpio, ret);
		gpio_free(lms->stat_gpio);
		free_irq(lms->rx_irq, lms);
		gpio_free(lms->rx_gpio);
		free_lms(lms);
		return ret;
	}

	enable_irq_wake(lms->stat_irq);

	ret = lms_CG2H_mailboxs_init(lms);
	if (ret) {
		netdev_info(lms->net_dev, "Failed to init mailboxes.\n");
		free_irq(lms->stat_irq, lms);
		gpio_free(lms->stat_gpio);
		free_irq(lms->rx_irq, lms);
		gpio_free(lms->rx_gpio);
		free_lms(lms);
		return ret;
	}

	lms->spi_state = SPI_OK;
	if (lms_check_stat_zero(lms, true)) {
		dev_err(&spi->dev, "%s: Status 0 sanity failed\n", LMS_DRV_NAME);
		lms->spi_state = SPI_DISABLED;
	} else {
		dev_info(&spi->dev, "Status 0 sanity check passed\n");
	}

	/* read mailbox0 once on init for modem_status update */
	lms_get_CG2H_mailbox(&lms->CG2H_mailbox_zero, &mailbox_output);

	if (mailbox_output.val == -1 && lms->spi_state == SPI_OK)
		lms_update_modem_status_flag(lms);

	lms_spi_devs = alloc_etherdev(sizeof(struct lms_spi *));
	if (!lms_spi_devs) {
		lms_CG2H_mailboxs_uninit(lms);
		free_irq(lms->stat_irq, lms);
		gpio_free(lms->stat_gpio);
		free_irq(lms->rx_irq, lms);
		gpio_free(lms->rx_gpio);
		free_lms(lms);
		return -ENOMEM;
	}

	ret = lms_spi_netdev_setup(lms, lms_spi_devs);
	if (ret) {
		free_netdev(lms_spi_devs);
		lms_CG2H_mailboxs_uninit(lms);
		free_irq(lms->stat_irq, lms);
		gpio_free(lms->stat_gpio);
		free_irq(lms->rx_irq, lms);
		gpio_free(lms->rx_gpio);
		free_lms(lms);
		return ret;
	}
	SET_NETDEV_DEV(lms_spi_devs, &spi->dev);

	lms_p = netdev_priv(lms_spi_devs);
	*lms_p = lms;

	lms->net_dev = lms_spi_devs;

	/* Create notification sysfs */
	ret = lms_init_notif_sysfs(lms);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: Unable to init notification sysfs\n",
			LMS_DRV_NAME);
		free_netdev(lms_spi_devs);
		lms_CG2H_mailboxs_uninit(lms);
		free_irq(lms->stat_irq, lms);
		gpio_free(lms->stat_gpio);
		free_irq(lms->rx_irq, lms);
		gpio_free(lms->rx_gpio);
		free_lms(lms);
		return ret;
	}

#if KERNEL_VERSION(5, 13, 0) <=  LINUX_VERSION_CODE
	ret = of_get_mac_address(spi->dev.of_node, mac);
	if (ret) {
		eth_random_addr(mac);
		mac[0] = LMS_OUI_0;
		mac[1] = LMS_OUI_1;
		mac[2] = LMS_OUI_2;
		dev_info(&spi->dev, "Using random MAC address: %pM\n",
			 mac);
#if KERNEL_VERSION(5, 15, 0) <=  LINUX_VERSION_CODE
		eth_hw_addr_set(lms->net_dev, mac);
#else
		ether_addr_copy(lms->net_dev->dev_addr, mac);
#endif
	}
#else
	mac = of_get_mac_address(spi->dev.of_node);

	if (mac && !IS_ERR(mac))
		ether_addr_copy(lms->net_dev->dev_addr, mac);

	if (!is_valid_ether_addr(lms->net_dev->dev_addr)) {
		eth_hw_addr_random(lms->net_dev);
		lms->net_dev->dev_addr[0] = LMS_OUI_0;
		lms->net_dev->dev_addr[1] = LMS_OUI_1;
		lms->net_dev->dev_addr[2] = LMS_OUI_2;
		dev_info(&spi->dev, "Using random MAC address: %pM\n",
			 lms->net_dev->dev_addr);
	}
#endif

	lms->net_dev->mtu = LMS_FRM_MAX_MTU;
	lms->net_dev->type = ARPHRD_ETHER;

	ret = lms_init_tx_queue(&lms->queue);
	if (ret) {
		free_netdev(lms_spi_devs);
		lms_remove_notif_sysfs(lms);
		lms_CG2H_mailboxs_uninit(lms);
		free_irq(lms->stat_irq, lms);
		gpio_free(lms->stat_gpio);
		free_irq(lms->rx_irq, lms);
		gpio_free(lms->rx_gpio);
		free_lms(lms);
		dev_err(&spi->dev, "Failed to init tx_queue.\n");
		return ret;
	}

	if (register_netdev(lms_spi_devs)) {
		dev_err(&spi->dev, "Unable to register net device %s\n",
			lms_spi_devs->name);
		lms_destroy_tx_queue(lms);
		free_netdev(lms_spi_devs);
		lms_remove_notif_sysfs(lms);
		lms_CG2H_mailboxs_uninit(lms);
		free_irq(lms->stat_irq, lms);
		gpio_free(lms->stat_gpio);
		free_irq(lms->rx_irq, lms);
		gpio_free(lms->rx_gpio);
		free_lms(lms);
		return -EFAULT;
	}

	dev_info(&spi->dev, "Registered net device: %s", lms->net_dev->name);

	return 0;
}

#if KERNEL_VERSION(5, 18, 0) <=  LINUX_VERSION_CODE
static void
#else
static int
#endif
lms_spi_remove(struct spi_device *spi)
{
	struct lms_spi *lms = spi_get_drvdata(spi);
	struct net_device *lms_spi_devs = lms->net_dev;

	unregister_netdev(lms_spi_devs);
	lms_destroy_tx_queue(lms);
	free_netdev(lms_spi_devs);

	lms_remove_notif_sysfs(lms);
	lms_CG2H_mailboxs_uninit(lms);
	free_irq(lms->stat_irq, lms);
	gpio_free(lms->stat_gpio);
	free_irq(lms->rx_irq, lms);
	gpio_free(lms->rx_gpio);
	free_lms(lms);

#if KERNEL_VERSION(5, 18, 0) >  LINUX_VERSION_CODE
	return 0;
#endif
}

static const struct spi_device_id lms_spi_id[] = {
	{ .name = "cg5317", .driver_data = 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, lms_spi_id);
static struct spi_driver lms_spi_driver = {
	.driver = {
		.name = LMS_DRV_NAME,
		.of_match_table = lms_spi_of_match,
	},
	.id_table = lms_spi_id,
	.probe = lms_spi_probe,
	.remove = lms_spi_remove,
};

static int lms_spi_init_driver(struct spi_driver *sdrv)
{
	int ret;

	pr_info("Initialising %s version %s\n", LMS_DRV_NAME, LMS_SPI_DRV_VERSION);

	lms_probe_counter = 0;
	lms_root_device = root_device_register(LMS_DRV_NAME);
	if (IS_ERR(lms_root_device)) {
		return PTR_ERR(lms_root_device);
	}

	ret = spi_register_driver(sdrv);
	if (ret) {
		root_device_unregister(lms_root_device);
	}

	return ret;
}

static void lms_spi_exit_driver(struct spi_driver *sdrv)
{
	pr_info("Exiting %s version %s\n", LMS_DRV_NAME, LMS_SPI_DRV_VERSION);
	spi_unregister_driver(sdrv);
	root_device_unregister(lms_root_device);
}

MODULE_DESCRIPTION("Lumissil ETH2SPI Driver");
MODULE_AUTHOR("Lumissil Microsystem");
MODULE_AUTHOR("Orr Mazor <orr.mazor@gmail.com>");
MODULE_LICENSE("Dual MIT/GPL");

module_driver(lms_spi_driver, lms_spi_init_driver, lms_spi_exit_driver)
