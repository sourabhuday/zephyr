/*
 * Copyright (c) 2018, NXP
 * Copyright (c) 2018, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>
#include <device.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <openamp/open_amp.h>
#include <metal/device.h>

#define APP_TASK_STACK_SIZE (512)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

static struct rpmsg_channel *rp_channel;
static struct rpmsg_endpoint *rp_endpoint;

static K_SEM_DEFINE(channel_created, 0, 1);

static K_SEM_DEFINE(message_received, 0, 1);
static volatile unsigned int received_data;

static struct rsc_table_info rsc_info;

#define SHM_START_ADDRESS       0x04000400
#define SHM_SIZE                0x7c00
#define SHM_DEVICE_NAME         "sramx.shm"

#define VRING_COUNT             2
#define VRING_RX_ADDRESS        0x04000400
#define VRING_TX_ADDRESS        0x04000800
#define VRING_ALIGNMENT         4
#define VRING_SIZE              32

static metal_phys_addr_t shm_physmap[] = { SHM_START_ADDRESS };
static struct metal_device shm_device = {
	.name = SHM_DEVICE_NAME,
	.bus = NULL,
	.num_regions = 1,
	{
		{
			.virt       = (void *) SHM_START_ADDRESS,
			.physmap    = shm_physmap,
			.size       = SHM_SIZE,
			.page_shift = 0xffffffff,
			.page_mask  = 0xffffffff,
			.mem_flags  = 0,
			.ops        = { NULL },
		},
	},
	.node = { NULL },
	.irq_num = 0,
	.irq_info = NULL
};

#define RSC_TABLE_ADDRESS       0x04000000

OPENAMP_PACKED_BEGIN
struct lpc_resource_table {
	uint32_t ver;
	uint32_t num;
	uint32_t reserved[2];
	uint32_t offset[2];
	struct fw_rsc_rproc_mem mem;
	struct fw_rsc_vdev vdev;
	struct fw_rsc_vdev_vring vring0, vring1;
} OPENAMP_PACKED_END;

struct lpc_resource_table *rsc_table_ptr = (void *) RSC_TABLE_ADDRESS;

#if defined(CPU_LPC54114J256BD64_cm4)
static const struct lpc_resource_table rsc_table = {
	.ver = 1,
	.num = 2,
	.offset = {
		offsetof(struct lpc_resource_table, mem),
		offsetof(struct lpc_resource_table, vdev),
	},
	.mem = { RSC_RPROC_MEM, SHM_START_ADDRESS, SHM_START_ADDRESS, SHM_SIZE, 0 },
	.vdev = { RSC_VDEV, VIRTIO_ID_RPMSG, 0, 1 << VIRTIO_RPMSG_F_NS, 0, 0, 0, VRING_COUNT, { 0, 0 } },
	.vring0 = { VRING_TX_ADDRESS, VRING_ALIGNMENT, VRING_SIZE, 1, 0 },
	.vring1 = { VRING_RX_ADDRESS, VRING_ALIGNMENT, VRING_SIZE, 2, 0 },
};
#endif

void resource_table_init(void **table_ptr, int *length)
{
#if defined(CPU_LPC54114J256BD64_cm4)
	/* Master: copy the resource table to shared memory. */
	memcpy(rsc_table_ptr, &rsc_table, sizeof(struct lpc_resource_table));
#endif

	*length = sizeof(struct lpc_resource_table);
	*table_ptr = rsc_table_ptr;
}

static void rpmsg_recv_callback(struct rpmsg_channel *channel, void *data,
				int data_length, void *private, unsigned long src)
{
	received_data = *((unsigned int *) data);
	k_sem_give(&message_received);
}

static void rpmsg_channel_created(struct rpmsg_channel *channel)
{
	rp_channel = channel;
	rp_endpoint = rpmsg_create_ept(rp_channel, rpmsg_recv_callback, RPMSG_NULL, RPMSG_ADDR_ANY);
	k_sem_give(&channel_created);
}

static void rpmsg_channel_deleted(struct rpmsg_channel *channel)
{
	rpmsg_destroy_ept(rp_endpoint);
}

static unsigned int receive_message(void)
{
	while (k_sem_take(&message_received, K_NO_WAIT) != 0) {
		;
#if 0
		hil_poll(proc, 0);
#endif
	}
	return received_data;
}

static int send_message(unsigned int message)
{
	return rpmsg_send(rp_channel, &message, sizeof(message));
}

void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	int status = 0;

	printk("\r\nOpenAMP demo started\r\n");

	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	metal_init(&metal_params);

	status = metal_register_generic_device(&shm_device);
	if (status != 0) {
		printk("metal_register_generic_device(): could not register shared memory device: error code %d\n", status);
		goto _cleanup;
	}

	resource_table_init((void **) &rsc_info.rsc_tab, &rsc_info.size);

	while (k_sem_take(&channel_created, K_NO_WAIT) != 0) {
#if 0
		hil_poll(proc, 0);
#endif
		;
	}

	unsigned int message = 0;
	status = send_message(message);
	if (status < 0) {
		printk("send_message(%d) failed with status %d\n", message, status);
		goto _cleanup;
	}

	while (message <= 100) {
		message = receive_message();
		printk("Primary core received a message: %d\n", message);

		message++;
		status = send_message(message);
		if (status < 0) {
			printk("send_message(%d) failed with status %d\n", message, status);
			goto _cleanup;
		}
	}

_cleanup:
	metal_finish();

	printk("OpenAMP demo ended.\n");
}

void main(void)
{
	printk("Starting application thread!\n");
	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);
}
