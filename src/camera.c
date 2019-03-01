#include <zephyr.h>
#include <misc/printk.h>
#include <misc/byteorder.h>

#include <device.h>
#include <i2c.h>

#include "video/fsl_camera.h"
#include "video/fsl_mt9m114.h"
#include "video/fsl_camera_receiver.h"
#include "video/fsl_camera_device.h"
#include "video/fsl_csi_camera_adapter.h"

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"

#include "camera.h"

#define APP_CAMERA_HEIGHT 272
#define APP_CAMERA_WIDTH 480
#define APP_CAMERA_CONTROL_FLAGS (kCAMERA_HrefActiveHigh | kCAMERA_DataLatchOnRisingEdge)

/* Pixel format YUV422, bytesPerPixel is 2. */
#define APP_BPP 2
#define APP_CAMERA_FRAME_BUFFER_COUNT 4
#define FRAME_BUFFER_ALIGN 64

struct device *i2c_dev;

/* CAMERA CONFIG */
const camera_config_t cameraConfig = {
	.pixelFormat = kVIDEO_PixelFormatRGB565,
	.bytesPerPixel = APP_BPP,
	.resolution = FSL_VIDEO_RESOLUTION(APP_CAMERA_WIDTH, APP_CAMERA_HEIGHT),
	.frameBufferLinePitch_Bytes = APP_CAMERA_WIDTH * APP_BPP,
	.interface = kCAMERA_InterfaceGatedClock,
	.controlFlags = APP_CAMERA_CONTROL_FLAGS,
	.framePerSec = 30,
};

status_t camera_i2c_send(uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, const uint8_t *txBuff, uint8_t txBuffSize)
{
	if (subAddressSize == 2)
		subAddress = __bswap_16(subAddress);

	printk("Write %d bytes to %02x\n", txBuffSize, subAddress);
	return i2c_burst_write16(i2c_dev, deviceAddress, subAddress, txBuff, txBuffSize);

	return 0;
}

status_t camera_i2c_receive(uint8_t deviceAddress, uint32_t subAddress, uint8_t subAddressSize, uint8_t *rxBuff, uint8_t rxBuffSize)
{
	if (subAddressSize == 2)
		subAddress = __bswap_16(subAddress);

	printk("Read %d bytes to %02x\n", rxBuffSize, subAddress);
	return i2c_burst_read16(i2c_dev, deviceAddress, subAddress, rxBuff, rxBuffSize);
}

static void camera_reset(bool pullUp)
{
    /* Reset pin is connected to DCDC_3V3. */
    return;
}

static mt9m114_resource_t mt9m114Resource = {
    .i2cSendFunc = camera_i2c_send,
    .i2cReceiveFunc = camera_i2c_receive,
    .pullResetPin = camera_reset,
    .inputClockFreq_Hz = 24000000, /* 24MHz */
};

/* CSI CONFIG */
camera_device_handle_t cameraDevice = {
    .resource = &mt9m114Resource, .ops = &mt9m114_ops,
};

static csi_resource_t csiResource = {
    .csiBase = CSI,
};

static csi_private_data_t csiPrivateData;

camera_receiver_handle_t cameraReceiver = {
    .resource = &csiResource, .ops = &csi_ops, .privateData = &csiPrivateData,
};

/* CAPTURE BUFFERS */
static __nocache uint16_t __aligned(FRAME_BUFFER_ALIGN)
s_cameraBuffer[APP_CAMERA_FRAME_BUFFER_COUNT][APP_CAMERA_HEIGHT][APP_CAMERA_WIDTH];

extern void CSI_DriverIRQHandler(void);
static void csi_mcux_isr(void *p)
{
	printk("CSI IRQ: %u ms\n", k_uptime_get_32());

	/* Forward to fsl_csi */
	CSI_DriverIRQHandler();
}

void camera_run(void)
{
	uint32_t activeFrameAddr;
	int status;

	/* Configure extra clock (move this to soc.c mimxrt) */
	CLOCK_DisableClock(kCLOCK_Csi); /* Disable CSI clock gate */
	CLOCK_SetDiv(kCLOCK_CsiDiv, 0); /* Set CSI divider to 1 */
	CLOCK_SetMux(kCLOCK_CsiMux, 0); /* Set CSI source to OSC 24M */

	/* Configure Extra pinmux (move this to mimxrt1064 pinmux.c) */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_04_CSI_PIXCLK, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_05_CSI_MCLK, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_CSI_VSYNC, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_07_CSI_HSYNC, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_CSI_DATA09, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_CSI_DATA08, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_CSI_DATA07, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_11_CSI_DATA06, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_12_CSI_DATA05, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_13_CSI_DATA04, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_CSI_DATA03, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_CSI_DATA02, 0);

	/* Register CSI IRQ */
	IRQ_CONNECT(43, 0, csi_mcux_isr, NULL, 0);
	irq_enable(43);


	/* I2C_1 */
	i2c_dev = device_get_binding(ARDUINO_I2C_LABEL);
	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}

	/* CSI INIT */
	status = CAMERA_RECEIVER_Init(&cameraReceiver, &cameraConfig, NULL, NULL);
	if (status)
	{
		printk("CAMERA_RECEIVER_Init ERROR\n");
		return;
	}

	/* MT9M114 INIT */
	status = CAMERA_DEVICE_Init(&cameraDevice, &cameraConfig);
	if (status)
	{
		printk("CAMERA_DEVICE_Init ERROR\n");
		return;
	}

	/* MT9M114 start streaming */
	status = CAMERA_DEVICE_Start(&cameraDevice);
	if (status)
	{
		printk("CAMERA_DEVICE_Start ERROR\n");
		return;
	}

	/* Submit the empty frame buffers to buffer queue. */
	for (uint32_t i = 0; i < APP_CAMERA_FRAME_BUFFER_COUNT; i++)
	{
		printk("Submit capture buffer %p\n", s_cameraBuffer[i]);
		status = CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(s_cameraBuffer[i]));
		if (status)
		{
			printk("CAMERA_RECEIVER_SubmitEmptyBuffer ERROR\n");
			return;
		}
	}

	/* Start CSI capture */
	status = CAMERA_RECEIVER_Start(&cameraReceiver);
	if (status) {
		printk("CAMERA_RECEIVER_Start ERROR\n");
		return;
	}

	printk("Camera Started!\n");

	/* Capture Loop */
	while (1)
	{
		/* Wait to get the full frame buffer to show. */
		while (kStatus_Success != CAMERA_RECEIVER_GetFullBuffer(&cameraReceiver, &activeFrameAddr))
		{
		}

		printk("Frame received\n");

		/* Resubmit Frame */
		CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, activeFrameAddr);
    	}
}
