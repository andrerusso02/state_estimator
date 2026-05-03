#include "udp_stream.h"
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(udp_stream, LOG_LEVEL_INF);


#if DT_NODE_HAS_STATUS(DT_NODELABEL(usbotg_fs), okay)

#include <zephyr/usb/usbd.h>


#include <zephyr/usb/usb_ch9.h> /* Contains USB_SCD_ macros */

/* 1. Define the Device Context */
/* Using 'usbotg_fs' directly to match your H750 hardware node */
USBD_DEVICE_DEFINE(my_usbd, 
                   DEVICE_DT_GET(DT_NODELABEL(usbotg_fs)), 
                   0x0483, 0x0001);

/* 2. Define Basic String Descriptors */
USBD_DESC_LANG_DEFINE(my_lang);
USBD_DESC_MANUFACTURER_DEFINE(my_mfr, "Zephyr");
USBD_DESC_PRODUCT_DEFINE(my_prdc, "DD's NCM Network Interface");

/* 3. Define the Full-Speed Configuration */


USBD_DESC_CONFIG_DEFINE(my_cfg_desc, "Default Config");

/* Fixed macro: removed '_ATTRIBUTES_' from the constant name */
USBD_CONFIGURATION_DEFINE(my_fs_cfg, 
                          USB_SCD_SELF_POWERED, 
                          100, 
                          &my_cfg_desc);
int start_usb_networking(void)
{
    int err;
    struct usbd_context *ctx = &my_usbd;

    /* Add Mandatory String Descriptors */
    usbd_add_descriptor(ctx, &my_lang);
    usbd_add_descriptor(ctx, &my_mfr);
    usbd_add_descriptor(ctx, &my_prdc);

    /* Setup Device Triple for IAD (Required for CDC NCM/Compound devices) */
    /* 0xEF = Miscellaneous, 0x02 = Common Class, 0x01 = Interface Association */
    usbd_device_set_code_triple(ctx, USBD_SPEED_FS, 0xEF, 0x02, 0x01);

    /* Bind Configuration and Register NCM Class */
    err = usbd_add_configuration(ctx, USBD_SPEED_FS, &my_fs_cfg);
    if (err) return err;

    /* Register all classes enabled in prj.conf (e.g. CDC NCM) */
    err = usbd_register_all_classes(ctx, USBD_SPEED_FS, 1, NULL);
    if (err) return err;

    /* Initialize and Fire it up */
    err = usbd_init(ctx);
    if (err) return err;

    return usbd_enable(ctx);
}
#endif


static int udp_sock = -1;
static struct sockaddr_in destination_addr;

int udp_stream_init(const char *dest_ip, uint16_t port)
{
    if (udp_sock >= 0) {
        LOG_WRN("UDP stream already initialized.");
        return 0;
    }

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usbotg_fs), okay)

	start_usb_networking();
	printk("USB device enabled\n");
	
#endif

    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_sock < 0) {
        LOG_ERR("Failed to create UDP socket: %d", udp_sock);
        return udp_sock;
    }

    destination_addr.sin_family = AF_INET;
    destination_addr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, dest_ip, &destination_addr.sin_addr) <= 0) {
        LOG_ERR("Invalid destination IP address: %s", dest_ip);
        close(udp_sock);
        udp_sock = -1;
        return -EINVAL;
    }

    LOG_INF("UDP stream initialized targeting %s:%u", dest_ip, port);
    return 0;
}

int udp_stream_send(const void *data, size_t length)
{
    if (udp_sock < 0 || data == NULL || length == 0) {
        return -EINVAL;
    }

    int ret = sendto(udp_sock, data, length, 0, 
                     (struct sockaddr *)&destination_addr, 
                     sizeof(destination_addr));

    if (ret < 0) {
        LOG_ERR("UDP send failed: %d", errno);
        return -errno;
    }

    return ret;
}