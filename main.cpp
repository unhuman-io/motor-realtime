#include <iostream>
#include <libusb-1.0/libusb.h>

void cb(struct libusb_transfer *transfer) {
	transfer->buffer[4] = 0;
	printf("cb****************************************** ep:%x, len:%d data: %s\n",transfer->endpoint, transfer->actual_length, transfer->buffer);
}

int main() {
    std::cout << "Realtime process" << std::endl;

    libusb_context *ctx;
    libusb_init(&ctx);
    libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_DEBUG);

    uint16_t vid = 0x483;
    uint16_t pid = 0x5740;
libusb_device_handle *handle;
	libusb_device *dev;
	uint8_t bus, port_path[8];
	struct libusb_bos_descriptor *bos_desc;
	struct libusb_config_descriptor *conf_desc;
	const struct libusb_endpoint_descriptor *endpoint;
	int i, j, k, r;
	int iface, nb_ifaces, first_iface = -1;
	struct libusb_device_descriptor dev_desc;
	const char* const speed_name[5] = { "Unknown", "1.5 Mbit/s (USB LowSpeed)", "12 Mbit/s (USB FullSpeed)",
		"480 Mbit/s (USB HighSpeed)", "5000 Mbit/s (USB SuperSpeed)" };
	char string[128];
	uint8_t string_index[3];	// indexes of the string descriptors
	uint8_t endpoint_in = 0, endpoint_out = 0;	// default IN and OUT endpoints

	printf("Opening device %04X:%04X...\n", vid, pid);
	handle = libusb_open_device_with_vid_pid(NULL, vid, pid);

	if (handle == NULL) {
		printf("  Failed.\n");
		return -1;
	}

// 	dev = libusb_get_device(handle);
// 	bus = libusb_get_bus_number(dev);
// 	if (1) {
// 		r = libusb_get_port_numbers(dev, port_path, sizeof(port_path));
// 		if (r > 0) {
// 			printf("\nDevice properties:\n");
// 			printf("        bus number: %d\n", bus);
// 			printf("         port path: %d", port_path[0]);
// 			for (i=1; i<r; i++) {
// 				printf("->%d", port_path[i]);
// 			}
// 			printf(" (from root hub)\n");
// 		}
// 		r = libusb_get_device_speed(dev);
// 		if ((r<0) || (r>4)) r=0;
// 		printf("             speed: %s\n", speed_name[r]);
// 	}

// printf("\nReading device descriptor:\n");
// 	libusb_get_device_descriptor(dev, &dev_desc);
// 	printf("            length: %d\n", dev_desc.bLength);
// 	printf("      device class: %d\n", dev_desc.bDeviceClass);
// 	printf("               S/N: %d\n", dev_desc.iSerialNumber);
// 	printf("           VID:PID: %04X:%04X\n", dev_desc.idVendor, dev_desc.idProduct);
// 	printf("         bcdDevice: %04X\n", dev_desc.bcdDevice);
// 	printf("   iMan:iProd:iSer: %d:%d:%d\n", dev_desc.iManufacturer, dev_desc.iProduct, dev_desc.iSerialNumber);
// 	printf("          nb confs: %d\n", dev_desc.bNumConfigurations);
// 	// Copy the string descriptors for easier parsing
// 	string_index[0] = dev_desc.iManufacturer;
// 	string_index[1] = dev_desc.iProduct;
// 	string_index[2] = dev_desc.iSerialNumber;

//     printf("\nReading first configuration descriptor:\n");
// 	libusb_get_config_descriptor(dev, 0, &conf_desc);
// 	nb_ifaces = conf_desc->bNumInterfaces;
// 	printf("             nb interfaces: %d\n", nb_ifaces);
// 	if (nb_ifaces > 0)
// 		first_iface = conf_desc->interface[0].altsetting[0].bInterfaceNumber;
// 	for (i=0; i<nb_ifaces; i++) {
// 		printf("              interface[%d]: id = %d\n", i,
// 			conf_desc->interface[i].altsetting[0].bInterfaceNumber);
// 		for (j=0; j<conf_desc->interface[i].num_altsetting; j++) {
// 			printf("interface[%d].altsetting[%d]: num endpoints = %d\n",
// 				i, j, conf_desc->interface[i].altsetting[j].bNumEndpoints);
// 			printf("   Class.SubClass.Protocol: %02X.%02X.%02X\n",
// 				conf_desc->interface[i].altsetting[j].bInterfaceClass,
// 				conf_desc->interface[i].altsetting[j].bInterfaceSubClass,
// 				conf_desc->interface[i].altsetting[j].bInterfaceProtocol);
// 			if ( (conf_desc->interface[i].altsetting[j].bInterfaceClass == LIBUSB_CLASS_MASS_STORAGE)
// 			  && ( (conf_desc->interface[i].altsetting[j].bInterfaceSubClass == 0x01)
// 			  || (conf_desc->interface[i].altsetting[j].bInterfaceSubClass == 0x06) )
// 			  && (conf_desc->interface[i].altsetting[j].bInterfaceProtocol == 0x50) ) {
// 				// Mass storage devices that can use basic SCSI commands

// 			}
// 			for (k=0; k<conf_desc->interface[i].altsetting[j].bNumEndpoints; k++) {
// 				struct libusb_ss_endpoint_companion_descriptor *ep_comp = NULL;
// 				endpoint = &conf_desc->interface[i].altsetting[j].endpoint[k];
// 				printf("       endpoint[%d].address: %02X\n", k, endpoint->bEndpointAddress);
// 				// Use the first interrupt or bulk IN/OUT endpoints as default for testing
// 				if ((endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT)) {
// 					if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
// 						if (!endpoint_in)
// 							endpoint_in = endpoint->bEndpointAddress;
// 					} else {
// 						if (!endpoint_out)
// 							endpoint_out = endpoint->bEndpointAddress;
// 					}
// 				}
// 				printf("           max packet size: %04X\n", endpoint->wMaxPacketSize);
// 				printf("          polling interval: %02X\n", endpoint->bInterval);
// 				libusb_get_ss_endpoint_companion_descriptor(NULL, endpoint, &ep_comp);
// 				if (ep_comp) {
// 					printf("                 max burst: %02X   (USB 3.0)\n", ep_comp->bMaxBurst);
// 					printf("        bytes per interval: %04X (USB 3.0)\n", ep_comp->wBytesPerInterval);
// 					libusb_free_ss_endpoint_companion_descriptor(ep_comp);
// 				}
// 			}
// 		}
// 	}
// 	libusb_free_config_descriptor(conf_desc);

// 	libusb_set_auto_detach_kernel_driver(handle, 1);
//     	for (iface = 0; iface < nb_ifaces; iface++)
// 	{
// 		printf("\nClaiming interface %d...\n", iface);
// 		r = libusb_claim_interface(handle, iface);
// 		if (r != LIBUSB_SUCCESS) {
// 			printf("   Failed.\n");
// 		}
// 	}
//     	printf("\nReading string descriptors:\n");
// 	for (i=0; i<3; i++) {
// 		if (string_index[i] == 0) {
// 			continue;
// 		}
// 		if (libusb_get_string_descriptor_ascii(handle, string_index[i], (unsigned char*)string, sizeof(string)) > 0) {
// 			printf("   String (0x%02X): \"%s\"\n", string_index[i], string);
// 		}
// 	}



    
		unsigned char *buf1 = (unsigned char*) malloc(64 + 1);
		struct libusb_transfer *transfer1;
	transfer1 = libusb_alloc_transfer(0);
	if (!transfer1) {
		free(buf1);
		return -1;
	}

	libusb_fill_bulk_transfer(transfer1, handle, 0x82, buf1, 64, cb, NULL, 1000);
    for(int i=0; i<3; i++) {
        libusb_submit_transfer(transfer1);
        r = libusb_handle_events(ctx);
    }

    libusb_close(handle);
    libusb_exit(ctx);

    
    return 0;
}