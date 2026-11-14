/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2014 Aurelien Jacobs <aurel@gnuage.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <inttypes.h>
#include <string.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "scpi.h"

#define LOG_PREFIX "scpi_usbtmc"

#define MAX_TRANSFER_LENGTH 2048
#define TRANSFER_TIMEOUT 1000
/* Stream reading: bulk IN transfers kept in flight at most. */
#define STREAM_TRANSFERS 16

struct scpi_usbtmc_libusb {
	struct sr_context *ctx;
	struct sr_usb_dev_inst *usb;
	int detached_kernel_driver;
	uint8_t interface;
	uint8_t bulk_in_ep;
	uint8_t bulk_out_ep;
	uint8_t interrupt_ep;
	uint8_t usbtmc_int_cap;
	uint8_t usbtmc_dev_cap;
	uint8_t usb488_dev_cap;
	uint8_t bTag;
	uint8_t bulkin_attributes;
	uint8_t buffer[MAX_TRANSFER_LENGTH];
	uint16_t bulk_in_max_packet;
	int32_t max_in_message;
	/* Stream reading, see scpi_usbtmc_stream_fill(). */
	gboolean stream;
	struct libusb_transfer *xfer[STREAM_TRANSFERS];
	int xfer_done[STREAM_TRANSFERS];
	int xfer_head, xfer_count;
	uint8_t *fifo;
	size_t fifo_start, fifo_end;
	/* Current message: size with header (0 = header not seen yet), received. */
	size_t msg_total, msg_received;
	int response_length;
	int response_bytes_read;
	int remaining_length;
};

/* Some USBTMC-specific enums, as defined in the USBTMC standard. */
#define SUBCLASS_USBTMC 0x03
#define USBTMC_USB488   0x01

enum {
	/* USBTMC control requests */
	INITIATE_ABORT_BULK_OUT     =   1,
	CHECK_ABORT_BULK_OUT_STATUS =   2,
	INITIATE_ABORT_BULK_IN      =   3,
	CHECK_ABORT_BULK_IN_STATUS  =   4,
	INITIATE_CLEAR              =   5,
	CHECK_CLEAR_STATUS          =   6,
	GET_CAPABILITIES            =   7,
	INDICATOR_PULSE             =  64,

	/* USB488 control requests */
	READ_STATUS_BYTE            = 128,
	REN_CONTROL                 = 160,
	GO_TO_LOCAL                 = 161,
	LOCAL_LOCKOUT               = 162,
};

/* USBTMC status codes */
#define USBTMC_STATUS_SUCCESS      0x01

/* USBTMC capabilities */
#define USBTMC_INT_CAP_LISTEN_ONLY 0x01
#define USBTMC_INT_CAP_TALK_ONLY   0x02
#define USBTMC_INT_CAP_INDICATOR   0x04

#define USBTMC_DEV_CAP_TERMCHAR    0x01

#define USB488_DEV_CAP_DT1         0x01
#define USB488_DEV_CAP_RL1         0x02
#define USB488_DEV_CAP_SR1         0x04
#define USB488_DEV_CAP_SCPI        0x08

/* Bulk messages constants */
#define USBTMC_BULK_HEADER_SIZE 12

/* Bulk MsgID values */
#define DEV_DEP_MSG_OUT        1
#define REQUEST_DEV_DEP_MSG_IN 2
#define DEV_DEP_MSG_IN         2

/* bmTransferAttributes */
#define EOM               0x01
#define TERM_CHAR_ENABLED 0x02

struct usbtmc_blacklist {
	uint16_t vid;
	uint16_t pid;
};

/* Devices that publish RL1 support, but don't support it. */
static struct usbtmc_blacklist blacklist_remote[] = {
	{ 0x1ab1, 0x0588 }, /* Rigol DS1000 series */
	{ 0x1ab1, 0x04b0 }, /* Rigol DS2000 series */
	{ 0x1ab1, 0x04b1 }, /* Rigol DS4000 series */
	{ 0x1ab1, 0x0515 }, /* Rigol MSO5000 series */
	{ 0x0957, 0x0588 }, /* Agilent DSO1000 series (rebadged Rigol DS1000) */
	{ 0x0b21, 0xffff }, /* All Yokogawa devices */
	{ 0xf4ec, 0xffff }, /* All Siglent SDS devices */
	ALL_ZERO
};

/* Devices that shall get reset during open(). */
static struct usbtmc_blacklist whitelist_usb_reset[] = {
	{ 0xf4ec, 0xffff }, /* All Siglent SDS devices */
	ALL_ZERO
};

static GSList *scpi_usbtmc_libusb_scan(struct drv_context *drvc)
{
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	struct libusb_config_descriptor *confdes;
	const struct libusb_interface_descriptor *intfdes;
	GSList *resources = NULL;
	int confidx, intfidx, ret, i;
	char *res;

	ret = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (ret < 0) {
		sr_err("Failed to get device list: %s.",
		       libusb_error_name(ret));
		return NULL;
	}
	for (i = 0; devlist[i]; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		for (confidx = 0; confidx < des.bNumConfigurations; confidx++) {
			if ((ret = libusb_get_config_descriptor(devlist[i], confidx, &confdes)) < 0) {
				if (ret != LIBUSB_ERROR_NOT_FOUND)
					sr_dbg("Failed to get configuration descriptor: %s, "
					       "ignoring device.", libusb_error_name(ret));
				break;
			}
			for (intfidx = 0; intfidx < confdes->bNumInterfaces; intfidx++) {
				intfdes = confdes->interface[intfidx].altsetting;
				if (intfdes->bInterfaceClass    != LIBUSB_CLASS_APPLICATION ||
				    intfdes->bInterfaceSubClass != SUBCLASS_USBTMC          ||
				    intfdes->bInterfaceProtocol != USBTMC_USB488)
					continue;
				sr_dbg("Found USBTMC device (VID:PID = %04x:%04x, "
				       "bus.address = %d.%d).", des.idVendor, des.idProduct,
				       libusb_get_bus_number(devlist[i]),
				       libusb_get_device_address(devlist[i]));
				res = g_strdup_printf("usbtmc/%d.%d",
				                      libusb_get_bus_number(devlist[i]),
				                      libusb_get_device_address(devlist[i]));
				resources = g_slist_append(resources, res);
			}
			libusb_free_config_descriptor(confdes);
		}
	}
	libusb_free_device_list(devlist, 1);

	/* No log message for #devices found (caller will log that). */

	return resources;
}

static int scpi_usbtmc_libusb_dev_inst_new(void *priv, struct drv_context *drvc,
		const char *resource, char **params, const char *serialcomm)
{
	struct scpi_usbtmc_libusb *uscpi = priv;
	GSList *devices;

	(void)resource;
	(void)serialcomm;

	if (!params || !params[1]) {
		sr_err("Invalid parameters.");
		return SR_ERR;
	}

	uscpi->ctx = drvc->sr_ctx;
	devices = sr_usb_find(uscpi->ctx->libusb_ctx, params[1]);
	if (g_slist_length(devices) != 1) {
		sr_err("Failed to find USB device '%s'.", params[1]);
		g_slist_free_full(devices, (GDestroyNotify)sr_usb_dev_inst_free);
		return SR_ERR;
	}
	uscpi->usb = devices->data;
	g_slist_free(devices);

	return SR_OK;
}

static int check_usbtmc_blacklist(struct usbtmc_blacklist *blacklist,
		uint16_t vid, uint16_t pid)
{
	int i;

	for (i = 0; blacklist[i].vid; i++) {
		if ((blacklist[i].vid == vid && blacklist[i].pid == 0xFFFF) ||
			(blacklist[i].vid == vid && blacklist[i].pid == pid))
			return TRUE;
	}

	return FALSE;
}

static int scpi_usbtmc_remote(struct scpi_usbtmc_libusb *uscpi)
{
	struct sr_usb_dev_inst *usb = uscpi->usb;
	struct libusb_device *dev;
	struct libusb_device_descriptor des;
	int ret;
	uint8_t status;

	if (!(uscpi->usb488_dev_cap & USB488_DEV_CAP_RL1))
		return SR_OK;

	dev = libusb_get_device(usb->devhdl);
	libusb_get_device_descriptor(dev, &des);
	if (check_usbtmc_blacklist(blacklist_remote, des.idVendor, des.idProduct))
		return SR_OK;

	sr_dbg("Locking out local control.");
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_ENDPOINT_IN |
		LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
		REN_CONTROL, 1, uscpi->interface, &status, 1, TRANSFER_TIMEOUT);
	if (ret < 0 || status != USBTMC_STATUS_SUCCESS) {
		if (ret < 0)
			sr_dbg("Failed to enter REN state: %s.", libusb_error_name(ret));
		else
			sr_dbg("Failed to enter REN state: USBTMC status %d.", status);
		return SR_ERR;
	}

	ret = libusb_control_transfer(usb->devhdl, LIBUSB_ENDPOINT_IN |
		LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
		LOCAL_LOCKOUT, 0, uscpi->interface, &status, 1,
		TRANSFER_TIMEOUT);
	if (ret < 0 || status != USBTMC_STATUS_SUCCESS) {
		if (ret < 0)
			sr_dbg("Failed to enter local lockout state: %s.",
					libusb_error_name(ret));
		else
			sr_dbg("Failed to enter local lockout state: USBTMC "
					"status %d.", status);
		return SR_ERR;
	}

	return SR_OK;
}

static void scpi_usbtmc_local(struct scpi_usbtmc_libusb *uscpi)
{
	struct sr_usb_dev_inst *usb = uscpi->usb;
	struct libusb_device *dev;
	struct libusb_device_descriptor des;
	int ret;
	uint8_t status;

	if (!(uscpi->usb488_dev_cap & USB488_DEV_CAP_RL1))
		return;

	dev = libusb_get_device(usb->devhdl);
	libusb_get_device_descriptor(dev, &des);
	if (check_usbtmc_blacklist(blacklist_remote, des.idVendor, des.idProduct))
		return;

	sr_dbg("Returning local control.");
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_ENDPOINT_IN |
		LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
		GO_TO_LOCAL, 0, uscpi->interface, &status, 1, TRANSFER_TIMEOUT);
	if (ret < 0 || status != USBTMC_STATUS_SUCCESS) {
		if (ret < 0)
			sr_dbg("Failed to clear local lockout state: %s.",
					libusb_error_name(ret));
		else
			sr_dbg("Failed to clear local lockout state: USBTMC "
					"status %d.", status);
	}

	return;
}

static int scpi_usbtmc_libusb_open(struct sr_scpi_dev_inst *scpi)
{
	struct scpi_usbtmc_libusb *uscpi = scpi->priv;
	struct sr_usb_dev_inst *usb = uscpi->usb;
	struct libusb_device *dev;
	struct libusb_device_descriptor des;
	struct libusb_config_descriptor *confdes;
	const struct libusb_interface_descriptor *intfdes;
	const struct libusb_endpoint_descriptor *ep;
	int confidx, intfidx, epidx, config = 0, current_config;
	uint8_t capabilities[24];
	int ret, found = 0;
	int do_reset;

	if (usb->devhdl)
		return SR_OK;

	if (sr_usb_open(uscpi->ctx->libusb_ctx, usb) != SR_OK)
		return SR_ERR;

	dev = libusb_get_device(usb->devhdl);
	libusb_get_device_descriptor(dev, &des);

	for (confidx = 0; confidx < des.bNumConfigurations; confidx++) {
		if ((ret = libusb_get_config_descriptor(dev, confidx, &confdes)) < 0) {
			if (ret != LIBUSB_ERROR_NOT_FOUND)
				sr_dbg("Failed to get configuration descriptor: %s, "
				       "ignoring device.", libusb_error_name(ret));
			continue;
		}
		for (intfidx = 0; intfidx < confdes->bNumInterfaces; intfidx++) {
			intfdes = confdes->interface[intfidx].altsetting;
			if (intfdes->bInterfaceClass    != LIBUSB_CLASS_APPLICATION ||
			    intfdes->bInterfaceSubClass != SUBCLASS_USBTMC ||
			    intfdes->bInterfaceProtocol != USBTMC_USB488)
				continue;
			uscpi->interface = intfdes->bInterfaceNumber;
			config = confdes->bConfigurationValue;
			sr_dbg("Interface %d configuration %d.", uscpi->interface, config);
			for (epidx = 0; epidx < intfdes->bNumEndpoints; epidx++) {
				ep = &intfdes->endpoint[epidx];
				if (ep->bmAttributes == LIBUSB_TRANSFER_TYPE_BULK &&
				    !(ep->bEndpointAddress & (LIBUSB_ENDPOINT_DIR_MASK))) {
					uscpi->bulk_out_ep = ep->bEndpointAddress;
					sr_dbg("Bulk OUT EP %d", uscpi->bulk_out_ep);
				}
				if (ep->bmAttributes == LIBUSB_TRANSFER_TYPE_BULK &&
				    ep->bEndpointAddress & (LIBUSB_ENDPOINT_DIR_MASK)) {
					uscpi->bulk_in_ep = ep->bEndpointAddress;
					uscpi->bulk_in_max_packet = ep->wMaxPacketSize & 0x7ff;
					sr_dbg("Bulk IN EP %d", uscpi->bulk_in_ep & 0x7f);
				}
				if (ep->bmAttributes == LIBUSB_TRANSFER_TYPE_INTERRUPT &&
				    ep->bEndpointAddress & (LIBUSB_ENDPOINT_DIR_MASK)) {
					uscpi->interrupt_ep = ep->bEndpointAddress;
					sr_dbg("Interrupt EP %d", uscpi->interrupt_ep & 0x7f);
				}
			}
			found = 1;
		}
		libusb_free_config_descriptor(confdes);
		if (found)
			break;
	}

	if (!found) {
		sr_err("Failed to find USBTMC interface.");
		return SR_ERR;
	}

	/*
	 * Some devices (e.g. the Rigol DS1000Z series) declare 64 byte bulk
	 * endpoints also at high speed, where bulk endpoints must have 512
	 * bytes. Such a device sends stale buffer contents when a
	 * DEV_DEP_MSG_IN message is longer than one 512 byte packet, without
	 * any error. Ask for at most one packet's worth of data per message
	 * (header included).
	 */
	uscpi->max_in_message = INT32_MAX;
	if (libusb_get_device_speed(dev) == LIBUSB_SPEED_HIGH &&
	    uscpi->bulk_in_max_packet < 512) {
		uscpi->max_in_message = 512 - USBTMC_BULK_HEADER_SIZE;
		sr_info("Bulk IN EP has a max packet size of %d bytes at high "
			"speed (must be 512); limiting messages to %d bytes.",
			uscpi->bulk_in_max_packet, uscpi->max_in_message);
		/*
		 * Each transfer of such a device ends after one 64 byte
		 * packet. Read with transfers kept in flight, so the device
		 * is polled without gaps (also between the short messages).
		 */
		uscpi->stream = TRUE;
	}

	if (libusb_kernel_driver_active(usb->devhdl, uscpi->interface) == 1) {
		if ((ret = libusb_detach_kernel_driver(usb->devhdl,
		                                       uscpi->interface)) < 0) {
			sr_err("Failed to detach kernel driver: %s.",
			       libusb_error_name(ret));
			return SR_ERR;
		}
		uscpi->detached_kernel_driver = 1;
	}

	if (libusb_get_configuration(usb->devhdl, &current_config) == 0
	    && current_config != config) {
		if ((ret = libusb_set_configuration(usb->devhdl, config)) < 0) {
			sr_err("Failed to set configuration: %s.",
			       libusb_error_name(ret));
			return SR_ERR;
		}
	}

	if ((ret = libusb_claim_interface(usb->devhdl, uscpi->interface)) < 0) {
		sr_err("Failed to claim interface: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}

	/* Optionally reset the USB device. */
	do_reset = check_usbtmc_blacklist(whitelist_usb_reset,
		des.idVendor, des.idProduct);
	if (do_reset)
		libusb_reset_device(usb->devhdl);

	/* Get capabilities. */
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_ENDPOINT_IN |
		LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
		GET_CAPABILITIES, 0, uscpi->interface, capabilities,
		sizeof(capabilities), TRANSFER_TIMEOUT);
	if (ret == sizeof(capabilities)) {
		uscpi->usbtmc_int_cap = capabilities[ 4];
		uscpi->usbtmc_dev_cap = capabilities[ 5];
		uscpi->usb488_dev_cap = capabilities[15];
	}
	sr_dbg("Device capabilities: %s%s%s%s%s, %s, %s",
	       uscpi->usb488_dev_cap & USB488_DEV_CAP_SCPI        ? "SCPI, "    : "",
	       uscpi->usbtmc_dev_cap & USBTMC_DEV_CAP_TERMCHAR    ? "TermChar, ": "",
	       uscpi->usbtmc_int_cap & USBTMC_INT_CAP_LISTEN_ONLY ? "L3, " :
	       uscpi->usbtmc_int_cap & USBTMC_INT_CAP_TALK_ONLY   ? ""     : "L4, ",
	       uscpi->usbtmc_int_cap & USBTMC_INT_CAP_TALK_ONLY   ? "T5, " :
	       uscpi->usbtmc_int_cap & USBTMC_INT_CAP_LISTEN_ONLY ? ""     : "T6, ",
	       uscpi->usb488_dev_cap & USB488_DEV_CAP_SR1         ? "SR1"  : "SR0",
	       uscpi->usb488_dev_cap & USB488_DEV_CAP_RL1         ? "RL1"  : "RL0",
	       uscpi->usb488_dev_cap & USB488_DEV_CAP_DT1         ? "DT1"  : "DT0");

	scpi_usbtmc_remote(uscpi);

	return SR_OK;
}

static int scpi_usbtmc_libusb_connection_id(struct sr_scpi_dev_inst *scpi,
		char **connection_id)
{
	struct scpi_usbtmc_libusb *uscpi = scpi->priv;
	struct sr_usb_dev_inst *usb = uscpi->usb;

	*connection_id = g_strdup_printf("%s/%" PRIu8 ".%" PRIu8 "",
		scpi->prefix, usb->bus, usb->address);

	return SR_OK;
}

static int scpi_usbtmc_libusb_source_add(struct sr_session *session,
		void *priv, int events, int timeout, sr_receive_data_callback cb,
		void *cb_data)
{
	struct scpi_usbtmc_libusb *uscpi = priv;
	(void)events;
	return usb_source_add(session, uscpi->ctx, timeout, cb, cb_data);
}

static int scpi_usbtmc_libusb_source_remove(struct sr_session *session,
		void *priv)
{
	struct scpi_usbtmc_libusb *uscpi = priv;
	return usb_source_remove(session, uscpi->ctx);
}

static void usbtmc_bulk_out_header_write(void *header, uint8_t MsgID,
                                         uint8_t bTag,
                                         uint32_t TransferSize,
                                         uint8_t bmTransferAttributes,
                                         char TermChar)
{
	  W8(header +  0, MsgID);
	  W8(header +  1, bTag);
	  W8(header +  2, ~bTag);
	  W8(header +  3, 0);
	WL32(header +  4, TransferSize);
	  W8(header +  8, bmTransferAttributes);
	  W8(header +  9, TermChar);
	WL16(header + 10, 0);
}

static int usbtmc_bulk_in_header_read(void *header, uint8_t MsgID,
                                      unsigned char bTag,
                                      int32_t *TransferSize,
                                      uint8_t *bmTransferAttributes)
{
	if (R8(header + 0) != MsgID ||
	    R8(header + 1) != bTag  ||
	    R8(header + 2) != (unsigned char)~bTag)
		return SR_ERR;
	if (TransferSize)
		*TransferSize = RL32(header + 4);
	if (bmTransferAttributes)
		*bmTransferAttributes = R8(header + 8);

	return SR_OK;
}

static int scpi_usbtmc_bulkout(struct scpi_usbtmc_libusb *uscpi,
                               uint8_t msg_id, const void *data, int32_t size,
                               uint8_t transfer_attributes)
{
	struct sr_usb_dev_inst *usb = uscpi->usb;
	int padded_size, ret, transferred;

	if (data && (size + USBTMC_BULK_HEADER_SIZE + 3) > (int)sizeof(uscpi->buffer)) {
		sr_err("USBTMC bulk out transfer is too big.");
		return SR_ERR;
	}

	uscpi->bTag++;
	uscpi->bTag += !uscpi->bTag; /* bTag == 0 is invalid so avoid it. */

	usbtmc_bulk_out_header_write(uscpi->buffer, msg_id, uscpi->bTag,
	                             size, transfer_attributes, 0);
	if (data)
		memcpy(uscpi->buffer + USBTMC_BULK_HEADER_SIZE, data, size);
	else
		size = 0;
	size += USBTMC_BULK_HEADER_SIZE;
	padded_size = (size + 3) & ~0x3;
	memset(uscpi->buffer + size, 0, padded_size - size);

	ret = libusb_bulk_transfer(usb->devhdl, uscpi->bulk_out_ep,
	                           uscpi->buffer, padded_size, &transferred,
	                           TRANSFER_TIMEOUT);
	if (ret < 0) {
		sr_err("USBTMC bulk out transfer error: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}

	if (transferred < padded_size) {
		sr_dbg("USBTMC bulk out partial transfer (%d/%d bytes).",
		       transferred, padded_size);
		return SR_ERR;
	}

	return transferred - USBTMC_BULK_HEADER_SIZE;
}

static int scpi_usbtmc_bulkin_start(struct scpi_usbtmc_libusb *uscpi,
                                    uint8_t msg_id, void *data, int32_t size,
                                    uint8_t *transfer_attributes)
{
	struct sr_usb_dev_inst *usb = uscpi->usb;
	int ret, transferred, message_size, tries;

	for (tries = 0; ; tries++) {
		ret = libusb_bulk_transfer(usb->devhdl, uscpi->bulk_in_ep, data,
					   size, &transferred,
					   TRANSFER_TIMEOUT);
		if (ret < 0) {
			sr_err("USBTMC bulk in transfer error: %s.",
			       libusb_error_name(ret));
			return SR_ERR;
		}

		if (transferred == 0 && tries < 1) {
			/*
			 * The DEV_DEP_MSG_IN message is empty, and the TMC
			 * spec says it should at least contain a header.
			 * The Rigol DS1054Z does this before many messages at
			 * high speed, and it follows up with a valid message.
			 * Give the device one more chance to send a header.
			 */
			sr_dbg("USBTMC bulk in start was empty; retrying.");
			continue;
		}

		if (transferred < USBTMC_BULK_HEADER_SIZE) {
			sr_err("USBTMC bulk in returned too little data: %d/%d bytes\n", transferred, size);
			return SR_ERR;
		}

		break;
	}

	if (usbtmc_bulk_in_header_read(data, msg_id, uscpi->bTag, &message_size,
	                               transfer_attributes) != SR_OK) {
		sr_err("USBTMC invalid bulk in header.");
		return SR_ERR;
	}

	message_size += USBTMC_BULK_HEADER_SIZE;
	uscpi->response_length = MIN(transferred, message_size);
	uscpi->response_bytes_read = USBTMC_BULK_HEADER_SIZE;
	uscpi->remaining_length = message_size - uscpi->response_length;

	return transferred - USBTMC_BULK_HEADER_SIZE;
}

static int scpi_usbtmc_bulkin_continue(struct scpi_usbtmc_libusb *uscpi,
                                       void *data, int size)
{
	struct sr_usb_dev_inst *usb = uscpi->usb;
	int ret, transferred;

	ret = libusb_bulk_transfer(usb->devhdl, uscpi->bulk_in_ep, data, size,
	                           &transferred, TRANSFER_TIMEOUT);
	if (ret < 0) {
		sr_err("USBTMC bulk in transfer error: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}

	uscpi->response_length = MIN(transferred, uscpi->remaining_length);
	uscpi->response_bytes_read = 0;
	uscpi->remaining_length -= uscpi->response_length;

	return transferred;
}

static int scpi_usbtmc_libusb_send(void *priv, const char *command)
{
	struct scpi_usbtmc_libusb *uscpi = priv;

	if (scpi_usbtmc_bulkout(uscpi, DEV_DEP_MSG_OUT,
	                        command, strlen(command), EOM) <= 0)
		return SR_ERR;

	sr_spew("Successfully sent SCPI command: '%s'.", command);

	return SR_OK;
}

/*
 * Stream reading, for devices whose every bulk IN transfer ends after one
 * short packet (see scpi_usbtmc_libusb_open()). Reading one transfer at a
 * time leaves a gap after every packet in which the device is not polled:
 * the Rigol DS1000Z then reaches about 0.5 MB/s at high speed. Here up to
 * STREAM_TRANSFERS transfers of one packet each are kept in flight while a
 * message is read, and the received bytes are consumed in order from a
 * FIFO. Only as many transfers are queued as the rest of the message can
 * fill, so none is left over to cancel: cancelling is slow, and while the
 * device sends it can lose a packet.
 */
static void LIBUSB_CALL scpi_usbtmc_stream_cb(struct libusb_transfer *transfer)
{
	*(int *)transfer->user_data = 1;
}

static void scpi_usbtmc_stream_cancel(struct scpi_usbtmc_libusb *uscpi)
{
	int i, slot;

	for (i = 0; i < uscpi->xfer_count; i++) {
		slot = (uscpi->xfer_head + i) % STREAM_TRANSFERS;
		if (!uscpi->xfer_done[slot])
			libusb_cancel_transfer(uscpi->xfer[slot]);
	}
	for (i = 0; i < uscpi->xfer_count; i++) {
		slot = (uscpi->xfer_head + i) % STREAM_TRANSFERS;
		while (!uscpi->xfer_done[slot])
			libusb_handle_events_completed(uscpi->ctx->libusb_ctx,
				&uscpi->xfer_done[slot]);
	}
	uscpi->xfer_head = uscpi->xfer_count = 0;
	uscpi->fifo_start = uscpi->fifo_end = 0;
}

static void scpi_usbtmc_stream_free(struct scpi_usbtmc_libusb *uscpi)
{
	int i;

	scpi_usbtmc_stream_cancel(uscpi);
	for (i = 0; i < STREAM_TRANSFERS; i++) {
		if (!uscpi->xfer[i])
			continue;
		g_free(uscpi->xfer[i]->buffer);
		libusb_free_transfer(uscpi->xfer[i]);
		uscpi->xfer[i] = NULL;
	}
	g_free(uscpi->fifo);
	uscpi->fifo = NULL;
}

/* Keep STREAM_TRANSFERS transfers in flight. */
static int scpi_usbtmc_stream_submit(struct scpi_usbtmc_libusb *uscpi)
{
	struct libusb_transfer *transfer;
	int slot, ret;

	size_t expect;

	/* Before the header is in, one packet. */
	expect = uscpi->msg_total ? uscpi->msg_total : 1;
	while (uscpi->xfer_count < STREAM_TRANSFERS &&
			uscpi->msg_received + uscpi->xfer_count *
			(size_t)uscpi->bulk_in_max_packet < expect) {
		slot = (uscpi->xfer_head + uscpi->xfer_count) % STREAM_TRANSFERS;
		if (!uscpi->xfer[slot]) {
			if (!(uscpi->xfer[slot] = libusb_alloc_transfer(0)))
				return SR_ERR_MALLOC;
			uscpi->xfer[slot]->buffer = g_malloc(uscpi->bulk_in_max_packet);
		}
		transfer = uscpi->xfer[slot];
		uscpi->xfer_done[slot] = 0;
		libusb_fill_bulk_transfer(transfer, uscpi->usb->devhdl,
			uscpi->bulk_in_ep, transfer->buffer, uscpi->bulk_in_max_packet,
			scpi_usbtmc_stream_cb, &uscpi->xfer_done[slot],
			TRANSFER_TIMEOUT);
		if ((ret = libusb_submit_transfer(transfer)) < 0) {
			sr_err("USBTMC bulk in submit error: %s.",
			       libusb_error_name(ret));
			uscpi->xfer_done[slot] = 1;
			return SR_ERR;
		}
		uscpi->xfer_count++;
	}

	return SR_OK;
}

/* Wait until the FIFO holds at least need bytes. */
static int scpi_usbtmc_stream_fill(struct scpi_usbtmc_libusb *uscpi,
	size_t need)
{
	struct libusb_transfer *transfer;
	size_t len;
	int slot;

	if (!uscpi->fifo)
		uscpi->fifo = g_malloc(MAX_TRANSFER_LENGTH +
			STREAM_TRANSFERS * uscpi->bulk_in_max_packet);

	while (uscpi->fifo_end - uscpi->fifo_start < need) {
		if (scpi_usbtmc_stream_submit(uscpi) != SR_OK)
			return SR_ERR;
		if (!uscpi->xfer_count) {
			sr_err("USBTMC message ended early.");
			return SR_ERR;
		}
		slot = uscpi->xfer_head;
		while (!uscpi->xfer_done[slot])
			libusb_handle_events_completed(uscpi->ctx->libusb_ctx,
				&uscpi->xfer_done[slot]);
		transfer = uscpi->xfer[slot];
		uscpi->xfer_head = (slot + 1) % STREAM_TRANSFERS;
		uscpi->xfer_count--;
		if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
			sr_err("USBTMC bulk in transfer error: %s.",
			       libusb_error_name(transfer->status ==
					LIBUSB_TRANSFER_TIMED_OUT ?
					LIBUSB_ERROR_TIMEOUT : LIBUSB_ERROR_IO));
			return SR_ERR;
		}
		len = transfer->actual_length;
		if (uscpi->fifo_start) {
			memmove(uscpi->fifo, uscpi->fifo + uscpi->fifo_start,
				uscpi->fifo_end - uscpi->fifo_start);
			uscpi->fifo_end -= uscpi->fifo_start;
			uscpi->fifo_start = 0;
		}
		memcpy(uscpi->fifo + uscpi->fifo_end, transfer->buffer, len);
		uscpi->fifo_end += len;
		uscpi->msg_received += len;
	}

	return SR_OK;
}

/*
 * Stop the transfers in flight between two reads, keeping what they
 * received (in order) for the next read. Transfers in flight between reads
 * stalled the session. Cancelling a transfer while the device sends can
 * lose a packet, so first receive the rest of the current message (at most
 * one message of max_in_message bytes): then the device sends nothing
 * until the next request, and the transfers still in flight are idle.
 */
static void scpi_usbtmc_stream_pause(struct scpi_usbtmc_libusb *uscpi)
{
	struct libusb_transfer *transfer;
	int i, slot;
	size_t len;

	if (uscpi->remaining_length > 0 && scpi_usbtmc_stream_fill(uscpi,
			uscpi->fifo_end - uscpi->fifo_start +
			(uscpi->msg_total - uscpi->msg_received)) != SR_OK) {
		scpi_usbtmc_stream_cancel(uscpi);
		return;
	}
	for (i = 0; i < uscpi->xfer_count; i++) {
		slot = (uscpi->xfer_head + i) % STREAM_TRANSFERS;
		if (!uscpi->xfer_done[slot])
			libusb_cancel_transfer(uscpi->xfer[slot]);
	}
	if (uscpi->fifo_start) {
		memmove(uscpi->fifo, uscpi->fifo + uscpi->fifo_start,
			uscpi->fifo_end - uscpi->fifo_start);
		uscpi->fifo_end -= uscpi->fifo_start;
		uscpi->fifo_start = 0;
	}
	for (i = 0; i < uscpi->xfer_count; i++) {
		slot = (uscpi->xfer_head + i) % STREAM_TRANSFERS;
		while (!uscpi->xfer_done[slot])
			libusb_handle_events_completed(uscpi->ctx->libusb_ctx,
				&uscpi->xfer_done[slot]);
		transfer = uscpi->xfer[slot];
		len = transfer->actual_length;
		if (transfer->status == LIBUSB_TRANSFER_COMPLETED ||
				transfer->status == LIBUSB_TRANSFER_CANCELLED) {
			memcpy(uscpi->fifo + uscpi->fifo_end, transfer->buffer, len);
			uscpi->fifo_end += len;
		}
	}
	uscpi->xfer_head = uscpi->xfer_count = 0;
}

/* Request the next DEV_DEP_MSG_IN message and read its header. */
static int scpi_usbtmc_stream_begin_message(struct scpi_usbtmc_libusb *uscpi)
{
	int32_t message_size;

	uscpi->fifo_start = uscpi->fifo_end = 0;
	uscpi->msg_total = uscpi->msg_received = 0;
	if (scpi_usbtmc_bulkout(uscpi, REQUEST_DEV_DEP_MSG_IN,
	    NULL, uscpi->max_in_message, 0) < 0)
		return SR_ERR;
	/* An empty transfer may come first, it is skipped. */
	if (scpi_usbtmc_stream_fill(uscpi, USBTMC_BULK_HEADER_SIZE) != SR_OK)
		return SR_ERR;
	if (usbtmc_bulk_in_header_read(uscpi->fifo + uscpi->fifo_start,
			DEV_DEP_MSG_IN, uscpi->bTag, &message_size,
			&uscpi->bulkin_attributes) != SR_OK) {
		sr_err("USBTMC invalid bulk in header.");
		return SR_ERR;
	}
	uscpi->fifo_start += USBTMC_BULK_HEADER_SIZE;
	uscpi->msg_total = USBTMC_BULK_HEADER_SIZE + message_size;
	uscpi->remaining_length = message_size;
	uscpi->response_length = uscpi->response_bytes_read = 0;

	return SR_OK;
}

static int scpi_usbtmc_stream_read_begin(struct scpi_usbtmc_libusb *uscpi)
{
	scpi_usbtmc_stream_cancel(uscpi);
	if (scpi_usbtmc_stream_begin_message(uscpi) != SR_OK) {
		scpi_usbtmc_stream_cancel(uscpi);
		return SR_ERR;
	}

	return SR_OK;
}

static int scpi_usbtmc_stream_read_chunk(struct scpi_usbtmc_libusb *uscpi,
	char *buf, int maxlen)
{
	int len;

	if (uscpi->remaining_length <= 0) {
		if (uscpi->bulkin_attributes & EOM)
			return SR_ERR;
		if (scpi_usbtmc_stream_begin_message(uscpi) != SR_OK)
			goto fail;
	}

	/* The FIFO holds a few packets more than MAX_TRANSFER_LENGTH. */
	len = MIN(MIN(maxlen, uscpi->remaining_length), MAX_TRANSFER_LENGTH);
	if (scpi_usbtmc_stream_fill(uscpi, len) != SR_OK)
		goto fail;
	memcpy(buf, uscpi->fifo + uscpi->fifo_start, len);
	uscpi->fifo_start += len;
	uscpi->remaining_length -= len;

	if (uscpi->remaining_length == 0) {
		/* Whatever else came with the message is alignment. */
		uscpi->fifo_start = uscpi->fifo_end = 0;
		if (uscpi->xfer_count)
			scpi_usbtmc_stream_cancel(uscpi);
	}

	return len;

fail:
	scpi_usbtmc_stream_cancel(uscpi);
	return SR_ERR;
}

static int scpi_usbtmc_libusb_read_begin(void *priv)
{
	struct scpi_usbtmc_libusb *uscpi = priv;

	if (uscpi->stream)
		return scpi_usbtmc_stream_read_begin(uscpi);

	uscpi->remaining_length = 0;

	if (scpi_usbtmc_bulkout(uscpi, REQUEST_DEV_DEP_MSG_IN,
	    NULL, uscpi->max_in_message, 0) < 0)
		return SR_ERR;
	if (scpi_usbtmc_bulkin_start(uscpi, DEV_DEP_MSG_IN,
	                             uscpi->buffer, sizeof(uscpi->buffer),
	                             &uscpi->bulkin_attributes) < 0)
		return SR_ERR;

	return SR_OK;
}

static int scpi_usbtmc_libusb_read_complete(void *priv);

/* Read what the current transfer holds, fetching the next one if needed. */
static int scpi_usbtmc_libusb_read_chunk(struct scpi_usbtmc_libusb *uscpi,
	char *buf, int maxlen)
{
	int read_length;

	if (uscpi->stream)
		return scpi_usbtmc_stream_read_chunk(uscpi, buf, maxlen);

	if (uscpi->response_bytes_read >= uscpi->response_length) {
		if (uscpi->remaining_length > 0) {
			if (scpi_usbtmc_bulkin_continue(uscpi, uscpi->buffer,
			                                sizeof(uscpi->buffer)) <= 0)
				return SR_ERR;
		} else {
			if (uscpi->bulkin_attributes & EOM)
				return SR_ERR;
			if (scpi_usbtmc_libusb_read_begin(uscpi) < 0)
				return SR_ERR;
		}
	}

	read_length = MIN(uscpi->response_length - uscpi->response_bytes_read, maxlen);

	memcpy(buf, uscpi->buffer + uscpi->response_bytes_read, read_length);

	uscpi->response_bytes_read += read_length;

	return read_length;
}

/*
 * Fill the buffer across transfers and DEV_DEP_MSG_IN messages until it is
 * full or the response is complete. Callers often read once per poll
 * interval, so returning after every (possibly short) message makes large
 * responses very slow.
 */
static int scpi_usbtmc_libusb_read_data(void *priv, char *buf, int maxlen)
{
	struct scpi_usbtmc_libusb *uscpi = priv;
	int total, len;

	total = 0;
	while (total < maxlen) {
		len = scpi_usbtmc_libusb_read_chunk(uscpi, buf + total,
			maxlen - total);
		if (len < 0) {
			total = total ? total : len;
			break;
		}
		total += len;
		if (len == 0 || scpi_usbtmc_libusb_read_complete(uscpi))
			break;
	}
	if (uscpi->stream && uscpi->xfer_count)
		scpi_usbtmc_stream_pause(uscpi);

	return total;
}

static int scpi_usbtmc_libusb_read_complete(void *priv)
{
	struct scpi_usbtmc_libusb *uscpi = priv;
	return uscpi->response_bytes_read >= uscpi->response_length &&
	       uscpi->remaining_length <= 0 &&
	       uscpi->bulkin_attributes & EOM;
}

static int scpi_usbtmc_libusb_close(struct sr_scpi_dev_inst *scpi)
{
	struct scpi_usbtmc_libusb *uscpi = scpi->priv;
	struct sr_usb_dev_inst *usb = uscpi->usb;
	int ret;

	if (!usb->devhdl)
		return SR_ERR;

	scpi_usbtmc_stream_free(uscpi);

	scpi_usbtmc_local(uscpi);

	if ((ret = libusb_release_interface(usb->devhdl, uscpi->interface)) < 0)
		sr_err("Failed to release interface: %s.",
		       libusb_error_name(ret));

	if (uscpi->detached_kernel_driver) {
		if ((ret = libusb_attach_kernel_driver(usb->devhdl,
						uscpi->interface)) < 0)
			sr_err("Failed to re-attach kernel driver: %s.",
			       libusb_error_name(ret));

		uscpi->detached_kernel_driver = 0;
	}
	sr_usb_close(usb);

	return SR_OK;
}

static void scpi_usbtmc_libusb_free(void *priv)
{
	struct scpi_usbtmc_libusb *uscpi = priv;
	sr_usb_dev_inst_free(uscpi->usb);
}

SR_PRIV const struct sr_scpi_dev_inst scpi_usbtmc_libusb_dev = {
	.name          = "USBTMC",
	.prefix        = "usbtmc",
	.transport     = SCPI_TRANSPORT_USBTMC,
	.priv_size     = sizeof(struct scpi_usbtmc_libusb),
	.scan          = scpi_usbtmc_libusb_scan,
	.dev_inst_new  = scpi_usbtmc_libusb_dev_inst_new,
	.open          = scpi_usbtmc_libusb_open,
	.connection_id = scpi_usbtmc_libusb_connection_id,
	.source_add    = scpi_usbtmc_libusb_source_add,
	.source_remove = scpi_usbtmc_libusb_source_remove,
	.send          = scpi_usbtmc_libusb_send,
	.read_begin    = scpi_usbtmc_libusb_read_begin,
	.read_data     = scpi_usbtmc_libusb_read_data,
	.read_complete = scpi_usbtmc_libusb_read_complete,
	.close         = scpi_usbtmc_libusb_close,
	.free          = scpi_usbtmc_libusb_free,
};
