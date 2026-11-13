/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2026 FluffyPotatoKitten <356412062+FluffyPotatoKitten@users.noreply.github.com>
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

/*
 * SCPI transport for the vendor specific USB protocol of the Rigol VS5000
 * series (USB ID 0400:03e8, interface class 255, no USBTMC).
 *
 * See https://sigrok.org/wiki/Rigol_VS5000_series. All control transfers
 * are vendor IN requests (bmRequestType 0xc0):
 *
 *   0x01  Send one command character in wValue, CR ends the command.
 *         The transfer that carries the CR completes after the command
 *         has been executed.
 *   0x00  wValue 0: length of the last answer (1 byte).
 *         wValue 1: the answer itself. The answer stays readable until
 *         the next query replaces it, so it is only read after a query.
 *   0x04  Number of queued waveform bytes (u32, little endian). wIndex
 *         holds a source mask, which the VS5042 ignores. If more than
 *         0x800 bytes are announced, exactly that amount must be read
 *         from bulk endpoint 0x81.
 *   0x05  Size of the acquisition memory (u32) for the channels in the
 *         wValue mask (bit 0 CH1, bit 1 CH2): 512K samples per channel
 *         on the VS5042. Exactly that amount is then read from endpoint
 *         0x81, without header, CH1 first. The data is not consumed.
 *         Used by the vendor software on stopped captures.
 *
 * This transport sends commands and returns answers like the other SCPI
 * transports. The query ":WAVEFORM:DATA?" is mapped to request 0x04 and
 * returns the raw bulk data: a 2048 byte header (u16 at offset 4 is the
 * number of records, u32 at offset 8 the total size), followed by the
 * records, oldest first, 2048 bytes per displayed analog channel each.
 * The transport-private query ":WAVEFORM:MEMORY? <mask>" is mapped to
 * request 0x05 and returns the acquisition memory. It is never sent to
 * the scope. Neither is ":WAVEFORM:QUEUED?", which is answered here with
 * 1 when request 0x04 reports a complete waveform record queued, else 0.
 * The scope queues no record while it waits for a trigger or is stopped.
 * Request 0x04 hands the queued data over: asked again it returns 0, and
 * the announced bytes wait on the bulk endpoint. So the size is kept for
 * the next ":WAVEFORM:DATA?", which then reads exactly that.
 */

#include "config.h"

#include <glib.h>
#include <libusb.h>
#include <stdlib.h>
#include <string.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "scpi.h"

#define LOG_PREFIX "scpi_usb_vs5000"

#define VS5000_VID		0x0400
#define VS5000_PID		0x03e8
#define VS5000_INTERFACE	0
#define VS5000_EP_IN		0x81
#define VS5000_REQ_TYPE		(LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define VS5000_REQ_ANSWER	0x00
#define VS5000_REQ_CHAR		0x01
#define VS5000_REQ_WAVE		0x04
#define VS5000_WAVE_MIN		0x800
#define VS5000_WAVE_MASK	0x03
#define VS5000_WAVE_CMD		":WAVEFORM:DATA?"
#define VS5000_REQ_MEMORY	0x05
#define VS5000_MEMORY_CMD	":WAVEFORM:MEMORY? "
#define VS5000_QUEUED_CMD	":WAVEFORM:QUEUED?"
#define CTRL_TIMEOUT_MS		2000
#define BULK_TIMEOUT_MS		2000
#define WAVE_WAIT_US		(3 * 1000 * 1000)
#define BULK_CHUNK		4096

enum vs5000_mode {
	MODE_NONE,
	MODE_ANSWER,
	MODE_WAVE,
	MODE_MEMORY,
};

struct scpi_usb_vs5000 {
	struct sr_context *ctx;
	struct sr_usb_dev_inst *usb;
	enum vs5000_mode mode;
	/*
	 * MODE_WAVE or MODE_MEMORY while a data reply has bytes left. It is
	 * kept apart from mode: a query may come in between two reads of a
	 * reply (another thread changing a setting), the reply continues
	 * after its answer has been read.
	 */
	enum vs5000_mode data_mode;
	gboolean answer_pending;
	gboolean answer_local;
	uint8_t answer[256];
	size_t answer_len;
	size_t answer_pos;
	uint32_t wave_total;
	/* Announced by request 0x04 for :WAVEFORM:QUEUED?, not read yet. */
	uint32_t wave_announced;
	uint64_t wave_remaining;
	uint8_t chunk[BULK_CHUNK];
	size_t chunk_len;
	size_t chunk_pos;
};

static GSList *scpi_usb_vs5000_scan(struct drv_context *drvc)
{
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	GSList *resources = NULL;
	int ret, i;

	ret = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (ret < 0) {
		sr_err("Failed to get device list: %s.", libusb_error_name(ret));
		return NULL;
	}
	for (i = 0; devlist[i]; i++) {
		libusb_get_device_descriptor(devlist[i], &des);
		if (des.idVendor != VS5000_VID || des.idProduct != VS5000_PID)
			continue;
		sr_dbg("Found VS5000 at bus.address %d.%d.",
			libusb_get_bus_number(devlist[i]),
			libusb_get_device_address(devlist[i]));
		resources = g_slist_append(resources,
			g_strdup_printf("usb-vs5000/%d.%d",
				libusb_get_bus_number(devlist[i]),
				libusb_get_device_address(devlist[i])));
	}
	libusb_free_device_list(devlist, 1);

	return resources;
}

static int scpi_usb_vs5000_dev_inst_new(void *priv, struct drv_context *drvc,
		const char *resource, char **params, const char *serialcomm)
{
	struct scpi_usb_vs5000 *uv = priv;
	GSList *devices;

	(void)resource;
	(void)serialcomm;

	uv->ctx = drvc->sr_ctx;
	devices = sr_usb_find(uv->ctx->libusb_ctx,
		(params && params[1]) ? params[1] : "0400.03e8");
	if (g_slist_length(devices) != 1) {
		sr_err("Failed to find exactly one VS5000 USB device.");
		g_slist_free_full(devices, (GDestroyNotify)sr_usb_dev_inst_free);
		return SR_ERR;
	}
	uv->usb = devices->data;
	g_slist_free(devices);

	return SR_OK;
}

static int vs5000_ctrl(struct scpi_usb_vs5000 *uv, uint8_t req,
		uint16_t value, uint16_t index, uint8_t *data, uint16_t len)
{
	int ret;

	ret = libusb_control_transfer(uv->usb->devhdl, VS5000_REQ_TYPE, req,
		value, index, data, len, CTRL_TIMEOUT_MS);
	if (ret < 0) {
		sr_err("Control request 0x%02x failed: %s.", req,
			libusb_error_name(ret));
		return SR_ERR;
	}

	return ret;
}

/* Check that a waveform reply starts with a valid header. */
static gboolean vs5000_header_ok(const uint8_t *buf, size_t len, uint32_t total)
{
	uint32_t num_records, size;

	if (len < 12) {
		sr_dbg("Waveform reply starts with a short packet (%zu bytes).", len);
		return FALSE;
	}
	num_records = read_u16le(buf + 4);
	size = read_u32le(buf + 8);
	if (size != total || !num_records ||
			(size - VS5000_WAVE_MIN) % num_records ||
			((size - VS5000_WAVE_MIN) / num_records) % 2048) {
		sr_dbg("Bulk data out of sync (header %u records, %u bytes; "
			"announced %u).", num_records, size, total);
		return FALSE;
	}

	return TRUE;
}

/* Poll request 0x04 until more than the header is queued. */
static int vs5000_wave_poll(struct scpi_usb_vs5000 *uv, gint64 wait_us,
		uint32_t *total)
{
	uint8_t buf[4];
	gint64 deadline;

	deadline = g_get_monotonic_time() + wait_us;
	do {
		if (vs5000_ctrl(uv, VS5000_REQ_WAVE, 0, VS5000_WAVE_MASK,
				buf, sizeof(buf)) != sizeof(buf))
			return SR_ERR;
		*total = read_u32le(buf);
		if (*total > VS5000_WAVE_MIN)
			return SR_OK;
		g_usleep(5000);
	} while (g_get_monotonic_time() < deadline);

	return SR_ERR_TIMEOUT;
}

/*
 * Request queued waveform data and read all of it, checking that the
 * header arrives where expected. This discards old records and tells
 * whether the bulk pipe is in sync. Never read bulk data that was not
 * announced by request 0x04: the device then starts streaming and stops
 * answering later requests.
 */
static int vs5000_sync(struct scpi_usb_vs5000 *uv)
{
	uint32_t total, got;
	int ret, len, empty;

	ret = vs5000_wave_poll(uv, 1000 * 1000, &total);
	if (ret == SR_ERR_TIMEOUT) {
		/* Stopped scope, nothing to check. */
		sr_dbg("No waveform data queued, cannot check sync.");
		return SR_OK;
	}
	if (ret != SR_OK)
		return ret;

	for (got = 0, empty = 0; got < total; got += len) {
		ret = libusb_bulk_transfer(uv->usb->devhdl, VS5000_EP_IN,
			uv->chunk, MIN(sizeof(uv->chunk), total - got),
			&len, BULK_TIMEOUT_MS);
		if (ret != 0) {
			sr_dbg("Sync read failed after %u of %u bytes: %s.",
				got, total, libusb_error_name(ret));
			return SR_ERR;
		}
		if (len == 0) {
			/* The first packet may be empty. */
			if (++empty > 10)
				return SR_ERR;
			continue;
		}
		if (got == 0 && !vs5000_header_ok(uv->chunk, len, total))
			return SR_ERR;
	}
	sr_dbg("Discarded %u bytes of old waveform data.", total);

	return SR_OK;
}

/*
 * If a session stopped in the middle of a USB packet, stale packets stay
 * in the device's FIFO and shift all further data. SET_CONFIGURATION does
 * not remove them, only a bus reset does. It takes about a second and
 * leaves the scope's settings alone.
 */
static int vs5000_recover(struct scpi_usb_vs5000 *uv)
{
	int ret;

	sr_info("Bulk data out of sync, resetting the USB device.");
	ret = libusb_reset_device(uv->usb->devhdl);
	if (ret < 0) {
		sr_err("USB reset failed: %s.", libusb_error_name(ret));
		return SR_ERR;
	}
	g_usleep(200 * 1000);
	if (vs5000_sync(uv) != SR_OK) {
		sr_err("Bulk data still out of sync after USB reset.");
		return SR_ERR;
	}

	return SR_OK;
}

static int scpi_usb_vs5000_open(struct sr_scpi_dev_inst *scpi)
{
	struct scpi_usb_vs5000 *uv = scpi->priv;
	struct sr_usb_dev_inst *usb = uv->usb;
	int ret;

	if (usb->devhdl)
		return SR_OK;

	if (sr_usb_open(uv->ctx->libusb_ctx, usb) != SR_OK)
		return SR_ERR;

	/*
	 * SET_CONFIGURATION resets the bulk engine of the device, so that
	 * a waveform transfer which an earlier session did not finish is
	 * sent again from the start.
	 */
	if ((ret = libusb_set_configuration(usb->devhdl, 1)) < 0)
		sr_dbg("Failed to set configuration: %s.", libusb_error_name(ret));

	if ((ret = libusb_claim_interface(usb->devhdl, VS5000_INTERFACE)) < 0) {
		sr_err("Failed to claim interface: %s.", libusb_error_name(ret));
		sr_usb_close(usb);
		return SR_ERR;
	}

	if (vs5000_sync(uv) != SR_OK && vs5000_recover(uv) != SR_OK) {
		libusb_release_interface(usb->devhdl, VS5000_INTERFACE);
		sr_usb_close(usb);
		return SR_ERR;
	}

	uv->mode = uv->data_mode = MODE_NONE;
	uv->answer_pending = FALSE;
	uv->answer_local = FALSE;
	uv->wave_announced = 0;

	return SR_OK;
}

static int scpi_usb_vs5000_connection_id(struct sr_scpi_dev_inst *scpi,
		char **connection_id)
{
	struct scpi_usb_vs5000 *uv = scpi->priv;

	*connection_id = g_strdup_printf("%s/%" PRIu8 ".%" PRIu8,
		scpi->prefix, uv->usb->bus, uv->usb->address);

	return SR_OK;
}

static int scpi_usb_vs5000_source_add(struct sr_session *session, void *priv,
		int events, int timeout, sr_receive_data_callback cb, void *cb_data)
{
	struct scpi_usb_vs5000 *uv = priv;

	(void)events;

	return usb_source_add(session, uv->ctx, timeout, cb, cb_data);
}

static int scpi_usb_vs5000_source_remove(struct sr_session *session, void *priv)
{
	struct scpi_usb_vs5000 *uv = priv;

	return usb_source_remove(session, uv->ctx);
}

/*
 * Read and drop what is left of an unfinished data reply, so that the
 * next reply starts in sync. This happens when an acquisition stops in
 * the middle of a record.
 */
static int vs5000_discard(struct scpi_usb_vs5000 *uv)
{
	uint64_t left;
	int ret, len, empty;

	uv->chunk_len = uv->chunk_pos = 0;
	left = uv->wave_remaining;
	uv->wave_remaining = 0;
	uv->data_mode = MODE_NONE;
	if (!left)
		return SR_OK;

	sr_dbg("Discarding %" PRIu64 " bytes of an unfinished reply.", left);
	for (empty = 0; left; left -= len) {
		ret = libusb_bulk_transfer(uv->usb->devhdl, VS5000_EP_IN,
			uv->chunk, MIN(sizeof(uv->chunk), left),
			&len, BULK_TIMEOUT_MS);
		if (ret != 0 || (len == 0 && ++empty > 10)) {
			sr_dbg("Discard failed with %" PRIu64 " bytes left.", left);
			return vs5000_recover(uv);
		}
	}

	return SR_OK;
}

/* Ask for queued waveform data, wait until at least one record is there. */
static int vs5000_wave_request(struct scpi_usb_vs5000 *uv)
{
	uint32_t total;
	int ret;

	if (uv->wave_announced) {
		total = uv->wave_announced;
		uv->wave_announced = 0;
		ret = SR_OK;
	} else
		ret = vs5000_wave_poll(uv, WAVE_WAIT_US, &total);
	if (ret != SR_OK) {
		if (ret == SR_ERR_TIMEOUT)
			sr_err("Timeout waiting for waveform data.");
		return ret;
	}
	sr_spew("%u waveform bytes queued.", total);
	uv->wave_total = total;
	uv->wave_remaining = total;
	uv->chunk_len = uv->chunk_pos = 0;
	uv->data_mode = MODE_WAVE;

	return SR_OK;
}

/* Ask for the acquisition memory of the channels in mask. */
static int vs5000_memory_request(struct scpi_usb_vs5000 *uv, const char *arg)
{
	uint8_t buf[4];
	uint32_t total;
	char *end;
	unsigned long mask;

	mask = strtoul(arg, &end, 10);
	if (end == arg || *end || !mask || mask > 0xff) {
		sr_err("Invalid memory channel mask '%s'.", arg);
		return SR_ERR_ARG;
	}
	if (vs5000_ctrl(uv, VS5000_REQ_MEMORY, mask, 0, buf, sizeof(buf))
			!= sizeof(buf))
		return SR_ERR;
	total = read_u32le(buf);
	sr_dbg("%u bytes of acquisition memory for mask 0x%02lx.", total, mask);
	if (!total) {
		sr_err("No acquisition memory available.");
		return SR_ERR;
	}
	uv->wave_total = total;
	uv->wave_remaining = total;
	uv->chunk_len = uv->chunk_pos = 0;
	uv->data_mode = MODE_MEMORY;

	return SR_OK;
}

static int scpi_usb_vs5000_send(void *priv, const char *command)
{
	struct scpi_usb_vs5000 *uv = priv;
	uint8_t status;
	size_t len, i;
	uint32_t total;
	char *arg;
	int ret;

	len = strlen(command);
	while (len && (command[len - 1] == '\n' || command[len - 1] == '\r'))
		len--;

	if (len == strlen(VS5000_WAVE_CMD) &&
			!g_ascii_strncasecmp(command, VS5000_WAVE_CMD, len)) {
		if (vs5000_discard(uv) != SR_OK)
			return SR_ERR;
		uv->mode = MODE_WAVE;
		return vs5000_wave_request(uv);
	}
	if (len > strlen(VS5000_MEMORY_CMD) && !g_ascii_strncasecmp(command,
			VS5000_MEMORY_CMD, strlen(VS5000_MEMORY_CMD))) {
		if (vs5000_discard(uv) != SR_OK)
			return SR_ERR;
		arg = g_strndup(command + strlen(VS5000_MEMORY_CMD),
			len - strlen(VS5000_MEMORY_CMD));
		ret = vs5000_memory_request(uv, arg);
		g_free(arg);
		uv->mode = ret == SR_OK ? MODE_MEMORY : MODE_NONE;
		return ret;
	}

	if (len == strlen(VS5000_QUEUED_CMD) &&
			!g_ascii_strncasecmp(command, VS5000_QUEUED_CMD, len)) {
		if (vs5000_discard(uv) != SR_OK)
			return SR_ERR;
		if (!uv->wave_announced) {
			ret = vs5000_wave_poll(uv, 0, &total);
			if (ret == SR_ERR)
				return SR_ERR;
			if (ret == SR_OK)
				uv->wave_announced = total;
		}
		uv->answer[0] = uv->wave_announced ? '1' : '0';
		uv->answer_len = 1;
		uv->answer_pos = 0;
		uv->answer_local = TRUE;
		uv->answer_pending = FALSE;
		uv->mode = MODE_ANSWER;
		return SR_OK;
	}

	for (i = 0; i <= len; i++) {
		if (vs5000_ctrl(uv, VS5000_REQ_CHAR,
				i < len ? (uint8_t)command[i] : '\r', 0,
				&status, 1) < 0)
			return SR_ERR;
	}
	sr_spew("Successfully sent SCPI command: '%.*s'.", (int)len, command);

	uv->answer_pending = len && command[len - 1] == '?';
	uv->answer_local = FALSE;
	uv->mode = uv->answer_pending ? MODE_ANSWER : MODE_NONE;

	return SR_OK;
}

static int scpi_usb_vs5000_read_begin(void *priv)
{
	struct scpi_usb_vs5000 *uv = priv;
	uint8_t n;

	if (uv->mode != MODE_ANSWER || uv->answer_local)
		return SR_OK;

	if (!uv->answer_pending && uv->data_mode != MODE_NONE) {
		/* A query came in between, continue the data reply. */
		uv->mode = uv->data_mode;
		return SR_OK;
	}

	uv->answer_len = uv->answer_pos = 0;
	if (!uv->answer_pending) {
		sr_err("No query pending, no answer to read.");
		return SR_ERR;
	}
	uv->answer_pending = FALSE;

	if (vs5000_ctrl(uv, VS5000_REQ_ANSWER, 0, 0, &n, 1) != 1)
		return SR_ERR;
	if (n && vs5000_ctrl(uv, VS5000_REQ_ANSWER, 1, 0, uv->answer, n) != n)
		return SR_ERR;
	uv->answer_len = n;

	return SR_OK;
}

/*
 * Get the next bulk chunk of the current waveform reply.
 * Returns 1 when data is available, 0 to try again later, or an error.
 */
static int vs5000_fill(struct scpi_usb_vs5000 *uv)
{
	int ret, transferred;

	/* Never ask for more than announced. */
	ret = libusb_bulk_transfer(uv->usb->devhdl, VS5000_EP_IN, uv->chunk,
		MIN(sizeof(uv->chunk), uv->wave_remaining),
		&transferred, BULK_TIMEOUT_MS);
	if (ret != 0) {
		sr_err("Bulk read failed: %s.", libusb_error_name(ret));
		return SR_ERR;
	}
	if (transferred == 0)
		/* The first packet may be empty. */
		return 0;
	if (uv->data_mode == MODE_WAVE && uv->wave_remaining == uv->wave_total &&
			!vs5000_header_ok(uv->chunk, transferred, uv->wave_total)) {
		/* Shifted data. Recover and ask again. */
		if (vs5000_recover(uv) != SR_OK ||
				vs5000_wave_request(uv) != SR_OK)
			return SR_ERR;
		return 0;
	}
	uv->chunk_len = transferred;
	uv->chunk_pos = 0;
	uv->wave_remaining -= MIN((uint64_t)transferred, uv->wave_remaining);

	return 1;
}

static int scpi_usb_vs5000_read_data(void *priv, char *buf, int maxlen)
{
	struct scpi_usb_vs5000 *uv = priv;
	size_t len, copied;
	int ret;

	if (maxlen <= 0)
		return 0;

	if (uv->mode == MODE_ANSWER && uv->answer_pos < uv->answer_len) {
		len = MIN((size_t)maxlen, uv->answer_len - uv->answer_pos);
		memcpy(buf, uv->answer + uv->answer_pos, len);
		uv->answer_pos += len;
		return len;
	}

	/* The rest of a data reply, also after a query in between. */
	if (uv->data_mode == MODE_NONE)
		return 0;
	uv->mode = uv->data_mode;
	for (copied = 0; copied < (size_t)maxlen; copied += len) {
		if (uv->chunk_pos == uv->chunk_len) {
			if (!uv->wave_remaining)
				break;
			ret = vs5000_fill(uv);
			if (ret < 0)
				return copied ? (int)copied : ret;
			if (ret == 0)
				break;
		}
		len = MIN((size_t)maxlen - copied,
			uv->chunk_len - uv->chunk_pos);
		memcpy(buf + copied, uv->chunk + uv->chunk_pos, len);
		uv->chunk_pos += len;
	}
	if (!uv->wave_remaining && uv->chunk_pos == uv->chunk_len)
		uv->data_mode = MODE_NONE;

	return copied;
}

static int scpi_usb_vs5000_read_complete(void *priv)
{
	struct scpi_usb_vs5000 *uv = priv;

	switch (uv->mode) {
	case MODE_ANSWER:
		return uv->answer_pos >= uv->answer_len;
	case MODE_WAVE:
	case MODE_MEMORY:
		return uv->data_mode == MODE_NONE;
	default:
		return TRUE;
	}
}

static int scpi_usb_vs5000_close(struct sr_scpi_dev_inst *scpi)
{
	struct scpi_usb_vs5000 *uv = scpi->priv;
	int ret;

	if (!uv->usb->devhdl)
		return SR_ERR;

	/* Leave the bulk pipe empty for the next session. */
	vs5000_discard(uv);

	if ((ret = libusb_release_interface(uv->usb->devhdl, VS5000_INTERFACE)) < 0)
		sr_err("Failed to release interface: %s.", libusb_error_name(ret));
	sr_usb_close(uv->usb);

	return SR_OK;
}

static void scpi_usb_vs5000_free(void *priv)
{
	struct scpi_usb_vs5000 *uv = priv;

	if (!uv->usb)
		return;
	if (uv->usb->devhdl)
		sr_usb_close(uv->usb);
	sr_usb_dev_inst_free(uv->usb);
}

SR_PRIV const struct sr_scpi_dev_inst scpi_usb_vs5000_dev = {
	.name          = "VS5000 USB",
	.prefix        = "usb-vs5000",
	.transport     = SCPI_TRANSPORT_USB_VS5000,
	.priv_size     = sizeof(struct scpi_usb_vs5000),
	.scan          = scpi_usb_vs5000_scan,
	.dev_inst_new  = scpi_usb_vs5000_dev_inst_new,
	.open          = scpi_usb_vs5000_open,
	.connection_id = scpi_usb_vs5000_connection_id,
	.source_add    = scpi_usb_vs5000_source_add,
	.source_remove = scpi_usb_vs5000_source_remove,
	.send          = scpi_usb_vs5000_send,
	.read_begin    = scpi_usb_vs5000_read_begin,
	.read_data     = scpi_usb_vs5000_read_data,
	.read_complete = scpi_usb_vs5000_read_complete,
	.close         = scpi_usb_vs5000_close,
	.free          = scpi_usb_vs5000_free,
};
