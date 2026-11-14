/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2012 Martin Ling <martin-git@earth.li>
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2013 Mathias Grimmberger <mgri@zaphod.sax.de>
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
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "scpi.h"
#include "protocol.h"

/*
 * This is a unified protocol driver for the DS1000 and DS2000 series.
 *
 * DS1000 support tested with a Rigol DS1102D.
 *
 * DS2000 support tested with a Rigol DS2072 using firmware version 01.01.00.02.
 *
 * The Rigol DS2000 series scopes try to adhere to the IEEE 488.2 (I think)
 * standard. If you want to read it - it costs real money...
 *
 * Every response from the scope has a linefeed appended because the
 * standard says so. In principle this could be ignored because sending the
 * next command clears the output queue of the scope. This driver tries to
 * avoid doing that because it may cause an error being generated inside the
 * scope and who knows what bugs the firmware has WRT this.
 *
 * Waveform data is transferred in a format called "arbitrary block program
 * data" specified in IEEE 488.2. See Agilents programming manuals for their
 * 2000/3000 series scopes for a nice description.
 *
 * Each data block from the scope has a header, e.g. "#900000001400".
 * The '#' marks the start of a block.
 * Next is one ASCII decimal digit between 1 and 9, this gives the number of
 * ASCII decimal digits following.
 * Last are the ASCII decimal digits giving the number of bytes (not
 * samples!) in the block.
 *
 * After this header as many data bytes as indicated follow.
 *
 * Each data block has a trailing linefeed too.
 */

static int parse_int(const char *str, int *ret)
{
	char *e;
	long tmp;

	errno = 0;
	tmp = strtol(str, &e, 10);
	if (e == str || *e != '\0') {
		sr_dbg("Failed to parse integer: '%s'", str);
		return SR_ERR;
	}
	if (errno) {
		sr_dbg("Failed to parse integer: '%s', numerical overflow", str);
		return SR_ERR;
	}
	if (tmp > INT_MAX || tmp < INT_MIN) {
		sr_dbg("Failed to parse integer: '%s', value to large/small", str);
		return SR_ERR;
	}

	*ret = (int)tmp;
	return SR_OK;
}

/* Set the next event to wait for in rigol_ds_receive */
static void rigol_ds_set_wait_event(struct dev_context *devc, enum wait_events event)
{
	if (event == WAIT_STOP)
		devc->wait_status = 2;
	else
		devc->wait_status = 1;
	devc->wait_event = event;
}

/*
 * Waiting for a event will return a timeout after 2 to 3 seconds in order
 * to not block the application.
 */
static int rigol_ds_event_wait(const struct sr_dev_inst *sdi, char status1, char status2)
{
	char *buf, c;
	struct dev_context *devc;
	time_t start;

	if (!(devc = sdi->priv))
		return SR_ERR;

	start = time(NULL);

	/*
	 * Trigger status may return:
	 * "TD" or "T'D" - triggered
	 * "AUTO"        - autotriggered
	 * "RUN"         - running
	 * "WAIT"        - waiting for trigger
	 * "STOP"        - stopped
	 */

	if (devc->wait_status == 1) {
		do {
			if (time(NULL) - start >= 3) {
				sr_dbg("Timeout waiting for trigger");
				return SR_ERR_TIMEOUT;
			}

			if (sr_scpi_get_string(sdi->conn, ":TRIG:STAT?", &buf) != SR_OK)
				return SR_ERR;
			c = buf[0];
			g_free(buf);
		} while (c == status1 || c == status2);

		devc->wait_status = 2;
	}
	if (devc->wait_status == 2) {
		do {
			if (time(NULL) - start >= 3) {
				sr_dbg("Timeout waiting for trigger");
				return SR_ERR_TIMEOUT;
			}

			if (sr_scpi_get_string(sdi->conn, ":TRIG:STAT?", &buf) != SR_OK)
				return SR_ERR;
			c = buf[0];
			g_free(buf);
		} while (c != status1 && c != status2);

		rigol_ds_set_wait_event(devc, WAIT_NONE);
	}

	return SR_OK;
}

/*
 * For live capture we need to wait for a new trigger event to ensure that
 * sample data is not returned twice.
 *
 * Unfortunately this will never really work because for sufficiently fast
 * timebases and trigger rates it just can't catch the status changes.
 *
 * What would be needed is a trigger event register with autoreset like the
 * Agilents have. The Rigols don't seem to have anything like this.
 *
 * The workaround is to only wait for the trigger when the timebase is slow
 * enough. Of course this means that for faster timebases sample data can be
 * returned multiple times, this effect is mitigated somewhat by sleeping
 * for about one sweep time in that case.
 */
/*
 * Number of divisions the VS5000 record spans. Up to 20 ms/div it holds
 * 100 samples per division (20.48 divisions). From 50 ms/div it scans
 * and the same 2048 samples span the 14 divisions of its screen.
 * Measured with the 1 kHz calibrator; the scan mode sample rate equals
 * :ACQ:SAMP? / 256.
 */
SR_PRIV double rigol_ds_v1_record_divs(const struct dev_context *devc)
{
	if (devc->timebase < 0.0499)
		return devc->model->series->live_samples / 100.0;
	return devc->model->series->num_horizontal_divs;
}

/*
 * VS5000 live record: 2048 samples around the trigger. Up to 20 ms/div it
 * spans 20.48 divisions (100 samples per division), in scan mode from
 * 50 ms/div the 14 divisions of the screen. Measured with the 1 kHz
 * calibrator.
 */
SR_PRIV uint64_t rigol_ds_v1_live_rate(const struct dev_context *devc)
{
	return llround(devc->model->series->live_samples /
		(devc->timebase * rigol_ds_v1_record_divs(devc)));
}

/*
 * VS5000: input voltage range of an analog channel. The ADC codes 0..255
 * cover +5 to -5.2 divisions around the screen centre (code 125), which
 * is at -offset volts. Same scaling as the sample conversion.
 */
SR_PRIV void rigol_ds_v1_input_range(const struct dev_context *devc,
		int ch, double *low, double *high)
{
	double inc = devc->vdiv[ch] / 25.0;

	*low = (125 - V1_CODE_MAX) * inc - devc->vert_offset[ch];
	*high = (125 - V1_CODE_MIN) * inc - devc->vert_offset[ch];
}

static char *v1_volts_string(double v, gboolean milli)
{
	if (milli)
		return g_strdup_printf("%+.1f mV", v * 1000.0);
	return g_strdup_printf("%+.2f V", v);
}

/* The input range as text, for example "-8.80 V .. +1.40 V". */
SR_PRIV char *rigol_ds_v1_range_string(const struct dev_context *devc, int ch)
{
	double low, high;
	gboolean milli;
	char *l, *h, *res;

	rigol_ds_v1_input_range(devc, ch, &low, &high);
	milli = fabs(low) < 1.0 && fabs(high) < 1.0;
	l = v1_volts_string(low, milli);
	h = v1_volts_string(high, milli);
	res = g_strdup_printf("%s .. %s", l, h);
	g_free(l);
	g_free(h);

	return res;
}

/*
 * VS5000: stop the acquisition and wait until the scope reports STOP (up to
 * the memory span plus 2 s, the scope finishes the record it is writing).
 */
SR_PRIV int rigol_ds_v1_stop(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	int64_t deadline;
	char *buf;
	gboolean stopped;

	if (rigol_ds_config_set(sdi, ":STOP") != SR_OK)
		return SR_ERR;
	deadline = g_get_monotonic_time() + 2000000 +
		(int64_t)(V1_MEMORY_DIVS * devc->timebase * 1e6);
	do {
		if (sr_scpi_get_string(sdi->conn, ":TRIG:STAT?", &buf) != SR_OK)
			return SR_ERR;
		stopped = buf[0] == 'S';
		g_free(buf);
		if (stopped)
			return SR_OK;
		g_usleep(20000);
	} while (g_get_monotonic_time() < deadline);
	sr_warn("Scope did not report STOP after :STOP.");

	return SR_OK;
}

/*
 * Patch level of VS5000 firmware with the community LAN fixes: its *IDN?
 * version is "03.00.00.01.P<n>" (n = 1..99). Rigol's own versions are
 * numeric, so anything else is stock firmware and returns 0.
 */
SR_PRIV int rigol_ds_v1_fw_patch(const char *version)
{
	static const char base[] = "03.00.00.01.P";
	const char *p;
	char *end;
	unsigned long level;

	if (!version || strncmp(version, base, strlen(base)))
		return 0;
	p = version + strlen(base);
	if (!g_ascii_isdigit(*p))
		return 0;
	level = strtoul(p, &end, 10);
	if (*end || level < 1 || level > 99)
		return 0;

	return (int)level;
}

/*
 * The VS5000 acquisition memory can be read over its USB protocol, and
 * over LAN with :WAVEFORM:ALL? on patched firmware. Stock firmware
 * corrupts or drops that reply and can lock up its LAN interface, so it
 * is only used from patch level 1.
 */
SR_PRIV gboolean rigol_ds_v1_memory_supported(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_scpi_dev_inst *scpi = sdi->conn;

	if (!devc || !scpi || devc->model->series->protocol != PROTOCOL_V1)
		return FALSE;

	return scpi->transport == SCPI_TRANSPORT_USB_VS5000 ||
		(scpi->transport == SCPI_TRANSPORT_RIGOL_TCP &&
		 devc->v1_fw_patch >= 1);
}

/*
 * VS5000 over LAN: after a client was killed during a transfer, the
 * firmware's TCP stack can send copies of earlier replies on a later
 * connection (seen in packet captures, the firmware hands each reply over
 * only once), so every later answer is one or more off. A trigger status
 * is a few capital letters (STOP, WAIT, RUN, T'D); anything else means the
 * replies are out of step. See rigol_ds_v1_retry().
 */
static gboolean rigol_ds_v1_status_ok(const char *s)
{
	size_t i, n = strlen(s);

	if (n < 3 || n > 5)
		return FALSE;
	for (i = 0; i < n; i++)
		if (!g_ascii_isupper(s[i]) && s[i] != '\'')
			return FALSE;

	return TRUE;
}

/*
 * Reconnect over LAN. Commands sent while the firmware still winds down a
 * dropped reply (well under 0.3 s) get replies of their own, and all later
 * answers are one off: wait before the new connection.
 */
static int rigol_ds_v1_lan_reopen(struct sr_scpi_dev_inst *scpi)
{
	sr_scpi_close(scpi);
	g_usleep(500 * 1000);

	return sr_scpi_open(scpi);
}

/*
 * In scan mode (50 ms/div and slower) the VS5000 writes the record in
 * place while it acquires. :PAR:ROLL_POINT? returns the number of ADC
 * samples still to be acquired after the trigger, and 0 once the sweep
 * is complete; the record then stays unchanged until the next trigger.
 * Wait for a sweep to start and to finish, so that every frame is one
 * complete new sweep. Poll once per call to keep the session responsive,
 * the receive callback calls again.
 */
static int rigol_ds_v1_sweep_wait(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	int remaining;

	if (!(devc = sdi->priv))
		return SR_ERR;

	if (sr_scpi_get_int(sdi->conn, ":PAR:ROLL_POINT?", &remaining) != SR_OK)
		return ((struct sr_scpi_dev_inst *)sdi->conn)->transport ==
			SCPI_TRANSPORT_RIGOL_TCP ? SR_ERR_DATA : SR_ERR;

	if (devc->wait_status == 1) {
		if (remaining == 0)
			return SR_ERR_TIMEOUT;
		devc->wait_status = 2;
	}
	if (remaining != 0)
		return SR_ERR_TIMEOUT;

	rigol_ds_set_wait_event(devc, WAIT_NONE);
	return SR_OK;
}

/*
 * VS5000 below scan mode: wait until a new record is there. Over USB the
 * scope queues one record per trigger, and none while it waits for a
 * trigger or is stopped. Over LAN only the trigger status is known.
 * Poll once per call, the receive callback calls again.
 */
static int rigol_ds_v1_record_wait(const struct sr_dev_inst *sdi)
{
	struct sr_scpi_dev_inst *scpi;
	struct dev_context *devc;
	gboolean ready;
	char *buf;
	int queued;

	scpi = sdi->conn;
	if (!(devc = sdi->priv))
		return SR_ERR;

	if (scpi->transport == SCPI_TRANSPORT_USB_VS5000) {
		/* Transport-private, see scpi_usb_vs5000.c. */
		if (sr_scpi_get_int(scpi, ":WAVEFORM:QUEUED?", &queued) != SR_OK)
			return SR_ERR;
		ready = queued != 0;
	} else {
		if (sr_scpi_get_string(scpi, ":TRIG:STAT?", &buf) != SR_OK)
			return SR_ERR;
		if (!rigol_ds_v1_status_ok(buf)) {
			sr_warn("Unexpected trigger status '%.20s'.", buf);
			g_free(buf);
			return SR_ERR_DATA;
		}
		/* WAIT: no trigger yet. SINGLE sweep: wait for its STOP. */
		ready = buf[0] != 'W' && (!devc->v1_single || buf[0] == 'S');
		g_free(buf);
	}
	if (!ready)
		return SR_ERR_TIMEOUT;

	rigol_ds_set_wait_event(devc, WAIT_NONE);
	return SR_OK;
}

static int rigol_ds_trigger_wait(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	long s;

	if (!(devc = sdi->priv))
		return SR_ERR;

	/*
	 * If timebase < 50 msecs/DIV just sleep about one sweep time except
	 * for really fast sweeps.
	 */
	if (devc->model->series->protocol == PROTOCOL_V1)
		return devc->timebase >= 0.0499 ?
			rigol_ds_v1_sweep_wait(sdi) :
			rigol_ds_v1_record_wait(sdi);

	if (devc->timebase < 0.0499) {
		if (devc->timebase > 0.99e-6) {
			/*
			 * Timebase * num hor. divs * 85(%) * 1e6(usecs) / 100
			 * -> 85 percent of sweep time
			 */
			s = (devc->timebase * devc->model->series->num_horizontal_divs
			     * 85e6) / 100L;
			sr_spew("Sleeping for %ld usecs instead of trigger-wait", s);
			g_usleep(s);
		}
		rigol_ds_set_wait_event(devc, WAIT_NONE);
		return SR_OK;
	} else {
		return rigol_ds_event_wait(sdi, 'T', 'A');
	}
}

/* Wait for scope to got to "Stop" in single shot mode */
/*
 * VS5000 memory mode: wait for the end of the single shot, one status query
 * per call. Like the live mode, the user's sweep decides what happens
 * without a trigger: with AUTO the shot is triggered with :FORC once the
 * memory has been filled (memory span plus a margin, so that a real trigger
 * wins and the pre-trigger part holds data of this shot); NORMAL and SINGLE
 * wait for a real trigger.
 */
static int rigol_ds_v1_memory_wait(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	int64_t timeout_us;
	char *buf, c;

	if (sr_scpi_get_string(sdi->conn, ":TRIG:STAT?", &buf) != SR_OK)
		return SR_ERR;
	if (!rigol_ds_v1_status_ok(buf)) {
		sr_warn("Unexpected trigger status '%.20s'.", buf);
		g_free(buf);
		return SR_ERR_DATA;
	}
	c = buf[0];
	g_free(buf);

	/* Right after :RUN the status can still be the previous STOP. */
	if (devc->wait_status == 1) {
		if (c == 'S')
			return SR_ERR_TIMEOUT;
		devc->wait_status = 2;
	}
	if (c == 'S') {
		rigol_ds_set_wait_event(devc, WAIT_NONE);
		return SR_OK;
	}

	timeout_us = V1_FORCE_MARGIN_US +
		(int64_t)(V1_MEMORY_DIVS * devc->timebase * 1e6);
	if (c == 'W' && devc->v1_sweep &&
			!g_ascii_strcasecmp(devc->v1_sweep, "AUTO") && !devc->v1_forced &&
			g_get_monotonic_time() - devc->v1_shot_start > timeout_us) {
		sr_dbg("Forcing the trigger.");
		if (rigol_ds_config_set(sdi, ":FORC") != SR_OK)
			return SR_ERR;
		devc->v1_forced = TRUE;
	}

	return SR_ERR_TIMEOUT;
}

static int rigol_ds_stop_wait(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	int64_t now, force_at, give_up;
	gboolean force;
	char *buf, c;

	if (devc->model->series->protocol == PROTOCOL_V1 &&
			devc->data_source == DATA_SOURCE_MEMORY)
		return rigol_ds_v1_memory_wait(sdi);

	if (devc->data_source != DATA_SOURCE_MEMORY && !devc->live_single)
		return rigol_ds_event_wait(sdi, 'S', 'S');

	/*
	 * Memory captures (and live captures with the SINGLE sweep) start a
	 * single shot with :SING. Until the scope
	 * has taken the command it still reports the stop of the previous
	 * shot (for about 100 ms on the DS1000Z), so only a stop after a
	 * non-stop status counts (or after a second, in case the shot was
	 * too fast to see it running).
	 *
	 * With the Auto sweep the user expects a waveform without a trigger,
	 * but a single shot waits for one: force a trigger when none came
	 * within the screen time plus a margin.
	 */
	force = devc->saved_sweep &&
		!g_ascii_strncasecmp(devc->saved_sweep, "AUTO", 4);
	force_at = devc->single_start + 300000 + (int64_t)(1e6 *
		devc->timebase * devc->model->series->num_horizontal_divs);
	give_up = g_get_monotonic_time() + 2000000;
	for (;;) {
		if (sr_scpi_get_string(sdi->conn, ":TRIG:STAT?", &buf) != SR_OK)
			return SR_ERR;
		c = buf[0];
		g_free(buf);
		now = g_get_monotonic_time();
		if (c != 'S') {
			devc->single_running = TRUE;
		} else if (devc->single_running ||
				now - devc->single_start > 1000000) {
			rigol_ds_set_wait_event(devc, WAIT_NONE);
			return SR_OK;
		}
		if (force && !devc->single_forced && now > force_at) {
			sr_dbg("No trigger with the Auto sweep, forcing one.");
			if (rigol_ds_config_set(sdi, ":TFOR") != SR_OK)
				return SR_ERR;
			devc->single_forced = TRUE;
		}
		if (now > give_up)
			/* Don't block the application, try again later. */
			return SR_ERR_TIMEOUT;
		g_usleep(10000);
	}
}

/* Check that a single shot acquisition actually succeeded on the DS2000 */
static int rigol_ds_check_stop(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_channel *ch;
	int tmp;

	if (!(devc = sdi->priv))
		return SR_ERR;

	ch = devc->channel_entry->data;

	if (devc->model->series->protocol != PROTOCOL_V3)
		return SR_OK;

	if (ch->type == SR_CHANNEL_LOGIC) {
		if (rigol_ds_config_set(sdi, ":WAV:SOUR LA") != SR_OK)
			return SR_ERR;
	} else {
		if (rigol_ds_config_set(sdi, ":WAV:SOUR CHAN%d",
				ch->index + 1) != SR_OK)
			return SR_ERR;
	}
	/* Check that the number of samples will be accepted */
	if (rigol_ds_config_set(sdi, ":WAV:POIN %d",
			ch->type == SR_CHANNEL_LOGIC ?
				devc->digital_frame_size :
				devc->analog_frame_size) != SR_OK)
		return SR_ERR;
	if (sr_scpi_get_int(sdi->conn, "*ESR?", &tmp) != SR_OK)
		return SR_ERR;
	/*
	 * If we get an "Execution error" the scope went from "Single" to
	 * "Stop" without actually triggering. There is no waveform
	 * displayed and trying to download one will fail - the scope thinks
	 * it has 1400 samples (like display memory) and the driver thinks
	 * it has a different number of samples.
	 *
	 * In that case just try to capture something again. Might still
	 * fail in interesting ways.
	 *
	 * Ain't firmware fun?
	 */
	if (tmp & 0x10) {
		sr_warn("Single shot acquisition failed, retrying...");
		/* Sleep a bit, otherwise the single shot will often fail */
		g_usleep(500 * 1000);
		rigol_ds_config_set(sdi, ":SING");
		rigol_ds_set_wait_event(devc, WAIT_STOP);
		return SR_ERR;
	}

	return SR_OK;
}

/* Wait for enough data becoming available in scope output buffer */
static int rigol_ds_block_wait(const struct sr_dev_inst *sdi)
{
	char *buf, c;
	struct dev_context *devc;
	time_t start;
	int len, ret;

	if (!(devc = sdi->priv))
		return SR_ERR;

	if (devc->model->series->protocol == PROTOCOL_V3) {

		start = time(NULL);

		do {
			if (time(NULL) - start >= 3) {
				sr_dbg("Timeout waiting for data block");
				return SR_ERR_TIMEOUT;
			}

			/*
			 * The scope copies data really slowly from sample
			 * memory to its output buffer, so try not to bother
			 * it too much with SCPI requests but don't wait too
			 * long for short sample frame sizes.
			 */
			g_usleep(devc->analog_frame_size < (15 * 1000) ? (100 * 1000) : (1000 * 1000));

			/* "READ,nnnn" (still working) or "IDLE,nnnn" (finished) */
			if (sr_scpi_get_string(sdi->conn, ":WAV:STAT?", &buf) != SR_OK)
				return SR_ERR;
			ret = parse_int(buf + 5, &len);
			c = buf[0];
			g_free(buf);
			if (ret != SR_OK)
				return SR_ERR;
		} while (c == 'R' && len < (1000 * 1000));
	}

	rigol_ds_set_wait_event(devc, WAIT_NONE);

	return SR_OK;
}

/* Send a configuration setting. */
SR_PRIV int rigol_ds_config_set(const struct sr_dev_inst *sdi, const char *format, ...)
{
	struct dev_context *devc = sdi->priv;
	va_list args;
	int ret;

	/*
	 * VS5000 while acquiring: settings may change from another thread
	 * (PulseView) while a waveform reply is being read. A command and
	 * its *OPC? would then mix with the reply, over LAN the query would
	 * read reply bytes as its answer. Queue the command, the acquisition
	 * sends it between two replies.
	 */
	if (devc->model->series->protocol == PROTOCOL_V1 && devc->v1_acquiring &&
			g_thread_self() != devc->v1_session_thread) {
		va_start(args, format);
		g_mutex_lock(&devc->v1_cmd_lock);
		devc->v1_pending_cmds = g_slist_append(devc->v1_pending_cmds,
			g_strdup_vprintf(format, args));
		g_mutex_unlock(&devc->v1_cmd_lock);
		va_end(args);
		return SR_OK;
	}

	va_start(args, format);
	ret = sr_scpi_send_variadic(sdi->conn, format, args);
	va_end(args);

	if (ret != SR_OK)
		return SR_ERR;

	if (devc->model->series->protocol == PROTOCOL_V2) {
		/* The DS1000 series needs this stupid delay, *OPC? doesn't work. */
		sr_spew("delay %dms", 100);
		g_usleep(100 * 1000);
		return SR_OK;
	} else {
		return sr_scpi_get_opc(sdi->conn);
	}
}

/*
 * Memory depth per channel of the current setting (V4: DS1000Z, DS4000).
 * It can be chosen in the menu and depends on the number of active
 * channels and on installed options (DS1000Z: 12M points, 24M with the
 * memory option), so it is read from the scope. In the automatic setting
 * the scope reports "AUTO" and uses sample rate x screen time.
 * Returns the number of points, or 0 if it cannot be determined.
 */
static int rigol_ds_memory_depth(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	char *buf;
	float srate;
	int depth;

	if (sr_scpi_get_string(sdi->conn, ":ACQ:MDEP?", &buf) != SR_OK)
		return 0;
	if (sr_atoi(buf, &depth) == SR_OK && depth > 0) {
		g_free(buf);
		sr_dbg("Memory depth %d points.", depth);
		return depth;
	}
	sr_dbg("Memory depth '%s', computing it from the sample rate.", buf);
	g_free(buf);
	if (sr_scpi_get_float(sdi->conn, ":ACQ:SRAT?", &srate) != SR_OK ||
			srate <= 0)
		return 0;
	depth = (int)(srate * devc->timebase *
		devc->model->series->num_horizontal_divs + 0.5);
	sr_dbg("Memory depth %d points (%g Sa/s).", depth, srate);
	return depth;
}

/*
 * Points to request per :WAV:DATA? when the memory is read in blocks. The
 * DS1000Z returns up to 250000 points per query in the BYTE format
 * (programming guide); every query costs about 100 ms, so fewer and larger
 * blocks are much faster.
 */
static int rigol_ds_block_points(const struct dev_context *devc)
{
	if (!strcmp(devc->model->series->name, "DS1000Z"))
		return 250 * 1000;
	return ACQ_BLOCK_SIZE;
}

/* Start capturing a new frameset */
SR_PRIV int rigol_ds_capture_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	gchar *trig_mode;
	unsigned int num_channels, i, j;
	int buffer_samples;
	int ret;

	if (!(devc = sdi->priv))
		return SR_ERR;

	const gboolean first_frame = (devc->num_frames == 0);

	uint64_t limit_frames = devc->limit_frames;
	if (devc->num_frames_segmented != 0 && devc->num_frames_segmented < limit_frames)
		limit_frames = devc->num_frames_segmented;
	if (limit_frames == 0)
		sr_dbg("Starting data capture for frameset %" PRIu64,
		       devc->num_frames + 1);
	else
		sr_dbg("Starting data capture for frameset %" PRIu64 " of %"
		       PRIu64, devc->num_frames + 1, limit_frames);

	switch (devc->model->series->protocol) {
	case PROTOCOL_V1:
		if (devc->data_source == DATA_SOURCE_MEMORY) {
			/*
			 * Take a single shot (the sweep was set to SINGLE) and
			 * read the memory once the scope has stopped. Over USB
			 * the command returns after it has been executed, so
			 * the STOP that follows is the end of this shot.
			 */
			if (rigol_ds_config_set(sdi, ":RUN") != SR_OK)
				return SR_ERR;
			devc->v1_shot_start = g_get_monotonic_time();
			devc->v1_forced = FALSE;
			rigol_ds_set_wait_event(devc, WAIT_STOP);
		} else {
			/*
			 * The SINGLE sweep stops after each shot, arm the next
			 * one. acquisition_start armed the first.
			 */
			if (devc->v1_single && !first_frame &&
					rigol_ds_config_set(sdi, ":RUN") != SR_OK)
				return SR_ERR;
			rigol_ds_set_wait_event(devc, WAIT_TRIGGER);
		}
		break;
	case PROTOCOL_V2:
		if (devc->data_source == DATA_SOURCE_LIVE) {
			if (rigol_ds_config_set(sdi, ":WAV:POIN:MODE NORMAL") != SR_OK)
				return SR_ERR;
			rigol_ds_set_wait_event(devc, WAIT_TRIGGER);
		} else {
			if (rigol_ds_config_set(sdi, ":STOP") != SR_OK)
				return SR_ERR;
			if (rigol_ds_config_set(sdi, ":WAV:POIN:MODE RAW") != SR_OK)
				return SR_ERR;
			if (sr_scpi_get_string(sdi->conn, ":TRIG:MODE?", &trig_mode) != SR_OK)
				return SR_ERR;
			ret = rigol_ds_config_set(sdi, ":TRIG:%s:SWE SING", trig_mode);
			g_free(trig_mode);
			if (ret != SR_OK)
				return SR_ERR;
			if (rigol_ds_config_set(sdi, ":RUN") != SR_OK)
				return SR_ERR;
			rigol_ds_set_wait_event(devc, WAIT_STOP);
		}
		break;
	case PROTOCOL_V3:
	case PROTOCOL_V4:
	case PROTOCOL_V5:
		if (first_frame && rigol_ds_config_set(sdi, ":WAV:FORM BYTE") != SR_OK)
			return SR_ERR;
		if (devc->data_source == DATA_SOURCE_LIVE) {
			if (first_frame && rigol_ds_config_set(sdi, ":WAV:MODE NORM") != SR_OK)
				return SR_ERR;
			devc->analog_frame_size = devc->model->series->live_samples;
			devc->digital_frame_size = devc->model->series->live_samples;
			if (devc->live_single) {
				/* The scope stops after each shot: take a new one
				 * per frame instead of reading the same one again. */
				if (rigol_ds_config_set(sdi, ":SING") != SR_OK)
					return SR_ERR;
				devc->single_start = g_get_monotonic_time();
				devc->single_running = FALSE;
				devc->single_forced = FALSE;
				rigol_ds_set_wait_event(devc, WAIT_STOP);
			} else {
				rigol_ds_set_wait_event(devc, WAIT_TRIGGER);
			}
		} else {
			if (devc->model->series->protocol == PROTOCOL_V3) {
				if (first_frame && rigol_ds_config_set(sdi, ":WAV:MODE RAW") != SR_OK)
					return SR_ERR;
			} else if (devc->model->series->protocol >= PROTOCOL_V4) {
				num_channels = 0;

				/* Channels 3 and 4 are multiplexed with D0-7 and D8-15 */
				for (i = 0; i < devc->model->analog_channels; i++) {
					if (devc->analog_channels[i]) {
						num_channels++;
					} else if (i >= 2 && devc->model->has_digital) {
						for (j = 0; j < 8; j++) {
							if (devc->digital_channels[8 * (i - 2) + j]) {
								num_channels++;
								break;
							}
						}
					}
				}

				if (first_frame) {
					buffer_samples = rigol_ds_memory_depth(sdi);
					if (buffer_samples <= 0) {
						/* Fall back to the series' maximum, shared by
						 * the active channels. */
						buffer_samples = devc->model->series->buffer_samples /
							(num_channels == 1 ? 1 : num_channels == 2 ? 2 : 4);
						sr_warn("Cannot get the memory depth, assuming %d points.",
							buffer_samples);
					}
					devc->analog_frame_size = devc->digital_frame_size =
						buffer_samples;
				}
			}

			/* Memory: one new single shot per frame. Segmented reads
			 * the recorded frames, without a new acquisition. */
			if (devc->data_source == DATA_SOURCE_MEMORY) {
				if (rigol_ds_config_set(sdi, ":SING") != SR_OK)
					return SR_ERR;
				devc->single_start = g_get_monotonic_time();
				devc->single_running = FALSE;
				devc->single_forced = FALSE;
			}
			rigol_ds_set_wait_event(devc, WAIT_STOP);
			if (devc->data_source == DATA_SOURCE_SEGMENTED &&
					devc->model->series->protocol <= PROTOCOL_V4)
				if (rigol_ds_config_set(sdi, "FUNC:WREP:FCUR %d", devc->num_frames + 1) != SR_OK)
					return SR_ERR;
		}
		break;
	}

	return SR_OK;
}

/*
 * VS5000: samples per channel in the acquisition memory of the last shot.
 * 512K with ":ACQ:MEM FULL", 4096 (the display rate) with NORMAL. The
 * reply has a stray suffix ("524288l").
 */
static int rigol_ds_v1_memory_depth(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	char *resp, *end;
	guint64 depth;

	if (sr_scpi_get_string(sdi->conn, ":ACQ:MEM?", &resp) != SR_OK)
		return SR_ERR;
	depth = g_ascii_strtoull(resp, &end, 10);
	if (end == resp || !depth ||
			depth > (guint64)devc->model->series->buffer_samples) {
		sr_err("Unexpected memory depth '%s'.", resp);
		g_free(resp);
		return SR_ERR;
	}
	g_free(resp);
	if (depth != devc->analog_frame_size)
		sr_dbg("Memory depth %" PRIu64 " samples per channel.", depth);
	devc->analog_frame_size = depth;

	return SR_OK;
}

/* Start reading data from the current channel */
static uint32_t rigol_ds_v1_needed(const struct dev_context *devc,
		uint32_t *mask);

SR_PRIV int rigol_ds_channel_start(const struct sr_dev_inst *sdi)
{
	uint32_t mask;
	float sample_rate;
	struct dev_context *devc;
	struct sr_channel *ch;

	if (!(devc = sdi->priv))
		return SR_ERR;

	ch = devc->channel_entry->data;

	sr_dbg("Starting reading data from channel %d", ch->index + 1);

	const gboolean first_frame = (devc->num_frames == 0);

	switch (devc->model->series->protocol) {
	case PROTOCOL_V1:
		/*
		 * The VS5000 ignores the source argument of :WAV:DATA? and
		 * returns a stale transmit buffer. Its own PC software uses
		 * :WAVEFORM:DATA?, whose reply holds all displayed channels
		 * (CH1, CH2, then LA). Request it once per frame, the other
		 * channels are read from the same reply.
		 */
		if (devc->channel_entry == devc->enabled_channels) {
			if (devc->data_source == DATA_SOURCE_MEMORY) {
				/*
				 * :ACQ:SAMP? reports the rate of the last
				 * capture, so ask after each shot.
				 */
				if (sr_scpi_get_float(sdi->conn, ":ACQ:SAMP?",
						&sample_rate) != SR_OK || sample_rate <= 0) {
					sr_err("Cannot get the sample rate (:ACQ:SAMP?).");
					return SR_ERR;
				}
				if (llround(sample_rate) != llround(devc->sample_rate)) {
					devc->sample_rate = sample_rate;
					sr_session_send_meta(sdi, SR_CONF_SAMPLERATE,
						g_variant_new_uint64(llround(sample_rate)));
				}
				if (rigol_ds_v1_memory_depth(sdi) != SR_OK)
					return SR_ERR;
				rigol_ds_v1_needed(devc, &mask);
				if (((struct sr_scpi_dev_inst *)sdi->conn)->transport ==
						SCPI_TRANSPORT_USB_VS5000) {
					/* Transport-private, see scpi_usb_vs5000.c. */
					if (sr_scpi_send(sdi->conn,
							":WAVEFORM:MEMORY? %u", mask) != SR_OK)
						return SR_ERR;
				} else {
					/*
					 * LAN, patched firmware: the memory of
					 * the displayed channels (= the enabled
					 * ones, see acquisition_start), CH1 then
					 * CH2, with a length prefix that counts
					 * only the data.
					 */
					if (sr_scpi_send(sdi->conn,
							":WAVEFORM:ALL?") != SR_OK)
						return SR_ERR;
				}
			} else if (sr_scpi_send(sdi->conn, ":WAVEFORM:DATA?") != SR_OK)
				return SR_ERR;
			devc->v1_header_pending = TRUE;
		}
		rigol_ds_set_wait_event(devc, WAIT_NONE);
		break;
	case PROTOCOL_V2:
		if (ch->type == SR_CHANNEL_LOGIC) {
			if (sr_scpi_send(sdi->conn, ":WAV:DATA? DIG") != SR_OK)
				return SR_ERR;
		} else {
			if (sr_scpi_send(sdi->conn, ":WAV:DATA? CHAN%d",
					ch->index + 1) != SR_OK)
				return SR_ERR;
		}
		rigol_ds_set_wait_event(devc, WAIT_NONE);
		break;
	case PROTOCOL_V3:
		if (ch->type == SR_CHANNEL_LOGIC) {
			if (rigol_ds_config_set(sdi, ":WAV:SOUR LA") != SR_OK)
				return SR_ERR;
		} else {
			if (rigol_ds_config_set(sdi, ":WAV:SOUR CHAN%d",
					ch->index + 1) != SR_OK)
				return SR_ERR;
		}
		if (devc->data_source != DATA_SOURCE_LIVE) {
			if (rigol_ds_config_set(sdi, ":WAV:RES") != SR_OK)
				return SR_ERR;
			if (rigol_ds_config_set(sdi, ":WAV:BEG") != SR_OK)
				return SR_ERR;
		}
		break;
	case PROTOCOL_V4:
	case PROTOCOL_V5:
		if (ch->type == SR_CHANNEL_ANALOG) {
			if (rigol_ds_config_set(sdi, ":WAV:SOUR CHAN%d",
					ch->index + 1) != SR_OK)
				return SR_ERR;
		} else {
			if (rigol_ds_config_set(sdi, ":WAV:SOUR D%d",
					ch->index) != SR_OK)
				return SR_ERR;
		}

		if (first_frame && rigol_ds_config_set(sdi,
					devc->data_source == DATA_SOURCE_LIVE ?
						":WAV:MODE NORM" :":WAV:MODE RAW") != SR_OK)
			return SR_ERR;

		if (devc->data_source != DATA_SOURCE_LIVE) {
			if (rigol_ds_config_set(sdi, ":WAV:RES") != SR_OK)
				return SR_ERR;
		}
		break;
	}

	if (devc->model->series->protocol >= PROTOCOL_V3 &&
			ch->type == SR_CHANNEL_ANALOG) {
		/* Vertical increment. */
		if (first_frame && sr_scpi_get_float(sdi->conn, ":WAV:YINC?",
				&devc->vert_inc[ch->index]) != SR_OK)
			return SR_ERR;
		/* Vertical origin. */
		if (first_frame && sr_scpi_get_float(sdi->conn, ":WAV:YOR?",
			&devc->vert_origin[ch->index]) != SR_OK)
			return SR_ERR;
		/* Vertical reference. */
		if (first_frame && sr_scpi_get_int(sdi->conn, ":WAV:YREF?",
				&devc->vert_reference[ch->index]) != SR_OK)
			return SR_ERR;
	} else if (ch->type == SR_CHANNEL_ANALOG) {
		/* The VS5000 uses 25 codes per vertical division. */
		devc->vert_inc[ch->index] = devc->vdiv[ch->index] /
			(devc->model->series->protocol == PROTOCOL_V1 ? 25.0 : 25.6);
	}

	rigol_ds_set_wait_event(devc, WAIT_BLOCK);

	devc->num_channel_bytes = 0;
	devc->num_header_bytes = 0;
	devc->num_block_bytes = 0;

	return SR_OK;
}

/*
 * Read the header of a VS5000 :WAVEFORM:DATA? reply.
 *
 * The reply is a 20-byte header followed by 2048 bytes per displayed
 * analog channel and 4 trailing bytes. The first header word is the
 * total reply length including itself; the tcp-rigol transport strips
 * it as a length prefix. The remaining 16 header bytes are:
 *   u32 flags (0x82000000 seen), u32 reply length minus 4,
 *   u32 unknown, u32 source mask (bit 0 CH1, bit 1 CH2, bit 2 LA;
 *   bit 16 toggles between replies).
 *
 * Returns 1 when the header is complete, 0 if more data is needed,
 * -1 on error.
 */
#define V1_HEADER_BYTES 16
#define V1_USB_HEADER_BYTES 2048

/* Read and drop len bytes of the current reply. */
static int rigol_ds_v1_skip(struct sr_dev_inst *sdi, uint64_t len)
{
	struct dev_context *devc = sdi->priv;
	int ret, idle;

	for (idle = 0; len && idle < 1000; ) {
		ret = sr_scpi_read_data(sdi->conn, (char *)devc->buffer,
			MIN(len, ACQ_BUFFER_SIZE));
		if (ret < 0)
			return SR_ERR;
		if (ret == 0) {
			idle++;
			g_usleep(1000);
			continue;
		}
		len -= MIN((uint64_t)ret, len);
	}

	return len ? SR_ERR : SR_OK;
}

/* Bytes of one record for the enabled channels, and their source mask. */
static uint32_t rigol_ds_v1_needed(const struct dev_context *devc,
		uint32_t *mask)
{
	struct sr_channel *ch;
	const GSList *l;
	uint32_t needed;

	*mask = 0;
	needed = 0;
	for (l = devc->enabled_channels; l; l = l->next) {
		ch = l->data;
		if (ch->type == SR_CHANNEL_ANALOG) {
			*mask |= 1 << ch->index;
			needed += devc->analog_frame_size;
		} else {
			*mask |= 1 << 2;
			needed += devc->digital_frame_size;
		}
	}

	return needed;
}

/*
 * Read the header of a VS5000 :WAVEFORM:DATA? reply.
 *
 * LAN (tcp-rigol): the reply is a 20-byte header followed by 2048 bytes
 * per displayed analog channel and 4 trailing bytes. The first header
 * word is the total reply length including itself; the tcp-rigol
 * transport strips it as a length prefix. The remaining 16 header bytes
 * are: u32 flags (0x82000000 seen), u32 reply length minus 4, u32
 * unknown, u32 source mask (bit 0 CH1, bit 1 CH2, bit 2 LA; bit 16
 * toggles between replies).
 *
 * USB (usb-vs5000): the reply is a 2048-byte header, followed by all
 * records acquired since the last request, oldest first, each holding
 * 2048 bytes per displayed analog channel. Header: u16 at offset 4 is
 * the number of records, u32 at offset 8 the total reply size. Only the
 * newest record is used, the older ones are skipped.
 *
 * Returns 1 when the header is complete, 0 if more data is needed,
 * -1 on error.
 */
static int rigol_ds_v1_read_header(struct sr_dev_inst *sdi)
{
	struct sr_scpi_dev_inst *scpi = sdi->conn;
	struct dev_context *devc = sdi->priv;
	uint32_t remaining, mask, expected_mask, needed, header_len;
	uint32_t num_records, record_size;
	gboolean usb;
	int ret;

	if (!devc->v1_header_pending)
		return 1;

	usb = scpi->transport == SCPI_TRANSPORT_USB_VS5000;
	header_len = usb ? V1_USB_HEADER_BYTES : V1_HEADER_BYTES;

	if (devc->data_source == DATA_SOURCE_MEMORY) {
		/* Acquisition memory: no header, the channels back to back. */
		if (sr_scpi_read_begin(scpi) != SR_OK)
			return -1;
		devc->v1_header_pending = FALSE;
		devc->v1_remaining = rigol_ds_v1_needed(devc, &mask);
		return 1;
	}

	if (devc->num_header_bytes == 0) {
		if (sr_scpi_read_begin(scpi) != SR_OK)
			return -1;
	}

	ret = sr_scpi_read_data(scpi, (char *)devc->buffer + devc->num_header_bytes,
		header_len - devc->num_header_bytes);
	if (ret < 0)
		return -1;
	devc->num_header_bytes += ret;
	if (devc->num_header_bytes < header_len)
		return 0;
	devc->num_header_bytes = 0;
	devc->v1_header_pending = FALSE;

	needed = rigol_ds_v1_needed(devc, &expected_mask);

	if (usb) {
		num_records = read_u16le(devc->buffer + 4);
		remaining = read_u32le(devc->buffer + 8);
		sr_dbg("Waveform header: %u records, %u bytes.",
			num_records, remaining);
		if (!num_records || remaining < header_len ||
				(remaining - header_len) % num_records) {
			sr_err("Invalid waveform header: %u records, %u bytes.",
				num_records, remaining);
			return -1;
		}
		record_size = (remaining - header_len) / num_records;
		if (record_size != needed) {
			/* The record holds the displayed channels. */
			sr_err("Waveform record has %u bytes, expected %u.",
				record_size, needed);
			rigol_ds_v1_skip(sdi, remaining - header_len);
			return -1;
		}
		if (num_records > 1 && rigol_ds_v1_skip(sdi,
				(uint64_t)(num_records - 1) * record_size) != SR_OK) {
			sr_err("Could not skip older waveform records.");
			return -1;
		}
		devc->v1_remaining = record_size;
		return 1;
	}

	remaining = read_u32le(devc->buffer + 4);
	mask = read_u32le(devc->buffer + 12) & 0xff;
	sr_dbg("Waveform header: flags 0x%08x, length %u, word2 0x%08x, mask 0x%02x",
		read_u32le(devc->buffer), remaining,
		read_u32le(devc->buffer + 8), mask);

	if (remaining < V1_HEADER_BYTES ||
			remaining - V1_HEADER_BYTES < needed) {
		sr_err("Waveform reply too short: %u bytes, need %u.",
			remaining, needed + V1_HEADER_BYTES);
		return -1;
	}
	if ((mask & expected_mask) != expected_mask) {
		sr_err("Waveform reply has sources 0x%02x, expected 0x%02x.",
			mask, expected_mask);
		return -1;
	}
	if (mask != expected_mask)
		sr_warn("Waveform reply has extra sources 0x%02x.",
			mask & ~expected_mask);
	devc->v1_remaining = remaining - V1_HEADER_BYTES;

	return 1;
}

/*
 * Restart the current frame (nothing of it has been sent yet). Returns
 * TRUE when the frame was restarted, at most 3 times in a row.
 *
 * USB, live: a waveform request failed, typically because the transport
 * had to reset the USB device to get the bulk pipe back in sync, which
 * drops the queued record. With the SINGLE sweep no other record follows,
 * so re-arm and wait for the next one instead of aborting.
 *
 * LAN, live or memory: the replies are out of step (an implausible
 * trigger status or waveform header, see rigol_ds_v1_status_ok()). Drop
 * the stream with the connection, check that the new one answers, read
 * the vertical settings again and take the frame again.
 */
/* Forget the current frame and wait for the next one (or take a new shot). */
static gboolean rigol_ds_v1_restart_frame(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	devc->v1_header_pending = FALSE;
	devc->v1_remaining = 0;
	devc->num_header_bytes = 0;
	devc->num_block_bytes = 0;
	devc->num_block_read = 0;
	devc->v1_fill = 0;
	devc->num_channel_bytes = 0;
	devc->channel_entry = devc->enabled_channels;
	if (devc->data_source == DATA_SOURCE_MEMORY) {
		/* A new single shot, as in rigol_ds_capture_start(). */
		if (rigol_ds_config_set(sdi, ":RUN") != SR_OK)
			return FALSE;
		devc->v1_shot_start = g_get_monotonic_time();
		devc->v1_forced = FALSE;
		rigol_ds_set_wait_event(devc, WAIT_STOP);
		return TRUE;
	}
	if (devc->v1_single && rigol_ds_config_set(sdi, ":RUN") != SR_OK)
		return FALSE;
	rigol_ds_set_wait_event(devc, WAIT_TRIGGER);

	return TRUE;
}

/*
 * The reconnect for rigol_ds_v1_retry(), from a one-shot timer: the
 * receive source cannot be replaced from inside its own callback (its
 * removal only takes effect when the callback returns, so a new source
 * for the same connection would be refused).
 */
static int rigol_ds_v1_resync_cb(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi = cb_data;
	struct sr_scpi_dev_inst *scpi = sdi->conn;
	struct dev_context *devc = sdi->priv;
	gboolean ok;
	char *idn;

	(void)fd;
	(void)revents;

	if (!devc->v1_resyncing)
		return FALSE;
	sr_scpi_source_remove(sdi->session, scpi);
	ok = rigol_ds_v1_lan_reopen(scpi) == SR_OK;
	if (ok)
		sr_scpi_source_add(sdi->session, scpi, G_IO_IN,
			devc->data_source == DATA_SOURCE_MEMORY ? 2 : 50,
			rigol_ds_receive, (void *)sdi);
	if (ok && sr_scpi_get_string(scpi, "*IDN?", &idn) == SR_OK) {
		ok = g_str_has_prefix(idn, "RIGOL");
		g_free(idn);
	} else {
		ok = FALSE;
	}
	ok = ok && rigol_ds_get_dev_cfg_vertical(sdi) == SR_OK;
	devc->v1_resyncing = FALSE;
	if (ok)
		ok = rigol_ds_v1_restart_frame(sdi);
	if (!ok) {
		sr_err("Could not get the replies in step again, aborting capture.");
		std_session_send_df_frame_end(sdi);
		sr_dev_acquisition_stop(sdi);
	}

	return FALSE;
}

/* Acquisition stopped while a reconnect was pending. */
SR_PRIV void rigol_ds_v1_cancel_resync(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	if (devc && devc->v1_resyncing) {
		devc->v1_resyncing = FALSE;
		sr_session_source_remove(sdi->session, -1);
	}
}

/*
 * Restart the current frame (nothing of it has been sent yet). Returns
 * TRUE when the frame will be repeated, at most 3 times in a row.
 *
 * USB, live: a waveform request failed, typically because the transport
 * had to reset the USB device to get the bulk pipe back in sync, which
 * drops the queued record. With the SINGLE sweep no other record follows,
 * so re-arm and wait for the next one instead of aborting.
 *
 * LAN, live or memory: the replies are out of step (an implausible
 * trigger status or waveform header, see rigol_ds_v1_status_ok()). Drop
 * the stream with the connection (rigol_ds_v1_resync_cb()), check that
 * the new one answers, read the vertical settings again and take the
 * frame again.
 */
static gboolean rigol_ds_v1_retry(const struct sr_dev_inst *sdi)
{
	struct sr_scpi_dev_inst *scpi = sdi->conn;
	struct dev_context *devc = sdi->priv;
	gboolean lan;

	lan = scpi->transport == SCPI_TRANSPORT_RIGOL_TCP;
	if (!(scpi->transport == SCPI_TRANSPORT_USB_VS5000 &&
			devc->data_source == DATA_SOURCE_LIVE) &&
			!(lan && (devc->data_source == DATA_SOURCE_LIVE ||
			devc->data_source == DATA_SOURCE_MEMORY)))
		return FALSE;
	if (++devc->v1_failures > 3)
		return FALSE;

	if (!lan) {
		sr_warn("No waveform data, waiting for the next trigger.");
		return rigol_ds_v1_restart_frame(sdi);
	}

	sr_warn("Replies from the scope out of step: reconnecting, repeating the frame.");
	devc->v1_resyncing = TRUE;
	rigol_ds_set_wait_event(devc, WAIT_NONE);
	if (sr_session_source_add(sdi->session, -1, 0, 10,
			rigol_ds_v1_resync_cb, (void *)sdi) != SR_OK) {
		devc->v1_resyncing = FALSE;
		return FALSE;
	}

	return TRUE;
}

/* Send the commands queued by rigol_ds_config_set(), in this thread. */
SR_PRIV void rigol_ds_v1_flush(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	GSList *cmds, *l;

	g_mutex_lock(&devc->v1_cmd_lock);
	cmds = devc->v1_pending_cmds;
	devc->v1_pending_cmds = NULL;
	g_mutex_unlock(&devc->v1_cmd_lock);

	for (l = cmds; l; l = l->next) {
		sr_dbg("Sending queued command '%s'.", (char *)l->data);
		if (sr_scpi_send(sdi->conn, "%s", (char *)l->data) != SR_OK ||
				sr_scpi_get_opc(sdi->conn) != SR_OK)
			sr_warn("Queued command '%s' failed.", (char *)l->data);
	}
	g_slist_free_full(cmds, g_free);
}

/*
 * VS5000 over LAN, acquisition stopped in the middle of a waveform reply:
 * read the rest, or the next query reads it as its answer. The USB
 * transport drops an unfinished reply itself.
 */
SR_PRIV void rigol_ds_v1_abort_reply(struct sr_dev_inst *sdi)
{
	struct sr_scpi_dev_inst *scpi = sdi->conn;
	struct dev_context *devc = sdi->priv;
	char buf[256];
	gint64 deadline;
	int len;

	if (!devc || devc->model->series->protocol != PROTOCOL_V1 ||
			scpi->transport == SCPI_TRANSPORT_USB_VS5000)
		return;

	/*
	 * A memory reply (up to 1 MiB at about 120 KB/s) takes too long to
	 * drain: drop it with the connection. Only used on patched firmware,
	 * which handles a connection closed during a reply. The caller has
	 * removed the receive source.
	 */
	if (devc->data_source == DATA_SOURCE_MEMORY &&
			(devc->v1_header_pending || devc->v1_remaining)) {
		sr_dbg("Dropping the memory reply: reconnecting.");
		if (rigol_ds_v1_lan_reopen(scpi) != SR_OK)
			sr_err("Could not reconnect after dropping the memory reply.");
		devc->v1_header_pending = FALSE;
		devc->num_header_bytes = 0;
		devc->v1_remaining = 0;
		return;
	}

	deadline = g_get_monotonic_time() + 2 * G_USEC_PER_SEC;
	while (devc->v1_header_pending && g_get_monotonic_time() < deadline) {
		len = rigol_ds_v1_read_header(sdi);
		if (len < 0)
			break;
		if (len == 0)
			g_usleep(1000);
	}
	if (devc->v1_remaining)
		sr_dbg("Dropping %" PRIu64 " bytes of the waveform reply.",
			devc->v1_remaining);
	while (devc->v1_remaining && g_get_monotonic_time() < deadline) {
		len = sr_scpi_read_data(scpi, buf,
			MIN(devc->v1_remaining, sizeof(buf)));
		if (len < 0)
			break;
		devc->v1_remaining -= MIN((uint64_t)len, devc->v1_remaining);
		if (len == 0)
			g_usleep(1000);
	}
	devc->v1_header_pending = FALSE;
	devc->num_header_bytes = 0;
	devc->v1_remaining = 0;
}

/* Discard the unread end of a VS5000 waveform reply. */
static int rigol_ds_v1_drain(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	char buf[64];
	int len, tries;

	for (tries = 0; devc->v1_remaining && tries < 100; tries++) {
		len = sr_scpi_read_data(sdi->conn, buf,
			MIN(devc->v1_remaining, sizeof(buf)));
		if (len < 0)
			return SR_ERR;
		devc->v1_remaining -= MIN((uint64_t)len, devc->v1_remaining);
		if (devc->v1_remaining && len == 0)
			g_usleep(1000);
	}

	return devc->v1_remaining ? SR_ERR : SR_OK;
}

/* Read the header of a data block */
static int rigol_ds_read_header(struct sr_dev_inst *sdi)
{
	struct sr_scpi_dev_inst *scpi = sdi->conn;
	struct dev_context *devc = sdi->priv;
	char *buf = (char *) devc->buffer;
	size_t header_length;
	int ret;

	/* Try to read the hashsign and length digit. */
	if (devc->num_header_bytes < 2) {
		ret = sr_scpi_read_data(scpi, buf + devc->num_header_bytes,
				2 - devc->num_header_bytes);
		if (ret < 0) {
			sr_err("Read error while reading data header.");
			return SR_ERR;
		}
		devc->num_header_bytes += ret;
	}

	if (devc->num_header_bytes < 2)
		return 0;

	if (buf[0] != '#' || !isdigit(buf[1]) || buf[1] == '0') {
		sr_err("Received invalid data block header '%c%c'.", buf[0], buf[1]);
		return SR_ERR;
	}

	header_length = 2 + buf[1] - '0';

	/* Try to read the length. */
	if (devc->num_header_bytes < header_length) {
		ret = sr_scpi_read_data(scpi, buf + devc->num_header_bytes,
				header_length - devc->num_header_bytes);
		if (ret < 0) {
			sr_err("Read error while reading data header.");
			return SR_ERR;
		}
		devc->num_header_bytes += ret;
	}

	if (devc->num_header_bytes < header_length)
		return 0;

	/* Read the data length. */
	buf[header_length] = '\0';

	if (parse_int(buf + 2, &ret) != SR_OK) {
		sr_err("Received invalid data block length '%s'.", buf + 2);
		return -1;
	}

	sr_dbg("Received data block header: '%s' -> block length %d", buf, ret);

	return ret;
}

/* VS5000: send raw codes of one analog channel as volts. */
static void rigol_ds_v1_send_analog(const struct sr_dev_inst *sdi,
		struct sr_channel *ch, const unsigned char *raw, int num_samples)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	float vdiv, offset, vdivlog;
	int i, digits;

	vdiv = devc->vert_inc[ch->index];
	offset = devc->vert_offset[ch->index];
	/* Codes 25..225 span the 8 divisions, centre at code 125. */
	for (i = 0; i < num_samples; i++)
		devc->data[i] = (125 - raw[i]) * vdiv - offset;
	vdivlog = log10f(vdiv);
	digits = -(int)vdivlog + (vdivlog < 0.0);
	sr_analog_init(&analog, &encoding, &meaning, &spec, digits);
	analog.meaning->channels = g_slist_append(NULL, ch);
	analog.num_samples = num_samples;
	analog.data = devc->data;
	analog.meaning->mq = SR_MQ_VOLTAGE;
	analog.meaning->unit = SR_UNIT_VOLT;
	analog.meaning->mqflags = 0;
	packet.type = SR_DF_ANALOG;
	packet.payload = &analog;
	sr_session_send(sdi, &packet);
	g_slist_free(analog.meaning->channels);
}

/*
 * VS5000 memory mode: the reply holds each channel's whole memory, one
 * channel after the other. Hold back the channels before the last one
 * and send all channels in turns once the last one arrives, so that
 * every packet of a channel is followed by the same range of the next
 * channel (the CSV output relies on that).
 */
static int rigol_ds_v1_memory_packet(const struct sr_dev_inst *sdi, int len)
{
	struct dev_context *devc = sdi->priv;
	const GSList *l;
	size_t frame, pos, need, k;

	frame = devc->analog_frame_size;
	pos = g_slist_position(devc->enabled_channels, devc->channel_entry);
	if (devc->num_channel_bytes + len > frame) {
		sr_err("Memory data exceeds the channel size.");
		return SR_ERR;
	}
	if (devc->channel_entry->next) {
		need = (pos + 1) * frame;
		if (devc->v1_mem_size < need) {
			devc->v1_mem = g_realloc(devc->v1_mem, need);
			devc->v1_mem_size = need;
		}
		memcpy(devc->v1_mem + pos * frame + devc->num_channel_bytes,
			devc->buffer, len);
		return SR_OK;
	}
	for (l = devc->enabled_channels, k = 0; l != devc->channel_entry;
			l = l->next, k++)
		rigol_ds_v1_send_analog(sdi, l->data, devc->v1_mem +
			k * frame + devc->num_channel_bytes, len);
	rigol_ds_v1_send_analog(sdi, devc->channel_entry->data,
		devc->buffer, len);

	return SR_OK;
}

SR_PRIV int rigol_ds_receive(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct sr_scpi_dev_inst *scpi;
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_datafeed_logic logic;
	double vdiv, offset, origin;
	int head;
	uint32_t num_samples = 0;
	int len, i, vref, ret;
	struct sr_channel *ch;
	gsize expected_data_bytes;

	(void)fd;

	if (!(sdi = cb_data))
		return TRUE;

	if (!(devc = sdi->priv))
		return TRUE;

	scpi = sdi->conn;

	if (!(revents == G_IO_IN || revents == 0))
		return TRUE;

	if (devc->model->series->protocol == PROTOCOL_V1 && devc->v1_resyncing)
		/* The old connection's data: dropped with it. */
		return TRUE;

	if (devc->model->series->protocol == PROTOCOL_V1) {
		devc->v1_session_thread = g_thread_self();
		/*
		 * Only between two waveform replies: WAIT_BLOCK is also set
		 * between the channels of one reply.
		 */
		if (!devc->v1_header_pending && !devc->v1_remaining &&
				devc->num_header_bytes == 0)
			rigol_ds_v1_flush(sdi);
	}

	const gboolean first_frame = (devc->num_frames == 0);

	switch (devc->wait_event) {
	case WAIT_NONE:
		break;
	case WAIT_TRIGGER:
		ret = rigol_ds_trigger_wait(sdi);
		if (ret == SR_ERR_DATA)
			goto out_of_step;
		if (ret != SR_OK)
			return TRUE;
		if (rigol_ds_channel_start(sdi) != SR_OK)
			goto start_failed;
		return TRUE;
	case WAIT_BLOCK:
		if (rigol_ds_block_wait(sdi) != SR_OK)
			return TRUE;
		break;
	case WAIT_STOP:
		ret = rigol_ds_stop_wait(sdi);
		if (ret == SR_ERR_DATA)
			goto out_of_step;
		if (ret != SR_OK)
			return TRUE;
		if (rigol_ds_check_stop(sdi) != SR_OK)
			return TRUE;
		if (rigol_ds_channel_start(sdi) != SR_OK)
			goto start_failed;
		return TRUE;
	default:
		sr_err("BUG: Unknown event target encountered");
		break;
	}

	ch = devc->channel_entry->data;

	expected_data_bytes = ch->type == SR_CHANNEL_ANALOG ?
			devc->analog_frame_size : devc->digital_frame_size;

	if (devc->num_block_bytes == 0) {
		/*
		 * Live frames always cover the same points. Memory is read in
		 * blocks, each block of each frame needs its own range.
		 */
		const gboolean set_range = first_frame ||
			devc->data_source != DATA_SOURCE_LIVE;
		if (devc->model->series->protocol >= PROTOCOL_V4) {
			if (set_range && rigol_ds_config_set(sdi, ":WAV:START %d",
					devc->num_channel_bytes + 1) != SR_OK)
				return TRUE;
			if (set_range && rigol_ds_config_set(sdi, ":WAV:STOP %d",
					MIN(devc->num_channel_bytes + rigol_ds_block_points(devc),
						devc->analog_frame_size)) != SR_OK)
				return TRUE;
		}

		if (devc->model->series->protocol >= PROTOCOL_V3) {
			if (rigol_ds_config_set(sdi, ":WAV:BEG") != SR_OK)
				return TRUE;
			if (sr_scpi_send(sdi->conn, ":WAV:DATA?") != SR_OK)
				return TRUE;
		}

		if (devc->model->series->protocol == PROTOCOL_V1) {
			len = rigol_ds_v1_read_header(sdi);
			if (len == 0)
				/* Still reading the header. */
				return TRUE;
			if (len < 0 && rigol_ds_v1_retry(sdi))
				return TRUE;
			if (len < 0) {
				sr_err("Error while reading waveform header, aborting capture.");
				std_session_send_df_frame_end(sdi);
				sr_dev_acquisition_stop(sdi);
				return TRUE;
			}
			devc->v1_failures = 0;
		} else if (sr_scpi_read_begin(scpi) != SR_OK)
			return TRUE;

		if (devc->format == FORMAT_IEEE488_2) {
			sr_dbg("New block header expected");
			len = rigol_ds_read_header(sdi);
			if (len == 0)
				/* Still reading the header. */
				return TRUE;
			if (len == -1) {
				sr_err("Error while reading block header, aborting capture.");
				std_session_send_df_frame_end(sdi);
				sr_dev_acquisition_stop(sdi);
				return TRUE;
			}
			/* At slow timebases in live capture the DS2072 and
			 * DS1054Z sometimes return "short" data blocks, with
			 * apparently no way to get the rest of the data.
			 * Discard these, the complete data block will appear
			 * eventually.
			 */
			if (devc->data_source == DATA_SOURCE_LIVE
					&& (unsigned)len < expected_data_bytes) {
				sr_dbg("Discarding short data block: got %d/%d bytes\n", len, (int)expected_data_bytes);
				sr_scpi_read_data(scpi, (char *)devc->buffer, len + 1);
				devc->num_header_bytes = 0;
				return TRUE;
			}
			devc->num_block_bytes = len;
		} else {
			devc->num_block_bytes = expected_data_bytes;
		}
		devc->num_block_read = 0;
		devc->v1_fill = 0;
	}

	/*
	 * V1: collect the data in packets of ACQ_BUFFER_SIZE (or the rest of
	 * the channel) and send each as one packet. Over LAN it arrives in
	 * TCP segments of any size, over USB in bulk reads of any size, and
	 * consumers such as the CSV output need equally sized packets for
	 * all channels.
	 */
	const gboolean v1 = devc->model->series->protocol == PROTOCOL_V1;

	len = devc->num_block_bytes - devc->num_block_read;
	if (len > ACQ_BUFFER_SIZE - (int)(v1 ? devc->v1_fill : 0))
		len = ACQ_BUFFER_SIZE - (int)(v1 ? devc->v1_fill : 0);
	sr_dbg("Requesting read of %d bytes", len);

	len = sr_scpi_read_data(scpi, (char *)devc->buffer +
		(v1 ? devc->v1_fill : 0), len);

	if (len == -1) {
		sr_err("Error while reading block data, aborting capture.");
		std_session_send_df_frame_end(sdi);
		sr_dev_acquisition_stop(sdi);
		return TRUE;
	}

	sr_dbg("Received %d bytes.", len);

	devc->num_block_read += len;
	if (v1)
		devc->v1_remaining -= MIN((uint64_t)len, devc->v1_remaining);
	if (len == 0)
		/* Nothing yet (a bulk read can be empty), try again. */
		return TRUE;
	if (v1) {
		devc->v1_fill += len;
		if (devc->v1_fill < (uint64_t)ACQ_BUFFER_SIZE &&
				devc->num_block_read < devc->num_block_bytes) {
			sr_dbg("%" PRIu64 " of %" PRIu64 " block bytes read",
				devc->num_block_read, devc->num_block_bytes);
			return TRUE;
		}
		len = devc->v1_fill;
		devc->v1_fill = 0;
	}

	if (v1 && devc->data_source == DATA_SOURCE_MEMORY) {
		if (rigol_ds_v1_memory_packet(sdi, len) != SR_OK) {
			std_session_send_df_frame_end(sdi);
			sr_dev_acquisition_stop(sdi);
			return TRUE;
		}
	} else if (ch->type == SR_CHANNEL_ANALOG) {
		vref = devc->vert_reference[ch->index];
		vdiv = devc->vert_inc[ch->index];
		origin = devc->vert_origin[ch->index];
		offset = devc->vert_offset[ch->index];
		if (devc->model->series->protocol >= PROTOCOL_V3)
			for (i = 0; i < len; i++)
				devc->data[i] = ((int)devc->buffer[i] - vref - origin) * vdiv;
		else if (devc->model->series->protocol == PROTOCOL_V1) {
			/*
			 * VS5000: codes 25..225 span the 8 vertical divisions
			 * of the screen, top to bottom, centre at code 125.
			 *
			 * The first V1_HEAD_SAMPLES bytes of each channel are
			 * the samples that follow the end of the record (the
			 * capture buffer is rotated). Hold them back and send
			 * them after the rest of the channel's data.
			 */
			head = 0;
			while (devc->data_source == DATA_SOURCE_LIVE &&
					devc->num_channel_bytes + head < V1_HEAD_SAMPLES &&
					head < len) {
				devc->v1_head[devc->num_channel_bytes + head] =
					devc->buffer[head];
				head++;
			}
			/*
			 * With both channels enabled, the last byte of a live
			 * record often reads 0 (far above the screen) instead
			 * of the sample. A real off-scale value cannot appear
			 * next to an on-screen one, so replace it with the
			 * mean of its neighbours in time: the byte before it,
			 * and the first held-back head byte.
			 */
			if (devc->data_source == DATA_SOURCE_LIVE &&
					devc->num_channel_bytes + len >= expected_data_bytes &&
					len - head >= 2 && devc->buffer[len - 1] == 0 &&
					devc->buffer[len - 2] >= V1_GLITCH_MIN_NEIGHBOUR)
				devc->buffer[len - 1] = (devc->buffer[len - 2] +
					devc->v1_head[0] + 1) / 2;
			num_samples = 0;
			for (i = head; i < len; i++)
				devc->data[num_samples++] =
					(125 - devc->buffer[i]) * vdiv - offset;
			if (devc->data_source == DATA_SOURCE_LIVE &&
					devc->num_channel_bytes + len >= expected_data_bytes)
				for (i = 0; i < V1_HEAD_SAMPLES; i++)
					devc->data[num_samples++] =
						(125 - devc->v1_head[i]) * vdiv - offset;
		} else
			for (i = 0; i < len; i++)
				devc->data[i] = (128 - devc->buffer[i]) * vdiv - offset;
		float vdivlog = log10f(vdiv);
		int digits = -(int)vdivlog + (vdivlog < 0.0);
		sr_analog_init(&analog, &encoding, &meaning, &spec, digits);
		analog.meaning->channels = g_slist_append(NULL, ch);
		analog.num_samples = devc->model->series->protocol == PROTOCOL_V1 ?
			num_samples : (uint32_t)len;
		analog.data = devc->data;
		analog.meaning->mq = SR_MQ_VOLTAGE;
		analog.meaning->unit = SR_UNIT_VOLT;
		analog.meaning->mqflags = 0;
		packet.type = SR_DF_ANALOG;
		packet.payload = &analog;
		sr_session_send(sdi, &packet);
		g_slist_free(analog.meaning->channels);
	} else {
		logic.length = len;
		// TODO: For the MSO1000Z series, we need a way to express that
		// this data is in fact just for a single channel, with the valid
		// data for that channel in the LSB of each byte.
		logic.unitsize = devc->model->series->protocol >= PROTOCOL_V4 ? 1 : 2;
		logic.data = devc->buffer;
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic;
		sr_session_send(sdi, &packet);
	}

	if (devc->num_block_read == devc->num_block_bytes) {
		sr_dbg("Block has been completed");
		if (devc->model->series->protocol >= PROTOCOL_V3) {
			/* Discard the terminating linefeed */
			sr_scpi_read_data(scpi, (char *)devc->buffer, 1);
		}
		if (devc->format == FORMAT_IEEE488_2) {
			/* Prepare for possible next block */
			devc->num_header_bytes = 0;
			devc->num_block_bytes = 0;
			if (devc->data_source != DATA_SOURCE_LIVE)
				rigol_ds_set_wait_event(devc, WAIT_BLOCK);
		}
		if (devc->model->series->protocol == PROTOCOL_V1) {
			/*
			 * The length prefix of the VS5000 waveform reply
			 * counts itself, so the transport's completion check
			 * does not apply. Drop the bytes after the last
			 * channel to keep the next reply aligned.
			 */
			if (!devc->channel_entry->next &&
					rigol_ds_v1_drain(sdi) != SR_OK)
				sr_warn("Could not drain end of waveform reply.");
			/*
			 * The :WAVEFORM:ALL? prefix counts only the data:
			 * here the transport can tell a reply of another
			 * length.
			 */
			if (!devc->channel_entry->next &&
					devc->data_source == DATA_SOURCE_MEMORY &&
					scpi->transport == SCPI_TRANSPORT_RIGOL_TCP &&
					!sr_scpi_read_complete(scpi)) {
				sr_err("Memory reply longer than %" PRIu64
					" bytes per channel.", (uint64_t)expected_data_bytes);
				std_session_send_df_frame_end(sdi);
				sr_dev_acquisition_stop(sdi);
				return TRUE;
			}
		} else if (!sr_scpi_read_complete(scpi) && !devc->channel_entry->next) {
			sr_err("Read should have been completed");
		}
		devc->num_block_read = 0;
	} else {
		sr_dbg("%" PRIu64 " of %" PRIu64 " block bytes read",
			devc->num_block_read, devc->num_block_bytes);
	}

	devc->num_channel_bytes += len;

	if (devc->num_channel_bytes < expected_data_bytes)
		/* Don't have the full data for this channel yet, re-run. */
		return TRUE;

	/* End of data for this channel. */
	if (devc->model->series->protocol == PROTOCOL_V3) {
		/* Signal end of data download to scope */
		if (devc->data_source != DATA_SOURCE_LIVE)
			/*
			 * This causes a query error, without it switching
			 * to the next channel causes an error. Fun with
			 * firmware...
			 */
			rigol_ds_config_set(sdi, ":WAV:END");
	}

	if (devc->channel_entry->next) {
		/* We got the frame for this channel, now get the next channel. */
		devc->channel_entry = devc->channel_entry->next;
		rigol_ds_channel_start(sdi);
	} else {
		/* Done with this frame. */
		std_session_send_df_frame_end(sdi);

		devc->num_frames++;

		/* V5 has no way to read the number of recorded frames, so try to set the
		 * next frame and read it back instead.
		 */
		if (devc->data_source == DATA_SOURCE_SEGMENTED &&
				devc->model->series->protocol == PROTOCOL_V5) {
			int frames = 0;
			if (rigol_ds_config_set(sdi, "REC:CURR %d", devc->num_frames + 1) != SR_OK)
				return SR_ERR;
			if (sr_scpi_get_int(sdi->conn, "REC:CURR?", &frames) != SR_OK)
				return SR_ERR;
			devc->num_frames_segmented = frames;
		}

		/* Memory takes a new single shot per frame, like live mode
		 * it runs until the frame limit. */
		if (devc->num_frames == devc->limit_frames ||
				devc->num_frames == devc->num_frames_segmented) {
			/* Last frame, stop capture. */
			sr_dev_acquisition_stop(sdi);
		} else {
			/* Get the next frame, starting with the first channel. */
			devc->channel_entry = devc->enabled_channels;

			rigol_ds_capture_start(sdi);

			/* Start of next frame. */
			std_session_send_df_frame_begin(sdi);
		}
	}

	return TRUE;

out_of_step:
	if (rigol_ds_v1_retry(sdi))
		return TRUE;
	sr_err("Replies from the scope out of step, aborting capture.");
	std_session_send_df_frame_end(sdi);
	sr_dev_acquisition_stop(sdi);
	return TRUE;

start_failed:
	if (devc->model->series->protocol == PROTOCOL_V1 && rigol_ds_v1_retry(sdi))
		return TRUE;
	/* Nothing was requested, there is no reply to read. */
	sr_err("Cannot request waveform data, aborting capture.");
	std_session_send_df_frame_end(sdi);
	sr_dev_acquisition_stop(sdi);
	return TRUE;
}

SR_PRIV int rigol_ds_get_dev_cfg(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_channel *ch;
	char *cmd, *t_s;
	unsigned int i;
	int res;

	devc = sdi->priv;

	/* Analog channel state. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:DISP?", i + 1);
		res = sr_scpi_get_bool(sdi->conn, cmd, &devc->analog_channels[i]);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
		ch = g_slist_nth_data(sdi->channels, i);
		ch->enabled = devc->analog_channels[i];
	}
	sr_dbg("Current analog channel state:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %s", i + 1, devc->analog_channels[i] ? "on" : "off");

	/* Digital channel state. */
	if (devc->model->has_digital) {
		if (sr_scpi_get_bool(sdi->conn,
				devc->model->series->protocol >= PROTOCOL_V3 ?
					":LA:STAT?" : ":LA:DISP?",
				&devc->la_enabled) != SR_OK)
			return SR_ERR;
		sr_dbg("Logic analyzer %s, current digital channel state:",
				devc->la_enabled ? "enabled" : "disabled");
		for (i = 0; i < ARRAY_SIZE(devc->digital_channels); i++) {
			if (devc->model->series->protocol >= PROTOCOL_V5)
				cmd = g_strdup_printf(":LA:DISP? D%d", i);
			else if (devc->model->series->protocol >= PROTOCOL_V3)
				cmd = g_strdup_printf(":LA:DIG%d:DISP?", i);
			else
				cmd = g_strdup_printf(":DIG%d:TURN?", i);
			res = sr_scpi_get_bool(sdi->conn, cmd, &devc->digital_channels[i]);
			g_free(cmd);
			if (res != SR_OK)
				return SR_ERR;
			ch = g_slist_nth_data(sdi->channels, i + devc->model->analog_channels);
			ch->enabled = devc->digital_channels[i];
			sr_dbg("D%d: %s", i, devc->digital_channels[i] ? "on" : "off");
		}
	}

	/* Timebase. */
	if (sr_scpi_get_float(sdi->conn, ":TIM:SCAL?", &devc->timebase) != SR_OK)
		return SR_ERR;
	sr_dbg("Current timebase %g", devc->timebase);

	/* Probe attenuation. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:PROB?", i + 1);

		/* DSO1000B series prints an X after the probe factor, so
		 * we get a string and check for that instead of only handling
		 * floats. */
		char *response;
		res = sr_scpi_get_string(sdi->conn, cmd, &response);
		if (res != SR_OK)
			return SR_ERR;

		int len = strlen(response);
		if (response[len-1] == 'X')
			response[len-1] = 0;

		res = sr_atof_ascii(response, &devc->attenuation[i]);
		g_free(response);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
	}
	sr_dbg("Current probe attenuation:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %g", i + 1, devc->attenuation[i]);

	/* Vertical gain and offset. */
	if (rigol_ds_get_dev_cfg_vertical(sdi) != SR_OK)
		return SR_ERR;

	/* Coupling. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:COUP?", i + 1);
		g_free(devc->coupling[i]);
		devc->coupling[i] = NULL;
		res = sr_scpi_get_string(sdi->conn, cmd, &devc->coupling[i]);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
	}
	sr_dbg("Current coupling:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %s", i + 1, devc->coupling[i]);

	/* Trigger source. */
	g_free(devc->trigger_source);
	devc->trigger_source = NULL;
	if (sr_scpi_get_string(sdi->conn, ":TRIG:EDGE:SOUR?", &devc->trigger_source) != SR_OK)
		return SR_ERR;
	sr_dbg("Current trigger source %s", devc->trigger_source);

	/* Horizontal trigger position. */
	if (sr_scpi_get_float(sdi->conn, devc->model->cmds[CMD_GET_HORIZ_TRIGGERPOS].str,
			&devc->horiz_triggerpos) != SR_OK)
		return SR_ERR;
	sr_dbg("Current horizontal trigger position %g", devc->horiz_triggerpos);

	/* Trigger slope. */
	g_free(devc->trigger_slope);
	devc->trigger_slope = NULL;
	if (sr_scpi_get_string(sdi->conn, ":TRIG:EDGE:SLOP?", &devc->trigger_slope) != SR_OK)
		return SR_ERR;
	sr_dbg("Current trigger slope %s", devc->trigger_slope);

	/* Trigger level. */
	if (sr_scpi_get_float(sdi->conn, ":TRIG:EDGE:LEV?", &devc->trigger_level) != SR_OK)
		return SR_ERR;
	sr_dbg("Current trigger level %g", devc->trigger_level);

	/* Trigger sweep. */
	if (devc->model->series->protocol == PROTOCOL_V1) {
		if (sr_scpi_get_string(sdi->conn, ":TRIG:MODE?", &t_s) != SR_OK)
			return SR_ERR;
		g_free(devc->trigger_type);
		devc->trigger_type = t_s;
		cmd = g_strdup_printf(":TRIG:%s:SWE?", t_s);
		g_free(devc->trigger_sweep);
		devc->trigger_sweep = NULL;
		res = sr_scpi_get_string(sdi->conn, cmd, &devc->trigger_sweep);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
		sr_dbg("Current trigger sweep %s", devc->trigger_sweep);
	}

	return SR_OK;
}

SR_PRIV int rigol_ds_get_dev_cfg_vertical(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	char *cmd;
	unsigned int i;
	int res;

	devc = sdi->priv;

	/* Vertical gain. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:SCAL?", i + 1);
		res = sr_scpi_get_float(sdi->conn, cmd, &devc->vdiv[i]);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
	}
	sr_dbg("Current vertical gain:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %g", i + 1, devc->vdiv[i]);

	/* Vertical offset. */
	for (i = 0; i < devc->model->analog_channels; i++) {
		cmd = g_strdup_printf(":CHAN%d:OFFS?", i + 1);
		res = sr_scpi_get_float(sdi->conn, cmd, &devc->vert_offset[i]);
		g_free(cmd);
		if (res != SR_OK)
			return SR_ERR;
	}
	sr_dbg("Current vertical offset:");
	for (i = 0; i < devc->model->analog_channels; i++)
		sr_dbg("CH%d %g", i + 1, devc->vert_offset[i]);

	return SR_OK;
}
