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
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "scpi.h"
#include "protocol.h"

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
	SR_CONF_SERIALCOMM,
};

static const uint32_t drvopts[] = {
	SR_CONF_OSCILLOSCOPE,
};

static const uint32_t devopts[] = {
	SR_CONF_LIMIT_FRAMES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET,
	SR_CONF_TIMEBASE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_HDIV | SR_CONF_GET,
	SR_CONF_HORIZ_TRIGGERPOS | SR_CONF_SET,
	SR_CONF_TRIGGER_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_SLOPE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_LEVEL | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_DATA_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

/* VS5000: devopts plus the trigger sweep and the memory depth. */
static const uint32_t devopts_v1[] = {
	SR_CONF_LIMIT_FRAMES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET,
	SR_CONF_TIMEBASE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_HDIV | SR_CONF_GET,
	SR_CONF_HORIZ_TRIGGERPOS | SR_CONF_SET,
	SR_CONF_TRIGGER_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_SLOPE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_LEVEL | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_SWEEP | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_BUFFERSIZE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_DATA_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

/* V3 and later: devopts plus the trigger sweep. */
static const uint32_t devopts_sweep[] = {
	SR_CONF_LIMIT_FRAMES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET,
	SR_CONF_TIMEBASE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_HDIV | SR_CONF_GET,
	SR_CONF_HORIZ_TRIGGERPOS | SR_CONF_SET,
	SR_CONF_TRIGGER_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_SLOPE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_LEVEL | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_SWEEP | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_DATA_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

/*
 * VS5000 memory depths (:ACQ:MEM NORMAL / FULL): the 4096 points of the
 * display record at the display rate, or 512K points at the full rate.
 */
static const uint64_t v1_mem_depths[] = { 4096, 524288 };

static const uint32_t devopts_cg_analog[] = {
	SR_CONF_NUM_VDIV | SR_CONF_GET,
	SR_CONF_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_FACTOR | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_OFFSET | SR_CONF_GET | SR_CONF_SET,
};

/* VS5000: per channel also the input range given by V/div and offset. */
static const uint32_t devopts_cg_analog_v1[] = {
	SR_CONF_NUM_VDIV | SR_CONF_GET,
	SR_CONF_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_FACTOR | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_RANGE | SR_CONF_GET,
};

/* DS1000Z: devopts_v3 plus the memory depth. */
static const uint32_t devopts_ds1000z[] = {
	SR_CONF_LIMIT_FRAMES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET,
	SR_CONF_TIMEBASE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_HDIV | SR_CONF_GET,
	SR_CONF_HORIZ_TRIGGERPOS | SR_CONF_SET,
	SR_CONF_TRIGGER_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_SLOPE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_LEVEL | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_TRIGGER_SWEEP | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_BUFFERSIZE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_DATA_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

/* DS1000Z: per channel also the input range given by V/div and offset. */
static const uint32_t devopts_cg_analog_ds1000z[] = {
	SR_CONF_NUM_VDIV | SR_CONF_GET,
	SR_CONF_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_FACTOR | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_RANGE | SR_CONF_GET,
};

/*
 * DS1000Z memory depths in points with one active channel; halved with
 * two, quartered with three or four. 24M needs the memory option (the
 * scope rejects it otherwise).
 */
static const uint64_t ds1000z_mem_depths[] = {
	12000, 120000, 1200000, 12000000, 24000000,
};

static gboolean is_ds1000z(const struct dev_context *devc)
{
	return devc && !strcmp(devc->model->series->name, "DS1000Z");
}

/*
 * DS1000Z input range of a channel: the BYTE samples are 25 codes per
 * division, (code - 127) * V/div / 25 - offset, codes 0 to 255.
 */
static char *ds1000z_range_string(const struct dev_context *devc, int ch)
{
	double inc = devc->vdiv[ch] / 25.0;
	double low = -127 * inc - devc->vert_offset[ch];
	double high = 128 * inc - devc->vert_offset[ch];

	if (fabs(low) < 1.0 && fabs(high) < 1.0)
		return g_strdup_printf("%+.1f mV .. %+.1f mV", low * 1000, high * 1000);
	return g_strdup_printf("%+.2f V .. %+.2f V", low, high);
}

/*
 * Active channels for the memory depth: the channels enabled for the
 * acquisition (they are switched on and off on the scope when it starts).
 * On models with digital channels, D0-D7 and D8-D15 take the places of
 * CH3 and CH4 when those are off.
 */
static int ds1000z_active_channels(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	gboolean used[MAX_ANALOG_CHANNELS] = { FALSE };
	struct sr_channel *ch;
	GSList *l;
	int i, n = 0;

	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (!ch->enabled)
			continue;
		if (ch->type == SR_CHANNEL_ANALOG && ch->index < MAX_ANALOG_CHANNELS)
			used[ch->index] = TRUE;
		else if (ch->type == SR_CHANNEL_LOGIC)
			used[ch->index < 8 ? 2 : 3] = TRUE;
	}
	for (i = 0; i < (int)devc->model->analog_channels; i++)
		if (used[i])
			n++;
	return n ? n : 1;
}

/* Memory depth in points of a level (see mem_level) with n channels. */
static uint64_t ds1000z_level_depth(int level, int n)
{
	if (level <= 0)
		return 0;
	return ds1000z_mem_depths[level - 1] / (n == 1 ? 1 : n == 2 ? 2 : 4);
}

/* Apply the chosen memory depth, after the channels are switched. */
static int ds1000z_apply_mem_depth(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint64_t want, got;
	char *buf = NULL;

	if (!is_ds1000z(devc) || devc->mem_level < 0)
		return SR_OK;
	want = ds1000z_level_depth(devc->mem_level, ds1000z_active_channels(sdi));
	/* The scope takes a new depth only while running. */
	if (sr_scpi_get_string(sdi->conn, ":TRIG:STAT?", &buf) != SR_OK)
		return SR_ERR;
	if (buf && buf[0] == 'S' && rigol_ds_config_set(sdi, ":RUN") != SR_OK) {
		g_free(buf);
		return SR_ERR;
	}
	g_free(buf);
	if (want)
		rigol_ds_config_set(sdi, ":ACQ:MDEP %" PRIu64, want);
	else
		rigol_ds_config_set(sdi, ":ACQ:MDEP AUTO");
	if (sr_scpi_get_string(sdi->conn, ":ACQ:MDEP?", &buf) != SR_OK)
		return SR_ERR;
	got = g_ascii_strtoull(buf, NULL, 10);
	g_free(buf);
	devc->mem_depth = got;
	if (got != want)
		sr_warn("Memory depth %" PRIu64 " not accepted (24M needs the "
			"memory option), the scope uses %" PRIu64 ".", want, got);
	return SR_OK;
}

static const uint64_t timebases[][2] = {
	/* nanoseconds */
	{ 1, 1000000000 },
	{ 2, 1000000000 },
	{ 5, 1000000000 },
	{ 10, 1000000000 },
	{ 20, 1000000000 },
	{ 50, 1000000000 },
	{ 100, 1000000000 },
	{ 200, 1000000000 },
	{ 500, 1000000000 },
	/* microseconds */
	{ 1, 1000000 },
	{ 2, 1000000 },
	{ 5, 1000000 },
	{ 10, 1000000 },
	{ 20, 1000000 },
	{ 50, 1000000 },
	{ 100, 1000000 },
	{ 200, 1000000 },
	{ 500, 1000000 },
	/* milliseconds */
	{ 1, 1000 },
	{ 2, 1000 },
	{ 5, 1000 },
	{ 10, 1000 },
	{ 20, 1000 },
	{ 50, 1000 },
	{ 100, 1000 },
	{ 200, 1000 },
	{ 500, 1000 },
	/* seconds */
	{ 1, 1 },
	{ 2, 1 },
	{ 5, 1 },
	{ 10, 1 },
	{ 20, 1 },
	{ 50, 1 },
	{ 100, 1 },
	{ 200, 1 },
	{ 500, 1 },
	{ 1000, 1 },
};

static const uint64_t vdivs[][2] = {
	/* microvolts */
	{ 500, 1000000 },
	/* millivolts */
	{ 1, 1000 },
	{ 2, 1000 },
	{ 5, 1000 },
	{ 10, 1000 },
	{ 20, 1000 },
	{ 50, 1000 },
	{ 100, 1000 },
	{ 200, 1000 },
	{ 500, 1000 },
	/* volts */
	{ 1, 1 },
	{ 2, 1 },
	{ 5, 1 },
	{ 10, 1 },
	{ 20, 1 },
	{ 50, 1 },
	{ 100, 1 },
};

static const char *trigger_sources_2_chans[] = {
	"CH1", "CH2",
	"EXT", "AC Line",
	"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7",
	"D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15",
};

static const char *trigger_sources_4_chans[] = {
	"CH1", "CH2", "CH3", "CH4",
	"EXT", "AC Line",
	"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7",
	"D8", "D9", "D10", "D11", "D12", "D13", "D14", "D15",
};

static const char *trigger_slopes[] = {
	"r", "f",
};

/* Trigger sweeps, and how the scope names them (V3 and later answer NORM, SING). */
static const char *trigger_sweeps[] = {
	"Auto", "Normal", "Single",
};

static const char *trigger_sweeps_scpi[] = {
	"AUTO", "NORMAL", "SINGLE",
};

static const char *coupling[] = {
	"AC", "DC", "GND",
};

static const uint64_t probe_factor[] = {
	1, 2, 5, 10, 20, 50, 100, 200, 500, 1000,
};

/* Do not change the order of entries */
static const char *data_sources[] = {
	"Live",
	"Memory",
	"Segmented",
};

static const struct rigol_ds_command std_cmd[] = {
	{ CMD_GET_HORIZ_TRIGGERPOS, ":TIM:OFFS?" },
	{ CMD_SET_HORIZ_TRIGGERPOS, ":TIM:OFFS %s" },
};

static const struct rigol_ds_command mso7000a_cmd[] = {
	{ CMD_GET_HORIZ_TRIGGERPOS, ":TIM:POS?" },
	{ CMD_SET_HORIZ_TRIGGERPOS, ":TIM:POS %s" },
};

enum vendor {
	RIGOL,
	AGILENT,
};

enum series {
	VS5000,
	DS1000,
	DS2000,
	DS2000A,
	DSO1000,
	DSO1000B,
	DS1000Z,
	DS4000,
	MSO5000,
	MSO7000A,
};

/* short name, full name */
static const struct rigol_ds_vendor supported_vendors[] = {
	[RIGOL] = {"Rigol", "Rigol Technologies"},
	[AGILENT] = {"Agilent", "Agilent Technologies"},
};

#define VENDOR(x) &supported_vendors[x]
/* vendor, series/name, protocol, data format, max timebase, min vdiv,
 * number of horizontal divs, live waveform samples, memory buffer samples */
static const struct rigol_ds_series supported_series[] = {
	[VS5000] = {VENDOR(RIGOL), "VS5000", PROTOCOL_V1, FORMAT_RAW,
		{50, 1}, {2, 1000}, 14, 2048, 524288},
	[DS1000] = {VENDOR(RIGOL), "DS1000", PROTOCOL_V2, FORMAT_IEEE488_2,
		{50, 1}, {2, 1000}, 12, 600, 1048576},
	[DS2000] = {VENDOR(RIGOL), "DS2000", PROTOCOL_V3, FORMAT_IEEE488_2,
		{500, 1}, {500, 1000000}, 14, 1400, 14000},
	[DS2000A] = {VENDOR(RIGOL), "DS2000A", PROTOCOL_V3, FORMAT_IEEE488_2,
		{1000, 1}, {500, 1000000}, 14, 1400, 14000},
	[DSO1000] = {VENDOR(AGILENT), "DSO1000", PROTOCOL_V3, FORMAT_IEEE488_2,
		{50, 1}, {2, 1000}, 12, 600, 20480},
	[DSO1000B] = {VENDOR(AGILENT), "DSO1000", PROTOCOL_V3, FORMAT_IEEE488_2,
		{50, 1}, {2, 1000}, 12, 600, 20480},
	[DS1000Z] = {VENDOR(RIGOL), "DS1000Z", PROTOCOL_V4, FORMAT_IEEE488_2,
		{50, 1}, {1, 1000}, 12, 1200, 12000000},
	[DS4000] = {VENDOR(RIGOL), "DS4000", PROTOCOL_V4, FORMAT_IEEE488_2,
		{1000, 1}, {1, 1000}, 14, 1400, 0},
	[MSO5000] = {VENDOR(RIGOL), "MSO5000", PROTOCOL_V5, FORMAT_IEEE488_2,
		{1000, 1}, {500, 1000000}, 10, 1000, 0},
	[MSO7000A] = {VENDOR(AGILENT), "MSO7000A", PROTOCOL_V4, FORMAT_IEEE488_2,
		{50, 1}, {2, 1000}, 10, 1000, 8000000},
};

#define SERIES(x) &supported_series[x]
/*
 * Use a macro to select the correct list of trigger sources and its length
 * based on the number of analog channels and presence of digital channels.
 */
#define CH_INFO(num, digital) \
	num, digital, trigger_sources_##num##_chans, \
	digital ? ARRAY_SIZE(trigger_sources_##num##_chans) : (num + 2)
/* series, model, min timebase, analog channels, digital */
static const struct rigol_ds_model supported_models[] = {
	{SERIES(VS5000), "VS5022", {20, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(VS5000), "VS5042", {10, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(VS5000), "VS5062", {5, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(VS5000), "VS5102", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(VS5000), "VS5202", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(VS5000), "VS5022D", {20, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(VS5000), "VS5042D", {10, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(VS5000), "VS5062D", {5, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(VS5000), "VS5102D", {2, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(VS5000), "VS5202D", {2, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(DS1000), "DS1052E", {5, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS1000), "DS1102E", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS1000), "DS1152E", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS1000), "DS1152E-EDU", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS1000), "DS1052D", {5, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(DS1000), "DS1102D", {2, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(DS1000), "DS1152D", {2, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(DS2000), "DS2072", {5, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS2000), "DS2102", {5, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS2000), "DS2202", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS2000), "DS2302", {1, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS2000A), "DS2072A", {5, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS2000A), "DS2102A", {5, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS2000A), "DS2202A", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS2000A), "DS2302A", {1, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS2000A), "MSO2072A", {5, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(DS2000A), "MSO2102A", {5, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(DS2000A), "MSO2202A", {2, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(DS2000A), "MSO2302A", {1, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(DSO1000), "DSO1002A", {5, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DSO1000), "DSO1004A", {5, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(DSO1000), "DSO1012A", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DSO1000), "DSO1014A", {2, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(DSO1000), "DSO1022A", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DSO1000), "DSO1024A", {2, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(DSO1000B), "DSO1052B", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DSO1000B), "DSO1072B", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DSO1000B), "DSO1102B", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DSO1000B), "DSO1152B", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS1000Z), "DS1054Z", {5, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(DS1000Z), "DS1074Z", {5, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(DS1000Z), "DS1104Z", {5, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(DS1000Z), "DS1074Z-S", {5, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(DS1000Z), "DS1104Z-S", {5, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(DS1000Z), "DS1074Z Plus", {5, 1000000000}, CH_INFO(4, true), std_cmd},
	{SERIES(DS1000Z), "DS1104Z Plus", {5, 1000000000}, CH_INFO(4, true), std_cmd},
	{SERIES(DS1000Z), "DS1102Z-E", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS1000Z), "DS1202Z-E", {2, 1000000000}, CH_INFO(2, false), std_cmd},
	{SERIES(DS1000Z), "MSO1074Z", {5, 1000000000}, CH_INFO(4, true), std_cmd},
	{SERIES(DS1000Z), "MSO1104Z", {5, 1000000000}, CH_INFO(4, true), std_cmd},
	{SERIES(DS1000Z), "MSO1074Z-S", {5, 1000000000}, CH_INFO(4, true), std_cmd},
	{SERIES(DS1000Z), "MSO1104Z-S", {5, 1000000000}, CH_INFO(4, true), std_cmd},
	{SERIES(DS4000), "DS4014", {1, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(DS4000), "DS4024", {1, 1000000000}, CH_INFO(4, false), std_cmd},
	{SERIES(MSO5000), "MSO5072", {1, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(MSO5000), "MSO5074", {1, 1000000000}, CH_INFO(4, true), std_cmd},
	{SERIES(MSO5000), "MSO5102", {1, 1000000000}, CH_INFO(2, true), std_cmd},
	{SERIES(MSO5000), "MSO5104", {1, 1000000000}, CH_INFO(4, true), std_cmd},
	{SERIES(MSO5000), "MSO5204", {1, 1000000000}, CH_INFO(4, true), std_cmd},
	{SERIES(MSO5000), "MSO5354", {1, 1000000000}, CH_INFO(4, true), std_cmd},
	/* TODO: Digital channels are not yet supported on MSO7000A. */
	{SERIES(MSO7000A), "MSO7034A", {2, 1000000000}, CH_INFO(4, false), mso7000a_cmd},
};

static struct sr_dev_driver rigol_ds_driver_info;

static int analog_frame_size(const struct sr_dev_inst *);

static void clear_helper(struct dev_context *devc)
{
	unsigned int i;

	g_free(devc->data);
	g_free(devc->buffer);
	for (i = 0; i < ARRAY_SIZE(devc->coupling); i++)
		g_free(devc->coupling[i]);
	g_free(devc->trigger_source);
	g_free(devc->trigger_slope);
	g_free(devc->trigger_sweep);
	g_free(devc->trigger_type);
	g_slist_free_full(devc->v1_pending_cmds, g_free);
	g_mutex_clear(&devc->v1_cmd_lock);
	g_free(devc->saved_sweep);
	g_free(devc->analog_groups);
}

static int dev_clear(const struct sr_dev_driver *di)
{
	return std_dev_clear_with_callback(di, (std_dev_clear_callback)clear_helper);
}

static struct sr_dev_inst *probe_device(struct sr_scpi_dev_inst *scpi)
{
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_scpi_hw_info *hw_info;
	struct sr_channel *ch;
	long n[3];
	unsigned int i;
	const struct rigol_ds_model *model = NULL;
	gchar *channel_name, **version;

	if (sr_scpi_get_hw_id(scpi, &hw_info) != SR_OK) {
		sr_info("Couldn't get IDN response, retrying.");
		sr_scpi_close(scpi);
		sr_scpi_open(scpi);
		if (sr_scpi_get_hw_id(scpi, &hw_info) != SR_OK) {
			sr_info("Couldn't get IDN response.");
			return NULL;
		}
	}

	for (i = 0; i < ARRAY_SIZE(supported_models); i++) {
		if (!g_ascii_strcasecmp(hw_info->manufacturer,
					supported_models[i].series->vendor->full_name) &&
				!strcmp(hw_info->model, supported_models[i].name)) {
			model = &supported_models[i];
			break;
		}
	}

	if (!model) {
		sr_scpi_hw_info_free(hw_info);
		return NULL;
	}

	sdi = g_malloc0(sizeof(struct sr_dev_inst));
	sdi->vendor = g_strdup(model->series->vendor->name);
	sdi->model = g_strdup(model->name);
	sdi->version = g_strdup(hw_info->firmware_version);
	sdi->conn = scpi;
	sdi->driver = &rigol_ds_driver_info;
	sdi->inst_type = SR_INST_SCPI;
	sdi->serial_num = g_strdup(hw_info->serial_number);
	devc = g_malloc0(sizeof(struct dev_context));
	g_mutex_init(&devc->v1_cmd_lock);
	devc->limit_frames = 0;
	devc->model = model;
	devc->format = model->series->format;

	if (model->series == SERIES(VS5000)) {
		devc->v1_fw_patch = rigol_ds_v1_fw_patch(hw_info->firmware_version);
		if (devc->v1_fw_patch)
			sr_info("VS5000 firmware patch level %d (%s): memory over LAN available.",
				devc->v1_fw_patch, hw_info->firmware_version);
	}

	/* DS1000 models with firmware before 0.2.4 used the old data format. */
	if (model->series == SERIES(DS1000)) {
		version = g_strsplit(hw_info->firmware_version, ".", 0);
		do {
			if (!version[0] || !version[1] || !version[2])
				break;
			if (version[0][0] == 0 || version[1][0] == 0 || version[2][0] == 0)
				break;
			for (i = 0; i < 3; i++) {
				if (sr_atol(version[i], &n[i]) != SR_OK)
					break;
			}
			if (i != 3)
				break;
			scpi->firmware_version = n[0] * 100 + n[1] * 10 + n[2];
			if (scpi->firmware_version < 24) {
				sr_dbg("Found DS1000 firmware < 0.2.4, using raw data format.");
				devc->format = FORMAT_RAW;
			}
			break;
		} while (0);
		g_strfreev(version);
	}

	sr_scpi_hw_info_free(hw_info);

	devc->analog_groups = g_malloc0(sizeof(struct sr_channel_group*) *
					model->analog_channels);

	for (i = 0; i < model->analog_channels; i++) {
		channel_name = g_strdup_printf("CH%d", i + 1);
		ch = sr_channel_new(sdi, i, SR_CHANNEL_ANALOG, TRUE, channel_name);

		devc->analog_groups[i] = sr_channel_group_new(sdi,
			channel_name, NULL);
		devc->analog_groups[i]->channels = g_slist_append(NULL, ch);
	}

	if (devc->model->has_digital) {
		devc->digital_group = sr_channel_group_new(sdi, "LA", NULL);
		for (i = 0; i < ARRAY_SIZE(devc->digital_channels); i++) {
			channel_name = g_strdup_printf("D%d", i);
			ch = sr_channel_new(sdi, i, SR_CHANNEL_LOGIC, TRUE, channel_name);
			g_free(channel_name);
			devc->digital_group->channels = g_slist_append(
					devc->digital_group->channels, ch);
		}
	}

	for (i = 0; i < ARRAY_SIZE(timebases); i++) {
		if (!memcmp(&devc->model->min_timebase, &timebases[i], sizeof(uint64_t[2])))
			devc->timebases = &timebases[i];
		if (!memcmp(&devc->model->series->max_timebase, &timebases[i], sizeof(uint64_t[2])))
			devc->num_timebases = &timebases[i] - devc->timebases + 1;
	}

	for (i = 0; i < ARRAY_SIZE(vdivs); i++) {
		if (!memcmp(&devc->model->series->min_vdiv,
					&vdivs[i], sizeof(uint64_t[2]))) {
			devc->vdivs = &vdivs[i];
			devc->num_vdivs = ARRAY_SIZE(vdivs) - i;
		}
	}

	devc->buffer = g_malloc(ACQ_BUFFER_SIZE);
	devc->data = g_malloc((ACQ_BUFFER_SIZE + V1_HEAD_SAMPLES) * sizeof(float));

	devc->data_source = DATA_SOURCE_LIVE;

	sdi->priv = devc;

	return sdi;
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	return sr_scpi_scan(di->context, options, probe_device);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	int ret;
	struct sr_scpi_dev_inst *scpi = sdi->conn;

	if ((ret = sr_scpi_open(scpi)) < 0) {
		sr_err("Failed to open SCPI device: %s.", sr_strerror(ret));
		return SR_ERR;
	}

	if ((ret = rigol_ds_get_dev_cfg(sdi)) < 0) {
		/*
		 * Retry once on a new connection. Seen on the VS5000 over LAN
		 * right after a client died in the middle of a waveform
		 * transfer: the first query got no answer within the read
		 * timeout. Asking again on the same connection could read the
		 * late answer of the first query as the answer to the next.
		 */
		sr_warn("Failed to get device config (%s), retrying on a new "
			"connection.", sr_strerror(ret));
		sr_scpi_close(scpi);
		g_usleep(2 * G_USEC_PER_SEC);
		if ((ret = sr_scpi_open(scpi)) < 0) {
			sr_err("Failed to reopen SCPI device: %s.", sr_strerror(ret));
			return SR_ERR;
		}
		if ((ret = rigol_ds_get_dev_cfg(sdi)) < 0) {
			sr_err("Failed to get device config: %s.", sr_strerror(ret));
			sr_scpi_close(scpi);
			return SR_ERR;
		}
	}

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	struct sr_scpi_dev_inst *scpi;
	struct dev_context *devc;

	scpi = sdi->conn;
	devc = sdi->priv;

	if (!scpi)
		return SR_ERR_BUG;

	if (devc->model->series->protocol == PROTOCOL_V2)
		rigol_ds_config_set(sdi, ":KEY:LOCK DISABLE");

	return sr_scpi_close(scpi);
}

static int analog_frame_size(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_channel *ch;
	int analog_channels = 0;
	GSList *l;

	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch->type == SR_CHANNEL_ANALOG && ch->enabled)
			analog_channels++;
	}

	if (analog_channels == 0)
		return 0;

	switch (devc->data_source) {
	case DATA_SOURCE_LIVE:
		return devc->model->series->live_samples;
	case DATA_SOURCE_MEMORY:
	case DATA_SOURCE_SEGMENTED:
		/* The VS5000 keeps 512K samples for each channel. */
		if (devc->model->series->protocol == PROTOCOL_V1)
			return devc->model->series->buffer_samples;
		return devc->model->series->buffer_samples / analog_channels;
	default:
		return 0;
	}
}

static int digital_frame_size(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	switch (devc->data_source) {
	case DATA_SOURCE_LIVE:
		return devc->model->series->live_samples * 2;
	case DATA_SOURCE_MEMORY:
	case DATA_SOURCE_SEGMENTED:
		return devc->model->series->buffer_samples * 2;
	default:
		return 0;
	}
}

/*
 * VS5000 volts/div at the probe tip: 2 mV to 10 V times the probe factor
 * (the scope keeps the setting within that, tested with 1x, 10x, 100x).
 * Fills vd with the 1-2-5 steps in that range, returns their number.
 */
#define V1_MAX_VDIVS 32
static int v1_vdivs(const struct dev_context *devc, int ch,
		uint64_t vd[][2])
{
	static const uint64_t mant[] = { 1, 2, 5 };
	double probe, v;
	uint64_t num, den;
	int e, m, n;

	probe = devc->attenuation[ch] > 0 ? devc->attenuation[ch] : 1;
	n = 0;
	for (e = -3; e <= 3 && n < V1_MAX_VDIVS; e++) {
		for (m = 0; m < 3 && n < V1_MAX_VDIVS; m++) {
			/* Millivolts below 1 V, so that they print as mV. */
			if (e < 0) {
				num = mant[m] * (uint64_t)pow(10, e + 3);
				den = 1000;
			} else {
				num = mant[m] * (uint64_t)pow(10, e);
				den = 1;
			}
			v = (double)num / den;
			if (v < 0.002 * probe * 0.999 || v > 10.0 * probe * 1.001)
				continue;
			vd[n][0] = num;
			vd[n][1] = den;
			n++;
		}
	}

	return n;
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	struct sr_channel *ch;
	const char *tmp_str;
	int analog_channel = -1;
	float smallest_diff = INFINITY;
	int idx = -1;
	unsigned i;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	/* If a channel group is specified, it must be a valid one. */
	if (cg && !g_slist_find(sdi->channel_groups, cg)) {
		sr_err("Invalid channel group specified.");
		return SR_ERR;
	}

	if (cg) {
		ch = g_slist_nth_data(cg->channels, 0);
		if (!ch)
			return SR_ERR;
		if (ch->type == SR_CHANNEL_ANALOG) {
			if (ch->name[2] < '1' || ch->name[2] > '4')
				return SR_ERR;
			analog_channel = ch->name[2] - '1';
		}
	}

	switch (key) {
	case SR_CONF_NUM_HDIV:
		*data = g_variant_new_int32(devc->model->series->num_horizontal_divs);
		break;
	case SR_CONF_NUM_VDIV:
		*data = g_variant_new_int32(devc->num_vdivs);
		break;
	case SR_CONF_DATA_SOURCE:
		if (devc->data_source == DATA_SOURCE_LIVE)
			*data = g_variant_new_string("Live");
		else if (devc->data_source == DATA_SOURCE_MEMORY)
			*data = g_variant_new_string("Memory");
		else
			*data = g_variant_new_string("Segmented");
		break;
	case SR_CONF_LIMIT_FRAMES:
		*data = g_variant_new_uint64(devc->limit_frames);
		break;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->sample_rate);
		break;
	case SR_CONF_TRIGGER_SOURCE:
		/*
		 * Most models answer CHAN1 and ACL, the VS5000 answers the
		 * long forms CHANNEL1 and ACLINE: map both to the names in
		 * the list.
		 */
		tmp_str = devc->trigger_source;
		if (!strncmp(tmp_str, "ACL", 3))
			tmp_str = "AC Line";
		else if (!strncmp(tmp_str, "CHAN", 4)) {
			const char *num = tmp_str + strcspn(tmp_str, "0123456789");
			if (num[0] >= '1' && num[0] <= '4' && !num[1]) {
				static const char *const chn[] = { "CH1", "CH2", "CH3", "CH4" };
				tmp_str = chn[num[0] - '1'];
			}
		}
		*data = g_variant_new_string(tmp_str);
		break;
	case SR_CONF_BUFFERSIZE:
		if (devc->model->series->protocol == PROTOCOL_V1)
			*data = g_variant_new_uint64(
				v1_mem_depths[devc->v1_mem_normal ? 0 : 1]);
		else if (is_ds1000z(devc))
			*data = g_variant_new_uint64(devc->mem_level < 0 ?
				devc->mem_depth : ds1000z_level_depth(devc->mem_level,
					ds1000z_active_channels(sdi)));
		else
			return SR_ERR_NA;
		break;
	case SR_CONF_TRIGGER_SWEEP:
		if (!devc->trigger_sweep)
			return SR_ERR_NA;
		for (i = 0; i < ARRAY_SIZE(trigger_sweeps); i++)
			if (!g_ascii_strncasecmp(devc->trigger_sweep,
					trigger_sweeps_scpi[i], 4))
				break;
		if (i == ARRAY_SIZE(trigger_sweeps)) {
			sr_dbg("Unknown trigger sweep: '%s'.", devc->trigger_sweep);
			return SR_ERR_NA;
		}
		*data = g_variant_new_string(trigger_sweeps[i]);
		break;
	case SR_CONF_TRIGGER_SLOPE:
		if (!strncmp(devc->trigger_slope, "POS", 3)) {
			tmp_str = "r";
		} else if (!strncmp(devc->trigger_slope, "NEG", 3)) {
			tmp_str = "f";
		} else {
			sr_dbg("Unknown trigger slope: '%s'.", devc->trigger_slope);
			return SR_ERR_NA;
		}
		*data = g_variant_new_string(tmp_str);
		break;
	case SR_CONF_TRIGGER_LEVEL:
		*data = g_variant_new_double(devc->trigger_level);
		break;
	case SR_CONF_TIMEBASE:
		for (i = 0; i < devc->num_timebases; i++) {
			float tb = (float)devc->timebases[i][0] / devc->timebases[i][1];
			float diff = fabs(devc->timebase - tb);
			if (diff < smallest_diff) {
				smallest_diff = diff;
				idx = i;
			}
		}
		if (idx < 0) {
			sr_dbg("Negative timebase index: %d.", idx);
			return SR_ERR_NA;
		}
		*data = g_variant_new("(tt)", devc->timebases[idx][0],
		                              devc->timebases[idx][1]);
		break;
	case SR_CONF_VDIV:
		if (analog_channel < 0) {
			sr_dbg("Negative analog channel: %d.", analog_channel);
			return SR_ERR_NA;
		}
		if (devc->model->series->protocol == PROTOCOL_V1) {
			uint64_t vd[V1_MAX_VDIVS][2];
			unsigned int n = v1_vdivs(devc, analog_channel, vd);
			for (i = 0; i < n; i++) {
				float diff = fabs(devc->vdiv[analog_channel] -
					(float)vd[i][0] / vd[i][1]);
				if (diff < smallest_diff) {
					smallest_diff = diff;
					idx = i;
				}
			}
			if (idx < 0)
				return SR_ERR_NA;
			*data = g_variant_new("(tt)", vd[idx][0], vd[idx][1]);
			break;
		}
		for (i = 0; i < ARRAY_SIZE(vdivs); i++) {
			float vdiv = (float)vdivs[i][0] / vdivs[i][1];
			float diff = fabs(devc->vdiv[analog_channel] - vdiv);
			if (diff < smallest_diff) {
				smallest_diff = diff;
				idx = i;
			}
		}
		if (idx < 0) {
			sr_dbg("Negative vdiv index: %d.", idx);
			return SR_ERR_NA;
		}
		*data = g_variant_new("(tt)", vdivs[idx][0], vdivs[idx][1]);
		break;
	case SR_CONF_COUPLING:
		if (analog_channel < 0) {
			sr_dbg("Negative analog channel: %d.", analog_channel);
			return SR_ERR_NA;
		}
		*data = g_variant_new_string(devc->coupling[analog_channel]);
		break;
	case SR_CONF_PROBE_FACTOR:
		if (analog_channel < 0) {
			sr_dbg("Negative analog channel: %d.", analog_channel);
			return SR_ERR_NA;
		}
		*data = g_variant_new_uint64(devc->attenuation[analog_channel]);
		break;
	case SR_CONF_RANGE:
		/* Input voltages that do not clip, as text. */
		if (analog_channel < 0)
			return SR_ERR_NA;
		if (devc->model->series->protocol == PROTOCOL_V1)
			*data = g_variant_new_take_string(
				rigol_ds_v1_range_string(devc, analog_channel));
		else if (is_ds1000z(devc))
			*data = g_variant_new_take_string(
				ds1000z_range_string(devc, analog_channel));
		else
			return SR_ERR_NA;
		break;
	case SR_CONF_OFFSET:
		/*
		 * Vertical offset in volts at the probe tip, as the scope
		 * reports it. It moves the trace on the screen and so the
		 * input range: the ADC covers the 8 divisions around
		 * -offset. Sample values are input voltages, they do not
		 * include the offset.
		 */
		if (analog_channel < 0) {
			sr_dbg("Negative analog channel: %d.", analog_channel);
			return SR_ERR_NA;
		}
		*data = g_variant_new_double(devc->vert_offset[analog_channel]);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

/*
 * Re-read gain and offset after a vertical change. Not while another thread
 * acquires from a VS5000: its commands are queued (rigol_ds_config_set) and
 * a query would interleave with the waveform reply. The values are read
 * again when the next acquisition starts.
 */
static void vertical_readback(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	if (devc->model->series->protocol == PROTOCOL_V1 && devc->v1_acquiring &&
			g_thread_self() != devc->v1_session_thread)
		return;
	rigol_ds_get_dev_cfg_vertical(sdi);
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	uint64_t p;
	double t_dbl;
	int ret, idx, i;
	const char *tmp_str;
	char buffer[16];

	devc = sdi->priv;

	/* If a channel group is specified, it must be a valid one. */
	if (cg && !g_slist_find(sdi->channel_groups, cg)) {
		sr_err("Invalid channel group specified.");
		return SR_ERR;
	}

	switch (key) {
	case SR_CONF_LIMIT_FRAMES:
		devc->limit_frames = g_variant_get_uint64(data);
		break;
	case SR_CONF_TRIGGER_SLOPE:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(trigger_slopes))) < 0)
			return SR_ERR_ARG;
		g_free(devc->trigger_slope);
		devc->trigger_slope = g_strdup((trigger_slopes[idx][0] == 'r') ? "POS" : "NEG");
		return rigol_ds_config_set(sdi, ":TRIG:EDGE:SLOP %s", devc->trigger_slope);
	case SR_CONF_HORIZ_TRIGGERPOS:
		t_dbl = g_variant_get_double(data);
		if (t_dbl < 0.0 || t_dbl > 1.0) {
			sr_err("Invalid horiz. trigger position: %g.", t_dbl);
			return SR_ERR;
		}
		devc->horiz_triggerpos = t_dbl;
		/* We have the trigger offset as a percentage of the frame, but
		 * need to express this in seconds. The VS5000 frame holds
		 * 20.48 or 14 divisions, other models fill the screen. */
		t_dbl = -(devc->horiz_triggerpos - 0.5) * devc->timebase *
			(devc->model->series->protocol == PROTOCOL_V1 ?
				rigol_ds_v1_record_divs(devc) :
				devc->model->series->num_horizontal_divs);
		g_ascii_formatd(buffer, sizeof(buffer), "%.6f", t_dbl);
		return rigol_ds_config_set(sdi,
			devc->model->cmds[CMD_SET_HORIZ_TRIGGERPOS].str, buffer);
	case SR_CONF_TRIGGER_LEVEL:
		t_dbl = g_variant_get_double(data);
		g_ascii_formatd(buffer, sizeof(buffer), "%.3f", t_dbl);
		ret = rigol_ds_config_set(sdi, ":TRIG:EDGE:LEV %s", buffer);
		if (ret == SR_OK)
			devc->trigger_level = t_dbl;
		return ret;
	case SR_CONF_BUFFERSIZE: {
		uint64_t depth = g_variant_get_uint64(data);
		int level, n;
		if (devc->model->series->protocol == PROTOCOL_V1) {
			if ((idx = std_u64_idx(data, ARRAY_AND_SIZE(v1_mem_depths))) < 0)
				return SR_ERR_ARG;
			/* Sent when a memory capture starts. */
			devc->v1_mem_normal = idx == 0;
			return SR_OK;
		}
		if (!is_ds1000z(devc))
			return SR_ERR_NA;
		/*
		 * Remember the level, it is applied when an acquisition
		 * starts: the channels (and so the possible depths) may still
		 * change until then.
		 */
		n = ds1000z_active_channels(sdi);
		for (level = 0; level <= (int)ARRAY_SIZE(ds1000z_mem_depths); level++)
			if (ds1000z_level_depth(level, n) == depth)
				break;
		if (level > (int)ARRAY_SIZE(ds1000z_mem_depths))
			return SR_ERR_ARG;
		devc->mem_level = level;
		return SR_OK;
	}
	case SR_CONF_TRIGGER_SWEEP:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(trigger_sweeps))) < 0)
			return SR_ERR_ARG;
		if (devc->model->series->protocol >= PROTOCOL_V3) {
			ret = rigol_ds_config_set(sdi, ":TRIG:SWE %s",
				trigger_sweeps_scpi[idx]);
			if (ret != SR_OK)
				return ret;
			g_free(devc->trigger_sweep);
			devc->trigger_sweep = g_strdup(trigger_sweeps_scpi[idx]);
			/* Live capture with SINGLE takes a new shot per frame. */
			devc->live_single = idx == 2;
			return SR_OK;
		}
		if (devc->model->series->protocol != PROTOCOL_V1 ||
				!devc->trigger_type)
			return SR_ERR_NA;
		/*
		 * Only commands, no queries: this may run in another thread
		 * while a waveform reply is being read, and over LAN a query
		 * would read reply bytes as its answer.
		 */
		ret = rigol_ds_config_set(sdi, ":TRIG:%s:SWE %s",
			devc->trigger_type, trigger_sweeps_scpi[idx]);
		if (ret != SR_OK)
			return ret;
		g_free(devc->trigger_sweep);
		devc->trigger_sweep = g_strdup(trigger_sweeps_scpi[idx]);
		if (devc->v1_acquiring && devc->data_source == DATA_SOURCE_LIVE) {
			/* A running live capture re-arms each SINGLE shot. */
			devc->v1_single = idx == 2;
			/* A finished SINGLE shot left the scope stopped. */
			if (!devc->v1_single)
				return rigol_ds_config_set(sdi, ":RUN");
		}
		return SR_OK;
	case SR_CONF_TIMEBASE:
		if ((idx = std_u64_tuple_idx(data, devc->timebases, devc->num_timebases)) < 0)
			return SR_ERR_ARG;
		devc->timebase = (float)devc->timebases[idx][0] / devc->timebases[idx][1];
		g_ascii_formatd(buffer, sizeof(buffer), "%.9f",
		                devc->timebase);
		ret = rigol_ds_config_set(sdi, ":TIM:SCAL %s", buffer);
		/*
		 * Frontends such as PulseView read the sample rate before
		 * they start an acquisition, keep it in step.
		 */
		if (ret == SR_OK && devc->model->series->protocol == PROTOCOL_V1 &&
				devc->data_source == DATA_SOURCE_LIVE)
			devc->sample_rate = rigol_ds_v1_live_rate(devc);
		return ret;
	case SR_CONF_TRIGGER_SOURCE:
		if ((idx = std_str_idx(data, devc->model->trigger_sources, devc->model->num_trigger_sources)) < 0)
			return SR_ERR_ARG;
		g_free(devc->trigger_source);
		devc->trigger_source = g_strdup(devc->model->trigger_sources[idx]);
		if (!strcmp(devc->trigger_source, "AC Line"))
			tmp_str = "ACL";
		else if (!strcmp(devc->trigger_source, "CH1"))
			tmp_str = "CHAN1";
		else if (!strcmp(devc->trigger_source, "CH2"))
			tmp_str = "CHAN2";
		else if (!strcmp(devc->trigger_source, "CH3"))
			tmp_str = "CHAN3";
		else if (!strcmp(devc->trigger_source, "CH4"))
			tmp_str = "CHAN4";
		else
			tmp_str = (char *)devc->trigger_source;
		return rigol_ds_config_set(sdi, ":TRIG:EDGE:SOUR %s", tmp_str);
	case SR_CONF_VDIV:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		if ((i = std_cg_idx(cg, devc->analog_groups, devc->model->analog_channels)) < 0)
			return SR_ERR_ARG;
		if (devc->model->series->protocol == PROTOCOL_V1) {
			uint64_t vd[V1_MAX_VDIVS][2];
			int n = v1_vdivs(devc, i, vd);
			if ((idx = std_u64_tuple_idx(data, vd, n)) < 0)
				return SR_ERR_ARG;
			devc->vdiv[i] = (float)vd[idx][0] / vd[idx][1];
		} else {
			if ((idx = std_u64_tuple_idx(data, ARRAY_AND_SIZE(vdivs))) < 0)
				return SR_ERR_ARG;
			devc->vdiv[i] = (float)vdivs[idx][0] / vdivs[idx][1];
		}
		g_ascii_formatd(buffer, sizeof(buffer), "%.3f", devc->vdiv[i]);
		ret = rigol_ds_config_set(sdi, ":CHAN%d:SCAL %s", i + 1, buffer);
		/* The scope may limit the offset to the new range. */
		if (ret == SR_OK)
			vertical_readback(sdi);
		return ret;
	case SR_CONF_COUPLING:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		if ((i = std_cg_idx(cg, devc->analog_groups, devc->model->analog_channels)) < 0)
			return SR_ERR_ARG;
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(coupling))) < 0)
			return SR_ERR_ARG;
		g_free(devc->coupling[i]);
		devc->coupling[i] = g_strdup(coupling[idx]);
		return rigol_ds_config_set(sdi, ":CHAN%d:COUP %s", i + 1, devc->coupling[i]);
	case SR_CONF_PROBE_FACTOR:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		if ((i = std_cg_idx(cg, devc->analog_groups, devc->model->analog_channels)) < 0)
			return SR_ERR_ARG;
		if ((idx = std_u64_idx(data, ARRAY_AND_SIZE(probe_factor))) < 0)
			return SR_ERR_ARG;
		p = g_variant_get_uint64(data);
		devc->attenuation[i] = probe_factor[idx];
		ret = rigol_ds_config_set(sdi, ":CHAN%d:PROB %"PRIu64, i + 1, p);
		if (ret == SR_OK)
			vertical_readback(sdi);
		return ret;
	case SR_CONF_OFFSET:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		if ((i = std_cg_idx(cg, devc->analog_groups, devc->model->analog_channels)) < 0)
			return SR_ERR_ARG;
		t_dbl = g_variant_get_double(data);
		if (!isfinite(t_dbl))
			return SR_ERR_ARG;
		g_ascii_formatd(buffer, sizeof(buffer), "%.4f", t_dbl);
		ret = rigol_ds_config_set(sdi, ":CHAN%d:OFFS %s", i + 1, buffer);
		if (ret != SR_OK)
			return ret;
		/* Read back, the scope limits the offset per V/div. */
		devc->vert_offset[i] = t_dbl;
		vertical_readback(sdi);
		return SR_OK;
	case SR_CONF_DATA_SOURCE:
		tmp_str = g_variant_get_string(data, NULL);
		if (!strcmp(tmp_str, "Live"))
			devc->data_source = DATA_SOURCE_LIVE;
		else if ((devc->model->series->protocol >= PROTOCOL_V2 ||
				rigol_ds_v1_memory_supported(sdi))
			&& !strcmp(tmp_str, "Memory"))
			devc->data_source = DATA_SOURCE_MEMORY;
		else if (devc->model->series->protocol >= PROTOCOL_V3
			 && !strcmp(tmp_str, "Segmented"))
			devc->data_source = DATA_SOURCE_SEGMENTED;
		else {
			sr_err("Unknown data source: '%s'.", tmp_str);
			return SR_ERR;
		}
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;

	devc = (sdi) ? sdi->priv : NULL;

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		if (!cg && is_ds1000z(devc))
			return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts_ds1000z);
		if (!cg && devc && devc->model->series->protocol == PROTOCOL_V1)
			return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts_v1);
		if (!cg && devc && devc->model->series->protocol >= PROTOCOL_V3)
			return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts_sweep);
		if (!cg)
			return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
		if (!devc)
			return SR_ERR_ARG;
		if (cg == devc->digital_group) {
			*data = std_gvar_array_u32(NULL, 0);
			return SR_OK;
		} else {
			if (std_cg_idx(cg, devc->analog_groups, devc->model->analog_channels) < 0)
				return SR_ERR_ARG;
			if (devc->model->series->protocol == PROTOCOL_V1)
				*data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_cg_analog_v1));
			else if (is_ds1000z(devc))
				*data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_cg_analog_ds1000z));
			else
				*data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_cg_analog));
			return SR_OK;
		}
		break;
	case SR_CONF_COUPLING:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		*data = g_variant_new_strv(ARRAY_AND_SIZE(coupling));
		break;
	case SR_CONF_PROBE_FACTOR:
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		*data = std_gvar_array_u64(ARRAY_AND_SIZE(probe_factor));
		break;
	case SR_CONF_VDIV:
		if (!devc)
			/* Can't know this until we have the exact model. */
			return SR_ERR_ARG;
		if (!cg)
			return SR_ERR_CHANNEL_GROUP;
		if (devc->model->series->protocol == PROTOCOL_V1) {
			uint64_t vd[V1_MAX_VDIVS][2];
			int ch;
			if ((ch = std_cg_idx(cg, devc->analog_groups,
					devc->model->analog_channels)) < 0)
				return SR_ERR_ARG;
			*data = std_gvar_tuple_array(vd, v1_vdivs(devc, ch, vd));
			break;
		}
		*data = std_gvar_tuple_array(devc->vdivs, devc->num_vdivs);
		break;
	case SR_CONF_TIMEBASE:
		if (!devc)
			/* Can't know this until we have the exact model. */
			return SR_ERR_ARG;
		if (devc->num_timebases <= 0)
			return SR_ERR_NA;
		*data = std_gvar_tuple_array(devc->timebases, devc->num_timebases);
		break;
	case SR_CONF_TRIGGER_SOURCE:
		if (!devc)
			/* Can't know this until we have the exact model. */
			return SR_ERR_ARG;
		*data = g_variant_new_strv(devc->model->trigger_sources, devc->model->num_trigger_sources);
		break;
	case SR_CONF_BUFFERSIZE: {
		if (devc && devc->model->series->protocol == PROTOCOL_V1) {
			*data = std_gvar_array_u64(ARRAY_AND_SIZE(v1_mem_depths));
			break;
		}
		if (!is_ds1000z(devc))
			return SR_ERR_NA;
		uint64_t depths[1 + ARRAY_SIZE(ds1000z_mem_depths)];
		int n;
		unsigned int k;
		n = ds1000z_active_channels(sdi);
		for (k = 0; k <= ARRAY_SIZE(ds1000z_mem_depths); k++)
			depths[k] = ds1000z_level_depth(k, n);
		*data = std_gvar_array_u64(depths, ARRAY_SIZE(depths));
		break;
	}
	case SR_CONF_TRIGGER_SWEEP:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(trigger_sweeps));
		break;
	case SR_CONF_TRIGGER_SLOPE:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(trigger_slopes));
		break;
	case SR_CONF_DATA_SOURCE:
		if (!devc)
			/* Can't know this until we have the exact model. */
			return SR_ERR_ARG;
		switch (devc->model->series->protocol) {
		case PROTOCOL_V1:
			/* Memory over USB, or over LAN on patched firmware. */
			*data = g_variant_new_strv(data_sources, ARRAY_SIZE(data_sources) -
				(rigol_ds_v1_memory_supported(sdi) ? 1 : 2));
			break;
		case PROTOCOL_V2:
			*data = g_variant_new_strv(data_sources, ARRAY_SIZE(data_sources) - 1);
			break;
		default:
			*data = g_variant_new_strv(ARRAY_AND_SIZE(data_sources));
			break;
		}
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct sr_scpi_dev_inst *scpi;
	struct dev_context *devc;
	struct sr_channel *ch;
	gboolean some_digital;
	GSList *l;
	char *cmd, *v1_mode, *v1_sweep;
	int protocol;
	int ret;
	float sample_rate = 0;

	scpi = sdi->conn;
	devc = sdi->priv;
	protocol = devc->model->series->protocol;

	devc->num_frames = 0;
	devc->num_frames_segmented = 0;

	some_digital = FALSE;
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		sr_dbg("handling channel %s", ch->name);
		if (ch->type == SR_CHANNEL_ANALOG) {
			if (ch->enabled)
				devc->enabled_channels = g_slist_append(
						devc->enabled_channels, ch);
			if (ch->enabled != devc->analog_channels[ch->index]) {
				/* Enabled channel is currently disabled, or vice versa. */
				if (rigol_ds_config_set(sdi, ":CHAN%d:DISP %s", ch->index + 1,
						ch->enabled ? "ON" : "OFF") != SR_OK)
					return SR_ERR;
				devc->analog_channels[ch->index] = ch->enabled;
			}
		} else if (ch->type == SR_CHANNEL_LOGIC) {
			/* Only one list entry for older protocols. All channels are
			 * retrieved together when this entry is processed. */
			if (ch->enabled && (
						protocol > PROTOCOL_V3 ||
						!some_digital))
				devc->enabled_channels = g_slist_append(
						devc->enabled_channels, ch);
			if (ch->enabled) {
				some_digital = TRUE;
				/* Turn on LA module if currently off. */
				if (!devc->la_enabled) {
					if (rigol_ds_config_set(sdi, protocol >= PROTOCOL_V3 ?
								":LA:STAT ON" : ":LA:DISP ON") != SR_OK)
						return SR_ERR;
					devc->la_enabled = TRUE;
				}
			}
			if (ch->enabled != devc->digital_channels[ch->index]) {
				/* Enabled channel is currently disabled, or vice versa. */
				if (protocol >= PROTOCOL_V5)
					cmd = ":LA:DISP D%d,%s";
				else if (protocol >= PROTOCOL_V3)
					cmd = ":LA:DIG%d:DISP %s";
				else
					cmd = ":DIG%d:TURN %s";

				if (rigol_ds_config_set(sdi, cmd, ch->index,
						ch->enabled ? "ON" : "OFF") != SR_OK)
					return SR_ERR;
				devc->digital_channels[ch->index] = ch->enabled;
			}
		}
	}

	if (!devc->enabled_channels)
		return SR_ERR;

	/* Turn off LA module if on and no digital channels selected. */
	if (devc->la_enabled && !some_digital)
		if (rigol_ds_config_set(sdi,
				devc->model->series->protocol >= PROTOCOL_V3 ?
					":LA:STAT OFF" : ":LA:DISP OFF") != SR_OK)
			return SR_ERR;

	/* Set memory mode. */
	if (devc->data_source == DATA_SOURCE_SEGMENTED) {
		switch (protocol) {
		case PROTOCOL_V1:
		case PROTOCOL_V2:
			/* V1 and V2 do not have segmented data */
			sr_err("Data source 'Segmented' not supported on this model");
			break;
		case PROTOCOL_V3:
		case PROTOCOL_V4:
		{
			int frames = 0;
			if (sr_scpi_get_int(sdi->conn,
						protocol == PROTOCOL_V4 ? "FUNC:WREP:FEND?" :
						"FUNC:WREP:FMAX?", &frames) != SR_OK)
				return SR_ERR;
			if (frames <= 0) {
				sr_err("No segmented data available");
				return SR_ERR;
			}
			devc->num_frames_segmented = frames;
			break;
		}
		case PROTOCOL_V5:
			/* The frame limit has to be read on the fly, just set up
			 * reading of the first frame */
			if (rigol_ds_config_set(sdi, "REC:CURR 1") != SR_OK)
				return SR_ERR;
			break;
		default:
			sr_err("Data source 'Segmented' not yet supported");
			return SR_ERR;
		}
	}

	devc->analog_frame_size = analog_frame_size(sdi);
	devc->digital_frame_size = digital_frame_size(sdi);

	if (protocol == PROTOCOL_V1 && devc->data_source == DATA_SOURCE_MEMORY) {
		if (some_digital) {
			sr_err("Memory data source: digital channels not supported.");
			return SR_ERR;
		}
		/* Take single shots, restore the sweep when done. */
		g_free(devc->v1_trigger_mode);
		g_free(devc->v1_sweep);
		devc->v1_trigger_mode = devc->v1_sweep = NULL;
		if (sr_scpi_get_string(sdi->conn, ":TRIG:MODE?",
				&devc->v1_trigger_mode) != SR_OK)
			return SR_ERR;
		cmd = g_strdup_printf(":TRIG:%s:SWE?", devc->v1_trigger_mode);
		ret = sr_scpi_get_string(sdi->conn, cmd, &devc->v1_sweep);
		g_free(cmd);
		if (ret != SR_OK)
			return SR_ERR;
		/*
		 * Stop a running acquisition first. A first shot armed while
		 * the scope was running continuously sometimes held a 128K
		 * block of one channel from another acquisition (seam at
		 * sample 262144 or 393216). Shots armed from a stopped scope
		 * never did.
		 */
		if (rigol_ds_v1_stop(sdi) != SR_OK)
			return SR_ERR;
		if (rigol_ds_config_set(sdi, ":TRIG:%s:SWE SINGLE",
				devc->v1_trigger_mode) != SR_OK)
			return SR_ERR;
		/*
		 * The chosen memory depth (SR_CONF_BUFFERSIZE, FULL by
		 * default): 512K samples per channel, or with NORMAL only the
		 * 4096 samples of the display record. The setting itself
		 * cannot be read back (:ACQ:MEM? reports the depth of the
		 * last shot), so it is sent for every memory capture.
		 */
		if (rigol_ds_config_set(sdi, devc->v1_mem_normal ?
				":ACQ:MEM NORMAL" : ":ACQ:MEM FULL") != SR_OK)
			return SR_ERR;
		/*
		 * Preliminary: this is the rate of the last capture. It is
		 * read again after each shot and sent as meta if it differs.
		 */
		if (sr_scpi_get_float(sdi->conn, ":ACQ:SAMP?", &sample_rate) != SR_OK ||
				sample_rate <= 0) {
			sr_err("Cannot get the sample rate (:ACQ:SAMP?).");
			return SR_ERR;
		}
	}

	/* Gain and offset scale the samples, they may have been changed
	 * by other software since the device was opened. */
	if (protocol == PROTOCOL_V1 && rigol_ds_get_dev_cfg_vertical(sdi) != SR_OK)
		return SR_ERR;

	devc->v1_single = FALSE;
	devc->v1_failures = 0;
	if (protocol == PROTOCOL_V1 && devc->data_source == DATA_SOURCE_LIVE) {
		/* The SINGLE sweep stops after each shot, see capture_start. */
		if (sr_scpi_get_string(sdi->conn, ":TRIG:MODE?", &v1_mode) != SR_OK)
			return SR_ERR;
		cmd = g_strdup_printf(":TRIG:%s:SWE?", v1_mode);
		g_free(v1_mode);
		ret = sr_scpi_get_string(sdi->conn, cmd, &v1_sweep);
		g_free(cmd);
		if (ret != SR_OK)
			return SR_ERR;
		devc->v1_single = !g_ascii_strcasecmp(v1_sweep, "SINGLE");
		g_free(v1_sweep);
	}

	/* Live capture with the SINGLE sweep: a new single shot per frame. */
	devc->live_single = FALSE;
	if (devc->data_source == DATA_SOURCE_LIVE && protocol >= PROTOCOL_V3) {
		char *sweep = NULL;
		if (sr_scpi_get_string(scpi, ":TRIG:SWE?", &sweep) == SR_OK && sweep)
			devc->live_single = !g_ascii_strncasecmp(sweep, "SING", 4);
		g_free(sweep);
	}

	/* Memory captures take single shots: remember what to restore. */
	g_free(devc->saved_sweep);
	devc->saved_sweep = NULL;
	if (devc->data_source == DATA_SOURCE_MEMORY && protocol >= PROTOCOL_V3) {
		char *status = NULL;
		if (sr_scpi_get_string(scpi, ":TRIG:SWE?", &devc->saved_sweep) != SR_OK)
			devc->saved_sweep = NULL;
		devc->saved_running = sr_scpi_get_string(scpi, ":TRIG:STAT?",
			&status) == SR_OK && status && status[0] != 'S';
		g_free(status);
		sr_dbg("Sweep %s, %s: restored after the capture.",
			devc->saved_sweep ? devc->saved_sweep : "unknown",
			devc->saved_running ? "running" : "stopped");
	}

	if (ds1000z_apply_mem_depth(sdi) != SR_OK)
		return SR_ERR;

	switch (devc->model->series->protocol) {
	case PROTOCOL_V2:
		if (rigol_ds_config_set(sdi, ":ACQ:MEMD LONG") != SR_OK)
			return SR_ERR;
		break;
	case PROTOCOL_V3:
		/* Apparently for the DS2000 the memory
		 * depth can only be set in Running state -
		 * this matches the behaviour of the UI. */
		if (rigol_ds_config_set(sdi, ":RUN") != SR_OK)
			return SR_ERR;
		if (rigol_ds_config_set(sdi, ":ACQ:MDEP %d",
					devc->analog_frame_size) != SR_OK)
			return SR_ERR;
		if (rigol_ds_config_set(sdi, ":STOP") != SR_OK)
			return SR_ERR;
		break;
	default:
		break;
	}

	if (devc->data_source == DATA_SOURCE_LIVE)
		if (rigol_ds_config_set(sdi, ":RUN") != SR_OK)
			return SR_ERR;

	devc->v1_session_thread = g_thread_self();
	devc->v1_acquiring = protocol == PROTOCOL_V1;

	/*
	 * The receive callback reads one buffer per call. Memory captures
	 * (e.g. 512K samples per channel on the VS5000, up to 24M points on
	 * the DS1000Z) are read after the shot: poll fast, or the poll
	 * interval limits the speed (32 KB per 50 ms).
	 */
	sr_scpi_source_add(sdi->session, scpi, G_IO_IN,
			devc->data_source == DATA_SOURCE_LIVE ? 50 : 2,
			rigol_ds_receive, (void *)sdi);

	devc->channel_entry = devc->enabled_channels;

	if (devc->data_source == DATA_SOURCE_LIVE &&
			devc->model->series->protocol == PROTOCOL_V1) {
		devc->sample_rate = rigol_ds_v1_live_rate(devc);
	} else if (devc->model->series->protocol == PROTOCOL_V1) {
		/* Memory: full ADC rate, read above. */
		devc->sample_rate = sample_rate;
	} else if (devc->data_source == DATA_SOURCE_LIVE) {
		devc->sample_rate = analog_frame_size(sdi) /
			(devc->timebase * devc->model->series->num_horizontal_divs);
	} else {
		float xinc, srate;
		/*
		 * WAV:XINC? describes the waveform of the current :WAV:MODE,
		 * which is still the screen (NORM) here: ask for the sample
		 * rate of the memory directly where the scope reports it.
		 */
		if (devc->data_source == DATA_SOURCE_MEMORY &&
				devc->model->series->protocol == PROTOCOL_V4 &&
				sr_scpi_get_float(sdi->conn, ":ACQ:SRAT?", &srate) == SR_OK &&
				srate > 0) {
			devc->sample_rate = srate;
			goto rate_done;
		}
		if (devc->model->series->protocol < PROTOCOL_V3) {
			sr_err("Cannot get samplerate (below V3).");
			return SR_ERR;
		}
		ret = sr_scpi_get_float(sdi->conn, "WAV:XINC?", &xinc);
		if (ret != SR_OK) {
			sr_err("Cannot get samplerate (WAV:XINC? failed).");
			return SR_ERR;
		}
		if (!xinc) {
			sr_err("Cannot get samplerate (zero XINC value).");
			return SR_ERR;
		}
		devc->sample_rate = 1. / xinc;
	}
rate_done:

	/* After the sample rate is known, outputs read it from the header. */
	std_session_send_df_header(sdi);
	/*
	 * The rate depends on the timebase and the data source, a value a
	 * frontend read before the start can be out of date (e.g. live
	 * before memory): send the rate of this capture.
	 */
	if (devc->sample_rate > 0)
		sr_session_send_meta(sdi, SR_CONF_SAMPLERATE,
			g_variant_new_uint64(devc->sample_rate));

	if (rigol_ds_capture_start(sdi) != SR_OK)
		return SR_ERR;

	/* Start of first frame. */
	std_session_send_df_frame_begin(sdi);

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_scpi_dev_inst *scpi;

	devc = sdi->priv;

	std_session_send_df_end(sdi);
	devc->v1_acquiring = FALSE;
	/* Before abort_reply, which may reconnect (new socket). */
	scpi = sdi->conn;
	rigol_ds_v1_cancel_resync(sdi);
	sr_scpi_source_remove(sdi->session, scpi);
	rigol_ds_v1_abort_reply(sdi);
	if (devc->model->series->protocol == PROTOCOL_V1)
		rigol_ds_v1_flush(sdi);

	/* VS5000 memory mode: restore the sweep, resume continuous sweeps. */
	if (devc->v1_trigger_mode && devc->v1_sweep) {
		rigol_ds_config_set(sdi, ":TRIG:%s:SWE %s",
			devc->v1_trigger_mode, devc->v1_sweep);
		if (g_ascii_strcasecmp(devc->v1_sweep, "SINGLE"))
			rigol_ds_config_set(sdi, ":RUN");
	}
	g_free(devc->v1_trigger_mode);
	g_free(devc->v1_sweep);
	devc->v1_trigger_mode = devc->v1_sweep = NULL;
	g_free(devc->v1_mem);
	devc->v1_mem = NULL;
	devc->v1_mem_size = 0;

	g_slist_free(devc->enabled_channels);
	devc->enabled_channels = NULL;

	if (devc->saved_sweep) {
		rigol_ds_config_set(sdi, ":TRIG:SWE %s", devc->saved_sweep);
		g_free(devc->saved_sweep);
		devc->saved_sweep = NULL;
		if (devc->saved_running)
			rigol_ds_config_set(sdi, ":RUN");
	}

	return SR_OK;
}

static struct sr_dev_driver rigol_ds_driver_info = {
	.name = "rigol-ds",
	.longname = "Rigol DS",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(rigol_ds_driver_info);
