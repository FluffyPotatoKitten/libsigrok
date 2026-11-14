/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2012 Martin Ling <martin-git@earth.li>
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
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

#ifndef LIBSIGROK_HARDWARE_RIGOL_DS_PROTOCOL_H
#define LIBSIGROK_HARDWARE_RIGOL_DS_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "rigol-ds"

/* Size of acquisition buffers */
#define ACQ_BUFFER_SIZE (32 * 1024)
/* VS5000: samples at the start of a channel block that end the record. */
#define V1_HEAD_SAMPLES 2
/* Minimum code next to a 0 byte at a live record end to treat it as a glitch. */
#define V1_GLITCH_MIN_NEIGHBOUR 16
/* VS5000 ADC codes: 0 is the top (+5 div), 255 the bottom (-5.2 div). */
#define V1_CODE_MIN 0
#define V1_CODE_MAX 255
/* VS5000: the acquisition memory spans up to about 26.2 divisions. */
#define V1_MEMORY_DIVS 26.3
/*
 * VS5000: margin after the memory span before a forced trigger (us). A shot
 * with an immediate real trigger takes 260..470 ms plus the pre-trigger fill
 * from :RUN to STOP (measured), a real trigger should win.
 */
#define V1_FORCE_MARGIN_US 600000

/* Maximum number of samples to retrieve at once. */
#define ACQ_BLOCK_SIZE (30 * 1000)

#define MAX_ANALOG_CHANNELS 4
#define MAX_DIGITAL_CHANNELS 16

enum protocol_version {
	PROTOCOL_V1, /* VS5000 */
	PROTOCOL_V2, /* DS1000 */
	PROTOCOL_V3, /* DS2000, DSO1000 */
	PROTOCOL_V4, /* DS1000Z */
	PROTOCOL_V5, /* MSO5000 */
};

enum data_format {
	/* Used by DS1000 versions up to 2.02, and VS5000 series */
	FORMAT_RAW,
	/* Used by DS1000 versions from 2.04 onwards and all later series */
	FORMAT_IEEE488_2,
};

enum data_source {
	DATA_SOURCE_LIVE,
	DATA_SOURCE_MEMORY,
	DATA_SOURCE_SEGMENTED,
};

struct rigol_ds_vendor {
	const char *name;
	const char *full_name;
};

struct rigol_ds_series {
	const struct rigol_ds_vendor *vendor;
	const char *name;
	enum protocol_version protocol;
	enum data_format format;
	uint64_t max_timebase[2];
	uint64_t min_vdiv[2];
	int num_horizontal_divs;
	int live_samples;
	int buffer_samples;
};

enum cmds {
	CMD_GET_HORIZ_TRIGGERPOS,
	CMD_SET_HORIZ_TRIGGERPOS,
};

struct rigol_ds_command {
	int cmd;
	const char *str;
};

struct rigol_ds_model {
	const struct rigol_ds_series *series;
	const char *name;
	uint64_t min_timebase[2];
	unsigned int analog_channels;
	bool has_digital;
	const char **trigger_sources;
	unsigned int num_trigger_sources;
	const struct rigol_ds_command *cmds;
};

enum wait_events {
	WAIT_NONE,    /* Don't wait */
	WAIT_TRIGGER, /* Wait for trigger (only live capture) */
	WAIT_BLOCK,   /* Wait for block data (only when reading sample mem) */
	WAIT_STOP,    /* Wait for scope stopping (only single shots) */
};

struct dev_context {
	const struct rigol_ds_model *model;
	enum data_format format;

	/* Device properties */
	const uint64_t (*timebases)[2];
	uint64_t num_timebases;
	const uint64_t (*vdivs)[2];
	uint64_t num_vdivs;

	/* Channel groups */
	struct sr_channel_group **analog_groups;
	struct sr_channel_group *digital_group;

	/* Acquisition settings */
	GSList *enabled_channels;
	uint64_t limit_frames;
	enum data_source data_source;
	uint64_t analog_frame_size;
	uint64_t digital_frame_size;

	/* Device settings */
	gboolean analog_channels[MAX_ANALOG_CHANNELS];
	gboolean digital_channels[MAX_DIGITAL_CHANNELS];
	gboolean la_enabled;
	float timebase;
	float sample_rate;
	float attenuation[MAX_ANALOG_CHANNELS];
	float vdiv[MAX_ANALOG_CHANNELS];
	int vert_reference[MAX_ANALOG_CHANNELS];
	float vert_origin[MAX_ANALOG_CHANNELS];
	float vert_offset[MAX_ANALOG_CHANNELS];
	float vert_inc[MAX_ANALOG_CHANNELS];
	char *trigger_source;
	float horiz_triggerpos;
	char *trigger_slope;
	/* DS1000Z: memory depth of the scope in points, 0 = AUTO (:ACQ:MDEP). */
	uint64_t mem_depth;
	/*
	 * DS1000Z: memory depth chosen through SR_CONF_BUFFERSIZE, as a level
	 * like the scope's menu (0 = AUTO, n = n-th depth, which is halved or
	 * quartered with more channels), -1 = not chosen. Applied when an
	 * acquisition starts, after the channels are switched.
	 */
	int mem_level;
	/* DS1000Z: the chosen depth itself, used when the channels allow it. */
	uint64_t mem_req;
	float trigger_level;
	/*
	 * Trigger sweep as the scope reports it: AUTO, NORMAL, SINGLE (V1),
	 * AUTO, NORM, SING (V3 and later).
	 */
	char *trigger_sweep;
	/* Trigger type (:TRIG:MODE?, e.g. EDGE) the sweep belongs to, V1 only. */
	char *trigger_type;
	/* V1: a live or memory acquisition is running. */
	gboolean v1_acquiring;
	/* V1: the thread that runs the acquisition (receive callback). */
	GThread *v1_session_thread;
	/*
	 * V1: commands from other threads while acquiring, sent by the
	 * acquisition between two waveform replies (rigol_ds_v1_flush).
	 */
	GMutex v1_cmd_lock;
	GSList *v1_pending_cmds;
	/* V1 live over USB: waveform requests failed in a row. */
	int v1_failures;
	char *coupling[MAX_ANALOG_CHANNELS];

	/* Number of frames received in total. */
	uint64_t num_frames;
	/* Number of frames available from the Segmented data source */
	uint64_t num_frames_segmented;
	/* GSList entry for the current channel. */
	GSList *channel_entry;
	/* Number of bytes received for current channel. */
	uint64_t num_channel_bytes;
	/* Number of bytes of block header read */
	uint64_t num_header_bytes;
	/* Number of bytes in current data block, if 0 block header expected */
	uint64_t num_block_bytes;
	/* Number of data block bytes already read */
	uint64_t num_block_read;
	/* What to wait for in *_receive */
	enum wait_events wait_event;
	/* Trigger/block copying/stop waiting status */
	int wait_status;
	/* Acq buffers used for reading from the scope and sending data to app */
	unsigned char *buffer;
	float *data;
	/* V1 (VS5000): one :WAVEFORM:DATA? reply carries all channels. */
	gboolean v1_header_pending;
	/* V1: reply bytes after the header that have not been read yet. */
	uint64_t v1_remaining;
	/* V1: first bytes of a channel, they belong at its end. */
	unsigned char v1_head[V1_HEAD_SAMPLES];
	/* V1: bytes collected in buffer for the next packet. */
	uint64_t v1_fill;
	/*
	 * VS5000 memory mode: raw data of the channels before the last one,
	 * held back so that the channels can be sent alternately.
	 */
	unsigned char *v1_mem;
	size_t v1_mem_size;
	/* VS5000 memory mode: trigger mode and sweep to restore at the end. */
	char *v1_trigger_mode;
	char *v1_sweep;
	/* VS5000 memory mode: start of the current shot, forced trigger sent. */
	int64_t v1_shot_start;
	gboolean v1_forced;
	/* VS5000 live capture with the SINGLE sweep: one :RUN per frame. */
	gboolean v1_single;
	/*
	 * VS5000 firmware patch level, from the *IDN? version: 0 for stock
	 * firmware, n for "03.00.00.01.P<n>" (community fixes for the LAN
	 * interface; level 1 makes :WAVEFORM:ALL? safe over LAN).
	 */
	int v1_fw_patch;
	/* VS5000 memory depth for memory captures: FULL (512K) or NORMAL (4096). */
	gboolean v1_mem_normal;
	/* VS5000 over LAN: a reconnect after out-of-step replies is pending. */
	gboolean v1_resyncing;
	/*
	 * Memory capture (V3 and later): trigger sweep and run state at the
	 * start (a single shot sets the sweep to SINGLE and stops the scope),
	 * restored at the end; start of the current shot, whether the scope
	 * was seen running in it and whether a trigger was forced in it
	 * (Auto sweep).
	 */
	char *saved_sweep;
	gboolean saved_running;
	int64_t single_start;
	gboolean single_running;
	gboolean single_forced;
	/* Live capture with the SINGLE sweep: one single shot per frame. */
	gboolean live_single;
};

SR_PRIV int rigol_ds_config_set(const struct sr_dev_inst *sdi, const char *format, ...);
SR_PRIV int rigol_ds_capture_start(const struct sr_dev_inst *sdi);
SR_PRIV int rigol_ds_channel_start(const struct sr_dev_inst *sdi);
SR_PRIV int rigol_ds_receive(int fd, int revents, void *cb_data);
SR_PRIV int rigol_ds_get_dev_cfg(const struct sr_dev_inst *sdi);
SR_PRIV int rigol_ds_get_dev_cfg_vertical(const struct sr_dev_inst *sdi);
SR_PRIV double rigol_ds_v1_record_divs(const struct dev_context *devc);
SR_PRIV uint64_t rigol_ds_v1_live_rate(const struct dev_context *devc);
SR_PRIV void rigol_ds_v1_input_range(const struct dev_context *devc,
	int ch, double *low, double *high);
SR_PRIV char *rigol_ds_v1_range_string(const struct dev_context *devc, int ch);
SR_PRIV gboolean rigol_ds_v1_memory_supported(const struct sr_dev_inst *sdi);
SR_PRIV int rigol_ds_v1_fw_patch(const char *version);
SR_PRIV void rigol_ds_v1_cancel_resync(const struct sr_dev_inst *sdi);
SR_PRIV int rigol_ds_v1_stop(const struct sr_dev_inst *sdi);
SR_PRIV void rigol_ds_v1_abort_reply(struct sr_dev_inst *sdi);
SR_PRIV void rigol_ds_v1_flush(const struct sr_dev_inst *sdi);

#endif
