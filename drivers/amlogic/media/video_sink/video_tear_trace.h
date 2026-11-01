/* SPDX-License-Identifier: GPL-2.0 */
/*
 * drivers/amlogic/media/video_sink/video_tear_trace.h
 *
 * Opt-in timing trace for torn canvas-based video frames. Observation only.
 */

#ifndef VIDEO_TEAR_TRACE_H
#define VIDEO_TEAR_TRACE_H

#include <linux/types.h>

extern bool tear_trace_on;

/* video_hw.c, also declared in video_priv.h */
u32 get_cur_enc_line(void);

void tear_trace_isr_enter(u32 enc_line);
void tear_trace_isr_exit(u32 enc_line, bool over_sync);
/* A frame is about to be programmed for VD1. */
void tear_trace_frame(bool canvas_path, bool compressed, u32 canvas_reg,
		      u32 target, u32 other);
/* The VD1 canvas pointer queued for the next vsync. */
void tear_trace_queued(u32 canvas_reg, u32 value);
ssize_t tear_trace_read(char *buf, size_t size);
void tear_trace_set(bool on);

#endif
