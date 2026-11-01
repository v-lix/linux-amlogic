// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/amlogic/media/video_sink/video_tear_trace.c
 *
 * Opt-in timing trace for torn canvas-based (uncompressed) video frames.
 *
 * Canvas-based frames are written into the canvas set the vsync ISR expects
 * to be off screen, while the VD1 pointer switch to that set is queued by
 * RDMA for the next vsync. If the switch has not been applied when the next
 * ISR writes, or lands inside the active picture, the frame being scanned
 * changes mid-scan. This trace records, per vsync:
 *  - where the video ISR starts and ends (encoder line) and its duration,
 *  - whether the canvas pointer queued by the previous ISR is live when the
 *    next ISR starts (PEND),
 *  - whether a canvas set about to be rewritten is the one VD1 is pointing
 *    at (COLL),
 *  - sanity counters showing the PEND/COLL checks ran and compared real
 *    canvas pointers (checks, matches, the other set, unknown values),
 *  - each RDMA channel's executed table length (GUI register traffic),
 *  - VPU read/write arbiter registers, logged when they change,
 *  - the video layer, scaler and OSD blend/path registers, sampled at each
 *    ISR entry (E), each video (V) and OSD (O) RDMA completion and every
 *    millisecond (T); every change is counted and the first few per
 *    register and second are events with old/new value and encoder line.
 *    The sample point says when a change became visible, not who wrote
 *    it: both tables run at vsync, so V may show an OSD write.
 *    A change first seen at T, mid-frame, was not written by RDMA. V and O
 *    are sampled before that channel's irq_cb, so direct writes made by the
 *    callback itself (osd_hw_reset) show up at the next T or E,
 *  - per writer: the RDMA tables of the video, OSD and other channels are
 *    scanned for the same registers; video and other rdma_mgr tables when
 *    committed (they run at the next vsync), the OSD driver's own table
 *    from reset_rdma_table() (the entries that just ran). Each report gets
 *    the epoch of the vsync it runs at. A value that changes for a writer
 *    is counted (the first few per register and second are events); two
 *    writers putting one register in the same epoch is counted as shared,
 *    and as a fight, with an event, when their values differ: which value
 *    lands then depends on the order the channels run in,
 *  - where every RDMA channel (video, OSD, ...) IRQ runs, and its table
 *    length (summarised per channel each second),
 *  - every canvas LUT write request (canvas_config_ex/copy/update): how
 *    many asked for values the entry already holds, how many reached the
 *    hardware, how many of those landed inside the active picture, by
 *    canvas range (display, OSD, decoder, other), the most in one vsync,
 *    and a histogram of their encoder lines.
 *
 * Off unless /sys/class/video/tear_trace is set to 1. Kodi sets it only
 * while its debug logging is on and drains the result into kodi.log. When
 * off, the hot paths cost one predicted branch. Observation only.
 */

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/compiler.h>
#include <linux/math64.h>
#include <linux/hrtimer.h>
#include <linux/kallsyms.h>
#include <linux/amlogic/media/utils/vdec_reg.h>
#include <linux/amlogic/media/rdma/rdma_mgr.h>
#include <linux/amlogic/media/canvas/canvas.h>
#include "video_tear_trace.h"
#ifdef CONFIG_AMLOGIC_MEDIA_VSYNC_RDMA
#include "../common/rdma/rdma.h"
#endif


#define TT_BUCKETS 16
#define TT_EVENTS 128
#define TT_CHANNELS 8 /* RDMA_NUM in rdma_mgr.c */
#define TT_SUMMARY_NS (1000 * NSEC_PER_MSEC)
#define TT_DRAIN 32
#define TT_TRUNC_RESERVE 16

enum {
	TT_EV_PEND,
	TT_EV_COLL,
	TT_EV_OVER,
	TT_EV_REG,
	TT_EV_QREG,
	TT_EV_FIGHT,
};

static const char * const tt_ev_name[] = {"PEND", "COLL", "OVER", "REG",
					  "QREG", "FIGHT"};

/* RDMA table writers */
enum { TT_W_VID, TT_W_OSD, TT_W_OTH, TT_W_N };
static const char * const tt_w_name[TT_W_N] = {"vid", "osd", "oth"};

/* register sample points */
enum { TT_AT_E, TT_AT_V, TT_AT_O, TT_AT_T, TT_AT_N };
static const char tt_at_name[TT_AT_N] = {'E', 'V', 'O', 'T'};

struct tt_reg {
	u32 addr;
	const char *name;
};

/*
 * Video layer (written by the video driver), VPP scaler/blend, and OSD1,
 * OSD blend and OSD scaler (written by the OSD driver). The OSD blend
 * registers are only named in osd/osd_reg.h, hence the raw addresses.
 */
static const struct tt_reg tt_regs[] = {
	{VPP_MISC, "VPP_MISC"},
	{VPP_POSTBLEND_H_SIZE, "VPP_POSTBLEND_H_SIZE"},
	{VPP_OUT_H_V_SIZE, "VPP_OUT_H_V_SIZE"},
	{VPP_POSTBLEND_VD1_H_START_END, "VPP_POSTBLEND_VD1_H"},
	{VPP_POSTBLEND_VD1_V_START_END, "VPP_POSTBLEND_VD1_V"},
	{VPP_PREBLEND_VD1_H_START_END, "VPP_PREBLEND_VD1_H"},
	{VPP_PREBLEND_VD1_V_START_END, "VPP_PREBLEND_VD1_V"},
	{VD1_BLEND_SRC_CTRL, "VD1_BLEND_SRC_CTRL"},
	{VPP_LINE_IN_LENGTH, "VPP_LINE_IN_LENGTH"},
	{VPP_PIC_IN_HEIGHT, "VPP_PIC_IN_HEIGHT"},
	{VPP_HSC_REGION12_STARTP, "VPP_HSC_REGION12"},
	{VPP_VSC_REGION12_STARTP, "VPP_VSC_REGION12"},
	{VPP_HSC_PHASE_CTRL, "VPP_HSC_PHASE_CTRL"},
	{VPP_VSC_PHASE_CTRL, "VPP_VSC_PHASE_CTRL"},
	{VPP_SC_MISC, "VPP_SC_MISC"},
	{VD1_IF0_GEN_REG, "VD1_IF0_GEN_REG"},
	{VD1_IF0_GEN_REG2, "VD1_IF0_GEN_REG2"},
	{VD1_IF0_GEN_REG3, "VD1_IF0_GEN_REG3"},
	{VD1_IF0_LUMA_X0, "VD1_IF0_LUMA_X0"},
	{VD1_IF0_LUMA_Y0, "VD1_IF0_LUMA_Y0"},
	{VD1_IF0_CHROMA_X0, "VD1_IF0_CHROMA_X0"},
	{VD1_IF0_CHROMA_Y0, "VD1_IF0_CHROMA_Y0"},
	{VIU_VD1_FMT_CTRL, "VIU_VD1_FMT_CTRL"},
	{VIU_VD1_FMT_W, "VIU_VD1_FMT_W"},
	{VD1_AFBCD0_MISC_CTRL, "VD1_AFBCD0_MISC_CTRL"},
	{VIU_MISC_CTRL0, "VIU_MISC_CTRL0"},
	{VIU_MISC_CTRL1, "VIU_MISC_CTRL1"},
	{0x39b0, "VIU_OSD_BLEND_CTRL"},
	{0x39c0, "VIU_OSD_BLEND_CTRL1"},
	{0x39b1, "VIU_OSD_BLEND_DIN0_SCOPE_H"},
	{0x39b2, "VIU_OSD_BLEND_DIN0_SCOPE_V"},
	{0x39bb, "VIU_OSD_BLEND_BLEND0_SIZE"},
	{OSD1_BLEND_SRC_CTRL, "OSD1_BLEND_SRC_CTRL"},
	{OSD2_BLEND_SRC_CTRL, "OSD2_BLEND_SRC_CTRL"},
	{0x1df1, "VPP_OSD1_IN_SIZE"},
	{0x1df5, "VPP_OSD1_BLD_H_SCOPE"},
	{0x1df6, "VPP_OSD1_BLD_V_SCOPE"},
	{0x1dff, "VPP_OSD_SCALE_CTRL"},
	{VIU_OSD1_CTRL_STAT, "VIU_OSD1_CTRL_STAT"},
	{VIU_OSD1_BLK0_CFG_W0, "VIU_OSD1_BLK0_CFG_W0"},
	{VIU_OSD1_BLK0_CFG_W3, "VIU_OSD1_BLK0_CFG_W3"},
	{VIU_OSD1_BLK0_CFG_W4, "VIU_OSD1_BLK0_CFG_W4"},
	{VPP_OSD_SC_CTRL0, "VPP_OSD_SC_CTRL0"},
	{VPP_OSD_SCI_WH_M1, "VPP_OSD_SCI_WH_M1"},
	{VPP_OSD_SCO_H_START_END, "VPP_OSD_SCO_H"},
	{VPP_OSD_SCO_V_START_END, "VPP_OSD_SCO_V"},
};

#define TT_NREGS ARRAY_SIZE(tt_regs)
#define TT_REG_SPAN 0x4000 /* table entries above this are not watched */
#define TT_REG_EVENTS 2 /* per register and second */
#define TT_FIGHT_EVENTS 8 /* per register and second, fights included */
#define TT_TIMER_NS (1000 * NSEC_PER_USEC)

struct tt_event {
	u64 t_us;
	u32 seq;
	u32 type;
	u32 line;
	u32 aux0, aux1, aux2;
};

struct tt_hist {
	u32 count, max, active;
	u32 bucket[TT_BUCKETS];
};

struct tt_channel {
	struct tt_hist hist;
	u32 last;
	void *cb;
	u32 items_max;
	u64 items_sum;
};

#define TT_ARB_REGS 8
static const u32 tt_arb_reg[TT_ARB_REGS] = {
	0x2790, /* VPU_RDARB_MODE_L1C1 */
	0x2799, /* VPU_RDARB_MODE_L1C2 */
	VPU_RDARB_MODE_L2C1,
	VPU_WRARB_MODE_L2C1,
	0x27c0, 0x27c1, 0x27c2, 0x27c3, /* 0x27c2: OSD "basic urgent" */
};

struct tt_geom {
	u32 mux, max_lncnt, vavon_b, vavon_e, vso_b;
};

struct tt_state {
	/* geometry, refreshed on enable and on each read */
	u32 mux, max_lncnt, vavon_b, vavon_e, vso_b;
	/* interval */
	u32 vsyncs, over, pend, coll, canvas_frames, comp_frames;
	/* sanity: the checks ran and saw real canvas pointers */
	u32 pend_chk, pend_same, coll_other, coll_unknown, self_over;
	u32 last_live, last_target, last_other, last_q_live, last_q_val;
	u32 arb[TT_ARB_REGS];
	bool arb_logged;
	/* blend/path registers */
	u32 rv[TT_NREGS];
	bool rv_valid;
	u32 rchg[TT_NREGS][TT_AT_N];
	u8 rbudget[TT_NREGS];
	u32 timer_samples;
	/* RDMA table values per writer: last, and per epoch parity */
	u32 qlast[TT_NREGS][TT_W_N];
	u64 qvalid[TT_W_N];
	u32 qev[TT_NREGS][TT_W_N][2], qee[TT_NREGS][TT_W_N][2];
	u64 last_enter_ns;
	u32 frame_ns;
	u32 qchg[TT_NREGS][TT_W_N];
	u32 qshare[TT_NREGS], qfight[TT_NREGS];
	u8 qbudget[TT_NREGS];
	u32 commits[TT_W_N];
	u32 q_late;
	/* canvas LUT write requests */
	u32 cv_req, cv_same, cv_written, cv_active;
	u32 cv_range[4]; /* display, osd, decoder, other */
	u32 cv_vsync, cv_vsync_max, cv_seq;
	struct tt_hist cv_hist;
	u32 dur_max_us;
	u64 dur_sum_us;
	struct tt_hist enter, exit;
	struct tt_channel ch[TT_CHANNELS];
	/* events */
	struct tt_event ev[TT_EVENTS];
	u32 ev_head, ev_count, ev_lost;
	/* pairing */
	u32 seq;
	u64 enter_ns;
	u32 enter_line;
	u32 q_reg, q_val, q_line;
	bool q_fresh;
	u64 summary_ns;
	bool geom_logged;
};

bool tear_trace_on;
static struct tt_state tt;
static struct tt_state tt_snap;
static struct tt_event tt_drain[TT_DRAIN];
static DEFINE_SPINLOCK(tt_lock);
static DEFINE_MUTEX(tt_read_mutex);
static void *tt_video_cb; /* irq_cb of the video vsync RDMA channel */
static void *tt_osd_cb;   /* irq_cb of the channel named osd_rdma_irq */
static struct hrtimer tt_timer;
static bool tt_timer_ready;
static u8 tt_reg_idx[TT_REG_SPAN]; /* address -> tt_regs index + 1 */

static void tt_read_geometry(struct tt_geom *s)
{
	s->mux = READ_VCBUS_REG(VPU_VIU_VENC_MUX_CTRL) & 0x3;
	if (s->mux == 2) {
		s->max_lncnt = READ_VCBUS_REG(ENCP_VIDEO_MAX_LNCNT);
		s->vavon_b = READ_VCBUS_REG(ENCP_VIDEO_VAVON_BLINE);
		s->vavon_e = READ_VCBUS_REG(ENCP_VIDEO_VAVON_ELINE);
		s->vso_b = READ_VCBUS_REG(ENCP_VIDEO_VSO_BLINE);
	} else {
		s->max_lncnt = 0;
		s->vavon_b = 0;
		s->vavon_e = 0;
		s->vso_b = 0;
	}
}

static bool tt_in_active(u32 line)
{
	return tt.vavon_e > tt.vavon_b &&
		line >= tt.vavon_b && line <= tt.vavon_e;
}

static void tt_hist_add(struct tt_hist *h, u32 line)
{
	u32 span = tt.max_lncnt + 1;
	u32 b;

	h->count++;
	if (line > h->max)
		h->max = line;
	if (tt_in_active(line))
		h->active++;
	b = span > 1 ? (line * TT_BUCKETS) / span : 0;
	if (b >= TT_BUCKETS)
		b = TT_BUCKETS - 1;
	h->bucket[b]++;
}

static struct tt_event *tt_event(u32 type, u32 line, u32 aux0, u32 aux1,
				 u32 aux2)
{
	struct tt_event *e;

	if (tt.ev_count == TT_EVENTS) {
		tt.ev_lost++;
		return NULL;
	}
	e = &tt.ev[(tt.ev_head + tt.ev_count) % TT_EVENTS];
	tt.ev_count++;
	e->t_us = ktime_to_us(ktime_get());
	e->seq = tt.seq;
	e->type = type;
	e->line = line;
	e->aux0 = aux0;
	e->aux1 = aux1;
	e->aux2 = aux2;
	return e;
}

/* tt_lock held */
static void tt_regs_sample(u32 at, u32 line)
{
	u32 i, v;

	for (i = 0; i < TT_NREGS; i++) {
		v = READ_VCBUS_REG(tt_regs[i].addr);
		if (tt.rv_valid && v != tt.rv[i]) {
			tt.rchg[i][at]++;
			if (tt.rbudget[i] < TT_REG_EVENTS) {
				tt.rbudget[i]++;
				tt_event(TT_EV_REG, line, tt.rv[i], v,
					 (i << 8) | at);
			}
		}
		tt.rv[i] = v;
	}
	tt.rv_valid = true;
}

static enum hrtimer_restart tt_timer_fn(struct hrtimer *timer)
{
	unsigned long flags;

	spin_lock_irqsave(&tt_lock, flags);
	if (tear_trace_on) {
		tt.timer_samples++;
		tt_regs_sample(TT_AT_T, get_cur_enc_line());
	}
	spin_unlock_irqrestore(&tt_lock, flags);
	if (!READ_ONCE(tear_trace_on))
		return HRTIMER_NORESTART;
	hrtimer_forward_now(timer, ns_to_ktime(TT_TIMER_NS));
	return HRTIMER_RESTART;
}

void tear_trace_isr_enter(u32 enc_line)
{
	unsigned long flags;

	spin_lock_irqsave(&tt_lock, flags);
	if (!tear_trace_on)
		goto out;
	tt.seq++;
	tt.vsyncs++;
	tt.enter_ns = ktime_get_ns();
	if (tt.last_enter_ns &&
	    tt.enter_ns - tt.last_enter_ns < 100 * NSEC_PER_MSEC)
		tt.frame_ns = (u32)(tt.enter_ns - tt.last_enter_ns);
	tt.last_enter_ns = tt.enter_ns;
	tt.enter_line = enc_line;
	tt_hist_add(&tt.enter, enc_line);
	tt_regs_sample(TT_AT_E, enc_line);
	if (tt.q_fresh) {
		u32 live = READ_VCBUS_REG(tt.q_reg);

		tt.pend_chk++;
		tt.last_q_live = live;
		tt.last_q_val = tt.q_val;
		if (live == tt.q_val)
			tt.pend_same++;
		if (live != tt.q_val) {
			tt.pend++;
			tt_event(TT_EV_PEND, enc_line, live, tt.q_val,
				 tt.q_line);
		}
		tt.q_fresh = false;
	}
out:
	spin_unlock_irqrestore(&tt_lock, flags);
}

void tear_trace_isr_exit(u32 enc_line, bool over_sync)
{
	unsigned long flags;
	u32 dur_us;

	spin_lock_irqsave(&tt_lock, flags);
	if (!tear_trace_on || !tt.enter_ns)
		goto out;
	dur_us = (u32)div_u64(ktime_get_ns() - tt.enter_ns, NSEC_PER_USEC);
	tt_hist_add(&tt.exit, enc_line);
	tt.dur_sum_us += dur_us;
	if (dur_us > tt.dur_max_us)
		tt.dur_max_us = dur_us;
	/* independent of over_sync: the encoder line wrapped during the ISR */
	if (enc_line < tt.enter_line)
		tt.self_over++;
	if (over_sync) {
		tt.over++;
		tt_event(TT_EV_OVER, enc_line, tt.enter_line, dur_us, 0);
	}
	tt.enter_ns = 0;
out:
	spin_unlock_irqrestore(&tt_lock, flags);
}

void tear_trace_frame(bool canvas_path, bool compressed, u32 canvas_reg,
		      u32 target, u32 other)
{
	unsigned long flags;

	spin_lock_irqsave(&tt_lock, flags);
	if (!tear_trace_on)
		goto out;
	if (compressed)
		tt.comp_frames++;
	if (canvas_path) {
		u32 live = READ_VCBUS_REG(canvas_reg);

		tt.canvas_frames++;
		tt.last_live = live;
		tt.last_target = target;
		tt.last_other = other;
		if (live == target) {
			tt.coll++;
			tt_event(TT_EV_COLL, get_cur_enc_line(), live, target,
				 compressed);
		} else if (live == other) {
			tt.coll_other++;
		} else {
			tt.coll_unknown++;
		}
	}
out:
	spin_unlock_irqrestore(&tt_lock, flags);
}

void tear_trace_queued(u32 canvas_reg, u32 value)
{
	unsigned long flags;

	spin_lock_irqsave(&tt_lock, flags);
	if (tear_trace_on) {
		tt.q_reg = canvas_reg;
		tt.q_val = value;
		tt.q_line = get_cur_enc_line();
		tt.q_fresh = true;
	}
	spin_unlock_irqrestore(&tt_lock, flags);
}

static void tt_rdma_done(int handle, void *irq_cb, u32 items)
{
	unsigned long flags;
	u32 line;

	if (handle < 0 || handle >= TT_CHANNELS)
		return;
	spin_lock_irqsave(&tt_lock, flags);
	if (!tear_trace_on)
		goto out;
	line = get_cur_enc_line();
	tt.ch[handle].cb = irq_cb;
	tt.ch[handle].last = line;
	tt.ch[handle].items_sum += items;
	if (items > tt.ch[handle].items_max)
		tt.ch[handle].items_max = items;
	tt_hist_add(&tt.ch[handle].hist, line);
	/* the line is when this IRQ ran, not when the table executed */
	if (irq_cb && irq_cb == tt_video_cb)
		tt_regs_sample(TT_AT_V, line);
	else if (irq_cb && irq_cb == tt_osd_cb)
		tt_regs_sample(TT_AT_O, line);
out:
	spin_unlock_irqrestore(&tt_lock, flags);
}

static u32 tt_writer(void *irq_cb)
{
	if (irq_cb && irq_cb == tt_video_cb)
		return TT_W_VID;
	if (irq_cb && irq_cb == tt_osd_cb)
		return TT_W_OSD;
	return TT_W_OTH;
}

/*
 * The vsync a table runs at, numbered by the video ISR that precedes it: a
 * table committed during ISR n runs at the next vsync, epoch n. An executed
 * table is reported early in the frame it ran at, before or after that
 * frame's video ISR (n + 1); within half a frame of it, it was after.
 * That holds while both IRQs run in the first half of the frame; an
 * executed table reported later is not matched (epoch 0) but counted.
 */
static u32 tt_epoch(bool executed, u64 now, u32 line)
{
	u32 half = tt.frame_ns ? tt.frame_ns / 2 : 8 * NSEC_PER_MSEC;

	if (!executed)
		return tt.seq;
	if (tt.max_lncnt && line > tt.max_lncnt / 2) {
		tt.q_late++;
		return 0;
	}
	if (tt.last_enter_ns && now - tt.last_enter_ns < half)
		return tt.seq - 1;
	return tt.seq;
}

/* The last entry per register in a table is what lands. */
static void tt_rdma_config(int handle, void *irq_cb, const u32 *table,
			   u32 items, bool executed)
{
	unsigned long flags;
	struct tt_event *e;
	u64 seen = 0;
	u32 w, w2, i, r, v, line, epoch, slot;

	spin_lock_irqsave(&tt_lock, flags);
	if (!tear_trace_on)
		goto out;
	w = tt_writer(irq_cb);
	tt.commits[w]++;
	line = get_cur_enc_line();
	epoch = tt_epoch(executed, ktime_get_ns(), line);
	slot = epoch & 1;
	for (i = items; i-- > 0;) {
		u32 adr = table[i << 1];

		if (adr >= TT_REG_SPAN || !tt_reg_idx[adr])
			continue;
		r = tt_reg_idx[adr] - 1;
		if (seen & BIT_ULL(r))
			continue;
		seen |= BIT_ULL(r);
		v = table[(i << 1) + 1];
		for (w2 = 0; epoch && w2 < TT_W_N; w2++) {
			if (w2 == w || !(tt.qvalid[w2] & BIT_ULL(r)) ||
			    tt.qee[r][w2][slot] != epoch)
				continue;
			tt.qshare[r]++;
			if (tt.qev[r][w2][slot] == v)
				continue;
			tt.qfight[r]++;
			if (tt.qbudget[r] < TT_FIGHT_EVENTS) {
				tt.qbudget[r]++;
				e = tt_event(TT_EV_FIGHT, line,
					     tt.qev[r][w2][slot], v,
					     (r << 8) | (w2 << 4) | (w << 1));
				if (e)
					e->seq = epoch;
			}
		}
		if ((tt.qvalid[w] & BIT_ULL(r)) && v != tt.qlast[r][w]) {
			tt.qchg[r][w]++;
			if (tt.qbudget[r] < TT_REG_EVENTS) {
				tt.qbudget[r]++;
				e = tt_event(TT_EV_QREG, line, tt.qlast[r][w],
					     v, (r << 8) | (w << 1));
				if (e)
					e->seq = epoch;
			}
		}
		tt.qvalid[w] |= BIT_ULL(r);
		tt.qlast[r][w] = v;
		tt.qev[r][w][slot] = v;
		tt.qee[r][w][slot] = epoch;
	}
out:
	spin_unlock_irqrestore(&tt_lock, flags);
}

static u32 tt_canvas_range(u32 index)
{
	if (index >= 0x58 && index <= 0x6f)
		return 0;
	if (index >= 0x40 && index <= 0x48)
		return 1;
	if (index <= 0x1a || (index >= 0x78 && index <= 0xbf))
		return 2;
	return 3;
}

/* called under the canvas lock */
static void tt_canvas_write(u32 index, bool same, bool written)
{
	unsigned long flags;
	u32 line;

	spin_lock_irqsave(&tt_lock, flags);
	if (!tear_trace_on)
		goto out;
	tt.cv_req++;
	if (same)
		tt.cv_same++;
	if (!written)
		goto out;
	line = get_cur_enc_line();
	tt.cv_written++;
	if (tt_in_active(line))
		tt.cv_active++;
	tt.cv_range[tt_canvas_range(index)]++;
	tt_hist_add(&tt.cv_hist, line);
	if (tt.cv_seq != tt.seq) {
		tt.cv_seq = tt.seq;
		tt.cv_vsync = 0;
	}
	if (++tt.cv_vsync > tt.cv_vsync_max)
		tt.cv_vsync_max = tt.cv_vsync;
out:
	spin_unlock_irqrestore(&tt_lock, flags);
}

void tear_trace_set(bool on)
{
	unsigned long flags;
	struct tt_geom geom;
	u32 i;

	BUILD_BUG_ON(TT_NREGS > 64); /* tt.qvalid and the seen masks */
	if (on) {
#ifdef CONFIG_AMLOGIC_MEDIA_VSYNC_RDMA
		struct rdma_op_s *op = get_rdma_ops(VSYNC_RDMA);

		tt_video_cb = op ? (void *)op->irq_cb : NULL;
#endif
		/* static in osd_rdma.c; only compared, never called */
		tt_osd_cb = (void *)kallsyms_lookup_name("osd_rdma_irq");
		memset(tt_reg_idx, 0, sizeof(tt_reg_idx));
		for (i = 0; i < TT_NREGS; i++)
			if (tt_regs[i].addr < TT_REG_SPAN)
				tt_reg_idx[tt_regs[i].addr] = i + 1;
		spin_lock_irqsave(&tt_lock, flags);
		memset(&tt, 0, sizeof(tt));
		tt_read_geometry(&geom);
		tt.mux = geom.mux;
		tt.max_lncnt = geom.max_lncnt;
		tt.vavon_b = geom.vavon_b;
		tt.vavon_e = geom.vavon_e;
		tt.vso_b = geom.vso_b;
		tt.summary_ns = ktime_get_ns();
		tear_trace_on = true;
		spin_unlock_irqrestore(&tt_lock, flags);
		rdma_set_done_trace(tt_rdma_done);
		rdma_set_config_trace(tt_rdma_config);
		canvas_set_write_trace(tt_canvas_write);
		if (!tt_timer_ready) {
			hrtimer_init(&tt_timer, CLOCK_MONOTONIC,
				     HRTIMER_MODE_REL);
			tt_timer.function = tt_timer_fn;
			tt_timer_ready = true;
		}
		hrtimer_start(&tt_timer, ns_to_ktime(TT_TIMER_NS),
			      HRTIMER_MODE_REL);
	} else {
		rdma_set_done_trace(NULL);
		rdma_set_config_trace(NULL);
		canvas_set_write_trace(NULL);
		spin_lock_irqsave(&tt_lock, flags);
		tear_trace_on = false;
		spin_unlock_irqrestore(&tt_lock, flags);
		if (tt_timer_ready)
			hrtimer_cancel(&tt_timer);
	}
	pr_info("video: tear trace %s\n", on ? "on" : "off");
}

static int tt_hist_print(char *buf, size_t size, const char *name,
			 const struct tt_hist *h)
{
	int n, i;

	n = scnprintf(buf, size, " %s=%u/%u/%u[", name, h->count, h->max,
		      h->active);
	for (i = 0; i < TT_BUCKETS; i++)
		n += scnprintf(buf + n, size - n, "%s%u", i ? "," : "",
			       h->bucket[i]);
	n += scnprintf(buf + n, size - n, "]");
	return n;
}

/*
 * Lines, in output order. A read is either an event read or, once a
 * second, a summary read (events wait for the next read):
 *  tear-geom mux= max_lncnt= vavon_b= vavon_e= vso_b=   (on change)
 *  tear-arb rXXXX=...                                    (on change)
 *  tear-ev t_us= seq= type= line= a0= a1= a2=          (each anomaly)
 *  tear-reg t_us= seq= at=E|V|O|T line= NAME(addr) old->new
 *                         read-back change (first per register and second)
 *  tear-q t_us= epoch= by= line= NAME(addr) old->new
 *                         RDMA table value change, by writer (same budget)
 *  tear-fight t_us= epoch= by= vs= line= NAME(addr) 0xby vs 0xvs
 *                         two writers, one register, one vsync, two values
 *  tear-ev lost=          events dropped since the last read
 *  summary read:
 *  tear-int ...
 *  tear-canvas req= same= written= skipped= active= disp= osd= dec= oth=
 *              max_per_vsync= lines=n/max/active[16 buckets]
 *  tear-regchg timer= osd_cb= NAME=E/V/O/T ...  read-back changes per sample
 *                                                point, nonzero registers only
 *  tear-qchg commits=vid/osd/oth late= NAME=vid/osd/oth/shared/fight ...
 *                                                nonzero registers only
 *  tear-chk ... sanity of the PEND/COLL checks
 *  tear-rdma h= cb= last= items_max= items_avg= n/max/active[16 buckets]
 *  tear-trunc             the page filled up; the lines above it are cut
 * Histograms are count/max_line/active_count[buckets over the frame].
 */
ssize_t tear_trace_read(char *buf, size_t size)
{
	unsigned long flags;
	struct tt_state *s = &tt_snap;
	struct tt_geom geom;
	u32 arb[TT_ARB_REGS];
	bool summary, geom_changed, arb_changed = false;
	u64 now;
	int n = 0;
	u32 i, ev_count, lost;

	if (!tear_trace_on)
		return scnprintf(buf, size, "tear-off\n");
	if (size <= TT_TRUNC_RESERVE)
		return 0;
	/* keep room to say the page filled up */
	size -= TT_TRUNC_RESERVE;

	mutex_lock(&tt_read_mutex);
	tt_read_geometry(&geom);
	for (i = 0; i < TT_ARB_REGS; i++)
		arb[i] = READ_VCBUS_REG(tt_arb_reg[i]);
	now = ktime_get_ns();

	spin_lock_irqsave(&tt_lock, flags);
	geom_changed = !tt.geom_logged || geom.mux != tt.mux ||
		geom.max_lncnt != tt.max_lncnt ||
		geom.vavon_b != tt.vavon_b || geom.vavon_e != tt.vavon_e;
	tt.mux = geom.mux;
	tt.max_lncnt = geom.max_lncnt;
	tt.vavon_b = geom.vavon_b;
	tt.vavon_e = geom.vavon_e;
	tt.vso_b = geom.vso_b;
	tt.geom_logged = true;
	if (!tt.arb_logged || memcmp(arb, tt.arb, sizeof(arb))) {
		memcpy(tt.arb, arb, sizeof(arb));
		tt.arb_logged = true;
		arb_changed = true;
	}
	summary = now - tt.summary_ns >= TT_SUMMARY_NS;
	/* drain at most what fits; the rest stays for the next read */
	ev_count = summary ? 0 : min_t(u32, tt.ev_count, TT_DRAIN);
	for (i = 0; i < ev_count; i++)
		tt_drain[i] = tt.ev[(tt.ev_head + i) % TT_EVENTS];
	tt.ev_head = (tt.ev_head + ev_count) % TT_EVENTS;
	tt.ev_count -= ev_count;
	lost = tt.ev_lost;
	tt.ev_lost = 0;
	if (summary) {
		/* stats only; the event ring stays in tt */
		*s = tt;
		tt.summary_ns = now;
		tt.vsyncs = 0;
		tt.pend_chk = 0;
		tt.pend_same = 0;
		tt.coll_other = 0;
		tt.coll_unknown = 0;
		tt.self_over = 0;
		tt.over = 0;
		tt.pend = 0;
		tt.coll = 0;
		tt.canvas_frames = 0;
		tt.comp_frames = 0;
		tt.dur_max_us = 0;
		tt.dur_sum_us = 0;
		memset(&tt.enter, 0, sizeof(tt.enter));
		memset(&tt.exit, 0, sizeof(tt.exit));
		for (i = 0; i < TT_CHANNELS; i++) {
			memset(&tt.ch[i].hist, 0, sizeof(tt.ch[i].hist));
			tt.ch[i].items_max = 0;
			tt.ch[i].items_sum = 0;
		}
		memset(tt.rchg, 0, sizeof(tt.rchg));
		memset(tt.rbudget, 0, sizeof(tt.rbudget));
		tt.timer_samples = 0;
		memset(tt.qchg, 0, sizeof(tt.qchg));
		memset(tt.qshare, 0, sizeof(tt.qshare));
		memset(tt.qfight, 0, sizeof(tt.qfight));
		memset(tt.qbudget, 0, sizeof(tt.qbudget));
		memset(tt.commits, 0, sizeof(tt.commits));
		tt.q_late = 0;
		tt.cv_req = 0;
		tt.cv_same = 0;
		tt.cv_written = 0;
		tt.cv_active = 0;
		memset(tt.cv_range, 0, sizeof(tt.cv_range));
		tt.cv_vsync_max = 0;
		memset(&tt.cv_hist, 0, sizeof(tt.cv_hist));
	}
	spin_unlock_irqrestore(&tt_lock, flags);

	if (geom_changed)
		n += scnprintf(buf + n, size - n,
			       "tear-geom mux=%u max_lncnt=%u vavon_b=%u vavon_e=%u vso_b=%u\n",
			       geom.mux, geom.max_lncnt, geom.vavon_b,
			       geom.vavon_e, geom.vso_b);
	if (arb_changed) {
		n += scnprintf(buf + n, size - n, "tear-arb");
		for (i = 0; i < TT_ARB_REGS; i++)
			n += scnprintf(buf + n, size - n, " r%04x=0x%08x",
				       tt_arb_reg[i], arb[i]);
		n += scnprintf(buf + n, size - n, "\n");
	}
	for (i = 0; i < ev_count; i++) {
		const struct tt_event *e = &tt_drain[i];
		u32 r = e->aux2 >> 8;

		if (e->type == TT_EV_REG && r < TT_NREGS)
			n += scnprintf(buf + n, size - n,
				       "tear-reg t_us=%llu seq=%u at=%c line=%u %s(0x%04x) 0x%08x->0x%08x\n",
				       e->t_us, e->seq,
				       tt_at_name[(e->aux2 & 0xff) % TT_AT_N],
				       e->line, tt_regs[r].name, tt_regs[r].addr,
				       e->aux0, e->aux1);
		else if (e->type == TT_EV_QREG && r < TT_NREGS)
			n += scnprintf(buf + n, size - n,
				       "tear-q t_us=%llu epoch=%u by=%s line=%u %s(0x%04x) 0x%08x->0x%08x\n",
				       e->t_us, e->seq,
				       tt_w_name[((e->aux2 >> 1) & 7) % TT_W_N],
				       e->line, tt_regs[r].name, tt_regs[r].addr,
				       e->aux0, e->aux1);
		else if (e->type == TT_EV_FIGHT && r < TT_NREGS)
			n += scnprintf(buf + n, size - n,
				       "tear-fight t_us=%llu epoch=%u by=%s vs=%s line=%u %s(0x%04x) 0x%08x vs 0x%08x\n",
				       e->t_us, e->seq,
				       tt_w_name[((e->aux2 >> 1) & 7) % TT_W_N],
				       tt_w_name[((e->aux2 >> 4) & 0xf) % TT_W_N],
				       e->line, tt_regs[r].name, tt_regs[r].addr,
				       e->aux1, e->aux0);
		else
			n += scnprintf(buf + n, size - n,
				       "tear-ev t_us=%llu seq=%u type=%s line=%u a0=0x%x a1=0x%x a2=%u\n",
				       e->t_us, e->seq, tt_ev_name[e->type],
				       e->line, e->aux0, e->aux1, e->aux2);
	}
	if (lost)
		n += scnprintf(buf + n, size - n, "tear-ev lost=%u\n", lost);
	if (summary) {
		n += scnprintf(buf + n, size - n,
			       "tear-int t_us=%llu vsyncs=%u canvas=%u comp=%u over=%u pend=%u coll=%u dur_max_us=%u dur_avg_us=%u",
			       ktime_to_us(ktime_get()), s->vsyncs,
			       s->canvas_frames, s->comp_frames, s->over,
			       s->pend, s->coll, s->dur_max_us,
			       s->vsyncs ? (u32)div_u64(s->dur_sum_us,
							 s->vsyncs) : 0);
		n += tt_hist_print(buf + n, size - n, "enter", &s->enter);
		n += tt_hist_print(buf + n, size - n, "exit", &s->exit);
		n += scnprintf(buf + n, size - n, "\n");
		n += scnprintf(buf + n, size - n,
			       "tear-canvas req=%u same=%u written=%u skipped=%u active=%u disp=%u osd=%u dec=%u oth=%u max_per_vsync=%u",
			       s->cv_req, s->cv_same, s->cv_written,
			       s->cv_req - s->cv_written, s->cv_active,
			       s->cv_range[0], s->cv_range[1], s->cv_range[2],
			       s->cv_range[3], s->cv_vsync_max);
		n += tt_hist_print(buf + n, size - n, "lines", &s->cv_hist);
		n += scnprintf(buf + n, size - n, "\n");
		n += scnprintf(buf + n, size - n,
			       "tear-regchg timer=%u osd_cb=%ps", s->timer_samples,
			       tt_osd_cb);
		for (i = 0; i < TT_NREGS; i++) {
			const u32 *c = s->rchg[i];

			if (!(c[TT_AT_E] | c[TT_AT_V] | c[TT_AT_O] | c[TT_AT_T]))
				continue;
			n += scnprintf(buf + n, size - n, " %s=%u/%u/%u/%u",
				       tt_regs[i].name, c[TT_AT_E], c[TT_AT_V],
				       c[TT_AT_O], c[TT_AT_T]);
		}
		n += scnprintf(buf + n, size - n, "\n");
		n += scnprintf(buf + n, size - n,
			       "tear-qchg commits=%u/%u/%u late=%u",
			       s->commits[TT_W_VID], s->commits[TT_W_OSD],
			       s->commits[TT_W_OTH], s->q_late);
		for (i = 0; i < TT_NREGS; i++) {
			const u32 *c = s->qchg[i];

			if (!(c[TT_W_VID] | c[TT_W_OSD] | c[TT_W_OTH] |
			      s->qshare[i]))
				continue;
			n += scnprintf(buf + n, size - n, " %s=%u/%u/%u/%u/%u",
				       tt_regs[i].name, c[TT_W_VID],
				       c[TT_W_OSD], c[TT_W_OTH], s->qshare[i],
				       s->qfight[i]);
		}
		n += scnprintf(buf + n, size - n, "\n");
		n += scnprintf(buf + n, size - n,
			       "tear-chk pend_chk=%u pend_same=%u coll_chk=%u coll_other=%u coll_unknown=%u self_over=%u q_live=0x%x q_val=0x%x live=0x%x target=0x%x other=0x%x\n",
			       s->pend_chk, s->pend_same, s->canvas_frames,
			       s->coll_other, s->coll_unknown, s->self_over,
			       s->last_q_live, s->last_q_val, s->last_live,
			       s->last_target, s->last_other);
		for (i = 0; i < TT_CHANNELS; i++) {
			if (!s->ch[i].hist.count)
				continue;
			n += scnprintf(buf + n, size - n,
				       "tear-rdma h=%u cb=%ps last=%u items_max=%u items_avg=%u",
				       i, s->ch[i].cb, s->ch[i].last,
				       s->ch[i].items_max,
				       (u32)div_u64(s->ch[i].items_sum,
						    s->ch[i].hist.count));
			n += tt_hist_print(buf + n, size - n, "done",
					   &s->ch[i].hist);
			n += scnprintf(buf + n, size - n, "\n");
		}
	}
	if ((size_t)n >= size - 1)
		n += scnprintf(buf + n, TT_TRUNC_RESERVE + 1,
			       "\ntear-trunc\n");
	mutex_unlock(&tt_read_mutex);
	return n;
}
