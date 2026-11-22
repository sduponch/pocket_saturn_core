// A JVS I/O board as the Saturn's two pads or two Virtua Guns: the Analogizer SNAC adapter's JVS wiring
// (glue/sat_jvs.sv, an_jvs.sv, rtl/lightgun.sv). Included by smpc_fw.c before smpc_world.c, whose poll_run() calls
// jvs_step() (at most once a millisecond).
//
// The hardware frames, escapes and checks the bus's bytes and resends one frame every PERIOD ms by itself, keeping the
// last good answer in its buffer (128..255); this side (ported from MIRLO64's lang/n64rt/n64rt_jvs.c) never waits for
// the bus:
//   - the switch: the menu's "JVS (Analogizer)" (sat_jvs ON: pads, P1 gun or P1 + P2 guns). Off: the pins and the
//     pads are the Pocket's again.
//   - the init, a step at each look: reset twice a second apart, 3 s for the board to come back (a Sega 838-13683B
//     does not answer a SETADDR sent sooner), set the address 1, the identity, the features (players, switch bytes,
//     analog channels, general outputs), each frame 10 ms or more after the previous answer. Then the poll
//     01 20 <players> <bytes> [22 <channels>] [32 <bytes> <OUTPUT1>] is the hardware's, every POLL_MS (written again
//     when the outputs change). Answers stopping for RESTART_MS (the board off, the cable out): the Pocket's pad
//     again, and the init again.
//   - the JVS profile (data slot "JVS Profile", jvs_vendor.bin as on the other Analogizer JVS cores; sat_jvs keeps
//     its bytes 0xD8 .. 0xFF): 0xD8 .. 0xDB the axes of P1 X, P1 Y, P2 X, P2 Y (bit 7 defined, bits 6..4 node - 1:
//     node 1 only here, bit 3 inverted, bits 2..0 channel), 0xE8 .. 0xEB the guns' trigger masks (2 bytes a player,
//     JVS order), 0xF0 bit 0 the outputs valid, 0xF1 .. 0xF3 OUTPUT1's base, 0xF4 .. 0xF6 / 0xF7 .. 0xF9 the recoil
//     masks of P1 / P2. Read at each init and whenever a new one is loaded (PROF_SEQ).
//   - the switches (JVS order: Start, Service, Up, Down, Left, Right, Push 1, Push 2; Push 3 .. Push 8) as a Saturn
//     pad, player 1 on port 1 and player 2 on port 2 (a gun's trigger switches left out):
//       Push 1 2 3 = A B C   Push 4 5 6 = X Y Z   Push 7 8 = L R   Start = Start   (Service: unused)
//   - a gun: its axes (X 0..255 across the screen, Y the line + 8 for the core's 224 / 240 / 256 lines), its trigger
//     (the profile's mask) and Start, to sat_jvs's GUN registers; rtl/hps2pad.sv makes the port a Virtua Gun.
//   - the recoil (the games drive none: made from the trigger, as the other Analogizer JVS cores): menu "JVS recoil"
//     single (one 50 ms pulse a press) or auto (50 ms on, 150 ms off while held); OUTPUT1 = base | the recoil mask of
//     each player whose pulse is on.
#include "hw.h"
#define JV_IDX       IO(0x78)   // sat_jvs: W the register JV_REG reaches; R {ON, PADS, GUNS, RECOIL, VRES, ..., IDX}
#define JV_REG       IO(0x7C)
enum { JR_ID, JR_CTRL, JR_STAT, JR_XFER, JR_PTR, JR_DATA, JR_PADS = 8, JR_PADS_ON = 9, JR_PROF_A = 10, JR_PROF_D = 11,
       JR_PROF_SEQ = 12, JR_GUN1 = 13, JR_GUN2 = 14 };
enum {
#ifndef JV_MS_EDGES
#define JV_MS_EDGES 4000        // (a bench's firmware: -DJV_MS_EDGES=400, its milliseconds 10 times shorter)
#endif
	JV_MS = JV_MS_EDGES,        // SMPC CE edges (4 MHz): IO_TIMER would not do, CDB_HLE holds it at 0 while the SMPC
	                            // keeps the CD block in reset
	JV_INIT_PERIOD = 20,        // ms an init frame's answer is waited for
	JV_POLL_MS = 14,            // a 2-player answer with 8 channels and the frame take ~4 ms; the 838-13683B wants
	                            // 10 ms between them
	JV_GAP_MS = 10, JV_RESET_MS = 1000, JV_SETTLE_MS = 3000, JV_RESTART_MS = 500,
	JV_RECOIL_ON_MS = 50, JV_RECOIL_CYC_MS = 200
};
enum { J_OFF, J_RESET1, J_RESET2, J_SETADDR, J_IDENT, J_FEAT, J_RUN };

static uint8_t jv_st, jv_sent, jv_players, jv_bytes, jv_good, jv_nch, jv_nout, jv_seq;
static uint32_t jv_at, jv_seen, jv_next;   // the next init step; the last time GOOD moved; the next look (jv_now)
static uint32_t jv_now, jv_t24;            // the SMPC's CE edges (SMPC_FW TIME [23:0], 4 MHz) made 32 bits
static uint8_t jv_prof[40];                // the profile's bytes 0xD8 .. 0xFF
static uint32_t jv_out;                    // OUTPUT1 as last sent (bytes 1..3 in [7:0], [15:8], [23:16])
static uint8_t jv_trig_d[2], jv_rc_on[2];
static uint32_t jv_rc_t0[2];

static inline void jv_wr(uint32_t r, uint32_t v) { JV_IDX = r; JV_REG = v; }
static inline uint32_t jv_rd(uint32_t r) { JV_IDX = r; return JV_REG; }
static inline bool jv_before(uint32_t t, uint32_t at) { return (int32_t)(t - at) < 0; }
static inline uint32_t jv_p24(unsigned a) { return jv_prof[a] | jv_prof[a + 1] << 8 | (uint32_t)jv_prof[a + 2] << 16; }
#define JV_P(off) (off - 0xD8)

static void jv_send(const uint8_t* f, uint32_t n, uint32_t ctrl)
{
	jv_wr(JR_CTRL, ctrl & ~2u);                          // POLL off while the frame is written (else a torn frame goes out)
	jv_wr(JR_PTR, 0);
	JV_IDX = JR_DATA;
	for (uint32_t i = 0; i < n; i++) JV_REG = f[i];      // (each write moves PTR on)
	jv_wr(JR_XFER, n);
	if (ctrl & 2u) jv_wr(JR_CTRL, ctrl);                 // then POLL: resent from the next PERIOD on
	jv_sent = 1;
}
static uint32_t jv_answer(uint8_t* b, uint32_t max)    // the last good answer (node .. sum), up to max bytes
{
	uint32_t n = jv_rd(JR_XFER) & 0xFF;
	if (n > max) n = max;
	jv_wr(JR_PTR, 128);
	JV_IDX = JR_DATA;
	for (uint32_t i = 0; i < n; i++) b[i] = (uint8_t)JV_REG;   // (each read moves PTR on)
	return n;
}
static void jv_profile(void)                           // sat_jvs's copy of the profile (words 0x36 .. 0x3F)
{
	jv_seq = (uint8_t)jv_rd(JR_PROF_SEQ);
	for (unsigned w = 0; w < 10; w++) {
		jv_wr(JR_PROF_A, w);
		const uint32_t v = jv_rd(JR_PROF_D);
		for (unsigned k = 0; k < 4; k++) jv_prof[4 * w + k] = (uint8_t)(v >> (8 * k));   // (file byte 4k in [7:0])
	}
}
static bool jv_outputs(void) { return (jv_prof[JV_P(0xF0)] & 1) && jv_nout; }
static void jv_restart(uint32_t t, uint32_t after_ms)
{
	jv_wr(JR_PADS_ON, 0);                       // the Pocket's pad on port 1 until the board answers again
	jv_wr(JR_CTRL, 1u | JV_INIT_PERIOD << 8);   // (ON, POLL off)
	jv_st = J_RESET1; jv_sent = 0; jv_players = 0; jv_nch = 0; jv_nout = 0;
	jv_at = t + after_ms * JV_MS;
	jv_profile();
}
static void jv_poll(void)                       // the poll frame (the hardware's from now on), OUTPUT1 = jv_out
{
	uint8_t f[12];
	uint32_t n = 0;
	f[n++] = 0x01; f[n++] = 0x20; f[n++] = jv_players; f[n++] = jv_bytes;
	if (jv_nch) { f[n++] = 0x22; f[n++] = jv_nch; }
	if (jv_outputs()) {
		f[n++] = 0x32; f[n++] = jv_nout;
		for (unsigned k = 0; k < jv_nout; k++) f[n++] = (uint8_t)(jv_out >> (8 * k));
	}
	jv_send(f, n, 3u | JV_POLL_MS << 8);         // (ON, POLL)
	jv_sent = 0;
}

// a player's two switch bytes as a pad (pk_pad's joy1: active low, opposite directions cancel)
static uint32_t jv_pad(uint32_t a, uint32_t b)
{
	const bool up = a & 0x20, dn = a & 0x10, lt = a & 0x08, rt = a & 0x04;
	uint32_t p = 0;                              // (active high here)
	if (rt && !lt) p |= 1u << 15;
	if (lt && !rt) p |= 1u << 14;
	if (dn && !up) p |= 1u << 13;
	if (up && !dn) p |= 1u << 12;
	if (a & 0x80) p |= 1u << 11;                 // Start
	if (a & 0x02) p |= 1u << 10;                 // Push 1: A
	if (b & 0x80) p |= 1u << 9;                  // Push 3: C
	if (a & 0x01) p |= 1u << 8;                  // Push 2: B
	if (b & 0x04) p |= 1u << 7;                  // Push 8: R
	if (b & 0x40) p |= 1u << 6;                  // Push 4: X
	if (b & 0x20) p |= 1u << 5;                  // Push 5: Y
	if (b & 0x10) p |= 1u << 4;                  // Push 6: Z
	if (b & 0x08) p |= 1u << 3;                  // Push 7: L
	return ~p & 0xFFFF;
}
// an axis of the profile (0 P1 X, 1 P1 Y, 2 P2 X, 3 P2 Y) from the answer's channels (16 bits each); 0x8000 undefined
static uint32_t jv_axis(unsigned i, const uint8_t* ch)
{
	const uint8_t d = jv_prof[i];
	if (!(d & 0x80) || (d & 0x70) || (d & 7) >= jv_nch) return 0x8000;
	const uint32_t v = (uint32_t)ch[2 * (d & 7)] << 8 | ch[2 * (d & 7) + 1];
	return (d & 8) ? (~v & 0xFFFF) : v;
}

static void jv_features(const uint8_t* b, uint32_t n)  // 00 len 01 01, then 4 bytes a function, 00 at the end
{
	jv_players = 0; jv_bytes = 0; jv_nch = 0; jv_nout = 0;
	for (uint32_t i = 4; i + 3 < n; i += 4) {
		if (b[i] == 0x00) break;
		if (b[i] == 0x01) { jv_players = b[i + 1]; jv_bytes = (uint8_t)((b[i + 2] + 7u) / 8u); }
		if (b[i] == 0x03) jv_nch = b[i + 1];
		if (b[i] == 0x12) jv_nout = (uint8_t)((b[i + 1] + 7u) / 8u);
	}
	if (jv_players > 2) jv_players = 2;          // (the poll asks for these only)
	if (jv_bytes > 2) jv_bytes = 2;
	if (jv_nch > 8) jv_nch = 8;
	if (jv_nout > 3) jv_nout = 3;
	bool axes = false;                           // (no axis in the profile: the channels are not polled)
	for (unsigned i = 0; i < 4; i++) if (jv_prof[i] & 0x80) axes = true;
	if (!axes) jv_nch = 0;
}

// a new answer: 00 len 01 | 01 system, the players' bytes | [01 the channels, 2 bytes each] | [01 the outputs]
static void jv_take(const uint8_t* b, uint32_t n, uint32_t st, uint32_t t)
{
	const uint32_t a = 5u + (uint32_t)jv_players * jv_bytes;
	if (n < a || b[2] != 0x01 || b[3] != 0x01) return;
	const uint8_t* ch = (jv_nch && n >= a + 1u + 2u * jv_nch && b[a] == 0x01) ? &b[a + 1] : 0;
	const unsigned guns = (st >> 28) & 3, recoil = (st >> 26) & 3, vres = (st >> 24) & 3;
	const uint32_t lines = vres == 1 ? 240 : vres == 2 ? 256 : 224;
	uint32_t pad[2] = { 0xFFFF, 0xFFFF }, out = jv_p24(JV_P(0xF1));
	for (uint32_t i = 0; i < jv_players; i++) {
		const uint32_t s0 = b[5 + i * jv_bytes], s1 = jv_bytes > 1 ? b[6 + i * jv_bytes] : 0;
		const uint32_t mask = (uint32_t)jv_prof[JV_P(0xE8) + 2 * i] << 8 | jv_prof[JV_P(0xE9) + 2 * i];
		const bool trig = ((s0 << 8 | s1) & mask) != 0;
		pad[i] = jv_pad(s0 & ~(mask >> 8), s1 & ~(mask & 0xFF));     // (the trigger is the gun's only)
		if (i < guns) {
			uint32_t x = 0x8000, y = 0x8000;
			if (ch) { x = jv_axis(2 * i, ch); y = jv_axis(2 * i + 1, ch); }
			uint32_t y8 = 8 + ((y * lines) >> 16);
			if (y8 > 255) y8 = 255;
			jv_wr(i ? JR_GUN2 : JR_GUN1, (s0 & 0x80 ? 1u << 17 : 0) | (trig ? 1u << 16 : 0) | y8 << 8 | (x >> 8));
		}
		// the recoil: a press fires (auto: again every cycle while held); the pulse is on its first 50 ms
		if (recoil && trig && (!jv_trig_d[i] || (recoil == 2 && !jv_before(t, jv_rc_t0[i] + JV_RECOIL_CYC_MS * JV_MS)))) {
			jv_rc_on[i] = 1; jv_rc_t0[i] = t;
		}
		if (jv_rc_on[i] && !jv_before(t, jv_rc_t0[i] + JV_RECOIL_ON_MS * JV_MS)) jv_rc_on[i] = 0;
		if (!recoil) jv_rc_on[i] = 0;
		if (jv_rc_on[i]) out |= jv_p24(i ? JV_P(0xF7) : JV_P(0xF4));
		jv_trig_d[i] = trig;
	}
	jv_wr(JR_PADS, pad[1] << 16 | pad[0]);
	jv_wr(JR_PADS_ON, 1);
	if (jv_outputs() && out != jv_out) { jv_out = out; jv_poll(); }   // (the new OUTPUT1 in the hardware's frame)
}

static void jvs_step(void)
{
	const uint32_t t24 = IO(0x80) & 0xFFFFFF;          // (SMPC_FW TIME; poll_run comes far more often than its 4 s wrap)
	jv_now += (t24 - jv_t24) & 0xFFFFFF; jv_t24 = t24;
	const uint32_t t = jv_now;
	if (jv_before(t, jv_next)) return;
	jv_next = t + JV_MS;
	const uint32_t st = JV_IDX;
	if (!(st >> 31)) { jv_st = J_OFF; return; }        // (the menu's "JVS" off: sat_jvs gives the pins and pads back)
	if (jv_st == J_OFF) { jv_restart(t, 0); return; }
	if (jv_st == J_RUN) {
		if ((uint8_t)jv_rd(JR_PROF_SEQ) != jv_seq) {    // a new profile: its axes, masks and outputs from now on
			jv_profile();
			jv_out = jv_p24(JV_P(0xF1)); jv_poll();
		}
		const uint8_t g = (uint8_t)(jv_rd(JR_STAT) >> 8);
		if (g != jv_good) {
			// read again if another came meanwhile (GOOD moved)
			for (int k = 0; k < 3; k++) {
				uint8_t b[48];
				const uint32_t n = jv_answer(b, sizeof b);
				if ((uint8_t)(jv_rd(JR_STAT) >> 8) != g) continue;
				jv_take(b, n, st, t);
				break;
			}
			jv_good = g; jv_seen = t;
		} else if (!jv_before(t, jv_seen + JV_RESTART_MS * JV_MS)) jv_restart(t, JV_RESET_MS);
		return;
	}
	if (jv_before(t, jv_at)) return;
	if (jv_sent) {                                       // an init frame is out: its answer
		const uint32_t x = jv_rd(JR_XFER);
		if (x >> 31) return;                             // (BUSY)
		jv_sent = 0;
		uint8_t b[64];
		const uint32_t n = (x & (3u << 29)) ? 0 : jv_answer(b, sizeof b);
		jv_at = t + JV_GAP_MS * JV_MS;
		switch (jv_st) {
		case J_RESET1: jv_st = J_RESET2; jv_at = t + JV_RESET_MS * JV_MS; break;
		case J_RESET2: jv_st = J_SETADDR; jv_at = t + JV_SETTLE_MS * JV_MS; break;
		case J_SETADDR:
			if (n >= 4 && b[2] == 0x01 && b[3] == 0x01) jv_st = J_IDENT;
			else jv_restart(t, JV_RESET_MS);
			break;
		case J_IDENT: jv_st = J_FEAT; break;
		case J_FEAT:
			if (n > 4) jv_features(b, n);
			if (jv_players == 0) { jv_restart(t, JV_RESET_MS); break; }
			jv_out = jv_p24(JV_P(0xF1));
			jv_poll();
			jv_st = J_RUN; jv_good = (uint8_t)(jv_rd(JR_STAT) >> 8); jv_seen = t;
			break;
		default: break;
		}
		return;
	}
	static const uint8_t f_reset[] = { 0xFF, 0xF0, 0xD9 }, f_addr[] = { 0xFF, 0xF1, 0x01 }, f_ident[] = { 0x01, 0x10 },
		f_feat[] = { 0x01, 0x14 };
	const uint32_t c = 1u | JV_INIT_PERIOD << 8;
	switch (jv_st) {
	case J_RESET1: case J_RESET2: jv_send(f_reset, 3, c); break;
	case J_SETADDR: jv_send(f_addr, 3, c); break;
	case J_IDENT: jv_send(f_ident, 2, c); break;
	case J_FEAT: jv_send(f_feat, 2, c); break;
	default: break;
	}
}
