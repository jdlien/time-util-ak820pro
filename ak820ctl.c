// ak820ctl -- host utility for the AK820 Pro QMK port.
//
//   ak820ctl clock [YYYY-MM-DDTHH:MM:SS]   set the RTC (default: host time)
//   ak820ctl info                          flash id + writable base
//   ak820ctl flash write <addr> <file>     erase + program + verify
//   ak820ctl flash erase <addr> [n]        erase n 4K sectors
//   ak820ctl flash crc   <addr> <len>      CRC32 a range
//   ak820ctl list                          list HID interfaces
//
// Everything speaks VIA's custom-value layout, so one protocol works against
// both the VIA and non-VIA builds:
//   [ID_CUSTOM_SET_VALUE, channel, cmd, payload...]
// Channel 0x10 is the clock, 0x11 the flash. The reply echoes the buffer;
// data[0] stays ID_CUSTOM_SET_VALUE when handled, and for flash commands
// data[3] carries a status byte.
//
// cc ak820ctl.c -o ak820ctl $(pkg-config --cflags --libs hidapi)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <hidapi.h>

#define VID 0x0C45
#define PID 0x8009            // the QMK build's PID
#define RAW_USAGE_PAGE 0xFF60
#define RAW_USAGE      0x61

#define ID_CUSTOM_SET_VALUE 0x07
#define ID_UNHANDLED        0xFF

#define RTC_CHANNEL   0x10
#define RTC_SET_TIME  0x01
#define RTC_GET_TIME  0x02

#define FLASH_CHANNEL  0x11
#define FC_INFO        0x01
#define FC_ERASE       0x02
#define FC_WRITE_BEGIN 0x03
#define FC_WRITE_DATA  0x04
#define FC_WRITE_END   0x05
#define FC_CRC32       0x06
#define FC_STATUS      0x07
#define FC_UNLOCK      0x08
#define FC_CRC_NEXT    0x09

#define FS_OK      0x00
#define FS_BUSY    0x01
#define FS_REFUSED 0x02
#define FS_BADARG  0x03
#define FS_MORE    0x04

#define SECTOR 4096u
#define PAGE    256u
// 32-byte reports minus the report id, the 3-byte VIA header and our length
// byte. This is what caps throughput at roughly 27 KB/s.
#define CHUNK   27

static hid_device *dev;

static const char *fs_str(unsigned char s) {
    switch (s) {
        case FS_OK:      return "ok";
        case FS_BUSY:    return "busy";
        case FS_REFUSED: return "refused (write floor / locked / animation running)";
        case FS_BADARG:  return "bad argument";
        case FS_MORE:    return "in progress";
        default:         return "unknown status";
    }
}

// One command/reply round trip. rep[] receives the 32-byte reply payload.
static int xfer(const unsigned char *payload, int len, unsigned char *rep) {
    unsigned char buf[33] = {0};      // buf[0] = report id (stripped on the wire)
    memcpy(&buf[1], payload, len);
    if (hid_write(dev, buf, sizeof buf) < 0) {
        fprintf(stderr, "write failed: %ls\n", hid_error(dev));
        return -1;
    }
    int n = hid_read_timeout(dev, rep, 32, 2000);
    if (n <= 0) { fprintf(stderr, "no reply\n"); return -1; }
    if (rep[0] == ID_UNHANDLED) { fprintf(stderr, "command not handled by firmware\n"); return -1; }
    return 0;
}

// Fire-and-forget write: no reply is read, so this works in BT/2.4G mode too.
//
// WHY THIS EXISTS: the firmware routes raw-HID REPLIES through the active host
// driver (see host_raw_hid_send), so in wireless mode the answer is sent over the
// air where we are not listening, and every round-trip command fails -- even with
// the USB cable plugged in. Inbound reports are unaffected, which is why a
// one-way command still lands. Setting the clock needs no confirmation: if it
// silently fails, the next scheduled run fixes it.
static int xfer_nowait(const unsigned char *payload, int len) {
    unsigned char buf[33] = {0};      // buf[0] = report id (stripped on the wire)
    memcpy(&buf[1], payload, len);
    if (hid_write(dev, buf, sizeof buf) < 0) {
        fprintf(stderr, "write failed: %ls\n", hid_error(dev));
        return -1;
    }
    return 0;
}

// Flash commands share a header; retries absorb FS_BUSY, which the firmware
// returns instead of blocking (a synchronous erase would stall the matrix scan).
static int fcmd(unsigned char cmd, const unsigned char *args, int nargs, unsigned char *rep) {
    unsigned char p[32] = {ID_CUSTOM_SET_VALUE, FLASH_CHANNEL, cmd};
    if (nargs) memcpy(&p[3], args, nargs);
    for (int tries = 0; tries < 2000; tries++) {
        if (xfer(p, 3 + nargs, rep) < 0) return -1;
        if (rep[3] != FS_BUSY) break;
    }
    if (rep[3] != FS_OK && rep[3] != FS_MORE) {
        fprintf(stderr, "flash: %s\n", fs_str(rep[3])); return -1;
    }
    return 0;
}

/* Sleep until just before the next second boundary.
 *
 * WHY: time() returns WHOLE seconds. Firing at 12:34:56.812 tells the device
 * "56" when it is nearly 57 -- a lag uniform on [0,1), mean 0.5 s, injected by
 * the sync itself. The wire packet has no sub-second field and widening it was
 * rejected, so the only channel for sub-second precision is WHEN we transmit.
 *
 * lead_ns is how early to wake so the packet ARRIVES on the boundary: roughly
 * one 1 ms HID poll, plus firmware handling, plus the ~700 us bit-banged I2C
 * write to the PCF. 2 ms is a placeholder -- measure a round trip with
 * `clock --read` and use half of it. */
#ifndef XFER_LEAD_NS
#    define XFER_LEAD_NS 2000000L   /* 2 ms, placeholder -- see above */
#endif

static void align_to_second(long lead_ns) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    long ns = 1000000000L - ts.tv_nsec - lead_ns;
    if (ns > 0) {
        struct timespec req = {0, ns};
        nanosleep(&req, NULL);
    }
}

static int cmd_clock_legacy(const char *timearg, int nowait) {
    struct tm tmv;
    time_t t;
    if (timearg) {
        t = time(NULL);
    } else {
        /* Re-read AFTER the sleep and round to the NEAREST boundary, so both
         * nanosleep undershoot (wake at .998 -> round up) and overshoot (wake at
         * .003 past -> round down) send the second that is true on arrival.
         * macOS will overshoot under load; this is robust to it either way. */
        struct timespec ts;
        align_to_second(XFER_LEAD_NS);
        clock_gettime(CLOCK_REALTIME, &ts);
        t = ts.tv_sec + (ts.tv_nsec > 500000000L ? 1 : 0);
    }
    tmv = *localtime(&t);
    if (timearg) {
        int Y, M, D, h, m, s;
        if (sscanf(timearg, "%d-%d-%dT%d:%d:%d", &Y, &M, &D, &h, &m, &s) != 6) {
            fprintf(stderr, "bad time, use YYYY-MM-DDTHH:MM:SS\n"); return 1;
        }
        tmv.tm_year = Y - 1900; tmv.tm_mon = M - 1; tmv.tm_mday = D;
        tmv.tm_hour = h; tmv.tm_min = m; tmv.tm_sec = s;
        tmv.tm_isdst = -1;
        if (mktime(&tmv) == (time_t)-1) { fprintf(stderr, "invalid date/time\n"); return 1; }
    }
    unsigned char p[10] = {
        ID_CUSTOM_SET_VALUE, RTC_CHANNEL, RTC_SET_TIME,
        (unsigned char)(tmv.tm_year + 1900 - 2000), (unsigned char)(tmv.tm_mon + 1),
        (unsigned char)tmv.tm_mday, (unsigned char)tmv.tm_wday,
        (unsigned char)tmv.tm_hour, (unsigned char)tmv.tm_min, (unsigned char)tmv.tm_sec,
    };
    if (nowait) {
        if (xfer_nowait(p, sizeof p) < 0) return 1;
    } else {
        unsigned char rep[32];
        if (xfer(p, sizeof p, rep) < 0) return 1;
    }
    printf("clock set to %04d-%02d-%02d %02d:%02d:%02d%s\n",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec,
           nowait ? " (sent, unconfirmed)" : "");
    return 0;
}

/* Read the device clock back (legacy, whole seconds). Kept for firmware that
 * predates the sub-second tail; cmd_clock_read() below handles both. */
static int cmd_clock_read_legacy(void) {
    unsigned char p[3] = {ID_CUSTOM_SET_VALUE, RTC_CHANNEL, RTC_GET_TIME};
    unsigned char rep[32];
    if (xfer(p, sizeof p, rep) < 0) return 1;
    if (!rep[3]) { fprintf(stderr, "device clock not readable\n"); return 1; }
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    struct tm ht = *localtime(&now.tv_sec);
    printf("device %04d-%02d-%02d %02d:%02d:%02d\n",
           2000 + rep[4], rep[5], rep[6], rep[8], rep[9], rep[10]);
    printf("host   %04d-%02d-%02d %02d:%02d:%02d.%03ld\n",
           ht.tm_year + 1900, ht.tm_mon + 1, ht.tm_mday,
           ht.tm_hour, ht.tm_min, ht.tm_sec, now.tv_nsec / 1000000L);
    return 0;
}

/* ---- Sub-second clock sync (clock-sync plan 3.8) --------------------------
 * Firmware with RTC_PROTO_VERSION 2 returns a sub-second tail in the GET
 * reply ([11] version, [12..15] SECCNT, [16..17] period) and accepts
 * RTC_SET_TIME_MS: "at the instant this packet was received, true time was
 * t + ms". The host measures offset with NTP arithmetic (T3 = T2), sends
 * the set with an empirically CALIBRATED outbound lead, and learns the lead
 * from the firmware's own report of the offset it saw at receipt:
 *     o  = board - host              (GET, min-RTT sample)
 *     o' = target - board_at_receipt (SET reply)  = lead - delay - o
 *     e  = o' + o = lead - delay     ->  lead -= e/4   (converges on delay)
 * Signs verified in clock-sync-plan/review-codex-2026-09-01-round4.md. */
#define RTC_SET_TIME_MS   0x03
#define RTC_PROTO_VERSION 2

typedef struct { int proto; double lead_ms; int b_ppm; int has_bias; } cap_t;

static const char *cap_path(void) {
    static char p[512];
    const char *h = getenv("HOME");
    snprintf(p, sizeof p, "%s/.ak820ctl-cap", h ? h : ".");
    return p;
}
static cap_t cap_load(void) {
    cap_t c = { 0, 1.5, 0, 0 };
    FILE *f = fopen(cap_path(), "r");
    if (f) {
        int n = fscanf(f, "%d %lf %d", &c.proto, &c.lead_ms, &c.b_ppm);
        if (n < 2) { c.proto = 0; c.lead_ms = 1.5; }
        c.has_bias = (n == 3);
        fclose(f);
    }
    if (c.lead_ms < 0 || c.lead_ms > 10) c.lead_ms = 1.5;
    if (c.b_ppm < -600 || c.b_ppm > 600) c.has_bias = 0;
    return c;
}
static void cap_save(cap_t c) {
    FILE *f = fopen(cap_path(), "w");
    if (f) {
        if (c.has_bias) fprintf(f, "%d %.3f %d\n", c.proto, c.lead_ms, c.b_ppm);
        else            fprintf(f, "%d %.3f\n", c.proto, c.lead_ms);
        fclose(f);
    }
}

static double now_s(void) {
    struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}
static double host_sod(double t) {            /* host local seconds-of-day, fractional */
    time_t s = (time_t)t; struct tm lt = *localtime(&s);
    return lt.tm_hour * 3600.0 + lt.tm_min * 60.0 + lt.tm_sec + (t - (double)s);
}
/* Board seconds-of-day from a GET reply; <0 if the clock is unset.
 * The fraction is 1 - (active_period+1 - cnt)/(nominal_period+1): the
 * cycles REMAINING to the next boundary, measured in nominal seconds. In
 * steady state (active == nominal) that is cnt/(P+1); inside the shortened
 * first period after a phase set it is still correct, where the naive
 * cnt/(active+1) would read the fraction of the remaining window instead
 * (seen as an "after" residual growing to -790 ms across ten syncs). */
static double board_sod(const unsigned char *rep) {
    if (!rep[3]) return -1;
    unsigned cnt  = rep[12] | rep[13] << 8 | rep[14] << 16 | (unsigned)rep[15] << 24;
    unsigned per  = rep[16] | rep[17] << 8;
    unsigned pnom = rep[18] | rep[19] << 8;
    if (pnom == 0) pnom = per;
    double frac = 1.0 - ((double)per + 1.0 - (double)cnt) / ((double)pnom + 1.0);
    return rep[8] * 3600.0 + rep[9] * 60.0 + rep[10] + frac;
}
static double wrap_day(double d) { if (d > 43200) d -= 86400; if (d < -43200) d += 86400; return d; }

/* One GET. 0 ok, -1 no reply, -2 clock unset. off = board - host (ms). */
static int get_offset(double *off_ms, double *rtt_ms, int *proto, unsigned char *rep) {
    unsigned char p[3] = {ID_CUSTOM_SET_VALUE, RTC_CHANNEL, RTC_GET_TIME};
    double t0 = now_s();
    if (xfer(p, sizeof p, rep) < 0) return -1;
    double t1 = now_s();
    *proto  = rep[11];
    *rtt_ms = (t1 - t0) * 1000.0;
    double b = board_sod(rep);
    if (b < 0) return -2;
    *off_ms = wrap_day(b - host_sod((t0 + t1) / 2)) * 1000.0;
    return 0;
}

/* Build and send RTC_SET_TIME_MS for host time (t_enc + lead). */
static int send_set_ms(double lead_ms, int b_ppm, int has_bias, int nowait, unsigned char *rep) {
    double t_enc  = now_s();
    double target = t_enc + lead_ms / 1000.0;
    time_t ts = (time_t)target;
    unsigned ms = (unsigned)((target - (double)ts) * 1000.0);
    if (ms > 999) ms = 999;
    struct tm tmv = *localtime(&ts);
    unsigned char p[15] = {
        ID_CUSTOM_SET_VALUE, RTC_CHANNEL, RTC_SET_TIME_MS,
        (unsigned char)(tmv.tm_year + 1900 - 2000), (unsigned char)(tmv.tm_mon + 1),
        (unsigned char)tmv.tm_mday, (unsigned char)tmv.tm_wday,
        (unsigned char)tmv.tm_hour, (unsigned char)tmv.tm_min, (unsigned char)tmv.tm_sec,
        (unsigned char)(ms & 0xFF), (unsigned char)(ms >> 8),
        0,                       /* flags */
        (unsigned char)(has_bias ? (b_ppm & 0xFF) : 0xFF),          /* sof_bias s16 LE, */
        (unsigned char)(has_bias ? ((b_ppm >> 8) & 0xFF) : 0x7F),   /* 0x7FFF = unknown */
    };
    if (nowait) return xfer_nowait(p, sizeof p);
    return xfer(p, sizeof p, rep);
}

static int cmd_clock(const char *timearg, int nowait) {
    if (timearg) return cmd_clock_legacy(timearg, nowait);   /* explicit time: whole seconds, as before */
    cap_t cap = cap_load();

    if (nowait) {
        /* Send-only (a reply-less transport). Uses the cached capability. */
        if (cap.proto != RTC_PROTO_VERSION) return cmd_clock_legacy(NULL, 1);
        if (send_set_ms(cap.lead_ms, cap.b_ppm, cap.has_bias, 1, NULL) < 0) return 1;
        printf("clock set (sub-second, lead %.2f ms, sent, unconfirmed)\n", cap.lead_ms);
        return 0;
    }

    /* Measure: five GETs, keep the min-RTT sample. */
    unsigned char rep[32];
    double off = 0, rtt = 0, best_off = 0, rtt_min = 1e9, rtt_max = 0;
    int proto = 0, good = 0, was_slewing = 0;
    for (int i = 0; i < 5; i++) {
        int rc = get_offset(&off, &rtt, &proto, rep);
        if (rc == -1) return 1;
        if (proto != RTC_PROTO_VERSION) break;
        if (rc == -2) continue;                     /* clock unset: still fine to set */
        if (rep[20] & 0x02) was_slewing = 1;        /* a slew moves the board 20 ms/s: not a calibration sample */
        good++;
        if (rtt < rtt_min) { rtt_min = rtt; best_off = off; }
        if (rtt > rtt_max) rtt_max = rtt;
    }
    if (proto == 0) {
        printf("firmware without the sub-second protocol: legacy set\n");
        cap.proto = 0; cap_save(cap);
        return cmd_clock_legacy(NULL, 0);
    }
    if (proto != RTC_PROTO_VERSION) {
        fprintf(stderr, "unknown RTC protocol version %d -- refusing to act\n", proto);
        return 1;
    }
    double U = good ? (rtt_max - rtt_min) / 2.0 + 0.5 : 0;

    /* Set, with the calibrated lead; retry once on 0xFE. */
    int st = 0;
    for (int tries = 0; tries < 2; tries++) {
        if (send_set_ms(cap.lead_ms, cap.b_ppm, cap.has_bias, 0, rep) < 0) return 1;
        st = rep[3];
        if (st != 0xFE) break;
    }
    if (st == 0xFF) { fprintf(stderr, "firmware rejected the set (validation)\n"); return 1; }
    if (st == 0xFE) { fprintf(stderr, "firmware busy (stale read) twice -- try again\n"); return 1; }
    double o_prime = (double)(short)(rep[4] | rep[5] << 8);

    /* Learn the outbound lead from what the board saw at receipt -- only from
     * a board that was NOT mid-slew when measured (PLAN.md 3.8). */
    /* Only a SLEW-sized correction on a settled board is a calibration
     * sample: a step (|offset| > 500 ms, e.g. right after a flash while the
     * clock still drifts ~10 ms/s) blew the lead to its clamp once. Each
     * update moves the lead by at most 1 ms. */
    if (good && !was_slewing && st != 0 && best_off > -500 && best_off < 500) {
        double e = o_prime + best_off;
        double adj = -e / 4.0;
        if (adj > 1.0) adj = 1.0;
        if (adj < -1.0) adj = -1.0;
        cap.lead_ms += adj;
        if (cap.lead_ms < 0) cap.lead_ms = 0;
        if (cap.lead_ms > 10) cap.lead_ms = 10;
    }
    cap.proto = RTC_PROTO_VERSION;
    cap_save(cap);

    /* Verify. */
    double after = 0, rtt2 = 0; int pr2 = 0;
    int rc = get_offset(&after, &rtt2, &pr2, rep);
    printf("clock set (sub-second): before %+.1f ms, after %+.1f ms, rtt %.1f ms, U ~%.1f ms, lead now %.2f ms, bias %s%s\n",
           good ? best_off : 0.0, rc == 0 ? after : 0.0, rtt_min < 1e9 ? rtt_min : rtt2, U, cap.lead_ms,
           cap.has_bias ? "sent" : "unknown", st == 1 ? " (slewing)" : "");
    if (rc == 0 && good && st == 0 && (after > 3 * U + 3 || after < -(3 * U + 3)))
        printf("warning: residual %+.1f ms exceeds 3U -- lead still calibrating, or a slew is in progress\n", after);
    return 0;
}

static int cmd_clock_read(void) {
    unsigned char rep[32];
    double off = 0, rtt = 0; int proto = 0;
    int rc = get_offset(&off, &rtt, &proto, rep);
    if (rc == -1) return 1;
    if (proto != RTC_PROTO_VERSION) return cmd_clock_read_legacy();
    unsigned cnt = rep[12] | rep[13] << 8 | rep[14] << 16 | (unsigned)rep[15] << 24;
    unsigned per = rep[16] | rep[17] << 8, pnom = rep[18] | rep[19] << 8;
    {
        double frac = 1.0 - ((double)per + 1.0 - (double)cnt) / ((double)pnom + 1.0);
        printf("device %04d-%02d-%02d %02d:%02d:%02d%+.3f  (cnt %u / period %u, nominal %u)\n",
               2000 + rep[4], rep[5], rep[6], rep[8], rep[9], rep[10], frac, cnt, per, pnom);
    }
    if (rc == -2) { printf("device clock not set\n"); return 1; }
    printf("offset board-host %+.1f ms  rtt %.1f ms  flags 0x%02x  ref_state %u  sync_age %u min  last_host_offset %d ms\n",
           off, rtt, rep[20], rep[25], rep[26], (short)(rep[21] | rep[22] << 8));
    printf("sof_epoch %u  sof_frames_total %u  bias_in_use %d ppm\n", rep[27],
           rep[28] | rep[29] << 8 | rep[30] << 16 | (unsigned)rep[31] << 24, (short)(rep[23] | rep[24] << 8));
    return 0;
}

static int cmd_info(void) {
    unsigned char rep[32];
    if (fcmd(FC_INFO, NULL, 0, rep) < 0) return 1;
    unsigned long id   = ((unsigned long)rep[4] << 16) | (rep[5] << 8) | rep[6];
    unsigned long base = ((unsigned long)rep[7] << 16) | (rep[8] << 8) | rep[9];
    printf("flash jedec id : 0x%06lX\n", id);
    printf("writable from  : 0x%06lX (below this needs unlock, stock assets never)\n", base);
    return 0;
}

static int unlock(int on) {
    unsigned char a[1] = {(unsigned char)(on ? 1 : 0)}, rep[32];
    return fcmd(FC_UNLOCK, a, 1, rep);
}

static int erase_sectors(unsigned long addr, unsigned n) {
    unsigned char rep[32];
    for (unsigned i = 0; i < n; i++) {
        unsigned long a = addr + (unsigned long)i * SECTOR;
        unsigned char args[3] = {(unsigned char)(a >> 16), (unsigned char)(a >> 8), (unsigned char)a};
        if (fcmd(FC_ERASE, args, 3, rep) < 0) return -1;
        printf("\rerasing %u/%u", i + 1, n); fflush(stdout);
    }
    printf("\n");
    return 0;
}

// The firmware folds only ~1KB per call so the HID callback stays short (a
// single-shot verify of 184KB dropped its matrix scan from ~1396Hz to ~300Hz).
// It answers FS_MORE until the range is consumed, so pump it here.
static int device_crc(unsigned long addr, unsigned long len, unsigned long *out) {
    unsigned char rep[32];
    unsigned char c[6] = {(unsigned char)(addr >> 16), (unsigned char)(addr >> 8), (unsigned char)addr,
                          (unsigned char)(len >> 16), (unsigned char)(len >> 8), (unsigned char)len};
    if (fcmd(FC_CRC32, c, 6, rep) < 0) return -1;
    while (rep[3] == FS_MORE)
        if (fcmd(FC_CRC_NEXT, NULL, 0, rep) < 0) return -1;
    *out = ((unsigned long)rep[4] << 24) | ((unsigned long)rep[5] << 16) |
           ((unsigned long)rep[6] << 8)  | rep[7];
    return 0;
}

static unsigned long crc32_of(const unsigned char *p, size_t n) {
    unsigned long c = 0xFFFFFFFFul;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int b = 0; b < 8; b++) c = (c >> 1) ^ (0xEDB88320ul & (-(long)(c & 1)));
    }
    return ~c & 0xFFFFFFFFul;
}

static int cmd_write(unsigned long addr, const char *file, int do_unlock) {
    if (addr % PAGE) { fprintf(stderr, "address must be 256-byte aligned\n"); return 1; }
    FILE *f = fopen(file, "rb");
    if (!f) { perror(file); return 1; }
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fprintf(stderr, "empty file\n"); fclose(f); return 1; }
    unsigned char *buf = malloc((size_t)sz);
    if (!buf || fread(buf, 1, (size_t)sz, f) != (size_t)sz) {
        fprintf(stderr, "read failed\n"); fclose(f); free(buf); return 1;
    }
    fclose(f);

    if (do_unlock && unlock(1) < 0) { free(buf); return 1; }

    unsigned nsec = (unsigned)((sz + SECTOR - 1) / SECTOR);
    printf("writing %ld bytes to 0x%06lX (%u sectors)\n", sz, addr, nsec);
    if (erase_sectors(addr, nsec) < 0) { free(buf); return 1; }

    unsigned char rep[32];
    unsigned char args[3] = {(unsigned char)(addr >> 16), (unsigned char)(addr >> 8), (unsigned char)addr};
    if (fcmd(FC_WRITE_BEGIN, args, 3, rep) < 0) { free(buf); return 1; }

    for (long off = 0; off < sz; ) {
        unsigned char a[1 + CHUNK];
        long n = sz - off; if (n > CHUNK) n = CHUNK;
        a[0] = (unsigned char)n;
        memcpy(&a[1], buf + off, (size_t)n);
        if (fcmd(FC_WRITE_DATA, a, 1 + (int)n, rep) < 0) { free(buf); return 1; }
        off += n;
        if ((off & 0x3FFF) < CHUNK || off == sz) {
            printf("\r%ld/%ld bytes (%.0f%%)", off, sz, 100.0 * off / sz); fflush(stdout);
        }
    }
    printf("\n");
    if (fcmd(FC_WRITE_END, NULL, 0, rep) < 0) { free(buf); return 1; }

    // Verify on the device: CRC the range there and compare, rather than
    // reading megabytes back over 27-byte packets.
    unsigned long got;
    if (device_crc(addr, (unsigned long)sz, &got) < 0) { free(buf); return 1; }
    unsigned long want = crc32_of(buf, (size_t)sz);
    free(buf);
    if (do_unlock) unlock(0);

    if (got != want) {
        fprintf(stderr, "VERIFY FAILED: device 0x%08lX, file 0x%08lX\n", got, want);
        return 1;
    }
    printf("verified crc32=0x%08lX\n", got);
    return 0;
}

static int cmd_crc(unsigned long addr, unsigned long len) {
    unsigned long c;
    if (device_crc(addr, len, &c) < 0) return 1;
    printf("crc32(0x%06lX, %lu) = 0x%08lX\n", addr, len, c);
    return 0;
}

static void usage(const char *a0) {
    fprintf(stderr,
        "usage:\n"
        "  %s clock [YYYY-MM-DDTHH:MM:SS] [--no-wait]   set the RTC (default: host time)\n        --no-wait: do not read a reply, so it also works in BT/2.4G mode\n"
        "  %s info                          flash id + writable base\n"
        "  %s flash write <addr> <file> [--unlock]\n"
        "  %s flash erase <addr> [sectors]  4K sectors (default 1)\n"
        "  %s flash crc   <addr> <len>\n"
        "  %s list                          list HID interfaces\n"
        "\n"
        "addresses are hex (0x...) or decimal. --unlock permits writing the stock\n"
        "animation slots; the stock LCD assets are never writable.\n",
        a0, a0, a0, a0, a0, a0);
}

static int open_dev(int list) {
    if (hid_init()) { fprintf(stderr, "hid_init failed\n"); return -1; }
    struct hid_device_info *devs = hid_enumerate(VID, PID), *cur = devs;
    char path[512] = {0};
    for (; cur; cur = cur->next) {
        if (list)
            printf("usage_page=0x%04hX usage=0x%02hX iface=%d path=%s\n",
                   cur->usage_page, cur->usage, cur->interface_number, cur->path);
        if (cur->usage_page == RAW_USAGE_PAGE && cur->usage == RAW_USAGE)
            strncpy(path, cur->path, sizeof(path) - 1);
    }
    hid_free_enumeration(devs);
    if (list) return 1;   // caller exits
    if (!path[0]) {
        fprintf(stderr, "Raw HID interface (0xFF60) not found -- flashed with raw enabled?\n");
        return -1;
    }
    dev = hid_open_path(path);
    if (!dev) { fprintf(stderr, "open failed: %ls\n", hid_error(NULL)); return -1; }
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) { usage(argv[0]); return 1; }
    if (!strcmp(argv[1], "--help") || !strcmp(argv[1], "help")) { usage(argv[0]); return 0; }
    if (!strcmp(argv[1], "list")) { int r = open_dev(1); hid_exit(); return r < 0; }

    if (open_dev(0) < 0) return 1;
    int rc = 1;

    if (!strcmp(argv[1], "clock")) {
        int nowait = 0;
        const char *targ = NULL;
        for (int i = 2; i < argc; i++) {
            if (!strcmp(argv[i], "--no-wait")) nowait = 1;
            else if (!strcmp(argv[i], "--read")) targ = argv[i];
            else if (!targ)                    targ   = argv[i];
        }
        if (targ && !strcmp(targ, "--bias") && argc > 3) {
            cap_t cap = cap_load(); cap.b_ppm = atoi(argv[3]); cap.has_bias = 1; cap_save(cap);
            printf("sof bias %d ppm cached in %s\n", cap.b_ppm, cap_path()); return 0;
        }
        if (targ && !strcmp(targ, "--read")) rc = cmd_clock_read();
        else                                 rc = cmd_clock(targ, nowait);
    } else if (!strcmp(argv[1], "info")) {
        rc = cmd_info();
    } else if (!strcmp(argv[1], "flash") && argc >= 3) {
        int do_unlock = 0;
        for (int i = 3; i < argc; i++) if (!strcmp(argv[i], "--unlock")) do_unlock = 1;
        if (!strcmp(argv[2], "write") && argc >= 5) {
            rc = cmd_write(strtoul(argv[3], NULL, 0), argv[4], do_unlock);
        } else if (!strcmp(argv[2], "erase") && argc >= 4) {
            unsigned n = argc >= 5 && argv[4][0] != '-' ? (unsigned)strtoul(argv[4], NULL, 0) : 1;
            if (do_unlock && unlock(1) < 0) rc = 1;
            else { rc = erase_sectors(strtoul(argv[3], NULL, 0), n) < 0; if (do_unlock) unlock(0); }
        } else if (!strcmp(argv[2], "crc") && argc >= 5) {
            rc = cmd_crc(strtoul(argv[3], NULL, 0), strtoul(argv[4], NULL, 0));
        } else {
            usage(argv[0]);
        }
    } else {
        usage(argv[0]);
    }

    hid_close(dev);
    hid_exit();
    return rc;
}
