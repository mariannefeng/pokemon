#include <stdio.h>
#include <string.h>
#ifdef __ARM_ARCH
#undef __ARM_ARCH
#endif
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "pico/bootrom.h"
#include "tusb.h"
#include "i2s_out.pio.h"
#include "ff.h"
#include "sd_card.h"
#include <math.h> 

/* ── Pins ──────────────────────────────────────────────────────────────── */
#define DOUT_PIN         9    // MAX98357A DIN
#define BCLK_PIN        10    // MAX98357A BCLK  (LRCLK = GP11, implicit as BCLK+1)

#define VOLUME_PIN    26    // ADC0
#define SPEED_PIN     27    // ADC1
#define VIBRATO_PIN  28    // ADC2 (vibrato depth for SFX)

#define SAMPLE_RATE  48000

/* Active-low buttons, internal pull-up. */
#define PIN_UNUSED 255

#define SHIFT_PIN         4
#define NUM_SFX_BUTTONS   4
#define NUM_MUSIC_BUTTONS 3

#define SHIFT_LED_PIN 5

bool letsReset = false;

typedef struct {
    const char *path;   /* FatFs path under mount "0:" */
    bool        is_loop; /* true: music — one loop at a time; toggle off by pressing same again */
} audio_track_t;

// todo: add fart song // NEED TO UPDATE SD CARD + THINK ABOUT HOW TO TRIGGER
// other secret song choices:
    // magnetic by ILLIT
    // gameboy by katseye

/* Track order: 0..7 = one-shot SFX, 8..13 = looped music. Button→track mapping is below. */
static const audio_track_t tracks[] = {
    { "0:bulbasaur.wav",           false }, // fine
    { "0:charmander.wav",          false }, // fine
    { "0:pikachu.wav",             false }, // fine
    { "0:squirtle.wav",            false }, // NEED TO UPDATE SD CARD
    { "0:healing.wav",             false }, // NEED TO UPDATE SD CARD
    { "0:level-up.wav",            false }, // NEED TO UPDATE SD CARD
    { "0:plink.wav",               false }, // NEED TO UPDATE SD CARD
    { "0:gameboy-on.wav",          false }, // NEED TO UPDATE SD CARD

    { "0:music-theme.wav",         true }, // loops from beginning, sounds fine as-is with lil silence after the PO-KE-MON! part
    { "0:music-poke-center.wav",   true }, // NEED TO UPDATE SD CARD - loops from beginning
    { "0:music-wild-battle.wav",   true }, // NEED TO UPDATE SD CARD - loops from 644335 sample (need to *2)
    { "0:music-violet-city.wav",   true }, // NEED TO UPDATE SD CARD - loops from 481096 sample (need to *2)
    { "0:music-azalea-city.wav",   true }, // loops not from beginning - updated
    { "0:music-ecruteak-city.wav", true }, // NEED TO UPDATE SD CARD - loops from beginning
};

#define NUM_TRACKS ((int)(sizeof(tracks) / sizeof(*tracks)))

/* GP for each SFX / music key (unshifted & shifted use same GPIO; hold GP4 / SHIFT). */
static const uint8_t sfx_button_pin[NUM_SFX_BUTTONS]   = { 13, 20, 12, 21 };
static const uint8_t music_button_pin[NUM_MUSIC_BUTTONS] = { 15, 8, 14 };

/* Row 0 = unshifted, row 1 = with shift held. */
static const uint8_t sfx_track_id[2][NUM_SFX_BUTTONS] = {
    { 0, 1, 2, 3 },
    { 4, 5, 6, 7 },
};
static const uint8_t music_track_id[2][NUM_MUSIC_BUTTONS] = {
    { 8,  9,  10 },
    { 11, 12, 13 },
};

// TODO: refactor these fucken names they're bad and confuses me

/* Core 0 → core 1: loop layer — at most one music-* file; 255 = none / silence. */
static volatile uint8_t  active_loop_track = 255;
static volatile bool     loop_playing      = false;

/* One-shot SFX: increment trigger after setting sfx_track_id; any new press restarts that one-shot. */
// Replace single SFX trigger with a small core0->core1 event queue.
typedef struct {
    uint8_t track_id;
} sfx_event_t;

#define SFX_Q_CAP 32u                  // power of 2
#define SFX_Q_MASK (SFX_Q_CAP - 1u)
static volatile uint32_t sfx_q_w = 0;  // writer index (core0)
static volatile uint32_t sfx_q_r = 0;  // reader index (core1)
static sfx_event_t sfx_q[SFX_Q_CAP];
static volatile uint32_t sfx_q_drops = 0;

static inline bool sfx_q_push(uint8_t track_id) {
    uint32_t w = sfx_q_w;
    uint32_t r = sfx_q_r;
    if ((uint32_t)(w - r) >= SFX_Q_CAP) {
        sfx_q_drops++;
        return false;
    }
    sfx_q[w & SFX_Q_MASK].track_id = track_id;
    __dmb(); // publish event before advancing write index
    sfx_q_w = w + 1u;
    return true;
}

static inline bool sfx_q_pop(uint8_t *out_track_id) {
    uint32_t r = sfx_q_r;
    uint32_t w = sfx_q_w;
    if (r == w) return false;
    uint8_t id = sfx_q[r & SFX_Q_MASK].track_id;
    __dmb(); // ensure event read before advancing read index
    sfx_q_r = r + 1u;
    *out_track_id = id;
    return true;
}

/* Master volume from dial (core 0 updates, core 1 reads). Q16.16: 65536 = unity gain. */
static volatile uint32_t master_gain_q16;
static uint32_t volume_smooth_q16;

/* Music speed (core 0 updates, core 1 reads). Q16.16: 65536 = 1.0x */
static volatile uint32_t music_speed_q16 = 65536u;
static uint32_t speed_smooth_q16 = 65536u;

/* SFX vibrato (core 0 updates, core 1 reads).
 * depth_q16: 0..65536 (0 = no vibrato). This scales a small ±speed deviation.
 * rate_hz_q16: Hz in Q16.16
 */
static volatile uint32_t sfx_vibrato_depth_q16 = 0u;
static volatile uint32_t sfx_vibrato_rate_hz_q16 = 6u * 65536u; /* default 6 Hz */
static uint32_t vibrato_depth_smooth_q16 = 0u;

/* Wide dead band at ADC rails: at max, noise often dips tens–hundreds of counts below
 * a tiny 64-LSB plateau, which maps into the linear region and makes gain hunt. */
#define DIAL_ADC_LO_TH   256u   /* below: mute */
#define DIAL_ADC_HI_TH   3840u  /* at or above: unity (4096 − 256) */

/* Speed pot: map ADC → 0.50x..1.5x with a snap-to-1.0x zone. */
#define SPEED_ADC_LO_TH   32u
#define SPEED_ADC_HI_TH   4064u
#define SPEED_SNAP_PCT    5u        /* ±5% around 1.0x snaps to exactly 1.0x */
#define SPEED_MIN_Q16     32768u    /* 0.5x */
#define SPEED_MAX_Q16     98304u    /* 1.5x */

/* USB dial log: only when applied master gain (Q16) moves by at least this much.
 * 1024 ≈ 1.6 % linear (~0.13 dB at unity); increase if logs are too chatty. */
#define DIAL_LOG_GAIN_MIN_DELTA 1024u

/* ── DMA output buffer ───────────────────────────────────────────────────
   Each word: [left 16b | right 16b] (same mono sample on both channels).
   core1 fills these halves; DMA autonomously feeds the PIO TX FIFO.      */
// Keep this modest: these buffers live in `.bss` (RP2040 RAM is only 264KB).
// 4096 @ 48kHz ≈ 85ms per buffer; double-buffered ≈ 170ms worst-case latency.
#define MIX_SAMPLES (4 * 1024)

static uint32_t dial_adc_to_gain_q16(uint16_t adc) {
    if (adc < DIAL_ADC_LO_TH) {
        return 0u;
    }
    // if this was >=, we'd have to do -1 in the span calculation below
    if (adc > DIAL_ADC_HI_TH) {
        return 65536u;
    }
    uint32_t x = (uint32_t)adc - DIAL_ADC_LO_TH;
    uint32_t span = DIAL_ADC_HI_TH - DIAL_ADC_LO_TH;
    return (x * 65536u) / span;
}

static uint32_t adc_to_speed_q16(uint16_t adc) {
    if (adc < SPEED_ADC_LO_TH) {
        return SPEED_MIN_Q16;
    }
    if (adc > SPEED_ADC_HI_TH) {
        return SPEED_MAX_Q16;
    }

    uint32_t x = (uint32_t)adc - SPEED_ADC_LO_TH;
    uint32_t span = SPEED_ADC_HI_TH - SPEED_ADC_LO_TH;

    // t in 0..1 as Q16
    uint32_t t_q16 = (x * 65536u) / span;
    uint32_t range = SPEED_MAX_Q16 - SPEED_MIN_Q16;
    uint32_t sp = SPEED_MIN_Q16 + (uint32_t)(((uint64_t)t_q16 * (uint64_t)range) >> 16);

    // ensures snapping towards 1.0x (5% room for error)
    uint32_t snap = (65536u * SPEED_SNAP_PCT) / 100u;
    if (sp > 65536u - snap && sp < 65536u + snap) {
        sp = 65536u;
    }
    return sp;
}

static uint32_t adc_to_unit_q16(uint16_t adc) {
    if (adc >= 4095u) return 65536u;
    return ((uint32_t)adc * 65536u) / 4095u;
}

/* ── SFX vibrato helpers (core1) ───────────────────────────────────────── */
static inline uint32_t tri_lfo_unipolar_q16(uint32_t phase_q32) {
    // triangle LFO in Q16, 0..65536
    uint32_t x = phase_q32 >> 16; // 0..65535
    if (phase_q32 & 0x80000000u) {
        x = 65535u - x;
    }
    uint32_t v = x << 1;          // 0..131070
    return (v > 65536u) ? 65536u : v;
}

static inline int32_t tri_lfo_bipolar_q16(uint32_t phase_q32) {
    // -65536..+65536
    return (int32_t)tri_lfo_unipolar_q16(phase_q32) - 32768 - 32768;
}

static void apply_vibrato_to_sfx(int16_t *dst, const int16_t *src, uint32_t src_filled, uint32_t *phase_q32_io) {
    uint32_t depth_q16 = sfx_vibrato_depth_q16;
    uint32_t rate_hz_q16 = sfx_vibrato_rate_hz_q16;

    if (depth_q16 == 0u || rate_hz_q16 == 0u || src_filled < 2u) {
        uint32_t copy = src_filled < (uint32_t)MIX_SAMPLES ? src_filled : (uint32_t)MIX_SAMPLES;
        if (copy) memcpy(dst, src, copy * sizeof(int16_t));
        if (copy < (uint32_t)MIX_SAMPLES) {
            memset(dst + copy, 0, ((uint32_t)MIX_SAMPLES - copy) * sizeof(int16_t));
        }
        return;
    }

    const uint32_t VIB_MAX_DEV_Q16 = (65536u * 3u) / 4u;
    uint32_t dev_q16 = (uint32_t)(((uint64_t)depth_q16 * (uint64_t)VIB_MAX_DEV_Q16) >> 16); // 0..VIB_MAX_DEV_Q16

    // inc_q32 = (rate_hz_q16 * 2^16) / SAMPLE_RATE
    uint32_t inc_q32 = (uint32_t)(((uint64_t)rate_hz_q16 << 16) / (uint64_t)SAMPLE_RATE);

    uint32_t phase_q16 = 0;
    for (int i = 0; i < MIX_SAMPLES; i++) {
        uint32_t idx = phase_q16 >> 16;
        uint32_t frac = phase_q16 & 0xFFFFu;
        if (idx + 1u >= src_filled) {
            dst[i] = 0;
        } else {
            int32_t a = src[idx];
            int32_t b = src[idx + 1u];
            int32_t v = (int32_t)(((int64_t)a * (int64_t)(65536u - frac) + (int64_t)b * (int64_t)frac) >> 16);
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            dst[i] = (int16_t)v;
        }

        int32_t lfo = tri_lfo_bipolar_q16(*phase_q32_io); // -65536..+65536
        int32_t step_q16 = (int32_t)65536 + (int32_t)(((int64_t)(int32_t)dev_q16 * (int64_t)lfo) >> 16);
        if (step_q16 < 1) step_q16 = 1;
        phase_q16 += (uint32_t)step_q16;
        *phase_q32_io += inc_q32;
    }
}

// the 2 audio ping pong buffers that dma reads from
static uint32_t mix_buf[2][MIX_SAMPLES];  // zero-initialised aka silence

// set by core1, cleared by DMA IRQ. Indicates whether or not a buffer is filled and ready
static volatile bool     mix_ready[2];    
static volatile uint32_t mix_underruns = 0;

/* ── DMA ─────────────────────────────────────────────────────────────────
   Two DMA channels chained: when one completes it auto-triggers the other.
   IRQ fires on each completion, reconfigures that channel for next use
   (without triggering — the chain from the other channel will trigger it).
   Mirrors the pattern from elehobica/pico_audio_i2s_32b audio_i2s.c.     */
static int dma_a, dma_b;
static dma_channel_config dc_a, dc_b;
static volatile uint32_t *txf;   // PIO TX FIFO address, set in main()

static void __isr dma_irq_handler(void) {
    if (dma_channel_get_irq0_status(dma_a)) {
        dma_channel_acknowledge_irq0(dma_a);
        mix_ready[0] = false;
        if (!mix_ready[1]) {
            mix_underruns++;
        }
        dma_channel_configure(dma_a, &dc_a, (void *)txf, mix_buf[0], MIX_SAMPLES, false);
    }
    if (dma_channel_get_irq0_status(dma_b)) {
        dma_channel_acknowledge_irq0(dma_b);
        mix_ready[1] = false;
        if (!mix_ready[0]) {
            mix_underruns++;
        }
        dma_channel_configure(dma_b, &dc_b, (void *)txf, mix_buf[1], MIX_SAMPLES, false);
    }
}

/* ── WAV helper: scan RIFF chunks, return byte offset of "data" payload ── */
static uint32_t wav_data_offset(FIL *f) {
    f_lseek(f, 12);  // skip RIFF header ("RIFF", size, "WAVE")
    uint8_t c[8]; UINT br;
    while (f_read(f, c, 8, &br) == FR_OK && br == 8) {
        uint32_t sz = (uint32_t)c[4] | ((uint32_t)c[5] << 8)
                    | ((uint32_t)c[6] << 16) | ((uint32_t)c[7] << 24);
        if (c[0]=='d' && c[1]=='a' && c[2]=='t' && c[3]=='a')
            return (uint32_t)f_tell(f);
        f_lseek(f, f_tell(f) + sz + (sz & 1));
    }
    return 44;
}

static int16_t music_raw[MIX_SAMPLES];
static int16_t music_in[(MIX_SAMPLES * 2) + 4];

#define SFX_VOICES 3
typedef struct {
    bool     active;
    uint8_t  track_id;
    uint32_t vib_phase_q32;
} sfx_voice_t;

static int16_t sfx_raw[SFX_VOICES][MIX_SAMPLES];
static int16_t sfx_in[SFX_VOICES][(MIX_SAMPLES * 2) + 4];

// when we get r over USB, reset
void tud_cdc_rx_cb(uint8_t itf) {
    (void)itf;
    while (tud_cdc_available()) {
        char c = tud_cdc_read_char();
        if (c == 'r') {
            letsReset = true;
        }
    }
}

// core1_entry does everything related to audio in a tight loop like reading wav files from SD card, mixing active sounds, and filling the audio buffer. 
static void core1_entry(void) {
    printf("[c1] core1 started\n");

    FATFS fs;
    printf("[c1] attempting f_mount...\n");
    FRESULT fr = f_mount(&fs, "0:", 1);
    printf("[c1] f_mount result: %d\n", fr);

    // core0 waits for 0xCAFE from multicore fifo to tell if mount was successful
    if (fr != FR_OK) {
        multicore_fifo_push_blocking(0xDEAD);
        return;
    }
    printf("[c1] SD mounted\n");
    multicore_fifo_push_blocking(0xCAFE);

    FIL      track_files[NUM_TRACKS];

    // tracks starting byte offset for wav files
    uint32_t track_starts[NUM_TRACKS]; 
    uint32_t track_loop_start[NUM_TRACKS]; 

    // tracks whether or not we could open the file
    bool     track_ok[NUM_TRACKS];

    for (int t = 0; t < NUM_TRACKS; t++) {
        track_starts[t] = 0;
        track_ok[t] = false;
        if (f_open(&track_files[t], tracks[t].path, FA_READ) == FR_OK) {
            track_starts[t] = wav_data_offset(&track_files[t]);
            track_loop_start[t] = track_starts[t];
            track_ok[t] = true;
        } else {
            printf("[c1] cannot open %s\n", tracks[t].path);
        }

        // azalea city doesn't loop from the beginning
        if (t == 11) {
            track_loop_start[11] = track_loop_start[11] + 4125068;
            printf("[c1] setting track 11 start to 4125068 \n");
        }
    }

    int      fill = 0;

    bool     was_loop_playing = false;
    uint8_t  last_loop_track  = 255;
    sfx_voice_t voices[SFX_VOICES];
    for (int v = 0; v < SFX_VOICES; v++) {
        voices[v].active = false;
        voices[v].track_id = 255;
        voices[v].vib_phase_q32 = 0;
    }

    // each loop will fill 16384 samples
    while (true) {
        // this if statement is core1 waiting for DMA to consume the active buffer
        if (mix_ready[fill]) { tight_loop_contents(); continue; }

        bool     lp = loop_playing;
        uint8_t  lt = active_loop_track;

        // if we're supposed to be playing music, and the user 
        // just pressed the play button or the last played track was different, do something
        if (lp && (!was_loop_playing || lt != last_loop_track)) {

            // make sure active track idx is valid, the file was opened successfully, and is supposed to be looped
            if (lt < (uint8_t)NUM_TRACKS && track_ok[lt] && tracks[lt].is_loop) {
                // rewind to the start of the track
                f_lseek(&track_files[lt], track_starts[lt]);
            }
        }


        was_loop_playing = lp;
        last_loop_track  = lt;

        // Drain SFX queue: start/steal a voice for each event.
        uint8_t ev_id = 255;
        static uint32_t sfx_steal_rr = 0;
        while (sfx_q_pop(&ev_id)) {
            if (ev_id >= (uint8_t)NUM_TRACKS) continue;
            if (!track_ok[ev_id]) continue;
            if (tracks[ev_id].is_loop) continue; // SFX queue is for one-shots only

            int chosen = -1;
            for (int v = 0; v < SFX_VOICES; v++) {
                if (!voices[v].active) { chosen = v; break; }
            }
            if (chosen < 0) {
                // All voices busy: steal a voice round-robin.
                chosen = (int)(sfx_steal_rr % (uint32_t)SFX_VOICES);
                sfx_steal_rr++;
            }
            voices[chosen].track_id = ev_id;
            voices[chosen].active = true;
            voices[chosen].vib_phase_q32 = 0;
            f_lseek(&track_files[ev_id], track_starts[ev_id]);
        }
        
        // Fill music_raw with MIX_SAMPLES samples from the loop track, applying speed by resampling.
        // (This changes both pitch + tempo; SFX are left untouched.)
        if (lp && lt < (uint8_t)NUM_TRACKS && track_ok[lt] && tracks[lt].is_loop) {
            uint32_t sp_q16 = music_speed_q16;

            // clamp min/max to be within allowed speed range
            if (sp_q16 < SPEED_MIN_Q16) sp_q16 = SPEED_MIN_Q16;
            if (sp_q16 > SPEED_MAX_Q16) sp_q16 = SPEED_MAX_Q16;

            // if we're exactly at 1x speed
            if (sp_q16 == 65536u) {
                // TODO: this might not be perfect loop playback
                UINT br = 0;
                f_read(&track_files[lt], music_raw, MIX_SAMPLES * sizeof(int16_t), &br);
                int samples_read = (int)(br / sizeof(int16_t));

                // if we get to the end of the file, start from beginning again
                if (samples_read < MIX_SAMPLES) {
                    f_lseek(&track_files[lt], track_loop_start[lt]);
                    br = 0;
                    f_read(&track_files[lt], music_raw, MIX_SAMPLES * sizeof(int16_t), &br);
                }
            } else {
                // Need ~= (MIX_SAMPLES-1)*speed + 2 input samples for linear interpolation.
                uint32_t need_in = (uint32_t)((((uint64_t)(MIX_SAMPLES - 1) * (uint64_t)sp_q16) + 65535u) >> 16) + 2u;
                if (need_in > (MIX_SAMPLES * 2u)) {
                    need_in = (MIX_SAMPLES * 2u);
                }

                uint32_t filled = 0;
                while (filled < need_in) {
                    UINT br = 0;
                    uint32_t want = need_in - filled;
                    f_read(&track_files[lt], &music_in[filled], want * sizeof(int16_t), &br);
                    uint32_t got = (uint32_t)(br / sizeof(int16_t));
                    filled += got;
                    if (got < want) {
                        // EOF: wrap and continue reading.
                        // f_lseek(&track_files[lt], track_starts[lt]);
                        f_lseek(&track_files[lt], track_loop_start[lt]);
                        if (got == 0) {
                            break; // defensive: avoid infinite loop on read error
                        }
                    }
                }

                if (filled < 2u) {
                    memset(music_raw, 0, MIX_SAMPLES * sizeof(int16_t));
                } else {
                    uint32_t phase_q16 = 0;
                    for (int i = 0; i < MIX_SAMPLES; i++) {
                        uint32_t idx = phase_q16 >> 16;
                        uint32_t frac = phase_q16 & 0xFFFFu;
                        if (idx + 1u >= filled) {
                            music_raw[i] = 0;
                        } else {
                            int32_t a = music_in[idx];
                            int32_t b = music_in[idx + 1u];
                            int32_t v = (int32_t)(((int64_t)a * (int64_t)(65536u - frac) + (int64_t)b * (int64_t)frac) >> 16);
                            if (v > 32767) v = 32767;
                            if (v < -32768) v = -32768;
                            music_raw[i] = (int16_t)v;
                        }
                        phase_q16 += sp_q16;
                    }
                }
            }
        } else {
            memset(music_raw, 0, MIX_SAMPLES * sizeof(int16_t));
        }

        // Fill each SFX voice. Apply vibrato per-voice.
        for (int v = 0; v < SFX_VOICES; v++) {
            if (!voices[v].active) {
                memset(sfx_raw[v], 0, MIX_SAMPLES * sizeof(int16_t));
                continue;
            }

            UINT br = 0;
            uint8_t sid = voices[v].track_id;
            if (sid < (uint8_t)NUM_TRACKS && track_ok[sid] && !tracks[sid].is_loop) {
                // Read enough samples to handle worst-case instantaneous speed (>1.0x) for linear interpolation.
                const uint32_t VIB_MAX_DEV_Q16 = (65536u * 3u) / 100u;
                uint32_t dev_q16 = (uint32_t)(((uint64_t)sfx_vibrato_depth_q16 * (uint64_t)VIB_MAX_DEV_Q16) >> 16);
                uint32_t max_step_q16 = 65536u + dev_q16;
                uint32_t need_in = (uint32_t)((((uint64_t)(MIX_SAMPLES - 1) * (uint64_t)max_step_q16) + 65535u) >> 16) + 2u;
                if (need_in > (MIX_SAMPLES * 2u)) need_in = (MIX_SAMPLES * 2u);

                f_read(&track_files[sid], sfx_in[v], need_in * sizeof(int16_t), &br);
                uint32_t got = (uint32_t)(br / sizeof(int16_t));
                apply_vibrato_to_sfx(sfx_raw[v], sfx_in[v], got, &voices[v].vib_phase_q32);
            } else {
                br = 0;
            }

            int samples_read = (int)(br / sizeof(int16_t));
            if (br == 0 || samples_read < 2) {
                memset(sfx_raw[v], 0, MIX_SAMPLES * sizeof(int16_t));
                voices[v].active = false;
                voices[v].track_id = 255;
            }
        }

        /* Sum layers, apply master gain (Q16), clamp (allows loop + one-shots together), and pack to stereo i2s word */
        uint32_t g = master_gain_q16;
        for (int i = 0; i < MIX_SAMPLES; i++) {
            int64_t m = (int64_t)music_raw[i];
            for (int v = 0; v < SFX_VOICES; v++) {
                m += (int64_t)sfx_raw[v][i];
            }
            m = (m * (int64_t)g) >> 16;
            if (m > 32767) {
                m = 32767;
            }
            if (m < -32768) {
                m = -32768;
            }
            int16_t s = (int16_t)m;
            mix_buf[fill][i] = ((uint32_t)(uint16_t)s << 16) | (uint16_t)s;
        }

        __dmb();
        mix_ready[fill] = true;
        fill ^= 1;
    }
}


/* ── Core 0: main ────────────────────────────────────────────────────────
   No longer on the audio path – DMA handles sample delivery.
   Handles button, launches core1, and sets up PIO + DMA.                 */
int main(void) {
    stdio_init_all();
    tusb_init();
    // while (!stdio_usb_connected()) sleep_ms(100);


    printf("Pokemon SD player starting...\n");

    /* ADC + dial gain before core1 mixes: avoids one buffer at wrong volume. */
    adc_init();
    adc_gpio_init(VOLUME_PIN);
    adc_gpio_init(SPEED_PIN);
    adc_gpio_init(VIBRATO_PIN);
    adc_select_input(0); /* GPIO26 = ADC0 */
    
    // read the first 8 values and discard cause ADC start up values is garbage
    for (int w = 0; w < 8; w++) {
        (void)adc_read();
    }
    {
        uint32_t acc = 0;
        // take the average of 4 values cause natural jitter
        for (int s = 0; s < 4; s++) {
            acc += adc_read();
        }
        // adding by 2 and then dividing by 4 is standard way of doing integer division + round up
        uint16_t adc0 = (uint16_t)((acc + 2u) / 4u);
        uint32_t t = dial_adc_to_gain_q16(adc0);

        // initialize gain + smoothing value to current value
        volume_smooth_q16 = t;
        master_gain_q16   = t;
    }

    // Initialize speed pot (ADC1 / GPIO27)
    adc_select_input(1); /* GPIO27 = ADC1 */
    
    for (int w = 0; w < 8; w++) {
        (void)adc_read();
    }
    {
        uint32_t acc = 0;
        for (int s = 0; s < 4; s++) {
            acc += adc_read();
        }
        uint16_t adc1 = (uint16_t)((acc + 2u) / 4u);
        uint32_t sp = adc_to_speed_q16(adc1);
        speed_smooth_q16 = sp;
        music_speed_q16 = sp;
    }

    // Initialize vibrato pot (ADC2 / GPIO28) - depth only, SFX only
    adc_select_input(2); /* GPIO28 = ADC2 */
    for (int w = 0; w < 8; w++) {
        (void)adc_read();
    }
    {
        uint32_t acc = 0;
        for (int s = 0; s < 4; s++) {
            acc += adc_read();
        }
        uint16_t adc2 = (uint16_t)((acc + 2u) / 4u);
        uint32_t d = adc_to_unit_q16(adc2);
        vibrato_depth_smooth_q16 = d;
        sfx_vibrato_depth_q16 = d;
        // rate default already set; keep at 6 Hz unless changed later
    }

    // Return ADC selection to volume for the main loop.
    adc_select_input(0);

    multicore_launch_core1(core1_entry);
    uint32_t sig = multicore_fifo_pop_blocking();
    if (sig != 0xCAFE) {
        printf("Core1 SD init failed – halting\n");
        for (;;) tight_loop_contents();
    }

    /* I2S */
    PIO  pio = pio0;
    uint sm  = pio_claim_unused_sm(pio, true);
    uint off = pio_add_program(pio, &i2s_out_program);
    i2s_out_program_init(pio, sm, off, DOUT_PIN, BCLK_PIN, SAMPLE_RATE);

    /* DMA: two chained channels, ping-pong double-buffering.
       Channel A plays mix_buf[0] then chains to B.
       Channel B plays mix_buf[1] then chains back to A.
       Only A is manually started; B auto-fires via the chain.
       IRQ reconfigures the just-finished channel for its next use.       */
    dma_a = dma_claim_unused_channel(true);
    dma_b = dma_claim_unused_channel(true);
    txf   = (volatile uint32_t *)&pio->txf[sm];

    dc_a = dma_channel_get_default_config(dma_a);
    channel_config_set_transfer_data_size(&dc_a, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_a, true);
    channel_config_set_write_increment(&dc_a, false);
    channel_config_set_dreq(&dc_a, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&dc_a, dma_b);   // A → B

    dc_b = dma_channel_get_default_config(dma_b);
    channel_config_set_transfer_data_size(&dc_b, DMA_SIZE_32);
    channel_config_set_read_increment(&dc_b, true);
    channel_config_set_write_increment(&dc_b, false);
    channel_config_set_dreq(&dc_b, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&dc_b, dma_a);   // B → A

    // Pre-load both channels (don't trigger yet)
    dma_channel_configure(dma_a, &dc_a, (void *)txf, mix_buf[0], MIX_SAMPLES, false);
    dma_channel_configure(dma_b, &dc_b, (void *)txf, mix_buf[1], MIX_SAMPLES, false);

    // Enable IRQ for both channels
    dma_channel_set_irq0_enabled(dma_a, true);
    dma_channel_set_irq0_enabled(dma_b, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // wait for core1 to fill BOTH buffers before starting DMA
    while (!mix_ready[0] || !mix_ready[1]) tight_loop_contents();

    // Kick off only A; B will auto-start via chain when A completes
    dma_channel_start(dma_a);

    static bool prev_sfx_in[NUM_SFX_BUTTONS];
    static bool prev_music_in[NUM_MUSIC_BUTTONS];
    static absolute_time_t last_press_sfx[NUM_SFX_BUTTONS];
    static absolute_time_t last_press_music[NUM_MUSIC_BUTTONS];

    gpio_init(SHIFT_PIN);
    gpio_set_dir(SHIFT_PIN, GPIO_IN);
    gpio_pull_up(SHIFT_PIN);

    gpio_init(SHIFT_LED_PIN);
    gpio_set_dir(SHIFT_LED_PIN, GPIO_OUT);
    gpio_pull_up(SHIFT_LED_PIN);

    for (int s = 0; s < NUM_SFX_BUTTONS; s++) {
        prev_sfx_in[s] = false;
        last_press_sfx[s] = get_absolute_time();
        uint8_t p = sfx_button_pin[s];
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        gpio_pull_up(p);
    }
    for (int m = 0; m < NUM_MUSIC_BUTTONS; m++) {
        prev_music_in[m] = false;
        last_press_music[m] = get_absolute_time();
        uint8_t p = music_button_pin[m];
        gpio_init(p);
        gpio_set_dir(p, GPIO_IN);
        gpio_pull_up(p);
    }

    printf("Ready. GP%u=shift, %u SFX + %u music (active low).\n",
           SHIFT_PIN, NUM_SFX_BUTTONS, NUM_MUSIC_BUTTONS);
    printf("Dial GP%u: master volume (USB logs when the knob moves).\n", VOLUME_PIN);
    printf("Speed pot GP%u: music speed 0.5x..1.5x (snaps to 1.0x).\n", SPEED_PIN);
    printf("Vibrato pot GP%u: SFX vibrato depth (0 = off).\n", VIBRATO_PIN);

    typedef struct {
        uint32_t delta_ms;
        uint8_t  track_id;
    } record_ev_t;

    typedef enum { REC_IDLE = 0, REC_RECORDING = 1, REC_PLAYING = 2 } rec_state_t;

    #define RECORD_MAX_EVENTS 96u

    typedef struct {
        uint8_t pin;
        uint8_t led_pin;

        bool record_prev_in;
        rec_state_t state;

        record_ev_t events[RECORD_MAX_EVENTS];
        uint32_t event_count;

        uint32_t record_start_ms;
        uint32_t record_last_ev_ms;
        uint32_t snippet_len_ms;

        // Playback scheduler state
        uint32_t playback_cycle_start_ms;
        uint32_t playback_idx;
        uint32_t playback_next_due_ms;
    } recorder_t;

    static recorder_t rec_a = {
        .pin = 0,      // GP0 record button
        .led_pin = 1,  // GP1 LED
    };

    static recorder_t rec_b = {
        .pin = 2,      // GP2 record button
        .led_pin = 3,  // GP3 LED
    };

    recorder_t *recorders[] = { &rec_a, &rec_b };
    const uint32_t recorder_count = (uint32_t)(sizeof(recorders) / sizeof(recorders[0]));

    for (uint32_t ri = 0; ri < recorder_count; ri++) {
        recorder_t *r = recorders[ri];
        r->record_prev_in = false;
        r->state = REC_IDLE;
        r->event_count = 0;
        r->record_start_ms = 0;
        r->record_last_ev_ms = 0;
        r->snippet_len_ms = 0;
        r->playback_cycle_start_ms = 0;
        r->playback_idx = 0;
        r->playback_next_due_ms = 0;

        if (r->pin != PIN_UNUSED) {
            gpio_init(r->pin);
            gpio_set_dir(r->pin, GPIO_IN);
            gpio_pull_up(r->pin);
        }

        if (r->led_pin != PIN_UNUSED) {
            gpio_init(r->led_pin);
            gpio_set_dir(r->led_pin, GPIO_OUT);
            gpio_put(r->led_pin, 0);
        }
    }

    while (true) {
        tud_task();

        if (letsReset) {
            reset_usb_boot(0, 0);
        }

        // Record button(s) + playback tick (two independent recorders).
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        for (uint32_t ri = 0; ri < recorder_count; ri++) {
            recorder_t *r = recorders[ri];
            if (r->pin == PIN_UNUSED) continue;

            bool record_in = !gpio_get(r->pin);

            // LED behavior:
            // - While holding record: solid ON
            // - While recorded loop is playing: pulse continuously
            // - Otherwise: OFF
            if (r->led_pin != PIN_UNUSED) {
                bool led_on = false;
                if (record_in) {
                    led_on = true;
                } else if (r->state == REC_PLAYING) {
                    // 1 Hz pulse (500ms on, 500ms off)
                    led_on = ((now_ms / 500u) & 1u) != 0u;
                }
                gpio_put(r->led_pin, led_on ? 1 : 0);
            }

            // Down edge
            if (record_in && !r->record_prev_in) {
                if (r->state == REC_PLAYING) {
                    r->state = REC_IDLE;
                } else {
                    r->state = REC_RECORDING;
                    r->event_count = 0;
                    r->record_start_ms = now_ms;
                    r->record_last_ev_ms = now_ms;
                    r->snippet_len_ms = 0;
                }
            }

            // Up edge
            if (!record_in && r->record_prev_in) {
                if (r->state == REC_RECORDING) {
                    if (r->event_count == 0) {
                        r->state = REC_IDLE;
                    } else {
                        r->snippet_len_ms = now_ms - r->record_start_ms;
                        if (r->snippet_len_ms < 1u) r->snippet_len_ms = 1u;
                        r->state = REC_PLAYING;
                        r->playback_cycle_start_ms = now_ms;
                        r->playback_idx = 0;
                        r->playback_next_due_ms = r->events[0].delta_ms;
                    }
                }
            }

            r->record_prev_in = record_in;

            // Playback tick
            if (r->state == REC_PLAYING && r->event_count > 0) {
                uint32_t elapsed = now_ms - r->playback_cycle_start_ms;

                while (r->playback_idx < r->event_count && elapsed >= r->playback_next_due_ms) {
                    (void)sfx_q_push(r->events[r->playback_idx].track_id);
                    r->playback_idx++;
                    if (r->playback_idx < r->event_count) {
                        r->playback_next_due_ms += r->events[r->playback_idx].delta_ms;
                    }
                }

                if (elapsed >= r->snippet_len_ms) {
                    r->playback_cycle_start_ms += r->snippet_len_ms;
                    r->playback_idx = 0;
                    r->playback_next_due_ms = r->events[0].delta_ms;
                }
            }
        }

        gpio_put(SHIFT_LED_PIN, !gpio_get(SHIFT_PIN) ? 1 : 0);

        for (int m = 0; m < NUM_MUSIC_BUTTONS; m++) {
            bool in = !gpio_get(music_button_pin[m]);
            if (in && !prev_music_in[m]) {
                absolute_time_t now = get_absolute_time();
                if (absolute_time_diff_us(last_press_music[m], now) > 50000) {
                    last_press_music[m] = now;
                    uint8_t r = !gpio_get(SHIFT_PIN) ? 1u : 0u;
                    uint8_t tid = music_track_id[r][m];
                    if (loop_playing && active_loop_track == tid) {
                        loop_playing = false;
                        printf("\nloop stop: %s\n", tracks[tid].path);
                    } else {
                        active_loop_track = tid;
                        loop_playing = true;
                        printf("\nloop start: %s\n", tracks[tid].path);
                    }
                }
            }
            prev_music_in[m] = in;
        }

        for (int s = 0; s < NUM_SFX_BUTTONS; s++) {
            bool in = !gpio_get(sfx_button_pin[s]);
            if (in && !prev_sfx_in[s]) {
                absolute_time_t now = get_absolute_time();
                if (absolute_time_diff_us(last_press_sfx[s], now) > 50000) {
                    last_press_sfx[s] = now;
                    uint8_t r = !gpio_get(SHIFT_PIN) ? 1u : 0u;
                    uint8_t tid = sfx_track_id[r][s];
                    (void)sfx_q_push(tid);
                    printf("\none-shot: %s\n", tracks[tid].path);

                    for (uint32_t ri = 0; ri < recorder_count; ri++) {
                        recorder_t *r = recorders[ri];
                        if (r->state != REC_RECORDING) continue;
                        if (r->event_count >= RECORD_MAX_EVENTS) continue;
                        uint32_t now2_ms = to_ms_since_boot(get_absolute_time());
                        uint32_t d = now2_ms - r->record_last_ev_ms;
                        r->record_last_ev_ms = now2_ms;
                        r->events[r->event_count].delta_ms = d;
                        r->events[r->event_count].track_id = tid;
                        r->event_count++;
                    }
                }
            }
            prev_sfx_in[s] = in;
        }

        /* Dials: 4× average each; EMA (~⅛ per 20 ms) updates master_gain_q16 and music_speed_q16. */
        static uint32_t last_dial_ms;
        static uint32_t last_logged_gain_q16 = UINT32_MAX;
        static uint32_t last_logged_speed_q16 = UINT32_MAX;
        static uint32_t last_logged_vib_q16 = UINT32_MAX;

        uint32_t ms = to_ms_since_boot(get_absolute_time());
        if (ms - last_dial_ms >= 20u) {
            last_dial_ms = ms;

            uint32_t acc = 0;
            for (int s = 0; s < 4; s++) {
                acc += adc_read();
            }
            uint16_t adc = (uint16_t)((acc + 2u) / 4u);

            uint32_t target_q16 = dial_adc_to_gain_q16(adc);
            {
                int32_t err = (int32_t)target_q16 - (int32_t)volume_smooth_q16;
                volume_smooth_q16 += (uint32_t)(err >> 3); /* ~⅛ toward target per poll */
            }
            master_gain_q16 = volume_smooth_q16;

            uint32_t g = volume_smooth_q16;
            if (last_logged_gain_q16 == UINT32_MAX) {
                printf("dial: adc %4u  gain %u/65536 (initial)\n", adc, g);
                last_logged_gain_q16 = g;
            } else {
                int32_t dg = (int32_t)g - (int32_t)last_logged_gain_q16;
                uint32_t adg = (uint32_t)(dg < 0 ? -dg : dg);
                if (adg >= DIAL_LOG_GAIN_MIN_DELTA) {
                    printf("dial: adc %4u  gain %u/65536  (gain Δ %+ld)\n",
                           adc, g, (long)dg);
                    last_logged_gain_q16 = g;
                }
            }

            // Speed pot on ADC1 (GPIO27)
            adc_select_input(1);
            acc = 0;
            for (int s = 0; s < 4; s++) {
                acc += adc_read();
            }
            uint16_t sp_adc = (uint16_t)((acc + 2u) / 4u);
            uint32_t sp_target = adc_to_speed_q16(sp_adc);
            {
                int32_t err = (int32_t)sp_target - (int32_t)speed_smooth_q16;
                speed_smooth_q16 += (uint32_t)(err >> 3);
            }
            music_speed_q16 = speed_smooth_q16;

            uint32_t sp = speed_smooth_q16;
            // TODO: log sp target too??
            if (last_logged_speed_q16 == UINT32_MAX) {
                printf("speed: adc %4u  speed %u/65536 (initial)\n", sp_adc, sp);
                last_logged_speed_q16 = sp;
            } else {
                int32_t ds = (int32_t)sp - (int32_t)last_logged_speed_q16;
                uint32_t ads = (uint32_t)(ds < 0 ? -ds : ds);
                if (ads >= 1024u) {
                    printf("speed: adc %4u  speed %u/65536  (Δ %+ld)\n",
                           sp_adc, sp, (long)ds);
                    last_logged_speed_q16 = sp;
                }
            }

            // Vibrato depth pot on ADC2 (GPIO28) - affects SFX only
            adc_select_input(2);
            acc = 0;
            for (int s = 0; s < 4; s++) {
                acc += adc_read();
            }
            uint16_t tr_adc = (uint16_t)((acc + 2u) / 4u);
            uint32_t tr_target = adc_to_unit_q16(tr_adc);
            {
                int32_t err = (int32_t)tr_target - (int32_t)vibrato_depth_smooth_q16;
                vibrato_depth_smooth_q16 += (uint32_t)(err >> 3);
            }
            sfx_vibrato_depth_q16 = vibrato_depth_smooth_q16;

            uint32_t tr = vibrato_depth_smooth_q16;
            if (last_logged_vib_q16 == UINT32_MAX) {
                printf("vib:   adc %4u  depth %u/65536 (initial)\n", tr_adc, tr);
                last_logged_vib_q16 = tr;
            } else {
                int32_t dt = (int32_t)tr - (int32_t)last_logged_vib_q16;
                uint32_t adt = (uint32_t)(dt < 0 ? -dt : dt);
                if (adt >= 1024u) {
                    printf("vib:   adc %4u  depth %u/65536  (Δ %+ld)\n",
                           tr_adc, tr, (long)dt);
                    last_logged_vib_q16 = tr;
                }
            }

            // Restore ADC0 selection for next volume read
            adc_select_input(0);
        }

        /* Log underruns once per second */
        static absolute_time_t next_log;
        if (time_reached(next_log)) {
            if (mix_underruns) {
                printf("[underrun] %lu\n", mix_underruns);
                mix_underruns = 0;
            }
            if (sfx_q_drops) {
                printf("[sfx_q_drops] %lu\n", sfx_q_drops);
                sfx_q_drops = 0;
            }
            next_log = make_timeout_time_ms(1000);
        }
    }
}