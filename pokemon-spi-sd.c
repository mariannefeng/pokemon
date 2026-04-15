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
#include "i2s_out.pio.h"
#include "ff.h"
#include "sd_card.h"
#include <math.h> 

/* ── Pins ──────────────────────────────────────────────────────────────── */
#define DOUT_PIN         9    // MAX98357A DIN
#define BCLK_PIN        10    // MAX98357A BCLK  (LRCLK = GP11, implicit as BCLK+1)
#define SAMPLE_RATE  48000

/* Set GPIO per track (active low). Use PIN_UNUSED until wired. */
#define PIN_UNUSED 255

typedef struct {
    const char *path;   /* FatFs path under mount "0:" */
    bool        is_loop; /* true: music-* style — one loop at a time; toggle off by pressing same again */
    uint8_t     pin;     /* GP number */
} audio_track_t;

/* Order matches your SD filenames; assign .pin when ready. */
static const audio_track_t g_tracks[] = {
    { "0:bulbasaur.wav",           false, 28 },
    { "0:charmander.wav",          false, 27 },
    { "0:healing.wav",             false, PIN_UNUSED },
    { "0:level-up.wav",            false, PIN_UNUSED },
    { "0:music-azalea-city.wav",   true,  14 },
    { "0:music-poke-center.wav",   true,  PIN_UNUSED },
    { "0:music-theme-fart.wav",    true,  PIN_UNUSED },
    { "0:music-theme.wav",         true,  15 },
    { "0:music-wild-battle.wav",   true,  PIN_UNUSED },
    { "0:pikachu.wav",             false, PIN_UNUSED },
    { "0:plink.wav",               false, PIN_UNUSED },
    { "0:squirtle.wav",            false, PIN_UNUSED },
};

#define NUM_TRACKS ((int)(sizeof(g_tracks) / sizeof(g_tracks[0])))

/* Core 0 → core 1: loop layer — at most one music-* file; 255 = none / silence. */
static volatile uint8_t  active_loop_track = 255;
static volatile bool     loop_playing      = false;

/* One-shot SFX: increment trigger after setting sfx_track_id; any new press restarts that one-shot. */
static volatile uint8_t  sfx_track_id   = 255;
static volatile uint32_t sfx_trigger    = 0;

/* ── DMA output buffer ───────────────────────────────────────────────────
   Each word: [left 16b | right 16b] (same mono sample on both channels).
   core1 fills these halves; DMA autonomously feeds the PIO TX FIFO.      */
#define MIX_SAMPLES (16 * 1024)

static uint32_t mix_buf[2][MIX_SAMPLES];  // zero-initialised = silence
static volatile bool     mix_ready[2];    // set by core1, cleared by DMA IRQ
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


// Plays four tones back to back: 
// 1. Low frequency, low volume
// 2. High frequency, high volume
// 3. Low frequency, high volume
// 4. High frequency, low volume
// static void core1_entry(void) {
//     // Signal core0 that we're ready (replaces the SD mount signal)
//     multicore_fifo_push_blocking(0xCAFE);

//     int fill = 0;

//     // Define tone test cases: {frequency [Hz], amplitude}
//     struct {
//         float freq;
//         float amplitude;
//         const char *desc;
//     } tones[] = {
//         { 200.0f,   1000.0f,  "Low freq, low vol" },   // Test 1
//         { 4000.0f, 30000.0f,  "High freq, high vol" }, // Test 2
//         { 200.0f,  30000.0f,  "Low freq, high vol" },  // Test 3
//         { 4000.0f,  1000.0f,  "High freq, low vol" },  // Test 4
//     };

//     const int num_tones = sizeof(tones) / sizeof(tones[0]);
//     const float two_pi = 2.0f * 3.14159265f;
//     const int tone_samples = SAMPLE_RATE * 2; // 2 seconds of each tone

//     while (true) {
//         for (int t = 0; t < num_tones; t++) {
//             float phase = 0.0f;
//             float phase_inc = two_pi * tones[t].freq / (float)SAMPLE_RATE;

//             int played = 0;
//             while (played < tone_samples) {
//                 if (mix_ready[fill]) { tight_loop_contents(); continue; }

//                 for (int i = 0; i < MIX_SAMPLES && played < tone_samples; i++, played++) {
//                     int16_t s = (int16_t)(sinf(phase) * tones[t].amplitude);
//                     mix_buf[fill][i] = ((uint32_t)(uint16_t)s << 16) | (uint16_t)s;
//                     phase += phase_inc;
//                     if (phase >= two_pi) phase -= two_pi;
//                 }

//                 __dmb();
//                 mix_ready[fill] = true;
//                 fill ^= 1;
//             }
//         }
//     }
// }

static int16_t music_raw[MIX_SAMPLES];
static int16_t sfx_raw[MIX_SAMPLES];

static void core1_entry(void) {
    printf("[c1] core1 started\n");

    FATFS fs;
    printf("[c1] attempting f_mount...\n");
    FRESULT fr = f_mount(&fs, "0:", 1);
    printf("[c1] f_mount result: %d\n", fr);

    if (fr != FR_OK) {
        multicore_fifo_push_blocking(0xDEAD);
        return;
    }
    printf("[c1] SD mounted\n");
    multicore_fifo_push_blocking(0xCAFE);

    FIL      track_files[NUM_TRACKS];
    uint32_t track_starts[NUM_TRACKS];
    bool     track_ok[NUM_TRACKS];

    for (int t = 0; t < NUM_TRACKS; t++) {
        track_ok[t] = false;
        track_starts[t] = 0;
        if (f_open(&track_files[t], g_tracks[t].path, FA_READ) == FR_OK) {
            track_starts[t] = wav_data_offset(&track_files[t]);
            track_ok[t] = true;
        } else {
            printf("[c1] cannot open %s\n", g_tracks[t].path);
        }
    }

    int      fill = 0;

    bool     was_loop_playing = false;
    uint8_t  last_loop_track  = 255;
    bool     sfx_active       = false;
    uint32_t last_sfx_trig    = 0;

    while (true) {
        if (mix_ready[fill]) { tight_loop_contents(); continue; }

        bool     lp = loop_playing;
        uint8_t  lt = active_loop_track;

        if (lp && (!was_loop_playing || lt != last_loop_track)) {
            if (lt < (uint8_t)NUM_TRACKS && track_ok[lt] && g_tracks[lt].is_loop) {
                f_lseek(&track_files[lt], track_starts[lt]);
            }
        }
        was_loop_playing = lp;
        last_loop_track  = lt;

        uint32_t trig = sfx_trigger;
        if (trig != last_sfx_trig) {
            last_sfx_trig = trig;
            uint8_t sid = sfx_track_id;
            if (sid < (uint8_t)NUM_TRACKS && track_ok[sid] && !g_tracks[sid].is_loop) {
                f_lseek(&track_files[sid], track_starts[sid]);
                sfx_active = true;
            } else {
                sfx_active = false;
            }
        }

        /* Layer 1: looped music (silence when off or missing file) */
        if (lp && lt < (uint8_t)NUM_TRACKS && track_ok[lt] && g_tracks[lt].is_loop) {
            UINT br = 0;
            f_read(&track_files[lt], music_raw, MIX_SAMPLES * sizeof(int16_t), &br);
            int samples_read = (int)(br / sizeof(int16_t));

            if (samples_read < MIX_SAMPLES) {
                memset(music_raw + samples_read, 0,
                       (MIX_SAMPLES - samples_read) * sizeof(int16_t));
                f_lseek(&track_files[lt], track_starts[lt]);
            }
        } else {
            memset(music_raw, 0, MIX_SAMPLES * sizeof(int16_t));
        }

        /* Layer 2: one-shot SFX; a new trigger seeks to start and replaces the current one-shot */
        if (sfx_active) {
            UINT br    = 0;
            uint8_t sid = sfx_track_id;
            if (sid < (uint8_t)NUM_TRACKS && track_ok[sid] && !g_tracks[sid].is_loop) {
                f_read(&track_files[sid], sfx_raw, MIX_SAMPLES * sizeof(int16_t), &br);
            } else {
                br = 0;
            }
            int samples_read = (int)(br / sizeof(int16_t));

            if (br == 0) {
                memset(sfx_raw, 0, MIX_SAMPLES * sizeof(int16_t));
                sfx_active = false;
            } else if (samples_read < MIX_SAMPLES) {
                memset(sfx_raw + samples_read, 0,
                       (MIX_SAMPLES - samples_read) * sizeof(int16_t));
                sfx_active = false;
            }
        } else {
            memset(sfx_raw, 0, MIX_SAMPLES * sizeof(int16_t));
        }

        /* Sum layers with clamp (allows loop + one-shots together) */
        for (int i = 0; i < MIX_SAMPLES; i++) {
            int32_t m = (int32_t)music_raw[i] + (int32_t)sfx_raw[i];
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
    printf("Pokemon SD player starting...\n");
    
    /* Launch SD reader on core1, wait for mount confirmation */
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

    static bool prev_in[NUM_TRACKS];
    static absolute_time_t last_press[NUM_TRACKS];

    for (int i = 0; i < NUM_TRACKS; i++) {
        prev_in[i] = false;
        last_press[i] = get_absolute_time();
        if (g_tracks[i].pin == PIN_UNUSED) {
            continue;
        }
        gpio_init(g_tracks[i].pin);
        gpio_set_dir(g_tracks[i].pin, GPIO_IN);
        gpio_pull_up(g_tracks[i].pin);
    }

    printf("Ready. Set g_tracks[].pin for each GP (active low). PIN_UNUSED skips a row.\n");

    for (;;) {
        for (int i = 0; i < NUM_TRACKS; i++) {
            if (g_tracks[i].pin == PIN_UNUSED) {
                continue;
            }
            uint8_t pin = g_tracks[i].pin;
            bool in = !gpio_get(pin);
            if (in && !prev_in[i]) {
                absolute_time_t now = get_absolute_time();
                if (absolute_time_diff_us(last_press[i], now) > 50000) {
                    last_press[i] = now;
                    if (g_tracks[i].is_loop) {
                        if (loop_playing && active_loop_track == (uint8_t)i) {
                            loop_playing = false;
                            printf("\nloop stop: %s\n", g_tracks[i].path);
                        } else {
                            active_loop_track = (uint8_t)i;
                            loop_playing = true;
                            printf("\nloop start: %s\n", g_tracks[i].path);
                        }
                    } else {
                        sfx_track_id = (uint8_t)i;
                        sfx_trigger++;
                        printf("\none-shot: %s\n", g_tracks[i].path);
                    }
                }
            }
            prev_in[i] = in;
        }

        /* Log underruns once per second */
        static absolute_time_t next_log;
        if (time_reached(next_log)) {
            if (mix_underruns) {
                printf("[underrun] %lu\n", mix_underruns);
                mix_underruns = 0;
            }
            next_log = make_timeout_time_ms(1000);
        }
    }
}


// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "pico/stdio_usb.h"
// #include "f_util.h"
// #include "ff.h"
// #include "hw_config.h"

// int main(void) {
//     stdio_init_all();
//     while (!stdio_usb_connected()) sleep_ms(100);
//     printf("SD card test starting...\n");

//     sleep_ms(3000);
//     FATFS fs;
//     FRESULT fr = f_mount(&fs, "", 1);
//     if (fr != FR_OK) {
//         printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
//         // printf("No filesystem found, formatting...\n");
    
//         // BYTE work[FF_MAX_SS];  // work buffer (512 bytes typically)
//         // fr = f_mkfs("", NULL, work, sizeof(work));  // NULL opts = default FAT
//         // if (fr != FR_OK) {
//         //     printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
//         //     for (;;) tight_loop_contents();
//         // }
//         // printf("Formatted successfully, remounting...\n");
//         // fr = f_mount(&fs, "", 1);

//         for (;;) tight_loop_contents();
//     }
//     printf("SD card mounted successfully!\n");

//     // Print card size
//     FATFS *fsp;
//     DWORD free_clusters;
//     fr = f_getfree("", &free_clusters, &fsp);
//     if (fr == FR_OK) {
//         DWORD total_sectors = (fsp->n_fatent - 2) * fsp->csize;
//         DWORD free_sectors  = free_clusters * fsp->csize;
//         printf("Total: %lu MB\n", total_sectors / 2 / 1024);
//         printf("Free:  %lu MB\n", free_sectors  / 2 / 1024);
//     }

//     // Write a test file
//     FIL fil;
//     fr = f_open(&fil, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
//     if (fr == FR_OK) {
//         f_printf(&fil, "Hello from Pico!\n");
//         f_close(&fil);
//         printf("Wrote test.txt\n");
//     } else {
//         printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
//     }

//     f_unmount("");
//     printf("Done.\n");

//     for (;;) tight_loop_contents();
// }