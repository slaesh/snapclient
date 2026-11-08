/**
 *
 */

#include <stdint.h>
#include <string.h>
#include <sys/time.h>

#include "driver/i2s_common.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// #include "lwip/stats.h"

#include "esp_log.h"
#include "esp_timer.h"
#if CONFIG_SOC_WIFI_SUPPORTED
#include "esp_wifi.h"
#endif
#include "soc/rtc.h"

#if SOC_I2S_SUPPORTS_APLL
#include "clk_ctrl_os.h"
#endif

#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#include <math.h>

#include "MedianFilter.h"
#include "TimeFilter.h"
#include "driver/gptimer.h"
#include "driver/i2s_std.h"
#include "player.h"
#include "snapcast.h"

#define USE_SAMPLE_INSERTION CONFIG_USE_SAMPLE_INSERTION

#define SYNC_TASK_PRIORITY 20
#define SYNC_TASK_CORE_ID 1  // tskNO_AFFINITY

static const char *TAG = "PLAYER";

#if USE_SAMPLE_INSERTION

#if CONFIG_PM_ENABLE
esp_pm_lock_handle_t player_pm_lock_handle = NULL;
#endif

#define INSERT_SAMPLES \
  1  //!< currently only allowed to be 1 or sync algorithm will break

const uint32_t SHORT_OFFSET = 15000;
const uint32_t MINI_OFFSET = 10000;

#else
const uint32_t SHORT_OFFSET = 2;
const uint32_t MINI_OFFSET = 1;
#endif

/**
 * @brief Pre define APLL parameters, save compute time. They are calculated in
 * player_setup_i2s() | bits_per_sample | rate | sdm0 | sdm1 | sdm2 | odir
 *
 * apll_freq = xtal_freq * (4 + sdm2 + sdm1/256 + sdm0/65536)/((o_div + 2) * 2)
 * I2S bit clock is (apll_freq / 16)
 */
static uint32_t apll_normal_predefine[6] = {0, 0, 0, 0, 0, 0};
static uint32_t apll_corr_predefine[][6] = {{0, 0, 0, 0, 0, 0},
                                            {0, 0, 0, 0, 0, 0}};

static SemaphoreHandle_t latencyBufSemaphoreHandle = NULL;
static SemaphoreHandle_t latencyBufFullSemaphoreHandle = NULL;

static gptimer_handle_t gptimer = NULL;

#if USE_TIMEFILTER
static sTimeFilter_t latencyTimeFilter;

static double latencyToServer = 0;
static double latencyDrift = 0;
static int64_t latencyLastUpdate = 0;
#else
static sMedianFilter_t latencyMedianFilter;
static sMedianNode_t latencyMedianLong[LATENCY_MEDIAN_FILTER_LEN];

static int64_t latencyToServer = 0;
#endif

static sMedianFilter_t shortMedianFilter;
static sMedianNode_t shortMedianBuffer[SHORT_BUFFER_LEN];

static sMedianFilter_t miniMedianFilter;
static sMedianNode_t miniMedianBuffer[MINI_BUFFER_LEN];

static int8_t currentDir = 0;  //!< current apll direction, see apll_adjust()

static QueueHandle_t pcmChkQHdl = NULL;

static TaskHandle_t playerTaskHandle = NULL;

static QueueHandle_t snapcastSettingQueueHandle = NULL;

static uint32_t i2sDmaBufCnt;
static uint32_t i2sDmaBufMaxLen;

static SemaphoreHandle_t snapcastSettingsMux = NULL;
static snapcastSetting_t currentSnapcastSetting;

static void tg0_timer_init(void);
static void tg0_timer_deinit(void);

static bool gpTimerRunning = false;

static void player_task(void *pvParameters);

bool gotSettings = false;
bool playerstarted = false;

extern void audio_set_mute(bool mute);
extern void audio_dac_enable(bool enabled);

static i2s_chan_handle_t tx_chan = NULL;  // I2S tx channel handler
static bool i2sEnabled = false;

static volatile bool s_beep_active = false;
static volatile bool s_beep_idle_running = false;
static volatile bool s_player_is_playing = false;
static int s_beep_channel = 0; // 0=both/mono, 1=left, 2=right
static uint32_t s_beep_remaining_samples = 0;
static uint32_t s_beep_total_samples = 0;
static float s_beep_phase = 0.0f;
static float s_beep_phase_step = 0.0f;
static int s_beep_fade_samples = 0;

i2s_std_gpio_config_t pin_config0;
i2s_port_t i2sNum;


/**
 *
 */
esp_err_t my_i2s_channel_disable(i2s_chan_handle_t handle) {
  if (tx_chan != NULL) {
    if (i2sEnabled == true) {
      s_player_is_playing = false;
      /* Safely stop DAC before cutting I2S clocks to prevent pops/screeches */
      audio_dac_enable(false);
      i2sEnabled = false;

      return i2s_channel_disable(handle);
    }
  }

  return ESP_OK;
}

/**
 *
 */
esp_err_t my_i2s_channel_enable(i2s_chan_handle_t handle) {
  if (tx_chan != NULL) {
    if (i2sEnabled == false) {
      i2sEnabled = true;

      return i2s_channel_enable(handle);
    }
  }

  return ESP_OK;
}

#if CONFIG_DAC_TAS5805M
static int32_t i2s_expand_buf[256];

/* Sub-audible triangular dither generator at 16-bit LSB level (-96 dBFS).
 * In 32-bit slot width, 16-bit audio occupies bits [31:16].
 * Pure digital silence (0x00000000) causes the TAS58xx internal zero-detect circuit
 * to abruptly collapse PWM modulation with an audible "plop" during silent song passages.
 * Adding +/-1 LSB of zero-mean dither directly to bit 16 keeps Class-D PWM switching active,
 * perfectly smooth and centered, while being completely inaudible.
 */
static inline int32_t player_expand_sample(int16_t sample) {
  static uint32_t dither_lfsr = 0x12345678;
  dither_lfsr = (dither_lfsr >> 1) ^ (-(dither_lfsr & 1u) & 0xD0000001u);
  int32_t r1 = (int32_t)(dither_lfsr & 1u);
  dither_lfsr = (dither_lfsr >> 1) ^ (-(dither_lfsr & 1u) & 0xD0000001u);
  int32_t r2 = (int32_t)(dither_lfsr & 1u);
  int32_t dither = (r1 - r2) << 16;

  int32_t s32 = ((int32_t)sample) << 16;
  if (sample == INT16_MAX && dither > 0) dither = 0;
  if (sample == INT16_MIN && dither < 0) dither = 0;
  return s32 + dither;
}

/* Expand 16-bit signed PCM samples to 32-bit S32_LE (left-shifted by 16 bits, MSB-aligned)
 * so the ESP32 I2S peripheral outputs true 32-bit stereo frames (BCLK = 64 * fs = 2.8224 MHz @ 44.1k),
 * matching the Linux TAS58xx driver (SNDRV_PCM_FMTBIT_S32_LE) 1:1.
 * Reports *bytes_loaded in 16-bit input bytes so caller frame accounting stays unchanged.
 */
static esp_err_t player_i2s_preload_data(i2s_chan_handle_t tx_handle, const void *src, size_t size, size_t *bytes_loaded) {
  const int16_t *src16 = (const int16_t *)src;
  size_t total_in_loaded = 0;
  size_t rem_samples = size / sizeof(int16_t);

  while (rem_samples > 0) {
    size_t chunk_samples = (rem_samples > 256) ? 256 : rem_samples;
    for (size_t i = 0; i < chunk_samples; i++) {
      i2s_expand_buf[i] = player_expand_sample(src16[i]);
    }
    size_t out_loaded = 0;
    esp_err_t err = i2s_channel_preload_data(tx_handle, i2s_expand_buf, chunk_samples * sizeof(int32_t), &out_loaded);
    size_t samples_loaded = out_loaded / sizeof(int32_t);
    total_in_loaded += samples_loaded * sizeof(int16_t);
    src16 += samples_loaded;
    rem_samples -= samples_loaded;
    if (err != ESP_OK || samples_loaded < chunk_samples) {
      if (bytes_loaded) *bytes_loaded = total_in_loaded;
      return err;
    }
  }
  if (bytes_loaded) *bytes_loaded = total_in_loaded;
  return ESP_OK;
}

static esp_err_t player_i2s_write(i2s_chan_handle_t tx_handle, const void *src, size_t size, size_t *bytes_written, uint32_t timeout_ms) {
  const int16_t *src16 = (const int16_t *)src;
  size_t total_in_written = 0;
  size_t rem_samples = size / sizeof(int16_t);

  while (rem_samples > 0) {
    size_t chunk_samples = (rem_samples > 256) ? 256 : rem_samples;
    for (size_t i = 0; i < chunk_samples; i++) {
      int16_t sample = src16[i];
      if (s_beep_active) {
        bool is_mono = (currentSnapcastSetting.ch == 1);
        bool is_right = is_mono ? false : (i % 2 == 1);
        bool is_left = is_mono ? true : (i % 2 == 0);
        bool apply_beep = (s_beep_channel == 0) ||
                          (s_beep_channel == 1 && is_left) ||
                          (s_beep_channel == 2 && is_right);
        if (apply_beep) {
          uint32_t curr = s_beep_total_samples - s_beep_remaining_samples;
          float env = 1.0f;
          if (curr < (uint32_t)s_beep_fade_samples) {
            env = 0.5f * (1.0f - cosf((float)curr * (float)M_PI / (float)s_beep_fade_samples));
          } else if (s_beep_remaining_samples < (uint32_t)s_beep_fade_samples) {
            env = 0.5f * (1.0f - cosf((float)s_beep_remaining_samples * (float)M_PI / (float)s_beep_fade_samples));
          }
          int16_t tone = (int16_t)(sinf(s_beep_phase) * 12000.0f * env);
          int32_t mixed = (int32_t)sample + tone;
          if (mixed > 32767) mixed = 32767;
          else if (mixed < -32768) mixed = -32768;
          sample = (int16_t)mixed;
        }
        if (is_mono || is_right) {
          s_beep_phase += s_beep_phase_step;
          if (s_beep_phase >= 2.0f * (float)M_PI) s_beep_phase -= 2.0f * (float)M_PI;
          if (s_beep_remaining_samples > 0) {
            s_beep_remaining_samples--;
          } else {
            s_beep_active = false;
          }
        }
      }
      i2s_expand_buf[i] = player_expand_sample(sample);
    }
    size_t out_written = 0;
    esp_err_t err = i2s_channel_write(tx_handle, i2s_expand_buf, chunk_samples * sizeof(int32_t), &out_written, timeout_ms);
    size_t samples_written = out_written / sizeof(int32_t);
    total_in_written += samples_written * sizeof(int16_t);
    src16 += samples_written;
    rem_samples -= samples_written;
    if (err != ESP_OK || samples_written < chunk_samples) {
      if (bytes_written) *bytes_written = total_in_written;
      return err;
    }
  }
  if (bytes_written) *bytes_written = total_in_written;
  return ESP_OK;
}

static void player_beep_idle_task(void *pv) {
  int channel = (int)(intptr_t)pv;
  int sr = (currentSnapcastSetting.sr > 0) ? currentSnapcastSetting.sr : 44100;
  int duration_ms = 1000;
  int total_frames = sr * duration_ms / 1000;
  int fade_frames = sr * 25 / 1000;
  float phase_step = 2.0f * (float)M_PI * 880.0f / (float)sr;
  float phase = 0.0f;

  ESP_LOGI(TAG, "Playing idle test beep: channel=%d, sr=%d, frames=%d", channel, sr, total_frames);

  my_i2s_channel_enable(tx_chan);
  audio_dac_enable(true);
  vTaskDelay(pdMS_TO_TICKS(15));
  audio_set_mute(false);

  int16_t frame_buf[256 * 2];
  int remaining_frames = total_frames;
  int current_frame = 0;

  while (remaining_frames > 0 && !s_player_is_playing) {
    int chunk_frames = (remaining_frames > 256) ? 256 : remaining_frames;
    for (int i = 0; i < chunk_frames; i++) {
      float env = 1.0f;
      if (current_frame < fade_frames) {
        env = 0.5f * (1.0f - cosf((float)current_frame * (float)M_PI / (float)fade_frames));
      } else if (current_frame > total_frames - fade_frames) {
        env = 0.5f * (1.0f - cosf((float)(total_frames - current_frame) * (float)M_PI / (float)fade_frames));
      }
      int16_t tone = (int16_t)(sinf(phase) * 12000.0f * env);
      phase += phase_step;
      if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;

      int16_t left = (channel == 0 || channel == 1) ? tone : 0;
      int16_t right = (channel == 0 || channel == 2) ? tone : 0;
      frame_buf[i * 2] = left;
      frame_buf[i * 2 + 1] = right;
      current_frame++;
    }
    remaining_frames -= chunk_frames;
    size_t written = 0;
    player_i2s_write(tx_chan, frame_buf, chunk_frames * 2 * sizeof(int16_t), &written, pdMS_TO_TICKS(200));
  }

  if (!s_player_is_playing) {
    vTaskDelay(pdMS_TO_TICKS(60));
    audio_set_mute(true);
    my_i2s_channel_disable(tx_chan);
  }

  s_beep_idle_running = false;
  vTaskDelete(NULL);
}

esp_err_t player_trigger_beep(int channel, int duration_ms) {
  if (s_beep_idle_running) {
    return ESP_ERR_INVALID_STATE;
  }
  if (duration_ms <= 0) {
    duration_ms = 1000;
  }
  int sr = (currentSnapcastSetting.sr > 0) ? currentSnapcastSetting.sr : 44100;

  if (s_player_is_playing && i2sEnabled) {
    s_beep_channel = channel;
    s_beep_total_samples = (uint32_t)sr * (uint32_t)duration_ms / 1000;
    s_beep_remaining_samples = s_beep_total_samples;
    s_beep_fade_samples = sr * 25 / 1000;
    s_beep_phase = 0.0f;
    s_beep_phase_step = 2.0f * (float)M_PI * 880.0f / (float)sr;
    s_beep_active = true;
    ESP_LOGI(TAG, "Triggered on-stream beep: channel=%d, dur=%dms", channel, duration_ms);
    return ESP_OK;
  } else {
    s_beep_idle_running = true;
    BaseType_t ret = xTaskCreate(player_beep_idle_task, "beep_idle", 3072, (void *)(intptr_t)channel, 5, NULL);
    if (ret != pdPASS) {
      s_beep_idle_running = false;
      return ESP_FAIL;
    }
    return ESP_OK;
  }
}
#else
#define player_i2s_preload_data i2s_channel_preload_data
#define player_i2s_write i2s_channel_write
#endif

/**
 * This is a dirty hack to ensure smooth audio output without pops/clicks.
 * It was originally developed to suppress es8388 noise between channel
 * initialization and enabling. But the root cause of the noise was eliminated
 * in es8388 driver. However: this function is still useful to suppress
 * pops/clicks, typically when "pcm chunk queue not created" happens. It streams
 * silence to I2S before starting actual audio playback. It's unclear, why this
 * works, but it does.
 */
static void ensure_noiseless(i2s_chan_handle_t tx) {
  // We only need to prime I2S once
  static bool i2s_primed = false;
  if (i2s_primed) {
    return;
  }
  i2s_primed = true;

  ESP_LOGI(TAG, "Priming I2S with silence to avoid noise");
  // enabling I2S channel stops noise
  my_i2s_channel_enable(tx);

  // This could be adjusted size according to sample rate, bits, channels
  size_t silence_size = 44100 * 2 * 2 / 10;
  char *silence_buf = calloc(1, silence_size);

  if (silence_buf) {
    size_t bytes_written;

    // Write silence
    // While this is not required to stop noise, it somehow prevents pops later.
    player_i2s_write(tx, silence_buf, silence_size, &bytes_written,
                     pdMS_TO_TICKS(200));

    // Wait for DMA to flush
    vTaskDelay(pdMS_TO_TICKS(150));

    free(silence_buf);
    ESP_LOGI(TAG, "Audio path primed with %d bytes of silence", bytes_written);
  }

  // Disable I2S channel again. Leaving it enabled may cause pops later.
  my_i2s_channel_disable(tx);
}

/**
 *
 */
static esp_err_t player_setup_i2s(snapcastSetting_t *setting) {
  // ensure save setting
  int32_t sr = setting->sr;
  if (sr == 0) {
    sr = 44100;
  }

  // ensure save setting
  int bits = setting->bits;
  if (bits == 0) {
    bits = I2S_DATA_BIT_WIDTH_16BIT;
  }

  // ensure save setting
  uint32_t chkInFrames = setting->chkInFrames;
  if (chkInFrames == 0) {
    chkInFrames = 1152;
  }

#if USE_SAMPLE_INSERTION
#if CONFIG_DAC_TAS5805M
  /* With 32-bit stereo frames (8 bytes/frame), ESP32 DMA descriptors max out at
   * 4092 / 8 = 511 frames per descriptor. Use 4 * 511 = 2044 frames (~2 * 1023).
   */
  i2sDmaBufCnt = 4;
  i2sDmaBufMaxLen = 511;
#else
  i2sDmaBufCnt = 2;
  // OPUS has a minimum frame size of 120
  // with DMA buffer set to this value sync algorithm
  // works for all decoders. We set it to 100 so
  // there will be free space for sample stuffing in each round
  i2sDmaBufMaxLen = 1023;
#endif
#else
  int fi2s_clk;
#if CONFIG_DAC_TAS5805M
  const int __dmaBufMaxLen = 511;
#else
  const int __dmaBufMaxLen = 1023;
#endif
  int __dmaBufCnt;
  int __dmaBufLen;

  __dmaBufCnt = 1;
  __dmaBufLen = chkInFrames;
  while ((__dmaBufLen >= __dmaBufMaxLen) || (__dmaBufCnt <= 1)) {
    if ((__dmaBufLen % 2) == 0) {
      __dmaBufCnt *= 2;
      __dmaBufLen /= 2;
    } else {
      ESP_LOGE(TAG,
               "player_setup_i2s: Can't setup i2s with this configuration");

      return -1;
    }
  }

  i2sDmaBufCnt = __dmaBufCnt * CHNK_CTRL_CNT;
  i2sDmaBufMaxLen = __dmaBufLen;

  // check i2s_set_get_apll_freq() how it is done
  fi2s_clk =
      2 * sr * I2S_MCLK_MULTIPLE_256;  // setting->ch * setting->bits * m_scale;

  apll_normal_predefine[0] = bits;
  apll_normal_predefine[1] = sr;
  if (rtc_clk_apll_coeff_calc(
          fi2s_clk, &apll_normal_predefine[5], &apll_normal_predefine[2],
          &apll_normal_predefine[3], &apll_normal_predefine[4]) == 0) {
    ESP_LOGE(TAG, "ERROR, fi2s_clk");
  }

#define UPPER_SR_SCALER 1.0001
#define LOWER_SR_SCALER 0.9999

  apll_corr_predefine[0][0] = bits;
  apll_corr_predefine[0][1] = sr * UPPER_SR_SCALER;
  if (rtc_clk_apll_coeff_calc(
          fi2s_clk * UPPER_SR_SCALER, &apll_corr_predefine[0][5],
          &apll_corr_predefine[0][2], &apll_corr_predefine[0][3],
          &apll_corr_predefine[0][4]) == 0) {
    ESP_LOGE(TAG, "ERROR, fi2s_clk * %f", UPPER_SR_SCALER);
  }
  apll_corr_predefine[1][0] = bits;
  apll_corr_predefine[1][1] = sr * LOWER_SR_SCALER;
  if (rtc_clk_apll_coeff_calc(
          fi2s_clk * LOWER_SR_SCALER, &apll_corr_predefine[1][5],
          &apll_corr_predefine[1][2], &apll_corr_predefine[1][3],
          &apll_corr_predefine[1][4]) == 0) {
    ESP_LOGE(TAG, "ERROR, fi2s_clk * %f", LOWER_SR_SCALER);
  }
#endif

  if (tx_chan) {
    my_i2s_channel_disable(tx_chan);
    i2s_del_channel(tx_chan);
    tx_chan = NULL;
  }

  i2s_chan_config_t tx_chan_cfg = {
      .id = i2sNum,
      .role = I2S_ROLE_MASTER,
      .dma_desc_num = i2sDmaBufCnt,
      .dma_frame_num = i2sDmaBufMaxLen,
      .auto_clear = true,
  };
  ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

  i2s_std_clk_config_t i2s_clkcfg = I2S_STD_CLK_DEFAULT_CONFIG(sr);

#if SOC_I2S_SUPPORTS_APLL && !defined(CONFIG_ETH_RMII_CLK_OUTPUT)
  /* Use APLL when available and not occupied by Ethernet RMII 50MHz clock */
  i2s_clkcfg.clk_src = I2S_CLK_SRC_APLL;
#else
  /* Ethernet uses APLL at 50MHz; I2S must use standard PLL_F160M / APB clock */
  i2s_clkcfg.clk_src = I2S_CLK_SRC_DEFAULT;
#endif

  ESP_LOGI(TAG,
           "player_setup_i2s: dma_buf_len is %ld, dma_buf_count is %ld, sample "
           "rate: %ld, bits: %d, clk_src: %d, mclk_mult: %d",
           i2sDmaBufMaxLen, i2sDmaBufCnt, sr, bits, (int)i2s_clkcfg.clk_src, (int)i2s_clkcfg.mclk_multiple);

#if CONFIG_DAC_TAS5805M
  /* TAS58xx requires 32-bit I2S framing (SNDRV_PCM_FMTBIT_S32_LE, BCLK = 64 * fs = 3.072 MHz)
   * matching the Linux driver. Keep mclk_multiple = 256 (256 / 64 = 4, exact integer divider).
   */
  i2s_std_config_t tx_std_cfg = {
      .clk_cfg = i2s_clkcfg,
#if CONFIG_I2S_USE_MSB_FORMAT
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
#else
      .slot_cfg =
          I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
#endif
      .gpio_cfg = pin_config0,
  };
#else
  // Please set the mclk_multiple to I2S_MCLK_MULTIPLE_384
  // while using 24 bits data width Otherwise the sample rate
  // might be imprecise since the BCLK division is not a integer
  if (bits > I2S_DATA_BIT_WIDTH_16BIT) {
    i2s_clkcfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;
  }

  i2s_std_config_t tx_std_cfg = {
      .clk_cfg = i2s_clkcfg,
#if CONFIG_I2S_USE_MSB_FORMAT
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bits, I2S_SLOT_MODE_STEREO),
#else
      .slot_cfg =
          I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits, I2S_SLOT_MODE_STEREO),
#endif
      .gpio_cfg = pin_config0,
  };
#endif

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
#if !CONFIG_DAC_TAS5805M
  // This prevents pops/clicks on some I2S codecs like ES8388, but causes clock glitches on TAS58xx
  ensure_noiseless(tx_chan);
#endif

  // my_i2s_channel_enable(tx_chan);

  return 0;
}

/**
 *
 */
static int destroy_pcm_queue(QueueHandle_t *queueHandle) {
  int ret = pdPASS;
  pcm_chunk_message_t *chnk = NULL;

  if (*queueHandle == NULL) {
    ESP_LOGV(TAG, "no pcm chunk queue created?");
    ret = pdFAIL;
  } else {
    // free all allocated memory
    while (uxQueueMessagesWaiting(*queueHandle)) {
      ret = xQueueReceive(*queueHandle, &chnk, pdMS_TO_TICKS(2000));
      if (ret != pdFAIL) {
        if (chnk != NULL) {
          free_pcm_chunk(chnk);
        }
      }
      else {
        ESP_LOGE(TAG, "%s: can't get pcm chunk", __func__);
      }
    }

    // delete the queue
    vQueueDelete(*queueHandle);
    *queueHandle = NULL;

    ret = pdPASS;
  }

  return ret;
}

/**
 * ensure this is called after http_task was killed!
 */
int deinit_player(void) {
  int ret = 0;

  // must disable i2s before stopping player task or it will hang
  my_i2s_channel_disable(tx_chan);
  
  //wait max 10s for task to stop itself
  for(int i = 0; i< 100; i++) {
    if (playerstarted) {
      vTaskDelay(pdMS_TO_TICKS(100));
    } else {
      break;
    }
  }

  // stop the task f still running
  if (playerTaskHandle != NULL) {
    vTaskDelete(playerTaskHandle);
    playerTaskHandle = NULL;
  }
  
  if (tx_chan) {
    i2s_del_channel(tx_chan);
    tx_chan = NULL;
  }

  if (snapcastSettingsMux != NULL) {
    vSemaphoreDelete(snapcastSettingsMux);
    snapcastSettingsMux = NULL;
  }
  ret = destroy_pcm_queue(&pcmChkQHdl);

  if (latencyBufSemaphoreHandle != NULL) {
    vSemaphoreDelete(latencyBufSemaphoreHandle);
    latencyBufSemaphoreHandle = NULL;
  }

  if (latencyBufFullSemaphoreHandle != NULL) {
    vSemaphoreDelete(latencyBufFullSemaphoreHandle);
    latencyBufFullSemaphoreHandle = NULL;
  }


  tg0_timer_deinit();
  
#if CONFIG_PM_ENABLE
  if (player_pm_lock_handle) {
    esp_pm_lock_delete(player_pm_lock_handle);
    player_pm_lock_handle = NULL;
  }
#endif

  ESP_LOGI(TAG, "deinit player done");

  return ret;
}

/**
 *  call before http task creation!
 */
int init_player(i2s_std_gpio_config_t pin_config0_, i2s_port_t i2sNum_) {
  int ret = 0;

  deinit_player();

  pin_config0 = pin_config0_;
  i2sNum = i2sNum_;

  currentSnapcastSetting.buf_ms = 0;
  currentSnapcastSetting.chkInFrames = 0;
  currentSnapcastSetting.codec = NONE;
  currentSnapcastSetting.sr = 0;
  currentSnapcastSetting.ch = 0;
  currentSnapcastSetting.bits = 0;
  currentSnapcastSetting.muted = true;
  currentSnapcastSetting.volume = 0;

  if (snapcastSettingsMux == NULL) {
    snapcastSettingsMux = xSemaphoreCreateMutex();
    xSemaphoreGive(snapcastSettingsMux);
  }

  /**
  ret = player_setup_i2s(&currentSnapcastSetting);
  if (ret < 0) {
    ESP_LOGE(TAG, "player_setup_i2s failed: %d", ret);

    return -1;
  }*/

  // create semaphore for time diff buffer to server
  if (latencyBufSemaphoreHandle == NULL) {
    latencyBufSemaphoreHandle = xSemaphoreCreateMutex();
  }

  if (latencyBufFullSemaphoreHandle == NULL) {
    latencyBufFullSemaphoreHandle = xSemaphoreCreateBinary();
  }
  xSemaphoreTake(latencyBufFullSemaphoreHandle, 0);

  
#if USE_TIMEFILTER
  // init Kalmann time filter 
  TIMEFILTER_Init(&latencyTimeFilter, 0.01, 0.0, 1.001, 0.75, 100, 2.0);
#else
  // init diff buff median filter
  latencyMedianFilter.numNodes = LATENCY_MEDIAN_FILTER_LEN;
  latencyMedianFilter.medianBuffer = latencyMedianLong;
  reset_latency_buffer();
#endif
  shortMedianFilter.numNodes = SHORT_BUFFER_LEN;
  shortMedianFilter.medianBuffer = shortMedianBuffer;
  MEDIANFILTER_Init(&shortMedianFilter);

  miniMedianFilter.numNodes = MINI_BUFFER_LEN;
  miniMedianFilter.medianBuffer = miniMedianBuffer;
  MEDIANFILTER_Init(&miniMedianFilter);
  
  #if CONFIG_PM_ENABLE
  esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "player", &player_pm_lock_handle);
  #endif

  ESP_LOGI(TAG, "init player done");

  return 0;
}

/**
 * call to start the player task
 */
int start_player(snapcastSetting_t *setting) {
    if (playerstarted){
        return -1;
    }
    playerstarted = true;
  int ret = 0;

  ret = player_setup_i2s(setting);
  if (ret < 0) {
    ESP_LOGE(TAG, "player_setup_i2s failed: %d", ret);
    playerstarted = false;
    return -1;
  }

  tg0_timer_init();

#if CONFIG_PM_ENABLE
  ESP_LOGI(TAG, "reset Latency buffer");

  while(reset_latency_buffer()<0) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  esp_pm_lock_acquire(player_pm_lock_handle);
#endif
  
  // create message queue to inform task of changed settings
  snapcastSettingQueueHandle = xQueueCreate(1, sizeof(uint8_t));
  
  if (pcmChkQHdl == NULL) 
  {
    snapcastSetting_t scSet;
    memset(&scSet, 0, sizeof(snapcastSetting_t));
    player_get_snapcast_settings(&scSet);
    
    // ensure we don't have a divide by zero situation
    uint32_t chkInFrames = scSet.chkInFrames;
    if (chkInFrames == 0) {
      chkInFrames = 1152; // choose a good default for now
    }
    
    int entries = ceil(((float)scSet.sr / (float)chkInFrames) *
                        ((float)scSet.buf_ms / 1000));

    // some chunks are placed in DMA buffer
    // so we can save a little RAM here
    entries -= ((i2sDmaBufMaxLen * i2sDmaBufCnt) / chkInFrames);

    pcmChkQHdl = xQueueCreate(entries, sizeof(pcm_chunk_message_t *));

    ESP_LOGI(TAG, "created new queue with %d", entries);
  }

  ESP_LOGI(TAG, "Start player_task");

  xTaskCreatePinnedToCore(player_task, "player", 1024 * 3, NULL,
                          SYNC_TASK_PRIORITY, &playerTaskHandle,
                          SYNC_TASK_CORE_ID);


  ESP_LOGI(TAG, "start player done");

  return 0;
}

/**
 *
 */
int8_t player_set_snapcast_settings(snapcastSetting_t *setting) {
  int8_t ret = pdPASS;

  xSemaphoreTake(snapcastSettingsMux, portMAX_DELAY);

  memcpy(&currentSnapcastSetting, setting, sizeof(snapcastSetting_t));

  xSemaphoreGive(snapcastSettingsMux);

  return ret;
}

/**
 *
 */
int8_t player_get_snapcast_settings(snapcastSetting_t *setting) {
  int8_t ret = pdPASS;

  xSemaphoreTake(snapcastSettingsMux, portMAX_DELAY);

  memcpy(setting, &currentSnapcastSetting, sizeof(snapcastSetting_t));

  xSemaphoreGive(snapcastSettingsMux);

  return ret;
}

#if USE_TIMEFILTER
/**
 *
 */
int32_t player_latency_insert(int64_t newValue, int64_t max_error, int64_t time_added) {
  TIMEFILTER_Insert(&latencyTimeFilter, newValue, max_error, time_added);
  int64_t last_update_ = latencyTimeFilter.last_update_;
  double offset_ = latencyTimeFilter.offset_;
  double drift_ = latencyTimeFilter.use_drift_ ? latencyTimeFilter.drift_ : 0.0;
  if (xSemaphoreTake(latencyBufSemaphoreHandle, pdMS_TO_TICKS(0)) == pdTRUE) {
    if (TIMEFILTER_isFull(&latencyTimeFilter, LATENCY_TIME_FILTER_FULL)) {
      xSemaphoreGive(latencyBufFullSemaphoreHandle);
      //ESP_LOGI(TAG, "offset: %fus, diff: %lld", offset_, newValue - (uint64_t)offset_);
    }
    //else {
      // ESP_LOGI(TAG, "not full: offset: %fus, diff: %lld", offset_, newValue - (uint64_t)offset_);
    //}

    latencyToServer = offset_;
    latencyDrift = drift_;
    latencyLastUpdate = last_update_;
    //ESP_LOGI(TAG, "Timefilter: drift %f, max err %lldus, \n%lldus new val\n%fus offset", drift_, max_error, newValue, offset_);
    xSemaphoreGive(latencyBufSemaphoreHandle);
  } else {
    ESP_LOGW(TAG, "couldn't set latencyToServer = medianValue");
  }

  return 0;
}
#else
/**
 *
 */
int32_t player_latency_insert(int64_t newValue) {
  int64_t medianValue;

  medianValue = MEDIANFILTER_Insert(&latencyMedianFilter, newValue);
  if (xSemaphoreTake(latencyBufSemaphoreHandle, pdMS_TO_TICKS(0)) == pdTRUE) {
    if (MEDIANFILTER_isFull(&latencyMedianFilter, LATENCY_MEDIAN_FILTER_FULL)) {
      xSemaphoreGive(latencyBufFullSemaphoreHandle);

      //      ESP_LOGI(TAG, "(full) latency median: %lldus", medianValue);
    }
    //    else {
    //      ESP_LOGI(TAG, "(not full) latency median: %lldus", medianValue);
    //    }

    latencyToServer = medianValue;

    xSemaphoreGive(latencyBufSemaphoreHandle);
  } else {
    ESP_LOGW(TAG, "couldn't set latencyToServer = medianValue");
  }

  return 0;
}
#endif

/**
 *
 */
int32_t player_send_snapcast_setting(snapcastSetting_t *setting) {
  int ret;
  snapcastSetting_t curSet;
  uint8_t settingChanged = 1;

  ret = player_get_snapcast_settings(&curSet);

  if ((curSet.bits != setting->bits) || (curSet.buf_ms != setting->buf_ms) ||
      (curSet.ch != setting->ch) ||
      (curSet.chkInFrames != setting->chkInFrames) ||
      (curSet.codec != setting->codec) || (curSet.muted != setting->muted) ||
      (curSet.sr != setting->sr) || (curSet.volume != setting->volume) ||
      (curSet.cDacLat_ms != setting->cDacLat_ms)) {
    ret = player_set_snapcast_settings(setting);
    if (ret != pdPASS) {
      ESP_LOGE(TAG,
               "player_send_snapcast_setting: couldn't change "
               "snapcast setting");
    }

    // check if it is only volume / mute related setting, which is handled by
    // http_get_task()
    // if ((((curSet.muted != setting->muted) ||
          //(curSet.volume != setting->volume)) &&
         //((curSet.bits == setting->bits) &&
          //(curSet.buf_ms == setting->buf_ms) && (curSet.ch == setting->ch) &&
          //(curSet.chkInFrames == setting->chkInFrames) &&
          //(curSet.codec == setting->codec) && (curSet.sr == setting->sr) &&
          //(curSet.cDacLat_ms == setting->cDacLat_ms))) == false) {
      // notify needed
    if ((playerTaskHandle != NULL) && (snapcastSettingQueueHandle != NULL)) {
      ret = xQueueOverwrite(snapcastSettingQueueHandle, &settingChanged);
      if (ret != pdPASS) {
        ESP_LOGE(TAG,
                  "player_send_snapcast_setting: couldn't notify "
                  "snapcast setting");
      } else {
                  ESP_LOGI(TAG,
                  "got settings and notified player_task");
      }
    }
  }

  if (!gotSettings && (setting->bits > 0) && ( setting->buf_ms > 0) && (setting->ch > 0) && 
      (setting->chkInFrames > 0) && (setting->sr > 0)) {
    gotSettings = true;
  }

  return pdPASS;
}

#if USE_TIMEFILTER
/**
 *
 */
int32_t reset_latency_buffer(void) {
  // init diff buff median filter
  TIMEFILTER_Reset(&latencyTimeFilter);

  if (latencyBufSemaphoreHandle == NULL) {
    ESP_LOGE(TAG, "reset_diff_buffer: latencyBufSemaphoreHandle == NULL");

    return -2;
  }
  xSemaphoreTake(latencyBufFullSemaphoreHandle, pdMS_TO_TICKS(10));
  if (xSemaphoreTake(latencyBufSemaphoreHandle, pdMS_TO_TICKS(100)) == pdTRUE) {
    latencyToServer = 0;
    latencyDrift = 0;
    latencyLastUpdate = 0;

    xSemaphoreGive(latencyBufSemaphoreHandle);
  } else {
    ESP_LOGW(TAG, "reset_diff_buffer: can't take semaphore");

    return -1;
  }

  return 0;
}

/**
 *
 */
int32_t latency_buffer_full(bool *is_full) {
  *is_full = TIMEFILTER_isFull(&latencyTimeFilter, LATENCY_TIME_FILTER_FULL);
  return 0;
}

/**
 *
 */
int32_t get_diff_to_server(int64_t *tDiff, int64_t now) {
  static double lastDiff = 0;
  static double lastDrift = 0;
  static int64_t lastLastUpdate = 0;

  if (latencyBufSemaphoreHandle == NULL) {
    ESP_LOGE(TAG, "get_diff_to_server: latencyBufSemaphoreHandle == NULL");

    return -2;
  }

  double dt;
  int64_t offset;
  if (xSemaphoreTake(latencyBufSemaphoreHandle, 0) == pdFALSE) {
    dt = now - lastLastUpdate;
    offset = round(lastDiff + lastDrift * dt);
    *tDiff = offset;

     // ESP_LOGW(TAG,
     //         "get_diff_to_server: can't take semaphore. Old diff retrieved");

    return -1;
  }

  dt = now - latencyLastUpdate;
  offset = round(latencyToServer + latencyDrift * dt);

  *tDiff = offset;
  lastLastUpdate = latencyLastUpdate;
  lastDrift = latencyDrift;
  lastDiff = latencyToServer;  // store value, so we can return a value if
                               // semaphore couldn't be taken

  xSemaphoreGive(latencyBufSemaphoreHandle);

  return 0;
}

#else

/**
 *
 */
int32_t reset_latency_buffer(void) {
  xSemaphoreTake(latencyBufFullSemaphoreHandle, pdMS_TO_TICKS(10));
  
  // init diff buff median filter
  if (MEDIANFILTER_Init(&latencyMedianFilter) < 0) {
    ESP_LOGE(TAG, "reset_diff_buffer: couldn't init median filter long. STOP");

    return -2;
  }

  if (latencyBufSemaphoreHandle == NULL) {
    ESP_LOGE(TAG, "reset_diff_buffer: latencyBufSemaphoreHandle == NULL");

    return -2;
  } 

  if (xSemaphoreTake(latencyBufSemaphoreHandle, portMAX_DELAY) == pdTRUE) {
    latencyToServer = 0;

    xSemaphoreGive(latencyBufSemaphoreHandle);
  } else {
    ESP_LOGW(TAG, "reset_diff_buffer: can't take semaphore");

    return -1;
  }

  return 0;
}

/**
 *
 */
int32_t latency_buffer_full(bool *is_full) {
  *is_full = MEDIANFILTER_isFull(&latencyMedianFilter, LATENCY_MEDIAN_FILTER_FULL);
  
  return 0;
}

int32_t get_diff_to_server(int64_t *tDiff, int64_t now) {
  static int64_t lastDiff = 0;

  if (latencyBufSemaphoreHandle == NULL) {
    ESP_LOGE(TAG, "get_diff_to_server: latencyBufSemaphoreHandle == NULL");

    return -2;
  }

  if (xSemaphoreTake(latencyBufSemaphoreHandle, 0) == pdFALSE) {
    *tDiff = lastDiff;

    // ESP_LOGW(TAG,
    //          "get_diff_to_server: can't take semaphore. Old diff retrieved");

    return -1;
  }

  *tDiff = latencyToServer;
  lastDiff = latencyToServer;  // store value, so we can return a value if
                               // semaphore couldn't be taken

  xSemaphoreGive(latencyBufSemaphoreHandle);

  return 0;
}
#endif

/**
 *
 */
int32_t server_now(int64_t *sNow, int64_t *diff2Server) {
  int64_t diff, now;

  if (sNow == NULL) {
    return -2;
  }

  now = esp_timer_get_time();

  if (get_diff_to_server(&diff, now) == -1) {
    // ESP_LOGW(TAG,
    //          "server_now: can't get current diff to server. Retrieved old
    //          one");
  }

  if (diff == 0) {
    // ESP_LOGW(TAG, "server_now: diff to server not initialized yet");

    return -1;
  }

  *sNow = now + diff;

  if (diff2Server) {
    *diff2Server = diff;
  }

  // ESP_LOGI(TAG, "now: %lldus", now);
  // ESP_LOGI(TAG, "diff: %lldus", diff);
  // ESP_LOGI(TAG, "serverNow: %lldus", *snow);

  return 0;
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with
 * IRAM_ATTR. If we're okay with the timer irq not being serviced while SPI
 * flash cache is disabled, we can allocate this interrupt without the
 * ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
static bool IRAM_ATTR timer_group0_alarm_cb(
    gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
    void *user_data) {
  // timer_spinlock_take(TIMER_GROUP_1);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  uint64_t timer_counter_value = edata->count_value;

  // Notify the task in the task's notification value.
  xTaskNotifyFromISR(playerTaskHandle, (uint32_t)timer_counter_value,
                     eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

  return xHigherPriorityTaskWoken == pdTRUE;
}

/**
 *
 */
esp_err_t my_gptimer_stop(gptimer_handle_t timer) {
  if (gpTimerRunning == true) {
    gpTimerRunning = false;
    
    esp_err_t ret = 0;
    ret |= gptimer_stop(timer);
    ret |= gptimer_disable(timer);
    
    return ret;
  }

  return ESP_OK;
}

/**
 *
 */
esp_err_t my_gptimer_start(gptimer_handle_t timer) {
  if (gpTimerRunning == false) {
    gpTimerRunning = true;

    return gptimer_start(timer);
  }

  return ESP_OK;
}

static void tg0_timer_deinit(void) {
  //  timer_deinit(TIMER_GROUP_1, TIMER_1);
  if (gptimer) {
    ESP_ERROR_CHECK(my_gptimer_stop(gptimer));
    ESP_ERROR_CHECK(gptimer_del_timer(gptimer));
    gptimer = NULL;
  }
}

/*
 *
 */
static void tg0_timer_init(void) {
  tg0_timer_deinit();

  // Select and initialize basic parameters of the timer
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000,  // 1MHz, 1 tick=1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = timer_group0_alarm_cb,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

  ESP_LOGI(TAG, "init initial sync timer");
}

/**
 *
 */
static void tg0_timer1_start(uint64_t alarm_value) {
  if (gptimer) {
    my_gptimer_stop(gptimer);
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
    gptimer_alarm_config_t alarm_config1 = {
        .alarm_count = alarm_value,  // period
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(my_gptimer_start(gptimer));
  }

  // ESP_LOGI(TAG, "started age timer");
}

#if !USE_SAMPLE_INSERTION
// void rtc_clk_apll_enable(bool enable, uint32_t sdm0, uint32_t sdm1, uint32_t
// sdm2, uint32_t o_div); apll_freq = xtal_freq * (4 + sdm2 + sdm1/256 +
// sdm0/65536)/((o_div + 2) * 2) xtal == 40MHz on lyrat v4.3 I2S bit_clock =
// rate * (number of channels) * bits_per_sample
void adjust_apll(int8_t direction) {
  int sdm0, sdm1, sdm2, o_div;

  // only change if necessary
  if (currentDir == direction) {
    return;
  }

  if (direction == 1) {
    // speed up
    sdm0 = apll_corr_predefine[0][2];
    sdm1 = apll_corr_predefine[0][3];
    sdm2 = apll_corr_predefine[0][4];
    o_div = apll_corr_predefine[0][5];
  } else if (direction == -1) {
    // slow down
    sdm0 = apll_corr_predefine[1][2];
    sdm1 = apll_corr_predefine[1][3];
    sdm2 = apll_corr_predefine[1][4];
    o_div = apll_corr_predefine[1][5];
  } else {
    // reset to normal playback speed
    sdm0 = apll_normal_predefine[2];
    sdm1 = apll_normal_predefine[3];
    sdm2 = apll_normal_predefine[4];
    o_div = apll_normal_predefine[5];

    direction = 0;
  }

  //  periph_rtc_apll_acquire();
  rtc_clk_apll_coeff_set(o_div, sdm0, sdm1, sdm2);
  // rtc_clk_apll_enable(1, sdm0, sdm1, sdm2, o_div);

  currentDir = direction;
}
#endif

/**
 *
 */
int8_t free_pcm_chunk_fragments(pcm_chunk_fragment_t *fragment) {
  if (fragment == NULL) {
    ESP_LOGE(TAG, "free_pcm_chunk_fragments() parameter Error");

    return -1;
  }

  // free all fragments recursive
  if (fragment->nextFragment == NULL) {
    if (fragment->payload != NULL) {
      free(fragment->payload);
      fragment->payload = NULL;
    }

    free(fragment);
    fragment = NULL;
  } else {
    free_pcm_chunk_fragments(fragment->nextFragment);
  }

  return 0;
}

/**
 *
 */
int8_t free_pcm_chunk(pcm_chunk_message_t *pcmChunk) {
  if (pcmChunk == NULL) {
    ESP_LOGE(TAG, "free_pcm_chunk() parameter Error");

    return -1;
  }

  free_pcm_chunk_fragments(pcmChunk->fragment);
  pcmChunk->fragment = NULL;  // was freed in free_pcm_chunk_fragments()

  free(pcmChunk);
  pcmChunk = NULL;

  return 0;
}

/**
 *
 */
int32_t allocate_pcm_chunk_memory_caps(pcm_chunk_message_t *pcmChunk,
                                       size_t bytes, uint32_t caps) {
  size_t largestFreeBlock, freeMem;
  int ret = -3;

  pcmChunk->caps = caps;

  // we got valid memory for pcm_chunk_message_t
  // first we try to allocated 32 bit aligned memory for payload
  // check available memory first so we can decide if we need to fragment the
  // data
  if (caps != 0) {
    freeMem = heap_caps_get_free_size(caps);
    largestFreeBlock = heap_caps_get_largest_free_block(caps);
    if ((freeMem >= bytes) && (largestFreeBlock >= bytes)) {
      // ESP_LOGI(TAG, "32b f %d b %d", freeMem, largestFreeBlock);

      pcmChunk->fragment->payload = (char *)heap_caps_malloc(bytes, caps);
      if (pcmChunk->fragment->payload == NULL) {
        ESP_LOGD(TAG, "Failed to allocate %d bytes of %s for pcm chunk payload",
                 bytes,
                 (caps == (MALLOC_CAP_32BIT | MALLOC_CAP_EXEC)) ? ("IRAM")
                                                                : ("DRAM"));

        ret = -2;
      } else {
        pcmChunk->totalSize = bytes;
        pcmChunk->fragment->nextFragment = NULL;
        pcmChunk->fragment->size = bytes;

        ret = 0;
      }
    } else {
      // ESP_LOGE (TAG, "couldn't get memory to insert chunk of size %d, IRAM
      // freemem: %d blocksize %d", bytes, freeMem, largestFreeBlock);
    }
  } else {
    pcmChunk->fragment->payload = (char *)malloc(bytes);
    if (pcmChunk->fragment->payload == NULL) {
      ESP_LOGE(TAG, "Failed to malloc memory for pcm chunk payload");

      ret = -2;
    } else {
      pcmChunk->totalSize = bytes;
      pcmChunk->fragment->nextFragment = NULL;
      pcmChunk->fragment->size = bytes;

      ret = 0;
    }
  }

  return ret;
}

/**
 *
 */
int32_t allocate_pcm_chunk_memory_caps_fragmented(pcm_chunk_message_t *pcmChunk,
                                                  size_t bytes, uint32_t caps) {
  size_t largestFreeBlock, freeMem;
  int ret = -3;

  // we got valid memory for pcm_chunk_message_t
  // first we try to allocated 32 bit aligned memory for payload
  // check available memory first so we can decide if we need to fragment the
  // data
  freeMem = heap_caps_get_free_size(caps);
  largestFreeBlock = heap_caps_get_largest_free_block(caps);
  if (freeMem >= bytes) {
    // ESP_LOGI(TAG, "32b f %d b %d", freeMem, largestFreeBlock);

    if (largestFreeBlock >= bytes) {
      pcmChunk->fragment->payload = (char *)heap_caps_malloc(bytes, caps);
      if (pcmChunk->fragment->payload == NULL) {
        ESP_LOGE(TAG, "Failed to allocate IRAM memory for pcm chunk payload");

        ret = -2;
      } else {
        pcmChunk->totalSize = bytes;
        pcmChunk->fragment->nextFragment = NULL;
        pcmChunk->fragment->size = bytes;

        ret = 0;
      }
    } else {
      size_t remainingBytes = bytes + (largestFreeBlock % 4);
      size_t needBytes = largestFreeBlock - (largestFreeBlock % 4);
      pcm_chunk_fragment_t *fragment = pcmChunk->fragment;

      pcmChunk->totalSize = 0;

      while (remainingBytes) {
        fragment->payload = (char *)heap_caps_malloc(needBytes, caps);
        if (fragment->payload == NULL) {
          ESP_LOGE(TAG,
                   "Failed to allocate fragmented IRAM memory for "
                   "pcm chunk payload %d %d %d %d",
                   needBytes, remainingBytes, heap_caps_get_free_size(caps),
                   heap_caps_get_largest_free_block(caps));

          ret = -2;

          break;
        } else {
          fragment->size = needBytes;
          remainingBytes -= needBytes;
          pcmChunk->totalSize += needBytes;

          if (remainingBytes > 0) {
            fragment->nextFragment =
                (pcm_chunk_fragment_t *)calloc(1, sizeof(pcm_chunk_fragment_t));
            if (fragment->nextFragment == NULL) {
              ESP_LOGE(TAG,
                       "Failed to fragmented IRAM memory "
                       "for pcm chunk fragment");

              ret = -2;

              break;
            } else {
              fragment = fragment->nextFragment;
              largestFreeBlock = heap_caps_get_largest_free_block(caps);
              if (largestFreeBlock >= remainingBytes) {
                needBytes = remainingBytes;
              } else {
                needBytes = largestFreeBlock - (largestFreeBlock % 4);
              }
            }
          } else {
            ret = 0;
          }
        }
      }
    }
  } else {
    // ESP_LOGE (TAG, "couldn't get memory to insert chunk of size %d, IRAM
    // freemem: %d blocksize %d", decodedWireChunk->size, freeMem,
    // largestFreeBlock);
  }

  return ret;
}

/**
 *
 */
int32_t allocate_pcm_chunk_memory(pcm_chunk_message_t **pcmChunk,
                                  size_t bytes) {
  int ret = -3;

  *pcmChunk = (pcm_chunk_message_t *)calloc(1, sizeof(pcm_chunk_message_t));
  if (*pcmChunk == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for pcm chunk message");

    return -2;
  }

  (*pcmChunk)->fragment =
      (pcm_chunk_fragment_t *)calloc(1, sizeof(pcm_chunk_fragment_t));
  if ((*pcmChunk)->fragment == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for pcm chunk fragment");

    free_pcm_chunk(*pcmChunk);

    return -2;
  }

#if CONFIG_SPIRAM && CONFIG_SPIRAM_BOOT_INIT
  ret = allocate_pcm_chunk_memory_caps(*pcmChunk, bytes,
                                       MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
#elif CONFIG_SPIRAM
  ret = allocate_pcm_chunk_memory_caps(*pcmChunk, bytes, 0);
#else
  // TODO: x should probably be dynamically calculated as a fraction of buffer
  // size if allocation fails we try again every 1ms for max. x ms waiting for
  // chunks to finish playback
  uint32_t x = 50;
  for (int i = 0; i < x; i++) {
    ret = allocate_pcm_chunk_memory_caps(*pcmChunk, bytes,
                                         MALLOC_CAP_32BIT | MALLOC_CAP_EXEC);
    if (ret < 0) {
      ret = allocate_pcm_chunk_memory_caps(*pcmChunk, bytes, MALLOC_CAP_8BIT);
      //      if (ret < 0) {
      //        //      ret = allocate_pcm_chunk_memory_caps_fragmented
      //        //(*pcmChunk, bytes, MALLOC_CAP_32BIT | MALLOC_CAP_EXEC);
      //        if (ret < 0) {
      //          // allocate_pcm_chunk_memory_caps_fragmented (*pcmChunk,
      //          bytes,
      //          // MALLOC_CAP_8BIT);
      //        }
      //      }
    }

    if (ret < 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
    } else {
      break;
    }
  }
#endif

  if (ret < 0) {
    ESP_LOGW(TAG,
             "couldn't get memory to insert chunk, inserting an chunk "
             "containing just 0");

    //    xSemaphoreTake(playerPcmQueueMux, portMAX_DELAY);
    //    ESP_LOGW(
    //        TAG, "%d, %d, %d, %d, %d",
    //        heap_caps_get_free_size(MALLOC_CAP_8BIT),
    //        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
    //        uxQueueMessagesWaiting(pcmChkQHdl),
    //        heap_caps_get_free_size(MALLOC_CAP_32BIT | MALLOC_CAP_EXEC),
    //        heap_caps_get_largest_free_block(MALLOC_CAP_32BIT |
    //        MALLOC_CAP_EXEC));
    //    xSemaphoreGive(playerPcmQueueMux);

    ESP_LOGW(
        TAG, "%d, %d, %d, %d", heap_caps_get_free_size(MALLOC_CAP_8BIT),
        heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
        heap_caps_get_free_size(MALLOC_CAP_32BIT | MALLOC_CAP_EXEC),
        heap_caps_get_largest_free_block(MALLOC_CAP_32BIT | MALLOC_CAP_EXEC));

    // simulate a chunk of all samples 0,
    // player_task() will know what to do with this
    (*pcmChunk)->fragment->payload = NULL;
    (*pcmChunk)->totalSize = bytes;
    (*pcmChunk)->fragment->nextFragment = NULL;
    (*pcmChunk)->fragment->size = bytes;

    ret = 0;
  } else {
    // ESP_LOGI (TAG, "got memory for pcm chunk %p %p %d", *pcmChunk,
    // (*pcmChunk)->fragment->payload, bytes);
  }

  return ret;
}

/**
 *
 */
int32_t insert_pcm_chunk(pcm_chunk_message_t *pcmChunk) {
  if (pcmChunk == NULL) {
    ESP_LOGE(TAG, "Parameter Error");

    return -1;
  }

  if (pcmChkQHdl == NULL) {
    ESP_LOGW(TAG, "pcm chunk queue not created. Player started: %s", playerstarted ? "True": "False");

    free_pcm_chunk(pcmChunk);

    snapcastSetting_t curSet;
    player_get_snapcast_settings(&curSet);
    if (!curSet.muted && gotSettings) {
        start_player(&curSet);
    }

    return -2;
  }

  bool isFull = false;
  latency_buffer_full(&isFull);
  if (isFull == false) {
    free_pcm_chunk(pcmChunk);

    //    ESP_LOGW(TAG, "%s: wait for initial latency measurement to finish",
    //    __func__);

    return -3;
  }
  //  if (uxQueueSpacesAvailable(pcmChkQHdl) == 0) {
  //    pcm_chunk_message_t *element;
  //
  //    xQueueReceive(pcmChkQHdl, &element, portMAX_DELAY);
  //
  //    free_pcm_chunk(element);
  //  }

  // if (xQueueSend(pcmChkQHdl, &pcmChunk, pdMS_TO_TICKS(10)) != pdTRUE) {
  if (xQueueSend(pcmChkQHdl, &pcmChunk, pdMS_TO_TICKS(1)) != pdTRUE) {
    ESP_LOGV(TAG, "send: pcmChunkQueue full, messages waiting %d",
             uxQueueMessagesWaiting(pcmChkQHdl));

    free_pcm_chunk(pcmChunk);
  }
//free_pcm_chunk(pcmChunk);

  return 0;
}

/**
 *
 */
int32_t pcm_chunk_queue_msg_waiting(void) {
  int ret = 0;

  if (pcmChkQHdl) {
    ret = uxQueueMessagesWaiting(pcmChkQHdl);
  }

  return ret;
}

static bool audioCodecCanSleep = false;

/**
 *
 */
static void player_task(void *pvParameters) {
  pcm_chunk_message_t *chnk = NULL;
  int64_t age;
  int64_t serverNow = 0;
  BaseType_t ret;
  int64_t chunkDuration_us = 24000;
  char *p_payload = NULL;
  size_t size = 0;
  uint32_t notifiedValue;
  snapcastSetting_t scSet;
  uint8_t scSetChgd = 0;
  uint64_t timer_val;
  int initialSync = 0;
  int dir = 0;
  int32_t dir_insert_sample = 0;
  int64_t insertedSamplesCounter = 0;
  int64_t buf_us = 0;
  pcm_chunk_fragment_t *fragment = NULL;
  size_t written;
  int64_t clientDacLatency_us = 0;
  int64_t diff2Server = 0;
  int64_t outputBufferDacTime_us = 0;
  int64_t dmaDescDuration_us = 0;
  size_t alreadyWritten = 0;
  static uint32_t queueCreatedWithChkInFrames = UINT32_MAX;
  int64_t playback_start_time_us = 0;
  uint64_t samples_written = 0;  
  UBaseType_t uxHighWaterMark;

  memset(&scSet, 0, sizeof(snapcastSetting_t));
  player_get_snapcast_settings(&scSet);

  ESP_LOGI(TAG, "started sync task");

  //  stats_init();

  queueCreatedWithChkInFrames = scSet.chkInFrames;

  initialSync = 0;
  s_player_is_playing = false;
  
  //audio_hal_ctrl_codec(audio_hal_handle_t audio_hal, audio_hal_codec_mode_t mode, audio_hal_ctrl_t audio_hal_ctrl)

  audio_set_mute(true);

  buf_us = (int64_t)(scSet.buf_ms) * 1000LL;
  clientDacLatency_us = (int64_t)scSet.cDacLat_ms * 1000LL;
  dmaDescDuration_us =
              1000000LL * (int64_t)i2sDmaBufMaxLen / (int64_t)scSet.sr;
#if !USE_SAMPLE_INSERTION
  // force adjust_apll() to set playback speed
  currentDir = 1;
  adjust_apll(0);
#endif

//  if (pcmChkQHdl == NULL) {
//    int entries = ceil(((float)scSet.sr / (float)scSet.chkInFrames) *
//                        ((float)scSet.buf_ms / 1000));
//
//    // some chunks are placed in DMA buffer
//    // so we can save a little RAM here
//    entries -= (i2sDmaBufMaxLen * i2sDmaBufCnt) / scSet.chkInFrames;
//
//    queueCreatedWithChkInFrames = scSet.chkInFrames;
//
//    pcmChkQHdl = xQueueCreate(entries, sizeof(pcm_chunk_message_t *));
//
//    ESP_LOGI(TAG, "created new queue with %d", entries);
//  }
  /* Keep mute active until initial sync and clock locking is complete */

  // wait for early time syncs to be ready
  xSemaphoreTake(latencyBufFullSemaphoreHandle, portMAX_DELAY);
  xSemaphoreGive(latencyBufFullSemaphoreHandle);
  while (1) {
    int64_t tDiff;
    if (get_diff_to_server(&tDiff, esp_timer_get_time())==0) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  while (1) {
    //ESP_LOGD(TAG, "HIGH: %u", uxTaskGetStackHighWaterMark( NULL ));
    
    // ESP_LOGW( TAG, "32b f %d b %d", heap_caps_get_free_size
    //(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block (MALLOC_CAP_8BIT));
    // ESP_LOGW (TAG, "stack free: %d", uxTaskGetStackHighWaterMark(NULL));

    // check if we got changed setting available, if so we need to
    // reinitialize
    ret = xQueueReceive(snapcastSettingQueueHandle, &scSetChgd, 0);
    if (ret == pdTRUE) {
      snapcastSetting_t __scSet;

      player_get_snapcast_settings(&__scSet);

      if ((__scSet.buf_ms > 0) && (__scSet.chkInFrames > 0) &&
          (__scSet.sr > 0)) {
        buf_us = (int64_t)(__scSet.buf_ms) * 1000LL;

        clientDacLatency_us = (int64_t)__scSet.cDacLat_ms * 1000LL;

        if ((scSet.sr != __scSet.sr) || (scSet.bits != __scSet.bits) ||
            (scSet.ch != __scSet.ch)) {
          my_i2s_channel_enable(tx_chan);
          audio_set_mute(true);
          my_i2s_channel_disable(tx_chan);

          ret = player_setup_i2s(&__scSet);
          if (ret < 0) {
            ESP_LOGE(TAG, "player_setup_i2s failed: %d", ret);

            return;
          }

          dmaDescDuration_us =
              1000000LL * (int64_t)i2sDmaBufMaxLen / (int64_t)__scSet.sr;

#if !USE_SAMPLE_INSERTION
          // force adjust_apll() to set playback speed
          currentDir = 1;
          adjust_apll(0);
#endif

          initialSync = 0;
  s_player_is_playing = false;
        }

        if ((scSet.buf_ms != __scSet.buf_ms) ||
            (queueCreatedWithChkInFrames > __scSet.chkInFrames)) {
          destroy_pcm_queue(&pcmChkQHdl);
        }

        if (pcmChkQHdl == NULL) {
          int entries = ceil(((float)__scSet.sr / (float)__scSet.chkInFrames) *
                             ((float)__scSet.buf_ms / 1000));

          // some chunks are placed in DMA buffer
          // so we can save a little RAM here
          entries -= (i2sDmaBufMaxLen * i2sDmaBufCnt) / __scSet.chkInFrames;

          queueCreatedWithChkInFrames = __scSet.chkInFrames;

          pcmChkQHdl = xQueueCreate(entries, sizeof(pcm_chunk_message_t *));

          ESP_LOGI(TAG, "created new queue with %d", entries);
        }

        if ((scSet.sr != __scSet.sr) || (scSet.bits != __scSet.bits) ||
            (scSet.ch != __scSet.ch) || (scSet.buf_ms != __scSet.buf_ms)) {
          ESP_LOGI(TAG,
                   "snapserver config changed, buffer %ldms, chunk %ld frames, "
                   "sample rate %ld, ch %d, bits %d mute %d latency %ld",
                   __scSet.buf_ms, __scSet.chkInFrames, __scSet.sr, __scSet.ch,
                   __scSet.bits, __scSet.muted, __scSet.cDacLat_ms);
        } else {
          audio_set_mute(__scSet.muted);
          ESP_LOGI(TAG, "snapserver config changed, mute: %d", __scSet.muted);
        }

        scSet = __scSet;  // store for next round

      }

    }

    if (chnk == NULL) {
      if (pcmChkQHdl != NULL) {
        ret = xQueueReceive(pcmChkQHdl, &chnk, pdMS_TO_TICKS(2000));
      } else {
         //ESP_LOGE (TAG, "Couldn't get PCM chunk, pcm queue not created");

        vTaskDelay(pdMS_TO_TICKS(100));

        continue;
      }

      if (ret != pdFAIL) {
        chunkDuration_us =
            1000000LL *
            (int64_t)(chnk->totalSize / ((scSet.bits >> 3) * scSet.ch)) /
            (int64_t)scSet.sr;

        // ESP_LOGI(TAG, "got pcm chunk with size %d", chnk->fragment->size);
      }
    } else {
      // ESP_LOGW(TAG, "already retrieved chunk needs service");
      ret = pdPASS;
    }

    if (ret != pdFAIL) {
      int64_t chunkStart = (int64_t)chnk->timestamp.sec * 1000000LL +
                           (int64_t)chnk->timestamp.usec;

      if (initialSync == 0) {
        if (server_now(&serverNow, &diff2Server) >= 0) {
          age = serverNow - chunkStart - buf_us + clientDacLatency_us;
        } else {
          // ESP_LOGW(TAG, "couldn't get server now");

          if (chnk != NULL) {
            free_pcm_chunk(chnk);
            chnk = NULL;
          }

          vTaskDelay(pdMS_TO_TICKS(1));

          continue;
        }

        if (age < 0) {  // get initial sync using hardware timer
          bool dmaFull = false;

          MEDIANFILTER_Init(&shortMedianFilter);
          MEDIANFILTER_Init(&miniMedianFilter);

          tg0_timer1_start(-age);  // timer with 1µs ticks

          my_i2s_channel_disable(tx_chan);

#if !USE_SAMPLE_INSERTION
          adjust_apll(0);  // reset to normal playback speed
#endif
          while (1) {
            if (chnk == NULL) {
              if (pcmChkQHdl != NULL) {
                ret = xQueueReceive(pcmChkQHdl, &chnk, pdMS_TO_TICKS(2000));
                // if (ret != pdFAIL) {
                //   ESP_LOGI(TAG, "got pcm chunk with size %d",
                //            chnk->fragment->size);
                // }
              }
            } else {
              // ESP_LOGI(TAG, "got pcm chunk with size %d",
              // chnk->fragment->size);
            }

            // If we still don't have a chunk, wait and retry
            if (chnk == NULL) {
              continue;
            }

            fragment = chnk->fragment;
            p_payload = fragment->payload;
            size = fragment->size;

            ESP_ERROR_CHECK(
                player_i2s_preload_data(tx_chan, p_payload, size, &written));

            // check if DMA is full at first try here
            if (written != size) {
              dmaFull = true;
            }

            size -= written;
            p_payload += written;

            if (size == 0) {
              if (fragment->nextFragment != NULL) {
                fragment = fragment->nextFragment;
                p_payload = fragment->payload;
                size = fragment->size;
              } else {
                free_pcm_chunk(chnk);
                chnk = NULL;
              }
            }

            if (dmaFull == true) {
              ESP_LOGI(TAG, "DMA completely loaded");

              alreadyWritten = 0;
              chunkStart -=
                  (1000000LL * (int64_t)(i2sDmaBufCnt * i2sDmaBufMaxLen) /
                   (int64_t)scSet.sr);

              break;
            }
          }

          // Wait to be notified of a timer interrupt.
          xTaskNotifyWait(pdFALSE,         // Don't clear bits on entry.
                          pdFALSE,         // Don't clear bits on exit.
                          &notifiedValue,  // Stores the notified value.
                          portMAX_DELAY);
          // or use simple task delay for this
          // vTaskDelay( pdMS_TO_TICKS(-age / 1000) );

          my_gptimer_stop(gptimer);
          
          int64_t dac_init_start = esp_timer_get_time();

          /* Enable I2S clock BEFORE enabling DAC so TAS58xx DSP boots with active BCLK/LRCLK */
          my_i2s_channel_enable(tx_chan);
          
          audio_dac_enable(true);
          
          playback_start_time_us = esp_timer_get_time();
          samples_written = i2sDmaBufCnt * i2sDmaBufMaxLen;

          // get timer value so we can get the real age
          timer_val = (int64_t)notifiedValue;

          // get actual age after alarm
          age = (int64_t)timer_val - (-age);

          initialSync = 1;
          s_player_is_playing = true;

          // TODO: use a timer to un-mute non blocking
          vTaskDelay(pdMS_TO_TICKS(2));
          audio_set_mute(scSet.muted);

          /* Compensate chunkStart by the time elapsed during DAC boot / I2C register writes
           * so shortMedian starts perfectly aligned at ~0 µs, eliminating initial burst of sample drops.
           */
          int64_t dac_init_time = esp_timer_get_time() - dac_init_start;
          chunkStart += dac_init_time;

          ESP_LOGI(TAG, "initial sync age: %lldus, chunk duration: %lldus, dac_init_time: %lldus", age,
                   chunkDuration_us, dac_init_time);

          if (size == 0) {
            continue;
          }
        } else if (age >= 0) {
          if (chnk != NULL) {
            free_pcm_chunk(chnk);
            chnk = NULL;
          }

#if CONFIG_SOC_WIFI_SUPPORTED
          wifi_ap_record_t ap = {0};
          esp_wifi_sta_get_ap_info(&ap);
          int8_t rssi = ap.rssi;
#else
          int8_t rssi = 0;
#endif

          my_gptimer_stop(gptimer);
          
          int msgWaiting = uxQueueMessagesWaiting(pcmChkQHdl);

          ESP_LOGW(TAG,
                   "RESYNCING HARD 1: age %lldus, latency %lldus, free %d, "
                   "largest block %d, rssi: %d, left in queue %d",
                   age, diff2Server, heap_caps_get_free_size(MALLOC_CAP_32BIT),
                   heap_caps_get_largest_free_block(MALLOC_CAP_32BIT), rssi, msgWaiting);
                   
          // get count of chunks we are late for
          uint32_t c = ceil((float)age / (float)chunkDuration_us);  // round up

          // now clear all those chunks which are probably late too
          while (c--) {
            ret = xQueueReceive(pcmChkQHdl, &chnk, pdMS_TO_TICKS(1));
            if (ret == pdPASS) {
              free_pcm_chunk(chnk);
              chnk = NULL;
            } else {
              break;
            }
          }

          dir = 0;

          insertedSamplesCounter = 0;

          audio_set_mute(true);

          my_i2s_channel_disable(tx_chan);

          continue;
        }
      }

      const bool enableControlLoop = true;

      const int64_t shortOffset = SHORT_OFFSET;  // µs, softsync
      const int64_t miniOffset = MINI_OFFSET;    // µs, softsync
      const int64_t hardResyncThreshold = 35000; // µs, hard sync (~1 chunk margin)

      if (initialSync == 1) {
        if (size == 0) {
          fragment = chnk->fragment;
          p_payload = fragment->payload;
          size = fragment->size;
        }

        if (p_payload != NULL) {
          #if 1
          do {
              size_t framesToBytes = (scSet.ch + (scSet.bits >> 3));
#if USE_SAMPLE_INSERTION
              uint32_t sampleSizeInBytes =
                  framesToBytes * INSERT_SAMPLES;
  
              if ((dir_insert_sample > 0) && (size >= sampleSizeInBytes * 2)) {
                /* Drop 1 frame at zero-crossing / lowest amplitude to eliminate audible clicks */
                size_t total_frames = size / framesToBytes;
                const int16_t *s = (const int16_t *)p_payload;
                size_t best_f = total_frames - 1;
                int32_t min_amp = INT32_MAX;
                for (size_t f = 0; f < total_frames; f++) {
                  int32_t amp = abs((int32_t)s[f * 2]) + abs((int32_t)s[f * 2 + 1]);
                  if (amp < min_amp) {
                    min_amp = amp;
                    best_f = f;
                    if (amp == 0) break;
                  }
                }
                if (best_f < total_frames - 1) {
                  int16_t *dest = (int16_t *)p_payload + best_f * 2;
                  const int16_t *src = dest + 2;
                  size_t bytes_to_move = (total_frames - best_f - 1) * framesToBytes;
                  memmove(dest, src, bytes_to_move);
                }
                size -= sampleSizeInBytes;
                dir_insert_sample = 0;
              } else if ((dir_insert_sample > 0) && (size >= sampleSizeInBytes)) {
                size -= sampleSizeInBytes;
                dir_insert_sample = 0;
              }
#endif             
              player_i2s_write(tx_chan, p_payload, size, &written, portMAX_DELAY);
              
              samples_written += (written / framesToBytes);
              size -= written;
              p_payload += written;
              chunkStart += (1000000ll * (written / framesToBytes) / scSet.sr); 

#if USE_SAMPLE_INSERTION
              if (dir_insert_sample < 0) {
                if (player_i2s_write(tx_chan, p_payload - sampleSizeInBytes, sampleSizeInBytes, &written, portMAX_DELAY) != ESP_OK) {
                  ESP_LOGE(TAG, "i2s_playback_task:  I2S write error %d", 1);
                }
                else {
                  samples_written += (written / framesToBytes);
                  dir_insert_sample = 0;
                  chunkStart += (1000000ll * (written / framesToBytes) / scSet.sr);
                } 
              }
#endif
              if (size == 0) {
                if (fragment->nextFragment != NULL) {
                  fragment = fragment->nextFragment;
                  p_payload = fragment->payload;
                  size = fragment->size;
  
                  // ESP_LOGI (TAG, "%s: fragmented", __func__);
                } else {
                  free_pcm_chunk(chnk);
                  chnk = NULL;
                  dir = 0;
  
                  break;
                }
              }
            } while (1);
            
            outputBufferDacTime_us = 1000000ULL * i2sDmaBufMaxLen * i2sDmaBufCnt / scSet.sr; 
          #else
          do {
            written = 0;

#if USE_SAMPLE_INSERTION
            uint32_t sampleSizeInBytes =
                (scSet.bits >> 3) * scSet.ch * INSERT_SAMPLES;

            if ((dir_insert_sample > 0) && (size >= sampleSizeInBytes)) {
              size -= sampleSizeInBytes;
            }
#endif
            int64_t alreadyWrittenTime_us = 0;
            size_t framesToBytes = (scSet.ch + (scSet.bits >> 3));
            while (size) {
              size_t i2sWriteLen;
              size_t tmpSize = i2sDmaBufMaxLen * framesToBytes;

              // #if USE_SAMPLE_INSERTION
              //               if (dir_insert_sample < 0) {
              //            tmpSize -= sampleSizeInBytes;
              //               }
              // #endif

              if (size >= tmpSize) {
                i2sWriteLen = i2sDmaBufMaxLen * framesToBytes - alreadyWritten;

                player_i2s_write(tx_chan, p_payload, i2sWriteLen, &written,
                                 portMAX_DELAY);

                alreadyWrittenTime_us =
                    1000000LL * (int64_t)(alreadyWritten / framesToBytes) /
                    (int64_t)scSet.sr;
                chunkStart += (dmaDescDuration_us - alreadyWrittenTime_us);

                alreadyWritten = 0;

                outputBufferDacTime_us =
                    1000000ULL * i2sDmaBufMaxLen * i2sDmaBufCnt / scSet.sr;
              } else {  // here we are at the end of a chunk
                i2sWriteLen = size;

#if USE_SAMPLE_INSERTION
                size_t insertedSamplesWritten = 0;

                if (i2sWriteLen + sampleSizeInBytes <= i2sDmaBufMaxLen) {
                  if (dir_insert_sample < 0) {
                    if (player_i2s_write(tx_chan, p_payload, sampleSizeInBytes,
                                         &insertedSamplesWritten,
                                         portMAX_DELAY) != ESP_OK) {
                      ESP_LOGE(TAG, "i2s_playback_task:  I2S write error %d",
                               1);
                    }
                  }

                  dir_insert_sample = 0;
                }
#endif

                player_i2s_write(tx_chan, p_payload, i2sWriteLen, &written,
                                 portMAX_DELAY);

#if USE_SAMPLE_INSERTION
                alreadyWritten = written + insertedSamplesWritten;
#else
                alreadyWritten = written;
#endif
                alreadyWrittenTime_us =
                    1000000LL * (int64_t)(alreadyWritten / framesToBytes) /
                    (int64_t)scSet.sr;
                chunkStart += alreadyWrittenTime_us;

                outputBufferDacTime_us = (1000000ULL * i2sDmaBufMaxLen *
                                          (i2sDmaBufCnt - 1) / scSet.sr) +
                                         alreadyWrittenTime_us;
              }

              samples_written += (written / (scSet.ch * (scSet.bits / 8)));
              size -= written;
              p_payload += written;
            }

            dir = 0;

            if (size == 0) {
              if (fragment->nextFragment != NULL) {
                fragment = fragment->nextFragment;
                p_payload = fragment->payload;
                size = fragment->size;

                // ESP_LOGI (TAG, "%s: fragmented", __func__);
              } else {
                free_pcm_chunk(chnk);
                chnk = NULL;
                dir = 0;

                break;
              }
            }
          } while (1);
          #endif
        } else {
          // here we have an empty fragment because of memory allocation error.
          // fill DMA with zeros so we don't get out of sync
          written = 0;
          const size_t write_size = 4;
          uint8_t tmpBuf[write_size];

          memset(tmpBuf, 0, sizeof(tmpBuf));

          do {
            if (player_i2s_write(tx_chan, tmpBuf, write_size, &written,
                                 portMAX_DELAY) != ESP_OK) {
              ESP_LOGE(TAG, "i2s_playback_task: I2S write error %d/%d", written,
                       size);
            }

            samples_written += (written / (scSet.ch * (scSet.bits / 8)));
            size -= written;
          } while (size);

          free_pcm_chunk(chnk);
          chnk = NULL;
        }

        if (server_now(&serverNow, &diff2Server) >= 0) {
          {
            int64_t now_us = esp_timer_get_time();
            // actually played out samples
            int64_t samples_played = samples_written - i2sDmaBufCnt * i2sDmaBufMaxLen;
            #if 0
            // actually played out samples estimate based on clock
            //double samples_played_est = (now_us - playback_start_time_us) * ((float)scSet.sr / 1e6);
            // samples expected to have played
            double samples_expected = (target_play_local_us - playback_start_time_us) * ((float)scSet.sr / 1e6);
            double error_samples = samples_expected - samples_played;
            ESP_LOGI(TAG, "%0.2lf", error_samples);
            #endif
            
            // expected play out based on timestamp
            int64_t target_play_local_us = chunkStart - diff2Server + buf_us - outputBufferDacTime_us - clientDacLatency_us;
            // Ideal playout time based on local audio clock                    
            int64_t actual_play_local_us = playback_start_time_us + (int64_t)((samples_played * 1000000ll) / (int64_t)scSet.sr);
            int64_t error_us = actual_play_local_us - target_play_local_us;

            // Standard Snapcast age calculation (jitter/packet latency relative to playout schedule)
            age = serverNow - chunkStart - buf_us + clientDacLatency_us +
                  outputBufferDacTime_us;

            static uint32_t sync_log_count = 0;
            if (++sync_log_count % 25 == 0) {
              ESP_LOGI(TAG, "SYNC: snap_age=%lldus, err_us=%lldus, now=%lld, play_start=%lld, target=%lld, actual=%lld, played=%lld, written=%lld, diff2Srv=%lld",
                       age, error_us, now_us, playback_start_time_us, target_play_local_us, actual_play_local_us,
                       samples_played, samples_written, diff2Server);
            }
          }

          int64_t shortMedian, miniMedian;

          shortMedian = MEDIANFILTER_Insert(&shortMedianFilter, age);
          miniMedian = MEDIANFILTER_Insert(&miniMedianFilter, age);

          int msgWaiting = uxQueueMessagesWaiting(pcmChkQHdl);

          // resync hard if we are getting very late / early.
          // rest gets tuned in through apll speed control or sample insertion.
          // NOTE: Do not hard resync just because msgWaiting == 0 (brief gap between tracks/packets)!
          if (MEDIANFILTER_isFull(&shortMedianFilter, 0) &&
              ((shortMedian > hardResyncThreshold) ||
               (shortMedian < -hardResyncThreshold))) 
          {
            if (chnk != NULL) {
              free_pcm_chunk(chnk);
              chnk = NULL;
            }

#if CONFIG_SOC_WIFI_SUPPORTED
            wifi_ap_record_t ap = {0};
            esp_wifi_sta_get_ap_info(&ap);
            int8_t rssi = ap.rssi;
#else
            int8_t rssi = 0;
#endif

            ESP_LOGW(TAG,
                     "RESYNCING HARD 2: age %lldus, latency %lldus, free "
                     "%d, largest block %d, %d, rssi: %d",
                     age, diff2Server,
                     heap_caps_get_free_size(MALLOC_CAP_32BIT),
                     heap_caps_get_largest_free_block(MALLOC_CAP_32BIT),
                     msgWaiting, rssi);

            my_gptimer_stop(gptimer);

            audio_set_mute(true);

            my_i2s_channel_disable(tx_chan);

            initialSync = 0;
  s_player_is_playing = false;

            insertedSamplesCounter = 0;

            continue;
          }

#if USE_SAMPLE_INSERTION  // insert samples to adjust sync
          static uint32_t last_sample_adjust_chunk = 0;
          static uint32_t current_chunk_counter = 0;
          current_chunk_counter++;

          if ((enableControlLoop == true) &&
              (MEDIANFILTER_isFull(&shortMedianFilter, 0)) &&
              (current_chunk_counter - last_sample_adjust_chunk >= 40)) {
            if ((shortMedian < -shortOffset) && (miniMedian < -miniOffset) &&
                (age < -miniOffset)) {  // we are early
              dir = -1;
              dir_insert_sample = -1;
              insertedSamplesCounter += INSERT_SAMPLES;
              last_sample_adjust_chunk = current_chunk_counter;
              ESP_LOGI(TAG, "Sync adjust: inserting %d sample(s) (shortMedian=%lld, age=%lld)",
                       INSERT_SAMPLES, shortMedian, age);
            } else if ((shortMedian > shortOffset) &&
                       (miniMedian > miniOffset) &&
                       (age > miniOffset)) {  // we are late
              dir = 1;
              dir_insert_sample = 1;
              insertedSamplesCounter -= INSERT_SAMPLES;
              last_sample_adjust_chunk = current_chunk_counter;
              ESP_LOGI(TAG, "Sync adjust: dropping %d sample(s) (shortMedian=%lld, age=%lld)",
                       INSERT_SAMPLES, shortMedian, age);
            }
          }
#else  // use APLL to adjust sync
          if ((enableControlLoop == true) &&
              (MEDIANFILTER_isFull(&shortMedianFilter, 0))) {
            if ((shortMedian < -shortOffset) && (miniMedian < -miniOffset) &&
                (age < -miniOffset)) {  // we are early
              dir = -1;
            } else if ((shortMedian > shortOffset) &&
                       (miniMedian > miniOffset) &&
                       (age > miniOffset)) {  // we are late
              dir = 1;
            }

            adjust_apll(dir);
          }
#endif
           ESP_LOGD(TAG, "%d, %lldus, %lldus, %lldus, q:%d, %lld, %lld", dir,
                   age, shortMedian, miniMedian,
                   uxQueueMessagesWaiting(pcmChkQHdl), insertedSamplesCounter,
                   chunkDuration_us);
          // ESP_LOGI(TAG, "%d, %lldus, %lldus, %lldus, q:%d", dir,
          //         age, shortMedian, miniMedian,
          //         uxQueueMessagesWaiting(pcmChkQHdl));
          // ESP_LOGI( TAG, "8b f %d b %d",
          //       heap_caps_get_free_size(MALLOC_CAP_8BIT |
          //                         MALLOC_CAP_INTERNAL),
          //           heap_caps_get_largest_free_block(MALLOC_CAP_8BIT |
          //                                            MALLOC_CAP_INTERNAL));
          // ESP_LOGI( TAG, "32b f %d b %d",
          //           heap_caps_get_free_size(MALLOC_CAP_32BIT |
          //                                   MALLOC_CAP_EXEC),
          //           heap_caps_get_largest_free_block(MALLOC_CAP_32BIT |
          //     MALLOC_CAP_EXEC));
        } else {
          // ESP_LOGW(TAG, "couldn't get server now");

          if (chnk != NULL) {
            free_pcm_chunk(chnk);
            chnk = NULL;
          }

          vTaskDelay(pdMS_TO_TICKS(1));

          continue;
        }
      }
    } else {
      if (scSet.muted) {
        /* When muted in Snapcast, the server stops transmitting audio chunks.
         * Keep the I2S clocks active and the player loop alive (amp stays in PLAY_MUTE 0x0B)
         * so unmuting is instantaneous, perfectly smooth, and completely click-free!
         */
        initialSync = 0;
  s_player_is_playing = false;
        vTaskDelay(pdMS_TO_TICKS(50));
        continue;
      }

      int64_t sec, msec, usec;

      sec = diff2Server / 1000000;
      usec = diff2Server - sec * 1000000;
      msec = usec / 1000;
      usec = usec % 1000;

      if (pcmChkQHdl != NULL) {
        ESP_LOGV(TAG,
                 "Couldn't get PCM chunk, recv: messages waiting %d, "
                 "diff2Server: %llds, %lld.%lldms",
                 uxQueueMessagesWaiting(pcmChkQHdl), sec, msec, usec);
      }

      dir = 0;
      initialSync = 0;
  s_player_is_playing = false;

      audio_set_mute(true);
      audio_dac_enable(false);
      my_i2s_channel_disable(tx_chan);
      i2s_del_channel(tx_chan);
      tx_chan = NULL;

      break;
    }
  }
  ret = 0;

  xSemaphoreTake(snapcastSettingsMux, portMAX_DELAY);
  // delete the queue
  vQueueDelete(snapcastSettingQueueHandle);
  snapcastSettingQueueHandle = NULL;
  xSemaphoreGive(snapcastSettingsMux);

#if CONFIG_PM_ENABLE
  esp_pm_lock_release(player_pm_lock_handle);
#endif

  ret = destroy_pcm_queue(&pcmChkQHdl);

  tg0_timer_deinit();
  playerstarted = false;
  ESP_LOGI(TAG, "stop player done");
  playerTaskHandle = NULL;
  vTaskDelete(NULL);
}

