/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2020 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in
 * which case, it is free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the
 * Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

#include "tas5805m.h"

#include "esp_log.h"
#include "i2c_bus.h"
#include "tas5805m_reg_cfg.h"
#include <math.h>

static const char *TAG = "TAS5805M";

#define TAS5805M_SET_BOOK_AND_PAGE(BOOK, PAGE) \
    do { \
      tas5805m_write_byte(TAS5805M_REG_PAGE_SET, TAS5805M_REG_PAGE_ZERO); \
      tas5805m_write_byte(TAS5805M_REG_BOOK_SET, BOOK);                   \
      tas5805m_write_byte(TAS5805M_REG_PAGE_SET, PAGE);                   \
    } while (0)

static tas58xx_chip_variant_t tas5805m_chip_variant = TAS58XX_CHIP_TAS5805M;

tas58xx_chip_variant_t tas5805m_get_chip_variant(void) {
  return tas5805m_chip_variant;
}

// State of TAS5805M (internal to this module)
static TAS5805_STATE tas5805m_state = {
  .volume = 0,
  .state = TAS5805M_CTRL_HI_Z | TAS5805M_CTRL_MUTE,
  .mixer_mode = MIXER_STEREO,
  .channel_gain_l = 0,
  .channel_gain_r = 0,
};

typedef struct {
  bool is_powered;
  bool is_muted;
  uint8_t raw_vol;
  uint8_t again;
  TAS5805M_DAC_MODE dac_mode;
  TAS5805M_MOD_MODE mod_mode;
  TAS5805M_SW_FREQ sw_freq;
  TAS5805M_BD_FREQ bd_freq;
  TAS5805M_EQ_MODE eq_mode;
  bool use_eq_profile[2];
} tas5805m_hw_cache_t;

static tas5805m_hw_cache_t tas5805m_hw = {
  .is_powered = false,
  .is_muted = true,
  .raw_vol = TAS5805M_VOLUME_DIGITAL_DEFAULT, /* 0x30 = 0 dB */
  .again = 3,                                 /* 3 = -1.5 dB (matches Linux driver default) */
  .dac_mode = TAS5805M_DAC_MODE_BTL,
  .mod_mode = MOD_MODE_BD,
  .sw_freq = SW_FREQ_384K,
  .bd_freq = SW_FREQ_100K,
  .eq_mode = TAS5805M_EQ_MODE_OFF,
  .use_eq_profile = {false, false},
};

typedef struct {
  uint8_t reg;
  uint8_t val;
} tas58xx_cfg_reg_t;

/* Exact preboot sequence from Linux driver (tas58xx.c) */
static const tas58xx_cfg_reg_t dsp_cfg_preboot[] = {
  { 0x00, 0x00 },
  { 0x7f, 0x00 },
  { 0x03, 0x02 },
  { 0x01, 0x11 },
  { 0x00, 0x00 },
  { 0x00, 0x00 },
  { 0x00, 0x00 },
  { 0x00, 0x00 },
  { 0x00, 0x00 },
  { 0x7f, 0x00 },
  { 0x03, 0x02 },
  { 0x30, 0x01 },
};

/* Exact GPIO preboot config for TAS5825M / TAS5830M from Linux driver (tas58xx.c) */
static const tas58xx_cfg_reg_t dsp_cfg_preboot_gpio_config[] = {
  { 0x00, 0x00 },
  { 0x7f, 0x00 },
  { 0x61, 0x08 },
  { 0x62, 0x0b },
  { 0x63, 0x09 },
  { 0x60, 0x07 },
};

static esp_err_t tas5805m_refresh(void);

// Tracks whether the DSP and initial registers have undergone full cold boot
static bool tas5805m_dsp_booted = false;

/* Task handle for fault monitoring */
static TaskHandle_t tas5805m_fault_monitor_task_handle = NULL;

/* Default I2C config */
static i2c_config_t i2c_cfg = {
    .mode = I2C_MODE_MASTER,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ,
};

/*
 * Operate fuction of PA
 */
audio_hal_func_t AUDIO_CODEC_TAS5805M_DEFAULT_HANDLE = {
    .audio_codec_initialize = tas5805m_init,
    .audio_codec_deinitialize = tas5805m_deinit,
    .audio_codec_ctrl = tas5805m_ctrl,
    .audio_codec_config_iface = tas5805m_config_iface,
    .audio_codec_set_mute = tas5805m_set_mute,
    .audio_codec_set_volume = tas5805m_set_volume,
    .audio_codec_get_volume = tas5805m_get_volume,
    .audio_hal_lock = NULL,
    .handle = NULL,
};

/* Fault monitoring task */
static void tas5805m_fault_monitor_task(void *pvParameters) {
  TAS5805M_FAULT fault;
  ESP_LOGI(TAG, "Fault monitoring task started");
  
  while (1) {
    /* Only poll fault registers while powered (matches Linux tas58xx_fault_monitor) */
    if (!tas5805m_hw.is_powered) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // Read fault registers
    esp_err_t ret = tas5805m_get_faults(&fault);
    if (ret == ESP_OK) {
      // Check if any faults are present (ignore lone clock fault bit 2 while idle)
      if (fault.err0 || (fault.err1 & ~(1 << 2)) || fault.err2 || fault.ot_warn) {
        ESP_LOGW(TAG, "Faults detected: err0=0x%02x, err1=0x%02x, err2=0x%02x, ot_warn=0x%02x",
                 fault.err0, fault.err1, fault.err2, fault.ot_warn);
        
        // Decode and log the faults
        tas5805m_decode_faults(fault);
        
        // Clear the faults
        ret = tas5805m_clear_faults();
        if (ret == ESP_OK) {
          ESP_LOGI(TAG, "Faults cleared");
        } else {
          ESP_LOGE(TAG, "Failed to clear faults: %s", esp_err_to_name(ret));
        }
      }
    } else {
      ESP_LOGE(TAG, "Failed to read faults: %s", esp_err_to_name(ret));
    }
    
    // Wait for 5 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

/* Init the I2C Driver */
void i2c_master_init() {
  int i2c_master_port = I2C_MASTER_NUM;

  ESP_ERROR_CHECK(get_i2c_pins(I2C_NUM_0, &i2c_cfg));

  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &i2c_cfg));

  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, i2c_cfg.mode,
                                     I2C_MASTER_RX_BUF_DISABLE,
                                     I2C_MASTER_TX_BUF_DISABLE, 0));
}

/* Helper Functions */

// Reading of TAS5805M-Register
esp_err_t tas5805m_read_byte(uint8_t register_name, uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TAS5805M_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, register_name, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_TAS5805M_MASTER_NUM, cmd,
                             1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "%s: I2C ERROR", __func__);
  }

  vTaskDelay(1 / portTICK_PERIOD_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TAS5805M_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_TAS5805M_MASTER_NUM, cmd,
                             1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  ESP_LOGV(TAG, "%s: Read 0x%02x from register 0x%02x", __func__, *data, register_name);
  return ret;
}

// Writing of TAS5805M-Register
esp_err_t tas5805m_write_byte(uint8_t register_name, uint8_t value) {
  int ret = 0;
  ESP_LOGV(TAG, "%s: Writing 0x%02x to register 0x%02x", __func__, value, register_name);

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TAS5805M_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, register_name, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
  i2c_master_stop(cmd);

  ret = i2c_master_cmd_begin(I2C_TAS5805M_MASTER_NUM, cmd,
                             1000 / portTICK_PERIOD_MS);

  // Check if ret is OK
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "%s: Error communicating over I2C: %s", __func__, esp_err_to_name(ret));
  }

  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t tas5805m_write_bytes(uint8_t *reg,
                               int regLen, uint8_t *data, int datalen)
{
  int ret = ESP_OK;
  ESP_LOGV(TAG, "%s: 0x%02x <- [%d] bytes", __func__, *reg, datalen);
  for (int i = 0; i < datalen; i++)
  {
    ESP_LOGV(TAG, "%s: 0x%02x", __func__, data[i]);
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ret |= i2c_master_start(cmd);
  ret |= i2c_master_write_byte(cmd, TAS5805M_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
  ret |= i2c_master_write(cmd, reg, regLen, ACK_CHECK_EN);
  ret |= i2c_master_write(cmd, data, datalen, ACK_CHECK_EN);
  ret |= i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_TAS5805M_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);

  // Check if ret is OK
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
  }

  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t tas5805m_read_bytes(uint8_t *reg, int regLen, uint8_t *data, int datalen)
{
  int ret = ESP_OK;
  ESP_LOGV(TAG, "%s: 0x%02x -> [%d] bytes", __func__, *reg, datalen);

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  ret |= i2c_master_start(cmd);
  ret |= i2c_master_write_byte(cmd, TAS5805M_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
  ret |= i2c_master_write(cmd, reg, regLen, ACK_CHECK_EN);
  ret |= i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_TAS5805M_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "%s: Error during I2C write phase: %s", __func__, esp_err_to_name(ret));
    return ret;
  }

  vTaskDelay(1 / portTICK_PERIOD_MS);
  
  cmd = i2c_cmd_link_create();
  ret |= i2c_master_start(cmd);
  ret |= i2c_master_write_byte(cmd, TAS5805M_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
  if (datalen > 1) {
    ret |= i2c_master_read(cmd, data, datalen - 1, ACK_VAL);
  }
  ret |= i2c_master_read_byte(cmd, data + datalen - 1, NACK_VAL);
  ret |= i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_TAS5805M_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "%s: Error during I2C read phase: %s", __func__, esp_err_to_name(ret));
  } else {
    for (int i = 0; i < datalen; i++) {
      ESP_LOGV(TAG, "%s: [%d] = 0x%02x", __func__, i, data[i]);
    }
  }

  i2c_cmd_link_delete(cmd);

  return ret;
}

bool tas5805m_wait_clock_stable(void) {
  uint8_t global1 = 0;
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);

  for (int i = 0; i < 40; i++) {
    tas5805m_write_byte(TAS5805M_FAULT_CLEAR_REGISTER, TAS5805M_ANALOG_FAULT_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(5));

    if (tas5805m_read_byte(TAS5805M_GLOBAL_FAULT1_REGISTER, &global1) != ESP_OK) {
      return false;
    }

    if (!(global1 & (1 << 2))) {
      ESP_LOGD(TAG, "%s: Clock stable after ~%d ms", __func__, (i + 1) * 5);
      return true;
    }
  }

  ESP_LOGW(TAG, "%s: Clock fault did not clear within 200ms (global1=0x%02x)", __func__, global1);
  return false;
}

static esp_err_t tas58xx_send_cfg(const tas58xx_cfg_reg_t *cfg, size_t len) {
  esp_err_t ret = ESP_OK;
  for (size_t i = 0; i < len; i++) {
    ret = tas5805m_write_byte(cfg[i].reg, cfg[i].val);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "%s: Failed writing reg 0x%02x=0x%02x: %s", __func__, cfg[i].reg, cfg[i].val, esp_err_to_name(ret));
      return ret;
    }
  }
  return ret;
}

/* Forward declarations for internal hardware sync helpers */
static esp_err_t tas5805m_refresh(void);
static esp_err_t tas5805m_apply_mixer_mode_hw(TAS5805M_MIXER_MODE mode);
static esp_err_t tas5805m_apply_channel_gain_hw(TAS5805M_EQ_CHANNELS channel, int8_t gain_db);
#if defined(CONFIG_DAC_TAS5805M_EQ_SUPPORT)
static esp_err_t tas5805m_apply_eq_gain_hw(TAS5805M_EQ_CHANNELS channel, int band, int gain);
static esp_err_t tas5805m_apply_eq_profile_hw(TAS5805M_EQ_CHANNELS channel, TAS5805M_EQ_PROFILE profile);
#endif

// Inits the TAS58xx (mirrors Linux tas58xx_i2c_probe: PDN toggle + DIE_ID detection; DSP boot deferred to tas5805m_ctrl START)
esp_err_t tas5805m_init() {
  ESP_LOGD(TAG, "%s: Initializing TAS58xx", __func__);
  int ret = 0;
  // Init the I2C-Driver
  i2c_master_init();
  /* Register the PDN pin as output with pull-down enabled.
   * Hold PDN LOW immediately to prevent output glitches during power-up.
   */
  gpio_set_level(TAS5805M_GPIO_PDN, 0);
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = TAS5805M_GPIO_PDN_MASK;
  io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  ESP_LOGI(TAG, "%s: Triggering power down pin: %d (with pull-down)", __func__, TAS5805M_GPIO_PDN);
  gpio_config(&io_conf);
  gpio_set_level(TAS5805M_GPIO_PDN, 0);
  vTaskDelay(pdMS_TO_TICKS(100));
  gpio_set_level(TAS5805M_GPIO_PDN, 1);
  vTaskDelay(pdMS_TO_TICKS(15));

  /* Switch to Book 0 Page 0 and set DEEP SLEEP (0x00) with mute (0xFF) while unclocked.
   * This tri-states the outputs completely and prevents unclocked analog charge pops during boot.
   */
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  tas5805m_write_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, TAS5805M_VOLUME_MUTE);
  ESP_ERROR_CHECK(tas5805m_set_state(TAS5805M_CTRL_DEEP_SLEEP));

  // Detect chip variant via DIE_ID (0x67)
  uint8_t die_id = 0;
  if (tas5805m_read_byte(TAS5805M_DIE_ID_REGISTER, &die_id) == ESP_OK) {
    if (die_id == TAS5830M_DIE_ID) {
      tas5805m_chip_variant = TAS58XX_CHIP_TAS5830M;
      ESP_LOGI(TAG, "%s: Detected TAS5830M (DIE_ID=0x%02x)", __func__, die_id);
    } else {
      tas5805m_chip_variant = TAS58XX_CHIP_TAS5805M;
      ESP_LOGI(TAG, "%s: Detected TAS5805M (DIE_ID=0x%02x)", __func__, die_id);
    }
  } else {
    ESP_LOGW(TAG, "%s: Failed to read DIE_ID, defaulting to TAS5805M", __func__);
    tas5805m_chip_variant = TAS58XX_CHIP_TAS5805M;
  }

  /* Initialize cached state to Linux driver defaults */
  tas5805m_hw.dac_mode = TAS5805M_DAC_MODE_BTL;
  tas5805m_hw.mod_mode = MOD_MODE_BD;
  tas5805m_hw.sw_freq = SW_FREQ_384K;
  tas5805m_hw.bd_freq = SW_FREQ_100K;
  tas5805m_hw.again = 3; /* -1.5 dB */
  tas5805m_hw.raw_vol = TAS5805M_VOLUME_MAX; /* 0x30 = 0 dB */
  tas5805m_hw.eq_mode = TAS5805M_EQ_MODE_ON;
  tas5805m_hw.is_powered = false;
  tas5805m_hw.is_muted = false;
  tas5805m_hw.use_eq_profile[0] = false;
  tas5805m_hw.use_eq_profile[1] = false;

  tas5805m_state.volume = 100;
  tas5805m_state.state = TAS5805M_CTRL_HI_Z;
  tas5805m_state.mixer_mode = MIXER_STEREO;
  tas5805m_state.channel_gain_l = 0;
  tas5805m_state.channel_gain_r = 0;

  /* Start fault monitoring task */
  if (tas5805m_fault_monitor_task_handle == NULL) {
    BaseType_t task_ret = xTaskCreate(
      tas5805m_fault_monitor_task,
      "tas5805m_faults",
      4096,
      NULL,
      5,
      &tas5805m_fault_monitor_task_handle
    );

    if (task_ret != pdPASS) {
      ESP_LOGE(TAG, "%s: Failed to create fault monitoring task", __func__);
      return ESP_FAIL;
    }

    ESP_LOGI(TAG, "%s: Fault monitoring task created", __func__);
  }

  return ret;
}

// Getting cached TAS5805 state
esp_err_t tas5805m_get_state(TAS5805_STATE *out_state)
{
  *out_state = tas5805m_state;
  return ESP_OK;
}

// Setting the DAC State of TAS5805M
esp_err_t tas5805m_set_state(TAS5805M_CTRL_STATE state)
{
  ESP_LOGI(TAG, "%s: Setting state to 0x%02X", __func__, state);
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  esp_err_t ret = tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, state);
  if (ret == ESP_OK) {
    /* Update in-memory state only after successful device write */
    tas5805m_state.state = state;
  } else {
    ESP_LOGW(TAG, "%s: Failed to set device state (0x%x): %s", __func__, state, esp_err_to_name(ret));
  }
  return ret;
}

// Setting the Volume (0..100%) using cubic perceptual loudness curve matching Linux snapclient
esp_err_t tas5805m_set_volume(int vol) {
  /* Clamp input percent to [0..100] */
  if (vol < 0) vol = 0;
  if (vol > 100) vol = 100;

  uint8_t reg_val = 0;
  float db = 0.0f;
  if (vol == 0) {
    reg_val = (uint8_t)TAS5805M_VOLUME_MUTE;
    ESP_LOGI(TAG, "Volume set: %d%% slider -> MUTE (reg 0x%02X, 0.00%% amplitude)", vol, reg_val);
  } else {
    /* Linux snapclient uses a cubic volume curve: linear_gain = (vol / 100.0)^3
     * In dB: dB = 20 * log10((vol / 100.0)^3) = 60 * log10(vol / 100.0).
     * TAS58xx VOL_CTRL (0x4C): 0x30 = 0.0 dB, +1 step = -0.5 dB (2 steps per dB).
     * Previously, linear mapping in dB made 50% = -30 dB (12 dB quieter than Linux's -18 dB!)
     */
    db = 60.0f * log10f((float)vol / 100.0f);
    int steps_from_0db = (int)lroundf(-db * 2.0f);
    int computed = (int)TAS5805M_VOLUME_MAX + steps_from_0db;
    if (computed < (int)TAS5805M_VOLUME_MAX) computed = (int)TAS5805M_VOLUME_MAX;
    if (computed > 0xFE) computed = 0xFE;
    reg_val = (uint8_t)computed;
    float actual_db = (float)((int)TAS5805M_VOLUME_MAX - (int)reg_val) * 0.5f;
    float lin_amp_pct = powf(10.0f, actual_db / 20.0f) * 100.0f;
    float dac_scale_pct = (float)(0xFE - reg_val) / (float)(0xFE - TAS5805M_VOLUME_MAX) * 100.0f;
    ESP_LOGI(TAG, "Volume set: %d%% slider -> reg 0x%02X (%.1f dB | %.2f%% amplitude | %.1f%% DAC scale)",
             vol, reg_val, actual_db, lin_amp_pct, dac_scale_pct);
  }

  tas5805m_hw.raw_vol = reg_val;
  tas5805m_state.volume = vol;

  /* Writing the Volume to the Register */
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  esp_err_t ret = tas5805m_write_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, reg_val);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "%s: Failed to write volume (reg 0x%02x): %s", __func__, reg_val, esp_err_to_name(ret));
  }
  return ret;
}

// Getting the Volume
esp_err_t tas5805m_get_volume(int *vol) {
  if (vol == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  *vol = tas5805m_state.volume;
  ESP_LOGD(TAG, "%s: Getting volume (cached): %d", __func__, *vol);
  return ESP_OK;
}


// Setting the raw register Volume [0..255], 0x30 is 0dB, 0xFF is mute
esp_err_t tas5805m_set_digital_volume(uint8_t vol)
{
  esp_err_t ret = ESP_OK;
  if (vol < TAS5805M_VOLUME_DIGITAL_MIN)
  {
    vol = TAS5805M_VOLUME_DIGITAL_MIN;
  }
  if (vol > TAS5805M_VOLUME_DIGITAL_MAX)
  {
    vol = TAS5805M_VOLUME_DIGITAL_MAX;
  }

  tas5805m_hw.raw_vol = vol;
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  ret = tas5805m_write_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, vol);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
  }

  return ret;
}

// Getting the raw register Volume [0..255]
esp_err_t tas5805m_get_digital_volume(uint8_t *vol)
{
  if (vol == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!tas5805m_hw.is_powered) {
    *vol = tas5805m_hw.raw_vol;
    return ESP_OK;
  }
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  esp_err_t ret = tas5805m_read_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, vol);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
    *vol = tas5805m_hw.raw_vol;
  }
  return ret;
}

// Deinit the TAS5805M
esp_err_t tas5805m_deinit(void) {
  tas5805m_hw.is_powered = false;
  tas5805m_dsp_booted = false;
  /* Stop fault monitoring task */
  if (tas5805m_fault_monitor_task_handle != NULL) {
    vTaskDelete(tas5805m_fault_monitor_task_handle);
    tas5805m_fault_monitor_task_handle = NULL;
    ESP_LOGI(TAG, "%s: Fault monitoring task deleted", __func__);
  }
  
  ESP_ERROR_CHECK(tas5805m_set_state(TAS5805M_CTRL_HI_Z));
  gpio_set_level(TAS5805M_GPIO_PDN, 0);
  vTaskDelay(6 / portTICK_PERIOD_MS);
  return ESP_OK;
}

// Setting mute state (matches Linux tas58xx_dac_mute)
esp_err_t tas5805m_set_mute(bool enable) {
  ESP_LOGI(TAG, "%s: Setting mute to %d (is_powered=%d)", __func__, enable, tas5805m_hw.is_powered);
  tas5805m_hw.is_muted = enable;

  if (!tas5805m_hw.is_powered) {
    if (enable) {
      tas5805m_state.state = (TAS5805M_CTRL_STATE)(tas5805m_state.state | TAS5805M_CTRL_MUTE);
    } else {
      tas5805m_state.state = (TAS5805M_CTRL_STATE)(tas5805m_state.state & ~TAS5805M_CTRL_MUTE);
    }
    return ESP_OK;
  }

  TAS5805M_CTRL_STATE new_state = enable ? TAS5805M_CTRL_PLAY_MUTE : TAS5805M_CTRL_PLAY;
  return tas5805m_set_state(new_state);
}

// Getting mute state
esp_err_t tas5805m_get_mute(bool *enabled) {
  if (enabled == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  *enabled = tas5805m_hw.is_muted;
  ESP_LOGD(TAG, "%s: Getting mute: %d", __func__, *enabled);
  return ESP_OK;
}

/* Map logical TAS5805M_SW_FREQ enum to hardware bits [6:4] */
static uint8_t tas58xx_sw_freq_to_hardware(TAS5805M_SW_FREQ freq, tas58xx_chip_variant_t variant) {
  /* Indices: 0=768k, 1=384k, 2=480k, 3=576k */
  static const uint8_t fsw_5805m[4] = { 0x0, 0x1, 0x3, 0x4 };
  static const uint8_t fsw_5830m[4] = { 0x4, 0x0, 0x2, 0x3 };
  uint8_t idx = 0;
  switch (freq) {
    case SW_FREQ_768K: idx = 0; break;
    case SW_FREQ_384K: idx = 1; break;
    case SW_FREQ_480K: idx = 2; break;
    case SW_FREQ_576K: idx = 3; break;
    default: idx = 1; break;
  }
  uint8_t field = (variant == TAS58XX_CHIP_TAS5830M) ? fsw_5830m[idx] : fsw_5805m[idx];
  return (field & 0x07) << 4;
}

/* Map hardware bits [6:4] to logical TAS5805M_SW_FREQ enum */
static TAS5805M_SW_FREQ tas58xx_hardware_to_sw_freq(uint8_t raw, tas58xx_chip_variant_t variant) {
  uint8_t field = (raw >> 4) & 0x07;
  if (variant == TAS58XX_CHIP_TAS5830M) {
    switch (field) {
      case 0x4: return SW_FREQ_768K;
      case 0x0: return SW_FREQ_384K;
      case 0x2: return SW_FREQ_480K;
      case 0x3: return SW_FREQ_576K;
      default:  return SW_FREQ_384K;
    }
  } else {
    switch (field) {
      case 0x0: return SW_FREQ_768K;
      case 0x1: return SW_FREQ_384K;
      case 0x3: return SW_FREQ_480K;
      case 0x4: return SW_FREQ_576K;
      default:  return SW_FREQ_384K;
    }
  }
}

/* Map logical TAS5805M_BD_FREQ enum to hardware bits [6:5] */
static uint8_t tas58xx_bd_freq_to_hardware(TAS5805M_BD_FREQ bd_freq, tas58xx_chip_variant_t variant) {
  /* Indices: 0=80k, 1=100k, 2=120k, 3=175k */
  static const uint8_t bd_5805m[4] = { 0x0, 0x1, 0x2, 0x3 };
  static const uint8_t bd_5830m[4] = { 0x1, 0x0, 0x2, 0x3 };
  uint8_t idx = 0;
  switch (bd_freq) {
    case SW_FREQ_80K:  idx = 0; break;
    case SW_FREQ_100K: idx = 1; break;
    case SW_FREQ_120K: idx = 2; break;
    case SW_FREQ_175K: idx = 3; break;
    default: idx = 1; break;
  }
  uint8_t field = (variant == TAS58XX_CHIP_TAS5830M) ? bd_5830m[idx] : bd_5805m[idx];
  return (field & 0x03) << 5;
}

/* Map hardware bits [6:5] to logical TAS5805M_BD_FREQ enum */
static TAS5805M_BD_FREQ tas58xx_hardware_to_bd_freq(uint8_t raw, tas58xx_chip_variant_t variant) {
  uint8_t field = (raw >> 5) & 0x03;
  if (variant == TAS58XX_CHIP_TAS5830M) {
    switch (field) {
      case 0x1: return SW_FREQ_80K;
      case 0x0: return SW_FREQ_100K;
      case 0x2: return SW_FREQ_120K;
      case 0x3: return SW_FREQ_175K;
      default:  return SW_FREQ_100K;
    }
  } else {
    switch (field) {
      case 0x0: return SW_FREQ_80K;
      case 0x1: return SW_FREQ_100K;
      case 0x2: return SW_FREQ_120K;
      case 0x3: return SW_FREQ_175K;
      default:  return SW_FREQ_100K;
    }
  }
}

/* Synchronize all cached settings to hardware while in Hi-Z mode.
 * Mirrors tas58xx_refresh() in the Linux driver (tas58xx.c:391-664) 1:1.
 * Must be called when I2S clocks are active and DSP has exited reset.
 */
static esp_err_t tas5805m_refresh(void) {
  esp_err_t ret = ESP_OK;

  ESP_LOGI(TAG, "%s: Refreshing TAS58xx hardware registers in Hi-Z (variant=%s)",
           __func__, tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M ? "TAS5830M" : "TAS5805M");

  /* Step 1: Ensure Book 0 Page 0 and Hi-Z state (0x02) */
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);

  /* Step 2: Clear analog faults (0x78 = 0x80) */
  tas5805m_write_byte(TAS5805M_FAULT_CLEAR_REGISTER, TAS5805M_ANALOG_FAULT_CLEAR);

  /* Step 3: Set Digital Volume (0x4C) and Analog Gain (0x54) */
  tas5805m_write_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, tas5805m_hw.raw_vol);
  /* Configure soft volume ramp rate in DIG_VOL_CTRL2 (0x4E): default 0x33 for smooth transitions */
  tas5805m_write_byte(TAS5805M_DIG_VOL_CTRL2_REGISTER, 0x33);
  uint8_t again_idx = tas5805m_hw.again;
  if (again_idx > TAS5805M_MIN_GAIN) again_idx = TAS5805M_MIN_GAIN;
  tas5805m_write_byte(TAS5805M_AGAIN_REGISTER, tas5805m_again[again_idx]);

  /* Step 4: Set DEVICE_CTRL_1 (0x02): Bridge mode (PBTL/BTL), FSW, and Modulation mode */
  uint8_t dev_ctrl1 = 0;
  if (tas5805m_hw.dac_mode == TAS5805M_DAC_MODE_PBTL) {
    dev_ctrl1 |= (1 << 2);
  }
  dev_ctrl1 |= (tas5805m_hw.mod_mode & 0x03);
  dev_ctrl1 |= tas58xx_sw_freq_to_hardware(tas5805m_hw.sw_freq, tas5805m_chip_variant);
  tas5805m_write_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, dev_ctrl1);

  /* Step 5: Read-modify-write ANA_CTRL (0x53) bits [6:5] for Class-D loop bandwidth */
  uint8_t current_ana = 0;
  if (tas5805m_read_byte(TAS5805M_ANA_CTRL_REGISTER, &current_ana) == ESP_OK) {
    current_ana &= ~(0x03 << 5);
    current_ana |= tas58xx_bd_freq_to_hardware(tas5805m_hw.bd_freq, tas5805m_chip_variant);
    tas5805m_write_byte(TAS5805M_ANA_CTRL_REGISTER, current_ana);
  }

  /* Step 6: Configure EQ mode / gang / bypass */
#if defined(CONFIG_DAC_TAS5805M_EQ_SUPPORT)
  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    bool eq_enable = (tas5805m_hw.eq_mode != TAS5805M_EQ_MODE_OFF &&
                      tas5805m_hw.eq_mode != TAS5805M_EQ_MODE_BIAMP_OFF);
    TAS5805M_SET_BOOK_AND_PAGE(TAS5825M_REG_BOOK_5, TAS5825M_REG_BOOK_5_PAGE);
    uint8_t gang_reg = TAS5825M_REG_EQ_GANG_ENABLE;
    uint8_t gang_val[4] = {0x00, 0x00, 0x00, 0x01};
    tas5805m_write_bytes(&gang_reg, 1, gang_val, 4);

    uint8_t bypass_reg = TAS5825M_REG_EQ_BYPASS_ENABLE;
    uint8_t bypass_val[4] = {0x00, 0x00, 0x00, eq_enable ? 0x00 : 0x01};
    tas5805m_write_bytes(&bypass_reg, 1, bypass_val, 4);
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  } else {
    tas5805m_write_byte(TAS5805M_DSP_MISC_REGISTER, (uint8_t)tas5805m_hw.eq_mode);
  }
#endif

  /* Step 7: Write 4-channel Mixer Matrix in Book 0x8C */
  tas5805m_apply_mixer_mode_hw(tas5805m_state.mixer_mode);

  /* Step 8: Write Left and Right Channel Volumes in Book 0x8C */
  tas5805m_apply_channel_gain_hw(TAS5805M_EQ_CHANNELS_LEFT, tas5805m_state.channel_gain_l);
  tas5805m_apply_channel_gain_hw(TAS5805M_EQ_CHANNELS_RIGHT, tas5805m_state.channel_gain_r);

  /* Step 9: Write EQ Biquad Coefficients in Book 0xAA */
#if defined(CONFIG_DAC_TAS5805M_EQ_SUPPORT)
  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    /* On TAS5830M with Gang EQ enabled (0x28 = 1), left channel table drives both channels */
    if (tas5805m_hw.use_eq_profile[TAS5805M_EQ_CHANNELS_LEFT]) {
      tas5805m_apply_eq_profile_hw(TAS5805M_EQ_CHANNELS_LEFT, tas5805m_state.eq_profile[TAS5805M_EQ_CHANNELS_LEFT]);
    } else {
      for (int band = 0; band < TAS5805M_EQ_BANDS; band++) {
        tas5805m_apply_eq_gain_hw(TAS5805M_EQ_CHANNELS_LEFT, band, tas5805m_state.eq_gain_l[band]);
      }
    }
  } else {
    for (int ch = 0; ch < 2; ch++) {
      TAS5805M_EQ_CHANNELS channel = (ch == 0) ? TAS5805M_EQ_CHANNELS_LEFT : TAS5805M_EQ_CHANNELS_RIGHT;
      if (tas5805m_hw.use_eq_profile[ch]) {
        tas5805m_apply_eq_profile_hw(channel, tas5805m_state.eq_profile[ch]);
      } else {
        for (int band = 0; band < TAS5805M_EQ_BANDS; band++) {
          int gain = (ch == 0) ? tas5805m_state.eq_gain_l[band] : tas5805m_state.eq_gain_r[band];
          tas5805m_apply_eq_gain_hw(channel, band, gain);
        }
      }
    }
  }
#endif

  /* Return to Book 0 Page 0 */
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  return ret;
}

// Control function of TAS58xx — mirrors Linux do_work() & tas58xx_trigger() 1:1
esp_err_t tas5805m_ctrl(audio_hal_codec_mode_t mode,
                        audio_hal_ctrl_t ctrl_state) {
  ESP_LOGI(TAG, "%s: Control state: %d", __func__, ctrl_state);

  if (ctrl_state == AUDIO_HAL_CTRL_STOP) {
    ESP_LOGI(TAG, "%s: Stopping DAC -> soft mute -> HI_Z", __func__);
    tas5805m_hw.is_powered = false;
    /* Soft mute before entering Hi-Z so PWM bridge ramps down without pops */
    tas5805m_set_state(TAS5805M_CTRL_PLAY_MUTE);
    vTaskDelay(pdMS_TO_TICKS(6));
    return tas5805m_set_state(TAS5805M_CTRL_HI_Z);
  } else if (ctrl_state == AUDIO_HAL_CTRL_START) {
    /* 1. Wait 5-10ms after I2S clock start (Linux: usleep_range(5000, 10000)) */
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!tas5805m_dsp_booted) {
      ESP_LOGI(TAG, "%s: First start (cold boot) with active I2S clock", __func__);

      /* Send dsp_cfg_preboot (resets DSP & control port in Hi-Z with active I2S clock, sets SDOUT=HIZ) */
      esp_err_t ret = tas58xx_send_cfg(dsp_cfg_preboot, sizeof(dsp_cfg_preboot) / sizeof(dsp_cfg_preboot[0]));
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s: Failed sending dsp_cfg_preboot", __func__);
        return ret;
      }

      /* Read DIE_ID (0x67) to confirm chip model */
      uint8_t die_id = 0;
      if (tas5805m_read_byte(TAS5805M_DIE_ID_REGISTER, &die_id) == ESP_OK) {
        if (die_id == TAS5830M_DIE_ID) {
          tas5805m_chip_variant = TAS58XX_CHIP_TAS5830M;
        } else {
          tas5805m_chip_variant = TAS58XX_CHIP_TAS5805M;
        }
        ESP_LOGI(TAG, "%s: Chip DIE_ID=0x%02x (%s)", __func__, die_id,
                 tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M ? "TAS5830M" : "TAS5805M");
      }

      /* Configure GPIO pins for TAS5830M/TAS5825M (matches Linux dsp_cfg_preboot_gpio_config) */
      if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
        tas58xx_send_cfg(dsp_cfg_preboot_gpio_config,
                         sizeof(dsp_cfg_preboot_gpio_config) / sizeof(dsp_cfg_preboot_gpio_config[0]));
      }

      /* Wait 5-10ms for DSP to boot out of reset with active I2S clocks */
      vTaskDelay(pdMS_TO_TICKS(10));

      /* Refresh all analog & DSP registers while in Hi-Z (Book 0x00, Book 0x8C, Book 0xAA) */
      tas5805m_refresh();

      tas5805m_dsp_booted = true;
    } else {
      ESP_LOGI(TAG, "%s: Warm resume (DSP already booted) - releasing soft-mute", __func__);
    }

    /* Enter PLAY + MUTE (0x0B), wait for clock lock, then unmute to PLAY (0x03) */
    tas5805m_set_state(TAS5805M_CTRL_PLAY_MUTE);
    tas5805m_hw.is_powered = true;
    tas5805m_wait_clock_stable();

    if (!tas5805m_hw.is_muted) {
      return tas5805m_set_state(TAS5805M_CTRL_PLAY);
    }
    return ESP_OK;
  } else {
    ESP_LOGW(TAG, "%s: Unknown control state: %d", __func__, ctrl_state);
    return ESP_FAIL;
  }
}

esp_err_t tas5805m_config_iface(audio_hal_codec_mode_t mode,
                                audio_hal_codec_i2s_iface_t *iface) {
  // TODO
  return ESP_OK;
}

esp_err_t tas5805m_get_dac_mode(TAS5805M_DAC_MODE *mode)
{
    if (mode == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!tas5805m_hw.is_powered) {
        *mode = tas5805m_hw.dac_mode;
        return ESP_OK;
    }
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    uint8_t current_value;
    esp_err_t err = tas5805m_read_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, &current_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(err));
        *mode = tas5805m_hw.dac_mode;
        return err;
    }

    if (current_value & (1 << 2)) {
        *mode = TAS5805M_DAC_MODE_PBTL;
    } else {
        *mode = TAS5805M_DAC_MODE_BTL;
    }

    return ESP_OK;
}

esp_err_t tas5805m_set_dac_mode(TAS5805M_DAC_MODE mode)
{
    ESP_LOGD(TAG, "%s: Setting DAC mode to %d", __func__, mode);
    tas5805m_hw.dac_mode = mode;

    if (!tas5805m_hw.is_powered) {
        return ESP_OK;
    }

    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    bool was_playing = (tas5805m_state.state & TAS5805M_CTRL_PLAY) == TAS5805M_CTRL_PLAY;
    if (was_playing) {
      tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);
    }

    // Read the current value of the register
    uint8_t current_value;
    esp_err_t err = tas5805m_read_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, &current_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(err));
        if (was_playing) tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
        return err;
    }

    // Update bit 2 based on the mode
    if (mode == TAS5805M_DAC_MODE_PBTL) {
        current_value |= (1 << 2);  // Set bit 2 to 1 (PBTL mode)
    } else {
        current_value &= ~(1 << 2); // Clear bit 2 to 0 (BTL mode)
    }

    // Write the updated value back to the register
    int ret = tas5805m_write_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, current_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
    }

    if (was_playing) {
      tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
    }

    return ret;
}

esp_err_t tas5805m_get_modulation_mode(TAS5805M_MOD_MODE *mode, TAS5805M_SW_FREQ *freq, TAS5805M_BD_FREQ *bd_freq)
{
  if (!tas5805m_hw.is_powered) {
    if (mode) *mode = tas5805m_hw.mod_mode;
    if (freq) *freq = tas5805m_hw.sw_freq;
    if (bd_freq) *bd_freq = tas5805m_hw.bd_freq;
    return ESP_OK;
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  uint8_t current_value;
  esp_err_t err = tas5805m_read_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, &current_value);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "%s: Error reading DEVICE_CTRL_1: %s", __func__, esp_err_to_name(err));
      if (mode) *mode = tas5805m_hw.mod_mode;
      if (freq) *freq = tas5805m_hw.sw_freq;
      if (bd_freq) *bd_freq = tas5805m_hw.bd_freq;
      return err;
  }

  // Extract bits 0-1
  if (mode) *mode = (TAS5805M_MOD_MODE)(current_value & 0b00000011);
  // Extract bits 4-6 and translate according to chip variant
  if (freq) *freq = tas58xx_hardware_to_sw_freq(current_value, tas5805m_chip_variant);

  // Read the BD frequency
  err = tas5805m_read_byte(TAS5805M_ANA_CTRL_REGISTER, &current_value);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "%s: Error reading ANA_CTRL: %s", __func__, esp_err_to_name(err));
      if (bd_freq) *bd_freq = tas5805m_hw.bd_freq;
      return err;
  }

  if (bd_freq) *bd_freq = tas58xx_hardware_to_bd_freq(current_value, tas5805m_chip_variant);
  return ESP_OK;
}

esp_err_t tas5805m_set_modulation_mode(TAS5805M_MOD_MODE mode, TAS5805M_SW_FREQ freq, TAS5805M_BD_FREQ bd_freq)
{
  ESP_LOGD(TAG, "%s: Setting modulation to %d, FSW: %d, Class-D bandwidth control: %d", __func__, mode, freq, bd_freq);
  tas5805m_hw.mod_mode = mode;
  tas5805m_hw.sw_freq = freq;
  tas5805m_hw.bd_freq = bd_freq;

  if (!tas5805m_hw.is_powered) {
    return ESP_OK;
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  bool was_playing = (tas5805m_state.state & TAS5805M_CTRL_PLAY) == TAS5805M_CTRL_PLAY;
  if (was_playing) {
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);
  }

  // Read the current value of the register
  uint8_t current_value;
  esp_err_t err = tas5805m_read_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, &current_value);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(err));
      if (was_playing) tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
      return err;
  }

  // Clear bits 0-1 and 4-6
  current_value &= ~((0x07 << 4) | (0x03 << 0));
  // Update bit 0-1 based on the mode
  current_value |= (mode & 0b00000011);
  // Update bits 4-6 based on sw freq translated per chip variant
  current_value |= tas58xx_sw_freq_to_hardware(freq, tas5805m_chip_variant);
  
  // Write the updated value back to the register
  int ret = tas5805m_write_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, current_value);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "%s: Error writing DEVICE_CTRL_1: %s", __func__, esp_err_to_name(ret));
  } 

  // Read-modify-write on ANA_CTRL bits 6:5 (BW_CTL) to preserve other analog bits
  uint8_t current_ana = 0;
  err = tas5805m_read_byte(TAS5805M_ANA_CTRL_REGISTER, &current_ana);
  if (err == ESP_OK) {
    current_ana &= ~(0x03 << 5);
    current_ana |= tas58xx_bd_freq_to_hardware(bd_freq, tas5805m_chip_variant);
    ret = tas5805m_write_byte(TAS5805M_ANA_CTRL_REGISTER, current_ana);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s: Error writing ANA_CTRL: %s", __func__, esp_err_to_name(ret));
    }
  } else {
    ESP_LOGE(TAG, "%s: Error reading ANA_CTRL: %s", __func__, esp_err_to_name(err));
    ret = err;
  }

  if (was_playing) {
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
  }

  return ret;
}

esp_err_t tas5805m_get_again(uint8_t *gain)
{
  if (gain == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!tas5805m_hw.is_powered) {
    *gain = tas5805m_again[tas5805m_hw.again];
    return ESP_OK;
  }
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  int ret = tas5805m_read_byte(TAS5805M_AGAIN_REGISTER, gain);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
    *gain = tas5805m_again[tas5805m_hw.again];
  }
  return ret;
}

esp_err_t tas5805m_set_again(uint8_t gain)
{
  // Gain is inverted!
  if (gain < TAS5805M_MAX_GAIN || gain > TAS5805M_MIN_GAIN)
  {
    ESP_LOGE(TAG, "%s: Invalid gain %d", __func__, gain);
    return ESP_ERR_INVALID_ARG;
  }

  tas5805m_hw.again = gain;
  if (!tas5805m_hw.is_powered) {
    return ESP_OK;
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  bool was_playing = (tas5805m_state.state & TAS5805M_CTRL_PLAY) == TAS5805M_CTRL_PLAY;
  if (was_playing) {
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);
  }

  uint8_t value = tas5805m_again[gain];
  int ret = tas5805m_write_byte(TAS5805M_AGAIN_REGISTER, value);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
  }

  if (was_playing) {
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
  }

  return ret;
}

esp_err_t tas5805m_get_mixer_mode(TAS5805M_MIXER_MODE *mode)
{
  *mode = tas5805m_state.mixer_mode;
  return ESP_OK;
}

static esp_err_t tas5805m_apply_mixer_mode_hw(TAS5805M_MIXER_MODE mode)
{
  uint32_t mixer_l_to_l, mixer_r_to_r, mixer_l_to_r, mixer_r_to_l;
  int ret = ESP_OK;

  switch (mode)
  {
  case MIXER_STEREO:
    mixer_l_to_l = TAS5805M_MIXER_VALUE_0DB;
    mixer_l_to_r = TAS5805M_MIXER_VALUE_MUTE;
    mixer_r_to_l = TAS5805M_MIXER_VALUE_MUTE;
    mixer_r_to_r = TAS5805M_MIXER_VALUE_0DB;
    break;

  case MIXER_STEREO_INVERSE:
    mixer_l_to_l = TAS5805M_MIXER_VALUE_MUTE;
    mixer_l_to_r = TAS5805M_MIXER_VALUE_0DB;
    mixer_r_to_l = TAS5805M_MIXER_VALUE_0DB;
    mixer_r_to_r = TAS5805M_MIXER_VALUE_MUTE;
    break;

  case MIXER_MONO:
    mixer_l_to_l = TAS5805M_MIXER_VALUE_MINUS6DB;
    mixer_r_to_r = TAS5805M_MIXER_VALUE_MINUS6DB;
    mixer_l_to_r = TAS5805M_MIXER_VALUE_MINUS6DB;
    mixer_r_to_l = TAS5805M_MIXER_VALUE_MINUS6DB;
    break;

  case MIXER_LEFT:
    mixer_l_to_l = TAS5805M_MIXER_VALUE_0DB;
    mixer_r_to_r = TAS5805M_MIXER_VALUE_MUTE;
    mixer_l_to_r = TAS5805M_MIXER_VALUE_0DB;
    mixer_r_to_l = TAS5805M_MIXER_VALUE_MUTE;
    break;

  case MIXER_RIGHT:
    mixer_l_to_l = TAS5805M_MIXER_VALUE_MUTE;
    mixer_r_to_r = TAS5805M_MIXER_VALUE_0DB;
    mixer_l_to_r = TAS5805M_MIXER_VALUE_MUTE;
    mixer_r_to_l = TAS5805M_MIXER_VALUE_0DB;
    break;

  default:
    ESP_LOGE(TAG, "%s: Invalid mixer mode %d", __func__, mode);
    return ESP_ERR_INVALID_ARG;
  }

  ret = ret | tas5805m_set_mixer_gain(TAS5805M_MIXER_CHANNEL_LEFT_TO_LEFT, mixer_l_to_l);
  ret = ret | tas5805m_set_mixer_gain(TAS5805M_MIXER_CHANNEL_RIGHT_TO_RIGHT, mixer_r_to_r);
  ret = ret | tas5805m_set_mixer_gain(TAS5805M_MIXER_CHANNEL_LEFT_TO_RIGHT, mixer_l_to_r);
  ret = ret | tas5805m_set_mixer_gain(TAS5805M_MIXER_CHANNEL_RIGHT_TO_LEFT, mixer_r_to_l);
  return ret;
}

esp_err_t tas5805m_set_mixer_mode(TAS5805M_MIXER_MODE mode)
{
  ESP_LOGD(TAG, "%s: Setting mixer mode to %d", __func__, mode);
  if (mode < MIXER_UNKNOWN || mode >= MIXER_MAX) {
    return ESP_ERR_INVALID_ARG;
  }
  tas5805m_state.mixer_mode = mode;

  if (!tas5805m_hw.is_powered) {
    return ESP_OK;
  }

  bool was_playing = (tas5805m_state.state & TAS5805M_CTRL_PLAY) == TAS5805M_CTRL_PLAY;
  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);
  }

  esp_err_t ret = tas5805m_apply_mixer_mode_hw(mode);

  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
  }

  return ret;
}

esp_err_t tas5805m_set_mixer_gain(TAS5805M_MIXER_CHANNELS channel, uint32_t gain)
{
  ESP_LOGD(TAG, "%s: Setting mixer gain for channel %d to 0x%08x", __func__, channel, (unsigned int)gain);
  uint8_t page;
  uint8_t reg;

  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    page = TAS5825M_REG_BOOK_5_PAGE;
    switch (channel)
    {
    case TAS5805M_MIXER_CHANNEL_LEFT_TO_LEFT:
      reg = TAS5825M_REG_LEFT_TO_LEFT_GAIN;
      break;
    case TAS5805M_MIXER_CHANNEL_RIGHT_TO_RIGHT:
      reg = TAS5825M_REG_RIGHT_TO_RIGHT_GAIN;
      break;
    case TAS5805M_MIXER_CHANNEL_LEFT_TO_RIGHT:
      reg = TAS5825M_REG_LEFT_TO_RIGHT_GAIN;
      break;
    case TAS5805M_MIXER_CHANNEL_RIGHT_TO_LEFT:
      reg = TAS5825M_REG_RIGHT_TO_LEFT_GAIN;
      break;
    default:
      ESP_LOGE(TAG, "%s: Invalid mixer channel %d", __func__, channel);
      return ESP_ERR_INVALID_ARG;
    }
  } else {
    page = TAS5805M_REG_BOOK_5_MIXER_PAGE;
    switch (channel)
    {
    case TAS5805M_MIXER_CHANNEL_LEFT_TO_LEFT:
      reg = TAS5805M_REG_LEFT_TO_LEFT_GAIN;
      break;
    case TAS5805M_MIXER_CHANNEL_RIGHT_TO_RIGHT:
      reg = TAS5805M_REG_RIGHT_TO_RIGHT_GAIN;
      break;
    case TAS5805M_MIXER_CHANNEL_LEFT_TO_RIGHT:
      reg = TAS5805M_REG_LEFT_TO_RIGHT_GAIN;
      break;
    case TAS5805M_MIXER_CHANNEL_RIGHT_TO_LEFT:
      reg = TAS5805M_REG_RIGHT_TO_LEFT_GAIN;
      break;
    default:
      ESP_LOGE(TAG, "%s: Invalid mixer channel %d", __func__, channel);
      return ESP_ERR_INVALID_ARG;
    }
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_5, page);
  int ret = tas5805m_write_bytes(&reg, 1, (uint8_t *)&gain, sizeof(gain));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write mixer register 0x%02x: %d", reg, ret);
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO); 
  return ret;
}

static esp_err_t tas5805m_apply_channel_gain_hw(TAS5805M_EQ_CHANNELS channel, int8_t gain_db)
{
  /* Convert dB to linear gain and then to Q9.23 register format */
  float linear = powf(10.0f, ((float)gain_db) / 20.0f);
  uint32_t reg_value = tas5805m_float_to_q9_23(linear);

  uint8_t page;
  uint8_t reg;
  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    page = TAS5825M_REG_BOOK_5_PAGE;
    if (channel == TAS5805M_EQ_CHANNELS_RIGHT) {
      reg = TAS5825M_REG_RIGHT_VOLUME;
    } else {
      reg = TAS5825M_REG_LEFT_VOLUME;
    }
  } else {
    page = TAS5805M_REG_BOOK_5_VOLUME_PAGE;
    if (channel == TAS5805M_EQ_CHANNELS_RIGHT) {
      reg = TAS5805M_REG_RIGHT_VOLUME;
    } else {
      reg = TAS5805M_REG_LEFT_VOLUME;
    }
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_5, page);
  int ret = tas5805m_write_bytes(&reg, 1, (uint8_t *)&reg_value, sizeof(reg_value));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "%s: Failed to write volume register 0x%02x: %s", __func__, reg, esp_err_to_name(ret));
  } else {
    ESP_LOGD(TAG, "%s: Wrote volume register 0x%02x with value 0x%08x", __func__, reg, (unsigned int)reg_value);
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  return ret;
}

// Set output channel volume
esp_err_t tas5805m_set_channel_gain(TAS5805M_EQ_CHANNELS channel, int8_t gain_db)
{
  ESP_LOGD(TAG, "%s: Setting channel %d volume to %d dB", __func__, channel, gain_db);

  if (gain_db < TAS5805M_MIXER_VALUE_MINDB || gain_db > TAS5805M_MIXER_VALUE_MAXDB) {
    ESP_LOGE(TAG, "%s: Invalid gain_db %d, must be between %d and %d", __func__, gain_db, TAS5805M_MIXER_VALUE_MINDB, TAS5805M_MIXER_VALUE_MAXDB);
    return ESP_ERR_INVALID_ARG;
  }

  if (channel == TAS5805M_EQ_CHANNELS_RIGHT) {
    tas5805m_state.channel_gain_r = gain_db;
  } else {
    tas5805m_state.channel_gain_l = gain_db;
  }

  if (!tas5805m_hw.is_powered) {
    return ESP_OK;
  }

  bool was_playing = (tas5805m_state.state & TAS5805M_CTRL_PLAY) == TAS5805M_CTRL_PLAY;
  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);
  }

  esp_err_t ret = tas5805m_apply_channel_gain_hw(channel, gain_db);

  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
  }

  return ret;
}

esp_err_t tas5805m_get_channel_gain(TAS5805M_EQ_CHANNELS channel, int8_t *gain_db)
{
  if (gain_db == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (channel == TAS5805M_EQ_CHANNELS_RIGHT) {
    *gain_db = tas5805m_state.channel_gain_r;
  } else {
    *gain_db = tas5805m_state.channel_gain_l;
  }

  ESP_LOGV(TAG, "%s: Returning cached channel %d gain %d dB", __func__, channel, *gain_db);
  return ESP_OK;
}

esp_err_t tas5805m_clear_faults()
{
  ESP_LOGD(TAG, "%s: Clearing faults", __func__);
  int ret = tas5805m_write_byte(TAS5805M_FAULT_CLEAR_REGISTER, TAS5805M_ANALOG_FAULT_CLEAR);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t tas5805m_get_faults(TAS5805M_FAULT *fault)
{
  int ret = ESP_OK;

  ret = tas5805m_read_byte(TAS5805M_CHAN_FAULT_REGISTER, &(fault->err0));
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
    return ret;
  }

  ret = tas5805m_read_byte(TAS5805M_GLOBAL_FAULT1_REGISTER, &(fault->err1));
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
    return ret;
  }

  ret = tas5805m_read_byte(TAS5805M_GLOBAL_FAULT2_REGISTER, &(fault->err2));
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
    return ret;
  }

  ret= tas5805m_read_byte(TAS5805M_OT_WARNING_REGISTER, &(fault->ot_warn));
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(ret));
    return ret;
  }

  return ret;
}

void tas5805m_decode_faults(TAS5805M_FAULT fault)
{
  if (fault.err0) {
    if (fault.err0 & (1 << 0))  
        ESP_LOGW(TAG, "%s: Right channel over current fault", __func__);

    if (fault.err0 & (1 << 1))
        ESP_LOGW(TAG, "%s: Left channel over current fault", __func__);

    if (fault.err0 & (1 << 2)) 
        ESP_LOGW(TAG, "%s: Right channel DC fault", __func__);

    if (fault.err0 & (1 << 3))  
        ESP_LOGW(TAG, "%s: Left channel DC fault", __func__);
  }

  if (fault.err1) {
    if (fault.err1 & (1 << 0))  
        ESP_LOGW(TAG, "%s: PVDD UV fault", __func__);

    if (fault.err1 & (1 << 1))
        ESP_LOGW(TAG, "%s: PVDD OV fault", __func__);

    if (fault.err1 & (1 << 2)) 
        ESP_LOGW(TAG, "%s: Clock fault", __func__);

    if (fault.err1 & (1 << 6))  
        ESP_LOGW(TAG, "%s: The recent BQ is written failed", __func__);

    if (fault.err1 & (1 << 7))  
        ESP_LOGW(TAG, "%s: Indicate OTP CRC check error", __func__);
  }

  if (fault.err2) {
    if (fault.err2 & (1 << 0))  
        ESP_LOGW(TAG, "%s: Over temperature shut down fault", __func__);
  }

  if (fault.ot_warn) {
    if (fault.ot_warn & (1 << 2))  
        ESP_LOGW(TAG, "%s: Over temperature warning", __func__);
  }
}

/* EQ-related functions and data: compile only when enabled in Kconfig */
#if defined(CONFIG_DAC_TAS5805M_EQ_SUPPORT)

esp_err_t tas5805m_get_eq_mode(TAS5805M_EQ_MODE *mode)
{
  if (mode == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!tas5805m_hw.is_powered) {
    *mode = tas5805m_hw.eq_mode;
    return ESP_OK;
  }

  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5825M_REG_BOOK_5, TAS5825M_REG_BOOK_5_PAGE);
    uint8_t bypass_reg = TAS5825M_REG_EQ_BYPASS_ENABLE;
    uint8_t bypass_val[4] = {0};
    esp_err_t err = tas5805m_read_bytes(&bypass_reg, 1, bypass_val, 4);
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "%s: Error reading EQ bypass register: %s", __func__, esp_err_to_name(err));
      *mode = tas5805m_hw.eq_mode;
      return err;
    }
    // bypass_val[3] == 0 means EQ is enabled (not bypassed)
    *mode = (bypass_val[3] == 0) ? TAS5805M_EQ_MODE_ON : TAS5805M_EQ_MODE_OFF;
    return ESP_OK;
  } else {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    uint8_t value = 0;
    esp_err_t err = tas5805m_read_byte(TAS5805M_DSP_MISC_REGISTER, &value);
    if (err != ESP_OK)
    {
      ESP_LOGE(TAG, "%s: Error during I2C transmission: %s", __func__, esp_err_to_name(err));
      *mode = tas5805m_hw.eq_mode;
      return err;
    }

    // Extract the EQ mode from the value
    *mode = (TAS5805M_EQ_MODE)(value & 0b00001111);
    return ESP_OK;
  }
}

esp_err_t tas5805m_set_eq(bool enable)
{
  ESP_LOGD(TAG, "%s: Setting EQ to %d", __func__, enable);
  tas5805m_hw.eq_mode = enable ? TAS5805M_EQ_MODE_ON : TAS5805M_EQ_MODE_OFF;

  if (!tas5805m_hw.is_powered) {
    return ESP_OK;
  }

  bool was_playing = (tas5805m_state.state & TAS5805M_CTRL_PLAY) == TAS5805M_CTRL_PLAY;
  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);
  }

  esp_err_t ret = ESP_OK;
  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5825M_REG_BOOK_5, TAS5825M_REG_BOOK_5_PAGE);
    uint8_t gang_reg = TAS5825M_REG_EQ_GANG_ENABLE;
    uint8_t gang_val[4] = {0x00, 0x00, 0x00, 0x01};
    tas5805m_write_bytes(&gang_reg, 1, gang_val, 4);

    uint8_t bypass_reg = TAS5825M_REG_EQ_BYPASS_ENABLE;
    uint8_t bypass_val[4] = {0x00, 0x00, 0x00, enable ? 0x00 : 0x01};
    ret = tas5805m_write_bytes(&bypass_reg, 1, bypass_val, 4);
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  } else {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    ret = tas5805m_write_byte(TAS5805M_DSP_MISC_REGISTER, enable ? TAS5805M_CTRL_EQ_ON : TAS5805M_CTRL_EQ_OFF);
  }

  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
  }

  return ret;
}

esp_err_t tas5805m_set_eq_mode(TAS5805M_EQ_MODE mode)
{
  ESP_LOGD(TAG, "%s: Setting EQ MODE to %d", __func__, mode);
  tas5805m_hw.eq_mode = mode;

  if (!tas5805m_hw.is_powered) {
    return ESP_OK;
  }

  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    bool enable = (mode != TAS5805M_EQ_MODE_OFF && mode != TAS5805M_EQ_MODE_BIAMP_OFF);
    return tas5805m_set_eq(enable);
  } else {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    return tas5805m_write_byte(TAS5805M_DSP_MISC_REGISTER, (uint8_t)mode);
  }
}

esp_err_t tas5805m_get_eq_gain(int band, int *gain)
{
  return tas5805m_get_eq_gain_channel(TAS5805M_EQ_CHANNELS_LEFT, band, gain);
}

esp_err_t tas5805m_get_eq_gain_channel(TAS5805M_EQ_CHANNELS channel, int band, int *gain)
{
  switch (channel)
  {
    case TAS5805M_EQ_CHANNELS_RIGHT:
      *gain = tas5805m_state.eq_gain_r[band];
      break;
    default:
      *gain = tas5805m_state.eq_gain_l[band];
      break;
  }
  return ESP_OK;
}

esp_err_t tas5805m_set_eq_gain(int band, int gain) {
  return tas5805m_set_eq_gain_channel(TAS5805M_EQ_CHANNELS_LEFT, band, gain);
}

static esp_err_t tas5805m_apply_eq_gain_hw(TAS5805M_EQ_CHANNELS channel, int band, int gain)
{
  int current_page = 0;
  int ret = ESP_OK;
  int x = gain + TAS5805M_EQ_MAX_DB;
  int y = band * TAS5805M_EQ_KOEF_PER_BAND * TAS5805M_EQ_REG_PER_KOEF;

  const reg_sequence_eq **eq_maps;
  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    eq_maps = tas5825m_eq_registers;
  } else {
    eq_maps = (channel == TAS5805M_EQ_CHANNELS_RIGHT) ? tas5805m_eq_registers_right : tas5805m_eq_registers_left;
  }

  for (int i = 0; i < TAS5805M_EQ_KOEF_PER_BAND * TAS5805M_EQ_REG_PER_KOEF; i += TAS5805M_EQ_REG_PER_KOEF)
  {
      const reg_sequence_eq *reg_value0 = &eq_maps[x][y + i + 0];
      const reg_sequence_eq *reg_value1 = &eq_maps[x][y + i + 1];
      const reg_sequence_eq *reg_value2 = &eq_maps[x][y + i + 2];
      const reg_sequence_eq *reg_value3 = &eq_maps[x][y + i + 3];

      if (reg_value0 == NULL || reg_value1 == NULL || reg_value2 == NULL || reg_value3 == NULL) {
          ESP_LOGW(TAG, "%s: NULL pointer encountered at row[%d]", __func__, y + i);
          continue;
      }

      if (reg_value0->page != current_page) {
        TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_EQ, reg_value0->page);
        current_page = reg_value0->page;
      }

      uint8_t address = reg_value0->offset;
      uint32_t value = reg_value0->value |
                     (reg_value1->value << 8) |
                     (reg_value2->value << 16) |
                     (reg_value3->value << 24);

      ret = ret | tas5805m_write_bytes(&address, 1, (uint8_t *)&value, sizeof(value));
      if (ret != ESP_OK) {
          ESP_LOGE(TAG, "%s: Error writing to register 0x%x", __func__, address);
      }
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  return ret;
}

esp_err_t tas5805m_set_eq_gain_channel(TAS5805M_EQ_CHANNELS channel, int band, int gain)
{
  if (band < 0 || band >= TAS5805M_EQ_BANDS)
  {
    ESP_LOGE(TAG, "%s: Invalid band %d", __func__, band);
    return ESP_ERR_INVALID_ARG;
  }

  if (gain < TAS5805M_EQ_MIN_DB || gain > TAS5805M_EQ_MAX_DB)
  {
    ESP_LOGE(TAG, "%s: Invalid gain %d", __func__, gain);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGD(TAG, "%s: Setting EQ band %d (%d Hz) to gain %d", __func__, band, tas5805m_eq_bands[band], gain);

  if (channel == TAS5805M_EQ_CHANNELS_RIGHT) {
    tas5805m_state.eq_gain_r[band] = gain;
    tas5805m_hw.use_eq_profile[TAS5805M_EQ_CHANNELS_RIGHT] = false;
  } else {
    tas5805m_state.eq_gain_l[band] = gain;
    tas5805m_hw.use_eq_profile[TAS5805M_EQ_CHANNELS_LEFT] = false;
  }

  if (!tas5805m_hw.is_powered) {
    return ESP_OK;
  }

  bool was_playing = (tas5805m_state.state & TAS5805M_CTRL_PLAY) == TAS5805M_CTRL_PLAY;
  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);
  }

  int ret = tas5805m_apply_eq_gain_hw(channel, band, gain);

  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
  }

  return ret;
}

esp_err_t tas5805m_get_eq_profile(TAS5805M_EQ_PROFILE *profile)
{
  return tas5805m_get_eq_profile_channel(TAS5805M_EQ_CHANNELS_LEFT, profile);
}

esp_err_t tas5805m_get_eq_profile_channel(TAS5805M_EQ_CHANNELS channel, TAS5805M_EQ_PROFILE *profile)
{
  *profile = tas5805m_state.eq_profile[channel];
  return ESP_OK;
}

esp_err_t tas5805m_set_eq_profile(TAS5805M_EQ_PROFILE profile) {
  return tas5805m_set_eq_profile_channel(TAS5805M_EQ_CHANNELS_LEFT, profile);
}

static esp_err_t tas5805m_apply_eq_profile_hw(TAS5805M_EQ_CHANNELS channel, TAS5805M_EQ_PROFILE profile)
{
  int current_page = 0;
  int ret = ESP_OK;

  const reg_sequence_eq **eq_maps;
  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    eq_maps = tas5825m_eq_profile_registers;
  } else {
    eq_maps = (channel == TAS5805M_EQ_CHANNELS_RIGHT) ? tas5805m_eq_profile_right_registers : tas5805m_eq_profile_left_registers;
  }

  int x = (uint8_t)profile;
  for (int i = 0; i < TAS5805M_EQ_PROFILE_REG_PER_STEP; i += TAS5805M_EQ_REG_PER_KOEF)
  {
    const reg_sequence_eq *reg_value0 = &eq_maps[x][i + 0];
    const reg_sequence_eq *reg_value1 = &eq_maps[x][i + 1];
    const reg_sequence_eq *reg_value2 = &eq_maps[x][i + 2];
    const reg_sequence_eq *reg_value3 = &eq_maps[x][i + 3];

    if (reg_value0 == NULL || reg_value1 == NULL || reg_value2 == NULL || reg_value3 == NULL) {
        ESP_LOGW(TAG, "%s: NULL pointer encountered at row[%d]", __func__, i);
        continue;
    }

    if (reg_value0->page != current_page) {
        current_page = reg_value0->page;
        TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_EQ, reg_value0->page);
    }

    uint8_t address = reg_value0->offset;
    uint32_t value = reg_value0->value |
                     (reg_value1->value << 8) |
                     (reg_value2->value << 16) |
                     (reg_value3->value << 24);

    ret = ret | tas5805m_write_bytes(&address, 1, (uint8_t *)&value, sizeof(value));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s: Error writing to register 0x%x", __func__, address);
    }
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  return ret;
}

esp_err_t tas5805m_set_eq_profile_channel(TAS5805M_EQ_CHANNELS channel, TAS5805M_EQ_PROFILE profile)
{
  ESP_LOGD(TAG, "%s: Setting EQ profile to %d", __func__, profile);

  int ch_idx = (channel == TAS5805M_EQ_CHANNELS_RIGHT) ? 1 : 0;
  tas5805m_state.eq_profile[ch_idx] = profile;
  tas5805m_hw.use_eq_profile[ch_idx] = true;

  if (!tas5805m_hw.is_powered) {
    return ESP_OK;
  }

  bool was_playing = (tas5805m_state.state & TAS5805M_CTRL_PLAY) == TAS5805M_CTRL_PLAY;
  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);
  }

  int ret = tas5805m_apply_eq_profile_hw(channel, profile);

  if (was_playing) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
  }

  return ret;
}

/* -------------------------
   Biquad coefficient access
   ------------------------- */

/**
 * @brief Get the register offset for a specific biquad coefficient.
 * 
 * Each band has 5 coefficients (B0, B1, B2, A1, A2) stored sequentially.
 * Each coefficient is 4 bytes in Q5.27 format.
 * 
 * @param channel Left or right channel
 * @param band Band index (0-14)
 * @param coef_index Coefficient index (0=B0, 1=B1, 2=B2, 3=A1, 4=A2)
 * @param page Output: page number
 * @param offset Output: register offset
 * @return ESP_OK on success
 */
static esp_err_t tas5805m_get_biquad_register(TAS5805M_EQ_CHANNELS channel, int band, 
                                                int coef_index, uint8_t *page, uint8_t *offset)
{
  if (band < 0 || band >= TAS5805M_EQ_BANDS) {
    ESP_LOGE(TAG, "%s: Invalid band %d", __func__, band);
    return ESP_ERR_INVALID_ARG;
  }
  
  if (coef_index < 0 || coef_index >= TAS5805M_EQ_KOEF_PER_BAND) {
    ESP_LOGE(TAG, "%s: Invalid coefficient index %d", __func__, coef_index);
    return ESP_ERR_INVALID_ARG;
  }

  // Calculate position in the register map
  // We need to look at the data tables to find the pattern
  // From tas5805m_set_eq_gain_channel: gain 0dB is at index TAS5805M_EQ_MAX_DB
  const reg_sequence_eq **eq_maps;
  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    eq_maps = tas5825m_eq_registers;
  } else {
    eq_maps = (channel == TAS5805M_EQ_CHANNELS_RIGHT) ? 
               tas5805m_eq_registers_right : tas5805m_eq_registers_left;
  }
  
  // Use 0dB gain (middle of the range) as reference for current coefficient positions
  int gain_index = TAS5805M_EQ_MAX_DB; // 0dB
  int base_index = band * TAS5805M_EQ_KOEF_PER_BAND * TAS5805M_EQ_REG_PER_KOEF;
  int coef_offset = coef_index * TAS5805M_EQ_REG_PER_KOEF;
  int reg_index = base_index + coef_offset;
  
  const reg_sequence_eq *reg = &eq_maps[gain_index][reg_index];
  *page = reg->page;
  *offset = reg->offset;
  
  return ESP_OK;
}

esp_err_t tas5805m_read_biquad_coefficients(TAS5805M_EQ_CHANNELS channel, int band, 
                                              float *b0, float *b1, float *b2, 
                                              float *a1, float *a2)
{
  if (!b0 || !b1 || !b2 || !a1 || !a2) {
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGD(TAG, "%s: Reading biquad coefficients for channel %d, band %d", 
           __func__, channel, band);

  esp_err_t ret = ESP_OK;
  uint8_t page, offset;
  uint32_t raw_value;
  
  // Read each coefficient
  float *coeffs[] = {b0, b1, b2, a1, a2};
  const char *names[] = {"B0", "B1", "B2", "A1", "A2"};
  
  for (int i = 0; i < TAS5805M_EQ_KOEF_PER_BAND; i++) {
    ret = tas5805m_get_biquad_register(channel, band, i, &page, &offset);
    if (ret != ESP_OK) {
      return ret;
    }
    
    TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_EQ, page);
    
    ret = tas5805m_read_bytes(&offset, 1, (uint8_t *)&raw_value, sizeof(raw_value));
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "%s: Failed to read coefficient %s: %s", 
               __func__, names[i], esp_err_to_name(ret));
      break;
    }
    
    *coeffs[i] = tas5805m_q5_27_to_float(raw_value);
    ESP_LOGD(TAG, "%s: %s = %f (raw: 0x%08X)", __func__, names[i], *coeffs[i], (unsigned int)raw_value);
  }
  
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);
  return ret;
}

esp_err_t tas5805m_write_biquad_coefficients(TAS5805M_EQ_CHANNELS channel, int band,
                                               float b0, float b1, float b2,
                                               float a1, float a2)
{
  ESP_LOGD(TAG, "%s: Writing biquad coefficients for channel %d, band %d", 
           __func__, channel, band);
  ESP_LOGD(TAG, "%s: B0=%f, B1=%f, B2=%f, A1=%f, A2=%f", 
           __func__, b0, b1, b2, a1, a2);

  bool was_playing = (tas5805m_state.state & TAS5805M_CTRL_PLAY) == TAS5805M_CTRL_PLAY;
  if (was_playing) {
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, TAS5805M_CTRL_HI_Z);
  }

  esp_err_t ret = ESP_OK;
  uint8_t current_page = 0;
  uint8_t page, offset;
  uint32_t raw_value;
  
  float coeffs[] = {b0, b1, b2, a1, a2};
  const char *names[] = {"B0", "B1", "B2", "A1", "A2"};
  
  for (int i = 0; i < TAS5805M_EQ_KOEF_PER_BAND; i++) {
    ret = tas5805m_get_biquad_register(channel, band, i, &page, &offset);
    if (ret != ESP_OK) {
      if (was_playing) tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
      return ret;
    }
    
    if (page != current_page) {
      TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_EQ, page);
      current_page = page;
    }
    
    raw_value = tas5805m_float_to_q5_27(coeffs[i]);
    ESP_LOGD(TAG, "%s: Writing %s = %f -> 0x%08X to offset 0x%02X", 
             __func__, names[i], coeffs[i], (unsigned int)raw_value, offset);
    
    ret = tas5805m_write_bytes(&offset, 1, (uint8_t *)&raw_value, sizeof(raw_value));
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "%s: Failed to write coefficient %s: %s", 
               __func__, names[i], esp_err_to_name(ret));
      break;
    }
  }
  
  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);

  if (was_playing) {
    tas5805m_write_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, tas5805m_state.state);
  }

  return ret;
}

#endif /* CONFIG_DAC_TAS5805M_EQ_SUPPORT */

/* -------------------------
   Q9.23 conversions
   ------------------------- */

float tas5805m_q9_23_to_float(uint32_t raw)
{
    uint32_t val = tas5805m_swap_endian_32(raw);
    int32_t signed_val = (int32_t)val;
    float result = (float)signed_val / 8388608.0f; // 2^23
    // ESP_LOGD(TAG, "%s: raw=0x%08X, signed_val=%d -> result=%f",
    //          __func__, (unsigned int)raw, signed_val, result);
    return result;
}

uint32_t tas5805m_float_to_q9_23(float value)
{
    if (value > 255.999999f) value = 255.999999f;
    if (value < -256.0f)     value = -256.0f;

    int32_t fixed_val = (int32_t)(value * (1 << 23));
    uint32_t le_val = tas5805m_swap_endian_32((uint32_t)fixed_val);

    // ESP_LOGD(TAG, "%s: value=%f -> fixed_val=%d, le_val=0x%08X",
    //          __func__, value, fixed_val, (unsigned int)le_val);

    return le_val;
}

float tas5805m_q5_27_to_float(uint32_t raw)
{
    uint32_t val = tas5805m_swap_endian_32(raw);
    int32_t signed_val = (int32_t)val;
    float result = (float)signed_val / 134217728.0f; // 2^27
    // ESP_LOGD(TAG, "%s: raw=0x%08X, signed_val=%d -> result=%f",
    //          __func__, (unsigned int)raw, signed_val, result);
    return result;
}

uint32_t tas5805m_float_to_q5_27(float value)
{
    if (value > 15.999999f) value = 15.999999f;
    if (value < -16.0f)     value = -16.0f;

    int32_t fixed_val = (int32_t)(value * (1 << 27));
    uint32_t le_val = tas5805m_swap_endian_32((uint32_t)fixed_val);

    // ESP_LOGD(TAG, "%s: value=%f -> fixed_val=%d, le_val=0x%08X",
    //          __func__, value, fixed_val, (unsigned int)le_val);

    return le_val;
}

// Utility: swap endian for 32-bit values
uint32_t tas5805m_swap_endian_32(uint32_t val)
{
    return ((val & 0xFF) << 24) |
           ((val & 0xFF00) << 8) |
           ((val & 0xFF0000) >> 8) |
           ((val >> 24) & 0xFF);
}

esp_err_t tas5805m_dump_registers(char *buf, size_t buf_len)
{
  size_t off = 0;
  #define DUMP_PRINT(...) do { \
    if (buf && off < buf_len) { \
      off += snprintf(buf + off, buf_len - off, __VA_ARGS__); \
    } \
  } while(0)

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);

  uint8_t die_id = 0, die_ver = 0, ctrl1 = 0, ctrl2 = 0;
  uint8_t vol = 0, again = 0, ana_ctrl = 0, dac_ctrl = 0;
  uint8_t sap1 = 0, sap2 = 0, sap3 = 0;
  uint8_t f_chan = 0, f_glob1 = 0, f_glob2 = 0, f_warn = 0;

  tas5805m_read_byte(TAS5805M_DIE_ID_REGISTER, &die_id);
  tas5805m_read_byte(0x68, &die_ver);
  tas5805m_read_byte(TAS5805M_DEVICE_CTRL_1_REGISTER, &ctrl1);
  tas5805m_read_byte(TAS5805M_DEVICE_CTRL_2_REGISTER, &ctrl2);
  tas5805m_read_byte(TAS5805M_DIG_VOL_CTRL_REGISTER, &vol);
  tas5805m_read_byte(TAS5805M_AGAIN_REGISTER, &again);
  tas5805m_read_byte(TAS5805M_ANA_CTRL_REGISTER, &ana_ctrl);
  tas5805m_read_byte(0x5E, &dac_ctrl);
  tas5805m_read_byte(0x24, &sap1);
  tas5805m_read_byte(0x25, &sap2);
  tas5805m_read_byte(0x26, &sap3);
  tas5805m_read_byte(TAS5805M_CHAN_FAULT_REGISTER, &f_chan);
  tas5805m_read_byte(TAS5805M_GLOBAL_FAULT1_REGISTER, &f_glob1);
  tas5805m_read_byte(TAS5805M_GLOBAL_FAULT2_REGISTER, &f_glob2);
  tas5805m_read_byte(TAS5805M_OT_WARNING_REGISTER, &f_warn);

  const char *chip_name = (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) ? "TAS5830M" : "TAS5805M";
  bool pbtl = (ctrl1 & (1 << 2)) != 0;
  uint8_t fsw = (ctrl1 >> 4) & 0x07;
  uint8_t mod = ctrl1 & 0x03;

  DUMP_PRINT("=== TAS58xx Hardware Register Dump ===\n");
  DUMP_PRINT("Chip Detected:    %s (DIE_ID=0x%02X, REV=0x%02X)\n", chip_name, die_id, die_ver);
  DUMP_PRINT("I2C Address:      0x%02X\n", TAS5805M_ADDRESS);
  DUMP_PRINT("DAC Powered:      %s\n\n", tas5805m_hw.is_powered ? "YES" : "NO");

  DUMP_PRINT("[Book 0x00 Page 0x00 - Control Registers]\n");
  DUMP_PRINT("  0x02 DEVICE_CTRL_1: 0x%02X  [PBTL=%d (%s), FSW=%d, MOD=%d]\n", 
             ctrl1, pbtl, pbtl ? "PBTL Mono (PARALLEL)" : "BTL Stereo", fsw, mod);
  DUMP_PRINT("  0x03 DEVICE_CTRL_2: 0x%02X  [State: %s (0x%02X)]\n", ctrl2, 
             ctrl2 == 0x03 ? "PLAY" : ctrl2 == 0x02 ? "Hi-Z" : ctrl2 == 0x0B ? "PLAY+MUTE" : ctrl2 == 0x01 ? "SLEEP" : "DEEP_SLEEP", ctrl2);
  DUMP_PRINT("  0x24 SAP_CTRL_1:    0x%02X\n", sap1);
  DUMP_PRINT("  0x25 SAP_CTRL_2:    0x%02X\n", sap2);
  DUMP_PRINT("  0x26 SAP_CTRL_3:    0x%02X\n", sap3);
  DUMP_PRINT("  0x4C DIG_VOL_CTRL:  0x%02X  (%.1f dB)\n", vol, (vol <= 0x30) ? (0x30 - vol) * 0.5f : -(vol - 0x30) * 0.5f);
  DUMP_PRINT("  0x54 AGAIN:         0x%02X  (Index %d = %.1f dB)\n", again, tas5805m_hw.again, -0.5f * tas5805m_hw.again);
  DUMP_PRINT("  0x5E DAC_CTRL:      0x%02X\n", dac_ctrl);
  DUMP_PRINT("  0x5F ANA_CTRL:      0x%02X\n", ana_ctrl);
  DUMP_PRINT("  0x70 CHAN_FAULT:    0x%02X  [L-OC:%d R-OC:%d L-DC:%d R-DC:%d]\n",
             f_chan, (f_chan>>1)&1, f_chan&1, (f_chan>>3)&1, (f_chan>>2)&1);
  DUMP_PRINT("  0x71 GLOBAL_FAULT1: 0x%02X  [CLK_FLT:%d PVDD_UV:%d]\n",
             f_glob1, (f_glob1>>2)&1, (f_glob1>>0)&1);
  DUMP_PRINT("  0x72 GLOBAL_FAULT2: 0x%02X\n", f_glob2);
  DUMP_PRINT("  0x73 OT_WARNING:    0x%02X\n\n", f_warn);

  /* Read DSP Registers from Book 0x8C */
  uint8_t vol_l[4] = {0}, vol_r[4] = {0};
  uint8_t m_l2l[4] = {0}, m_r2l[4] = {0}, m_l2r[4] = {0}, m_r2r[4] = {0};
  uint8_t eq_gang[4] = {0}, eq_byp[4] = {0};

  if (tas5805m_chip_variant == TAS58XX_CHIP_TAS5830M) {
    TAS5805M_SET_BOOK_AND_PAGE(TAS5825M_REG_BOOK_5, TAS5825M_REG_BOOK_5_PAGE);
    uint8_t r;
    r = TAS5825M_REG_LEFT_VOLUME;        tas5805m_read_bytes(&r, 1, vol_l, 4);
    r = TAS5825M_REG_RIGHT_VOLUME;       tas5805m_read_bytes(&r, 1, vol_r, 4);
    r = TAS5825M_REG_LEFT_TO_LEFT_GAIN;  tas5805m_read_bytes(&r, 1, m_l2l, 4);
    r = TAS5825M_REG_RIGHT_TO_LEFT_GAIN; tas5805m_read_bytes(&r, 1, m_r2l, 4);
    r = TAS5825M_REG_LEFT_TO_RIGHT_GAIN; tas5805m_read_bytes(&r, 1, m_l2r, 4);
    r = TAS5825M_REG_RIGHT_TO_RIGHT_GAIN;tas5805m_read_bytes(&r, 1, m_r2r, 4);
    r = TAS5825M_REG_EQ_GANG_ENABLE;     tas5805m_read_bytes(&r, 1, eq_gang, 4);
    r = TAS5825M_REG_EQ_BYPASS_ENABLE;   tas5805m_read_bytes(&r, 1, eq_byp, 4);

    DUMP_PRINT("[Book 0x8C Page 0x0B - TAS5830M DSP Volume & Mixer Registers]\n");
    DUMP_PRINT("  0x0C LEFT_VOLUME:   %02X %02X %02X %02X  (Raw=0x%08X)\n",
               vol_l[0], vol_l[1], vol_l[2], vol_l[3],
               (vol_l[0]<<24)|(vol_l[1]<<16)|(vol_l[2]<<8)|vol_l[3]);
    DUMP_PRINT("  0x10 RIGHT_VOLUME:  %02X %02X %02X %02X  (Raw=0x%08X)\n",
               vol_r[0], vol_r[1], vol_r[2], vol_r[3],
               (vol_r[0]<<24)|(vol_r[1]<<16)|(vol_r[2]<<8)|vol_r[3]);
    DUMP_PRINT("  0x14 MIXER L->L:    %02X %02X %02X %02X\n", m_l2l[0], m_l2l[1], m_l2l[2], m_l2l[3]);
    DUMP_PRINT("  0x18 MIXER R->L:    %02X %02X %02X %02X\n", m_r2l[0], m_r2l[1], m_r2l[2], m_r2l[3]);
    DUMP_PRINT("  0x1C MIXER L->R:    %02X %02X %02X %02X\n", m_l2r[0], m_l2r[1], m_l2r[2], m_l2r[3]);
    DUMP_PRINT("  0x20 MIXER R->R:    %02X %02X %02X %02X\n", m_r2r[0], m_r2r[1], m_r2r[2], m_r2r[3]);
    DUMP_PRINT("  0x28 EQ_GANG:       %02X %02X %02X %02X\n", eq_gang[0], eq_gang[1], eq_gang[2], eq_gang[3]);
    DUMP_PRINT("  0x2C EQ_BYPASS:     %02X %02X %02X %02X\n\n", eq_byp[0], eq_byp[1], eq_byp[2], eq_byp[3]);
  }

  TAS5805M_SET_BOOK_AND_PAGE(TAS5805M_REG_BOOK_CONTROL_PORT, TAS5805M_REG_PAGE_ZERO);

  const char *mix_str = "Unknown";
  switch (tas5805m_state.mixer_mode) {
    case MIXER_STEREO: mix_str = "Stereo"; break;
    case MIXER_STEREO_INVERSE: mix_str = "Stereo Inverse"; break;
    case MIXER_MONO: mix_str = "Mono"; break;
    case MIXER_LEFT: mix_str = "Left only"; break;
    case MIXER_RIGHT: mix_str = "Right only"; break;
    default: break;
  }

  DUMP_PRINT("[Driver In-Memory State]\n");
  DUMP_PRINT("  Volume Slider:      %d%%\n", tas5805m_state.volume);
  DUMP_PRINT("  Mixer Mode:         %d (%s)\n", tas5805m_state.mixer_mode, mix_str);
  DUMP_PRINT("  Channel Gain L:     %d dB\n", tas5805m_state.channel_gain_l);
  DUMP_PRINT("  Channel Gain R:     %d dB\n", tas5805m_state.channel_gain_r);
  DUMP_PRINT("  Muted:              %s\n", tas5805m_hw.is_muted ? "YES" : "NO");

  #undef DUMP_PRINT
  ESP_LOGI(TAG, "\n%s", buf ? buf : "Dump completed");
  return ESP_OK;
}