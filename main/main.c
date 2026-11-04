#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char *TAG = "ROBOT_3GDL_GATO_S3";

// ==============================================================================
// 0. SALIDA SERIAL DUAL ULTRA-CONFIABLE (UART0 + USB SERIAL/JTAG NATIVO COM12)
// ==============================================================================
static void serial_send_raw(const char *buf, size_t len) {
  if (!buf || len == 0)
    return;
  // 1. Transmitir por UART0 (Pines 43/44 si se usa adaptador FTDI/CH340)
  uart_write_bytes(UART_NUM_0, buf, len);
  // 2. Transmitir directamente por FIFO de hardware USB Serial/JTAG (COM12
  // Nativo)
  if (usb_serial_jtag_ll_txfifo_writable()) {
    usb_serial_jtag_ll_write_txfifo((const uint8_t *)buf, (uint32_t)len);
    usb_serial_jtag_ll_txfifo_flush();
  }
  // 3. Salida estándar stdout VFS
  fwrite(buf, 1, len, stdout);
  fflush(stdout);
}

static void serial_printf(const char *fmt, ...) {
  char s_buf[256];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(s_buf, sizeof(s_buf), fmt, args);
  va_end(args);
  if (len > 0) {
    if ((size_t)len > sizeof(s_buf) - 1)
      len = sizeof(s_buf) - 1;
    serial_send_raw(s_buf, (size_t)len);
  }
}
#define printf serial_printf

// ==============================================================================
// 1. CONFIGURACIÓN DE 3 BUSES I2C DEDICADOS E INDEPENDIENTES (ESP32-S3)
//    ➔ 3 ENCODERS AS5600 12-BIT (SIN MULTIPLEXOR)
// ==============================================================================
// Encoder 1 (Motor 1 - Base / Yaw / Eje 1)
#define ENC1_SDA_IO GPIO_NUM_1
#define ENC1_SCL_IO GPIO_NUM_2

// Encoder 2 (Motor 2 - Hombro / Shoulder / Eje 2)
#define ENC2_SDA_IO GPIO_NUM_8
#define ENC2_SCL_IO GPIO_NUM_9

// Encoder 3 (Motor 3 - Codo / Elbow / Eje 3)
#define ENC3_SDA_IO GPIO_NUM_10
#define ENC3_SCL_IO GPIO_NUM_11

// Dirección I2C fija del sensor AS5600
#define AS5600_I2C_ADDR 0x36

// Registros de lectura del sensor magnético AS5600
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_RAW_ANGLE 0x0C // 0x0C (MSB), 0x0D (LSB)
#define AS5600_REG_ANGLE 0x0E     // 0x0E (MSB), 0x0F (LSB)
#define AS5600_REG_AGC 0x1A       // Automatic Gain Control (0-255)
#define AS5600_REG_MAGNITUDE 0x1B // 0x1B (MSB), 0x1C (LSB)

// Máscaras de Estado del Imán (Registro 0x0B STATUS)
#define AS5600_STATUS_MD_BIT (1 << 5) // Magnet Detected (Óptimo)
#define AS5600_STATUS_ML_BIT (1 << 4) // Magnet Low (Débil / Lejos)
#define AS5600_STATUS_MH_BIT (1 << 3) // Magnet High (Saturado / Muy Cerca)

#define NUM_MOTORS 3
#define NUM_AXES NUM_MOTORS
#define NUM_CELLS 9

// ==============================================================================
// 2. CONFIGURACIÓN DE PINES Y TIMERS DE LOS 3 DRIVERS TB6600 (ESP32-S3)
//    ➔ MODO CÁTODO COMÚN (PUL-, DIR-, ENA- A GND ESP32-S3)
// ==============================================================================
// Motor 1 (M1 - Base)
#define M1_PIN_PUL GPIO_NUM_4
#define M1_PIN_DIR GPIO_NUM_5
#define M1_PIN_ENA GPIO_NUM_6
#define M1_LEDC_TIMER LEDC_TIMER_0
#define M1_LEDC_CHANNEL LEDC_CHANNEL_0

// Motor 2 (M2 - Hombro)
#define M2_PIN_PUL GPIO_NUM_15
#define M2_PIN_DIR GPIO_NUM_16
#define M2_PIN_ENA GPIO_NUM_17
#define M2_LEDC_TIMER LEDC_TIMER_1
#define M2_LEDC_CHANNEL LEDC_CHANNEL_1

// Motor 3 (M3 - Codo)
#define M3_PIN_PUL GPIO_NUM_12
#define M3_PIN_DIR GPIO_NUM_13
#define M3_PIN_ENA GPIO_NUM_14
#define M3_LEDC_TIMER LEDC_TIMER_2
#define M3_LEDC_CHANNEL LEDC_CHANNEL_2

// Efector Final: Gripper / Electroimán para Fichas del Gato
// (Asignado a GPIO 7 libre en ESP32-S3 para no colisionar con M2_PIN_PUL GPIO
// 15)
#define PIN_TOOL_GRIPPER GPIO_NUM_7

#define STEPPER_LEDC_MODE LEDC_LOW_SPEED_MODE
#define STEPPER_LEDC_RES LEDC_TIMER_10_BIT

// Configuración de micropasos de los drivers TB6600:
// 4 Micropasos (800 pulsos/rev) -> SW1=ON, SW2=OFF, SW3=OFF
// Corriente nominal 1.5A RMS / 1.7A Pico -> SW4=ON, SW5=ON, SW6=OFF
#define STEPPER_MICROSTEPS_PER_REV 800.0f

// ==============================================================================
// 3. PERSISTENCIA EN MEMORIA FLASH NVS (CALIBRACIÓN Y POSICIONES)
// ==============================================================================
#define NVS_NAMESPACE "as5600_cfg"
#define NVS_KEY_ZERO_PREFIX "zero_off_" // zero_off_1, zero_off_2, zero_off_3
#define NVS_KEY_POS_PREFIX "pos_m"      // pos_m1_1..5, etc.
#define MAX_PRESET_SLOTS 5

// ==============================================================================
// 4. ESTRUCTURAS DE DATOS Y VARIABLES GLOBALES (CINEMÁTICA REDUCTOR 31:1)
// ==============================================================================
#define GEAR_RATIO_CYCLOIDAL                                                   \
  31.0f // 31 vueltas de motor NEMA 17 = 1 vuelta de articulación (360°)

typedef struct {
  uint16_t raw_angle;        // 0 a 4095 (12 bits del AS5600)
  float raw_deg;             // 0.00° a 359.91° en rotor del motor NEMA 17
  float angle_deg;           // Ángulo en eje de motor
  int32_t full_turns;        // Vueltas completas de motor NEMA 17 acumuladas
  float multiturn_total_deg; // Grados totales acumulados del motor (turns * 360
                             // + raw)
  float joint_angle_deg; // Ángulo de salida del reductor 31:1 normalizado (0 a
                         // 360°)
  float calibrated_deg;  // Ángulo continuo del reductor con offset de cero
                         // aplicado
  float motor_turns_count; // Contador de vueltas del motor NEMA 17 (0.0 a 31.0)
  uint8_t status_reg;      // Registro STATUS (0x0B)
  bool magnet_detected;    // MD
  bool magnet_too_weak;    // ML
  bool magnet_too_strong;  // MH
  uint8_t agc;             // Ganancia AGC (0 - 255)
  uint16_t magnitude;      // Magnitud CORDIC
  bool valid;              // Comunicación I2C exitosa
} as5600_reading_t;

typedef enum {
  MOTOR_STATE_DISABLED = 0,
  MOTOR_STATE_IN_POSITION,
  MOTOR_STATE_MOVING_CW,
  MOTOR_STATE_MOVING_CCW,
  MOTOR_STATE_BRAKING
} motor_state_t;

typedef struct {
  float j1;
  float j2;
  float j3;
} robot_pose_t;

// Variables de Calibración y Control (Eje de salida del Reductor 31:1)
static float g_gear_ratio[NUM_MOTORS] = {
    GEAR_RATIO_CYCLOIDAL, GEAR_RATIO_CYCLOIDAL, GEAR_RATIO_CYCLOIDAL};
static float g_zero_offset_deg[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static float g_zero_raw_deg[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static float g_target_deg[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static bool g_motor_enabled[NUM_MOTORS] = {
    false, false, false}; // Inicio seguro: motores sin torque al arrancar
static float g_saved_slots[NUM_MOTORS][MAX_PRESET_SLOTS] = {
    {0.0f, 45.0f, 90.0f, 180.0f, 270.0f},
    {0.0f, 45.0f, 90.0f, 180.0f, 270.0f},
    {0.0f, 45.0f, 90.0f, 180.0f, 270.0f}};

// Parámetros de Control PID y Rampas adaptados a Reductor Cicloidal 31:1 (50 Hz)
// Kp=35.00, Ki=0.05, Kd=0.00 (sin inyección de ruido), Banda Muerta con Histéresis
// Proporciona movimiento fluido, potente y 100% libre de tics y vibraciones
static float g_kp[NUM_MOTORS] = {35.0f, 35.0f, 35.0f};
static float g_ki[NUM_MOTORS] = {0.05f, 0.05f, 0.05f};
static float g_kd[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static float g_integral_err[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static float g_prev_err[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static float g_tolerance_deg[NUM_MOTORS] = {0.60f, 0.60f, 1.25f}; // M3 = 1.25° para absorber holgura mecánica del codo
static float g_max_speed_hz = 2600.0f;
static float g_min_speed_hz = 35.0f;
static float g_accel_hz_per_sec = 2600.0f;

// Límites angulares de seguridad configurables (salida del reductor en grados)
// Límites estrictos del robot:
// M1: [-60.0°, +60.0°]
// M2: [-60.0°, +60.0°]
// M3: [  0.0°, +45.0°] (ni más ni menos)
static float g_limit_min_deg[NUM_MOTORS] = {-60.0f, -60.0f, 0.0f};
static float g_limit_max_deg[NUM_MOTORS] = {60.0f, 60.0f, 85.0f};

// Telemetría y Estado Compartido
static as5600_reading_t g_last_reading[NUM_MOTORS] = {0};
static motor_state_t g_motor_state[NUM_MOTORS] = {
    MOTOR_STATE_DISABLED, MOTOR_STATE_DISABLED, MOTOR_STATE_DISABLED};
static int32_t g_current_speed_hz[NUM_MOTORS] = {0, 0, 0};
static float g_current_error[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static bool g_gripper_active = false;
static SemaphoreHandle_t g_data_mutex = NULL;

// Posiciones calculadas para el Tablero de 3x3 del Gato (J1, J2, J3) en Z=-29.1 cm (mesa)
// Adaptadas estrictamente a M1 [-60, 60], M2 [-60, 60], M3 [0, 45]:
static robot_pose_t g_cell_poses[NUM_CELLS] = {
    {3.42f, -34.87f, 0.00f},   // Casilla 1 (Fila 1, Col 1)
    {0.00f, -34.92f, 0.00f},   // Casilla 2 (Fila 1, Col 2)
    {-3.42f, -34.87f, 0.00f},  // Casilla 3 (Fila 1, Col 3)
    {3.63f, -45.20f, 16.66f},  // Casilla 4 (Fila 2, Col 1)
    {0.00f, -45.86f, 17.83f},  // Casilla 5 (Fila 2, Col 2 - Centro Tablero)
    {-3.63f, -45.20f, 16.66f}, // Casilla 6 (Fila 2, Col 3)
    {3.88f, -58.76f, 39.33f},  // Casilla 7 (Fila 3, Col 1)
    {0.00f, -59.11f, 39.87f},  // Casilla 8 (Fila 3, Col 2)
    {-3.88f, -58.76f, 39.33f}  // Casilla 9 (Fila 3, Col 3)
};

// Posiciones de Aproximación Elevada (Hover) en Z=-25.0 cm
static robot_pose_t g_cell_hover_poses[NUM_CELLS] = {
    {3.42f, -31.10f, 0.00f},   // Casilla 1 Hover
    {0.00f, -31.15f, 0.00f},   // Casilla 2 Hover
    {-3.42f, -31.10f, 0.00f},  // Casilla 3 Hover
    {3.63f, -44.70f, 23.70f},  // Casilla 4 Hover
    {0.00f, -44.80f, 23.70f},  // Casilla 5 Hover
    {-3.63f, -44.70f, 23.70f}, // Casilla 6 Hover
    {3.88f, -56.70f, 43.60f},  // Casilla 7 Hover
    {0.00f, -56.80f, 43.60f},  // Casilla 8 Hover
    {-3.88f, -56.70f, 43.60f}  // Casilla 9 Hover
};

static robot_pose_t g_home_pose = {0.0f, 0.0f, 0.0f};
static robot_pose_t g_pick_pose = {-14.47f, -36.40f, 0.00f}; // Dispensador de Fichas

// Mapeo de pines para los 3 buses I2C dedicados
typedef struct {
  gpio_num_t sda_io;
  gpio_num_t scl_io;
} i2c_bus_pins_t;

static i2c_bus_pins_t g_encoder_buses[NUM_MOTORS] = {
    {.sda_io = ENC1_SDA_IO, .scl_io = ENC1_SCL_IO},
    {.sda_io = ENC2_SDA_IO, .scl_io = ENC2_SCL_IO},
    {.sda_io = ENC3_SDA_IO, .scl_io = ENC3_SCL_IO}};

// ==============================================================================
// 5. CONTROLADOR I2C DE BAJO NIVEL INDEPENDIENTE POR PINES (OPEN-DRAIN +
// PULLUP)
//    ➔ ~50 kHz CON RECUPERACIÓN DE BUS Y AUTO-DETECCIÓN DE INVERSIÓN SDA/SCL
// ==============================================================================
static inline void soft_i2c_delay(void) {
  esp_rom_delay_us(
      12); // ~40 kHz óptimo para resistencias pull-up internas y tiempo RC
}

static inline void soft_i2c_scl_high(gpio_num_t scl) {
  gpio_set_level(scl, 1);
  int timeout = 100;
  while (!gpio_get_level(scl) && timeout--) {
    esp_rom_delay_us(1);
  }
  soft_i2c_delay();
}

static inline void soft_i2c_scl_low(gpio_num_t scl) {
  gpio_set_level(scl, 0);
  soft_i2c_delay();
}

static inline void soft_i2c_sda_high(gpio_num_t sda) {
  gpio_set_level(sda, 1);
  int timeout = 100;
  while (!gpio_get_level(sda) && timeout--) {
    esp_rom_delay_us(1);
  }
  soft_i2c_delay();
}

static inline void soft_i2c_sda_low(gpio_num_t sda) {
  gpio_set_level(sda, 0);
  soft_i2c_delay();
}

static void soft_i2c_bus_recover(gpio_num_t sda, gpio_num_t scl) {
  gpio_set_level(sda, 1);
  esp_rom_delay_us(10);
  for (int i = 0; i < 9; i++) {
    if (gpio_get_level(sda))
      break;
    gpio_set_level(scl, 0);
    esp_rom_delay_us(10);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(10);
  }
  // Condición STOP
  gpio_set_level(sda, 0);
  esp_rom_delay_us(10);
  gpio_set_level(scl, 1);
  esp_rom_delay_us(10);
  gpio_set_level(sda, 1);
  esp_rom_delay_us(10);
}

static esp_err_t soft_i2c_init_bus(gpio_num_t sda, gpio_num_t scl) {
  uint64_t mask = (1ULL << sda) | (1ULL << scl);
  gpio_config_t conf = {
      .pin_bit_mask = mask,
      .mode = GPIO_MODE_INPUT_OUTPUT_OD,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t err = gpio_config(&conf);
  if (err != ESP_OK)
    return err;
  gpio_set_level(sda, 1);
  gpio_set_level(scl, 1);
  esp_rom_delay_us(50);
  soft_i2c_bus_recover(sda, scl);
  return ESP_OK;
}

static void soft_i2c_start(gpio_num_t sda, gpio_num_t scl) {
  soft_i2c_sda_high(sda);
  soft_i2c_scl_high(scl);
  soft_i2c_sda_low(sda);
  soft_i2c_scl_low(scl);
}

static void soft_i2c_stop(gpio_num_t sda, gpio_num_t scl) {
  soft_i2c_sda_low(sda);
  soft_i2c_scl_high(scl);
  soft_i2c_sda_high(sda);
}

static bool soft_i2c_write_byte(gpio_num_t sda, gpio_num_t scl, uint8_t byte) {
  for (int i = 0; i < 8; i++) {
    if (byte & 0x80) {
      soft_i2c_sda_high(sda);
    } else {
      soft_i2c_sda_low(sda);
    }
    soft_i2c_scl_high(scl);
    soft_i2c_scl_low(scl);
    byte <<= 1;
  }
  // Leer ACK: liberar SDA sin bloquear si el esclavo ya lo tiene en LOW
  gpio_set_level(sda, 1);
  soft_i2c_delay();
  soft_i2c_scl_high(scl);
  bool ack = (gpio_get_level(sda) == 0);
  soft_i2c_scl_low(scl);
  return ack;
}

static uint8_t soft_i2c_read_byte(gpio_num_t sda, gpio_num_t scl, bool ack) {
  uint8_t byte = 0;
  gpio_set_level(sda, 1); // Liberar SDA para lectura
  soft_i2c_delay();
  for (int i = 0; i < 8; i++) {
    soft_i2c_scl_high(scl);
    byte = (byte << 1) | (gpio_get_level(sda) ? 1 : 0);
    soft_i2c_scl_low(scl);
  }
  if (ack) {
    soft_i2c_sda_low(sda);
  } else {
    soft_i2c_sda_high(sda);
  }
  soft_i2c_scl_high(scl);
  soft_i2c_scl_low(scl);
  gpio_set_level(sda, 1);
  return byte;
}

static esp_err_t as5600_read_bytes_bus(gpio_num_t sda, gpio_num_t scl,
                                       uint8_t reg_addr, uint8_t *data,
                                       size_t len) {
  // Verificar que el bus no esté en corto a GND antes de iniciar (evita falsos
  // ACK en 0x00)
  gpio_set_level(sda, 1);
  gpio_set_level(scl, 1);
  esp_rom_delay_us(15);
  if (gpio_get_level(sda) == 0) {
    soft_i2c_bus_recover(sda, scl);
    if (gpio_get_level(sda) == 0) {
      return ESP_ERR_INVALID_STATE;
    }
  }
  if (gpio_get_level(scl) == 0) {
    return ESP_ERR_INVALID_STATE;
  }

  soft_i2c_start(sda, scl);
  if (!soft_i2c_write_byte(sda, scl, (AS5600_I2C_ADDR << 1) | 0)) {
    soft_i2c_stop(sda, scl);
    return ESP_ERR_NOT_FOUND;
  }
  if (!soft_i2c_write_byte(sda, scl, reg_addr)) {
    soft_i2c_stop(sda, scl);
    return ESP_FAIL;
  }
  soft_i2c_start(sda, scl);
  if (!soft_i2c_write_byte(sda, scl, (AS5600_I2C_ADDR << 1) | 1)) {
    soft_i2c_stop(sda, scl);
    return ESP_ERR_NOT_FOUND;
  }
  for (size_t i = 0; i < len; i++) {
    bool ack = (i + 1 < len);
    data[i] = soft_i2c_read_byte(sda, scl, ack);
  }
  soft_i2c_stop(sda, scl);
  return ESP_OK;
}

static esp_err_t as5600_read_u16_bus(gpio_num_t sda, gpio_num_t scl,
                                     uint8_t reg_addr, uint16_t *val) {
  uint8_t buf[2] = {0};
  esp_err_t err = as5600_read_bytes_bus(sda, scl, reg_addr, buf, 2);
  if (err == ESP_OK) {
    *val = ((uint16_t)(buf[0] & 0x0F) << 8) | (uint16_t)buf[1];
  }
  return err;
}

static inline float normalize_angle_deg(float deg) {
  deg = fmodf(deg, 360.0f);
  if (deg < 0.0f) {
    deg += 360.0f;
  }
  return deg;
}

// Variables estáticas de seguimiento multi-vuelta y filtrado de los encoders
// AS5600
static float s_prev_raw_deg[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static int32_t s_accum_turns[NUM_MOTORS] = {0, 0, 0};
static bool s_first_read[NUM_MOTORS] = {true, true, true};
static float s_filtered_deg[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
static as5600_reading_t s_last_valid[NUM_MOTORS] = {0};
static int s_consecutive_errs[NUM_MOTORS] = {0};

// Resetea el conteo de vueltas acumuladas para sincronización limpia de Cero
// absoluto en Flash
static void as5600_reset_zero_calibration(int motor_idx) {
  if (motor_idx >= 0 && motor_idx < NUM_MOTORS) {
    float raw = g_last_reading[motor_idx].valid
                    ? g_last_reading[motor_idx].raw_deg
                    : s_last_valid[motor_idx].raw_deg;
    float ratio = g_gear_ratio[motor_idx] > 0.001f
                      ? g_gear_ratio[motor_idx]
                      : GEAR_RATIO_CYCLOIDAL;
    g_zero_raw_deg[motor_idx] = raw;
    g_zero_offset_deg[motor_idx] = raw / ratio;
    s_accum_turns[motor_idx] = 0;
    s_prev_raw_deg[motor_idx] = raw;
    s_first_read[motor_idx] = false;
    s_filtered_deg[motor_idx] = 0.0f;
  }
}

static esp_err_t as5600_read_motor(int motor_idx, as5600_reading_t *out) {
  memset(out, 0, sizeof(as5600_reading_t));
  if (motor_idx < 0 || motor_idx >= NUM_MOTORS) {
    out->valid = false;
    return ESP_ERR_INVALID_ARG;
  }

  gpio_num_t sda = g_encoder_buses[motor_idx].sda_io;
  gpio_num_t scl = g_encoder_buses[motor_idx].scl_io;

  // 1. Registro STATUS (0x0B) (con auto-detección en caliente si SDA y SCL
  // están invertidos)
  uint8_t status = 0;
  esp_err_t err =
      as5600_read_bytes_bus(sda, scl, AS5600_REG_STATUS, &status, 1);
  if (err != ESP_OK) {
    err = as5600_read_bytes_bus(scl, sda, AS5600_REG_STATUS, &status, 1);
    if (err == ESP_OK) {
      g_encoder_buses[motor_idx].sda_io = scl;
      g_encoder_buses[motor_idx].scl_io = sda;
      sda = g_encoder_buses[motor_idx].sda_io;
      scl = g_encoder_buses[motor_idx].scl_io;
    } else {
      // Rechazo de glitch transitorio I2C
      s_consecutive_errs[motor_idx]++;
      if (s_consecutive_errs[motor_idx] <= 5 && s_last_valid[motor_idx].valid) {
        *out = s_last_valid[motor_idx];
        return ESP_OK;
      }
      out->valid = false;
      return err;
    }
  }
  out->status_reg = status;
  out->magnet_detected = (status & AS5600_STATUS_MD_BIT) != 0;
  out->magnet_too_weak = (status & AS5600_STATUS_ML_BIT) != 0;
  out->magnet_too_strong = (status & AS5600_STATUS_MH_BIT) != 0;

  // 2. Registro RAW_ANGLE (0x0C)
  uint16_t raw = 0;
  err = as5600_read_u16_bus(sda, scl, AS5600_REG_RAW_ANGLE, &raw);
  if (err != ESP_OK) {
    s_consecutive_errs[motor_idx]++;
    if (s_consecutive_errs[motor_idx] <= 5 && s_last_valid[motor_idx].valid) {
      *out = s_last_valid[motor_idx];
      return ESP_OK;
    }
    out->valid = false;
    return err;
  }
  s_consecutive_errs[motor_idx] = 0;

  out->raw_angle = raw;
  out->raw_deg = ((float)raw * 360.0f) / 4096.0f;
  out->angle_deg = out->raw_deg;

  // 3. Desenvolvimiento multi-vuelta continuo de las 31 revoluciones del NEMA
  // 17
  if (s_first_read[motor_idx]) {
    s_prev_raw_deg[motor_idx] = out->raw_deg;
    s_first_read[motor_idx] = false;

    // Sincronización circular precisa +-180° motor respecto al cero calibrado en NVS:
    float motor_diff = out->raw_deg - g_zero_raw_deg[motor_idx];
    while (motor_diff > 180.0f) motor_diff -= 360.0f;
    while (motor_diff < -180.0f) motor_diff += 360.0f;

    float raw_diff = out->raw_deg - g_zero_raw_deg[motor_idx];
    s_accum_turns[motor_idx] = (int32_t)roundf((motor_diff - raw_diff) / 360.0f);
  } else {
    float diff = out->raw_deg - s_prev_raw_deg[motor_idx];
    if (diff > 180.0f) {
      s_accum_turns[motor_idx]--;
    } else if (diff < -180.0f) {
      s_accum_turns[motor_idx]++;
    }
    s_prev_raw_deg[motor_idx] = out->raw_deg;
  }

  out->full_turns = s_accum_turns[motor_idx];
  out->multiturn_total_deg = ((float)out->full_turns * 360.0f) + out->raw_deg;

  // 4. Cinemática del Reductor 31:1 (31 vueltas de motor = 360° en la
  // articulación de salida)
  float ratio = g_gear_ratio[motor_idx];
  if (ratio <= 0.001f)
    ratio = GEAR_RATIO_CYCLOIDAL;
  float uncalibrated_joint_deg = out->multiturn_total_deg / ratio;
  float instant_calibrated =
      uncalibrated_joint_deg - g_zero_offset_deg[motor_idx];

  // Filtro Digital IIR / EMA Pasa-Bajas adaptativo:
  // Para M3 (Codo con holgura o brazo flojo): Alpha = 0.20 para suprimir micro-vibraciones mecánicas
  // Para M1 y M2: Alpha = 0.60
  float alpha = (motor_idx == 2) ? 0.20f : 0.60f;
  if (s_first_read[motor_idx] || s_filtered_deg[motor_idx] == 0.0f) {
    s_filtered_deg[motor_idx] = instant_calibrated;
  } else {
    s_filtered_deg[motor_idx] =
        (alpha * instant_calibrated) + ((1.0f - alpha) * s_filtered_deg[motor_idx]);
  }
  out->calibrated_deg = s_filtered_deg[motor_idx];
  out->joint_angle_deg = out->calibrated_deg; // Ángulo continuo con signo real
                                              // (sin mutilar a 0-360)
  out->motor_turns_count =
      (out->multiturn_total_deg - (g_zero_offset_deg[motor_idx] * ratio)) /
      360.0f;

  // 5. Ganancia AGC (0x1A) y Magnitud CORDIC (0x1B)
  uint8_t agc = 0;
  as5600_read_bytes_bus(sda, scl, AS5600_REG_AGC, &agc, 1);
  out->agc = agc;

  uint16_t mag = 0;
  as5600_read_u16_bus(sda, scl, AS5600_REG_MAGNITUDE, &mag);
  out->magnitude = mag;

  out->valid = true;
  s_last_valid[motor_idx] = *out;
  return ESP_OK;
}

// ==============================================================================
// 6. INICIALIZACIÓN DRIVERS TB6600 (LEDC HARDWARE PWM DESACOPLADO)
// ==============================================================================
static esp_err_t steppers_hardware_init(void) {
  uint64_t pin_mask = (1ULL << M1_PIN_DIR) | (1ULL << M1_PIN_ENA) |
                      (1ULL << M2_PIN_DIR) | (1ULL << M2_PIN_ENA) |
                      (1ULL << M3_PIN_DIR) | (1ULL << M3_PIN_ENA) |
                      (1ULL << PIN_TOOL_GRIPPER);

  gpio_config_t io_conf = {
      .pin_bit_mask = pin_mask,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t err = gpio_config(&io_conf);
  if (err != ESP_OK)
    return err;

  gpio_set_level(M1_PIN_DIR, 0);
  gpio_set_level(M1_PIN_ENA, 0); // 0 = ENA activo cátodo común (Energizado con torque firme al arranque)
  gpio_set_level(M2_PIN_DIR, 0);
  gpio_set_level(M2_PIN_ENA, 0);
  gpio_set_level(M3_PIN_DIR, 0);
  gpio_set_level(M3_PIN_ENA, 0);
  gpio_set_level(PIN_TOOL_GRIPPER, 0);

  // Configurar 3 Timers LEDC independientes (Timer 0, Timer 1, Timer 2)
  // Garantiza 0% interferencia de velocidad entre ejes
  const ledc_timer_t timers[NUM_MOTORS] = {M1_LEDC_TIMER, M2_LEDC_TIMER,
                                           M3_LEDC_TIMER};
  for (int i = 0; i < NUM_MOTORS; i++) {
    ledc_timer_config_t ledc_timer = {.speed_mode = STEPPER_LEDC_MODE,
                                      .timer_num = timers[i],
                                      .duty_resolution = STEPPER_LEDC_RES,
                                      .freq_hz = 1000,
                                      .clk_cfg = LEDC_AUTO_CLK};
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK)
      return err;
  }

  // Configurar Canales LEDC para M1, M2 y M3
  const int pul_pins[NUM_MOTORS] = {M1_PIN_PUL, M2_PIN_PUL, M3_PIN_PUL};
  const ledc_channel_t channels[NUM_MOTORS] = {M1_LEDC_CHANNEL, M2_LEDC_CHANNEL,
                                               M3_LEDC_CHANNEL};
  for (int i = 0; i < NUM_MOTORS; i++) {
    ledc_channel_config_t ch_cfg = {.speed_mode = STEPPER_LEDC_MODE,
                                    .channel = channels[i],
                                    .timer_sel = timers[i],
                                    .intr_type = LEDC_INTR_DISABLE,
                                    .gpio_num = pul_pins[i],
                                    .duty = 0,
                                    .hpoint = 0};
    err = ledc_channel_config(&ch_cfg);
    if (err != ESP_OK)
      return err;
  }

  return ESP_OK;
}

static void stepper_set_hw_speed(int motor_idx, uint32_t speed_hz, int dir) {
  if (motor_idx < 0 || motor_idx >= NUM_MOTORS)
    return;

  static const int dir_pins[NUM_MOTORS] = {M1_PIN_DIR, M2_PIN_DIR, M3_PIN_DIR};
  static const ledc_channel_t channels[NUM_MOTORS] = {
      M1_LEDC_CHANNEL, M2_LEDC_CHANNEL, M3_LEDC_CHANNEL};
  static const ledc_timer_t timers[NUM_MOTORS] = {M1_LEDC_TIMER, M2_LEDC_TIMER,
                                                  M3_LEDC_TIMER};

  static uint32_t s_last_speed[NUM_MOTORS] = {0, 0, 0};
  static int s_last_dir[NUM_MOTORS] = {-1, -1, -1};

  if (speed_hz < 10) {
    if (s_last_speed[motor_idx] != 0) {
      ledc_set_duty(STEPPER_LEDC_MODE, channels[motor_idx], 0);
      ledc_update_duty(STEPPER_LEDC_MODE, channels[motor_idx]);
      s_last_speed[motor_idx] = 0;
    }
  } else {
    if (speed_hz > 3000)
      speed_hz = 3000;

    if (dir != s_last_dir[motor_idx]) {
      gpio_set_level(dir_pins[motor_idx], dir);
      s_last_dir[motor_idx] = dir;
    }

    // Solo actualizar frecuencia y duty cuando sea necesario
    if (speed_hz != s_last_speed[motor_idx]) {
      ledc_set_freq(STEPPER_LEDC_MODE, timers[motor_idx], speed_hz);
      if (s_last_speed[motor_idx] == 0) {
        // Solo encender tren de pulsos (50% duty) una vez al arrancar desde cero
        ledc_set_duty(STEPPER_LEDC_MODE, channels[motor_idx], 512);
        ledc_update_duty(STEPPER_LEDC_MODE, channels[motor_idx]);
      }
      s_last_speed[motor_idx] = speed_hz;
    }
  }
}

static void stepper_set_enable(int motor_idx, bool enable) {
  if (motor_idx < 0 || motor_idx >= NUM_MOTORS)
    return;
  static const int ena_pins[NUM_MOTORS] = {M1_PIN_ENA, M2_PIN_ENA, M3_PIN_ENA};
  gpio_set_level(ena_pins[motor_idx],
                 enable ? 0 : 1); // 0 = ENA activo cátodo común
}

// ==============================================================================
// 7. PERSISTENCIA NVS (OFFSETS DE CERO Y RANURAS DE MEMORIA)
// ==============================================================================
static void nvs_load_settings(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
  if (err == ESP_OK) {
    for (int m = 0; m < NUM_MOTORS; m++) {
      float ratio = g_gear_ratio[m] > 0.001f ? g_gear_ratio[m] : GEAR_RATIO_CYCLOIDAL;
      char key[20];
      snprintf(key, sizeof(key), "zero_raw_%d", m + 1);
      int32_t raw_millideg = 0;
      if (nvs_get_i32(handle, key, &raw_millideg) == ESP_OK) {
        g_zero_raw_deg[m] = (float)raw_millideg / 1000.0f;
        g_zero_offset_deg[m] = g_zero_raw_deg[m] / ratio;
      } else {
        snprintf(key, sizeof(key), "zero_off_%d", m + 1);
        int32_t raw_offset = 0;
        if (nvs_get_i32(handle, key, &raw_offset) == ESP_OK) {
          g_zero_offset_deg[m] = (float)raw_offset / 100.0f;
          g_zero_raw_deg[m] = g_zero_offset_deg[m] * ratio;
        } else {
          snprintf(key, sizeof(key), "zero_j%d", m + 1);
          if (nvs_get_i32(handle, key, &raw_offset) == ESP_OK) {
            g_zero_offset_deg[m] = (float)raw_offset / 100.0f;
            g_zero_raw_deg[m] = g_zero_offset_deg[m] * ratio;
          }
        }
      }
      ESP_LOGI(TAG, "Motor %d Offset de Cero recuperado: Raw=%.2f deg | Reductor=%.2f deg",
               m + 1, g_zero_raw_deg[m], g_zero_offset_deg[m]);

      // Cargar Límites de Seguridad de NVS si existen
      char key_min[20], key_max[20];
      snprintf(key_min, sizeof(key_min), "lim_min_%d", m + 1);
      snprintf(key_max, sizeof(key_max), "lim_max_%d", m + 1);
      int32_t raw_lmin = 0, raw_lmax = 0;
      if (nvs_get_i32(handle, key_min, &raw_lmin) == ESP_OK) {
        g_limit_min_deg[m] = (float)raw_lmin / 100.0f;
      }
      if (nvs_get_i32(handle, key_max, &raw_lmax) == ESP_OK) {
        g_limit_max_deg[m] = (float)raw_lmax / 100.0f;
      }
      // Sanitización de límites estrictos: M1 [-60, 60], M2 [-60, 60], M3 [0, 45]
      if (m == 0) {
        if (g_limit_min_deg[0] < -60.0f) g_limit_min_deg[0] = -60.0f;
        if (g_limit_max_deg[0] > 60.0f)  g_limit_max_deg[0] = 60.0f;
      } else if (m == 1) {
        if (g_limit_min_deg[1] < -60.0f) g_limit_min_deg[1] = -60.0f;
        if (g_limit_max_deg[1] > 60.0f)  g_limit_max_deg[1] = 60.0f;
      } else if (m == 2) {
        if (g_limit_min_deg[2] < 0.0f)   g_limit_min_deg[2] = 0.0f;
        if (g_limit_max_deg[2] > 85.0f)  g_limit_max_deg[2] = 85.0f;
      }
      ESP_LOGI(TAG, "Motor %d Límites de Seguridad NVS: [%.1f, %.1f] deg",
               m + 1, g_limit_min_deg[m], g_limit_max_deg[m]);

      for (int s = 0; s < MAX_PRESET_SLOTS; s++) {
        snprintf(key, sizeof(key), "pos_m%d_%d", m + 1, s + 1);
        int32_t raw_slot = 0;
        if (nvs_get_i32(handle, key, &raw_slot) == ESP_OK) {
          g_saved_slots[m][s] = (float)raw_slot / 100.0f;
        }
      }
    }
    nvs_close(handle);
  }
}

static void nvs_save_limits(int motor_idx, float min_deg, float max_deg) {
  if (motor_idx < 0 || motor_idx >= NUM_MOTORS)
    return;
  if (min_deg > max_deg) {
    float tmp = min_deg;
    min_deg = max_deg;
    max_deg = tmp;
  }
  g_limit_min_deg[motor_idx] = min_deg;
  g_limit_max_deg[motor_idx] = max_deg;

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    char key_min[20], key_max[20];
    snprintf(key_min, sizeof(key_min), "lim_min_%d", motor_idx + 1);
    snprintf(key_max, sizeof(key_max), "lim_max_%d", motor_idx + 1);
    nvs_set_i32(handle, key_min, (int32_t)roundf(min_deg * 100.0f));
    nvs_set_i32(handle, key_max, (int32_t)roundf(max_deg * 100.0f));
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG,
             "Motor %d Límites de Seguridad guardados en NVS: [%.1f, %.1f] deg",
             motor_idx + 1, min_deg, max_deg);
  }
}

static void nvs_save_zero_offset(int motor_idx, float raw_deg) {
  if (motor_idx < 0 || motor_idx >= NUM_MOTORS)
    return;
  float ratio = g_gear_ratio[motor_idx] > 0.001f ? g_gear_ratio[motor_idx] : GEAR_RATIO_CYCLOIDAL;
  g_zero_raw_deg[motor_idx] = raw_deg;
  g_zero_offset_deg[motor_idx] = raw_deg / ratio;

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    char key[20];
    snprintf(key, sizeof(key), "zero_raw_%d", motor_idx + 1);
    int32_t raw_millideg = (int32_t)roundf(raw_deg * 1000.0f);
    nvs_set_i32(handle, key, raw_millideg);

    snprintf(key, sizeof(key), "zero_off_%d", motor_idx + 1);
    int32_t raw_offset = (int32_t)roundf(g_zero_offset_deg[motor_idx] * 100.0f);
    nvs_set_i32(handle, key, raw_offset);

    // Guardar réplica compatible
    snprintf(key, sizeof(key), "zero_j%d", motor_idx + 1);
    nvs_set_i32(handle, key, raw_offset);

    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "Motor %d nuevo Offset guardado en NVS: Raw=%.2f deg (Joint=%.2f deg)",
             motor_idx + 1, raw_deg, g_zero_offset_deg[motor_idx]);
  }
}

static void nvs_save_all_zero_offsets(const float raw_angles[NUM_MOTORS]) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    for (int m = 0; m < NUM_MOTORS; m++) {
      float ratio = g_gear_ratio[m] > 0.001f ? g_gear_ratio[m] : GEAR_RATIO_CYCLOIDAL;
      g_zero_raw_deg[m] = raw_angles[m];
      g_zero_offset_deg[m] = raw_angles[m] / ratio;

      char key[20];
      snprintf(key, sizeof(key), "zero_raw_%d", m + 1);
      int32_t raw_millideg = (int32_t)roundf(raw_angles[m] * 1000.0f);
      nvs_set_i32(handle, key, raw_millideg);

      snprintf(key, sizeof(key), "zero_off_%d", m + 1);
      int32_t raw_offset = (int32_t)roundf(g_zero_offset_deg[m] * 100.0f);
      nvs_set_i32(handle, key, raw_offset);

      snprintf(key, sizeof(key), "zero_j%d", m + 1);
      nvs_set_i32(handle, key, raw_offset);
    }
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(
        TAG,
        "Offsets de Cero NVS guardados para los 3 motores: Raw=[%.2f, %.2f, %.2f] deg | Joint=[%.2f, %.2f, %.2f] deg",
        raw_angles[0], raw_angles[1], raw_angles[2],
        g_zero_offset_deg[0], g_zero_offset_deg[1], g_zero_offset_deg[2]);
  }
}

static void nvs_save_slot_position(int motor_idx, int slot, float angle_deg) {
  if (motor_idx < 0 || motor_idx >= NUM_MOTORS)
    return;
  if (slot < 1 || slot > MAX_PRESET_SLOTS)
    return;
  g_saved_slots[motor_idx][slot - 1] = angle_deg;

  nvs_handle_t handle;
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    char key[20];
    snprintf(key, sizeof(key), "pos_m%d_%d", motor_idx + 1, slot);
    int32_t raw_val = (int32_t)roundf(angle_deg * 100.0f);
    nvs_set_i32(handle, key, raw_val);
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "Motor %d Posición %.2f deg guardada en Slot %d NVS",
             motor_idx + 1, angle_deg, slot);
  }
}

// ==============================================================================
// 8. TAREA DE CONTROL EN LAZO CERRADO SUAVE A 50 HZ (3 MOTORES INDEPENDIENTES)
// ==============================================================================
static void steppers_control_task(void *pvParameters) {
  const float dt = 0.020f; // 50 Hz (20 ms)
  float current_running_speed[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
  int current_dir[NUM_MOTORS] = {0, 0, 0};
  static float s_prev_meas_deg[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
  static bool s_in_position[NUM_MOTORS] = {true, true, true};

  // Variables de seguimiento para el Protocolo Anti-Choque / Detección de Atasco por Encoder AS5600:
  static float s_stall_ref_deg[NUM_MOTORS] = {0.0f, 0.0f, 0.0f};
  static uint32_t s_stall_timer_ms[NUM_MOTORS] = {0, 0, 0};
  static bool s_motor_stalled[NUM_MOTORS] = {false, false, false};

  while (1) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      as5600_reading_t r;
      esp_err_t err = as5600_read_motor(i, &r);

      float target = 0.0f;
      bool enabled = false;
      float kp = 0.0f, ki = 0.0f, kd = 0.0f, tol = 0.0f;
      float lim_min = 0.0f, lim_max = 0.0f;

      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_last_reading[i] = r;
        target = g_target_deg[i];
        enabled = g_motor_enabled[i];
        kp = g_kp[i];
        ki = g_ki[i];
        kd = g_kd[i];
        tol = g_tolerance_deg[i];
        lim_min = g_limit_min_deg[i];
        lim_max = g_limit_max_deg[i];
        xSemaphoreGive(g_data_mutex);
      }

      // Acotamiento estricto por límites de seguridad mecánica configurables
      if (target < lim_min)
        target = lim_min;
      if (target > lim_max)
        target = lim_max;

      if (err == ESP_OK && r.valid && enabled) {
        stepper_set_enable(i, true);

        // Error de posición angular directo en grados del reductor de salida 31:1:
        float error = target - r.calibrated_deg;
        float abs_error = fabsf(error);

        // Control con Histéresis de Banda Muerta para erradicar totalmente los tics
        // Para M3 (Codo con brazo flojo/holgura mecánica):
        // Se amplía la tolerancia de parada a 1.25° y el umbral de arranque a 2.80°
        // para que la holgura mecánica del codo no genere micro-oscilaciones o tics.
        float stop_tol = (i == 2) ? (tol < 1.25f ? 1.25f : tol) : tol;
        float start_tol = (i == 2) ? 2.80f : (tol * 2.2f);

        if (s_in_position[i]) {
          if (abs_error > start_tol) {
            s_in_position[i] = false;
            s_stall_ref_deg[i] = r.calibrated_deg;
            s_stall_timer_ms[i] = 0;
            s_motor_stalled[i] = false;
          }
        } else {
          if (abs_error <= stop_tol) {
            s_in_position[i] = true;
          }
        }

        if (s_in_position[i]) {
          // Motor en posición objetivo: Cero pulsos, cero consumo dinámico, cero tics
          stepper_set_hw_speed(i, 0, current_dir[i]);
          current_running_speed[i] = 0.0f;
          g_integral_err[i] = 0.0f;
          s_prev_meas_deg[i] = r.calibrated_deg;
          s_stall_ref_deg[i] = r.calibrated_deg;
          s_stall_timer_ms[i] = 0;
          s_motor_stalled[i] = false;

          if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            g_current_error[i] = error;
            g_current_speed_hz[i] = 0;
            g_motor_state[i] = MOTOR_STATE_IN_POSITION;
            xSemaphoreGive(g_data_mutex);
          }
        } else {
          // Movimiento activo: Control proporcional con rampa suave sin ruido derivativo
          float effective_error = abs_error - stop_tol;
          if (effective_error < 0.0f) effective_error = 0.0f;

          if (abs_error < 6.0f) {
            // Para M3 con brazo flojo: No acumular término integral cerca del umbral de arranque
            if (i == 2 && abs_error < start_tol) {
              g_integral_err[i] = 0.0f;
            } else {
              g_integral_err[i] += error * dt;
              if (g_integral_err[i] > 5.0f) g_integral_err[i] = 5.0f;
              if (g_integral_err[i] < -5.0f) g_integral_err[i] = -5.0f;
            }
          } else {
            g_integral_err[i] = 0.0f;
          }
          s_prev_meas_deg[i] = r.calibrated_deg;

          // -------------------------------------------------------------
          // PROTOCOLO ANTI-CHOQUE / DETECCIÓN DE ATASCO POR ENCODER AS5600:
          // Si el motor genera pulsos (velocidad >= 120 Hz) durante 1.2 segundos
          // pero el encoder NO se desplaza al menos 0.12° (en salida 31:1), se detecta atasco o choque.
          // -------------------------------------------------------------
          if (current_running_speed[i] >= 120.0f) {
            s_stall_timer_ms[i] += 20; // 50 Hz -> 20 ms
            if (s_stall_timer_ms[i] >= 1200) {
              float delta_move = fabsf(r.calibrated_deg - s_stall_ref_deg[i]);
              if (delta_move < 0.12f) {
                // ¡COLISIÓN O ATASCO DETECTADO! Frenado de emergencia inmediato
                s_motor_stalled[i] = true;
                stepper_set_hw_speed(i, 0, current_dir[i]);
                current_running_speed[i] = 0.0f;
                g_integral_err[i] = 0.0f;
                s_in_position[i] = true;
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                  g_target_deg[i] = r.calibrated_deg; // Fijar objetivo en posición actual
                  g_current_speed_hz[i] = 0;
                  g_motor_state[i] = MOTOR_STATE_IN_POSITION;
                  xSemaphoreGive(g_data_mutex);
                }
                // Emisión de alarma crítica por UART
                printf("$ALARM,STALL,M%d,%.2f,%.2f*00\r\n", i + 1, target, r.calibrated_deg);
                ESP_LOGE(TAG, "ALERTA ANTI-CHOQUE: Motor M%d bloqueado! Sin movimiento de encoder (Obj=%.2f, Actual=%.2f)",
                         i + 1, target, r.calibrated_deg);
              } else {
                s_stall_ref_deg[i] = r.calibrated_deg;
                s_stall_timer_ms[i] = 0;
              }
            }
          } else {
            s_stall_ref_deg[i] = r.calibrated_deg;
            s_stall_timer_ms[i] = 0;
          }

          if (s_motor_stalled[i]) {
            continue;
          }

          float target_speed_hz = (kp * effective_error) + (ki * fabsf(g_integral_err[i]));

          if (target_speed_hz < g_min_speed_hz) {
            target_speed_hz = g_min_speed_hz;
          }
          if (target_speed_hz > g_max_speed_hz)
            target_speed_hz = g_max_speed_hz;

          int desired_dir = (error > 0.0f) ? 1 : 0;
          float max_step = g_accel_hz_per_sec * dt;

          // Protección de reversa suave: Si cambia de sentido mientras corre, desacelera primero a cero
          if (desired_dir != current_dir[i] && current_running_speed[i] > 30.0f) {
            current_running_speed[i] -= max_step * 1.5f;
            if (current_running_speed[i] < 0.0f) current_running_speed[i] = 0.0f;
          } else {
            current_dir[i] = desired_dir;
            if (target_speed_hz > current_running_speed[i] + max_step) {
              current_running_speed[i] += max_step;
            } else if (target_speed_hz < current_running_speed[i] - max_step) {
              current_running_speed[i] -= max_step;
            } else {
              current_running_speed[i] = target_speed_hz;
            }
          }

          stepper_set_hw_speed(i, (uint32_t)current_running_speed[i],
                               current_dir[i]);

          if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            g_current_error[i] = error;
            g_current_speed_hz[i] =
                (int32_t)((current_dir[i] == 1) ? current_running_speed[i]
                                                : -current_running_speed[i]);
            g_motor_state[i] = (current_dir[i] == 1) ? MOTOR_STATE_MOVING_CW
                                                     : MOTOR_STATE_MOVING_CCW;
            xSemaphoreGive(g_data_mutex);
          }
        }
      } else {
        s_in_position[i] = true;
        stepper_set_hw_speed(i, 0, current_dir[i]);
        current_running_speed[i] = 0.0f;
        g_integral_err[i] = 0.0f;
        s_prev_meas_deg[i] = r.calibrated_deg;
        s_stall_ref_deg[i] = r.calibrated_deg;
        s_stall_timer_ms[i] = 0;
        s_motor_stalled[i] = false;
        if (!enabled) {
          // En MOTORES LIBRES: el objetivo persigue a la articulación en tiempo real
          // para que cuando el usuario active o bloquee, el error sea 0.0° sin saltos ni tics
          if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            g_target_deg[i] = r.calibrated_deg;
            xSemaphoreGive(g_data_mutex);
          }
          stepper_set_enable(i, false);
        }
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
          g_current_speed_hz[i] = 0;
          g_motor_state[i] = MOTOR_STATE_DISABLED;
          xSemaphoreGive(g_data_mutex);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // Ciclo de control a 50 Hz
  }
}

// ==============================================================================
// 9. SECUENCIA DE JUGADA DEL GATO (PICK AND PLACE AUTOMÁTICO CON ELEVACIÓN
// HOVER)
// ==============================================================================
static void play_cell_sequence_task(void *pvParameters) {
  int cell_idx = (int)(intptr_t)pvParameters;
  if (cell_idx < 0 || cell_idx >= NUM_CELLS) {
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "Iniciando Pick & Place suave en Casilla %d del Gato...",
           cell_idx + 1);

  // 1. Ir a posición de aproximación sobre el dispensador (Pick Hover)
  if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_target_deg[0] = g_pick_pose.j1;
    g_target_deg[1] = g_pick_pose.j2 - 15.0f; // Elevado sobre dispensador
    g_target_deg[2] = g_pick_pose.j3;
    g_motor_enabled[0] = true;
    g_motor_enabled[1] = true;
    g_motor_enabled[2] = true;
    xSemaphoreGive(g_data_mutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1400));

  // 2. Descender al Dispensador de Fichas (Pick)
  if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_target_deg[0] = g_pick_pose.j1;
    g_target_deg[1] = g_pick_pose.j2;
    g_target_deg[2] = g_pick_pose.j3;
    xSemaphoreGive(g_data_mutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1000));

  // 3. Activar Gripper / Electroimán
  gpio_set_level(PIN_TOOL_GRIPPER, 1);
  g_gripper_active = true;
  vTaskDelay(pdMS_TO_TICKS(500));

  // 4. Elevar ficha del dispensador (Pick Hover)
  if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_target_deg[1] = g_pick_pose.j2 - 15.0f;
    xSemaphoreGive(g_data_mutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1000));

  // 5. Mover a la altura de aproximación (Hover Z=8cm) sobre la casilla destino
  if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_target_deg[0] = g_cell_hover_poses[cell_idx].j1;
    g_target_deg[1] = g_cell_hover_poses[cell_idx].j2;
    g_target_deg[2] = g_cell_hover_poses[cell_idx].j3;
    xSemaphoreGive(g_data_mutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1600));

  // 6. Descender a la posición de colocación de la casilla en el tablero
  // (Z=3cm)
  if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_target_deg[0] = g_cell_poses[cell_idx].j1;
    g_target_deg[1] = g_cell_poses[cell_idx].j2;
    g_target_deg[2] = g_cell_poses[cell_idx].j3;
    xSemaphoreGive(g_data_mutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1000));

  // 7. Liberar la ficha (Gripper OFF)
  gpio_set_level(PIN_TOOL_GRIPPER, 0);
  g_gripper_active = false;
  vTaskDelay(pdMS_TO_TICKS(500));

  // 8. Retraer a altura Hover para evitar colisiones con las fichas del tablero
  if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_target_deg[0] = g_cell_hover_poses[cell_idx].j1;
    g_target_deg[1] = g_cell_hover_poses[cell_idx].j2;
    g_target_deg[2] = g_cell_hover_poses[cell_idx].j3;
    xSemaphoreGive(g_data_mutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1200));

  // 9. Retornar a postura HOME
  if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_target_deg[0] = g_home_pose.j1;
    g_target_deg[1] = g_home_pose.j2;
    g_target_deg[2] = g_home_pose.j3;
    xSemaphoreGive(g_data_mutex);
  }

  ESP_LOGI(TAG, "Jugada en Casilla %d completada con éxito.", cell_idx + 1);
  vTaskDelete(NULL);
}

// ==============================================================================
// 10. APP_MAIN (INICIALIZACIÓN, PARSER DE COMANDOS Y TELEMETRÍA)
// ==============================================================================
void app_main(void) {
  // Deshabilitar detector de brownout por hardware para evitar bucle de reinicios continuos
  // por caídas de tensión en cables USB largos o protoboards
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  ESP_LOGI(TAG, "==========================================================");
  ESP_LOGI(TAG,
           "   ROBOT 3 GDL JUGADOR DE GATO (ESP32-S3 + 3x TB6600 + 3x AS5600)");
  ESP_LOGI(TAG, "   - 3 Buses I2C Dedicados (Sin Mux):");
  ESP_LOGI(TAG, "     * M1: SDA=GPIO 1, SCL=GPIO 2  (Base)");
  ESP_LOGI(TAG, "     * M2: SDA=GPIO 8, SCL=GPIO 9  (Hombro)");
  ESP_LOGI(TAG, "     * M3: SDA=GPIO 10, SCL=GPIO 11 (Codo)");
  ESP_LOGI(TAG, "   - Drivers TB6600 (LEDC Timers 0, 1, 2 Independientes):");
  ESP_LOGI(TAG, "     * M1: PUL=GPIO 4, DIR=GPIO 5, ENA=GPIO 6   (Timer 0)");
  ESP_LOGI(TAG, "     * M2: PUL=GPIO 15, DIR=GPIO 16, ENA=GPIO 17 (Timer 1)");
  ESP_LOGI(TAG, "     * M3: PUL=GPIO 12, DIR=GPIO 13, ENA=GPIO 14 (Timer 2)");
  ESP_LOGI(TAG, "   - Herramienta Gripper: GPIO 7");
  ESP_LOGI(TAG, "==========================================================");

  g_data_mutex = xSemaphoreCreateMutex();

  // 1. Inicializar Memoria Flash NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  nvs_load_settings();

  // 2. Inicializar los 3 Buses I2C Dedicados para los Encoders AS5600 (con
  // auto-detección de inversión SDA/SCL)
  int detected_count = 0;
  for (int m = 0; m < NUM_MOTORS; m++) {
    ESP_ERROR_CHECK(soft_i2c_init_bus(g_encoder_buses[m].sda_io,
                                      g_encoder_buses[m].scl_io));
    int sda_lvl = gpio_get_level(g_encoder_buses[m].sda_io);
    int scl_lvl = gpio_get_level(g_encoder_buses[m].scl_io);
    uint8_t probe_st = 0;
    esp_err_t probe_err = as5600_read_bytes_bus(
        g_encoder_buses[m].sda_io, g_encoder_buses[m].scl_io, AS5600_REG_STATUS,
        &probe_st, 1);

    // Si no responde, probar si SDA y SCL están invertidos en ese mismo par de
    // pines
    if (probe_err != ESP_OK) {
      gpio_num_t swapped_sda = g_encoder_buses[m].scl_io;
      gpio_num_t swapped_scl = g_encoder_buses[m].sda_io;
      esp_err_t swap_err = as5600_read_bytes_bus(
          swapped_sda, swapped_scl, AS5600_REG_STATUS, &probe_st, 1);
      if (swap_err == ESP_OK) {
        g_encoder_buses[m].sda_io = swapped_sda;
        g_encoder_buses[m].scl_io = swapped_scl;
        probe_err = ESP_OK;
        ESP_LOGI(TAG,
                 "Encoder Motor %d detectado con SDA/SCL invertidos -> "
                 "auto-corregido a SDA=GPIO %d | SCL=GPIO %d",
                 m + 1, swapped_sda, swapped_scl);
      }
    }

    if (probe_err == ESP_OK) {
      detected_count++;
      ESP_LOGI(TAG,
               "Encoder Motor %d (AS5600) OK en SDA=GPIO %d (%d) | SCL=GPIO %d "
               "(%d) (STATUS=0x%02X)",
               m + 1, g_encoder_buses[m].sda_io, sda_lvl,
               g_encoder_buses[m].scl_io, scl_lvl, probe_st);
    } else {
      ESP_LOGW(TAG,
               "Encoder Motor %d NO responde en SDA=GPIO %d (nivel=%d) | "
               "SCL=GPIO %d (nivel=%d) (Revise conexiones)",
               m + 1, g_encoder_buses[m].sda_io, sda_lvl,
               g_encoder_buses[m].scl_io, scl_lvl);
    }
  }

  // Escáner de diagnóstico en GPIOs I2C si falta algún encoder
  if (detected_count < NUM_MOTORS) {
    ESP_LOGI(TAG, "Escaneando pines I2C en busca de sensores AS5600 (0x36)...");
    const gpio_num_t scan_pins[] = {GPIO_NUM_1, GPIO_NUM_2,  GPIO_NUM_8,
                                    GPIO_NUM_9, GPIO_NUM_10, GPIO_NUM_11};
    const int num_scan = sizeof(scan_pins) / sizeof(scan_pins[0]);
    for (int i = 0; i < num_scan; i++) {
      if (gpio_get_level(scan_pins[i]) == 0)
        continue;
      for (int j = 0; j < num_scan; j++) {
        if (i == j || gpio_get_level(scan_pins[j]) == 0)
          continue;
        uint8_t st_val = 0;
        if (as5600_read_bytes_bus(scan_pins[i], scan_pins[j], AS5600_REG_STATUS,
                                  &st_val, 1) == ESP_OK) {
          ESP_LOGI(TAG,
                   ">>> ENCONTRADO AS5600 (0x36) en SDA=GPIO %d | SCL=GPIO %d "
                   "(STATUS=0x%02X) <<<",
                   scan_pins[i], scan_pins[j], st_val);
        }
      }
    }
  }

  // 3. Inicializar Hardware de los 3 Drivers Steppers (LEDC PWM Independiente)
  ESP_ERROR_CHECK(steppers_hardware_init());
  for (int i = 0; i < NUM_MOTORS; i++) {
    stepper_set_hw_speed(i, 0, 0);
    stepper_set_enable(i, true);
  }
  ESP_LOGI(TAG, "Drivers Steppers TB6600 inicializados con torque de retención firme.");

  // Pre-leer encoders y sincronizar objetivo con la postura física actual
  // Garantiza 0% brincos o giros descontrolados de 360° al encender
  for (int m = 0; m < NUM_MOTORS; m++) {
    as5600_reading_t r = {0};
    as5600_read_motor(m, &r);
    g_last_reading[m] = r;
    g_target_deg[m] =
        r.calibrated_deg; // Objetivo = Medición actual -> Error inicial = 0.0°
    g_motor_enabled[m] = true; // Motores energizados y firmes en su postura actual
  }

  // 4. Periférico UART (115200 Baudios, 8N1, Buffer Circular 1024 Bytes)
  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  uart_param_config(UART_NUM_0, &uart_config);
  uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);

  // 5. Tarea de Control en Core 1 a 50 Hz
  xTaskCreatePinnedToCore(steppers_control_task, "steppers_ctrl", 4096, NULL, 5,
                          NULL, 1);

  char rx_buf[128];
  int rx_idx = 0;
  bool in_frame = false;
  uint32_t err_checksum_count = 0;
  uint32_t err_overflow_count = 0;
  uint32_t err_out_of_range_count = 0;

  int telemetry_timer = 0;

  while (1) {
    // --- A. Lectura No Bloqueante de UART0 y USB Serial/JTAG (COM nativo
    // ESP32-S3) ---
    uint8_t ch = 0;
    int len = uart_read_bytes(UART_NUM_0, &ch, 1, 0);
    if (len <= 0) {
      len = (int)usb_serial_jtag_ll_read_rxfifo(&ch, 1);
    }
    while (len > 0) {
      if (ch == '$') {
        in_frame = true;
        rx_idx = 0;
      } else if (in_frame) {
        if (ch == '\r' || ch == '\n') {
          if (rx_idx > 0) {
            rx_buf[rx_idx] = '\0';
            in_frame = false;

            // Validación de Integridad por Checksum XOR
            char *star_ptr = strrchr(rx_buf, '*');
            if (star_ptr != NULL) {
              *star_ptr = '\0';
              char *payload = rx_buf;
              char *received_cs_str = star_ptr + 1;
              uint32_t received_cs =
                  (uint32_t)strtoul(received_cs_str, NULL, 16);

              uint8_t calc_cs = 0;
              for (size_t k = 0; k < strlen(payload); k++) {
                calc_cs ^= (uint8_t)payload[k];
              }

              if (calc_cs != (uint8_t)received_cs) {
                err_checksum_count++;
                printf("$NACK,0,ERR_CHECKSUM,CALC_%02X*00\r\n", calc_cs);
                fflush(stdout);
              } else {
                int seq = 0;
                int motor_id = 0;
                char cmd[16] = {0};
                float p1 = 0, p2 = 0, p3 = 0;
                int parsed = sscanf(payload, "PKT,%d,%d,%15[^,],%f,%f,%f", &seq,
                                    &motor_id, cmd, &p1, &p2, &p3);

                if (parsed >= 3) {
                  // 1. Movimiento Sincronizado Simultáneo (3 Motores en grados
                  // de Reductor 31:1)
                  if (strcmp(cmd, "POS_ALL") == 0 ||
                      strcmp(cmd, "ANGLES") == 0 ||
                      strcmp(cmd, "MOVE_ALL") == 0 ||
                      strcmp(cmd, "SYNC") == 0) {
                    float t1 = p1;
                    float t2 = p2;
                    float t3 = p3;

                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                        pdTRUE) {
                      g_target_deg[0] = t1;
                      g_motor_enabled[0] = true;
                      g_target_deg[1] = t2;
                      g_motor_enabled[1] = true;
                      g_target_deg[2] = t3;
                      g_motor_enabled[2] = true;
                      xSemaphoreGive(g_data_mutex);
                    }
                    printf("$ACK,%d,POS_ALL_OK,%.2f,%.2f,%.2f*00\r\n", seq, t1,
                           t2, t3);

                    // 2. Movimiento Individual de Motor (en grados de Reductor
                    // 31:1)
                  } else if (strcmp(cmd, "POS") == 0 ||
                             strcmp(cmd, "GOTO") == 0) {
                    float target = p1;
                    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        g_target_deg[motor_id - 1] = target;
                        g_motor_enabled[motor_id - 1] = true;
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,POS_OK,M%d,%.2f*00\r\n", seq, motor_id,
                             target);
                    } else if (motor_id == 0) {
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        for (int m = 0; m < NUM_MOTORS; m++) {
                          g_target_deg[m] = target;
                          g_motor_enabled[m] = true;
                        }
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,POS_OK,ALL,%.2f*00\r\n", seq, target);
                    } else {
                      err_out_of_range_count++;
                      printf("$NACK,%d,ERR_INVALID_MOTOR*00\r\n", seq);
                    }

                    // 2b. Movimiento por Vueltas de Motor NEMA 17 (31 vueltas =
                    // 360° Reductor)
                  } else if (strcmp(cmd, "TURNS") == 0) {
                    float turns = p1;
                    float target = (turns / GEAR_RATIO_CYCLOIDAL) * 360.0f;
                    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        g_target_deg[motor_id - 1] = target;
                        g_motor_enabled[motor_id - 1] = true;
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,TURNS_OK,M%d,%.2f_TURNS,%.2f_DEG*00\r\n",
                             seq, motor_id, turns, target);
                    } else if (motor_id == 0) {
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        for (int m = 0; m < NUM_MOTORS; m++) {
                          g_target_deg[m] = target;
                          g_motor_enabled[m] = true;
                        }
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,TURNS_ALL_OK,%.2f_TURNS,%.2f_DEG*00\r\n",
                             seq, turns, target);
                    } else {
                      err_out_of_range_count++;
                      printf("$NACK,%d,ERR_INVALID_MOTOR*00\r\n", seq);
                    }

                    // 3. Jugada en Casilla del Gato (Pick & Place)
                  } else if (strcmp(cmd, "PLAY_CELL") == 0) {
                    int cell = (int)p1 - 1;
                    if (cell >= 0 && cell < NUM_CELLS) {
                      xTaskCreate(play_cell_sequence_task, "play_seq", 3072,
                                  (void *)(intptr_t)cell, 4, NULL);
                      printf("$ACK,%d,PLAY_CELL_OK,%d*00\r\n", seq, cell + 1);
                    } else {
                      err_out_of_range_count++;
                      printf("$NACK,%d,ERR_CELL_OUT_OF_RANGE*00\r\n", seq);
                    }

                    // 4. Control del Gripper / Electroimán
                  } else if (strcmp(cmd, "GRIPPER") == 0) {
                    bool state = (p1 != 0.0f);
                    gpio_set_level(PIN_TOOL_GRIPPER, state ? 1 : 0);
                    g_gripper_active = state;
                    printf("$ACK,%d,GRIPPER_OK,%d*00\r\n", seq, state ? 1 : 0);

                    // 5. Paro de Emergencia Jerárquico (Individual o Global)
                  } else if (strcmp(cmd, "STOP") == 0 ||
                             strcmp(cmd, "ESTOP") == 0 ||
                             strcmp(cmd, "BRAKE") == 0) {
                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                        pdTRUE) {
                      if (motor_id == 0) {
                        for (int m = 0; m < NUM_MOTORS; m++) {
                          g_motor_enabled[m] = false;
                          stepper_set_hw_speed(m, 0, 0);
                          stepper_set_enable(m, false);
                        }
                      } else if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                        g_motor_enabled[motor_id - 1] = false;
                        stepper_set_hw_speed(motor_id - 1, 0, 0);
                        stepper_set_enable(motor_id - 1, false);
                      }
                      xSemaphoreGive(g_data_mutex);
                    }
                    printf("$ACK,%d,STOP_OK,M%d*00\r\n", seq, motor_id);

                    // 6. Calibración de Posición Cero (Guardar Cero actual en
                    // Flash NVS)
                  } else if (strcmp(cmd, "ZERO") == 0 ||
                             strcmp(cmd, "SET_ZERO") == 0 ||
                             strcmp(cmd, "SAVE_GAME") == 0) {
                    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                      int ax = motor_id - 1;
                      float raw = 0.0f;
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) ==
                          pdTRUE) {
                        raw = g_last_reading[ax].valid
                                        ? g_last_reading[ax].raw_deg
                                        : s_last_valid[ax].raw_deg;
                        as5600_reset_zero_calibration(ax);
                        g_last_reading[ax].calibrated_deg = 0.0f;
                        g_last_reading[ax].joint_angle_deg = 0.0f;
                        g_last_reading[ax].multiturn_total_deg = raw;
                        g_target_deg[ax] = 0.0f;
                        g_integral_err[ax] = 0.0f;
                        g_prev_err[ax] = 0.0f;
                        g_motor_enabled[ax] = true;
                        xSemaphoreGive(g_data_mutex);
                      }
                      nvs_save_zero_offset(ax, raw);
                      printf("$ACK,%d,SAVE_GAME_OK,M%d,0.00*00\r\n", seq,
                             motor_id);
                    } else if (motor_id == 0) {
                      float new_raws[NUM_MOTORS] = {0.0f};
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) ==
                          pdTRUE) {
                        for (int ax = 0; ax < NUM_MOTORS; ax++) {
                          float raw = g_last_reading[ax].valid
                                          ? g_last_reading[ax].raw_deg
                                          : s_last_valid[ax].raw_deg;
                          new_raws[ax] = raw;
                          as5600_reset_zero_calibration(ax);
                          g_last_reading[ax].calibrated_deg = 0.0f;
                          g_last_reading[ax].joint_angle_deg = 0.0f;
                          g_last_reading[ax].multiturn_total_deg = raw;
                          g_target_deg[ax] = 0.0f;
                          g_integral_err[ax] = 0.0f;
                          g_prev_err[ax] = 0.0f;
                          g_motor_enabled[ax] = true;
                        }
                        xSemaphoreGive(g_data_mutex);
                      }
                      nvs_save_all_zero_offsets(new_raws);
                      printf("$ACK,%d,SAVE_GAME_ALL_OK*00\r\n", seq);
                    }

                    // 6b. Cargar Partida Guardada (Recuperar Cero de Flash NVS
                    // y regresar suavemente a la postura cero calibrada 0.0°)
                  } else if (strcmp(cmd, "LOAD_GAME") == 0 ||
                             strcmp(cmd, "LOAD_ZERO") == 0) {
                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                        pdTRUE) {
                      nvs_load_settings();
                      for (int ax = 0; ax < NUM_MOTORS; ax++) {
                        s_first_read[ax] = true;
                        g_target_deg[ax] = 0.0f; // Regresa a la postura cero calibrada (0.0°)
                        g_motor_enabled[ax] = true;
                        g_integral_err[ax] = 0.0f;
                        g_prev_err[ax] = 0.0f;
                      }
                      xSemaphoreGive(g_data_mutex);
                    }
                    printf("$ACK,%d,LOAD_GAME_OK,%.2f,%.2f,%.2f*00\r\n", seq,
                           g_zero_offset_deg[0], g_zero_offset_deg[1],
                           g_zero_offset_deg[2]);

                    // 6c. Motores Libres (Desactivar fuerza para mover y
                    // acomodar a mano)
                  } else if (strcmp(cmd, "FREE") == 0 ||
                             strcmp(cmd, "MOTORS_FREE") == 0 ||
                             strcmp(cmd, "TORQUE_OFF") == 0) {
                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                        pdTRUE) {
                      for (int ax = 0; ax < NUM_MOTORS; ax++) {
                        g_motor_enabled[ax] = false;
                        stepper_set_hw_speed(ax, 0, 0);
                        stepper_set_enable(ax, false);
                      }
                      xSemaphoreGive(g_data_mutex);
                    }
                    printf("$ACK,%d,MOTORS_FREE_OK*00\r\n", seq);

                    // 6d. Bloquear Motores (Activar torque de retención suave
                    // en postura actual)
                  } else if (strcmp(cmd, "LOCK") == 0 ||
                             strcmp(cmd, "MOTORS_LOCK") == 0 ||
                             strcmp(cmd, "TORQUE_ON") == 0 ||
                             strcmp(cmd, "ENABLE") == 0) {
                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                        pdTRUE) {
                      for (int ax = 0; ax < NUM_MOTORS; ax++) {
                        g_target_deg[ax] = g_last_reading[ax].calibrated_deg;
                        g_motor_enabled[ax] = true;
                        stepper_set_enable(ax, true);
                      }
                      xSemaphoreGive(g_data_mutex);
                    }
                    printf("$ACK,%d,MOTORS_LOCK_OK*00\r\n", seq);

                    // 6e. Borrar Offset de Cero (Ángulo Absoluto)
                  } else if (strcmp(cmd, "RESET_ZERO") == 0) {
                    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                      int ax = motor_id - 1;
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        nvs_save_zero_offset(ax, 0.0f);
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,RESET_ZERO_OK,M%d*00\r\n", seq, motor_id);
                    } else if (motor_id == 0) {
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        for (int ax = 0; ax < NUM_MOTORS; ax++) {
                          nvs_save_zero_offset(ax, 0.0f);
                        }
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,RESET_ZERO_OK,M0*00\r\n", seq);
                    }

                    // 7. Retorno a Origen (HOME)
                  } else if (strcmp(cmd, "HOME") == 0) {
                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                        pdTRUE) {
                      g_target_deg[0] = g_home_pose.j1;
                      g_target_deg[1] = g_home_pose.j2;
                      g_target_deg[2] = g_home_pose.j3;
                      g_motor_enabled[0] = true;
                      g_motor_enabled[1] = true;
                      g_motor_enabled[2] = true;
                      xSemaphoreGive(g_data_mutex);
                    }
                    printf("$ACK,%d,HOME_OK*00\r\n", seq);

                    // 7b. Ir Directo a Casilla del Gato (1 a 9)
                  } else if (strcmp(cmd, "CELL") == 0 ||
                             strcmp(cmd, "GOTO_CELL") == 0) {
                    int c_num = (int)p1;
                    if (c_num >= 1 && c_num <= NUM_CELLS) {
                      int c_idx = c_num - 1;
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        g_target_deg[0] = g_cell_poses[c_idx].j1;
                        g_target_deg[1] = g_cell_poses[c_idx].j2;
                        g_target_deg[2] = g_cell_poses[c_idx].j3;
                        g_motor_enabled[0] = true;
                        g_motor_enabled[1] = true;
                        g_motor_enabled[2] = true;
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,CELL_OK,%d,%.2f,%.2f,%.2f*00\r\n", seq,
                             c_num, g_cell_poses[c_idx].j1,
                             g_cell_poses[c_idx].j2, g_cell_poses[c_idx].j3);
                    } else {
                      err_out_of_range_count++;
                      printf("$NACK,%d,ERR_CELL_OUT_OF_RANGE*00\r\n", seq);
                    }

                    // 7c. Ir a Altura de Aproximación Hover sobre Casilla (1 a
                    // 9)
                  } else if (strcmp(cmd, "HOVER_CELL") == 0) {
                    int c_num = (int)p1;
                    if (c_num >= 1 && c_num <= NUM_CELLS) {
                      int c_idx = c_num - 1;
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        g_target_deg[0] = g_cell_hover_poses[c_idx].j1;
                        g_target_deg[1] = g_cell_hover_poses[c_idx].j2;
                        g_target_deg[2] = g_cell_hover_poses[c_idx].j3;
                        g_motor_enabled[0] = true;
                        g_motor_enabled[1] = true;
                        g_motor_enabled[2] = true;
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,HOVER_CELL_OK,%d,%.2f,%.2f,%.2f*00\r\n",
                             seq, c_num, g_cell_hover_poses[c_idx].j1,
                             g_cell_hover_poses[c_idx].j2,
                             g_cell_hover_poses[c_idx].j3);
                    } else {
                      err_out_of_range_count++;
                      printf("$NACK,%d,ERR_CELL_OUT_OF_RANGE*00\r\n", seq);
                    }

                    // 7d. Configuración de Límites Angulares de Seguridad
                  } else if (strcmp(cmd, "LIMITS") == 0 ||
                             strcmp(cmd, "SET_LIMITS") == 0) {
                    float lmin = p1;
                    float lmax = p2;
                    if (lmin > lmax) {
                      float tmp = lmin;
                      lmin = lmax;
                      lmax = tmp;
                    }
                    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        nvs_save_limits(motor_id - 1, lmin, lmax);
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,LIMITS_OK,M%d,%.1f,%.1f*00\r\n", seq,
                             motor_id, lmin, lmax);
                    } else if (motor_id == 0) {
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        for (int m = 0; m < NUM_MOTORS; m++) {
                          nvs_save_limits(m, lmin, lmax);
                        }
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,LIMITS_ALL_OK,%.1f,%.1f*00\r\n", seq,
                             lmin, lmax);
                    } else {
                      err_out_of_range_count++;
                      printf("$NACK,%d,ERR_INVALID_MOTOR*00\r\n", seq);
                    }

                    // 7e. Consulta de Límites Actuales
                  } else if (strcmp(cmd, "GET_LIMITS") == 0) {
                    printf("$ACK,%d,LIMITS,1:%.1f:%.1f,2:%.1f:%.1f,3:%.1f:%.1f*"
                           "00\r\n",
                           seq, g_limit_min_deg[0], g_limit_max_deg[0],
                           g_limit_min_deg[1], g_limit_max_deg[1],
                           g_limit_min_deg[2], g_limit_max_deg[2]);

                    // 8. Ajuste Dinámico de Ganancias PID
                  } else if (strcmp(cmd, "PID") == 0) {
                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                        pdTRUE) {
                      if (motor_id == 0) {
                        for (int m = 0; m < NUM_MOTORS; m++) {
                          g_kp[m] = p1;
                          g_ki[m] = p2;
                          g_kd[m] = p3;
                        }
                      } else if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                        g_kp[motor_id - 1] = p1;
                        g_ki[motor_id - 1] = p2;
                        g_kd[motor_id - 1] = p3;
                      }
                      xSemaphoreGive(g_data_mutex);
                    }
                    printf("$ACK,%d,PID_OK,M%d,%.2f,%.2f,%.2f*00\r\n", seq,
                           motor_id, p1, p2, p3);

                    // 9. Ajuste de Banda Muerta / Tolerancia
                  } else if (strcmp(cmd, "TOL") == 0) {
                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                        pdTRUE) {
                      if (motor_id == 0) {
                        for (int m = 0; m < NUM_MOTORS; m++)
                          g_tolerance_deg[m] = p1;
                      } else if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                        g_tolerance_deg[motor_id - 1] = p1;
                      }
                      xSemaphoreGive(g_data_mutex);
                    }
                    printf("$ACK,%d,TOL_OK,M%d,%.2f*00\r\n", seq, motor_id, p1);

                    // 10. Guardar Slot de Memoria en NVS
                  } else if (strcmp(cmd, "SAVE_SLOT") == 0) {
                    int slot = (int)p1;
                    float angle = p2;
                    if (motor_id >= 1 && motor_id <= NUM_MOTORS) {
                      nvs_save_slot_position(motor_id - 1, slot, angle);
                      printf("$ACK,%d,SAVE_SLOT_OK,M%d,%d,%.2f*00\r\n", seq,
                             motor_id, slot, angle);
                    }

                    // 11. Cargar Slot de Memoria
                  } else if (strcmp(cmd, "GOTO_SLOT") == 0) {
                    int slot = (int)p1;
                    if (slot >= 1 && slot <= MAX_PRESET_SLOTS &&
                        motor_id >= 1 && motor_id <= NUM_MOTORS) {
                      float target = g_saved_slots[motor_id - 1][slot - 1];
                      if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                          pdTRUE) {
                        g_target_deg[motor_id - 1] = target;
                        g_motor_enabled[motor_id - 1] = true;
                        xSemaphoreGive(g_data_mutex);
                      }
                      printf("$ACK,%d,GOTO_SLOT_OK,M%d,%d,%.2f*00\r\n", seq,
                             motor_id, slot, target);
                    }

                    // 12. Diagnóstico de Conexión de Encoders AS5600
                  } else if (strcmp(cmd, "CHECK_ENC") == 0 ||
                             strcmp(cmd, "TEST_ENC") == 0) {
                    as5600_reading_t r_copy[NUM_MOTORS];
                    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                        pdTRUE) {
                      for (int m = 0; m < NUM_MOTORS; m++) {
                        r_copy[m] = g_last_reading[m];
                      }
                      xSemaphoreGive(g_data_mutex);
                    }
                    printf("$ACK,%d,ENC_STATUS,1:%d:%d:%d,2:%d:%d:%d,3:%d:%d:%"
                           "d*00\r\n",
                           seq, r_copy[0].valid ? 1 : 0,
                           r_copy[0].magnet_detected ? 1 : 0,
                           r_copy[0].raw_angle, r_copy[1].valid ? 1 : 0,
                           r_copy[1].magnet_detected ? 1 : 0,
                           r_copy[1].raw_angle, r_copy[2].valid ? 1 : 0,
                           r_copy[2].magnet_detected ? 1 : 0,
                           r_copy[2].raw_angle);

                    printf("\n========================================\n");
                    printf(" DIAGNOSTICO DE CONEXION ENCODERS AS5600\n");
                    printf("========================================\n");
                    for (int m = 0; m < NUM_MOTORS; m++) {
                      const char *st_name = "DESCONECTADO (ERROR I2C)";
                      if (g_last_reading[m].valid) {
                        if (g_last_reading[m].magnet_detected) {
                          st_name = "CONECTADO (IMAN OPTIMO)";
                        } else if (g_last_reading[m].magnet_too_weak) {
                          st_name = "I2C OK (IMAN MUY DEBIL)";
                        } else {
                          st_name = "I2C OK (SIN IMAN DETECTADO)";
                        }
                      }
                      printf("M%d (SDA: %2d, SCL: %2d): %s | Raw: %4d | Ang: "
                             "%6.2f deg\n",
                             m + 1, g_encoder_buses[m].sda_io,
                             g_encoder_buses[m].scl_io, st_name,
                             g_last_reading[m].raw_angle,
                             g_last_reading[m].calibrated_deg);
                    }
                    printf("========================================\n\n");

                  } else {
                    printf("$NACK,%d,ERR_UNKNOWN_CMD*00\r\n", seq);
                  }
                  fflush(stdout);
                }
              }
            }
          }
        } else if (rx_idx < (int)sizeof(rx_buf) - 1) {
          rx_buf[rx_idx++] = (char)ch;
        } else {
          err_overflow_count++;
          in_frame = false;
          rx_idx = 0;
        }
      } else {
        // Compatibilidad con comandos ASCII simples de consola o software
        // heredado
        if (ch == '\r' || ch == '\n') {
          if (rx_idx > 0) {
            rx_buf[rx_idx] = '\0';
            if (strncmp(rx_buf, "CELL:", 5) == 0) {
              int c_num = atoi(rx_buf + 5);
              if (c_num >= 1 && c_num <= NUM_CELLS) {
                int c_idx = c_num - 1;
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                  g_target_deg[0] = g_cell_poses[c_idx].j1;
                  g_target_deg[1] = g_cell_poses[c_idx].j2;
                  g_target_deg[2] = g_cell_poses[c_idx].j3;
                  g_motor_enabled[0] = true;
                  g_motor_enabled[1] = true;
                  g_motor_enabled[2] = true;
                  xSemaphoreGive(g_data_mutex);
                }
                printf("ACK:CELL=%d (%.2f, %.2f, %.2f)\n", c_num,
                       g_cell_poses[c_idx].j1, g_cell_poses[c_idx].j2,
                       g_cell_poses[c_idx].j3);
              }
            } else if (strncmp(rx_buf, "HOVER:", 6) == 0) {
              int c_num = atoi(rx_buf + 6);
              if (c_num >= 1 && c_num <= NUM_CELLS) {
                int c_idx = c_num - 1;
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                  g_target_deg[0] = g_cell_hover_poses[c_idx].j1;
                  g_target_deg[1] = g_cell_hover_poses[c_idx].j2;
                  g_target_deg[2] = g_cell_hover_poses[c_idx].j3;
                  g_motor_enabled[0] = true;
                  g_motor_enabled[1] = true;
                  g_motor_enabled[2] = true;
                  xSemaphoreGive(g_data_mutex);
                }
                printf("ACK:HOVER=%d (%.2f, %.2f, %.2f)\n", c_num,
                       g_cell_hover_poses[c_idx].j1,
                       g_cell_hover_poses[c_idx].j2,
                       g_cell_hover_poses[c_idx].j3);
              }
            } else if (strncmp(rx_buf, "LIMITS:", 7) == 0) {
              int mid = 0;
              float lmin = 0, lmax = 0;
              if (sscanf(rx_buf + 7, "%d,%f,%f", &mid, &lmin, &lmax) >= 3) {
                if (mid >= 1 && mid <= NUM_MOTORS && lmin < lmax) {
                  if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) ==
                      pdTRUE) {
                    nvs_save_limits(mid - 1, lmin, lmax);
                    xSemaphoreGive(g_data_mutex);
                  }
                  printf("ACK:LIMITS=M%d,[%.1f,%.1f]\n", mid, lmin, lmax);
                }
              }
            } else if (strcmp(rx_buf, "GET_LIMITS") == 0) {
              printf(
                  "ACK:LIMITS=M1:[%.1f,%.1f],M2:[%.1f,%.1f],M3:[%.1f,%.1f]\n",
                  g_limit_min_deg[0], g_limit_max_deg[0], g_limit_min_deg[1],
                  g_limit_max_deg[1], g_limit_min_deg[2], g_limit_max_deg[2]);
            } else if (strncmp(rx_buf, "PLAY_CELL:", 10) == 0) {
              int cell = atoi(rx_buf + 10) - 1;
              if (cell >= 0 && cell < NUM_CELLS) {
                xTaskCreate(play_cell_sequence_task, "play_seq", 3072,
                            (void *)(intptr_t)cell, 4, NULL);
                printf("ACK:PLAY_CELL=%d\n", cell + 1);
              }
            } else if (strncmp(rx_buf, "SET_ANGLES:", 11) == 0) {
              float a = 0, b = 0, c = 0;
              if (sscanf(rx_buf + 11, "%f,%f,%f", &a, &b, &c) >= 3) {
                if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                  g_target_deg[0] = a;
                  g_motor_enabled[0] = true;
                  g_target_deg[1] = b;
                  g_motor_enabled[1] = true;
                  g_target_deg[2] = c;
                  g_motor_enabled[2] = true;
                  xSemaphoreGive(g_data_mutex);
                }
                printf("ACK:SET_ANGLES=%.2f,%.2f,%.2f (REDUCTOR)\n", a, b, c);
              }
            } else if (strncmp(rx_buf, "TURNS:", 6) == 0) {
              float turns = atof(rx_buf + 6);
              float target = (turns / GEAR_RATIO_CYCLOIDAL) * 360.0f;
              if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                for (int m = 0; m < NUM_MOTORS; m++) {
                  g_target_deg[m] = target;
                  g_motor_enabled[m] = true;
                }
                xSemaphoreGive(g_data_mutex);
              }
              printf("ACK:TURNS=%.2f (%.2f DEG REDUCTOR)\n", turns, target);
            } else if (strcmp(rx_buf, "ZERO") == 0 ||
                       strcmp(rx_buf, "SET_ZERO") == 0 ||
                       strcmp(rx_buf, "SAVE_GAME") == 0) {
              float new_raws[NUM_MOTORS] = {0.0f};
              if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                for (int ax = 0; ax < NUM_MOTORS; ax++) {
                  float raw = g_last_reading[ax].valid
                                  ? g_last_reading[ax].raw_deg
                                  : s_last_valid[ax].raw_deg;
                  new_raws[ax] = raw;
                  as5600_reset_zero_calibration(ax);
                  g_last_reading[ax].calibrated_deg = 0.0f;
                  g_last_reading[ax].joint_angle_deg = 0.0f;
                  g_last_reading[ax].multiturn_total_deg = raw;
                  g_target_deg[ax] = 0.0f;
                  g_integral_err[ax] = 0.0f;
                  g_prev_err[ax] = 0.0f;
                  g_motor_enabled[ax] = true;
                }
                xSemaphoreGive(g_data_mutex);
              }
              nvs_save_all_zero_offsets(new_raws);
              printf("ACK:SAVE_GAME_ALL\n");
            } else if (strcmp(rx_buf, "LOAD_GAME") == 0 ||
                       strcmp(rx_buf, "LOAD_ZERO") == 0) {
              if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                nvs_load_settings();
                for (int ax = 0; ax < NUM_MOTORS; ax++) {
                  s_first_read[ax] = true;
                  g_target_deg[ax] = 0.0f; // Regresa a la postura cero calibrada (0.0°)
                  g_motor_enabled[ax] = true;
                  g_integral_err[ax] = 0.0f;
                  g_prev_err[ax] = 0.0f;
                }
                xSemaphoreGive(g_data_mutex);
              }
              printf("ACK:LOAD_GAME_ALL\n");
            } else if (strcmp(rx_buf, "FREE") == 0 ||
                       strcmp(rx_buf, "MOTORS_FREE") == 0) {
              if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                for (int m = 0; m < NUM_MOTORS; m++) {
                  g_motor_enabled[m] = false;
                  stepper_set_hw_speed(m, 0, 0);
                  stepper_set_enable(m, false);
                }
                xSemaphoreGive(g_data_mutex);
              }
              printf("ACK:MOTORS_FREE\n");
            } else if (strcmp(rx_buf, "LOCK") == 0 ||
                       strcmp(rx_buf, "MOTORS_LOCK") == 0 ||
                       strcmp(rx_buf, "ENABLE") == 0) {
              if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                for (int m = 0; m < NUM_MOTORS; m++) {
                  g_target_deg[m] = g_last_reading[m].calibrated_deg;
                  g_motor_enabled[m] = true;
                  stepper_set_enable(m, true);
                }
                xSemaphoreGive(g_data_mutex);
              }
              printf("ACK:MOTORS_LOCK\n");
            } else if (strcmp(rx_buf, "HOME") == 0) {
              if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_target_deg[0] = 0.0f;
                g_target_deg[1] = 0.0f;
                g_target_deg[2] = 0.0f;
                g_motor_enabled[0] = true;
                g_motor_enabled[1] = true;
                g_motor_enabled[2] = true;
                xSemaphoreGive(g_data_mutex);
              }
              printf("Homing completado\r\nMovement completed\r\nACK:HOME\n");
            } else if (strncmp(rx_buf, "GRIPPER:", 8) == 0) {
              int st = atoi(rx_buf + 8);
              gpio_set_level(PIN_TOOL_GRIPPER, st ? 1 : 0);
              g_gripper_active = (st != 0);
              printf("ACK:GRIPPER=%d\n", st);
            } else if (strcmp(rx_buf, "STOP") == 0 ||
                       strcmp(rx_buf, "ESTOP") == 0) {
              if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                for (int m = 0; m < NUM_MOTORS; m++) {
                  g_motor_enabled[m] = false;
                  stepper_set_hw_speed(m, 0, 0);
                  stepper_set_enable(m, false);
                }
                xSemaphoreGive(g_data_mutex);
              }
              printf("ACK:STOP\n");
            } else if (strcmp(rx_buf, "CHECK_ENC") == 0 ||
                       strcmp(rx_buf, "TEST_ENC") == 0) {
              as5600_reading_t r_copy[NUM_MOTORS];
              if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                for (int m = 0; m < NUM_MOTORS; m++) {
                  r_copy[m] = g_last_reading[m];
                }
                xSemaphoreGive(g_data_mutex);
              }
              printf(
                  "$ACK,0,ENC_STATUS,1:%d:%d:%d,2:%d:%d:%d,3:%d:%d:%d*00\r\n",
                  r_copy[0].valid ? 1 : 0, r_copy[0].magnet_detected ? 1 : 0,
                  r_copy[0].raw_angle, r_copy[1].valid ? 1 : 0,
                  r_copy[1].magnet_detected ? 1 : 0, r_copy[1].raw_angle,
                  r_copy[2].valid ? 1 : 0, r_copy[2].magnet_detected ? 1 : 0,
                  r_copy[2].raw_angle);
            }
            fflush(stdout);
            rx_idx = 0;
          }
        } else if (rx_idx < (int)sizeof(rx_buf) - 1) {
          rx_buf[rx_idx++] = (char)ch;
        }
      }
      len = uart_read_bytes(UART_NUM_0, &ch, 1, 0);
      if (len <= 0) {
        len = (int)usb_serial_jtag_ll_read_rxfifo(&ch, 1);
      }
    }

    // --- B. Telemetría Periódica a 10 Hz (100 ms con bucle de 5 ms) ---
    telemetry_timer++;
    if (telemetry_timer >= 20) {
      telemetry_timer = 0;

      // 1. Trama Enmarcada $TEL3 para GUI (gui_robot_gato.py & gui_monitor.py)
      // Formato:
      // $TEL3,1,<ANG1>,<SPD1>,2,<ANG2>,<SPD2>,3,<ANG3>,<SPD3>,<GRIPPER>*<CS>\r\n
      char tel_payload[96];
      snprintf(tel_payload, sizeof(tel_payload),
               "TEL3,1,%.2f,%ld,2,%.2f,%ld,3,%.2f,%ld,%d",
               g_last_reading[0].calibrated_deg, (long)g_current_speed_hz[0],
               g_last_reading[1].calibrated_deg, (long)g_current_speed_hz[1],
               g_last_reading[2].calibrated_deg, (long)g_current_speed_hz[2],
               g_gripper_active ? 1 : 0);

      uint8_t tel_cs = 0;
      for (size_t k = 0; k < strlen(tel_payload); k++) {
        tel_cs ^= (uint8_t)tel_payload[k];
      }
      printf("$%s*%02X\r\n", tel_payload, tel_cs);

      // 2. Trama Enmarcada $ENC con Estado de Conexión de los 3 Encoders AS5600
      // Formato:
      // $ENC,1,<VALID>,<MD>,<RAW>,2,<VALID>,<MD>,<RAW>,3,<VALID>,<MD>,<RAW>*<CS>\r\n
      char enc_payload[96];
      snprintf(enc_payload, sizeof(enc_payload),
               "ENC,1,%d,%d,%d,2,%d,%d,%d,3,%d,%d,%d",
               g_last_reading[0].valid ? 1 : 0,
               g_last_reading[0].magnet_detected ? 1 : 0,
               g_last_reading[0].raw_angle, g_last_reading[1].valid ? 1 : 0,
               g_last_reading[1].magnet_detected ? 1 : 0,
               g_last_reading[1].raw_angle, g_last_reading[2].valid ? 1 : 0,
               g_last_reading[2].magnet_detected ? 1 : 0,
               g_last_reading[2].raw_angle);
      uint8_t enc_cs = 0;
      for (size_t k = 0; k < strlen(enc_payload); k++)
        enc_cs ^= (uint8_t)enc_payload[k];
      printf("$%s*%02X\r\n", enc_payload, enc_cs);

      // 3. Tramas Individuales $TEL por motor (solo si el encoder responde por
      // I2C)
      for (int m = 0; m < NUM_MOTORS; m++) {
        if (g_last_reading[m].valid) {
          char tel_ind[96];
          snprintf(tel_ind, sizeof(tel_ind), "TEL,%d,%.2f,%u,%ld,%.2f,%d,%u,%d",
                   m + 1, g_last_reading[m].calibrated_deg,
                   g_last_reading[m].raw_angle, (long)g_current_speed_hz[m],
                   g_current_error[m], (int)g_motor_state[m],
                   g_last_reading[m].agc,
                   g_last_reading[m].magnet_detected ? 1 : 0);
          uint8_t ind_cs = 0;
          for (size_t k = 0; k < strlen(tel_ind); k++)
            ind_cs ^= (uint8_t)tel_ind[k];
          printf("$%s*%02X\r\n", tel_ind, ind_cs);
        }
      }

      // 4. Impresión Periódica en Consola (Monitoreo cada 1 segundo para no
      // saturar el bus serial)
      static int s_debug_txt_timer = 0;
      if (++s_debug_txt_timer >= 10) {
        s_debug_txt_timer = 0;
        printf("---------------------------------------------------------------"
               "-------\n");
        printf("M1(Base):   %6.2f° (Vueltas NEMA: %5.2f/31) | Tgt: %6.2f° | "
               "Vel: %4ld Hz | Raw: %4d | MD:%d\n",
               g_last_reading[0].calibrated_deg,
               g_last_reading[0].motor_turns_count, g_target_deg[0],
               (long)g_current_speed_hz[0], g_last_reading[0].raw_angle,
               g_last_reading[0].magnet_detected);
        printf("M2(Hombro): %6.2f° (Vueltas NEMA: %5.2f/31) | Tgt: %6.2f° | "
               "Vel: %4ld Hz | Raw: %4d | MD:%d\n",
               g_last_reading[1].calibrated_deg,
               g_last_reading[1].motor_turns_count, g_target_deg[1],
               (long)g_current_speed_hz[1], g_last_reading[1].raw_angle,
               g_last_reading[1].magnet_detected);
        printf("M3(Codo):   %6.2f° (Vueltas NEMA: %5.2f/31) | Tgt: %6.2f° | "
               "Vel: %4ld Hz | Raw: %4d | MD:%d\n",
               g_last_reading[2].calibrated_deg,
               g_last_reading[2].motor_turns_count, g_target_deg[2],
               (long)g_current_speed_hz[2], g_last_reading[2].raw_angle,
               g_last_reading[2].magnet_detected);
        printf("Gripper: %s | Errs: [CS:%lu, OVF:%lu, RNG:%lu]\n",
               g_gripper_active ? "ON" : "OFF",
               (unsigned long)err_checksum_count,
               (unsigned long)err_overflow_count,
               (unsigned long)err_out_of_range_count);
        printf("---------------------------------------------------------------"
               "-------\n\n");
      }
      fflush(stdout);
    }

    vTaskDelay(
        pdMS_TO_TICKS(5)); // Bucle de procesamiento serie ultra-rápido a 200 Hz
  }
}
