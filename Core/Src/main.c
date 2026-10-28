/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { AX_IDLE = 0, AX_HOMING, AX_MOVING, AX_SETTLE } ax_state_t;   /* order is used to index "IHMS" */
typedef enum { EV_NONE = 0, EV_DONE, EV_HOMED, EV_STALL, EV_FAULT } ax_evt_t;

typedef struct
{
  char name;                                   /* 'X' or 'Z' */
  void (*fwd)(void);                           /* ahead */
  void (*rev)(void);                           /* back toward zero */
  void (*stop)(void);
  void (*zero)(void);                          /* make the current position = 0 */
  float   counts_per_mm;
  int32_t brake_init_fwd, brake_init_rev;
  volatile int32_t brake_fwd, brake_rev;       /* adaptive, per direction */
  volatile ax_state_t state;
  volatile uint8_t flag;                       /* 0 ok, 1 overshoot warning, 2 fault */
  volatile uint8_t req_move, req_home;
  volatile int32_t req_target;                 /* counts, absolute */
  volatile int32_t pos;                        /* counts, + = ahead, 0 = home */
  int32_t target, start, stop_at, travel, last;
  int8_t  dir;
  uint8_t ran, learn, zero_valid;
  uint16_t still;
  uint32_t t_ms;
  volatile ax_evt_t evt;
  volatile uint8_t  warn_evt;
  volatile int32_t  ev_target, ev_final, ev_err, ev_brake, ev_over;
  const char * volatile ev_text;
} axis_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- X axis (horizontal): TIM2 PA0/PA1, L298N IN1/IN2 on PB0/PB1 ---- */
#define X_COUNTS_PER_MM   827.5f
#define X_BRAKE_FWD       252L
#define X_BRAKE_REV       257L
#define X_MAX_MM          100.0f            /* set to the real travel (GUI has the same limit) */

/* ---- Z axis (vertical): TIM3 PA6/PA7, L298N IN3/IN4 on PB4/PB5 ---- */
#define Z_COUNTS_PER_MM   734.5f
#define Z_BRAKE_FWD       770L
#define Z_BRAKE_REV       (-25L)
#define Z_EXT_MM          6.0f              /* "Z1" position, put the real stroke here */

/* ---- common ---- */
#define STILL_TOL         2
#define STILL_MS          500               /* no counts this long while moving/homing = stalled */
#define SETTLE_MS         300
#define WRONGDIR_COUNTS   400L
#define OVERRUN_COUNTS    1600L
#define STALL_OK_COUNTS   100L              /* blocked this close to the target = arrived (end stop) */
#define HOME_TIMEOUT_MS   60000UL
#define OVER_LIMIT_MM     0.5f              /* overshoot beyond this -> WARN, please home */
#define LEARN_ENABLE      1
#define LEARN_DIV         2                 /* learn 1/2 of the measured overshoot each move */
#define BRAKE_TRIM        200L              /* learned brake stays within init +/- this */
#define PC_HB_TIMEOUT_MS  1000UL
#define STM_HB_PERIOD_MS  500
#define CMDQ_N            4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static axis_t   x_ax, z_ax;
static uint16_t z_last_cnt;                        /* TIM3 is 16 bit, integrated into z_ax.pos */

static uint8_t  rx_byte, rx_len;
static char     rx_buf[32];
static char     cmd_q[CMDQ_N][32];
static volatile uint8_t  q_head, q_tail;

static volatile uint8_t  stop_req, hb_flag, hb_lost, hb_loss_evt, pc_armed;
static volatile uint16_t hb_ms;
static volatile uint32_t pc_age_ms;
static uint32_t hb_seq;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline int32_t iabs32(int32_t v) { return v < 0 ? -v : v; }
static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* L298N: ahead = IN2 high, back = IN1 high (same as your tested code). BSRR changes both pins in one write. */
static void x_fwd(void)  { GPIOB->BSRR = X_IN2_Pin | ((uint32_t)X_IN1_Pin << 16); }
static void x_rev(void)  { GPIOB->BSRR = X_IN1_Pin | ((uint32_t)X_IN2_Pin << 16); }
static void x_stop(void) { GPIOB->BSRR = ((uint32_t)(X_IN1_Pin | X_IN2_Pin)) << 16; }
static void z_fwd(void)  { GPIOB->BSRR = Z_IN2_Pin | ((uint32_t)Z_IN1_Pin << 16); }
static void z_rev(void)  { GPIOB->BSRR = Z_IN1_Pin | ((uint32_t)Z_IN2_Pin << 16); }
static void z_stop(void) { GPIOB->BSRR = ((uint32_t)(Z_IN1_Pin | Z_IN2_Pin)) << 16; }

static void x_zero(void) { __HAL_TIM_SET_COUNTER(&htim2, 0); x_ax.pos = 0; }
static void z_zero(void) { __HAL_TIM_SET_COUNTER(&htim3, 0); z_last_cnt = 0; z_ax.pos = 0; }

static void axes_init(void)
{
  x_ax.name = 'X'; x_ax.fwd = x_fwd; x_ax.rev = x_rev; x_ax.stop = x_stop; x_ax.zero = x_zero;
  x_ax.counts_per_mm = X_COUNTS_PER_MM;
  x_ax.brake_init_fwd = x_ax.brake_fwd = X_BRAKE_FWD;
  x_ax.brake_init_rev = x_ax.brake_rev = X_BRAKE_REV;

  z_ax.name = 'Z'; z_ax.fwd = z_fwd; z_ax.rev = z_rev; z_ax.stop = z_stop; z_ax.zero = z_zero;
  z_ax.counts_per_mm = Z_COUNTS_PER_MM;
  z_ax.brake_init_fwd = z_ax.brake_fwd = Z_BRAKE_FWD;
  z_ax.brake_init_rev = z_ax.brake_rev = Z_BRAKE_REV;
}

/* position readers (called every 1 ms): both axes inverted like ENC_INVERT = 1 in the tested code */
static void x_read(void) { x_ax.pos = -(int32_t)__HAL_TIM_GET_COUNTER(&htim2); }
static void z_read(void)
{
  uint16_t c = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  z_ax.pos -= (int16_t)(c - z_last_cnt);          /* signed 16-bit difference is correct across wrap */
  z_last_cnt = c;
}

static long cnt2um(int32_t c, float cpm)
{
  float v = (float)c * 1000.0f / cpm;
  return (long)(v >= 0 ? v + 0.5f : v - 0.5f);
}

static void uart_puts(const char *s) { HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100); }

/* ---------------- axis motion (1 kHz, called from the TIM6 interrupt) ---------------- */
static void axis_abort(axis_t *a)
{
  a->stop();
  a->state = AX_IDLE;
  a->req_move = 0;
  a->req_home = 0;
}

static void axis_fault(axis_t *a, const char *why, int32_t p)
{
  a->stop();
  a->ev_final = p;
  a->ev_text = why;
  a->evt = EV_FAULT;
  a->flag = 2;
  a->state = AX_IDLE;
}

static void axis_start_home(axis_t *a, int32_t p)
{
  a->flag = 0;
  a->start = p; a->last = p; a->still = 0; a->t_ms = 0;
  a->rev();                                        /* retract until the counts stop */
  a->state = AX_HOMING;
}

static void axis_start_move(axis_t *a, int32_t tgt, int32_t p)
{
  int32_t delta = tgt - p;
  a->dir = (delta >= 0) ? 1 : -1;
  a->start = p; a->target = tgt; a->travel = iabs32(delta);
  a->last = p; a->still = 0; a->flag = 0;
  a->learn = (a->travel >= (int32_t)a->counts_per_mm);            /* moves shorter than 1 mm use no brake offset */
  int32_t brake = a->learn ? ((a->dir > 0) ? a->brake_fwd : a->brake_rev) : 0;
  a->stop_at = (a->dir > 0) ? (tgt - brake) : (tgt + brake);     /* where the motor is switched off */
  a->ran = 0;
  if (a->travel == 0 || (a->dir > 0 ? (p >= a->stop_at) : (p <= a->stop_at))) { a->state = AX_SETTLE; return; }
  a->ran = 1;
  if (a->dir > 0) a->fwd(); else a->rev();
  a->state = AX_MOVING;
}

static void axis_tick(axis_t *a)
{
  int32_t p = a->pos;

  switch (a->state)
  {
    case AX_IDLE:
      if (a->req_home)      { a->req_home = 0; axis_start_home(a, p); }
      else if (a->req_move) { a->req_move = 0; axis_start_move(a, a->req_target, p); }
      break;

    case AX_HOMING:
      if (iabs32(p - a->last) <= STILL_TOL) a->still++; else { a->still = 0; a->last = p; }
      a->t_ms++;
      if ((a->start - p) < -WRONGDIR_COUNTS)  axis_fault(a, "WRONGDIR", p);
      else if (a->t_ms >= HOME_TIMEOUT_MS)    axis_fault(a, "HOMETIMEOUT", p);
      else if (a->still >= STILL_MS)
      {
        a->stop();
        a->ev_err = a->zero_valid ? p : 0;         /* how far the old zero was from the new one */
        a->zero();
        a->zero_valid = 1;
        a->flag = 0;
        a->evt = EV_HOMED;
        a->state = AX_IDLE;
      }
      break;

    case AX_MOVING:
    {
      uint8_t reached = (a->dir > 0) ? (p >= a->stop_at) : (p <= a->stop_at);
      int32_t progress = (p - a->start) * a->dir;
      if (reached)
      {
        a->stop();
        a->last = p; a->still = 0;
        a->state = AX_SETTLE;
      }
      else if (progress < -WRONGDIR_COUNTS)                axis_fault(a, "WRONGDIR", p);
      else if (progress > a->travel + OVERRUN_COUNTS)      axis_fault(a, "OVERRUN", p);
      else
      {
        if (iabs32(p - a->last) <= STILL_TOL) a->still++; else { a->still = 0; a->last = p; }
        if (a->still >= STILL_MS)                          /* not moving any more */
        {
          a->stop();
          if (iabs32(p - a->target) <= STALL_OK_COUNTS)    /* blocked at the target (end stop) = arrived */
          {
            a->learn = 0; a->last = p; a->still = 0;
            a->state = AX_SETTLE;
          }
          else
          {
            a->ev_final = p;
            a->evt = EV_STALL;
            a->flag = 2;
            a->state = AX_IDLE;
          }
        }
      }
      break;
    }

    case AX_SETTLE:
      if (iabs32(p - a->last) <= STILL_TOL) a->still++; else { a->still = 0; a->last = p; }
      if (a->still >= SETTLE_MS)
      {
        int32_t err  = p - a->target;
        int32_t over = err * a->dir;                       /* + = went past the target */
#if LEARN_ENABLE
        if (a->ran && a->learn)                            /* adaptive brake, separate for each direction */
        {
          if (a->dir > 0) a->brake_fwd = clampi(a->brake_fwd + over / LEARN_DIV,
                                                a->brake_init_fwd - BRAKE_TRIM, a->brake_init_fwd + BRAKE_TRIM);
          else            a->brake_rev = clampi(a->brake_rev + over / LEARN_DIV,
                                                a->brake_init_rev - BRAKE_TRIM, a->brake_init_rev + BRAKE_TRIM);
        }
#endif
        a->ev_target = a->target; a->ev_final = p; a->ev_err = err; a->ev_over = over;
        a->ev_brake = (a->dir > 0) ? a->brake_fwd : a->brake_rev;
        a->evt = EV_DONE;
        if (over > (int32_t)(OVER_LIMIT_MM * a->counts_per_mm)) { a->flag = 1; a->warn_evt = 1; }
        a->state = AX_IDLE;
      }
      break;

    default: a->state = AX_IDLE; break;
  }
}

/* ---------------- reporting (main loop only, never in an ISR) ---------------- */
static char st_char(const axis_t *a)
{
  if (a->state != AX_IDLE) return "IHMS"[a->state];
  return (a->flag == 2) ? 'E' : ((a->flag == 1) ? 'W' : 'I');
}

static void send_heartbeat(void)
{
  char m[64];
  snprintf(m, sizeof m, "HB,%lu,%c,%ld,%c,%ld\r\n", (unsigned long)hb_seq++,
           st_char(&x_ax), cnt2um(x_ax.pos, X_COUNTS_PER_MM),
           st_char(&z_ax), cnt2um(z_ax.pos, Z_COUNTS_PER_MM));
  uart_puts(m);
}

static void report_axis(axis_t *a)
{
  char m[96];
  float c = a->counts_per_mm;

  if (a->evt != EV_NONE)
  {
    ax_evt_t e = a->evt;
    int32_t t = a->ev_target, f = a->ev_final, er = a->ev_err, b = a->ev_brake;
    const char *why = a->ev_text;
    a->evt = EV_NONE;
    if (e == EV_DONE)
      snprintf(m, sizeof m, "DONE,%c,%ld,%ld,%ld,%ld\r\n", a->name, cnt2um(t, c), cnt2um(f, c), cnt2um(er, c), (long)b);
    else if (e == EV_HOMED)
      snprintf(m, sizeof m, "HOMED,%c,%ld\r\n", a->name, cnt2um(er, c));
    else if (e == EV_STALL)
      snprintf(m, sizeof m, "STALL,%c,%ld\r\n", a->name, cnt2um(f, c));
    else
      snprintf(m, sizeof m, "FAULT,%c,%s,%ld\r\n", a->name, why ? why : "?", cnt2um(f, c));
    uart_puts(m);
  }
  if (a->warn_evt)
  {
    a->warn_evt = 0;
    snprintf(m, sizeof m, "WARN,%c,OVERLIMIT,%ld,%ld,%ld\r\n", a->name,
             cnt2um(a->ev_final, c), cnt2um(a->ev_target, c), cnt2um(a->ev_over, c));
    uart_puts(m);
  }
}

static void report_events(void)
{
  report_axis(&x_ax);
  report_axis(&z_ax);
  if (hb_loss_evt)
  {
    char m[64];
    hb_loss_evt = 0;
    snprintf(m, sizeof m, "INFO,HBLOSS,%ld,%ld\r\n", cnt2um(x_ax.pos, X_COUNTS_PER_MM), cnt2um(z_ax.pos, Z_COUNTS_PER_MM));
    uart_puts(m);
  }
}

/* ---------------- command parser (main loop) ---------------- */
static uint8_t axis_busy(const axis_t *a) { return (a->state != AX_IDLE || a->req_move || a->req_home); }

static void handle_command(char *s)
{
  char m[48];
  for (char *c = s; *c; c++) *c = (char)toupper((unsigned char)*c);

  if (s[0] == 'P' && s[1] == 'C')                         /* PC,<seq> heartbeat from the GUI */
  {
    pc_age_ms = 0; pc_armed = 1; hb_lost = 0;
    return;
  }
  if (hb_lost && s[0] != 'S' && strcmp(s, "LOG?") != 0)   /* no PC heartbeat: refuse motion */
  {
    uart_puts("NAK,NOHB\r\n");
    return;
  }

  if (strcmp(s, "LOG?") == 0)                             /* flash log not implemented yet */
  {
    uart_puts("LOG,0,0,0\r\n");
  }
  else if (s[0] == 'S' && s[1] == 0)                      /* S = stop everything */
  {
    stop_req = 1;
    uart_puts("OK,STOP\r\n");
  }
  else if (s[0] == 'X')                                   /* X20.870 = absolute position in mm */
  {
    float mm = (float)atof(&s[1]);
    if (s[1] == 0)                          uart_puts("NAK,BAD_VALUE\r\n");
    else if (mm < 0.0f || mm > X_MAX_MM)    uart_puts("NAK,RANGE\r\n");
    else if (axis_busy(&x_ax))              uart_puts("NAK,BUSY\r\n");
    else
    {
      x_ax.req_target = (int32_t)(mm * X_COUNTS_PER_MM + 0.5f);
      x_ax.req_move = 1;
      snprintf(m, sizeof m, "OK,X,%ld\r\n", cnt2um(x_ax.req_target, X_COUNTS_PER_MM));
      uart_puts(m);
    }
  }
  else if (s[0] == 'Z')                                   /* Z1 = extended, Z0 = home position */
  {
    if (s[1] != '0' && s[1] != '1')         uart_puts("NAK,BAD_VALUE\r\n");
    else if (axis_busy(&z_ax))              uart_puts("NAK,BUSY\r\n");
    else
    {
      z_ax.req_target = (s[1] == '1') ? (int32_t)(Z_EXT_MM * Z_COUNTS_PER_MM + 0.5f) : 0;
      z_ax.req_move = 1;
      uart_puts((s[1] == '1') ? "OK,Z,1\r\n" : "OK,Z,0\r\n");
    }
  }
  else if (s[0] == 'H' && (s[1] == 'X' || s[1] == 'Z'))   /* HX / HZ = home that axis */
  {
    axis_t *a = (s[1] == 'X') ? &x_ax : &z_ax;
    if (axis_busy(a))                       uart_puts("NAK,BUSY\r\n");
    else
    {
      a->req_home = 1;
      snprintf(m, sizeof m, "OK,H%c\r\n", s[1]);
      uart_puts(m);
    }
  }
  else uart_puts("NAK,BAD_CMD\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  axes_init();
  x_stop(); z_stop();                                   /* motors off first */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  z_last_cnt = 0;
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  HAL_TIM_Base_Start_IT(&htim6);                        /* 1 kHz tick last, after all state is initialised */
  uart_puts("HELLO,STAGE,2\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    if (hb_flag) { hb_flag = 0; send_heartbeat(); }
	    report_events();
	    if (q_tail != q_head)
	    {
	      char line[32];
	      strcpy(line, cmd_q[q_tail]);
	      q_tail = (uint8_t)((q_tail + 1) % CMDQ_N);
	      handle_command(line);
	    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, X_IN1_Pin|X_IN2_Pin|Z_IN1_Pin|Z_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : X_IN1_Pin X_IN2_Pin Z_IN1_Pin Z_IN2_Pin */
  GPIO_InitStruct.Pin = X_IN1_Pin|X_IN2_Pin|Z_IN1_Pin|Z_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)      /* TIM6 = 1 kHz */
{
  if (htim->Instance != TIM6) return;

  if (pc_armed && !hb_lost && ++pc_age_ms >= PC_HB_TIMEOUT_MS)   /* PC heartbeat lost */
  {
    hb_lost = 1;
    hb_loss_evt = 1;
    axis_abort(&x_ax); axis_abort(&z_ax);
    x_ax.flag = 2; z_ax.flag = 2;
    /* future: x_ax.pos and z_ax.pos are the positions to store in flash here */
  }
  if (stop_req)
  {
    stop_req = 0;
    axis_abort(&x_ax); axis_abort(&z_ax);
  }

  x_read(); z_read();
  axis_tick(&x_ax);
  axis_tick(&z_ax);
  if (++hb_ms >= STM_HB_PERIOD_MS) { hb_ms = 0; hb_flag = 1; }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    char ch = (char)rx_byte;
    if (ch == '\r' || ch == '\n')
    {
      if (rx_len > 0 && rx_len != 0xFF)
      {
        uint8_t nh = (uint8_t)((q_head + 1) % CMDQ_N);
        if (nh != q_tail)                                /* queue not full */
        {
          rx_buf[rx_len] = 0;
          strcpy(cmd_q[q_head], rx_buf);
          q_head = nh;
        }
      }
      rx_len = 0;
    }
    else if (rx_len < sizeof(rx_buf) - 1) rx_buf[rx_len++] = ch;
    else rx_len = 0xFF;                                  /* line too long: drop it completely */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  x_stop(); z_stop();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
