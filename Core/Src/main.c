/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>
#include <stdbool.h>

#ifdef PRINT_DEBUG
#include <stdio.h>
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum { MOTOR_STOP, MOTOR_FORWARD, MOTOR_BACKWARD } MotorState;

typedef enum { BUZZER_ON, BUZZER_OFF, BUZZER_INTERMITTENT } BuzzerMode;

typedef enum { MODE_MANUAL, MODE_AUTOMATIC } OperationMode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// TIM4 is set up with a PSC of 31999, meaning that it will increase
// in count once every 1ms
//
// Toggle buzzer once every 0.3s
#define BUZZER_TOGGLE_TIME 300
// Run the ADC once every 0.2s
#define ADC_TRIGGER_TIME 200

// MAX and MIN values are chosen by MAX being the maximum resolution
// of the ADC that has been set and MIN can be any arbitrary number
// between MAX and 0. The combination of both is used to control
// possible speed values of the motors
#define MAX_SPEED_CONTROL_VALUE 1024
#define MIN_SPEED_CONTROL_VALUE 350
// LED matrix display brightness. 0-F.
#define DISPLAY_BRIGHTNESS 0
#define DISPLAY_TX_TIMEOUT 1

#define UART_BUFFER_SIZE 10
#define UART_TIMEOUT 10

#define REPORT_MSG_MAX_LENGTH 64

#define HALF_RPMS_PER_TIM2_TICK 150000
// HALF_RPMS_PER_TIM2_TICK is calculated as:
//        DISC_SLITS = 20
//        TIM2_TICK_uS = (1e6 * (TIM2_PSC + 1) / 32e6)
//        RPMS_PER_TIM2_TICK = 1e6 * 60 / (DISC_SLITS * TIM2_TICK_uS) = 30000
//        HALF_RPMS_PER_TIM2_TICK = RPMS_PER_TIM2_TICK / 2 = 150000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Report templates
static const char* TEMPLATE_STATUS = "($M) $L : $R @ $p% of $s ($l : $r rpm)\n";
static const char* TEMPLATE_STATUS_SHORT = "$M>$L:$R@$p%$s($l:$r)\n";

// Display designs
static const uint64_t speedometer_display[8] = {
    0x3804022222222438,  // speedometer_1.txt
    0x3804022222120c38,  // speedometer_2.txt
    0x38040222120a0438,  // speedometer_3.txt
    0x380402221e020438,  // speedometer_4.txt
    0x3804021e22020438,  // speedometer_5.txt
    0x38040a1222020438,  // speedometer_6.txt
    0x380c122222020438,  // speedometer_7.txt
    0x3824222222020438,  // speedometer_8.txt
};

OperationMode current_mode = MODE_MANUAL;
MotorState right_motor_state = MOTOR_STOP;
MotorState left_motor_state = MOTOR_STOP;
uint8_t speed_modifier = 100;  // value range is [0, 100] (both inclusive)
uint16_t current_speed = 0;    // value range is defined by MAX/MIN_SPEED_CONTROL_VALUE
// Not expecting these to go over 255...
uint8_t measured_rpm_right = 0;
uint8_t measured_rpm_left = 0;

char uart1_receive_buffer[UART_BUFFER_SIZE];
char uart1_receive_byte = 0;
uint8_t uart1_receive_buffer_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if PRINT_DEBUG
// Provide a _write that printf and similar will use to write to serial
int _write(int file, char* message, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*) message, len, 1000);
    return len;
}
#else
// An empty define will make is that any uses of printf will just
// be consumed and treated as noop calls
#define printf
#endif  // PRINT_DEBUG

// All these inline static functions are short pieces of code used in more
// than one place. As such, and to streamline potentially updating their functionality,
// they are extracted in code, but inlined at compile time to avoid any runtime penalties
// from function calls.

inline __attribute__((always_inline)) void display_send(
    SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint8_t cs_pin, uint16_t data
) {
    cs_port->BSRR |= 1 << (cs_pin + 16);  // Reset pin -> LOW -> Chip Selected
    // HAL_SPI_Transmit signature doesn't change based on whether 16 bit words or
    // 8 bit words is selected, but the functionality does. Actually, the function
    // is a pointer to 16 bit, not 8. The cast is used to silence the warning.
    HAL_SPI_Transmit(hspi, (uint8_t*) (&data), 1, DISPLAY_TX_TIMEOUT);
    cs_port->BSRR |= 1 << cs_pin;  // Set pin -> HIGH -> Chip Deselected
}

/**
 * @brief Send a pattern to display on an 8x8 LED matrix found through hspi. Expects SPI and GPIO set up.
 * @param hspi SPI Handler from HAL
 * @param pattern 8-byte pattern containing the on-pattern to be displayed. Each byte corresponds to one
 * column and each bit in the byte corresponds to one one row. The order in the pattern is left-to-right,
 * top-to-bottom in in Big-endian reading order (MSB first)
 */
inline __attribute__((always_inline)) void display_draw(
    SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint8_t cs_pin, uint64_t pattern
) {
    // Each column is addressed by setting the column number (1..8) in the third half-byte, and the
    // content in the first byte.
    // For a pattern 0xABCDEFGHIJKLMNOP, we will send 0x01AB, 0x02CD, 0x03EF, 0x04GH, 0x05IJ, 0x06KL, etc.
    uint8_t* lines = (uint8_t*) &pattern;
    for (uint8_t i = 1; i <= 8; i++) {  // Loop through the columns 1..8
        uint16_t pattern_instruction = (i << 8) | lines[i - 1];
        display_send(hspi, cs_port, cs_pin, pattern_instruction);
    }
}

inline static __attribute__((always_inline)) void sleep(unsigned short milliseconds) {
    // TIM4 CH1 is used for sleeps, which MUST have been setup by the time this is used.
    // It must be setup with millisecond precision (PSC = 31999)
    //
    // Setup counter with value to reach
    TIM4->CCR1 = TIM4->CNT + milliseconds;  // Set value to reach
    TIM4->SR &= ~0b10;                      // Clear any lingering events

    // Wait
    while ((TIM4->SR & 0b10) == 0) {}

    // Clear event
    TIM4->SR &= ~0b10;
}

// This is optimized correctly and adds clarity
inline static __attribute__((always_inline)) void set_buzzer(BuzzerMode status) {
    TIM4->CCMR2 &= ~(0x00FF);  // Clear CH3 info

    // Set OCEM to 0b100 (low) to turn buzzer ON, to 0b101 (high) to turn it OFF
    // It is also initially turned ON for INTERMITTENT mode
    TIM4->CCMR2 |= (status == BUZZER_OFF ? 0b101 : 0b100) << 4;

    if (status == BUZZER_INTERMITTENT) {
        // Setup periodic changes
        TIM4->CCR3 = TIM4->CNT + BUZZER_TOGGLE_TIME;  // Set the value to reach
        TIM4->CCMR2 &= ~(0x00FF);                     // Clear CH3 info
        TIM4->CCMR2 |= 0b011 << 4;                    // OC3M = 011 (toggle external output)
    }
}

/**
 * @brief Update the robot's feedback systems. These include the buzzer, LED matrix, and Bluetooth
 * notifications
 */
inline static __attribute__((always_inline)) void update_feedback() {
    if (right_motor_state == left_motor_state) {
        // Both motors are in the same state -> either Forward, Backward or Stopped
        // Reading one motor's status is enough from this point on.
        //
        // Splitting with an IF allows for nicer compiler optimizations in each of the paths
        if (right_motor_state == MOTOR_FORWARD) {
            set_buzzer(BUZZER_OFF);
            HAL_UART_Transmit(&huart1, (uint8_t*) "Moving forward\n", 15, UART_TIMEOUT);
        } else {
            set_buzzer(BUZZER_ON);
            HAL_UART_Transmit(&huart1, (uint8_t*) "Stopped\n", 8, UART_TIMEOUT);
        }
    } else {
        set_buzzer(BUZZER_INTERMITTENT);

        const char* msg = right_motor_state == MOTOR_FORWARD ? "Turning left \n" : "Turning right\n";
        HAL_UART_Transmit(&huart1, (uint8_t*) msg, 14, UART_TIMEOUT);
    }
}

inline static __attribute__((always_inline)) void update_right_motor() {
    switch (right_motor_state) {
        case MOTOR_FORWARD:
            TIM3->CCR3 = current_speed * speed_modifier / 100;
            TIM3->CCR4 = 0;
            break;
        case MOTOR_BACKWARD:
            TIM3->CCR3 = 0;
            TIM3->CCR4 = current_speed * speed_modifier / 100;
            break;
        default:
            TIM3->CCR3 = 0;
            TIM3->CCR4 = 0;
            measured_rpm_right = 0;
            break;
    }
}

inline static __attribute__((always_inline)) void update_left_motor() {
    switch (left_motor_state) {
        case MOTOR_FORWARD:
            TIM3->CCR1 = current_speed * speed_modifier / 100;
            TIM3->CCR2 = 0;
            break;
        case MOTOR_BACKWARD:
            TIM3->CCR1 = 0;
            TIM3->CCR2 = current_speed * speed_modifier / 100;
            break;
        default:
            TIM3->CCR1 = 0;
            TIM3->CCR2 = 0;
            measured_rpm_left = 0;
            break;
    }
}

inline static __attribute__((always_inline)) void right_ir_update() {
    right_motor_state = ((GPIOC->IDR & (1 << 1)) == 0) ? MOTOR_FORWARD : MOTOR_STOP;
}

inline static __attribute__((always_inline)) void left_ir_update() {
    left_motor_state = ((GPIOC->IDR & (1 << 2)) == 0) ? MOTOR_FORWARD : MOTOR_STOP;
}

inline static __attribute__((always_inline)) void set_manual_mode() {
    current_mode = MODE_MANUAL;

    GPIOA->BSRR = 1 << 5;  // Turn on indicator LED
    // IMPORTANT: When setting ICER, do not use any bitwise operators or the
    // whole registry will get flushed!!
    NVIC->ICER[0] = 1 << 7;  // Disable EXTI1 handler (IR1)
    NVIC->ICER[0] = 1 << 8;  // Disable EXTI2 handler (IR2)

    // Stop motors
    right_motor_state = MOTOR_STOP;
    left_motor_state = MOTOR_STOP;
    update_right_motor();
    update_left_motor();
    // Disable the buzzer
    TIM4->CCMR2 &= ~(0x00FF);   // Clear CH3 info
    TIM4->CCMR2 |= 0b101 << 4;  // Force output to high (101) -> turn buzzer off
}

inline static __attribute__((always_inline)) void set_automatic_mode() {
    current_mode = MODE_AUTOMATIC;

    GPIOA->BSRR = 1 << (5 + 16);  // Turn off indicator LED
    NVIC->ISER[0] = 1 << 7;       // Enable EXTI1 handler (IR1)
    NVIC->ISER[0] = 1 << 8;       // Enable EXTI2 handler (IR2)

    // Initial update
    right_ir_update();
    left_ir_update();
    update_right_motor();
    update_left_motor();
    update_feedback();
}

/**
 * @brief Send a status report over UART1
 *
 * @param template A string template for the report sent. The following fields are supported:
 * $M -> Operating mode: A|M (len=1)
 * $L -> Left motor status: -1|0|1 (len=2)
 * $R -> Right motor status: -1|0|1 (len=2)
 * $l -> Left motor rpm: [0..99] (len=2)
 * $r -> Right motor rpm: [0..99] (len=2)
 * $s -> Speed value from potentiometer: [MIN..MAX] (len=4)
 * $p -> Speed modifier: [0..100] (len=3)
 */
inline static __attribute__((always_inline)) void send_status_report(const char* template) {
    char msg[REPORT_MSG_MAX_LENGTH];

    uint16_t n = 0;  // Number of characters set

    for (int i = 0; template[i] != '\0'; i++) {
        if (template[i] != '$') {  // Not a field -> Copy the character into out
            msg[n++] = template[i];
            continue;  // Having an early continue reduces indentation
        }
        // Field replacing
        switch (template[++i]) {
            case 'M':
                msg[n++] = (current_mode == MODE_MANUAL) ? 'M' : 'A';
                break;
            case 'L':
                msg[n++] = (left_motor_state == MOTOR_BACKWARD) ? '-' : ' ';
                msg[n++] = (left_motor_state == MOTOR_STOP) ? '0' : '1';
                break;
            case 'R':
                msg[n++] = (right_motor_state == MOTOR_BACKWARD) ? '-' : ' ';
                msg[n++] = (right_motor_state == MOTOR_STOP) ? '0' : '1';
                break;
            case 'l':
                msg[n++] = measured_rpm_left / 10 + '0';
                msg[n++] = measured_rpm_left % 10 + '0';
                break;
            case 'r':
                msg[n++] = measured_rpm_right / 10 + '0';
                msg[n++] = measured_rpm_right % 10 + '0';
                break;
            case 'p':
                msg[n++] = (speed_modifier == 100) ? '1' : ' ';
                msg[n++] = (speed_modifier < 10) ? ' ' : (speed_modifier / 10) % 10 + '0';
                msg[n++] = (speed_modifier % 10) + '0';
                break;
            case 's':
                msg[n++] = (current_speed < 1000) ? ' ' : current_speed / 1000 + '0';
                msg[n++] = (current_speed < 100) ? ' ' : (current_speed / 100) % 10 + '0';
                msg[n++] = (current_speed < 10) ? ' ' : (current_speed / 10) % 10 + '0';
                msg[n++] = current_speed % 10 + '0';
                break;
            default:
                msg[n++] = '?';
                break;
        }
    }

    HAL_UART_Transmit(&huart1, (uint8_t*) msg, n, UART_TIMEOUT);
}

// UART1 is the bluetooth module
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    uart1_receive_buffer[uart1_receive_buffer_index++] = uart1_receive_byte;
    HAL_UART_Receive_IT(huart, (uint8_t*) &uart1_receive_byte, 1);

    if (uart1_receive_buffer_index + 1 >= UART_BUFFER_SIZE) {
        // Unexpected: we received too many bytes, so just "flush" the data and do nothing with it
        uart1_receive_buffer_index = 0;
        return;
    }

    if (uart1_receive_byte == '\n') {
        // Switches should be efficient enough as they are precomputed gotos
        //
        // Possible commands are:
        //    'f' -> go forwards
        //    'b' -> go backwards
        //    'r' -> turn right
        //    'l' -> turn left
        //    's' -> stop motors
        //    'v{value}' -> Set speed to {value}
        switch (uart1_receive_buffer[0]) {
            case 'f':
                right_motor_state = MOTOR_FORWARD;
                left_motor_state = MOTOR_FORWARD;
                break;
            case 'b':
                right_motor_state = MOTOR_BACKWARD;
                left_motor_state = MOTOR_BACKWARD;
                break;
            case 'r':
                right_motor_state = MOTOR_BACKWARD;
                left_motor_state = MOTOR_FORWARD;
                break;
            case 'l':
                right_motor_state = MOTOR_FORWARD;
                left_motor_state = MOTOR_BACKWARD;
                break;
            case 's':
                right_motor_state = MOTOR_STOP;
                left_motor_state = MOTOR_STOP;
                break;
            case 'v':
                // Extract the number after the 'v'
                uint8_t num = strtoumax((char*) uart1_receive_buffer + 1, NULL, 10);

                if (num <= 100) {
                    speed_modifier = num;
                } else {
                    HAL_UART_Transmit(&huart1, (uint8_t*) "ERR:2\n", 6, UART_TIMEOUT);
                }
                break;
            case 'm':
                set_manual_mode();
                break;
            case 'a':
                HAL_UART_Transmit(&huart1, (uint8_t*) "Starting!\n", 10, UART_TIMEOUT);
                set_automatic_mode();
                break;
            case '?':
                send_status_report(TEMPLATE_STATUS);
                break;
            default:
                HAL_UART_Transmit(&huart1, (uint8_t*) "ERR:1\n", 6, UART_TIMEOUT);
                break;
        }

        update_right_motor();
        update_left_motor();

        uart1_receive_buffer_index = 0;
    }
}

// EXTI1 belongs to IR1 (right)
void EXTI1_IRQHandler() {
    EXTI->PR |= 1 << 1;  // Flag is cleared first because the IR are really finicky and
                         // they will fire several times. This way, we can ensure we don't
                         // end up with the wrong state, as EXTI will notice any other changes
                         // while we compute anything

    right_ir_update();
    update_right_motor();
    update_feedback();
}

// EXTI2 belongs to IR2 (left)
void EXTI2_IRQHandler() {
    EXTI->PR |= 1 << 2;  // Flag is cleared first because the IR are really finicky and
                         // will fire multiple times per change. Setting the flag first
                         // ensures the EXTI will warn us about any changes that can
                         // happen during the handling of the event, as well as
                         // not reading wrong information from the IDR

    left_ir_update();
    update_left_motor();
    update_feedback();
}

void ADC1_IRQHandler() {
    // Limit the value to not go lower than MIN_ADC_VALUE
    current_speed = ADC1->DR < MIN_SPEED_CONTROL_VALUE ? MIN_SPEED_CONTROL_VALUE : ADC1->DR;

    uint8_t level = (current_speed - MIN_SPEED_CONTROL_VALUE) /
                    ((MAX_SPEED_CONTROL_VALUE - MIN_SPEED_CONTROL_VALUE) / 8 + 1);

    display_draw(&hspi2, GPIOB, 1, speedometer_display[level]);

    update_right_motor();
    update_left_motor();
}

void TIM2_IRQHandler() {
    // Used so we can actually measure time lapsed
    static uint16_t count_start_left;
    static uint16_t count_start_right;
    // Used to average out the last two recorded intervals
    static uint16_t last_interval_left;
    static uint16_t last_interval_right;
    // This is where the current measurement will be stored
    uint16_t interval;

    if (TIM2->SR & 1 << 3) {
        // Channel 3 (left optocoupler) triggered

        // Must account for timer rolling over
        interval = (TIM2->CCR3 > count_start_left ? 0 : 0xFFFF) + TIM2->CCR3 - count_start_left;

        // Since we're averaging the last two measurements, we'll halve the factor.
        measured_rpm_left = HALF_RPMS_PER_TIM2_TICK / (interval + last_interval_left);

        last_interval_left = interval;
        count_start_left = TIM2->CNT;

        TIM2->SR &= ~(1 << 3);  // Clear flag

    } else if (TIM2->SR & (1 << 4)) {
        // Channel 4 (right optocoupler) triggered

        // Must account for timer rolling over
        interval = (TIM2->CCR4 > count_start_right ? 0 : 0xFFFF) + TIM2->CCR4 - count_start_right;

        // Since we're averaging the last two measurements, we'll halve the factor.
        measured_rpm_right = HALF_RPMS_PER_TIM2_TICK / (interval + last_interval_right);

        last_interval_right = interval;
        count_start_right = TIM2->CNT;

        TIM2->SR &= ~(1 << 4);  // Clear flag
    }
}

void TIM4_IRQHandler() {
    if (TIM4->SR & (1 << 2)) {
        // Channel 2 (ADC trigger) finished
        TIM4->CCR2 = TIM4->CNT + ADC_TRIGGER_TIME;  // Reschedule the timer
        ADC1->CR2 |= 0x40000000;                    // Start the conversion

        TIM4->SR &= ~(1 << 2);  // Clear flag
    } else if (TIM4->SR & (1 << 3)) {
        // Channel 3 (buzzer toggle) finished
        TIM4->CCR3 = TIM4->CNT + BUZZER_TOGGLE_TIME;  // Reschedule the timer

        TIM4->SR &= ~(1 << 3);  // Clear flag
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_ADC_Init();
    MX_SPI2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    /* USER CODE BEGIN 2 */

    ////////////////
    // GPIO SETUP //
    ////////////////
    // Digital Input      - 00
    // Digital Output     - 01
    // Alternate Function - 10
    // Analog             - 11

    // IN1 - left wheel forward - Alternate Function (linked to TIMER3 CH1 [AF2])
    GPIOC->MODER |= 1 << (6 * 2 + 1);
    GPIOC->MODER &= ~(1 << (6 * 2));
    GPIOC->AFR[0] |= 0x02 << (6 * 4);
    // IN2 - left wheel backwards - Alternate Function (linked to TIMER3 CH2 [AF2])
    GPIOC->MODER |= 1 << (7 * 2 + 1);
    GPIOC->MODER &= ~(1 << (7 * 2));
    GPIOC->AFR[0] |= 0x02 << (7 * 4);
    // IN3 - right wheel forward - Alternate Function (linked to TIMER3 CH3 [AF2])
    GPIOC->MODER |= 1 << (8 * 2 + 1);
    GPIOC->MODER &= ~(1 << (8 * 2));
    GPIOC->AFR[1] |= 0x02;
    // IN4 - right wheel backwards - Alternate Function (linked to TIMER3 CH4 [AF2])
    GPIOC->MODER |= 1 << (9 * 2 + 1);
    GPIOC->MODER &= ~(1 << (9 * 2));
    GPIOC->AFR[1] |= 0x02 << (1 * 4);

    // IR 1 (right) - Digital Input
    GPIOC->MODER &= ~(1 << (1 * 2 + 1));
    GPIOC->MODER &= ~(1 << (1 * 2));
    // IR 2 (left) - Digital Input
    GPIOC->MODER &= ~(1 << (2 * 2 + 1));
    GPIOC->MODER &= ~(1 << (2 * 2));

    // PB10 - Left optocoupler speed sensor - Alternate Function (linked to TIMER2 CH3 [AF1])
    GPIOB->MODER |= 1 << (10 * 2 + 1);
    GPIOB->MODER &= ~(1 << (10 * 2));
    GPIOB->AFR[1] |= 0x01 << (2 * 4);
    // PB11 - Right optocoupler speed sensor - Alternate Function (linked to TIMER2 CH4 [AF1])
    GPIOB->MODER |= 1 << (11 * 2 + 1);
    GPIOB->MODER &= ~(1 << (11 * 2));
    GPIOB->AFR[1] |= 0x01 << (3 * 4);

    // Potentiometer input - Analog
    GPIOA->MODER |= 1 << (4 * 2 + 1);
    GPIOA->MODER |= 1 << (4 * 2);

    // Buzzer - Alternate Function (linked to TIMER4 CH3 [AF2])
    GPIOB->MODER |= 1 << (8 * 2 + 1);
    GPIOB->MODER &= ~(1 << (8 * 2));
    GPIOB->AFR[1] |= 0x02;

    // User button - Digital Input
    GPIOC->MODER &= ~(1 << (13 * 2 + 1));
    GPIOC->MODER &= ~(1 << (13 * 2));

    // Green LED - Digital Output
    GPIOA->MODER &= ~(1 << (5 * 2 + 1));
    GPIOA->MODER |= 1 << (5 * 2);

    // LED Matrix Chip Select - Digital Output
    GPIOB->MODER &= ~(1 << (1 * 2 + 1));
    GPIOB->MODER |= 1 << (1 * 2);

    ////////////////
    // EXTI SETUP //
    ////////////////

    // IR 1 (PC1)
    SYSCFG->EXTICR[0] &= ~(0b1111 << 4);  // Reset EXTI1 settings
    SYSCFG->EXTICR[0] |= 0b0010 << 4;     // Associate PC1 its EXTI (EXTI1)
    EXTI->RTSR |= 1 << 1;                 // Enable rising edge for EXTI1
    EXTI->FTSR |= 1 << 1;                 // Enable falling edge for EXTI1
    EXTI->IMR |= 1 << 1;                  // Unmask EXTI1
    // Interrupt handler is enabled/disabled based on working mode

    // IR 2 (PC2)
    SYSCFG->EXTICR[0] &= ~(0b1111 << 8);  // Reset EXTI2 settings
    SYSCFG->EXTICR[0] |= 0b0010 << 8;     // Associate PC2 its EXTI (EXTI2)
    EXTI->RTSR |= 1 << 2;                 // Enable rising edge for EXTI2
    EXTI->FTSR |= 1 << 2;                 // Enable falling edge for EXTI2
    EXTI->IMR |= 1 << 2;                  // Unmask EXTI2
    // Interrupt handler enabled/disabled based on working mode

    ///////////////////////
    // ADC CONFIGURATION //
    ///////////////////////
    ADC1->CR2 &= ~(0x00000001);  // ADON = 0 (ADC powered off)
    ADC1->CR1 = 0x01000020;      // OVRIE = 0 (Overrun IRQ disabled)
                                 // RES = 01 (resolution = 10 bits)
                                 // SCAN = 0 (scan mode disabled)
                                 // EOCIE = 1 (EOC IRQ enabled)
    ADC1->CR2 = 0x00000400;      // EOCS = 1 (EOC to be activated after each conversion)
                                 // DELS = 000 (no delay)
                                 // CONT = 0 (single conversion)
                                 //   Conversions are started based on TIM4 CH2
    ADC1->SQR1 = 0x00000000;     // 1 channel in the sequence
    ADC1->SQR5 = 0x00000004;     // Channel is AIN4
    ADC1->CR2 |= 0x00000001;     // ADON = 1 (ADC powered on)
    NVIC->ISER[0] = 1 << 18;     // Enable the ADC interrupt

    while ((ADC1->SR & 0x0040) == 0) {}  // Wait for the converter to be ready
    // Conversion will be started by timer

    ////////////////
    // TIM2 SETUP //
    ////////////////
    // Used for optical speed measurement
    TIM2->CR1 = 0x0000;   // CKD, CMS, DIR, OPM, URS, UDIS = 0
                          // ARPE = 0 (ARR not used)
                          // CEN = 0 (will be enabled later when needed)
    TIM2->CR2 = 0x0000;   // Unused, recommended to set to 0x0000
    TIM2->PSC = 319;      // Usually 319 to allow for 0.01 milliseconds precision and up to 650 ms interval
    TIM2->SMCR = 0x0000;  // Unused, recommended to set to 0x0000
    TIM2->ARR = 0xFFFF;   // Unused, recommended to set to 0xFFFF

    // Channel 3
    // Linked to PB10 (left optocoupler)
    TIM2->CCMR2 &= ~(0xFF);         // Clear CH3 info
    TIM2->CCMR2 |= 0b01;            // CC3S = 01 (TIC)
                                    // IC3F, IC1PSC = 0 (as advised)
    TIM2->CCER &= ~(0xF << 2 * 4);  // Clear CH3 info
    TIM2->CCER |= 0b0001 << 2 * 4;  // CC3E = 1 (capture enabled)
                                    // CC3NP:CC3P = 00 (rising edge)
    TIM2->DIER |= 1 << 3;           // CC3IE = 1 (enable IRQ)

    // Channel 4
    // Linked to PB11 (right optocoupler)
    TIM2->CCMR2 &= ~(0xFF << 8);    // Clear CH1 info
    TIM2->CCMR2 |= 0b01 << 8;       // CC4S = 01 (TIC)
                                    // IC4F, IC1PSC = 0 (as advised)
    TIM2->CCER &= ~(0xF << 3 * 4);  // Clear CH4 info
    TIM2->CCER |= 0b0001 << 3 * 4;  // CC4E = 1 (capture enabled)
                                    // CC4NP:CC3P = 00 (rising edge)
    TIM2->DIER |= 1 << 4;           // CC4IE = 1 (enable IRQ)

    TIM2->SR = 0;              // Clear any lingering flags
    TIM2->EGR |= 1;            // Generate a register update
    NVIC->ISER[0] |= 1 << 28;  // Enable TIMER4 callback
    TIM2->CR1 |= 1;            // Enable the timer

    ////////////////
    // TIM3 SETUP //
    ////////////////
    TIM3->CR1 = 0x0080;                   // CKD, CMS, DIR, OPM, URS, UDIS = 0
                                          // ARPE = 1 (ARR used)
                                          // CEN = 0 (will be enabled later when needed)
    TIM3->CR2 = 0x0000;                   // Unused, recommended to set to 0x0000
    TIM3->PSC = 319;                      // Allow for sub millisecond precision (0.01 ms precision)
                                          // Allows us to control the motors with proper precision
    TIM3->SMCR = 0x0000;                  // Unused, recommended to set to 0x0000
    TIM3->ARR = MAX_SPEED_CONTROL_VALUE;  // Set to the max value the ADC will output
    TIM3->CNT = 0;                        // Initialize counter at 0
    TIM3->DIER = 0x0000;                  // Disable IRQ for when channels are finished counting

    // Channel 1
    //
    // Linked to PC6
    TIM3->CCR1 = 0;              // Start CCR1 at 0
    TIM3->CCMR1 &= ~(0x00FF);    // Clear CH1 info
    TIM3->CCMR1 |= 0b1101 << 3;  // CC1S = 00 (PWM)
                                 // OC1M = 110 (start PWM high)
                                 // OC1PE = 1 (enable preload)
    TIM3->CCER |= 0b0001;        // CC1NP = 0 (PWM)
                                 // CC1P = 0 (PWM)
                                 // CC1E = 1 (external output enabled)

    // Channel 2
    //
    // Linked to PC7
    TIM3->CCR2 = 0;                    // Start CCR2 at 0
    TIM3->CCMR1 &= ~(0xFF00);          // Clear CH2 info
    TIM3->CCMR1 |= 0b1101 << (3 + 8);  // CC2S = 00 (PWM)
                                       // OC2M = 110 (start PWM high)
                                       // OC2PE = 1 (enable preload)
    TIM3->CCER |= 0b0001 << 4;         // CC2NP = 0 (PWM)
                                       // CC2P = 0 (PWM)
                                       // CC2E = 1 (external output enabled)

    // Channel 3
    //
    // Linked to PC8
    TIM3->CCR3 = 0;                   // Start CCR3 at 0
    TIM3->CCMR2 &= ~(0x00FF);         // Clear CH3 info
    TIM3->CCMR2 |= 0b1101 << 3;       // CC3S = 00 (PWM)
                                      // OC3M = 110 (start PWM high)
                                      // OC3PE = 1 (enable preload)
    TIM3->CCER |= 0b0001 << (4 * 2);  // CC3NP = 0 (PWM)
                                      // CC3P = 0 (PWM)
                                      // CC3E = 1 (external output enabled)

    // Channel 4
    //
    // Linked to PC9
    TIM3->CCR4 = 0;                    // Start CCR4 at 0
    TIM3->CCMR2 &= ~(0xFF00);          // Clear CH4 info
    TIM3->CCMR2 |= 0b1101 << (3 + 8);  // CC4S = 00 (PWM)
                                       // OC4M = 110 (start PWM high)
                                       // OC4PE = 1 (enable preload)
    TIM3->CCER |= 0b0001 << (4 * 3);   // CC4NP = 0 (PWM)
                                       // CC4P = 0 (PWM)
                                       // CC4E = 1 (external output enabled)

    TIM3->SR = 0;    // Clear any lingering flags
    TIM3->EGR |= 1;  // Generate a register update
    TIM3->CR1 |= 1;  // Enable the timer

    ////////////////
    // TIM4 SETUP //
    ////////////////
    TIM4->CR1 = 0x0000;   // CKD, CMS, DIR, OPM, URS, UDIS = 0
                          // ARPE = 0 (ARR not used)
                          // CEN = 0 (will be enabled later when needed)
    TIM4->CR2 = 0x0000;   // Unused, recommended to set to 0x0000
    TIM4->PSC = 31999;    // Allow for millisecond precision
                          // IF THIS IS EVER CHANGED FOR ANY REASON, ADJUST `BUZZER_TOGGLE_TIME` TOO
    TIM4->SMCR = 0x0000;  // Unused, recommended to set to 0x0000
    TIM4->ARR = 0xFFFF;   // Unused, recommended to set to 0xFFFF

    // Channel 1
    //
    // Used in the `sleep` function
    TIM4->CCR1 = 0;           // Start counter at 0
    TIM4->DIER &= ~(1 << 1);  // CC2IE = 0 (do not generate an IRQ when reached)
    TIM4->CCMR1 &= ~0x00FF;   // CC1S = 0 (TOC)
                              // OC1M = 000 (Frozen output)
                              // OC1PE = 0 (no preload)
    TIM4->CCER &= ~0b1011;    // CC1NP = 0 (TOC)
                              // CC1P = 0 (TOC)
                              // CC3E = 0 (external output disabled)

    // Channel 2
    //
    // Used to trigger the ADC periodically
    TIM4->CCR2 = ADC_TRIGGER_TIME;  // Start counter at ADC_TRIGGER_TIME
    TIM4->DIER |= 1 << 2;           // CC2IE = 1 (generate an IRQ when reached)
    TIM4->CCMR1 &= ~0x00FF;         // CC1S = 0 (TOC)
                                    // OC1M = 000 (Frozen output)
                                    // OC1PE = 0 (no preload)
    TIM4->CCER &= ~0b1011;          // CC1NP = 0 (TOC)
                                    // CC1P = 0 (TOC)
                                    // CC3E = 0 (external output disabled)

    // Channel 3
    //
    // Setup as TOC with the output being PB8 (Buzzer)
    TIM4->CCR3 = 0;               // Start counter at 0
    TIM4->DIER |= 1 << 3;         // CC4IE = 1 (generate IRQ when CCR3 is reached)
    TIM4->CCMR2 &= ~(0x00FF);     // Clear CH3 info
    TIM4->CCMR2 |= 0b101 << 4;    // CC3S = 0 (TOC)
                                  // OC3M = 101 (force high -> turn off buzzer)
                                  // OC3PE = 0 (no preload)
    TIM4->CCER &= ~(0b101 << 9);  // CC3NP = 0 (TOC)
                                  // CC3P = 0 (TOC)
    TIM4->CCER |= (1 << 8);       // CC3E = 1 (external output enabled)

    TIM4->SR = 0;             // Clear any lingering flags
    TIM4->EGR |= 1;           // Generate a register update
    NVIC->ISER[0] = 1 << 30;  // Enable TIMER4 callback
    TIM4->CR1 |= 1;           // Enable the timer

    ///////////////
    // SPI SETUP //
    ///////////////
    // Uses PB13 as CLK, PB15 as MOSI, and PB1 as Chip Select
    display_send(&hspi2, GPIOB, 1, 0xA00 | DISPLAY_BRIGHTNESS);  // Set intensity
    display_send(&hspi2, GPIOB, 1, 0xB07);                       // Set scan limit to 7 (full)
    display_send(&hspi2, GPIOB, 1, 0xF00);                       // Disable test mode just in case
    display_draw(&hspi2, GPIOB, 1, 0);                           // Set an empty display
    display_send(&hspi2, GPIOB, 1, 0xC01);                       // Enable display (shutdown off)

    /////////////////
    // UART1 SETUP //
    /////////////////
    // Configuration is done through CubeMX, only thing left is to start reception through callbacks
    HAL_UART_Receive_IT(&huart1, (uint8_t*) &uart1_receive_byte, 1);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    set_manual_mode();

    HAL_UART_Transmit(&huart1, (uint8_t*) "Ready\n", 6, UART_TIMEOUT);

    while (true) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void) {

    /* USER CODE BEGIN ADC_Init 0 */

    /* USER CODE END ADC_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC_Init 1 */

    /* USER CODE END ADC_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
    hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
    hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.NbrOfConversion = 1;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    if (HAL_ADC_Init(&hadc) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample
     * time.
     */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC_Init 2 */

    /* USER CODE END ADC_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
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
static void MX_TIM3_Init(void) {

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
    if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {}
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
void assert_failed(uint8_t* file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
