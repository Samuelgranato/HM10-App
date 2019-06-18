#include <asf.h>
#include "conf_board.h"
#include <string.h>

/************************************************************************/
/* defines                                                              */
/************************************************************************/

// TRIGGER
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// usart (bluetooth)
#define USART_COM_ID ID_USART0
#define USART_COM    USART0


/** RTOS  */
#define TASK_PROCESS_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_PROCESS_STACK_PRIORITY        (tskIDLE_PRIORITY)

//francato
#define TASK_MM_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_MM_STACK_PRIORITY            (tskIDLE_PRIORITY)

volatile int delay_motor_passo = 3;

volatile int delay_mm = 100;

#define BUT1_PIO      PIOA
#define BUT1_PIO_ID   ID_PIOA
#define BUT1_IDX  11
#define BUT1_IDX_MASK (1 << BUT1_IDX)

#define PINO1_MOTOR      PIOC
#define PINO1_MOTOR_ID   ID_PIOC
#define PINO1_MOTOR_IDX  31
#define PINO1_MOTOR_IDX_MASK  (1 << PINO1_MOTOR_IDX)

#define PINO2_MOTOR      PIOB
#define PINO2_MOTOR_ID   ID_PIOB
#define PINO2_MOTOR_IDX  3
#define PINO2_MOTOR_IDX_MASK (1 << PINO2_MOTOR_IDX)

#define PINO3_MOTOR      PIOB
#define PINO3_MOTOR_ID   ID_PIOB
#define PINO3_MOTOR_IDX  2
#define PINO3_MOTOR_IDX_MASK (1 << PINO3_MOTOR_IDX)

#define PINO4_MOTOR      PIOC
#define PINO4_MOTOR_ID   ID_PIOC
#define PINO4_MOTOR_IDX  30
#define PINO4_MOTOR_IDX_MASK (1 << PINO4_MOTOR_IDX)

#define MOTOR_PIO           PIOD
#define MOTOR_PIO_ID        ID_PIOD
#define MOTOR_PIO_IDX       22u
#define MOTOR_PIO_IDX_MASK  (1u << MOTOR_PIO_IDX) 

#define ENC1_PIO_motor1 PIOC
#define ENC1_ID_motor1 ID_PIOC
#define ENC1_IDX_motor1 13
#define ENC1_MASK_motor1 (1 << ENC1_IDX_motor1)

#define ENC2_PIO_motor1 PIOD
#define ENC2_ID_motor1 ID_PIOD
#define ENC2_IDX_motor1 30
#define ENC2_MASK_motor1 (1 << ENC2_IDX_motor1)

//MOTOR 2

#define ENC1_PIO_motor2 PIOD
#define ENC1_ID_motor2 ID_PIOD
#define ENC1_IDX_motor2 11
#define ENC1_MASK_motor2 (1 << ENC1_IDX_motor2)

#define ENC2_PIO_motor2 PIOA
#define ENC2_ID_motor2 ID_PIOA
#define ENC2_IDX_motor2 6
#define ENC2_MASK_motor2 (1 << ENC2_IDX_motor2)

//MOTOR 3

#define ENC1_PIO_motor3 PIOD
#define ENC1_ID_motor3 ID_PIOD
#define ENC1_IDX_motor3 26
#define ENC1_MASK_motor3 (1 << ENC1_IDX_motor3)

#define ENC2_PIO_motor3 PIOC
#define ENC2_ID_motor3 ID_PIOC
#define ENC2_IDX_motor3 19
#define ENC2_MASK_motor3 (1 << ENC2_IDX_motor3)



#define PIO_PWM_0 PIOA
#define ID_PIO_PWM_0 ID_PIOA	
#define MASK_PIN_PWM_0 (1 << 0)


//####################################3
// CONFIG PWM
//###################################

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0

/** PWM channel instance for LEDs */
pwm_channel_t g_pwm_channel_led;

//##################################


#define TASK_TRIGGER_STACK_SIZE            (1024/sizeof(portSTACK_TYPE))
#define TASK_TRIGGER_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_UARTTX_STACK_SIZE             (2048/sizeof(portSTACK_TYPE))
#define TASK_UARTTX_STACK_PRIORITY         (tskIDLE_PRIORITY)
#define TASK_UARTRX_STACK_SIZE             (2048/sizeof(portSTACK_TYPE))
#define TASK_UARTRX_STACK_PRIORITY         (1)
#define TASK_PROCESS_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_PROCESS_STACK_PRIORITY        (2)


//###########################################
// VAR VINI
//##########################################

SemaphoreHandle_t StartGame;
volatile uint32_t counter_motor1 = 0;
volatile uint32_t counter2_motor1 = 0;

volatile uint32_t counter_motor2 = 0;
volatile uint32_t counter2_motor2 = 0;

volatile uint32_t counter_motor3 = 0;
volatile uint32_t counter2_motor3 = 0;

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);

volatile int final_1 = 0;
volatile int final_2 = 0;
volatile int final_3 = 0;




//############################################


SemaphoreHandle_t xSemaphore1;
//francato

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


/** prototypes */
void but_callback(void);
static void ECHO_init(void);
static void USART1_init(void);
uint32_t usart_puts(uint8_t *pstring);

void but1_callBack(){
	//counter = 0;
	static  BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
	
	//counter = 0;
	static  BaseType_t xHigherPriorityTaskWoken2;
	xHigherPriorityTaskWoken2 = pdFALSE;
	//xSemaphoreGiveFromISR(StartGame, &xHigherPriorityTaskWoken2);
}


QueueHandle_t xQueue1;
volatile uint32_t g_tcCv = 0;


/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void PWM0_init(uint channel, uint duty){
	/* Enable PWM peripheral clock */
	pmc_enable_periph_clk(ID_PWM0);

	/* Disable PWM channels for LEDs */
	pwm_channel_disable(PWM0, PIN_PWM_LED0_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_hz()
	};
	
	pwm_init(PWM0, &clock_setting);

	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_led.alignment = PWM_ALIGN_CENTER;
	/* Output waveform starts at a low level */
	g_pwm_channel_led.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_led.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_led.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	g_pwm_channel_led.ul_duty = duty;
	g_pwm_channel_led.channel = channel;
	pwm_channel_init(PWM0, &g_pwm_channel_led);
	
	/* Enable PWM channels for LEDs */
	pwm_channel_enable(PWM0, channel);
}

void counting_motor1(void){
	counter_motor1 +=1;
	
}

void counting2_motor1(void){
	counter2_motor1 +=1;
	
}

//MOTOR 2

void counting_motor2(void){
	counter_motor2 +=1;
	
}

void counting2_motor2(void){
	counter2_motor2 +=1;
	
}

//MOTOR 3

void counting_motor3(void){
	counter_motor3 +=1;
	
}

void counting2_motor3(void){
	counter2_motor3 +=1;
	
}
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC0);
	pmc_enable_periph_clk(ID_TC1);

	pio_set_peripheral(PIOA, PIO_PERIPH_B, (1u << 0));
	pio_set_peripheral(PIOA, PIO_PERIPH_B, (1u << 1));
	
	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	//tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	//tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_init(TC, 0, TC_CMR_TCCLKS_XC0 | TC_CMR_EEVT_XC0);
	tc_init(TC, 1, TC_CMR_TCCLKS_XC1 | TC_CMR_EEVT_XC1);
	
	tc_set_block_mode(TC0, TC_BMR_QDEN | TC_BMR_SPEEDEN | TC_BMR_POSEN);
	//tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	//NVIC_EnableIRQ((IRQn_Type) ID_TC);
	//tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, 0);
	tc_start(TC, 1);
}


/**
 * \brief Configure the console UART.
 */

static void configure_console(void){
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

uint32_t usart_puts(uint8_t *pstring){
	uint32_t i ;

	while(*(pstring + i))
		if(uart_is_tx_empty(USART_COM))
			usart_serial_putchar(USART_COM, *(pstring+i++));
}

void io_init(void){

  // Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(ENC1_ID_motor1);
	pmc_enable_periph_clk(ENC2_ID_motor1);
	
	pmc_enable_periph_clk(ENC1_ID_motor2);
	pmc_enable_periph_clk(ENC2_ID_motor2);
	
	pmc_enable_periph_clk(ENC1_ID_motor3);
	pmc_enable_periph_clk(ENC2_ID_motor3);
	
	
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	pio_configure(ENC1_PIO_motor1, PIO_INPUT, ENC1_MASK_motor1, PIO_DEFAULT| PIO_DEBOUNCE);
	pio_configure(ENC2_PIO_motor1, PIO_INPUT, ENC2_MASK_motor1, PIO_DEFAULT| PIO_DEBOUNCE);
	
	pio_configure(ENC1_PIO_motor2, PIO_INPUT, ENC1_MASK_motor2, PIO_DEFAULT| PIO_DEBOUNCE);
	pio_configure(ENC2_PIO_motor2, PIO_INPUT, ENC2_MASK_motor2, PIO_DEFAULT| PIO_DEBOUNCE);
	
	pio_configure(ENC1_PIO_motor3, PIO_INPUT, ENC1_MASK_motor3, PIO_DEFAULT| PIO_DEBOUNCE);
	pio_configure(ENC2_PIO_motor3, PIO_INPUT, ENC2_MASK_motor3, PIO_DEFAULT| PIO_DEBOUNCE);
	
	
	
	
	pio_set_output(MOTOR_PIO, MOTOR_PIO_IDX_MASK, 0, 0, 0);
	/* led */
	//pmc_enable_periph_clk(LED_PIO_ID);
	//pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pio_set_input(PIOA, (1u << 0), PIO_DEBOUNCE);
	pio_set_debounce_filter(PIOA, (1u << 0), 300);
	
	
	pio_set_input(PIOA, (1u << 1), PIO_DEBOUNCE);
	pio_set_debounce_filter(PIOA, (1u << 1), 300);
	
	
	
	//pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_set_output(PINO1_MOTOR,PINO1_MOTOR_IDX_MASK,0,0,0);
	pio_set_output(PINO2_MOTOR,PINO2_MOTOR_IDX_MASK,0,0,0);
	pio_set_output(PINO3_MOTOR,PINO3_MOTOR_IDX_MASK,0,0,0);
	pio_set_output(PINO4_MOTOR,PINO4_MOTOR_IDX_MASK,0,0,0);
	
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but1_callBack);
	
	//MOTOR1
	
	pio_handler_set(ENC1_PIO_motor1,
	ENC1_ID_motor1,
	ENC1_MASK_motor1,
	PIO_IT_RISE_EDGE,
	counting_motor1);
	
	pio_handler_set(ENC2_PIO_motor1,
	ENC2_ID_motor1,
	ENC2_MASK_motor1,
	PIO_IT_RISE_EDGE,
	counting2_motor1);
	
	//MOTOR2
	
	pio_handler_set(ENC1_PIO_motor2,
	ENC1_ID_motor2,
	ENC1_MASK_motor2,
	PIO_IT_RISE_EDGE,
	counting_motor2);
	
	pio_handler_set(ENC2_PIO_motor2,
	ENC2_ID_motor2,
	ENC2_MASK_motor2,
	PIO_IT_RISE_EDGE,
	counting2_motor2);
	
	//MOTOR 3
	
	pio_handler_set(ENC1_PIO_motor3,
	ENC1_ID_motor3,
	ENC1_MASK_motor3,
	PIO_IT_RISE_EDGE,
	counting_motor3);
	
	pio_handler_set(ENC2_PIO_motor3,
	ENC2_ID_motor3,
	ENC2_MASK_motor3,
	PIO_IT_RISE_EDGE,
	counting2_motor3);
	
	pio_set_debounce_filter(ENC1_PIO_motor1, ENC1_MASK_motor1, 300);
	pio_set_debounce_filter(ENC2_PIO_motor1, ENC2_MASK_motor1, 300);
	
	pio_set_debounce_filter(ENC1_PIO_motor2, ENC1_MASK_motor2, 300);
	pio_set_debounce_filter(ENC2_PIO_motor2, ENC2_MASK_motor2, 300);
	
	pio_set_debounce_filter(ENC1_PIO_motor3, ENC1_MASK_motor3, 300);
	pio_set_debounce_filter(ENC2_PIO_motor3, ENC2_MASK_motor3, 300);
	
	
	
	pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
	
	pio_enable_interrupt(ENC1_PIO_motor1, ENC1_MASK_motor1);
	pio_enable_interrupt(ENC2_PIO_motor1, ENC2_MASK_motor1);
	
	pio_enable_interrupt(ENC1_PIO_motor2, ENC1_MASK_motor2);
	pio_enable_interrupt(ENC2_PIO_motor2, ENC2_MASK_motor2);
	
	pio_enable_interrupt(ENC1_PIO_motor3, ENC1_MASK_motor3);
	pio_enable_interrupt(ENC2_PIO_motor3, ENC2_MASK_motor3);
	
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 5);
	
		
	NVIC_EnableIRQ(ENC1_ID_motor1);
	NVIC_SetPriority(ENC1_ID_motor1, 5);
	NVIC_EnableIRQ(ENC2_ID_motor1);
	NVIC_SetPriority(ENC2_ID_motor1, 5);
	
	NVIC_EnableIRQ(ENC1_ID_motor2);
	NVIC_SetPriority(ENC1_ID_motor2, 5);
	NVIC_EnableIRQ(ENC2_ID_motor2);
	NVIC_SetPriority(ENC2_ID_motor2, 5);
	
	NVIC_EnableIRQ(ENC1_ID_motor3);
	NVIC_SetPriority(ENC1_ID_motor3, 5);
	NVIC_EnableIRQ(ENC2_ID_motor3);
	NVIC_SetPriority(ENC2_ID_motor3, 5);
}

void fechando_mm(){
	pio_set(PINO4_MOTOR, PINO4_MOTOR_IDX_MASK);
	vTaskDelay(delay_motor_passo);
	pio_clear(PINO4_MOTOR, PINO4_MOTOR_IDX_MASK);
	pio_set(PINO3_MOTOR, PINO3_MOTOR_IDX_MASK);
	vTaskDelay(delay_motor_passo);
	pio_clear(PINO3_MOTOR, PINO3_MOTOR_IDX_MASK);
	pio_set(PINO2_MOTOR, PINO2_MOTOR_IDX_MASK);
	vTaskDelay(delay_motor_passo);
	pio_clear(PINO2_MOTOR, PINO2_MOTOR_IDX_MASK);
	pio_set(PINO1_MOTOR, PINO1_MOTOR_IDX_MASK);
	vTaskDelay(delay_motor_passo);
	pio_clear(PINO1_MOTOR, PINO1_MOTOR_IDX_MASK);
}

/////////////////////
void abrindo_mm(){
	pio_set(PINO1_MOTOR, PINO1_MOTOR_IDX_MASK);
	vTaskDelay(delay_motor_passo);
	pio_clear(PINO1_MOTOR, PINO1_MOTOR_IDX_MASK);
	pio_set(PINO2_MOTOR, PINO2_MOTOR_IDX_MASK);
	vTaskDelay(delay_motor_passo);
	pio_clear(PINO2_MOTOR, PINO2_MOTOR_IDX_MASK);
	pio_set(PINO3_MOTOR, PINO3_MOTOR_IDX_MASK);
	vTaskDelay(delay_motor_passo);
	pio_clear(PINO3_MOTOR, PINO3_MOTOR_IDX_MASK);
	pio_set(PINO4_MOTOR, PINO4_MOTOR_IDX_MASK);
	vTaskDelay(delay_motor_passo);
	pio_clear(PINO4_MOTOR, PINO4_MOTOR_IDX_MASK);
}


void usart_put_string(Usart *usart, char str[]) {
  usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
  uint timecounter = timeout_ms;
  uint32_t rx;
  uint32_t counter = 0;
  
  while( (timecounter > 0) && (counter < bufferlen - 1)) {
    if(usart_read(usart, &rx) == 0) {
      buffer[counter++] = rx;
    }
    else{
      timecounter--;
      vTaskDelay(1);
    }    
  }
  buffer[counter] = 0x00;
  return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
  usart_put_string(usart, buffer_tx);
  usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void hc05_config_server(void) {
  sysclk_enable_peripheral_clock(USART_COM_ID);
  usart_serial_options_t config;
  config.baudrate = 9600;
  config.charlength = US_MR_CHRL_8_BIT;
  config.paritytype = US_MR_PAR_NO;
  config.stopbits = false;
  usart_serial_init(USART_COM, &config);
  usart_enable_tx(USART_COM);
  usart_enable_rx(USART_COM);
  
  // RX - PB0  TX - PB1
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
  pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

int hc05_server_init(void) {
  char buffer_rx[128];
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100); printf("AT\n");
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100);
  usart_send_command(USART0, buffer_rx, 1000, "AT+NAMECACANIQUELZIKA", 100);
  usart_send_command(USART0, buffer_rx, 1000, "AT", 100);
  usart_send_command(USART0, buffer_rx, 1000, "AT+PIN0000", 100);
}

void pin_toggle(Pio *pio, uint32_t mask){
	
	if(pio_get_output_data_status(pio, mask)){
		pio_clear(pio, mask);
		}else{
		pio_set(pio,mask);
	}
}

void ficha_comprada(){
	pin_toggle(LED_PIO, LED_IDX_MASK);
	delay_ms(200);
	pin_toggle(LED_PIO, LED_IDX_MASK);
	
}

void jogada_autorizada(){
	int i = 15;
	while(i>0){
		pin_toggle(LED_PIO, LED_IDX_MASK);
		i--;
		delay_ms(40);
	}
	static  BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE; 
	xSemaphoreGiveFromISR(StartGame, &xHigherPriorityTaskWoken);
}

static int count_substr(const char *str, const char* substr, bool overlap) {
	if (strlen(substr) == 0) return -1; // forbid empty substr

	int count = 0;
	int increment = overlap ? 1 : strlen(substr);
	for (char* s = (char*)str; (s = strstr(s, substr)); s += increment)
	++count;
	return count;
}

void check_quadrante(){
	printf("MOTOR 1 ENC1 :%u\n",counter_motor1);
	printf("MOTOR 1 ENC2 :%u\n",counter2_motor1);
	
	
	counter_motor1 = (counter_motor1) % 8;
	//vTaskDelay(3000);
	if(counter_motor1 >=0 && counter_motor1 < 2){
		final_1 = 1;
		printf("Primeiro quadrante\n");
		
		} else if(counter_motor1 >=2 && counter_motor1 < 4){
			final_1 = 2;
		
			printf("Segundo quadrante\n");
		
		} else if(counter_motor1 >=4 && counter_motor1 < 6){
			final_1 = 3;
		
			printf("Terceiro quadrante\n");
		
		}else{
			final_1 = 4;
		
			printf("Quarto quadrante\n");
		}
	
	counter2_motor1 = counter2_motor1 % 12;
	//vTaskDelay(3000);
	if(counter2_motor1 >=0 && counter2_motor1 < 3){
		printf("Primeiro quadrante\n");
		
		} else if(counter2_motor1 >=3 && counter2_motor1 < 6){
		printf("Segundo quadrante\n");
		
		} else if(counter2_motor1 >=6 && counter2_motor1 < 9){
		printf("Terceiro quadrante\n");
		
		}else{
		printf("Quarto quadrante\n");
	}
	//MOTOR 2
	printf("MOTOR 2 ENC1 :%u\n",counter_motor2);
	printf("MOTOR 2 ENC2 :%u\n",counter2_motor2);
	counter_motor2 = counter_motor2 % 12;
	if(counter_motor2 >=0 && counter_motor2 < 2){
		final_2 = 1;
		
		printf("Primeiro quadrante\n");
		
		} else if(counter_motor2 >=2 && counter_motor2 < 4){
			final_2 = 2;
		
			printf("Segundo quadrante\n");
		
		} else if(counter_motor2 >=4 && counter_motor2 < 6){
			final_2 = 3;
		
			printf("Terceiro quadrante\n");
		
		}else{
			final_2 = 4;
		
			printf("Quarto quadrante\n");
	}
	
	counter2_motor2 = counter2_motor2 % 15;
	//vTaskDelay(3000);
	if(counter2_motor2 >=0 && counter2_motor2 < 3){
		
		printf("Primeiro quadrante\n");
		
		} else if(counter2_motor2 >=3 && counter2_motor2 < 6){
		printf("Segundo quadrante\n");
		
		} else if(counter2_motor2 >=6 && counter2_motor2 < 9){
		printf("Terceiro quadrante\n");
		
		}else{
		printf("Quarto quadrante\n");
	}
	
	//MOTOR 3
	printf("MOTOR 3 ENC1 :%u\n",counter_motor3);
	printf("MOTOR 3 ENC2 :%u\n",counter2_motor3);
	
	counter_motor3 = counter_motor3 % 12;
	if(counter_motor2 >=0 && counter_motor2 < 3){
		final_3 = 1;
		
		printf("Primeiro quadrante\n");
		
		} else if(counter_motor2 >=2 && counter_motor2 < 6){
			final_3 = 2;
		
			printf("Segundo quadrante\n");
		
		} else if(counter_motor2 >=4 && counter_motor2 < 9){
			final_3 = 3;
		
			printf("Terceiro quadrante\n");
		
		}else{
			final_3 = 4;
		
			printf("Quarto quadrante\n");
	}
	
	counter2_motor3 = counter2_motor3 % 12;
	//vTaskDelay(3000);
	if(counter2_motor3 >=0 && counter2_motor3 < 3){
		printf("Primeiro quadrante\n");
		
		} else if(counter2_motor3 >=3 && counter2_motor3 < 6){
		printf("Segundo quadrante\n");
		
		} else if(counter2_motor3 >=6 && counter2_motor3 < 9){
		printf("Terceiro quadrante\n");
		
		}else{
		printf("Quarto quadrante\n");
	}
	printf("-------------------------------------------\n");
	}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

void task_bluetooth(void){
  
  printf("Bluetooth initializing \n");
  hc05_config_server();
  hc05_server_init();
  //io_init();
  
	char buffer[1024];
	
  
  while(1){
   // printf("running task \n");
	
	//usart_put_string(USART0, "wait\n");
	usart_get_string(USART0, buffer, 1024,1000);
	
	//solicitacao de compra = 1
	//solicitacao de jogada= 2
	//sucesso na compra = 3
	//sucesso na jogada = 4
	
	for(int i = 0;i < count_substr(buffer,"1",0);i++){
		usart_put_string(USART0, "3\n");
		ficha_comprada();
	}
	
	for(int i = 0;i < count_substr(buffer,"2",0);i++){
		usart_put_string(USART0, "4\n");
		jogada_autorizada();
	}
	
	//usart_log("main", buffer);
	
    vTaskDelay( 100);
  }
}

static void task_mm(void *pvParameters)
{
	
	xSemaphore1 = xSemaphoreCreateBinary();
	while(1) {
		if( xSemaphoreTake(xSemaphore1, ( TickType_t ) 50) == pdTRUE ){
			for (int i = 0; i<50; i++){
				abrindo_mm();
			}
			vTaskDelay(delay_mm);
			for (int j = 0; j<50; j++){
				fechando_mm();
			}
			
		}
	}
}

int checkMatches(int final_1, int final_2,int final_3){
	int numLen = 3;
	int finals[5];
	int matches = 0;
	
	finals[0] = final_1;
	finals[1] = final_2;
	finals[2] = final_3;
	finals[3] = final_1;
	finals[4] = final_2;
	
	
	for(int i = 0; i < numLen; i++){
		if(finals[i] == finals[i+1] && finals[i] == finals[i+2]){
			matches = 3;
		}else{
			if(finals[i] == finals[i+1] || finals[i] == finals[i+2]){
				if(matches < 2) matches = 2;
			}
		}
	}
	return matches;
	
}

void winning(){
	int matches;
	matches = checkMatches(final_1,final_2,final_3);

	printf("MATCHESSSS %d\n", matches);
	if(matches == 2){
		static  BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
	} else if (matches > 2){
		static  BaseType_t xHigherPriorityTaskWoken;
		xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
		vTaskDelay(delay_mm*2);
		static  BaseType_t xHigherPriorityTaskWoken2;
		xHigherPriorityTaskWoken2 = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken2);
	}
	
	//if(final_1 == 1){
		//if(final_2 == 1){
			//static  BaseType_t xHigherPriorityTaskWoken;
			//xHigherPriorityTaskWoken = pdFALSE;
			//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//vTaskDelay(delay_mm*2);
			//if(final_3 == 1){
				//static  BaseType_t xHigherPriorityTaskWoken;
				//xHigherPriorityTaskWoken = pdFALSE;
				//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//}
		//}
		//if(final_2 == 2){
			//static  BaseType_t xHigherPriorityTaskWoken;
			//xHigherPriorityTaskWoken = pdFALSE;
			//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//vTaskDelay(delay_mm*2);
			//
			//if(final_3 == 2){
				//static  BaseType_t xHigherPriorityTaskWoken;
				//xHigherPriorityTaskWoken = pdFALSE;
				//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//}
		//}
		//if(final_2 == 3){
			//static  BaseType_t xHigherPriorityTaskWoken;
			//xHigherPriorityTaskWoken = pdFALSE;
			//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//vTaskDelay(delay_mm*2);
			//
			//if(final_3 == 3){
				//static  BaseType_t xHigherPriorityTaskWoken;
				//xHigherPriorityTaskWoken = pdFALSE;
				//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//}
		//}
		//
		//} if(final_1 == 2){
		//if(final_2 == 2){
			//static  BaseType_t xHigherPriorityTaskWoken;
			//xHigherPriorityTaskWoken = pdFALSE;
			//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//vTaskDelay(delay_mm*2);
			//
			//if(final_3 == 2){
				//static  BaseType_t xHigherPriorityTaskWoken;
				//xHigherPriorityTaskWoken = pdFALSE;
				//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//}
		//}
		//} if(final_1 == 3){
		//if(final_2 == 3){
			//static  BaseType_t xHigherPriorityTaskWoken;
			//xHigherPriorityTaskWoken = pdFALSE;
			//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//vTaskDelay(delay_mm*2);
			//
			//if(final_3 == 3){
				//static  BaseType_t xHigherPriorityTaskWoken;
				//xHigherPriorityTaskWoken = pdFALSE;
				//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//}
		//}
		//} if(final_1 == 4) {
		//if(final_2 == 4){
			//static  BaseType_t xHigherPriorityTaskWoken;
			//xHigherPriorityTaskWoken = pdFALSE;
			//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//vTaskDelay(delay_mm*2);
			//
			//if(final_3 == 4){
				//static  BaseType_t xHigherPriorityTaskWoken;
				//xHigherPriorityTaskWoken = pdFALSE;
				//xSemaphoreGiveFromISR(xSemaphore1, &xHigherPriorityTaskWoken);
			//}
		//}
	//}
}

void tsk_losing(){
	
	StartGame = xSemaphoreCreateBinary();
	io_init();
	
	pmc_enable_periph_clk(ID_PIO_PWM_0);
	pio_set_peripheral(PIO_PWM_0, PIO_PERIPH_A, MASK_PIN_PWM_0 );
	pio_clear(MOTOR_PIO,MOTOR_PIO_IDX_MASK);
	pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 0);
	uint duty = 0;
	PWM0_init(0, duty);
	
	counter_motor1 = 0 ;
	counter2_motor1 = 0;
	counter_motor2 = 0;
	counter2_motor2 = 0;
	counter_motor3 = 0;
	counter2_motor3 = 0;
	
	
// 	while(1){
// 		
// 		printf("%d\n", counter);
// 		vTaskDelay(100);
// 		
// 	}
	
	/* Infinite loop */
	/* fade in */
	for (;;){
		//xSemaphoreTake(xSemaphoreBut, (TickType_t) 50) == pdTRUE
		
		if(xSemaphoreTake(StartGame, (TickType_t) 50)){
			pio_set(MOTOR_PIO,MOTOR_PIO_IDX_MASK);
			pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 0);
			vTaskDelay(2000);
				
				
			/*for(duty = 0; duty <= 100; duty+=10){
				pio_clear(MOTOR_PIO,MOTOR_PIO_IDX_MASK);
				pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 100);
				vTaskDelay(100);
				
			}*/
				pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 0);
				pio_clear(MOTOR_PIO,MOTOR_PIO_IDX_MASK);
				vTaskDelay(3000);
				check_quadrante();
				//printf("enc1 : %d \n", counter);
				//printf("enc2 : %d \n", counter2);
				
				//pio_set(MOTOR_PIO,MOTOR_PIO_IDX_MASK);
				//pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 0);
				xSemaphoreTake(StartGame, (TickType_t) 200);
				//vTaskDelay(100);
				printf("M1 : %d, M2 : %d , M3: %d \n------------------------------------\n", final_1, final_2,final_3);
				winning();
				
			
			}else {
				pio_clear(MOTOR_PIO,MOTOR_PIO_IDX_MASK);
				pwm_channel_update_duty(PWM0, &g_pwm_channel_led, 0);
				counter_motor1 = 0 ;
				counter2_motor1 = 0;
				counter_motor2 = 0;
				counter2_motor2 = 0;
				counter_motor3 = 0;
				counter2_motor3 = 0;
				
			
		}
		//xSemaphoreTake(StartGame, (TickType_t) 200);
		
			
	}

		
		//vTaskDelay(100);
		
	}




 
/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void){
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	/* Initialize the console uart */
	configure_console();
	//TC_init(TC0, ID_TC0, 1, 4);
	
	
	if (xTaskCreate(task_mm, "MM", TASK_MM_STACK_SIZE, NULL,TASK_MM_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	///* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_PROCESS_STACK_SIZE, NULL,	TASK_PROCESS_STACK_PRIORITY, NULL);
	
	
	xTaskCreate(tsk_losing, "roleta", TASK_UARTTX_STACK_SIZE, NULL,TASK_UARTTX_STACK_PRIORITY, NULL);
  
	/* Start the scheduler. */
	
	vTaskStartScheduler();
	

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
