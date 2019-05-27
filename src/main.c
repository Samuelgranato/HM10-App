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

volatile int delay_mm = 1000;

#define BUT1_PIO      PIOA
#define BUT1_PIO_ID   ID_PIOA
#define BUT1_IDX  11
#define BUT1_IDX_MASK (1 << BUT1_IDX)

#define PINO1_MOTOR      PIOA
#define PINO1_MOTOR_ID   ID_PIOA
#define PINO1_MOTOR_IDX  19
#define PINO1_MOTOR_IDX_MASK  (1 << PINO1_MOTOR_IDX)

#define PINO2_MOTOR      PIOB
#define PINO2_MOTOR_ID   ID_PIOB
#define PINO2_MOTOR_IDX  2
#define PINO2_MOTOR_IDX_MASK (1 << PINO2_MOTOR_IDX)

#define PINO3_MOTOR      PIOC
#define PINO3_MOTOR_ID   ID_PIOC
#define PINO3_MOTOR_IDX  30
#define PINO3_MOTOR_IDX_MASK (1 << PINO3_MOTOR_IDX)

#define PINO4_MOTOR      PIOC
#define PINO4_MOTOR_ID   ID_PIOC
#define PINO4_MOTOR_IDX  17
#define PINO4_MOTOR_IDX_MASK (1 << PINO4_MOTOR_IDX)


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
	xSemaphoreGiveFromISR(xSemaphore1, NULL);
	
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
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	
	/* led */
	//pmc_enable_periph_clk(LED_PIO_ID);
	//pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
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
	
	pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 5);
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
}

static int count_substr(const char *str, const char* substr, bool overlap) {
	if (strlen(substr) == 0) return -1; // forbid empty substr

	int count = 0;
	int increment = overlap ? 1 : strlen(substr);
	for (char* s = (char*)str; (s = strstr(s, substr)); s += increment)
	++count;
	return count;
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
    printf("running task \n");
	
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
	
	usart_log("main", buffer);
	
    vTaskDelay( 1);
  }
}

static void task_mm(void *pvParameters)
{
	
	xSemaphore1 = xSemaphoreCreateBinary();
	io_init();
	while(1) {
		if( xSemaphoreTake(xSemaphore1, ( TickType_t ) 50) == pdTRUE ){
			for (int i = 0; i<100; i++){
				abrindo_mm();
			}
			vTaskDelay(delay_mm);
			for (int j = 0; j<100; j++){
				fechando_mm();
			}
			
		}
	}
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
	
	if (xTaskCreate(task_mm, "MM", TASK_MM_STACK_SIZE, NULL,TASK_MM_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}

	/* Create task to make led blink */
	xTaskCreate(task_bluetooth, "BLT", TASK_PROCESS_STACK_SIZE, NULL,	TASK_PROCESS_STACK_PRIORITY, NULL);
	

  
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
