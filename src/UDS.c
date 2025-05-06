#include "sdk_project_config.h"
#include <interrupt_manager.h>
#include <stdint.h>
#include "DID.h"
#include <stdbool.h>

/******************************************************************************
 * Definitions
 ******************************************************************************/

/* This example is setup to work by default with EVB. To use it with other boards
   please comment the following line
*/

#define EVB

#ifdef EVB
#define LED_PORT PORTD
#define GPIO_PORT PTD
#define PCC_INDEX PCC_PORTD_INDEX
#define LED0 15U
#define LED1 16U
#define BTN_GPIO PTC
#define BTN1_PIN 13U
#define BTN2_PIN 12U
#define BTN_PORT PORTC
#define BTN_PORT_IRQn PORTC_IRQn
#else
#define LED_PORT PORTC
#define GPIO_PORT PTC
#define PCC_INDEX PCC_PORTC_INDEX
#define LED0 0U
#define LED1 1U
#define BTN_GPIO PTC
#define BTN1_PIN 13U
#define BTN2_PIN 12U
#define BTN_PORT PORTC
#define BTN_PORT_IRQn PORTC_IRQn
#endif

/* Use this define to specify if the application runs as master or slave */
#define MASTER
/* #define SLAVE */

/* Definition of the TX and RX message buffers depending on the bus role */
#if defined(MASTER)
#define TX_MAILBOX (769UL)
#define TX_MSG_ID (769UL)
#define RX_MAILBOX (768UL)
#define RX_MSG_ID (768UL)
#elif defined(SLAVE)
#define TX_MAILBOX (768UL)
#define TX_MSG_ID (768UL)
#define RX_MAILBOX (769UL)
#define RX_MSG_ID (769UL)
#endif

typedef enum
{
    LED0_CHANGE_REQUESTED = 0x00U,
    LED1_CHANGE_REQUESTED = 0x01U
} can_commands_list;

uint8_t ledRequested = (uint8_t)LED0_CHANGE_REQUESTED;

extern support_DID_table[];

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void buttonISR(void);
void BoardInit(void);
void GPIOInit(void);

// Support Functions
void SendNRC(uint8_t SID, uint8_t NRC);
bool checkCondition(uint16_t DID);
bool isSupportedDID(uint16_t DID);

extern readECULifetime(uint8_t *buffer)

    /*Service Handler Functions*/
    void SID_22_Handler(can_message_t *message);

/******************************************************************************
 * Functions
 ******************************************************************************/
void SendNRC(uint8_t SID, uint8_t NRC)
{
    can_buff_config_t buffCfg = {
        .enableFD = false,
        .enableBRS = false,
        .fdPadding = 0U,
        .idType = CAN_MSG_ID_STD,
        .isRemote = false};

    /* Configure TX buffer with index TX_MAILBOX*/
    CAN_ConfigTxBuff(&can_pal1_instance, TX_MAILBOX, &buffCfg);

    can_message_t message = {
        .cs = 0U,
        .id = TX_MSG_ID,
        .data[0] = SID,
        .data[1] = NRC,
        .length = 2U};

    CAN_Send(&can_pal1_instance, TX_MAILBOX, &message);
}

bool checkCondition(uint16_t DID)
{
    // Apply Function Logic here
    return false;
}

bool isSupportedDID(uint16_t DID)
{
}

void SID_22_handler(const can_message_t *rcvMsg)
{
    if (rcvMsg->length < 3 || (rcvMsg->length - 1) % 2 != 0)
    {
        SendNRC(rcvMsg->data[0], 0x13);
        return;
    }

    if (rcvMsg->length > 8)
    {
        SendNRC(rcvMsg->data[0], 0x13);
        return;
    }

    volatile bool hasValidDID = false;
    uint8_t num_DID_support = sizeof(support_DID_table) / size_of(did_entry_t);

    for (uint8_t i = 0; i < num_DID_support; ++i)
    {
        if (support_DID_table[i].acess_permission == 0 || support_DID_table[i].acess_permission == 2)
        {

            if (checkCondition)
            {
                hasValidDID = true;
                continue;
            }
            else
            {
                SendNRC(rcvMsg->data[0], 0x22);
                return;
            }
        }
    }

    if(hasValidDID == false)
    {
        SendNRC(rcvMsg->data[0], 0x31);
        return;
    }
    //SendPositiveResponse();
}

/**
 * Button interrupt handler
 */
void buttonISR(void)
{
    /* Check if one of the buttons was pressed */
    uint32_t buttonsPressed = PINS_DRV_GetPortIntFlag(BTN_PORT) &
                              ((1 << BTN1_PIN) | (1 << BTN2_PIN));
    bool sendFrame = false;

    if (buttonsPressed != 0)
    {

        /* Set FlexCAN TX value according to the button pressed */
        switch (buttonsPressed)
        {
        case (1 << BTN1_PIN):
            ledRequested = LED0_CHANGE_REQUESTED;
            sendFrame = true;
            /* Clear interrupt flag */
            PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, BTN1_PIN);
            break;
        case (1 << BTN2_PIN):
            ledRequested = LED1_CHANGE_REQUESTED;
            sendFrame = true;
            /* Clear interrupt flag */
            PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, BTN2_PIN);
            break;
        default:
            PINS_DRV_ClearPortIntFlagCmd(BTN_PORT);
            break;
        }

        if (sendFrame)
        {
            /* Set information about the data to be sent
             *  - Standard message ID
             *  - Bit rate switch enabled to use a different bitrate for the data segment
             *  - Flexible data rate enabled
             *  - Use zeros for FD padding
             */
            can_buff_config_t buffCfg = {
                .enableFD = false,
                .enableBRS = false,
                .fdPadding = 0U,
                .idType = CAN_MSG_ID_STD,
                .isRemote = false};

            /* Configure TX buffer with index TX_MAILBOX*/
            CAN_ConfigTxBuff(&can_pal1_instance, TX_MAILBOX, &buffCfg);

            /* Prepare message to be sent */
            can_message_t message = {
                .cs = 0U,
                .id = TX_MSG_ID,
                .data[0] = ledRequested,
                .length = 1U};

            /* Send the information via CAN */
            CAN_Send(&can_pal1_instance, TX_MAILBOX, &message);
        }
    }
}

/*
 * @brief : Initialize clocks, pins and power modes
 */
void BoardInit(void)
{

    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   Configure FlexCAN clock, GPIO
     *  -   see clock manager component for more details
     */
    CLOCK_DRV_Init(&clockMan1_InitConfig0);

    /* Initialize pins
     *  -   Init FlexCAN and GPIO pins
     *  -   See PinSettings component for more info
     */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0, g_pin_mux_InitConfigArr0);
}

/*
 * @brief Function which configures the LEDs and Buttons
 */
void GPIOInit(void)
{
    /* Output direction for LEDs */
    PINS_DRV_SetPinsDirection(GPIO_PORT, (1 << LED1) | (1 << LED0));

    /* Set Output value LEDs */
    PINS_DRV_ClearPins(GPIO_PORT, (1 << LED1) | (1 << LED0));

    /* Setup button pin */
    PINS_DRV_SetPinsDirection(BTN_GPIO, ~((1 << BTN1_PIN) | (1 << BTN2_PIN)));

    /* Setup button pins interrupt */
    PINS_DRV_SetPinIntSel(BTN_PORT, BTN1_PIN, PORT_INT_RISING_EDGE);
    PINS_DRV_SetPinIntSel(BTN_PORT, BTN2_PIN, PORT_INT_RISING_EDGE);

    /* Install buttons ISR */
    INT_SYS_InstallHandler(BTN_PORT_IRQn, &buttonISR, NULL);

    /* Enable buttons interrupt */
    INT_SYS_EnableIRQ(BTN_PORT_IRQn);
}

volatile int exit_code = 0;

/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - __start (startup asm routine)
 * - __init_hardware()
 * - main()
 *     - Common_Init()
 *     - Peripherals_Init()
*/

int main(void)
{
    /* Do the initializations required for this application */
    BoardInit();
    GPIOInit();

    CAN_Init(&can_pal1_instance, &can_pal1_Config0);

    /* Set information about the data to be sent
     *  - Standard message ID
     *  - Bit rate switch enabled to use a different bitrate for the data segment
     *  - Flexible data rate enabled
     *  - Use zeros for FD padding
     */
    can_buff_config_t buffCfg = {
        .enableFD = false,
        .enableBRS = false,
        .fdPadding = 0U,
        .idType = CAN_MSG_ID_STD,
        .isRemote = false};

    /* Configure RX buffer with index RX_MAILBOX */
    CAN_ConfigRxBuff(&can_pal1_instance, RX_MAILBOX, &buffCfg, RX_MSG_ID);

    while (1)
    {
        /* Define receive buffer */
        can_message_t recvMsg;

        /* Start receiving data in RX_MAILBOX. */
        CAN_Receive(&can_pal1_instance, RX_MAILBOX, &recvMsg);

        /* Wait until the previous FlexCAN receive is completed */
        while (CAN_GetTransferStatus(&can_pal1_instance, RX_MAILBOX) == STATUS_BUSY)
            ;

        /* Check the received message ID and payload */
        if ((recvMsg.data[0] == LED0_CHANGE_REQUESTED) &&
            recvMsg.id == RX_MSG_ID)
        {
            /* Toggle output value LED1 */
            PINS_DRV_TogglePins(GPIO_PORT, (1 << LED0));
        }
        else if ((recvMsg.data[0] == LED1_CHANGE_REQUESTED) &&
                 recvMsg.id == RX_MSG_ID)
        {
            /* Toggle output value LED0 */
            PINS_DRV_TogglePins(GPIO_PORT, (1 << LED1));
        }
    }

    for (;;)
    {
        if (exit_code != 0)
        {
            break;
        }
    }
    return exit_code;
}
