/* ========================================
 *
 * Copyright Hamish Johnson 2026
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF SFU Physics.
 *
 * ========================================
*/
#include "project.h"

// Include FreeflyAPI
#include "QX_Protocol.h"
#include "QX_Protocol_App.h"

// Handle writing the command for the API
int ff_api_encode_control(uint8_t *out_buf, uint16_t *out_len)
{
    QX_TxMsgOptions_t options;
    QX_InitTxOptions(&options);
    options.Target_Addr = QX_DEV_ID_GIMBAL;
    options.TransReq_Addr = QX_DEV_ID_BROADCAST;
    options.RespReq_Addr = QX_DEV_ID_BROADCAST;

    // Build the message
    QX_Msg_t TxMsg;
    QX_InitMsg(&TxMsg);

    TxMsg.CommPort = QX_COMMS_PORT_UART;
    TxMsg.Header.Attrib = 277;
    TxMsg.Header.Type = QX_MSG_TYPE_WRITE_ABS;
    TxMsg.Header.Target_Addr = options.Target_Addr;
    TxMsg.Header.TransReq_Addr = options.TransReq_Addr;
    TxMsg.Header.RespReq_Addr = options.RespReq_Addr;
    TxMsg.Header.Source_Addr = QX_Clients[0].Address;
    TxMsg.Header.FF_Ext = options.FF_Ext;
    TxMsg.Header.AddCRC32 = options.use_CRC32;

    QX_TxMsg_Setup(&TxMsg);
    TxMsg.Parse_Type = QX_PARSE_TYPE_WRITE_ABS_SEND;
    TxMsg.MsgBuf_p = QX_Clients[0].Parser_CB(&TxMsg);
    QX_TxMsg_Finish(&TxMsg);

    // Copy the encoded message to out_buf
    for (uint16_t i = 0; i < TxMsg.MsgBuf_MsgLen; i++)
        out_buf[i] = TxMsg.MsgBufStart_p[i];

    *out_len = TxMsg.MsgBuf_MsgLen;
    return 0; // success
}

CY_ISR(UART_MONITOR_RX_ISR)
{
    uint8_t rxByte;
    // Process all bytes in the RX buffer
    while (UART_MONITOR_GetRxBufferSize() > 0)
    {
        (void)UART_MONITOR_GetByte(); // discard
        // rxByte = UART_MONITOR_GetByte(); // Get the next received byte
        // QX_StreamRxCharSM(QX_COMMS_PORT_UART, rxByte); // Pass to QX Protocol
    }
}
volatile uint32_t movi_rx_byte_count = 0;

CY_ISR(UART_MOVI_RX_ISR)
{
    uint8_t rxByte;
    // Process all bytes in the RX buffer
    while (UART_MOVI_GetRxBufferSize() > 0)
    {
        movi_rx_byte_count++;
        rxByte = UART_MOVI_GetByte(); // Get the next received byte
        QX_StreamRxCharSM(QX_COMMS_PORT_UART, rxByte); // Pass to QX Protocol
        UART_MONITOR_PutChar(rxByte);
    }
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Monitor over COM Initialization */
    UART_MONITOR_Start();           
    isr_rx_StartEx(UART_MONITOR_RX_ISR);             // Start UART for monitor
    /* Movi Control over COM Initialization */
    UART_MOVI_Start();     
    isr_rx_movi_StartEx(UART_MOVI_RX_ISR);                      // Start UART on hardware RxTx
    QX_InitCli(&QX_Clients[0],                   // Chose client instance, only one.
                QX_DEV_ID_MOVI_API_CONTROLLER,   // Movi API Controller Address.
                QX_ParsePacket_Cli_MoVI_Ctrl_CB  // Callback function. This parses the returned bits (Default).
    );
    UART_MONITOR_PutString("% -- Initialized! -- %\r\n");
    FreeflyAPI.begin();

    for(;;)
    {
        CyDelay(500); // Two updates per second
        
        uint8_t button_state = SW1_debounced_Read();
        char msg[20];
        sprintf(msg, "SW1: %d\r\n", button_state); // Converts 0/1 to ASCII
        UART_MONITOR_PutString(msg); // Prints "SW1: 0" or "SW1: 1"
        
        // Example: Set control values
        FreeflyAPI.control.pan.type = DEFER;
        FreeflyAPI.control.tilt.type = RATE;
		FreeflyAPI.control.roll.type = DEFER;
		FreeflyAPI.control.focus.type = DEFER;
		FreeflyAPI.control.iris.type = DEFER;
		FreeflyAPI.control.zoom.type = DEFER;
        FreeflyAPI.control.tilt.value = 0.0f;
        
        // Encode the control message
        uint8_t uart_buf[128];
        uint16_t uart_len;
        ff_api_encode_control(uart_buf, &uart_len);
        // Do nothing with packet for now
        
        // 4) Optional: every N loops, print status to MONITOR
        static uint16 print_counter = 0;
        if (++print_counter >= 0) {
            print_counter = 0;
    
            char msg[64];
            sprintf(msg, "MOVI RX bytes: %lu\r\n", (unsigned long)movi_rx_byte_count);
            UART_MONITOR_PutString(msg);

            sprintf(msg, "Gimbal battery L: %.2f V, R: %.2f V\r\n",
                    FreeflyAPI.status.battery_v_left,
                    FreeflyAPI.status.battery_v_right);
            UART_MONITOR_PutString(msg);
        // Add more status prints here as you confirm you are receiving QX287
    }

        // Poll for status every second (optional)
        CyDelay(1000);
    }
}

/* [] END OF FILE */
