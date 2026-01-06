/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"

// Necessary ports of the FreeflyAPI
#include "QX_Protocol.h"
#include "QX_Protocol_App.h"

// Constants
#define LOOP_DELAY_MS 20  // 50 Hz loop

// Example Rx handler
void UART_ReceiveHandler(void)
{
    uint8_t rxByte;
    // Poll for received bytes (or use interrupt if preferred)
    while ((rxByte = UART_GetChar()) != 0)
    {
        // Pass each received byte to the MOVI API
        QX_StreamRxCharSM(QX_COMMS_PORT_UART, rxByte);
    }
}
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

// UART RX ISR prototype.
// The UART componnent interrupts on Rx, this ISR then 
// processes on this interrupt.
CY_ISR(UART_RX_ISR)
{
    uint8_t rxByte;
    // Process all bytes in the RX buffer
    while (UART_GetRxBufferSize() > 0)
    {
        rxByte = UART_GetByte(); // Get the next received byte
        QX_StreamRxCharSM(QX_COMMS_PORT_UART, rxByte); // Pass to QX Protocol
    }
}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Freefly over COM Initialization */
    UART_Start();                                // Start UART on hardware RxTx
    QX_InitCli(&QX_Clients[0],                   // Chose client instance, only one.
                QX_DEV_ID_MOVI_API_CONTROLLER,   // Movi API Controller Address.
                QX_ParsePacket_Cli_MoVI_Ctrl_CB  // Callback function. This parses the returned bits (Default).
    );
    UART_PutString("% -- Initialized! -- %\r\n");
    isr_rx_StartEx(UART_RX_ISR);
    FreeflyAPI.begin();

    for(;;)
    {
        CyDelay(500);
        
        uint8_t button_state = SW1_debounced_Read();
        char msg[20];
        sprintf(msg, "SW1: %d\r\n", button_state); // Converts 0/1 to ASCII
        UART_PutString(msg); // Prints "SW1: 0" or "SW1: 1"
        
        // Example: Set control values
        FreeflyAPI.control.pan.type = RATE;
        FreeflyAPI.control.tilt.type = RATE;
        FreeflyAPI.control.tilt.value = 0.0f;  // No tilt
        
        if(button_state){
            FreeflyAPI.control.pan.value = 0.5f;   // Example value
        } else {
            FreeflyAPI.control.pan.value = 0.0f;   // Example value
        }
        
        // Encode the control message
        uint8_t uart_buf[128];
        uint16_t uart_len;
        ff_api_encode_control(uart_buf, &uart_len);
        
        // Calculate checksum on control packet
        uint8_t checksum = 0;
        for (int i = 3; i <= 30; i++) {
            checksum += uart_buf[i];
        }
        checksum = 255 - (checksum % 256);
        // Compare to packet
        if (uart_buf[31] == checksum) {
            // Checksum is valid
             UART_PutString("Checksum is valid!\r\n");
        } else {
            // Checksum error
             UART_PutString("Checksum error ):\r\n");
        }
        
        UART_PutString("Raw command:\r\n");
        // Send the message over UART
        UART_PutArray(uart_buf, uart_len);
        
        
        
        // Create spacing
        UART_PutString("\r\n");
        UART_PutString("\r\n");

        // Poll for status every second (optional)
        CyDelay(1000);
    }
}

/* [] END OF FILE */
