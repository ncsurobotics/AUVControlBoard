
#include <usbuart.h>
#include <circular_buffer.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define USBUART_CURR_RX_BUF_SZ          CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#define USBUART_CURR_TX_BUF_SZ          CONF_USB_CDCD_ACM_DATA_BULKOUT_MAXPKSZ
#define USBUART_TX_MSG_BUF_SZ           128
#define USBUART_RX_MSG_BUF_SZ           128


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Forward declared callback functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool usbuart_state_callback(usb_cdc_control_signal_t state);
bool usbuart_read_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);
bool usbuart_write_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool usbuart_initialized = false;                               // Tracks if usb is attached
bool usbuart_buffers_inited = false;                            // Tracks if buffers are initialzied
volatile uint8_t usbuart_curr_tx[USBUART_CURR_TX_BUF_SZ];       // Array to hold data currently being written
volatile uint8_t usbuart_curr_rx[USBUART_CURR_RX_BUF_SZ];       // Array to hold data currently being read
volatile uint8_t usbuart_tx_array[USBUART_TX_MSG_BUF_SZ];       // Backing array for TX buffer
volatile uint8_t usbuart_rx_array[USBUART_RX_MSG_BUF_SZ];       // Backing array for RX buffer
volatile circular_buffer usbuart_tx_buf;                        // Ring buffer of data to transmit
volatile circular_buffer usbuart_rx_buf;                        // Ring buffer holding data read
volatile uint32_t usbuart_curr_tx_len;                          // Size of uabuart_curr_tx


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool usbuart_init(void){
    if(!usbuart_buffers_inited){
        cb_init(&usbuart_tx_buf, usbuart_tx_array, USBUART_TX_MSG_BUF_SZ);
        cb_init(&usbuart_rx_buf, usbuart_rx_array, USBUART_RX_MSG_BUF_SZ);
    }
    if(!usbuart_initialized){
        if(cdcdf_acm_is_enabled()){
            cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usbuart_state_callback);
            usbuart_initialized = true;
        }
    }
    // TODO: Anything needed to handle reconnect?
    return usbuart_initialized;
}

unsigned int usbuart_write(uint8_t *data, unsigned int len){
    unsigned int i;
    
    // Add data into write buffer
    for(i = 0; i < len; ++i){
        if(CB_FULL(&usbuart_tx_buf))
            break;
        cb_write(&usbuart_tx_buf, data[i]);
    }

    // Number of bytes actually writeen (buffer may have become full)
    return i;
}

unsigned int usbuart_read(uint8_t *data, unsigned int len){
    unsigned int i = 0;

    // Remove data from read buffer
    for(i = 0; i < len; ++i){
        if(CB_EMPTY(&usbuart_rx_buf))
            break;
        cb_read(&usbuart_rx_buf, &data[i]);
    }

    // Number of bytes actually read (buffer may have become empty)
    return i;
}

bool usbuart_readone(uint8_t *data){
    if(CB_EMPTY(&usbuart_rx_buf))
        return false;
    cb_read(&usbuart_rx_buf, data);
    return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool usbuart_state_callback(usb_cdc_control_signal_t state){
    if (state.rs232.DTR) {
        // Register callbacks **AFTER** endpoints allocated
        cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usbuart_read_callback);
        cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usbuart_write_callback);

        // Initial write
        // Nothing is actually written, but the write callback is invoked when done
        // So next write should occur in the write callback
        usbuart_curr_tx_len = 0;
        cdcdf_acm_write((uint8_t *)usbuart_curr_tx, 0);

        // Initial read
        // Any data read will be available in the read callback
        // The next read should occur in the read callback
        cdcdf_acm_read((uint8_t *)usbuart_curr_rx, USBUART_CURR_RX_BUF_SZ);
    }
    return false;
}

// Called when read completes. "count" is number of bytes that were read
bool usbuart_read_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){
    // Move previously read data into read buffer
    for(uint32_t i = 0; i < count; ++i){
        cb_write(&usbuart_rx_buf, usbuart_curr_rx[i]);
    }

    // Start next read
    cdcdf_acm_read((uint8_t *)usbuart_curr_rx, USBUART_CURR_RX_BUF_SZ);

    // No error
    return false;
}

// Called when write complets. "count" is number of bytes that were written
bool usbuart_write_callback(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count){        
    // TODO

    // No error
    return false;
}
