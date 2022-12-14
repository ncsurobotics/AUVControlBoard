#include <app.h>
#include <tusb.h>
#include <limits.h>
#include <pccomm.h>
#include <cmdctrl.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  RTOS object handles
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Task handles
TaskHandle_t usb_device_task;
TaskHandle_t cmdctrl_task;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Thread (task) functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Command and control task
 * cmdctrl (and motor_control) functions are not thread safe, thus only this thread
 * should ever call such functions
 */
void cmdctrl_task_func(void *arg){
    uint32_t notification;
    
    while(1){
        // Wait until a notification is received (blocks this thread)
        // notification value is a set of 32 notification bits
        xTaskNotifyWait(pdFALSE, UINT32_MAX, &notification, portMAX_DELAY);

        // Handle the notification
        if(notification & NOTIF_CMDCTRL_PCDAATA){
            // There is data to handle from the PC
            // Read and parse the data
            if(pccomm_read_and_parse()){
                // Inform cmdctrl there is a complete message
                cmdctrl_handle_message();
            }

            // If there is still data to handle, ensure flag is set again
            // This is necessary because read_and_parse may return true part way through reading data
            // Which means that usb_device_task will not notify again, but there is unhandled data
            if(tud_cdc_available())
                xTaskNotify(cmdctrl_task, NOTIF_CMDCTRL_PCDAATA, eSetBits);
        }
    }
}

/**
 * Thread to handle TinyUSB device mode events
 */
void usb_device_task_func(void *argument){
    tud_init(BOARD_TUD_RHPORT);
    while(1){
        // This call will block thread until there is / are event(s)
        tud_task();

        // If data now available, notify the communication task
        if(tud_cdc_available()){
            xTaskNotify(cmdctrl_task, NOTIF_CMDCTRL_PCDAATA, eSetBits);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// App initialization & management
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_init(void){
    // Create RTOS threads
    xTaskCreate(
        usb_device_task_func,
        "usbd_task",
        TASK_USB_DEVICE_SSIZE,
        NULL,
        TASK_USB_DEVICE_PRIORITY,
        &usb_device_task
    );
    xTaskCreate(
        cmdctrl_task_func,
        "cmdctrl_task",
        TASK_CMDCTRL_SSIZE,
        NULL,
        TASK_CMDCTRL_PRIORITY,
        &cmdctrl_task
    );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////