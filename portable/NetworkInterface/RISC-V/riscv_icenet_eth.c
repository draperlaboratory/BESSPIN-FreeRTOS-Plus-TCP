#include "riscv_icenet_eth.h"

#include "icenet.h"
#include <stdio.h> // for printf
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "NetworkBufferManagement.h"

/* FreeRTOS+TCP includes. */
#include "NetworkInterface.h"

/* Driver instances*/
IceNetEthernet IceNetEthernetInstance;

static TaskHandle_t prvEMACDeferredInterruptHandlerTaskHandle = NULL;

/*
 * Counters to be incremented by callbacks
 */
volatile int FramesRx;	  /* Num of frames that have been received */
volatile int FramesTx;	  /* Num of frames that have been sent */
volatile int DeviceErrors; /* Num of errors detected in the device */

// @mpodhradsky: "Declaring AxiEthernetMAC as weak, so it can be overriden in the user app"
char AxiEthernetMAC[6]  __attribute__((weak)) = { configMAC_ADDR0, configMAC_ADDR1, configMAC_ADDR2, configMAC_ADDR3, configMAC_ADDR4, configMAC_ADDR5 };

/**
 * Handle RX data
 */
// Simple driver
void prvEMACDeferredInterruptHandlerTask( void *pvParameters ) {
	(void) pvParameters;
	NetworkBufferDescriptor_t *pxBufferDescriptor;
	int xBytesReceived = 0;
	/* Used to indicate that xSendEventStructToIPTask() is being called because
	of an Ethernet receive event. */
	IPStackEvent_t xRxEvent;
 	IceEthernetFrame * FramePtr;
	char *FramePtrIncr;
    int BdLimit = 1;
	uint32_t status;


	for(;;) {
		taskENTER_CRITICAL();
		int recvAvail = recv_comp_avail(&IceNetEthernetInstance);

		/* Wait for the Ethernet MAC interrupt to indicate that another packet
        has been received.  The task notification is used in a similar way to a
        counting semaphore to count Rx events, but is a lot more efficient than
        a semaphore. */
		if (recvAvail == 0) {
			icenet_alloc_recv(&IceNetEthernetInstance);
			icenet_set_intmask(&IceNetEthernetInstance, ICENET_INTMASK_RX);
			taskEXIT_CRITICAL();
			ulTaskNotifyTake( pdFALSE, portMAX_DELAY );
			continue;
		}

		configASSERT(recvAvail >= BdLimit);

		/* Examine the BD */
		status = icenet_recv(&IceNetEthernetInstance, &FramePtr, &xBytesReceived);
		FramePtrIncr = (char *) FramePtr + 2;
		configASSERT( status != 0 );

		if (xBytesReceived > 0) {
			/* Allocate a network buffer descriptor that points to a buffer
            large enough to hold the received frame.  As this is the simple
            rather than efficient example the received data will just be copied
            into this buffer. */
            pxBufferDescriptor = pxGetNetworkBufferWithDescriptor( xBytesReceived, 0 );

			configASSERT( pxBufferDescriptor != NULL);

			/* pxBufferDescriptor->pucEthernetBuffer now points to an Ethernet
                buffer large enough to hold the received data.  Copy the
                received data into pcNetworkBuffer->pucEthernetBuffer.  Here it
                is assumed ReceiveData() is a peripheral driver function that
                copies the received data into a buffer passed in as the function's
                parameter.  Remember! While is is a simple robust technique -
               it is not efficient.  An example that uses a zero copy technique
            is provided further down this page. */
            memcpy(pxBufferDescriptor->pucEthernetBuffer, FramePtrIncr, xBytesReceived - 2);
            taskEXIT_CRITICAL();
            pxBufferDescriptor->xDataLength = xBytesReceived - 2;

			/* See if the data contained in the received Ethernet frame needs
            to be processed.  NOTE! It is preferable to do this in
            the interrupt service routine itself, which would remove the need
            to unblock this task for packets that don't need processing. */
            if( eConsiderFrameForProcessing( pxBufferDescriptor->pucEthernetBuffer )
                                                                  == eProcessBuffer )
            {
				/* The event about to be sent to the TCP/IP is an Rx event. */
                xRxEvent.eEventType = eNetworkRxEvent;

                /* pvData is used to point to the network buffer descriptor that
                now references the received data. */
                xRxEvent.pvData = ( void * ) pxBufferDescriptor;

                /* Send the data to the TCP/IP stack. */
                if( xSendEventStructToIPTask( &xRxEvent, 0 ) == pdFALSE )
                {
                    /* The buffer could not be sent to the IP task so the buffer
                    must be released. */
                    vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );

                    /* Make a call to the standard trace macro to log the
                    occurrence. */
                    iptraceETHERNET_RX_EVENT_LOST();
                }
                else
                {
                    /* The message was successfully sent to the TCP/IP stack.
                    Call the standard trace macro to log the occurrence. */
                    iptraceNETWORK_INTERFACE_RECEIVE();
                }
			}
			else
            {
                /* The Ethernet frame can be dropped, but the Ethernet buffer
                must be released. */
                vReleaseNetworkBufferAndDescriptor( pxBufferDescriptor );
            }
		} else {
            taskEXIT_CRITICAL();
        }
	}
}

/**
 * Initialize Interrupt Handler Task
 */
int CreateInterruptTask()
{
	xTaskCreate( prvEMACDeferredInterruptHandlerTask, "prvEMACDeferredInterruptHandlerTask", configMINIMAL_STACK_SIZE*5, NULL,
		ipconfigIP_TASK_PRIORITY - 1, &prvEMACDeferredInterruptHandlerTaskHandle);

	return 0;
}


/**
 * Initialize Ethernet
 */
int NICSetup(IceNetEthernet *nic)
{
	configASSERT( nic->IsStarted == 0 )
	/*
	 * Set the base address
	 */
	nic->BaseAddress = ICENET_BASEADDR;

	/*
	 * Initialize IceNet NIC hardware.
	 */
	printf("Setting up icenet device\n");
	icenet_open(nic);
	CreateInterruptTask();

	nic->IsStarted = 1;

	return 0;
}

void IceRxCallBack()
{
	configASSERT( prvEMACDeferredInterruptHandlerTaskHandle != NULL);

	static BaseType_t askForContextSwitch = pdFALSE;
	vTaskNotifyGiveFromISR( prvEMACDeferredInterruptHandlerTaskHandle, &askForContextSwitch);

	FramesRx++;
}

void IceRxIntrHandler(IceNetEthernet *nic)
{
	icenet_clear_intmask(nic, ICENET_INTMASK_RX);
	IceRxCallBack();
}

int IceNetEthernetSetupIntrSystem(plic_instance_t *IntcInstancePtr,
				IceNetEthernet *IceNetEthernetInstancePtr,
				// u16 IceNetTxIntrId,
				u16 IceNetRxIntrId)
{
	int Status;

	/*
	 * Initialize the interrupt controller and connect the ISR
	 */
	Status = PLIC_register_interrupt_handler(IntcInstancePtr, IceNetRxIntrId,
						 (void *) IceRxIntrHandler, IceNetEthernetInstancePtr);
    if (Status == 0) {
        printf("Unable to connect ISR to interrupt controller: IceNetEthernet %u\r\n", IceNetRxIntrId);
        return 1;
    }

	return 0;
}

void IceNetEthernetDisableIntrSystem(plic_instance_t *IntcInstancePtr,
					// u16 IceNetTxIntrId,
					u16 IceNetRxIntrId)
{
	/* Stop the tasks */
	vTaskSuspend(prvEMACDeferredInterruptHandlerTaskHandle);

	/*
	 * Disconnect the interrupts for the DMA TX and RX channels
	 */
    PLIC_unregister_interrupt_handler(IntcInstancePtr, IceNetRxIntrId);

	/* Now the callbacks won't be called we can delete the tasks */
	vTaskDelete(prvEMACDeferredInterruptHandlerTaskHandle);
}
