/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RISCV_ICENET_ETH_H
#define RISCV_ICENET_ETH_H

#include "plic_driver.h"
#include "icenet.h"
#include "bsp.h"

/* Driver instances*/
extern IceNetEthernet IceNetEthernetInstance;
extern volatile int FramesTx;
extern volatile int FramesRx;

#define RXBD_CNT		64	/* Number of RxBDs to use */
#define TXBD_CNT		64	/* Number of TxBDs to use */

void prvEMACDeferredInterruptHandlerTask( void *pvParameters );

int NICSetup(IceNetEthernet *nic);
int CreateInterruptTask(void);
void IceRxCallBack(void);
void IceRxIntrHandler(IceNetEthernet *nic);
int IceNetEthernetSetupIntrSystem(plic_instance_t *IntcInstancePtr,
				IceNetEthernet *IceNetEthernetInstancePtr,
				u16 IceNetRxIntrId);
void IceNetEthernetDisableIntrSystem(plic_instance_t *IntcInstancePtr,
				u16 IceNetRxIntrId);

#endif /* RISCV_ICENET_ETH_H */
