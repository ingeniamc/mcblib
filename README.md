# MCBLib

Motion core library implements the Ingenia Motion Control Bus protocol.

## Highlights ##

* Develop for embedded devices, from low performance MCU to powerful MPU
* Developed in C language

## Requirements ##

The next elements are needed on the platform where this code is going to be added:

- SPI interface
- 1 General purpose input with interrupt capabilities
- 2 General purpose outputs

## Contribution guideline ##

- This repository follows a modified version of [gitflow](http://doc.ingeniamc.com/display/Instructions/Firmware+Development+Procedure)
- [Code style](http://doc.ingeniamc.com/display/Instructions/Low+layers+coding+style)

### HAL adaptation ###

This repository implements higher layers of the motion control bus. Users has to implement the required Hardware Abstraction Layer (HAL) / Board Support Layer (BSP) or any low layer drivers dependent on the platform.

In order to make the developer life easier, the next facilities has been added into the code:

1. User application code just need to include mcb.h file and use the included API
2. HAL adaptation can be done modifying the file mcb_usr.c (not recommended)
2. HAL adaptation can be done directly on the user code (recommended)
3. mcb_usr.h includes all the functions that should be implemented on the user low layers
4. Description of each function is added inside mcb_usr.h

The recommended procedure to implement the HAL is:

1. Include into the HAL file the mcb_usr.h header
2. Implement the function from the mcb_usr.h
    
	    ******************************************************************************
	      File Name  : SPI.c
	    
	      Description: This file provides code for the configuration
	       of the SPI instances.
	    ******************************************************************************
		
		include "mcb_usr.h"
	    
	    
	    ...
    
    
	    void 
		Mcb_IntfSPITransfer(uint16_t u16Id, uint16_t* pu16In, uint16_t* pu16Out, uint16_t u16Sz)
	    {
	    	switch (u16Id)
	    	{
	    		case 0:
	    			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	    			HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*) pu16In, (uint8_t*) pu16Out, u16Sz);
	    			break;
	    		default:
	    			/* Nothing */
	    			break;
	    	}
	    }


## Who do I talk to? ##

This repository is maintained by Ingenia FW team.
