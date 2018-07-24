# API description

Motion core library has been designed to be used inside any kind of software environment: bare-metal and OS based applications.

## Blocking / non-blocking modes
The library allows two mode of operation:

- **Blocking mode**. In this mode the read and write methods blocks the main thread until a reply from the slave is received or the timeout expires.
- **Non-blocking mode**. In this mode the read and write methods returns immediatly allowing the main trhead keep doing other things. Developers are the responsible of manage the state machine of the communications and implement their own timeout.

In case of need the non-blocking mode, it is highly recommended to use the blocking mode implementation as reference.

## Node identification
Motion control bus supports up to 15 slaves connected to the same SPI interface. See specific [Motion Control Bus documentation](http://doc.ingeniamc.com/pages/viewpage.action?pageId=70682569) for further details.

## Messages
This library has been implemented using message structs that simplifies the management of communications between threads in case of using OS based applications. 

The motion control bus protocol defines two communications states that use two different message types:

### Configuration messages
Configuration messages are stored into the next struct:

    /** Frame data struct */
    typedef struct
    {
	    /** Destination / source node */
	    uint16_t u16Node;
	    /** Target register address */
	    uint16_t u16Addr;
	    /** Master / slave command */
	    uint16_t u16Cmd;
	    /** Message total size (words) */
	    uint16_t u16Size;
	    /** Static data */
	    uint16_t u16Data[MCB_MAX_DATA_SZ];
	    /** Message status */
	    Mcb_EStatus eStatus;
    } Mcb_TMsg;

Mcb\_Write & Mcb\_Read are the functions used to operate with configuration messages. The arguments of these functions are Mcb\_TMsg which are use as input and outputs. The operation steps are:

1. Create and / or fill a Mcb_TMsg (Fields: u16Node, u16Addr, u16Size and u16Data)
2. Send the message as argument of Mcb_Write or Mcb_Read
3. Process the overwritten message to see the result: eStatus indicates the result of the operation, the other parameters contains the reply of the slave.

In blocking mode, both functions blocks the program until they get the reply of the slave or a timeout expires, so the returned eStatus might be:

    - MCB_SUCCESS
    - MCB_WRITE_ERROR
    - MCB_READ_ERROR
    - MCB_ERROR

In non-blocking mode, the developer must managed all the states of the write/read operation:

    /** Transmission successful */
    - MCB_SUCCESS
    /** Bus in stand by */
    - MCB_STANDBY
    /** Sending a write request */
    - MCB_WRITE_REQUEST
    /** Processing answer from write request */
    - MCB_WRITE_ANSWER
    /** Sending a read request */
    - MCB_READ_REQUEST
    /** Processing answer from read request */
    - MCB_READ_ANSWER
    /** Write config request error */
    - MCB_WRITE_ERROR
    /** Read config request error */
    - MCB_READ_ERROR
    /** Transaction error */
    - MCB_ERROR

### Cyclic messages
Cyclic messages have been designed to get a high update loop rate of critical task for control purpose. The configuration and use of cyclics requires the next steps:

1. Mapping the desired registers. This procedure configures the slaves to synchronize the data into the frames. The functions Mcb\_TxMap and Mcb\_RxMap returns a pointer to these frames so the user is able to use exchanged data as standard variables.
2. Enabling cyclic mode into the slave. Mcb\_SetCyclicMode is the responsible of enabling the cyclic capabilities of the slave
3. Calling cyclic process periodically. The cycle of the communications is fully managed by the master, so the user is the responsible of calling periodically the Mcb\_CyclicProcess.

Once the cyclic mode is enabled, the configuration messages are transmitted through cyclic messages so special function must be activated in order to keep configuration messages active.

A user function callback must be linked to cyclic process through the Mcb\_AttachCfgOverCyclicCB function. Then the Mcb\_Write & Mcb\_Read will request a configuration transmission but instead of blocking the thread until the slave reply, it will return immediately and the linked functin will be called once the transmission is finished.


## CRC implementation
There are three main types of CRC implementation:

1. Pure software
2. By software with hardware support
3. Pure hardware

This library support all of them. By default, a pure software implementation is available on the mcb\_usr.c file. The method is declared as weak, so the users may overwrite the function by its own implementation using hardware support from the device. Furthermore, if the device is able to compute automatically the CRC, during the initialization of the instance the parameter bCalcCrc is used to disable the software CRC. 