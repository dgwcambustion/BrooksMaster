/**
@file
Arduino library for communicating with Books slaves over R485, adapted from BrooksMaster.

@defgroup setup BrooksMaster Object Instantiation/Initialization
@defgroup buffer BrooksMaster Buffer Management
@defgroup discrete Brooks Function Codes for Discrete Coils/Inputs
@defgroup register Brooks Function Codes for Holding/Input Registers
@defgroup constant Brooks Function Codes, Exception Codes
*/
/*

  BrooksMaster.h - Arduino library for communicating with Brooks MFCs 
  over RS485, adapted from BrooksMaster

  Library:: BrooksMaster
  Autor:: David Walker


*/

  
#ifndef BrooksMaster_h
#define BrooksMaster_h


/**
@def __BrooksMASTER_DEBUG__ (0)
Set to 1 to enable debugging features within class:
  - PIN A cycles for each byte read in the Brooks response
  - PIN B cycles for each millisecond timeout during the Brooks response
*/
#define __BrooksMASTER_DEBUG__ (0)
#define __BrooksMASTER_DEBUG_PIN_A__ 4
#define __BrooksMASTER_DEBUG_PIN_B__ 5

/* _____STANDARD INCLUDES____________________________________________________ */
// include types & constants of Wiring core API
#include "Arduino.h"

/* _____UTILITY MACROS_______________________________________________________ */


/* _____PROJECT INCLUDES_____________________________________________________ */


/* _____CLASS DEFINITIONS____________________________________________________ */
/**
Arduino class library for communicating with Brooks slaves over RS485.
*/
class BrooksMaster
{
  public:
    BrooksMaster();
   
    void begin(uint8_t, Stream &serial);
    void idle(void (*)());
    void preTransmission(void (*)());
    void postTransmission(void (*)());

    // Brooks exception codes
    /**
    Brooks protocol illegal function exception.
    
    The function code received in the query is not an allowable action for
    the server (or slave). This may be because the function code is only
    applicable to newer devices, and was not implemented in the unit
    selected. It could also indicate that the server (or slave) is in the
    wrong state to process a request of this type, for example because it is
    unconfigured and is being asked to return register values.
    
    @ingroup constant
    */
    static const uint8_t ku8MBIllegalFunction            = 0x01;

    /**
    Brooks protocol illegal data address exception.
    
    The data address received in the query is not an allowable address for 
    the server (or slave). More specifically, the combination of reference 
    number and transfer length is invalid. For a controller with 100 
    registers, the ADU addresses the first register as 0, and the last one 
    as 99. If a request is submitted with a starting register address of 96 
    and a quantity of registers of 4, then this request will successfully 
    operate (address-wise at least) on registers 96, 97, 98, 99. If a 
    request is submitted with a starting register address of 96 and a 
    quantity of registers of 5, then this request will fail with Exception 
    Code 0x02 "Illegal Data Address" since it attempts to operate on 
    registers 96, 97, 98, 99 and 100, and there is no register with address 
    100. 
    
    @ingroup constant
    */
    static const uint8_t ku8MBIllegalDataAddress         = 0x02;
    
    /**
    Brooks protocol illegal data value exception.
    
    A value contained in the query data field is not an allowable value for 
    server (or slave). This indicates a fault in the structure of the 
    remainder of a complex request, such as that the implied length is 
    incorrect. It specifically does NOT mean that a data item submitted for 
    storage in a register has a value outside the expectation of the 
    application program, since the Brooks protocol is unaware of the 
    significance of any particular value of any particular register.
    
    @ingroup constant
    */
    static const uint8_t ku8MBIllegalDataValue           = 0x03;
    
    /**
    Brooks protocol slave device failure exception.
    
    An unrecoverable error occurred while the server (or slave) was
    attempting to perform the requested action.
    
    @ingroup constant
    */
    static const uint8_t ku8MBSlaveDeviceFailure         = 0x04;

    // Class-defined success/exception codes
    /**
    BrooksMaster success.
    
    Brooks transaction was successful; the following checks were valid:
      - slave ID
      - function code
      - response code
      - data
      - CRC
      
    @ingroup constant
    */
    static const uint8_t ku8MBSuccess                    = 0x00;
    
    /**
    BrooksMaster invalid response slave ID exception.
    
    The slave ID in the response does not match that of the request.
    
    @ingroup constant
    */
    static const uint8_t ku8MBInvalidSlaveID             = 0xE0;
    
    /**
    BrooksMaster invalid response function exception.
    
    The function code in the response does not match that of the request.
    
    @ingroup constant
    */
    static const uint8_t ku8MBInvalidFunction            = 0xE1;
    
    /**
    BrooksMaster response timed out exception.
    
    The entire response was not received within the timeout period, 
    BrooksMaster::ku8MBResponseTimeout. 
    
    @ingroup constant
    */
    static const uint8_t ku8MBResponseTimedOut           = 0xE2;
    
    /**
    BrooksMaster invalid response CRC exception.
    
    The CRC in the response does not match the one calculated.
    
    @ingroup constant
    */
    static const uint8_t ku8MBInvalidCRC                 = 0xE3;
    
    uint8_t getResponseBuffer(uint8_t);
    void     clearResponseBuffer();
    uint8_t  setTransmitBuffer(uint8_t, uint8_t);
    void     clearTransmitBuffer();
	uint8_t getStatusBuffer(uint8_t);
	void     clearStatusBuffer();
    
    void beginTransmission(uint8_t);
    uint8_t requestFrom(uint16_t, uint16_t);
    void sendBit(bool);
    void send(uint8_t);
    uint8_t available(void);
    uint16_t receive(void);
    
    
    uint8_t  readRegister(uint8_t);
    uint8_t  writeRegister(uint8_t);
    
  private:
    Stream* _serial;                                             ///< reference to serial port object
	uint8_t	 _u8Preamble    	    = 0xFF; 						 ///< Preamble characters
	uint8_t	 _u8Delimiter   	    = 0x02; 					///<	short frame address, STX frame
    uint8_t  _u8MBSlave;                                         ///< Brooks slave (1..255) initialized in begin()
    static const uint8_t ku8MaxBufferSize                = 64;   ///< size of response/transmit buffers    
    uint16_t _u8Command;                                    ///< Command to slave
	uint8_t  _u8StatusBuffer[2] = {0, 0};						///< Brooks status buffer
    uint16_t _u16ReadQty;                                        ///< quantity of words to read
    uint16_t _u8ResponseBuffer[ku8MaxBufferSize];               ///< buffer to store Brooks slave response; read via GetResponseBuffer()
    uint16_t _u8WriteAddress;                                   ///< slave register to which to write
    uint16_t _u8TransmitBuffer[ku8MaxBufferSize];               ///< buffer containing data to transmit to Brooks slave; set via SetTransmitBuffer()
    uint16_t* txBuffer; // from Wire.h -- need to clean this up Rx
    uint8_t _u8TransmitBufferIndex;
    uint16_t _u16TransmitBufferLength;
    uint16_t* rxBuffer; // from Wire.h -- need to clean this up Rx
    uint8_t _u8ResponseBufferIndex;
    uint8_t _u8ResponseBufferLength;

    // Brooks functions
    static const uint8_t ku8MBReadRegister    	    = 0x01; ///< 
    static const uint8_t ku8MBWriteRegister		    = 0x02; ///< 
    
    // Brooks timeout [milliseconds]
    static const uint16_t ku16MBResponseTimeout          = 2000; ///< Brooks timeout [milliseconds]
    
    // master function that conducts Brooks transactions
    uint8_t BrooksMasterTransaction(uint8_t u8MBFunction);
    
    // idle callback function; gets called during idle time between TX and RX
    void (*_idle)();
    // preTransmission callback function; gets called before writing a Brooks message
    void (*_preTransmission)();
    // postTransmission callback function; gets called after a Brooks message has been sent
    void (*_postTransmission)();
};
#endif


