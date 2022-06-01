/**
@file
Arduino library for communicating with Books slaves over R485, adapted from BrooksMaster.
*/
/*

  BrooksMaster.h - Arduino library for communicating with Brooks MFCs 
  over RS485, adapted from BrooksMaster

  Library:: BrooksMaster
  Autor:: David Walker


*/

/* _____PROJECT INCLUDES_____________________________________________________ */
#include "BrooksMaster.h"


/* _____GLOBAL VARIABLES_____________________________________________________ */


/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
Constructor.

Creates class object; initialize it using BrooksMaster::begin().

@ingroup setup
*/
BrooksMaster::BrooksMaster(void)
{
  _idle = 0;
  _preTransmission = 0;
  _postTransmission = 0;
}

/**
Initialize class object.

Assigns the Brooks slave ID and serial port.
Call once class has been instantiated, typically within setup().

@param slave Brooks slave ID (1..255), short frame address
@param &serial reference to serial port object (Serial, Serial1, ... Serial3)
@ingroup setup
*/
void BrooksMaster::begin(uint8_t slave, Stream &serial)
{
//  txBuffer = (uint16_t*) calloc(ku8MaxBufferSize, sizeof(uint16_t));
  _u8MBSlave = slave;
  _serial = &serial;
  _u8TransmitBufferIndex = 0;
  _u16TransmitBufferLength = 0;
  
#if __BrooksMASTER_DEBUG__
  pinMode(__BrooksMASTER_DEBUG_PIN_A__, OUTPUT);
  pinMode(__BrooksMASTER_DEBUG_PIN_B__, OUTPUT);
#endif
}


void BrooksMaster::beginTransmission(uint8_t u8Address)
{
  _u8WriteAddress = u8Address;
  _u8TransmitBufferIndex = 0;
  
}

// eliminate this function in favor of using existing MB request functions
uint8_t BrooksMaster::requestFrom(uint16_t address, uint16_t quantity)
{
  uint8_t read;
  // clamp to buffer length
  if (quantity > ku8MaxBufferSize)
  {
    quantity = ku8MaxBufferSize;
  }
  // set rx buffer iterator vars
  _u8ResponseBufferIndex = 0;
  _u8ResponseBufferLength = read;

  return read;
}


void BrooksMaster::send(uint8_t data)
{
  send(data);
}

uint8_t BrooksMaster::available(void)
{
  return _u8ResponseBufferLength - _u8ResponseBufferIndex;
}


uint16_t BrooksMaster::receive(void)
{
  if (_u8ResponseBufferIndex < _u8ResponseBufferLength)
  {
    return _u8ResponseBuffer[_u8ResponseBufferIndex++];
  }
  else
  {
    return 0xFFFF;
  }
}


/**
Set idle time callback function (cooperative multitasking).

This function gets called in the idle time between transmission of data
and response from slave. Do not call functions that read from the serial
buffer that is used by BrooksMaster. Use of i2c/TWI, 1-Wire, other
serial ports, etc. is permitted within callback function.

@see BrooksMaster::BrooksMasterTransaction()
*/
void BrooksMaster::idle(void (*idle)())
{
  _idle = idle;
}

/**
Set pre-transmission callback function.

This function gets called just before a Brooks message is sent over serial.
Typical usage of this callback is to enable an RS485 transceiver's
Driver Enable pin, and optionally disable its Receiver Enable pin.

@see BrooksMaster::BrooksMasterTransaction()
@see BrooksMaster::postTransmission()
*/
void BrooksMaster::preTransmission(void (*preTransmission)())
{
  _preTransmission = preTransmission;
}

/**
Set post-transmission callback function.

This function gets called after a Brooks message has finished sending
(i.e. after all data has been physically transmitted onto the serial
bus).

Typical usage of this callback is to enable an RS485 transceiver's
Receiver Enable pin, and disable its Driver Enable pin.

@see BrooksMaster::BrooksMasterTransaction()
@see BrooksMaster::preTransmission()
*/
void BrooksMaster::postTransmission(void (*postTransmission)())
{
  _postTransmission = postTransmission;
}


/**
Retrieve data from response buffer.

@see BrooksMaster::clearResponseBuffer()
@param u8Index index of response buffer array (0x00..0x3F)
@return value in position u8Index of response buffer (0x0000..0xFFFF)
@ingroup buffer
*/
uint8_t BrooksMaster::getResponseBuffer(uint8_t u8Index)
{
  if (u8Index < ku8MaxBufferSize)
  {
    return _u8ResponseBuffer[u8Index];
  }
  else
  {
    return 0xFFFF;
  }
}

/**
Retrieve status from status buffer.

@param u8Index index of response buffer array (0x00..0x3F)
@return value in position u8Index of response buffer (0x0000..0xFFFF)
@ingroup buffer
*/
uint8_t BrooksMaster::getStatusBuffer(uint8_t u8Index)
{
  if (u8Index < 3)
  {
    return _u8StatusBuffer[u8Index];
  }
  else
  {
    return 0xFFFF;
  }
}


/**
Clear Brooks response buffer.

@see BrooksMaster::getResponseBuffer(uint8_t u8Index)
@ingroup buffer
*/
void BrooksMaster::clearResponseBuffer()
{
  uint8_t i;
  
  for (i = 0; i < ku8MaxBufferSize; i++)
  {
    _u8ResponseBuffer[i] = 0;
  }
}

/**
Clear Brooks status buffer.

@see BrooksMaster::getStatusBuffer(uint8_t u8Index)
@ingroup buffer
*/
void BrooksMaster::clearStatusBuffer()
{
  uint8_t i;
  
  for (i = 0; i < 3; i++)
  {
    _u8StatusBuffer[i] = 0;
  }
}


/**
Place data in transmit buffer.

@see BrooksMaster::clearTransmitBuffer()
@param u8Index index of transmit buffer array (0x00..0x3F)
@param u16Value value to place in position u8Index of transmit buffer (0x0000..0xFFFF)
@return 0 on success; exception number on failure
@ingroup buffer
*/
uint8_t BrooksMaster::setTransmitBuffer(uint8_t u8Index, uint8_t u8Value)
{
  if (u8Index < ku8MaxBufferSize)
  {
    _u8TransmitBuffer[u8Index] = u8Value;
	_u16TransmitBufferLength++;
    return ku8MBSuccess;
  }
  else
  {
    return ku8MBIllegalDataAddress;
  }
}


/**
Clear Brooks transmit buffer.

@see BrooksMaster::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value)
@ingroup buffer
*/
void BrooksMaster::clearTransmitBuffer()
{
  uint8_t i;
  _u16TransmitBufferLength = 0;
  for (i = 0; i < ku8MaxBufferSize; i++)
  {
    _u8TransmitBuffer[i] = 0;
  }
}

/**
Brooks function Read Registers.

This function code is used to read the contents of a registers in a remote device. 
The request specifies the starting register address and the number of registers. 

The register data in the response buffer is packed as one word per 
register.

@param u16ReadAddress address of the first holding register (0x0000..0xFFFF)
@param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
@return 0 on success; exception number on failure
@ingroup register
*/
uint8_t BrooksMaster::readRegister(uint8_t u8ReadAddress)
{
  _u8Command = u8ReadAddress;
  return BrooksMasterTransaction(ku8MBReadRegister);
}


/**
Brooks function to Write Register.

This function code is used to write a single  register in a 
remote device. The request specifies the address of the register to be 
written. 

@param u16WriteAddress address of the holding register (0x0000..0xFFFF)
@param u16WriteValue value to be written to holding register (0x0000..0xFFFF)
@return 0 on success; exception number on failure
@ingroup register
*/
uint8_t BrooksMaster::writeRegister(uint8_t u8WriteAddress)
{
  _u8Command = u8WriteAddress;
  return BrooksMasterTransaction(ku8MBWriteRegister);
}


/* _____PRIVATE FUNCTIONS____________________________________________________ */
/**
Brooks transaction engine.
Sequence:
  - assemble Brooks Request Application Data Unit (ADU),
    based on particular function called
  - transmit request over selected serial port
  - wait for/retrieve response
  - evaluate/disassemble response
  - return status (success/exception)

@param u8MBFunction Brooks function (0x01..0xFF)
@return 0 on success; exception number on failure
*/
uint8_t BrooksMaster::BrooksMasterTransaction(uint8_t u8MBFunction)
{
  uint8_t u8BrooksADU[256];
  uint8_t u8BrooksADUSize = 0;
  uint8_t i, u8Qty;
  uint16_t u16CRC;
  uint32_t u32StartTime;
  uint8_t u8BytesLeft = ku8MaxBufferSize;
  uint8_t u8MBStatus = ku8MBSuccess;
  uint8_t u8CRC = 0x00;
  uint8_t u8MessageStart = 0;
  bool StartDetected = false; 
  uint8_t u8ResponseByteCount = 0xFF;
  
  
  // assemble Brooks Request Application Data Unit
  //5 preamble characters
  u8BrooksADU[u8BrooksADUSize++] = _u8Preamble;
  u8BrooksADU[u8BrooksADUSize++] = _u8Preamble;
  u8BrooksADU[u8BrooksADUSize++] = _u8Preamble;
  u8BrooksADU[u8BrooksADUSize++] = _u8Preamble;
  u8BrooksADU[u8BrooksADUSize++] = _u8Preamble;
  
  u8BrooksADU[u8BrooksADUSize++] = _u8Delimiter; 
  u8BrooksADU[u8BrooksADUSize++] = 0b10000000 + _u8MBSlave;//Device address
  
  u8BrooksADU[u8BrooksADUSize++] = _u8Command; //Command
  
  switch(u8MBFunction)
  {
    case ku8MBReadRegister:
		u8BrooksADU[u8BrooksADUSize++] = 0; //Byte Count
		break;
    case ku8MBWriteRegister:
		u8BrooksADU[u8BrooksADUSize++] = _u16TransmitBufferLength; //Byte Count
		break;
  }
  
  switch(u8MBFunction)
  {
    case ku8MBWriteRegister:
      for (i = 0; i < _u16TransmitBufferLength; i++)
      {
        u8BrooksADU[u8BrooksADUSize++] = _u8TransmitBuffer[i]; //data to write
      }
      break;
  }
  
  // append CRC
  for (i = 5; i < u8BrooksADUSize; i++)
  {
    u8CRC ^=u8BrooksADU[i];
  }
  u8BrooksADU[u8BrooksADUSize++] = u8CRC;
  //u8BrooksADU[u8BrooksADUSize] = 0; //is this needed?

  // flush receive buffer before transmitting request
  while (_serial->read() != -1);

  // transmit request
  if (_preTransmission)
  {
    _preTransmission();
  }
  for (i = 0; i < u8BrooksADUSize; i++)
  {
    _serial->write(u8BrooksADU[i]);
	//Serial.println(u8BrooksADU[i], HEX);
  }
  
  u8BrooksADUSize = 0;
  
  
  _serial->flush();    // flush transmit buffer
  if (_postTransmission)
  {
    _postTransmission();
  }
  
  // loop until we run out of time or bytes, or an error occurs
  u32StartTime = millis();
  while (u8BytesLeft && !u8MBStatus)
  {
    if (_serial->available())
    {
#if __BrooksMASTER_DEBUG__
      digitalWrite(__BrooksMASTER_DEBUG_PIN_A__, true);
#endif
      u8BrooksADU[u8BrooksADUSize++] = _serial->read();
	  //Serial.println(u8BrooksADU[u8BrooksADUSize-1], HEX);
      u8BytesLeft--;
#if __BrooksMASTER_DEBUG__
      digitalWrite(__BrooksMASTER_DEBUG_PIN_A__, false);
#endif
    }
    else
    {
#if __BrooksMASTER_DEBUG__
      digitalWrite(__BrooksMASTER_DEBUG_PIN_B__, true);
#endif
      if (_idle)
      {
        _idle();
      }
#if __BrooksMASTER_DEBUG__
      digitalWrite(__BrooksMASTER_DEBUG_PIN_B__, false);
#endif
    }
    
    // detect start of message
    if (u8BrooksADUSize >= 3 && u8BrooksADU[u8BrooksADUSize-3] == 0xFF && u8BrooksADU[u8BrooksADUSize-2] == 0xFF && u8BrooksADU[u8BrooksADUSize-1] == 6 && StartDetected == false){
		u8MessageStart = u8BrooksADUSize-1; 
		//Serial.print("Message start ");
		//Serial.println(u8MessageStart);
		StartDetected = true;
	}
	else if(u8BrooksADUSize >= 7 && StartDetected == false)
	{
		u8MBStatus = ku8MBResponseTimedOut;
		break;
	}
	
	//evaluate address, command, byte count
	if (StartDetected && (u8BrooksADUSize-u8MessageStart) >= 8)
    {	
      // verify response is for correct Brooks slave
      if (u8BrooksADU[u8MessageStart+1] != (0b10000000 + _u8MBSlave)) 
      {
        u8MBStatus = ku8MBInvalidSlaveID;
        break;
      }
      
      // verify response is for correct Brooks command
      if (u8BrooksADU[u8MessageStart+2] != _u8Command)
      {
        u8MBStatus = ku8MBInvalidFunction;
        break;
      }
      
      // evaluate Byte count and therefore the end of the message
	  if ((u8BrooksADUSize-u8MessageStart) >= 4 && (u8BrooksADUSize-u8MessageStart) == (u8BrooksADU[u8MessageStart+3] + 5)){
		  u8ResponseByteCount = u8BrooksADU[u8MessageStart+3]; //byte count includes two status bytes but not the checksum
		  //Serial.println(" End of message");
		  //Serial.print("Status 1 ");
		  //Serial.println(u8BrooksADU[u8MessageStart+4]);
		  _u8StatusBuffer[0] = u8BrooksADU[u8MessageStart+4];
		  //Serial.print("Status 2 ");
		  //Serial.println(u8BrooksADU[u8MessageStart+5]);
		  _u8StatusBuffer[1] = u8BrooksADU[u8MessageStart+5];
		  break;
	  }
    }
    if ((millis() - u32StartTime) > ku16MBResponseTimeout)
    {
      u8MBStatus = ku8MBResponseTimedOut;
    }
  }
  
  // verify response is large enough to inspect further
  if (!u8MBStatus && u8BrooksADUSize >= 7)
  {
    // calculate CRC
    u8CRC = 0x00;
    for (i = u8MessageStart; i < (u8BrooksADUSize - 1); i++)
    {
	  //Serial.println(u8BrooksADU[i], HEX);
      u8CRC ^= u8BrooksADU[i];
    }
    //Serial.print("CRC ");
	//Serial.println(u8CRC, HEX);
    // verify CRC
    if (!u8MBStatus && u8CRC != u8BrooksADU[u8BrooksADUSize - 1])
    {
      u8MBStatus = ku8MBInvalidCRC;
    }
  }

  // disassemble ADU into values
  if (!u8MBStatus)
  {
	//Serial.print("data ");
    // evaluate returned Brooks function code
    //switch(_u8Command)
    //{
     // case 1: //read flow
	//  case 236://set flow
	  // load bytes into response buffer 
        for (i = 0; i < (u8ResponseByteCount -2); i++)
        {
          if (i < ku8MaxBufferSize)
          {
            _u8ResponseBuffer[i] = u8BrooksADU[u8MessageStart +6 + i];
			//Serial.println(u8BrooksADU[u8MessageStart +5 + i], HEX);
          }
          
          _u8ResponseBufferLength = i;
        }
		//break;
	//}
  }
  
  _u8TransmitBufferIndex = 0;
  _u16TransmitBufferLength = 0;
  _u8ResponseBufferIndex = 0;
  return u8MBStatus;
}
