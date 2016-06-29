#ifndef CANOPEN_H_INCLUDED
#define CANOPEN_H_INCLUDED

#include "network/cansocket.h"
#include "ftl/highlow.h"

namespace network
{

namespace canopen
{
/*
https://en.wikipedia.org/wiki/CANopen#Service_Data_Object_.28SDO.29_protocol

The pre-defined connection set defines an SDO channel which can be used
even just after bootup (in the Pre-operational state) to configure the device.
The COB-IDs of this channel are
            0x600 + node ID for receiving and
            0x580 + node ID for transmitting.

Payload structure:
|--------------------------------------------|-----------|------------|------------|
|                  Byte 1 (Command byte)     |  Byte 2-3 |   Byte 4   |  Byte 5-8  |
|--------------------------------------------|-----------|------------|------------|
| 3 bits  1 bit         2 bits  1 bit  1 bit |  2 bytes  |   1 byte   |	 4 bytes   |
|--------------------------------------------|-----------|------------|------------|
| ccs=1   reserved(=0)  n       e      s     |   index   |  subindex  |    data    |
|--------------------------------------------|-----------|------------|------------|

  ccs= 001 - means "initiating download" - this is from device point of view,and really means "setting the value"
     = 010 - for initiating upload (= reading)
  r  =   0 - reserved
  n  =  00 - is the number of bytes in the data part of the message which do not contain data, only valid if e and s are set
  e  =   1 - if set, indicates an expedited transfer, i.e. all data exchanged are contained within the message.
  s  =   1 - if set, indicates that the data size is specified in n (if e is set) or in the data part of the message

Command byte (WRITE):
        cs  r n  e s
        001 0 00 1 1 = 0x23  //4byte payload
        001 0 10 1 1 = 0x2B  //2byte payload
        001 0 11 1 1 = 0x2F  //1byte payload

Command byte (READ):
        cs  r n  e s
        010 0 00 1 1 = 0x43  //4byte payload
        010 0 10 1 1 = 0x4B  //2byte payload
        010 0 11 1 1 = 0x4F  //1byte payload

*/

//payload command bytes - WRITE
const uint8_t WRITE_COMMAND_BYTE_4_BYTES_PAYLOAD  = 0x23; //length 4bytes
const uint8_t WRITE_COMMAND_BYTE_2_BYTES_PAYLOAD  = 0x2B; //length 2bytes
const uint8_t WRITE_COMMAND_BYTE_1_BYTES_PAYLOAD  = 0x2F; //length 1bytes
//payload command bytes - READ
const uint8_t READ_COMMAND_BYTE_4_BYTES_PAYLOAD  = 0x43; //length 4bytes
const uint8_t READ_COMMAND_BYTE_2_BYTES_PAYLOAD  = 0x4B; //length 2bytes
const uint8_t READ_COMMAND_BYTE_1_BYTES_PAYLOAD  = 0x4F; //length 1bytes


//predefined channels
//these channels work in all states of the CANopen device state machine
const uint16_t PREDEFINED_SDO_CHANNEL           = 0x600;
const uint16_t PREDEFINED_SDO_RESPONSE_CHANNEL  = 0x580;

const uint16_t CHANNEL_MASK = 0x780;

/*
    Write into a device property
    "Expedited" means that complete payload fits into a single message
*/
template <class datatype>
inline bool send_expedited_sdo_write (network::can_socket& _can, uint8_t node_id, uint16_t index, uint8_t subindex, datatype data)
{   //CANopen allows only payloads of size 1, 2 or 4 bytes long
    assert((sizeof(data)==1) || (sizeof(data)==2) || (sizeof(data)==4));
    //buffer
    uint8_t payload[8] = {0,0,0,0,0,0,0,0}; //CANopen mandates 8bytes-long payload
    //setting the command byte depending on the size of the datatype
    switch(sizeof(data))
    {
        case 4: payload[0] = WRITE_COMMAND_BYTE_4_BYTES_PAYLOAD; break;
        case 2: payload[0] = WRITE_COMMAND_BYTE_2_BYTES_PAYLOAD; break;
        case 1: payload[0] = WRITE_COMMAND_BYTE_1_BYTES_PAYLOAD; break;
        default: assert(false); break;
    }
    //setting the index - lowest byte first
    payload[1] = ftl::get_low(index);
    payload[2] = ftl::get_high(index);
    //setting the subindex
    payload[3] = subindex;
    //copy the data value
    assert(sizeof(data) <= 4);
    ::memcpy(&(payload[4]), &data, sizeof(data)); //PORTABILITY ISSUE: this is not portable since the order of bytes can be different on non-Intel-like platforms
    //sending out
    return _can.send(PREDEFINED_SDO_CHANNEL+node_id, payload, sizeof(payload)); //CANopen mandates 8bytes-long payload
} //canopen_send_expedited_sdo


/*
    Send a request to read from a device property
    "Expedited" means that complete payload fits into a single message
*/
inline bool send_expedited_sdo_read (network::can_socket& _can, uint8_t node_id, uint16_t index, uint8_t subindex, uint8_t expected_data_size)
{   //CANopen allows only payloads of size 1, 2 or 4 bytes long
    assert((expected_data_size==1) || (expected_data_size==2) || (expected_data_size==4));
    //buffer
    uint8_t payload[8] = {0,0,0,0,0,0,0,0}; //CANopen mandates 8bytes-long payload
    //setting the command byte depending on the size of the datatype
    switch(expected_data_size)
    {
        case 4: payload[0] = READ_COMMAND_BYTE_4_BYTES_PAYLOAD; break;
        case 2: payload[0] = READ_COMMAND_BYTE_2_BYTES_PAYLOAD; break;
        case 1: payload[0] = READ_COMMAND_BYTE_1_BYTES_PAYLOAD; break;
        default: assert(false); break;
    }
    //setting the index - lowest byte first
    payload[1] = ftl::get_low(index);
    payload[2] = ftl::get_high(index);
    //setting the subindex
    payload[3] = subindex;
    //sending out
    return _can.send(PREDEFINED_SDO_CHANNEL+node_id, payload, sizeof(payload)); //CANopen mandates 8bytes-long payload
} //canopen_send_expedited_sdo

template <class datatype>
inline bool send_expedited_rpdo (network::can_socket& _can, uint8_t node_id, uint16_t channel, uint16_t command, uint8_t offset, datatype data)
{   //CANopen allows only payloads of size 1, 2 or 4 bytes long
    assert((sizeof(data)==1) || (sizeof(data)==2) || (sizeof(data)==4));
    //buffer
    uint8_t payload[8] = {0,0,0,0,0,0,0,0}; //CANopen mandates 8bytes-long payload
    assert(offset<sizeof(payload));
    //setting the command field
    assert(sizeof(command)==2);
    ::memcpy(&(payload[0]), &command, sizeof(command)); //PORTABILITY ISSUE: this is not portable since the order of bytes can be different on non-Intel-like platforms

    //copy the data value
    assert(offset<sizeof(payload));
    ::memcpy(&(payload[offset]), &data, sizeof(data)); //PORTABILITY ISSUE: this is not portable since the order of bytes can be different on non-Intel-like platforms
    //sending out
    return _can.send(channel+node_id, payload, sizeof(payload)); //CANopen mandates 8bytes-long payload
} //send_expedited_rpdo

/*
    Extract Node ID from COB ID
    COB_ID = 4bits Function Code, 7bits Node ID
*/
inline uint8_t extract_node_id_from_cob_id(uint16_t cob_id)
{
    //assert(cob_id<=2047); //11bit only
    const uint8_t node_id = cob_id & 127; //0000-1111111
    return node_id;
} //extract_node_id_from_cob_id()

/*
    Extract Function Code from COB ID
    COB_ID = 4bits Function Code, 7bits Node ID
*/
inline uint16_t extract_function_code_from_cob_id(uint16_t cob_id)
{
    assert(cob_id<=2047); //11bit only
    const uint16_t function_code = cob_id & 0x780; //1111-0000000
    return function_code;
} //extract_function_code_from_cob_id()

/*
*/
inline uint16_t extract_index_from_payload(uint8_t payload[8])
{
    const uint16_t index = *((uint16_t*)&(payload[1]));
    return index;
} //extract_index_from_payload()


/*
*/
inline uint8_t extract_subindex_from_payload(uint8_t payload[8])
{
    const uint8_t subindex = payload[3];
    return subindex;
} //extract_subindex_from_payload()


} //namespace canopen

} //namespace network

#endif // CANOPEN_H_INCLUDED
