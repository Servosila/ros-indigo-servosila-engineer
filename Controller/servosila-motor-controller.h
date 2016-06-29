#ifndef DEVICES_SERVOSILA_MOTOR_CONTROLLER_H_INCLUDED
#define DEVICES_SERVOSILA_MOTOR_CONTROLLER_H_INCLUDED

#include "network/canopen.h"
#include "network/cansocket.h"
#include "control/timer.h"

namespace devices
{
//RPDOs
const uint16_t RPDO_SERVOSILA_CHANNEL_FOR_MOTOR_CONTROL         = 0x200; //same for legacy protocol and CANopen implementations
const uint16_t RPDO_SERVOSILA_CHANNEL_FOR_LEGACY_SPEED_CONTROL  = 0x300;
//TPDOs
const uint16_t TPDO_SERVOSILA_CHANNEL_FOR_MOTOR_TELEMETRY_1     = 0x180; //primary telemetry, same for legacy and CANopen implementations
const uint16_t TPDO_SERVOSILA_CHANNEL_FOR_MOTOR_TELEMETRY_2     = 0x280;
const uint16_t TPDO_SERVOSILA_CHANNEL_FOR_MOTOR_TELEMETRY_3     = 0x380;
const uint16_t TPDO_SERVOSILA_CHANNEL_FOR_MOTOR_TELEMETRY_4     = 0x480;
//mask for extracting "fault present" bit from status word (CANopen protocol only)
const uint16_t TELEMETRY_STATUS_FAULT_FLAGS_MASK = 0x7F00;

class servosila_motor_controller
{
public:
    enum protocol_version_t { PROTOCOL_VERSION_LEGACY, PROTOCOL_VERSION_2_0 }; //PROTOCOL_VERSION_1_0 - based on Roboteq implementation
    enum telemetry_state_t  { NO_SHAFT_TELEMETRY, SHAFT_TELEMETRY_COMING };
    enum operation_mode_t   { UNDEFINED_MODE, POSITION_MODE, SPEED_MODE, AMPS_MODE };
private:
    //CANopen Node ID
    uint8_t m_device_id;
    //protocol version
    protocol_version_t m_protocol_version;
    //position encoder availability flag
    bool m_is_position_encoder_available;
    //controller status (state)
    telemetry_state_t m_state; //changes based on telemetry healthcheck timer
    //switch - operation mode
    operation_mode_t  m_operation_mode; //set by commands given by an upper layer application code
    //timers
    control::timer m_rpdo_timer;
    control::timer m_shaft_healthcheck_timer;
private:
    //telemetry
    uint16_t m_position_telemetry;
    int16_t  m_speed_telemetry;
    int16_t  m_amps_telemetry;
    uint16_t m_status_telemetry;
    uint16_t m_faults_telemetry; //legacy protocol only
    //faults and warnings
    size_t   m_fault_ack_counter; //2.0 protocol only
public:
    //Position data
    uint16_t m_min_position_limit;
    uint16_t m_max_position_limit;
    uint16_t m_position_command;
    //Speed data
    int16_t  m_min_speed_limit;
    int16_t  m_max_speed_limit;
    int16_t  m_speed_command;
    //Current-Torque data
    int16_t m_min_amps_limit;
    int16_t m_max_amps_limit;
    int16_t m_amps_command;

public:
    servosila_motor_controller()
        :   m_device_id(0),
            m_protocol_version(PROTOCOL_VERSION_2_0),
            m_is_position_encoder_available(false),
            //
            m_state(NO_SHAFT_TELEMETRY),
            m_operation_mode(UNDEFINED_MODE),
            m_rpdo_timer(0),
            m_shaft_healthcheck_timer(0),
            //telemetry
            m_position_telemetry(0),
            m_speed_telemetry(0),
            m_amps_telemetry(0),
            m_status_telemetry(0),
            m_faults_telemetry(0),
            m_fault_ack_counter(0),
            //Position data
            m_min_position_limit(0),
            m_max_position_limit(0),
            m_position_command(0),
            //Speed data
            m_min_speed_limit(0),
            m_max_speed_limit(0),
            m_speed_command(0),
            //Current-Torque data
            m_min_amps_limit(0),
            m_max_amps_limit(0),
            m_amps_command(0)
    {
    }//servosila_motor_controller()

    ~servosila_motor_controller()
    {
        _reset_to_initial_state(); //just in case
    }

    void configure( uint8_t  device_id,
                    protocol_version_t protocol_version,
                    bool     position_encoder_available, //chassis drives don't have an encoder
                    control::usec_t rpdo_timeout,
                    control::usec_t shaft_telemetry_healthcheck_timeout,
                    uint16_t min_position_limit,
                    uint16_t max_position_limit,
                    int16_t  min_speed_limit,
                    int16_t  max_speed_limit,
                    int16_t  min_amps_limit,
                    int16_t  max_amps_limit
                  )
    {
        m_device_id = device_id;
        m_protocol_version = protocol_version;
        m_is_position_encoder_available = position_encoder_available;
        //timers
        m_rpdo_timer.configure(rpdo_timeout);
        m_shaft_healthcheck_timer.configure(shaft_telemetry_healthcheck_timeout);
        //position mode configuration
        m_min_position_limit = min_position_limit;
        m_max_position_limit = max_position_limit;
        m_position_telemetry = min_position_limit;  //just in case, in fact the telemetry value is undefined on start
        //speed mode configuration
        m_min_speed_limit = min_speed_limit;
        m_max_speed_limit = max_speed_limit;
        //amps mode configuration
        m_min_amps_limit = min_amps_limit;
        m_max_amps_limit = max_amps_limit;
        //
    } //configure()

    void change_timeouts(control::usec_t rpdo_timeout, control::usec_t shaft_telemetry_healthcheck_timeout)
    {   //configure timers
        m_rpdo_timer.configure(rpdo_timeout);
        m_shaft_healthcheck_timer.configure(shaft_telemetry_healthcheck_timeout);
    } //configure()

    telemetry_state_t get_state() const
    {
        return m_state;
    } //get_telemetry_state()

    operation_mode_t get_operation_mode() const
    {
        return m_operation_mode;
    } //get_operation_mode()

    bool is_operational() const
    {
        return (m_state==SHAFT_TELEMETRY_COMING) && (m_device_id != 0);
    }

    bool is_position_encoder_available() const
    {
        return m_is_position_encoder_available;
    } //is_position_encoder_available()

    size_t get_faults_ack_counter() const
    {
        return m_fault_ack_counter;
    } //get_faults_ack_counter()

    void execute(network::can_socket& can)
    {
        //Reaction to CANbus problems
        if(!can.is_connected())
        {   //sets both NO_TELEMETRY and UNDEFINED_MODE
            _reset_to_initial_state();
        }

        //Healthcheck timer verification
        if(m_shaft_healthcheck_timer.check()) //it is reset in process_canbus_callback() routine
        {   //TELEMETRY TIMEOUT! - handling a healthcheck problem
            //if the telemetry state indicates "there is a connection"
            if(m_state==SHAFT_TELEMETRY_COMING)
            {   //sets NO_TELEMETRY and UNDEFINED_MODE
                _reset_to_initial_state();
            }
        }

        //Sending out RPDO commands
        //...if time has come to send an RPDO
        if(m_rpdo_timer.check_and_restart())
        {   //...sending only when TELEMETRY_COMING
            if(m_state==SHAFT_TELEMETRY_COMING)
            {   //Sending out an RPDO frame
                if(can.is_connected()) _send_rpdo_as_per_current_operation_mode(can);
            }
        }
    } //execute()

    bool process_canbus_callback(network::can_socket& can, const uint8_t* buffer, uint8_t bytes_received, canid_t source_can_id, timeval /*timestamp*/)
    {   //has the message been processed?
        bool is_processed_flag = false;
        //Extrating Node ID
        const uint8_t node_id = network::canopen::extract_node_id_from_cob_id(source_can_id);
        //Filter
        if(node_id==m_device_id)
        {   //Extrating Function Code
            const uint16_t function_code = network::canopen::extract_function_code_from_cob_id(source_can_id);
            //process telemetry frames
            switch(function_code)
            {
                case TPDO_SERVOSILA_CHANNEL_FOR_MOTOR_TELEMETRY_1:
                {   //extracting telemetry values
                    _parse_tpdo1(buffer, bytes_received);
                    //reacting on fault bits in status word
                    _process_faults(can);
                    //Healthcheck timer reset
                    m_shaft_healthcheck_timer.restart(); //good health
                    //setting the status - on any incoming frame
                    m_state = SHAFT_TELEMETRY_COMING;
                    //setting result
                    is_processed_flag = true;
                    break;
                }
                case TPDO_SERVOSILA_CHANNEL_FOR_MOTOR_TELEMETRY_2:
                {   //extracting telemetry values
                    _parse_tpdo2(buffer, bytes_received);
                    //setting result
                    is_processed_flag = true;
                    break;
                }
                case TPDO_SERVOSILA_CHANNEL_FOR_MOTOR_TELEMETRY_3:
                {   //extracting telemetry values
                    _parse_tpdo3(buffer, bytes_received);
                    //setting result
                    is_processed_flag = true;
                    break;
                }
                case TPDO_SERVOSILA_CHANNEL_FOR_MOTOR_TELEMETRY_4:
                {   //extracting telemetry values
                    //TODO: ...
                    //setting result
                    is_processed_flag = true;
                    break;
                }
                default:
                {   //ignore all other/unknown/non-relevant function codes
                    break;
                }
            } //switch()
            //
        } //end of filter by Node ID
        //
        return is_processed_flag;
    } //process_canbus_frame_()

    void set_position_command(uint16_t position)
    {
        assert(position<=m_max_position_limit);
        assert(position>=m_min_position_limit);
        //
        m_operation_mode = POSITION_MODE;
        m_position_command = position;

        //std::cout<<"Position command: "<<m_position_command<<std::endl;
    } //set_position_command()

    void set_speed_command(int16_t speed)
    {
        assert(speed<=m_max_speed_limit);
        assert(speed>=m_min_speed_limit);
        //
        m_operation_mode = SPEED_MODE;
        m_speed_command = speed;
    } //set_speed_command()

    void set_amps_command(int16_t amps)
    {
        assert(amps<=m_max_amps_limit);
        assert(amps>=m_min_amps_limit);
        //
        m_operation_mode = AMPS_MODE;
        m_amps_command = amps;
    } //set_amps_command()

    void set_undefined_command()
    {
        m_operation_mode = UNDEFINED_MODE;
    } //set_amps_command()

    /*
        Emergency Stop Routine
        - tries to stop the motor under all circumstances
    */
    void halt(network::can_socket& can)
    {
        if(m_state == SHAFT_TELEMETRY_COMING)
        {   //normal situation - telemetry is coming, easy to halt
            switch(m_operation_mode)
            {
                case POSITION_MODE:
                {   //setting the position to the latest position telemetry
                    set_position_command(m_position_telemetry);
                    break;
                }
                case SPEED_MODE:
                {   //zero speed
                    set_speed_command(0);
                    break;
                }
                case AMPS_MODE:
                {   //zero amps/torque
                    set_amps_command(0);
                    break;
                }
                case UNDEFINED_MODE:
                {   //workaround - since UNDEFINED_MODE
                    //...normally, halt() shall not be called in UNDEFINED_MODE since the motor is not moving anyway
                    //...but this might happen after an abdrupt process reboot
                    //...when the process has just restarted, but the motor is still moving
                    //do nothing .. hope that the motor will timeout
                    break;
                }
                default:
                {   //unknown state
                    assert(false);
                    break;
                }
            } //switch()
        }
        else //NO SHAFT TELEMETRY
        {   //workaround - since NO SHAFT TELEMETRY
            //..might happen after a process reboot
            set_undefined_command(); //this stops RPDO sending
            //do nothing .. hope that the motor will timeout
        } //if SHAFT TELEMETRY IS COMING
        //
        //AT LEAST ONCE: forcefully sending out the command - just in case TELEMETRY IS NOT COMING
        if(can.is_connected())
        {   //sending at least once - since the mode can be switched due to no telemetry
            _send_rpdo_as_per_current_operation_mode(can);
        }
        else
        {   //cannot stop the motor if CANbus is not connected :-(
        }
        //
    }//halt()

    uint16_t get_position_telemetry() const
    {
        assert(m_state == SHAFT_TELEMETRY_COMING);
        return m_position_telemetry;
    }

    int16_t get_speed_telemetry() const
    {
        assert(m_state == SHAFT_TELEMETRY_COMING);
        return m_speed_telemetry;
    }

    int16_t get_amps_telemetry() const
    {
        assert(m_state == SHAFT_TELEMETRY_COMING);
        return m_amps_telemetry;
    }

    uint16_t get_status_telemetry() const
    {
        assert(m_state == SHAFT_TELEMETRY_COMING);
        return m_status_telemetry;
    }

    size_t get_fault_ack_counter() const
    {
        return m_fault_ack_counter;
    }

    uint8_t get_device_id() const
    {
        return m_device_id;
    }

private:

    //helper function
    void _reset_to_initial_state()
    {
        //stop sending RPDOs
        m_operation_mode = UNDEFINED_MODE;
        //waiting for telemetry to come
        m_state = NO_SHAFT_TELEMETRY; //setting the state to "no connection"
        //resetting fault statistics
        m_fault_ack_counter = 0;
    }

    //helper function - version router
    void _send_rpdo_as_per_current_operation_mode(network::can_socket& can) const
    {
        assert(can.is_connected());
        assert(m_device_id != 0);
        //depending on the version, call the right helper fucntion
        switch(m_protocol_version)
        {
            case PROTOCOL_VERSION_LEGACY:
            {   //legacy
                _send_rpdo_as_per_current_operation_mode_legacy_protocol(can);
                break;
            }
            case PROTOCOL_VERSION_2_0:
            {   //canopen
                _send_rpdo_as_per_current_operation_mode_protocol_2_0(can);
                break;
            }
            default:
            {   //unknown protocol version
                assert(false);
                break;
            }
        }
    } //_send_rpdo_as_per_current_operation_mode()

    //helper function
    void _send_rpdo_as_per_current_operation_mode_protocol_2_0(network::can_socket& can) const
    {
        assert(m_protocol_version == PROTOCOL_VERSION_2_0);
        assert(can.is_connected());
        assert(m_device_id != 0);
        switch(m_operation_mode)
        {
            case UNDEFINED_MODE:
            {   //don't send any RPDO command
                break;
            }
            case POSITION_MODE:
            {   //device protocol-specific constant
                const uint16_t RPDO_COMMAND_POSITION            = 0x0021;
                const uint8_t  RPDO_POSITION_OFFSET_IN_PAYLOAD  = 2;
                const uint16_t position = m_position_command;
                //sending
                network::canopen::send_expedited_rpdo(can, m_device_id, RPDO_SERVOSILA_CHANNEL_FOR_MOTOR_CONTROL, RPDO_COMMAND_POSITION, RPDO_POSITION_OFFSET_IN_PAYLOAD, position);
                //
                break;
            }
            case SPEED_MODE:
            {   //device protocol-specific constant
                const uint16_t RPDO_COMMAND_SPEED            = 0x0005;
                const uint8_t  RPDO_SPEED_OFFSET_IN_PAYLOAD  = 4;
                const uint16_t speed = m_speed_command;
                //sending
                network::canopen::send_expedited_rpdo(can, m_device_id, RPDO_SERVOSILA_CHANNEL_FOR_MOTOR_CONTROL, RPDO_COMMAND_SPEED, RPDO_SPEED_OFFSET_IN_PAYLOAD, speed);
                //
                break;
            }
            case AMPS_MODE:
            {   //device protocol-specific constant
                const uint16_t RPDO_COMMAND_AMPS            = 0x0001;
                const uint8_t  RPDO_AMPS_OFFSET_IN_PAYLOAD  = 6;
                const uint16_t amps = m_amps_command;
                //sending
                network::canopen::send_expedited_rpdo(can, m_device_id, RPDO_SERVOSILA_CHANNEL_FOR_MOTOR_CONTROL, RPDO_COMMAND_AMPS, RPDO_AMPS_OFFSET_IN_PAYLOAD, amps);
                //
                break;
            }
            default:
            {
                assert(false);
                break;
            }
        }//switch()
    } //_send_rpdo_as_per_current_operation_mode_protocol_2_0()

    //helper function
    void _send_rpdo_as_per_current_operation_mode_legacy_protocol(network::can_socket& can) const
    {
        assert(m_protocol_version == PROTOCOL_VERSION_LEGACY);
        assert(can.is_connected());
        assert(m_device_id != 0);

        switch(m_operation_mode)
        {
            case UNDEFINED_MODE:
            {   //don't send any RPDO command
                break;
            }
            case POSITION_MODE:
            {
                uint8_t command[8] = {0,0,0,0,0,0,0,0};
                memcpy(&command, &m_position_command, sizeof(m_position_command));
                command[4] = m_device_id; //workaround for a ROBOTEQ bug
                //can.send(RPDO_SERVOSILA_CHANNEL_FOR_MOTOR_CONTROL+m_device_id, &m_position_command, sizeof(m_position_command));
                can.send(RPDO_SERVOSILA_CHANNEL_FOR_MOTOR_CONTROL+m_device_id, &command, sizeof(command));
                //
                //std::cout<<"Position command: "<<m_position_command<<std::endl;
                break;
            }
            case SPEED_MODE:
            {
                //determine handling by the type of the drive - chassis drive or servo drive
                if(m_is_position_encoder_available)
                {
                    //Regular Servo Motors (not Chassis Drives) in Speed Mode
                    uint8_t command[8] = {0,0,0,0,0,0,0,0};
                    memcpy(&command, &m_speed_command, sizeof(m_speed_command));
                    command[4] = m_device_id; //workaround for a ROBOTEQ bug
                    //can.send(RPDO_SERVOSILA_CHANNEL_FOR_LEGACY_SPEED_CONTROL+m_device_id, &m_speed_command, sizeof(m_speed_command));
                    can.send(RPDO_SERVOSILA_CHANNEL_FOR_LEGACY_SPEED_CONTROL+m_device_id, &command, sizeof(command));
                }
                else
                {   //Chassis Drive Motors
                    uint8_t command[8] = {0,0,0,0,0,0,0,0};
                    memcpy(&command, &m_speed_command, sizeof(m_speed_command));
                    command[4] = m_device_id; //workaround for a ROBOTEQ bug
                    //can.send(RPDO_SERVOSILA_CHANNEL_FOR_MOTOR_CONTROL+m_device_id, &m_speed_command, sizeof(m_speed_command));
                    can.send(RPDO_SERVOSILA_CHANNEL_FOR_MOTOR_CONTROL+m_device_id, &command, sizeof(command));
                }

                //
                break;
            }
            case AMPS_MODE:
            {
                assert(false); //not supported
                //
                break;
            }
            default:
            {   //unknown mode
                assert(false);
                break;
            }
        }//switch()
    } //_send_rpdo_as_per_current_operation_mode_legacy_protocol()

    //helper function - version router
    void _parse_tpdo1(const uint8_t* buffer, uint8_t bytes_received)
    {
        //depending on the version, call the right helper fucntion
        switch(m_protocol_version)
        {
            case PROTOCOL_VERSION_LEGACY:
            {   //legacy
                _parse_tpdo1_legacy_protocol(buffer, bytes_received);
                break;
            }
            case PROTOCOL_VERSION_2_0:
            {   //canopen
                _parse_tpdo1_protocol_2_0(buffer, bytes_received);
                break;
            }
            default:
            {   //unknown protocol version
                assert(false);
                break;
            }
        }
    } //_parse_tpdo1()

    //helper function
    void _parse_tpdo1_legacy_protocol(const uint8_t* buffer, uint8_t bytes_received)
    {
        assert(m_protocol_version == PROTOCOL_VERSION_LEGACY);
        //
        if(bytes_received == 8) //TPDO frames size
        {
            //determine telemetry type by the type of the drive
            if(m_is_position_encoder_available)
            {
                uint16_t position = 0;
                memcpy(&position,   &(buffer[4]), sizeof(position));
                m_position_telemetry = position;
            }
            else
            {
                int16_t speed = 0;
                memcpy(&speed,   &(buffer[4]), sizeof(speed));
                m_speed_telemetry = speed;
            }
        }
    } //_parse_tpdo1_legacy_protocol()

    //helper function
    void _parse_tpdo1_protocol_2_0(const uint8_t* buffer, uint8_t bytes_received)
    {
        assert(m_protocol_version == PROTOCOL_VERSION_2_0);
        //
        if(bytes_received == 8) //TPDO frames size
        {
            uint16_t status = 0;
            uint16_t position = 0;
            int16_t  speed = 0;
            int16_t  amps = 0;


            //TPDO, 0x180, UINT16,Offset 0, Status Word
            //TPDO, 0x180, UINT16,Offset 2, Position
            //TPDO, 0x180, INT16, Offset 4, Speed
            //TPDO, 0x180, INT16, Offset 6, Amps
            //Extracting
            memcpy(&status,   &(buffer[0]), sizeof(status));
            memcpy(&position, &(buffer[2]), sizeof(position));
            memcpy(&speed,    &(buffer[4]), sizeof(speed));
            memcpy(&amps,     &(buffer[6]), sizeof(amps));
            //Setting
            m_status_telemetry   = status;
            m_position_telemetry = position;
            m_speed_telemetry    = speed;
            m_amps_telemetry     = amps;
        }
    } //_parse_tpdo1_protocol_2_0()


    //helper function - version router
    void _parse_tpdo2(const uint8_t* buffer, uint8_t bytes_received)
    {
        //depending on the version, call the right helper fucntion
        switch(m_protocol_version)
        {
            case PROTOCOL_VERSION_LEGACY:
            {   //legacy
                _parse_tpdo2_legacy_protocol(buffer, bytes_received);
                break;
            }
            case PROTOCOL_VERSION_2_0:
            {   //canopen
                break;
            }
            default:
            {   //unknown protocol version
                assert(false);
                break;
            }
        }
    } //_parse_tpdo1()

    //helper function
    void _parse_tpdo2_legacy_protocol(const uint8_t* buffer, uint8_t bytes_received)
    {
        assert(m_protocol_version == PROTOCOL_VERSION_LEGACY);
        //
        if(bytes_received == 8) //TPDO frames size
        {
            //determine telemetry type by the type of the drive
            if(m_is_position_encoder_available)
            {
                int16_t speed = 0;
                memcpy(&speed,   &(buffer[4]), sizeof(speed));
                m_speed_telemetry = speed;
            }
            else
            {   //chassis drives don't send this kind of TPDO
                assert(false);
            }
        }
    } //_parse_tpdo2_legacy_protocol()

    //helper function - version router
    void _parse_tpdo3(const uint8_t* buffer, uint8_t bytes_received)
    {
        //depending on the version, call the right helper fucntion
        switch(m_protocol_version)
        {
            case PROTOCOL_VERSION_LEGACY:
            {   //legacy
                _parse_tpdo3_legacy_protocol(buffer, bytes_received);
                break;
            }
            case PROTOCOL_VERSION_2_0:
            {   //canopen
                break;
            }
            default:
            {   //unknown protocol version
                assert(false);
                break;
            }
        }
    } //_parse_tpdo3()

    //helper function
    void _parse_tpdo3_legacy_protocol(const uint8_t* buffer, uint8_t bytes_received)
    {
        assert(m_protocol_version == PROTOCOL_VERSION_LEGACY);
        //
        if(bytes_received == 8) //TPDO frames size
        {
            uint16_t fault_and_status = 0;
            memcpy(&fault_and_status,   &(buffer[0]), sizeof(fault_and_status));
            m_faults_telemetry = fault_and_status;
        }
    } //_parse_tpdo3_legacy_protocol()


    //helper - version router
    void _process_faults(network::can_socket& can)
    {
        switch(m_protocol_version)
        {
            case PROTOCOL_VERSION_2_0:
            {
                _process_faults_protocol_2_0(can);
                break;
            }
            case PROTOCOL_VERSION_LEGACY:
            {   //no faults handling in the legacy protocol
                break;
            }
            default:
            {   //unknown protocol version
                assert(false);
            }
        } //switch()
    } //_process_faults()

    //helper function
    void _process_faults_protocol_2_0(network::can_socket& can)
    {
        //applying the bit mask: computing the fault flags
        const uint16_t fault_flags = m_status_telemetry & TELEMETRY_STATUS_FAULT_FLAGS_MASK;
        //fault handling logic
        if(fault_flags!=0)
        {
            if(can.is_connected())
            {   //automatically send ACK to all FAULTs
                _send_fault_ack(can, m_device_id);
                //increment fault counter
                m_fault_ack_counter++;
                //log the fault here
            }
        }
    } //_process_faults_protocol_2_0()

    //helper function
    void _send_fault_ack(network::can_socket& can, uint8_t device_id)
    {
        assert(m_protocol_version == PROTOCOL_VERSION_2_0);
        //device protocol-specific constant
        const uint16_t RPDO_COMMAND_FAULT_ACK = 0x0002;
        const uint8_t  RPDO_FAULT_ACK_OFFSET_IN_PAYLOAD  = 2;
        const uint8_t  dummy_fault_ack_command = 0;
        //sending out RPDO command
        network::canopen::send_expedited_rpdo(can, device_id, RPDO_SERVOSILA_CHANNEL_FOR_MOTOR_CONTROL, RPDO_COMMAND_FAULT_ACK, RPDO_FAULT_ACK_OFFSET_IN_PAYLOAD, dummy_fault_ack_command);
    }

}; //class servosila_motor_controller

} //namespace devices

#endif // DEVICES_SERVOSILA_MOTOR_CONTROLLER_H_INCLUDED
