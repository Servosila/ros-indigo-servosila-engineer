#ifndef FTL_CAN_SOCKET_H_INCLUDED
#define FTL_CAN_SOCKET_H_INCLUDED

/*
Encapsulation of SocketCAN API:
    https://en.wikipedia.org/wiki/SocketCAN

Loopback configuration (Virtual CAN):
$ modprobe can
$ modprobe can_raw
$ modprobe vcan
$ sudo ip link add dev vcan0 type vcan
$ sudo ip link set up vcan0
$ ip link show vcan0
3: vcan0: <NOARP,UP,LOWER_UP> mtu 16 qdisc noqueue state UNKNOWN
    link/can
*/

#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>      /* fcntl()      */
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <stdint.h>     /* uint8_t */
#include <errno.h>

namespace network
{

//TODO: Implement Filters
class can_socket
{
private:
    int m_socket_fd; //file descriptor for the socket

public:
    can_socket() : m_socket_fd(-1)
    {
    } //can_socket()

    virtual ~can_socket()
    {
        if (m_socket_fd != -1) shutdown();
    } //~can_socket()

    /*
        Initializes the CANbus socket.
    */
    bool startup(const char* can_interface_name = "vcan0", bool nonblocking = true)
    {
        assert(m_socket_fd == -1);
        bool result = false;
        //creating a socket
        m_socket_fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        //checking result
        if(m_socket_fd != -1)
        {   //interface request data structure
            ifreq ifr;
            //setting up the interface request structure
            strncpy(ifr.ifr_name, can_interface_name, sizeof(ifr.ifr_name));
            //Requesting the interface index by interface name
            const int ir = ::ioctl(m_socket_fd, SIOCGIFINDEX, &ifr);
            //Result code check
            if(ir != -1)
            {   //setting up the address structure
                sockaddr_can addr;
                addr.can_family  = AF_CAN;
                addr.can_ifindex = ifr.ifr_ifindex; //NOTE: Set this to 0 if want to bind it to all interfaces
                //printf("%s at index %d\n", can_interface_name, ifr.ifr_ifindex);
                //binding the socket
                const int br = ::bind(m_socket_fd, (struct sockaddr *)&addr, sizeof(addr));
                if(br != -1)
                {   //configuring the non-blocking behavior if specified
                    if(nonblocking)
                    {   //Set up non-blocking behavior
                        const int fr = ::fcntl(m_socket_fd, F_SETFL, O_NONBLOCK, 1); //No result code check
                        if(fr==-1) assert(false);
                    }
                    //OK
                    result = true;
                }
                else shutdown(); //ERROR
            }
            else shutdown();
        }
        //
        return result;
    } //startup()

    /*
        Deinitializes the CANbus socket.
    */
    void shutdown()
    {
        assert(m_socket_fd != -1);
        const int result = ::close(m_socket_fd);
        if(result<0) assert(false);
        m_socket_fd = -1;
    } //shutdown()

    /*
        Sends a CANbus packet to a specified destination.
    */
    bool send(canid_t destination_can_id, const void* payload, uint8_t payload_size)
    {
        assert(payload_size<=8);
        bool result = false;
        if(is_connected())
        {
            //frame (packet) stucture
            struct can_frame frame;
            //setting the destination CAN ID
            frame.can_id  = destination_can_id;
            //checking payload size
            assert(payload_size<=sizeof(frame.data));
            //copy the payload
            memcpy(frame.data, payload, payload_size);
            //setting payload size
            frame.can_dlc = payload_size;
            //sending the data
            int nbytes = ::write(m_socket_fd, &frame, sizeof(frame));
            if(nbytes != -1)
            {
                //printf("Wrote %d bytes\n", nbytes);
                result = true; //OK
            }
            else
            {   //Error code analysis - looking for USB Device Unplugged situations
                int error = errno;
                switch(error)
                {
                    case ENODEV: //"No such device"
                    {   //USB device unplugged
                        shutdown();
                        break;
                    }
                    case ENXIO: //"No such device or address"
                    {   //USB device unplugged
                        shutdown();
                        break;
                    }
                    default:
                    {   //ignoring all errors except those related to USB Device Unplugged
                        break;
                    }
                } //switch(error)
            } //else
        } //if is connected
        //
        return result;
    } // send()

    /*
        Fetches a CANbus frame
        Blocking or Non-blocking depending on the startup() parameter
    */
    bool receive(void* buffer, size_t buffer_size, uint8_t& bytes_received, canid_t& source_can_id)
    {
        bool result = false;
        if(is_connected())
        {
            //buffer data structure
            can_frame frame;
            //reading a CANbus frame (could be blocking or non-blocking)
            const int nbytes = ::read(m_socket_fd, &frame, sizeof(can_frame));
            assert(nbytes!=0); //no such thing as an empty CANbus frame
            //checking if a frame has been received
            if (nbytes != -1) //if a frame has been fetched
            {   //extracting data from the frame
                source_can_id = frame.can_id;
                bytes_received = frame.can_dlc;
                //checking for buffer overflow
                if(bytes_received<=buffer_size)
                {   //copying the data
                    memcpy(buffer, frame.data, bytes_received);
                    result = true;
                }
            }
            else
            {   //Error code analysis - looking for USB Device Unplugged situations
                int error = errno;
                switch(error)
                {
                    case ENODEV: //"No such device"
                    {   //USB device unplugged
                        shutdown();
                        break;
                    }
                    case ENXIO: //"No such device or address"
                    {   //USB device unplugged
                        shutdown();
                        break;
                    }
                    default:
                    {   //ignoring all errors except those related to USB Device Unplugged situatiopn
                        break;
                    }
                } //switch(error)
            } //else
        } //if is connected
        //
        return result;
    } //receive()

    /*
        Fetches a CANbus frame with Timestamp
            (this is a wrapper around the main function)
        Blocking or Non-blocking depending on the startup() parameter
    */
    bool receive_with_timestamp(void* buffer, size_t buffer_size, uint8_t& bytes_received, canid_t& source_can_id, timeval& timestamp)
    {
        //calling the main function first
        bool result = receive(buffer, buffer_size, bytes_received, source_can_id);
        if(result == true)
        {   //fetching exact timestamp
            const int ir = ::ioctl(m_socket_fd, SIOCGSTAMP, &timestamp); //No result check
            if(ir==-1) assert(false);
        }
        //
        return result;
    } //receive_with_timestamp()

    /*
        0 = disabled (default), 1 = enabled
    */
    bool set_recv_own_msgs_flag(int recv_own_msgs_flag = 1) const
    {
        assert(m_socket_fd != -1);
        const int r = ::setsockopt(m_socket_fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs_flag, sizeof(recv_own_msgs_flag));
        return (r!=-1);
    } //set_recv_own_msgs_flag()

    bool is_connected() const
    {
        return (m_socket_fd != -1);
    } //is_connected()

}; //class can_socket

} //namespace network

#endif // FTL_CAN_SOCKET_H_INCLUDED
