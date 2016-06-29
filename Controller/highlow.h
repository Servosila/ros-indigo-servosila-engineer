#ifndef FTL_HIGHLOW_H_INCLUDED
#define FTL_HIGHLOW_H_INCLUDED

namespace ftl
{

inline uint8_t get_high(uint16_t w)
{
    return uint8_t(w>>8);
} //get_high()

inline uint8_t get_low(uint16_t w)
{
    return uint16_t(w<<8)>>8;
} //get_low()

inline void set_high(uint16_t& w, uint8_t h)
{
    //cleaning the bits of the high byte
    w &= uint16_t(255);                 //00000000 11111111
    //setting the bits of the high byte
    w |= uint16_t(uint16_t(h)<<8);
} //set_high()

inline void set_low (uint16_t& w, uint8_t l)
{
    //cleaning the bits of the low byte
    w &= uint16_t(uint16_t(255)<<8);    //11111111 00000000
    //setting the bits of the low byte
    w |= uint16_t(l);
} //set_low()

} //namespace ftl

#endif // FTL_HIGHLOW_H_INCLUDED
