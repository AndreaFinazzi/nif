#pragma once
#include "./HeaderData.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>
#include <iomanip>

namespace tk { namespace data {
    
    class CanData {
        
    public:
        HeaderData  header;                 /**< Header, @see HeaderData */

        struct can_frame frame;
        
        void init() {
            header.init();
        }

        // return integer ID
        canid_t     id()     { return frame.can_id; }
       
        // return data as a uint64_t pointer, useful for DBC encoding
        uint64_t *data() { return reinterpret_cast<uint64_t*>(frame.data); }

        friend std::ostream& operator<<(std::ostream& os, const CanData& m) {
            os << std::setprecision(10) << m.header.stamp<< "\t"<<std::hex<<m.frame.can_id<<std::dec;
            os << "\t"<<std::hex;
            for(int i=0; i<m.frame.can_dlc; i++) {
                os << std::setfill('0') << std::setw(2) << int(m.frame.data[i]);
            }
            os << std::dec;
            return os;
        }
    };
}}
