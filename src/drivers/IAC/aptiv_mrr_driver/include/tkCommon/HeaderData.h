#pragma once

#include <cstdint>

namespace tk { namespace data {

    /**
     * @brief Header data class.
     * Standard metadata for higher-level data class.
     */
    class HeaderData {
    public:
        uint64_t            stamp     = 0; /**< Time stamp, expressed in microseconds. */
        
        void init() {
            this->stamp         = 0;
        }

        /**
         * @brief Overloading of operator = for class copy.
         *
         * @param s
         * @return
         */
        HeaderData &operator=(const HeaderData &s) {
            this->stamp         = s.stamp;
            return *this;
        }
    };
}}
