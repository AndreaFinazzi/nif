#pragma once
#include "./CanData.h"
#include <fstream>

namespace tk { namespace communication {
    class CanInterface {
    public:
        CanInterface();
        ~CanInterface();

        //Init
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * Method that create a recive socket
         *
         * @param port      SocketCan port
         * @return          Success
         */
        bool initSocket(const std::string port = "can0");

        /**
         * Method that init read from log file
         *
         * @param fileName  Saving file name
         * @param filter    Filter on recorder, default empty
         * @return          Success
         */
        bool initFile(const std::string fileName);


        //Reading
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * Method that return a packet
         * @param can_frame Packet data
         * @return          Packet lenght
         */
        int read(tk::data::CanData *data);

        /**
         * Write frem on socket, works only in online mode
         * @param frame
         * @return
         */
        bool write (tk::data::CanData *data);

        //Closing
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /**
         * Method that stop the recorder or close the replay file or socket
         * @return          Success
         */
        bool close();


    private:
        bool    offlineMode;
        int     soc = -1;
        std::ifstream                   logstream;
    };
}}