/**
 * @brief a quick utility to convert trajectories between frames
 **/

// Application
#include "bvs_utils/geodetic_conv.h"

// STD
#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <iomanip>

enum class Frame {
    FLU,
    NED,
    GEO,
    UNSET
};

std::string
frameString(Frame frame) {
    if(frame == Frame::FLU) return "FLU";
    if(frame == Frame::NED) return "NED";
    if(frame == Frame::GEO) return "GEO";
    if(frame == Frame::UNSET) return "UNSET";
    return "UNKNOWN";
}

using bvs_utils::GeodeticConverter;

int main(int argc, char** argv) {
    bool ltp_lat_set = false;
    bool ltp_lon_set = false;
    bool ltp_alt_set = true;
    double ltp_lat = 0.;
    double ltp_lon = 0.;
    double ltp_alt = 0.;
    Frame input_frame = Frame::UNSET;
    Frame output_frame = Frame::UNSET;
    std::string input_file_path = "";
    bool input_file_path_set = false;
    std::string output_file_path = "";
    bool output_file_path_set = false;

    // Process arguments... (I should prob figure out a more succinct way to do this)
    for(int i = 0; i < argc; ++i) {
        if(std::string(argv[i]) == "-h" || std::string(argv[i]) == "--help") {
            std::cout << "path_converter <input frame> <output frame> [-lat <lat>] [-lon <lon>] [-alt <alt>] -i <input file> -o <output file>" << std::endl;
            std::cout << "Used to convert paths between frames." << std::endl;
            std::cout << "Argument Details::" << std::endl;
            std::cout << "    -h, --help : shows this help message" << std::endl << std::endl;
            std::cout << "    -lat <latitude of LTP origin> : Set the latitude of the LTP origin" << std::endl;
            std::cout << "    -lot <longitude of LTP origin> : Set the longitude of the LTP origin" << std::endl;
            std::cout << "    -alt <altitude of LTP origin> : Set the altitude of the LTP origin (default 0.)" << std::endl;
            std::cout << "  Input Frame Selectors:" << std::endl;
            std::cout << "    -iflu : specify that the input path is in the FLU frame" << std::endl;
            std::cout << "    -ined : specify that the input path is in the NED frame" << std::endl;
            std::cout << "    -igeo : specify that the input path are in geodetic coordinates" << std::endl;
            std::cout << "  Output Frame Selectors:" << std::endl;
            std::cout << "    -oflu : specify that the target output is the FLU frame" << std::endl;
            std::cout << "    -oned : specify that the target output is the NED frame" << std::endl;
            std::cout << "    -ogeo : specify that the target path are in geodetic coordinates" << std::endl << std::endl;
            std::cout << "    -i <path to csv> : the path to the input csv path" << std::endl;
            std::cout << "    -o <path to csv> : the path to write the output csv to" << std::endl << std::endl;
            std::cout << "NOTE: the LTP origin for the NED, LTP frames should be specificed with -lat, -lon, -alt" << std::endl;
            return 0;
        }
        if(std::string(argv[i]) == "-iflu") input_frame = Frame::FLU;
        if(std::string(argv[i]) == "-ined") input_frame = Frame::NED;
        if(std::string(argv[i]) == "-igeo") input_frame = Frame::GEO;
        if(std::string(argv[i]) == "-oflu") output_frame = Frame::FLU;
        if(std::string(argv[i]) == "-oned") output_frame = Frame::NED;
        if(std::string(argv[i]) == "-ogeo") output_frame = Frame::GEO;

        if(std::string(argv[i]) == "-lat") {
            if(i + 1 >= argc) {
                throw std::out_of_range("-lat argument expects a float argument after.");
            }
            ltp_lat = std::stod(std::string(argv[i + 1]));
            ltp_lat_set = true;
        }
        if(std::string(argv[i]) == "-lon") {
            if(i + 1 >= argc) {
                throw std::out_of_range("-lon argument expects a float argument after.");
            }
            ltp_lon = std::stod(std::string(argv[i + 1]));
            ltp_lon_set = true;
        }
        if(std::string(argv[i]) == "-alt") {
            if(i + 1 >= argc) {
                throw std::out_of_range("-alt argument expects a float argument after.");
            }
            ltp_alt = std::stod(std::string(argv[i + 1]));
            ltp_alt_set = true;
        }
        if(std::string(argv[i]) == "-i") {
            if(i + 1 >= argc) {
                throw std::out_of_range("-i argument expects a string argument after.");
            }
            input_file_path = std::string(argv[i + 1]);
            input_file_path_set = true;
        }
        if(std::string(argv[i]) == "-o") {
            if(i + 1 >= argc) {
                throw std::out_of_range("-o argument expects a string argument after.");
            }
            output_file_path = std::string(argv[i + 1]);
            output_file_path_set = true;
        }
    }

    // Verify arguments are good
    bool arg_error = false;
    if(!output_file_path_set) {
        std::cerr << "Error: output file path must be set by -o flag." << std::endl;
        arg_error = true;
    }
    if(!input_file_path_set) {
        std::cerr << "Error: input file path must be set by -i flag." << std::endl;
        arg_error = true;
    }
    if((input_frame == Frame::GEO || output_frame == Frame::GEO) && !(ltp_alt_set && ltp_lat_set && ltp_lon_set)) {
        std::cerr << "Error: input path or output path is in the GEO frame, this requires the -lat, -lon, -alt to be set." << std::endl;
        arg_error = true;
    }
    if(input_frame == Frame::UNSET) {
        std::cerr << "Error: input frame must be set via -iflu, -ined, or -igeo flags" << std::endl;
        arg_error = true;
    }
    if(input_frame == Frame::UNSET) {
        std::cerr << "Error: output frame must be set via -oflu, -oned, or -ogeo flags" << std::endl;
        arg_error = true;
    }
    if(input_frame == output_frame && input_frame != Frame::UNSET) {
        std::cerr << "Error: input and output frames are the same - no action to take." << std::endl;
        arg_error = true;
    }
    if(arg_error) {
        return 1;
    }

    // Open files
    std::ifstream ifs(input_file_path, std::ifstream::in);
    std::ofstream ofs;
    ofs.open(output_file_path);

    GeodeticConverter conv;
    GeodeticConverter::GeoRef ref;
    ref.latitude = ltp_lat;
    ref.longitude = ltp_lon;
    ref.altitude = ltp_alt;
    conv.initializeReference(ref);
    int point_count = 0;

    std::cout << "Converting from " << frameString(input_frame) << " [" << input_file_path << "] " 
              << " to " << frameString(output_frame) << " [" << output_file_path << "]." << std::endl;
    std::cout << "LTP Lat: " << std::fixed << ref.latitude << " Lon: " << ref.longitude << " Alt: " << ref.altitude << std::endl;
    if(ifs.is_open()) {
        std::string line;
        while(std::getline(ifs, line)) {
            std::istringstream iss(line);
            double x, y, z = 0;
            char comma;
            if(!(iss >> x >> comma >> y)) {
                break;
            }
            ++point_count;

            GeodeticConverter::GeoRef geo;
            GeodeticConverter::CartesianPoint car;
            if(input_frame == Frame::GEO) {
                geo.latitude = x;
                geo.longitude = y;
                geo.altitude = z;
            } else if(input_frame == Frame::FLU) {
                // Convert to NED which is requred for going to NED or GEO
                car.x = x;
                car.y = -y;
                car.z = -z;
            } else {
                car.x = x;
                car.y = y;
                car.z = z;
            }
            // At this point car is in NED or geo is set.
            // If output frame is GEO we can convert car to GEO and be done
            if(output_frame == Frame::GEO) {
                conv.ned2Geodetic(car, geo);
                ofs << std::fixed << std::setprecision(14) << geo.latitude << "," << geo.longitude << std::endl;
                continue;
            } else if(input_frame == Frame::GEO) { 
                conv.geodetic2Ned(geo, car);

                // At this point we have NED
                // If the target is NED we are done otherwise
                // we convert to FLU and are done
                if(output_frame == Frame::FLU) {
                    car.x = car.x;
                    car.y = -car.y;
                    car.z = -car.z;
                }
            }
            ofs << std::fixed << car.x << "," << car.y << std::endl;
        }
    }
    std::cout << "Converted " << point_count << " points." << std::endl;

} /* main() */