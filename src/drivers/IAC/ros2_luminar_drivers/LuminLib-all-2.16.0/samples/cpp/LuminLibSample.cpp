/*
* LuminLibSample.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief LuminLibSample
*         This sample program allows users to interact with the ModelH sensor head, powering it on,
*         Changing scan patterns and displaying the LidarReturn data back out to the terminal
*         The default fingerprint is the 5118 port which works with the PCAPPlayer streaming
*         previously captured PCAPs
*/

#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <string>

#include "ModelHDistributor.h"

#include "ControlClientHelper.h"
#include "DataClientHelper.h"

using namespace lum;
using namespace std;

/*! \brief InputHandler
*         The InputHandler parses commands from the user before passing it on commands to the ControlClientHelper and DataClientHelper
*         The code to exercise the SDK can be found in those classes, defined in ControlClientHelper.h and DataClientHelper.h, respectively.
*/
class InputHandler
{
public:
    InputHandler();
    int ProcessCommand();
    void Help( const string& input );
    void Shutdown();

private:
    bool TokenizeAndValidate( const string& input, const char* expectedCommandParam, size_t expectedArgCount, std::vector<string>& tokens );
    bool TokenizeAndValidate( const string& input, const char* expectedCommandParam, std::vector<string>& tokens );
    uint8_t GetUInt( const string& arg, bool& success );
    float GetFloat( const string& arg, bool& success );

    void SelectSensorIndex( const string& input );
    void SetFOV( const string& input );
    void SetFrequency( const string& input );
    void DisplayReturns();
    sample::ControlClientHelper mControlHelper;
    sample::DataClientHelper mDataHelper;
};

/*
* Terminal usability functions, the rest of InputHandler can be found below
*/
void HelpDiscoverSensors()
{
    cout << "discover - return a list of sensors with their fingerprints" << endl;
}

void HelpSensorIndex()
{
    cout << "select # - Select which sensor index # is going to be the target of subsequent \"Set\" commands will run on" << endl;
}

void HelpVolatile()
{
    cout << "volatile - Scan profile commands will not persist through a sensor restart." << endl;
}

void HelpPersistent()
{
    cout << "persistent - Scan profile commands will persist through a sensor restart." << endl;
}

void HelpSetFOV()
{
    cout << "setFOV #a #b - Where #a is the FOV, between 0 and 30 and #b is the center of the FOV (#b optional, defaults to 0)" << endl;
}

void HelpSetHz()
{
    cout << "setHz # - Where # is the refresh rate per second" << endl;
    cout << "     Valid refresh rates are: 1, 2, 5, 8, 10, 20 and 30 hz" << endl;
}

void HelpDisplayReturns()
{
    std::cout << "returns - Display some returns for the currently selected sensor" << endl;
}

void HelpGeneral()
{
    HelpDiscoverSensors();
    HelpSensorIndex();
    HelpVolatile();
    HelpPersistent();
    HelpSetFOV();
    HelpSetHz();
    HelpDisplayReturns();
    std::cout << "Any \"Set \" commands operate on the last selected sensor head" << endl;
    std::cout << "exit - terminates the program" << endl;
    std::cout << "help specified command - returns information about the specified command" << endl;
}

void toLower( string& input )
{
    std::transform( input.begin(), input.end(), input.begin(), ::tolower );
}

InputHandler::InputHandler()
{
    mControlHelper.InitializeModelH();
}

int InputHandler::ProcessCommand()
{
    string input;
    std::getline( std::cin, input );

    if ( input != "" )
    {
        toLower( input );
        if ( input.find( "help" ) != string::npos )
        {
            Help( input );
        }
        else if ( input.find( "discover" ) != string::npos )
        {
            mControlHelper.DiscoverSensors();
        }
        else if ( input.find( "select" ) != string::npos )
        {
            SelectSensorIndex( input );
        }
        else if ( input.find( "setfov" ) != string::npos )
        {
            SetFOV( input );
        }
        else if ( input.find( "sethz" ) != string::npos )
        {
            SetFrequency( input );
        }
        else if ( input.find( "volatile" ) != string::npos )
        {
            mControlHelper.SetDuration( sample::SettingDuration::VOLATILE );
        }
        else if ( input.find( "persistent" ) != string::npos )
        {
            mControlHelper.SetDuration( sample::SettingDuration::PERSISTENT );
        }
        else if ( input.find( "returns" ) != string::npos )
        {
            DisplayReturns();
        }
        else if ( input == "exit" )
        {
            return 0;
        }
        else
        {
            HelpGeneral();
        }
    }

    return 1;
}

void InputHandler::Help( const string& input )
{
    if ( input.find( "discover" ) != string::npos )
    {
        HelpDiscoverSensors();
    }
    else if ( input.find( "select" ) != string::npos )
    {
        HelpSensorIndex();
    }
    else if ( input.find( "setfov" ) != string::npos )
    {
        HelpSetFOV();
    }
    else if ( input.find( "sethz" ) != string::npos )
    {
        HelpSetHz();
    }
    else if ( input.find( "volatile" ) != string::npos )
    {
        HelpVolatile();
    }
    else if ( input.find( "persistent" ) != string::npos )
    {
        HelpPersistent();
    }
    else if ( input.find( "returns" ) != string::npos )
    {
        HelpDisplayReturns();
    }
    else
    {
        HelpGeneral();
    }
}

void InputHandler::Shutdown()
{
    mDataHelper.Shutdown();
    mControlHelper.StopModelH();
}

bool InputHandler::TokenizeAndValidate( const string& input, const char* expectedCommandParam, size_t expectedArgCount, std::vector<string>& tokens )
{
    if ( !TokenizeAndValidate( input, expectedCommandParam, tokens ) )
    {
        cout << "Error validating command " << input << endl;
        return false;
    }

    if ( expectedArgCount == tokens.size() )
    {
        return true;
    }

    cout << "Error validating command " << input << endl;
    return false;
}

bool InputHandler::TokenizeAndValidate( const string& input, const char* expectedCommandParam, std::vector<string>& tokens )
{
    size_t startIdx = 0;
    auto endIdx = input.find_first_of( ' ' );

    auto tokenizedString = input.substr( startIdx, endIdx );

    if ( tokenizedString.compare( expectedCommandParam ) != 0 )
    {
        return false;
    }

    while ( endIdx != string::npos )
    {
        startIdx = endIdx + 1;
        endIdx = input.find( ' ', startIdx );

        if ( endIdx != string::npos )
        {
            tokens.push_back( input.substr( startIdx, endIdx - startIdx ) );
        }
        else
        {
            tokens.push_back( input.substr( startIdx ) );
        }
    }

    return true;
}

float InputHandler::GetFloat( const string& arg, bool& success )
{
    char* endptr;
    float retFloat = strtof( arg.c_str(), &endptr );

    if ( errno == ERANGE || *endptr != '\0' || arg == endptr )
    {
        success = false;
    }

    return retFloat;
}

uint8_t InputHandler::GetUInt( const string& arg, bool& success )
{
    char* endptr;
    uint8_t retUInt = static_cast<uint8_t>( strtoul( arg.c_str(), &endptr, 10 ) );

    if ( errno == ERANGE || *endptr != '\0' || arg == endptr )
    {
        success = false;
    }

    success = true;

    return retUInt;
}

void InputHandler::SelectSensorIndex( const string& input )
{
    //Grab the Sensor Index from the input string.
    vector<string> args;
    if ( !TokenizeAndValidate( input, "select", 1, args ) )
    {
        HelpSensorIndex();
        return;
    }

    bool successfulParse = true;

    uint8_t sensorIndex = GetUInt( args[0], successfulParse );
    if ( !successfulParse || sensorIndex >= mControlHelper.GetDiscoveredSensorCount() )
    {
        cout << "Invalid input: " << args[0] << "index possibly out of range" << endl;
        return;
    }

    //Set the new sensor index, and then grab the fingerprint for that sensor so the data client
    //Can start listening
    mControlHelper.SetSensorIndex( sensorIndex );
    auto sensorID = mControlHelper.GetSensorID();
    mDataHelper.SetSensorID( sensorID );
}

void InputHandler::SetFOV( const string& input )
{
    //Need to grab the FOV before passing it on to the response handler.
    vector<string> args;
    // The user needs to pass in a FOV, and can enter a center degree or it will be assumed to be 0
    if ( !TokenizeAndValidate( input, "setfov", args ) && !( args.size() == 1 || args.size() == 2 ) )
    {
        HelpSetFOV();
        return;
    }

    bool successfulParse = true;
    float fov = GetFloat( args[0], successfulParse );

    if ( !successfulParse || fov > 30 )
    {
        cout << "Invalid input: " << args[0] << endl;
        HelpSetFOV();
        return;
    }

    float center = 0.f;

    if ( args.size() == 2 )
    {
        center = GetFloat( args[1], successfulParse );

        if ( !successfulParse )
        {
            cout << "Invalid input: " << args[1] << endl;
            HelpSetFOV();
            return;
        }
    }

    mControlHelper.SetFOV( fov, center );
}

void InputHandler::SetFrequency( const string& input )
{
    //Parse the Hz and pass it on to the Control Client Helper.
    vector<string> args;

    if ( !TokenizeAndValidate( input, "sethz", 1, args ) )
    {
        HelpSetHz();
        return;
    }

    bool successfulParse = true;

    float hz = GetFloat( args[0], successfulParse );

    if ( !successfulParse || !mControlHelper.SetFrequency( hz ) )
    {
        cout << "Invalid input: " << args[0] << endl;
        HelpSetHz();
        return;
    }
}

void InputHandler::DisplayReturns()
{
    mDataHelper.DisplayReturns();
}

int main( int argc, char** argv )
{
    InputHandler handler;

    std::cout << "LuminLib Sample Program.  Interact with it with the following commands:" << endl
              << endl;
    HelpGeneral();

    while ( handler.ProcessCommand() == 1 )
    {
        //spin
        std::this_thread::sleep_for( std::chrono::milliseconds( 0 ) );
    }

    handler.Shutdown();

    return 0;
}
