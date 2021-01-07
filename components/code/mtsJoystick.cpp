/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2021-01-06

  (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <fcntl.h>
// #include <stdio.h>
// #include <unistd.h>
#include <linux/joystick.h>

#include <cisstCommon/cmnStrings.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <sawJoystick/mtsJoystick.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsJoystick, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

#if (CISST_OS == CISST_LINUX) || (CISST_OS == CISST_DARWIN)
#include <glob.h>
inline bool Glob(const std::string & pattern, std::vector<std::string> & paths) {
    glob_t glob_result;
    bool result = glob(pattern.c_str(), 0, 0, &glob_result);
    for (unsigned int i = 0; i < glob_result.gl_pathc; i++) {
        paths.push_back(std::string(glob_result.gl_pathv[i]));
    }
    globfree(&glob_result);
    return result;
}
#endif


void mtsJoystick::Init(void)
{
    mDeviceName = "/dev/input/js0";

    StateTable.AddData(mInputData, "input_data");
    mControllerInterface = AddInterfaceProvided("Controller");
    if (mControllerInterface) {
        mControllerInterface->AddMessageEvents();
        mControllerInterface->AddCommandReadState(StateTable, mInputData, "input_data");
        mControllerInterface->AddEventWrite(InputDataEvent, "input_data", mInputData);
        mControllerInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                                  "period_statistics");
    }
}


void mtsJoystick::SetDevice(const std::string & device)
{
    mDeviceName = device;
}


void mtsJoystick::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;

    if (filename.empty()) {
        return;
    }

    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    // make sure the file valid json
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                 << "File: " << filename << std::endl << "Error(s):" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        return;
    }
    // keep the content of the file in cisstLog for debugging
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                               << " using file \"" << filename << "\"" << std::endl
                               << "----> content of configuration file: " << std::endl
                               << jsonConfig << std::endl
                               << "<----" << std::endl;
#if 0

    // start looking for configuration parameters
    // name used for the tracking device, i.e. reference frame by default
    // if not specified, "NDI"
    jsonValue = jsonConfig["name"];
    if (!jsonValue.empty()) {
        mTrackerName = jsonValue.asString();
    }

    jsonValue = jsonConfig["serial-port"];
    // if the port is specified in the json file
    if (!jsonValue.empty()) {
        // and if it has not already been set
        if (mSerialPortName == "") {
            mSerialPortName = jsonValue.asString();
            if (mSerialPortName == "") {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to convert \"serial-port\" to a string" << std::endl;
                return;
            }
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found \"serial-port\": " << mSerialPortName << std::endl;
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "Configure: \"serial-port\" in file \"" << filename
                                       << "\" will be ignored since the serial port has already been set as: "
                                       << mSerialPortName << std::endl;
        }
    }

    // path to locate tool definitions
    const Json::Value definitionPath = jsonConfig["definition-path"];
    // preserve order from config file
    for (int index = (definitionPath.size() - 1);
         index >= 0;
         --index) {
        std::string path = definitionPath[index].asString();
        if (path != "") {
            mDefinitionPath.Add(path, cmnPath::HEAD);
        }
    }

    // stray markers
    mStrayMarkersReferenceFrame = mTrackerName;
    const Json::Value jsonStrayMarkers = jsonConfig["stray-markers"];
    if (!jsonStrayMarkers.empty()) {
        jsonValue = jsonStrayMarkers["reference"];
        if (!jsonValue.empty()) {
            mStrayMarkersReferenceFrame = jsonValue.asString();
        }
        jsonValue = jsonStrayMarkers["track"];
        if (!jsonValue.empty()) {
            mTrackingStrayMarkers = jsonValue.asBool();
        }
    }

    // get tools defined by user
    const Json::Value jsonTools = jsonConfig["tools"];
    for (unsigned int index = 0; index < jsonTools.size(); ++index) {
        std::string name, uniqueID, definition, reference;
        // tools
        const Json::Value jsonTool = jsonTools[index];
        // --- name
        jsonValue = jsonTool["name"];
        if (!jsonValue.empty()) {
            name = jsonValue.asString();
            if (name == mTrackerName) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: tools["
                                         << index << "] can't use the name \"" << name
                                         << "\" since it matches the tracker name.  You can either rename the tool or the tracker itself using \"name\" at the top level" << std::endl;
                return;
            }
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to find \"name\" for tools["
                                     << index << "]" << std::endl;
            return;
        }
        // --- serial number
        jsonValue = jsonTool["unique-id"];
        if (!jsonValue.empty()) {
            uniqueID = jsonValue.asString();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to find \"unique-id\" for tools["
                                     << index << "]" << std::endl;
            return;
        }
        // --- definition file (rom file)
        jsonValue = jsonTool["definition"];
        if (!jsonValue.empty()) {
            definition = jsonValue.asString();
            // try to locate the file
            if (!cmnPath::Exists(definition)) {
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: definition file \"" << definition
                                           << "\" not found, using definition-paths to locate it."
                                           << std::endl;
                std::string fullPath = mDefinitionPath.Find(definition);
                if (fullPath != "") {
                    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found definition file \"" << fullPath
                                               << "\" for \"" << definition << "\"" << std::endl;
                    definition = fullPath;
                } else {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find definition file \"" << definition
                                             << "\" using search path: " << mDefinitionPath << std::endl;
                    return;
                }
            }
        } else {
            definition = "";
        }
        // --- reference frame
        jsonValue = jsonTool["reference"];
        if (!jsonValue.empty()) {
            reference = jsonValue.asString();
        } else {
            // default reference based on tracker name
            reference = mTrackerName;
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: no reference frame \"for\" for tools["
                                     << index << "].  Position will be reported wrt NDI device frame or global reference frame" << std::endl;
        }

        AddTool(name, uniqueID, definition, reference);
    }

    // make sure the reference frame has been added if mReferenceFrame is set
    if (mStrayMarkersReferenceFrame != mTrackerName) {
        mStrayMarkersReferenceTool = mTools.GetItem(mStrayMarkersReferenceFrame, CMN_LOG_LEVEL_INIT_ERROR);
        if (!mStrayMarkersReferenceTool) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find reference \"" << mStrayMarkersReferenceFrame
                                     << "\".  Make sure reference frame/tool exists!" << std::endl;
            mStrayMarkersReferenceFrame = mTrackerName;
        }
    }

    // try to connect after configuring if serial port is defined
    if (mSerialPortName != "") {
        Connect(mSerialPortName);
    }
#endif
}

void mtsJoystick::Startup(void)
{
    if (mDevice == 0) {
        OpenDevice();
    }
}


void mtsJoystick::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

     if (mDevice != 0) {
        ssize_t bytes;
        struct js_event event;
        bytes = read(mDevice, &event, sizeof(event));
        if (bytes == sizeof(event)) {
            switch (event.type) {
            case JS_EVENT_BUTTON:
                mInputData.DigitalInputs().at(event.number) = event.value;
                break;
            case JS_EVENT_AXIS:
                mInputData.AnalogInputs().at(event.number) = event.value;
                break;
            default:
                /* Ignore init events. */
                break;
            }
            // emit event with new data
            InputDataEvent(mInputData);
        } else {
            mControllerInterface->SendError(this->GetName() + ": read error on " + mDeviceName);
            CloseDevice();
        }
    }
}


void mtsJoystick::Cleanup(void)
{
    CloseDevice();
}


void mtsJoystick::OpenDevice(void)
{
    if (mDevice != 0) {
        CloseDevice();
    }

    mControllerInterface->SendStatus(this->GetName() + ": opening " + mDeviceName);
    mDevice = open(mDeviceName.c_str(), O_RDONLY);

    if (mDevice == -1) {
        mControllerInterface->SendError(this->GetName() + ": can't open " + mDeviceName);
        mDevice = 0;
        return;
    }

    __u8 count;
    size_t size;
    if (ioctl(mDevice, JSIOCGAXES, &count) == -1) {
        mControllerInterface->SendError(this->GetName() + ": can't retrieve number of axes for " + mDeviceName);
        mDevice = 0;
        return;
    }
    size = count;
    mControllerInterface->SendStatus(this->GetName() + ": " + mDeviceName + " has " + std::to_string(size) + " axe(s)");
    mInputData.AnalogInputs().SetSize(size);
    mInputData.AnalogInputs().SetAll(0.0);

    if (ioctl(mDevice, JSIOCGBUTTONS, &count) == -1) {
        mControllerInterface->SendError(this->GetName() + ": can't retrieve number of buttons for " + mDeviceName);
        mDevice = 0;
        return;
    }
    size = count;
    mControllerInterface->SendStatus(this->GetName() + ": " + mDeviceName + " has " + std::to_string(size) + " button(s)");
    mInputData.DigitalInputs().SetSize(size);
    mInputData.DigitalInputs().SetAll(false);
    mInputData.SetValid(true);
}


void mtsJoystick::CloseDevice(void)
{
    if (mDevice != 0) {
        mControllerInterface->SendStatus(this->GetName() + ": closing " + mDeviceName);
        close(mDevice);
        mDevice = 0;
        mInputData.SetValid(false);
    }
}
