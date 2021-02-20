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
#include <errno.h>
#include <linux/joystick.h>

#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmInputDataConverter.h>

#include <sawJoystick/mtsJoystick.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsJoystick, mtsTaskContinuous, mtsTaskContinuousConstructorArg);

struct mtsJoystickInternals {
    int Device = 0;
};


mtsJoystick::mtsJoystick(const std::string & componentName):
    mtsTaskContinuous(componentName, 1000)
{
    Init();
}


mtsJoystick::mtsJoystick(const mtsTaskContinuousConstructorArg & arg):
    mtsTaskContinuous(arg)
{
    Init();
}


mtsJoystick::~mtsJoystick()
{
    delete mInternals;
    if (mConverter) {
        delete mConverter;
    }
}


void mtsJoystick::Init(void)
{
    mInternals = new mtsJoystickInternals;
    mDeviceName = "";
    mConverter = 0;

    StateTable.AddData(mInputData, "input_data");
    mControllerInterface = AddInterfaceProvided("joystick");
    if (mControllerInterface) {
        mControllerInterface->AddMessageEvents();
        mControllerInterface->AddCommandWrite(&mtsJoystick::SetDevice, this, "set_device", std::string("/dev/input/js0"));
        mControllerInterface->AddCommandVoid(&mtsJoystick::OpenDevice, this, "open_device");
        mControllerInterface->AddCommandVoid(&mtsJoystick::CloseDevice, this, "close_device");
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

    // look for converters
    jsonValue = jsonConfig["converters"];
    if (!jsonValue.empty()) {
        if (!mConverter) {
            mConverter = new prmInputDataConverter(*this);
        }
        mConverter->ConfigureJSON(jsonValue);
    }

    jsonValue = jsonConfig["device"];
    // if the device is specified in the json file
    if (!jsonValue.empty()) {
        // and if it has not already been set
        if (mDeviceName == "") {
            mDeviceName = jsonValue.asString();
            if (mDeviceName == "") {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to convert \"device\" to a string" << std::endl;
                return;
            }
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found \"device\": " << mDeviceName << std::endl;
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "Configure: \"device\" in file \"" << filename
                                       << "\" will be ignored since the device has already been set as: "
                                       << mDeviceName << std::endl;
        }
    }
}

void mtsJoystick::Startup(void)
{
    // if device name has not been set yet
    if (mDeviceName == "") {
        mDeviceName = "/dev/input/js0";
    }
    if (mInternals->Device == 0) {
        OpenDevice();
    }
}


void mtsJoystick::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    if (mInternals->Device != 0) {
        ssize_t readResult;
        struct js_event event;
        readResult = read(mInternals->Device, &event, sizeof(event));
        int errnum = errno;
        if (readResult == sizeof(event)) {
            switch (event.type) {
            case JS_EVENT_BUTTON:
                mInputData.DigitalInputs().at(event.number) = event.value;
                break;
            case JS_EVENT_AXIS:
                mInputData.AnalogInputs().at(event.number) = event.value;
                break;
            default:
                // ignore init events.
                break;
            }
            // emit event with new data
            InputDataEvent(mInputData);

            // update converters if any
            if (mConverter) {
                mConverter->Update(mInputData);
            }
        } else {
            if (errnum == EAGAIN) {
                Sleep(1.0 * cmn_ms); // pseudo 1kHz
                return;
            }
            // other cases should indicate an error
            mControllerInterface->SendError(this->GetName() + ": read error on " + mDeviceName + ", " + strerror(errnum));
            CloseDevice();
        }
    } else {
        // if no device, nothing to do so sleep to not hog CPU
        Sleep(1.0 * cmn_ms);
    }
}


void mtsJoystick::Cleanup(void)
{
    CloseDevice();
}


void mtsJoystick::OpenDevice(void)
{
    if (mInternals->Device != 0) {
        CloseDevice();
    }

    mControllerInterface->SendStatus(this->GetName() + ": opening " + mDeviceName);
    mInternals->Device = open(mDeviceName.c_str(), O_RDONLY | O_NONBLOCK);

    if (mInternals->Device == -1) {
        mControllerInterface->SendError(this->GetName() + ": can't open " + mDeviceName);
        mInternals->Device = 0;
        return;
    }

    __u8 count;
    size_t size;
    if (ioctl(mInternals->Device, JSIOCGAXES, &count) == -1) {
        mControllerInterface->SendError(this->GetName() + ": can't retrieve number of axes for " + mDeviceName);
        CloseDevice();
        return;
    }
    size = count;
    mControllerInterface->SendStatus(this->GetName() + ": " + mDeviceName + " has " + std::to_string(size) + " axe(s)");
    mInputData.AnalogInputs().SetSize(size);
    mInputData.AnalogInputs().SetAll(0.0);

    if (ioctl(mInternals->Device, JSIOCGBUTTONS, &count) == -1) {
        mControllerInterface->SendError(this->GetName() + ": can't retrieve number of buttons for " + mDeviceName);
        CloseDevice();
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
    if (mInternals->Device != 0) {
        mControllerInterface->SendStatus(this->GetName() + ": closing " + mDeviceName);
        close(mInternals->Device);
        mInternals->Device = 0;
        mInputData.AnalogInputs().SetAll(0.0);
        mInputData.DigitalInputs().SetAll(false);
        mInputData.SetValid(false);
    }
}
