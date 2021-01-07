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

#ifndef _mtsJoystick_h
#define _mtsJoystick_h

#include <cisstCommon/cmnPath.h>

#include <cisstMultiTask/mtsTaskContinuous.h>

#include <sawJoystick/sawJoystickConfig.h>
#include <sawJoystick/sawJoystickExport.h>  // always include last


class CISST_EXPORT mtsJoystick : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 protected:

 public:
    inline mtsJoystick(const std::string & componentName):
        mtsTaskContinuous(componentName, false, 1000)
    {
        Init();
    }

    inline mtsJoystick(const mtsTaskContinuousConstructorArg & arg):
        mtsTaskContinuous(arg)
    {
        Init();
    }

    ~mtsJoystick(void) {};

    void SetDevice(const std::string & device);

    /*! Configure the joystick using a JSON file. */
    void Configure(const std::string & filename = "");
    inline void Startup(void);
    void Run(void);
    void Cleanup(void);

 protected:
    void Init(void);

    std::string mDeviceName;
    mtsInterfaceProvided * mControllerInterface;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsJoystick);

#endif  // _mtsJoystick_h
