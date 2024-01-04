/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2021-01-06

  (C) Copyright 2021-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>

#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsSystemQtWidget.h>
#include <cisstParameterTypes/prmInputDataQtWidget.h>

#include <sawJoystick/mtsJoystick.h>

#include <QApplication>
#include <QMainWindow>

#if ROS1
#include <cisst_ros_bridge/mtsROSBridge.h>
#include <cisst_ros_crtk/mts_ros_crtk_bridge.h>
#elif ROS2
#include <cisst_ros2_bridge/mtsROSBridge.h>
#include <cisst_ros2_crtk/mts_ros_crtk_bridge.h>
#endif

int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsJoystick*", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
#if ROS1
    ros::init(argc, argv, "saw_joystick", ros::init_options::AnonymousName);
    ros::NodeHandle rosNode;
#elif ROS2
    rclcpp::init(argc, argv);
    auto rosNode = std::make_shared<rclcpp::Node>("joystick");
#endif

    // parse options
    cmnCommandLineOptions options;
    std::string device;
    std::string configFile;
    double rosPeriod = 20.0 * cmn_ms;
    double tfPeriod = 20.0 * cmn_ms;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &configFile);

    options.AddOptionOneValue("d", "device",
                              "device (e.g. /dev/input/js0...)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &device);

    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all components and publish (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the tracker's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);

    options.AddOptionOneValue("P", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).  There is no point to have a period higher than the tracker's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);

    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    typedef std::list<std::string> managerConfigType;
    managerConfigType managerConfig;
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON file to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // create the components
    mtsJoystick * joystick = new mtsJoystick("joystick");

    // command line argument overwrites configuration file
    if (device != "") {
        joystick->SetDevice(device);
    }

    // configure the components
    std::string configPath = "";
    // if there's a config file passed as argument, try to locate it
    if (configFile != "") {
        if (cmnPath::Exists(configFile)) {
            configPath = configFile;
        } else {
            // search in current working directory and source tree
            cmnPath searchPath;
            searchPath.Add(cmnPath::GetWorkingDirectory());
            searchPath.Add(std::string(sawJoystick_SOURCE_DIR) + "/../share", cmnPath::TAIL);
            configPath = searchPath.Find(configFile);
            // if still empty
            if (configPath.empty()) {
                std::cerr << "Failed to find configuration file \"" << configFile << "\"" << std::endl
                          << "Searched in: " << configPath << std::endl;
                return 1;
            }
        }
    }
    // configure
    joystick->Configure(configPath);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(joystick);

    // GUI
    mtsSystemQtWidgetComponent * systemWidget = new mtsSystemQtWidgetComponent("systemWidget");
    componentManager->AddComponent(systemWidget);
    componentManager->Connect("systemWidget", "Component",
                              "joystick", "joystick");
    tabWidget->addTab(systemWidget, "System");

    prmInputDataQtWidgetComponent * inputWidget = new prmInputDataQtWidgetComponent("inputWidget");
    componentManager->AddComponent(inputWidget);
    componentManager->Connect("inputWidget", "Component",
                              "joystick", "joystick");
    tabWidget->addTab(inputWidget, "Input");

    // ROS CRTK bridge
#if ROS1
    mts_ros_crtk_bridge * crtk_bridge
        = new mts_ros_crtk_bridge("joystick_crtk_bridge", &rosNode);
#elif ROS2
    mts_ros_crtk_bridge * crtk_bridge
        = new mts_ros_crtk_bridge("joystick_crtk_bridge", rosNode);
#endif

    crtk_bridge->bridge_all_interfaces_provided(joystick->GetName(), "",
                                                rosPeriod, tfPeriod);
    // custom commands
    std::string space = joystick->GetName() + "/";
    mtsROSBridge & subscribers_bridge = crtk_bridge->subscribers_bridge();
#if ROS1
    subscribers_bridge.AddSubscriberToCommandWrite<std::string, std_msgs::String>
        ("joystick", "set_device", space + "set_device");
#elif ROS2
    subscribers_bridge.AddSubscriberToCommandWrite<std::string, std_msgs::msg::String>
        ("joystick", "set_device", space + "set_device");
#endif

    subscribers_bridge.AddSubscriberToCommandVoid
        ("joystick", "open_device", space + "open_device");
    subscribers_bridge.AddSubscriberToCommandVoid
        ("joystick", "close_device", space + "close_device");
    // add and connect all
    componentManager->AddComponent(crtk_bridge);
    componentManager->Connect(subscribers_bridge.GetName(), "joystick",
                              "joystick", "joystick");
    crtk_bridge->Connect();

    // custom user component
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
#if ROS1
    ros::shutdown();
#elif ROS2
    rclcpp::shutdown();
#endif

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
