/**
 ******************************************************************************
 * @file    module_fwd.hpp
 * @brief   Forward declarations of every composed module of the MAVLink bridge.
 *
 * Every composed module receives a reference to all the others, so their 
 * headers cannot include each other. Headers include this file instead and 
 * include a module header only when a type nested in it is used by value 
 * (e.g. FlightStateManager::ActionScope, MissionHandler::PlanSnapshot).
 *
 * @author  Thiago Marques de Oliveira
 * @date    October, 2026
 ******************************************************************************
 */


#pragma once


/* Forward declarations ------------------------------------------------------*/

class FlightStateManager;
class GroundStationLogger;
class FlightPrimitives;
class ManeuverRunner;
class MissionHandler;
class MissionExecutor;
class FlightActionExecutor;
class ManualControlHandler;
class TelemetryPublisher;