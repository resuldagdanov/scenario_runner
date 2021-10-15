#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides access to a scenario configuration parser
"""

import carla
import glob
import os
import xml.etree.ElementTree as ET

from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration, ActorConfigurationData
from srunner.scenarioconfigs.route_scenario_configuration import RouteConfiguration



class ScenarioConfigurationParser(object):

    """
    Pure static class providing access to parser methods for scenario configuration files (*.xml)
    """

    @staticmethod
    def parse_scenario_configuration(scenario_name, config_file_name):
        """
        Parse all scenario configuration files at srunner/examples and the additional
        config files, providing a list of ScenarioConfigurations @return

        If scenario_name starts with "group:" all scenarios that
        have that type are parsed and returned. Otherwise only the
        scenario that matches the scenario_name is parsed and returned.
        """

        list_of_config_files = glob.glob("{}/srunner/examples/*.xml".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))

        if config_file_name != '':
            list_of_config_files.append(config_file_name)

        single_scenario_only = True
        if scenario_name.startswith("group:"):
            single_scenario_only = False
            scenario_name = scenario_name[6:]

        scenario_configurations = []

        for file_name in list_of_config_files:
            tree = ET.parse(file_name)

            for scenario in tree.iter("scenario"):
                scenario_config_name = scenario.attrib.get('name', None)
                scenario_config_type = scenario.attrib.get('type', None)

                if single_scenario_only:
                    # Check the scenario is the correct one
                    if scenario_config_name != scenario_name:
                        continue
                else:
                    # Check the scenario is of the correct type
                    if scenario_config_type != scenario_name:
                        continue

                config = ScenarioConfiguration()
                config.town = scenario.attrib.get('town')
                config.name = scenario_config_name
                config.type = scenario_config_type

                for elem in scenario.iter():
                    if elem.tag == 'scenario':
                        continue  # ET iters get the scenario element too, which has already been parsed

                    # Elements available for all scenarios
                    elif elem.tag == 'ego_vehicle':
                        config.ego_vehicles.append(ActorConfigurationData.parse_from_node(elem, 'hero'))
                        config.trigger_points.append(config.ego_vehicles[-1].transform)
                    elif elem.tag == 'other_actor':
                        config.other_actors.append(ActorConfigurationData.parse_from_node(elem, 'scenario'))
                    elif elem.tag == 'weather':
                        config.weather.cloudiness = float(elem.attrib.get("cloudiness", 0))
                        config.weather.precipitation = float(elem.attrib.get("precipitation", 0))
                        config.weather.precipitation_deposits = float(elem.attrib.get("precipitation_deposits", 0))
                        config.weather.wind_intensity = float(elem.attrib.get("wind_intensity", 0.35))
                        config.weather.sun_azimuth_angle = float(elem.attrib.get("sun_azimuth_angle", 0.0))
                        config.weather.sun_altitude_angle = float(elem.attrib.get("sun_altitude_angle", 15.0))
                        config.weather.fog_density = float(elem.attrib.get("fog_density", 0.0))
                        config.weather.fog_distance = float(elem.attrib.get("fog_distance", 0.0))
                        config.weather.wetness = float(elem.attrib.get("wetness", 0.0))
                    elif elem.tag == 'route':
                        route_conf = RouteConfiguration()
                        route_conf.parse_xml(elem)
                        config.route = route_conf

                    # Any other possible element, add it as a config attribute 
                    else:
                        exec('config.{}=elem.attrib'.format(elem.tag))

                scenario_configurations.append(config)

        return scenario_configurations

    

    @staticmethod
    def get_list_of_scenarios(config_file_name):
        """
        Parse *all* config files and provide a list with all scenarios @return
        """

        list_of_config_files = glob.glob("{}/srunner/examples/*.xml".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))
        list_of_config_files += glob.glob("{}/srunner/examples/*.xosc".format(os.getenv('SCENARIO_RUNNER_ROOT', "./")))

        if config_file_name != '':
            list_of_config_files.append(config_file_name)

        scenarios = []
        for file_name in list_of_config_files:
            if ".xosc" in file_name:
                tree = ET.parse(file_name)
                scenarios.append("{} (OpenSCENARIO)".format(tree.find("FileHeader").attrib.get('description', None)))
            else:
                tree = ET.parse(file_name)
                for scenario in tree.iter("scenario"):
                    scenarios.append(scenario.attrib.get('name', None))

        return scenarios
