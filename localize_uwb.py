#!/usr/bin/env python
"""
The Rutgers UWB localization script for NJT Innovation Train project. 
Please read the tutorial that accompanies this sketch:
https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Python

This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
parameters and upload this sketch. Watch the coordinates change as you move your device around!
"""

__author__ = "Zezhou Wang"
__copyright__ = "Copyright (C) 2022 Zezhou Wang"
__license__ = "LGPL-3.0 license"
__version__ = "1.0"


from time import sleep
from datetime import datetime
import sys
import pymap3d

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, POZYX_SUCCESS, PozyxConstants, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, PozyxRegisters)
from pythonosc.udp_client import SimpleUDPClient

from pypozyx.tools.version_check import perform_latest_version_check


class GeodeticLocation(object):
    """Class for geodetic location in default global projection (latitude, longitude, height)"""
    def __init__(self, lat, lon, height):
        self.lat = lat
        self.lon = lon
        self.h = height


class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""

    def __init__(self, pozyx, osc_udp_client, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None, convert_to_geodetic=False, geodetic_origin=None):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id
        self.convert_to_geodetic = convert_to_geodetic
        self.geodetic_origin = geodetic_origin

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")
        print("- System will manually configure tag")
        print("")
        print("- System will auto start positioning")
        print("")
        if self.remote_id is None:
            self.pozyx.printDeviceInfo(self.remote_id)
        else:
            for device_id in [None, self.remote_id]:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("------------POZYX POSITIONING V{} -------------".format(version))
        print("")

        self.setAnchorsManual(save_to_flash=False)
        self.printPublishConfigurationResult()

    def loop(self):
        """Performs positioning and displays/exports the results."""
        position = Coordinates()
        status = self.pozyx.doPositioning(
            position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)
        if status == POZYX_SUCCESS:
            self.printPublishPosition(position)
        else:
            self.printPublishErrorCode("positioning")
        return position, status

    def printPublishPosition(self, position):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        network_id = self.remote_id
        if network_id is None:
            network_id = 0
        if not self.convert_to_geodetic:
            print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
                "0x%0.4x" % network_id, pos=position))        
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/position", [network_id, int(position.x), int(position.y), int(position.z)])
        else:
            lat, lon, alt = pymap3d.enu.enu2geodetic(   position.x/1000, position.y/1000, position.z/1000,
                                                        self.geodetic_origin.lat, self.geodetic_origin.lon, self.geodetic_origin.h)
            tag_geodetic = GeodeticLocation(lat=lat, lon=lon, height=alt)
            print("POS ID {}, lat: {geo.lat} lon: {geo.lon} alt(meters): {geo.h}".format(
                "0x%0.4x" % network_id, geo=tag_geodetic))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/position", [network_id, tag_geodetic.lat, tag_geodetic.lon, tag_geodetic.h])

    def printPublishErrorCode(self, operation):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            print("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, error_code[0]])
            return
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("ERROR %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error", [operation, network_id, error_code[0]])
        else:
            self.pozyx.getErrorCode(error_code)
            print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                  (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, -1])
            # should only happen when not being able to communicate with a remote Pozyx.

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(remote_id=self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, remote_id=self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),
                                                       remote_id=self.remote_id)

        if save_to_flash:
            self.pozyx.saveAnchorIds(remote_id=self.remote_id)
            self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], remote_id=self.remote_id)
        return status

    def printPublishConfigurationResult(self):
        """Prints and potentially publishes the anchor configuration result in a human-readable way."""
        list_size = SingleRegister()

        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [device_list[i], int(anchor_coordinates.x), int(anchor_coordinates.y), int(anchor_coordinates.z)])
                sleep(0.025)

    def printPublishAnchorConfiguration(self):
        """Prints and potentially publishes the anchor configuration"""
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, int(anchor.coordinates.x), int(anchor.coordinates.y), int(anchor.coordinates.z)])
                sleep(0.025)

    def writeToLog(self, pos, writer):
        now_timestamp = str(datetime.now())
        writer.write(now_timestamp + ";")
        writer.write(str(pos))
        writer.write(";")
        writer.write(str(status))
        writer.write("\n")


if __name__ == "__main__":
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    # necessary data for calibration, change the IDs and coordinates yourself according to your measurement
    # ENU Coordinates in mm unit
    anchors = [
        ################ Warren_st ################
        DeviceCoordinates(0x6852, 1, Coordinates(-1470025.063382538 ,   836923.1546295199,  28906.718403148943)),
        DeviceCoordinates(0x6849, 1, Coordinates(-1476238.7779445837,   828888.7528694972,  28826.373417384275)),
        DeviceCoordinates(0x6833, 1, Coordinates(-1477924.2129642635,   819060.9448268565,  28669.79401375397)),
        DeviceCoordinates(0x6822, 1, Coordinates(-1484127.5074235614,   809840.9363642727,  29050.81268378393)),
        DeviceCoordinates(0x6818, 1, Coordinates(-1486213.1025745637,   798807.6521546288,  29083.006332569028)),
        ################ Penn ################
        DeviceCoordinates(0x680A, 1, Coordinates(0                  ,   0                   ,  2057.076320127741)),
        DeviceCoordinates(0x6843, 1, Coordinates(-6957.994192102367 ,   6937.585230431794   ,  1069.9519419342751)),
        DeviceCoordinates(0x685A, 1, Coordinates(-9666.944320674519 ,   9466.32149975476    ,  1058.4338467646317)),
        DeviceCoordinates(0x6859, 1, Coordinates(-16207.15403463636 ,   15124.946312484093  ,  1076.8917475791753)),
        DeviceCoordinates(0x6870, 1, Coordinates(-23700.366550654884,   20139.55687570101   ,  1165.1307180998902)),
        DeviceCoordinates(0x6872, 1, Coordinates(-21420.749460693273,   22674.154582084473  ,  1317.62621358809)),
        DeviceCoordinates(0x6839, 1, Coordinates(-4993.4323135565   ,   8815.756644632678   ,  1062.451837109625)),
        DeviceCoordinates(0x683D, 1, Coordinates( 2350.8370660298654,   2514.7043727960483  ,  1041.9989227180813)),
    ]

    # Whether or not to use geodetic instead of enu
    # convert_to_geodetic, geodetic_origin = False, None
    convert_to_geodetic, geodetic_origin = True, GeodeticLocation(lat=40.73516281847992, lon=-74.16378141248713, height=3.057076320127741)

    remote_id = 0x6e66               # remote device network ID
    remote = False                   # whether to use a remote device, default not
    if not remote:
        remote_id = None

    # enable to send position data through OSC
    use_processing = True

    # configure if you want to route OSC to outside your localhost. Networking knowledge is required.
    ip = "54.152.142.229"
    network_port = 57546

    osc_udp_client = None
    if use_processing:
        osc_udp_client = SimpleUDPClient(ip, network_port)

    # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
    algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
    # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
    dimension = PozyxConstants.DIMENSION_3D
    # height of device, required in 2.5D positioning
    height = 3000

    pozyx = PozyxSerial(serial_port)
    r = ReadyToLocalize(pozyx, osc_udp_client, anchors, algorithm, dimension, height, remote_id, convert_to_geodetic, geodetic_origin)
    r.setup()
    
    now = datetime.now()
    dt_string = now.strftime("%m-%d-%Y_%H-%M-%S")
    
    # Main thread positioning starts here
    with open("./{}.log".format(dt_string), "a") as writer:
        while True:
            pos, status = r.loop()
            r.writeToLog(pos, writer)
            



