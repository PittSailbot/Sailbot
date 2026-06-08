import os
from glob import glob

from setuptools import find_packages, setup

package_name = "sailbot"
data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))

data_files.append(("share/" + package_name + "/launch", glob("launch/*.launch.py")))
data_files.append(("share/" + package_name + "/config", glob("config/*.yaml")))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=['test']),
    package_data={
        "sailbot": [
            "config.ini",
            "websiteHosting/templates/*",
            "websiteHosting/static/css/*",
            "websiteHosting/static/js/*",
            "websiteHosting/static/images/*",
            "websiteHosting/static/leaflet/images/*",
            "websiteHosting/static/leaflet/*.*"
        ]
    },
    data_files=data_files,
    include_package_data=False,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pitt Sailbot",
    maintainer_email="pi@todo.todo",
    description="Pitt's Sailbot code!",
    license="See LICENSE",
    entry_points={
        "console_scripts": [
            "mcu_bridge = sailbot.peripherals.mcu_proto_bridge:main",
            "gps = sailbot.peripherals.gps:main",
            "rtk = sailbot.peripherals.rtk:main",
            "cameraServos = sailbot.peripherals.cameraServos:main",
            "main = sailbot.main:main",
            "website = sailbot.websiteHosting.website:ros_main",
            "virtualGPS = sailbot.virtualPeripherals.GPS:main",
            "virtualCompass = sailbot.virtualPeripherals.compass:main",
            "virtualWindvane = sailbot.virtualPeripherals.windvane:main",
            "networkLogger = sailbot.utils.NetworkLogger:main",
            "dummyEvent = sailbot.events.DummyEvent:main",
            "searchEvent = sailbot.events.DummyEventSearch:main",
            "heaveTo = sailbot.events.heaveTo:main",
            "tack_navigation = sailbot.navigation.TackingNavStrategy:main",
            "jybe_navigation = sailbot.navigation.jybingNavStrategy:main",
            "actionManager = sailbot.ActionManager:main",
        ],
    },
)
