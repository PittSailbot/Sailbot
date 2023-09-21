import os
from glob import glob

from setuptools import setup, find_packages

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

package_name = "sailbot"
data_files = [("share/ament_index/resource_index/packages", ["resource/" + package_name]),
              ("share/" + package_name, ["package.xml"]), ("share/" + package_name, glob("launch/*.py")),
              ("lib/python3.10/site-packages/sailbot/events/", glob(package_name + "/events/*.py")), (
              "lib/python3.10/site-packages/sailbot/virtualPeripherals/",
              glob(package_name + "/virtualPeripherals/*.py")),
              ("lib/python3.10/site-packages/sailbot/peripherals/", glob(package_name + "/peripherals/*.py"),),
              ("lib/python3.10/site-packages/sailbot/CV/", glob(package_name + "/CV/*")),
              ("lib//python3.10/site-packages/sailbot/website/", glob(package_name + "/website/*.py"),),
              ("lib//python3.10/site-packages/sailbot/website/templates/", glob(package_name + "/website/templates/*"),),
              ("lib//python3.10/site-packages/sailbot/website/static/css/",
               glob(package_name + "/website/static/css/*"),),
              ("lib//python3.10/site-packages/sailbot/website/static/js/", glob(package_name + "/website/static/js/*"),),
              ("lib//python3.10/site-packages/sailbot/website/static/images/",
               glob(package_name + "/website/static/images/*"),), (
              "lib//python3.10/site-packages/sailbot/website/static/leaflet/images/",
              glob(package_name + "/website/static/leaflet/images/*"),), (
              "lib//python3.10/site-packages/sailbot/website/static/leaflet/",
              glob(package_name + "/website/static/leaflet/*.*"),)]

setup(
    name="sailbot",
    version="0.0.0",
    packages=find_packages(),
    data_files=data_files,
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pitt Sailbot",
    maintainer_email="pi@todo.todo",
    description="Pitt's Sailbot code!",
    license="See LICENSE",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "compass = sailbot.peripherals.compass:main",
            "gps = sailbot.peripherals.GPS:main",
            "windvane = sailbot.peripherals.windvane:main",
            "main = sailbot.boatMain:main",
            "drivers = sailbot.peripherals.drivers:main",
            "offset = sailbot.deprecated.motorOffset:main",
            "website = sailbot.website.website:ros_main",
            "virtualDrivers = sailbot.virtualPeripherals.drivers:main",
            "virtualGPS = sailbot.virtualPeripherals.GPS:main",
        ],
    },
)
