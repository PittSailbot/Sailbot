import os
from glob import glob

from setuptools import setup, find_packages

DOCKER = os.environ.get("IS_DOCKER", False)
DOCKER = True if DOCKER == "True" else False

package_name = "sailbot"
data_files=[]
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
# data_files.append(('share/' + package_name, glob('src/sailbot/launch/*.py')))
data_files.append(('share/' + package_name, glob('launch/*.py')))

# data_files.append(('lib/python3.10/site-packages/sailbot/', glob(package_name + '/*.py')))
# data_files.append(('lib/python3.10/site-packages/sailbot/', glob('*.ini')))
# data_files.append(('lib/python3.10/site-packages/sailbot/CV/', glob(package_name + '/CV/*.py')))
# data_files.append(('lib/python3.10/site-packages/sailbot/events/', glob(package_name + '/events/*.py')))
# data_files.append(('lib/python3.10/site-packages/sailbot/peripherals/', glob(package_name + '/peripherals/*.py')))
# data_files.append(('lib/python3.10/site-packages/sailbot/utils/', glob(package_name + '/utils/*.py')))
# data_files.append(('lib/python3.10/site-packages/sailbot/virtualPeripherals/', glob(package_name + '/virtualPeripherals/*.py')))
# data_files.append(('lib//python3.10/site-packages/sailbot/website/', glob(package_name + '/website/*.py')))
# data_files.append(('lib//python3.10/site-packages/sailbot/website/templates/', glob(package_name + '/website/templates/*')))
# data_files.append(('lib//python3.10/site-packages/sailbot/website/static/css/', glob(package_name + '/website/static/css/*')))
# data_files.append(('lib//python3.10/site-packages/sailbot/website/static/js/', glob(package_name + '/website/static/js/*')))
# data_files.append(('lib//python3.10/site-packages/sailbot/website/static/images/', glob(package_name + '/website/static/images/*')))
# data_files.append(('lib//python3.10/site-packages/sailbot/website/static/leaflet/images/', glob(package_name + '/website/static/leaflet/images/*')))
# data_files.append(('lib//python3.10/site-packages/sailbot/website/static/leaflet/', glob(package_name + '/website/static/leaflet/*.*')))
# data_files.append(('lib//python3.10/site-packages/sailbot/telemetry/protobuf/', glob(package_name + '/telemetry/protobuf/*.py')))
data_files.append(('lib/sailbot/', glob(package_name + '/*.py')))
data_files.append(('lib/sailbot/', glob('*.ini')))
data_files.append(('lib/sailbot/CV/', glob(package_name + '/CV/*.py')))
data_files.append(('lib/sailbot/events/', glob(package_name + '/events/*.py')))
data_files.append(('lib/sailbot/peripherals/', glob(package_name + '/peripherals/*.py')))
data_files.append(('lib/sailbot/utils/', glob(package_name + '/utils/*.py')))
data_files.append(('lib/sailbot/telemetry/protobuf/', glob(package_name + '/telemetry/protobuf/*.py')))
data_files.append(('lib/sailbot/virtualPeripherals/', glob(package_name + '/virtualPeripherals/*.py')))
data_files.append(('lib/sailbot/websiteHosting/', glob(package_name + '/websiteHosting/*.py')))
data_files.append(('lib/sailbot/websiteHosting/templates/', glob(package_name + '/websiteHosting/templates/*')))
data_files.append(('lib/sailbot/websiteHosting/static/css/', glob(package_name + '/websiteHosting/static/css/*')))
data_files.append(('lib/sailbot/websiteHosting/static/js/', glob(package_name + '/websiteHosting/static/js/*')))
data_files.append(('lib/sailbot/websiteHosting/static/images/', glob(package_name + '/websiteHosting/static/images/*')))
data_files.append(('lib/sailbot/websiteHosting/static/leaflet/images/', glob(package_name + '/websiteHosting/static/leaflet/images/*')))
data_files.append(('lib/sailbot/websiteHosting/static/leaflet/', glob(package_name + '/websiteHosting/static/leaflet/*.*')))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where=package_name),
    package_dir={"":"sailbot"},
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
            "transceiver = sailbot.peripherals.transceiver:main",
            "main = sailbot.main:main",
            "drivers = sailbot.boatMovement:main",
            "offset = sailbot.deprecated.motorOffset:main",
            "website = sailbot.websiteHosting.website:ros_main",
            "virtualDrivers = sailbot.virtualPeripherals.drivers:main",
            "virtualGPS = sailbot.virtualPeripherals.GPS:main",
            "rosTest = sailbot.rosTest:main",
            'networkLogger = sailbot.utils.NetworkLogger:main'
        ],
    },
)
