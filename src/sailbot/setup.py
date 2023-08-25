from setuptools import setup
from glob import glob

import os
DOCKER = os.environ.get('IS_DOCKER', False)
DOCKER = True if DOCKER == 'True' else False

package_name = 'sailbot'

data_files=[]
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name, glob('launch/*.py')))
data_files.append(('lib/python3.8/site-packages/sailbot/events/', glob(package_name + '/events/*.py')))
data_files.append(('lib/python3.8/site-packages/sailbot/virtualPeripherals/', glob(package_name + '/virtualPeripherals/*.py')))
data_files.append(('lib/python3.8/site-packages/sailbot/peripherals/', glob(package_name + '/peripherals/*.py')))
data_files.append(('lib/python3.8/site-packages/sailbot/CV/', glob(package_name + '/CV/*')))

data_files.append(('lib//python3.8/site-packages/sailbot/website/', glob(package_name + '/website/*.py')))
data_files.append(('lib//python3.8/site-packages/sailbot/website/templates/', glob(package_name + '/website/templates/*')))
data_files.append(('lib//python3.8/site-packages/sailbot/website/static/css/', glob(package_name + '/website/static/css/*')))
data_files.append(('lib//python3.8/site-packages/sailbot/website/static/js/', glob(package_name + '/website/static/js/*')))
data_files.append(('lib//python3.8/site-packages/sailbot/website/static/images/', glob(package_name + '/website/static/images/*')))
data_files.append(('lib//python3.8/site-packages/sailbot/website/static/leaflet/images/', glob(package_name + '/website/static/leaflet/images/*')))
data_files.append(('lib//python3.8/site-packages/sailbot/website/static/leaflet/', glob(package_name + '/website/static/leaflet/*.*')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'compass = sailbot.peripherals.compass:main',
                    'gps = sailbot.peripherals.GPS:main',
                    'windvane = sailbot.peripherals.windvane:main',
                    'main = sailbot.boatMain:main',
                    'drivers = sailbot.peripherals.drivers:main',
                    'offset = sailbot.deprecated.motorOffset:main',
                    'website = sailbot.website.website:ros_main',

                    'virtualDrivers = sailbot.virtualPeripherals.drivers:main',
                    'virtualGPS = sailbot.virtualPeripherals.GPS:main',
        ],
    },
)
