# reads values from config.ini and returns them
import configparser
import logging
import os

root_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
config_path = rf"{root_dir}/sailbot/config.ini"

if not os.path.isfile(config_path):
    raise Exception(f"cannot find config.ini file in {config_path}")

config = configparser.ConfigParser()
config.read(config_path)

logging.basicConfig(
    filename="log.log",
    filemode="a",
    format="%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
    level=logging.DEBUG,
)


def save():
    with open(config_path, "w") as configfile:
        config.write(configfile)


if __name__ == "__main__":
    print(config.sections())
    print(config["MAIN"]["device"])
