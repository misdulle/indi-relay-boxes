"""Entry point: python3 -m indi_relay_gui"""

import argparse
import os
import sys

from PyQt5.QtWidgets import QApplication

from . import config as config_module
from .main_window import MainWindow


def main(argv=None):
    parser = argparse.ArgumentParser(description="Control panel for the indi-relay-boxes drivers")
    parser.add_argument("--config", default=config_module.DEFAULT_CONFIG_PATH,
                        help="YAML config file (default: %(default)s)")
    args = parser.parse_args(argv)

    try:
        config_module.ensure_config(args.config, _TEMPLATE())
        config = config_module.load(args.config)
    except Exception as error:
        print("Config error (%s): %s" % (args.config, error), file=sys.stderr)
        return 2

    app = QApplication(sys.argv if argv is None else ["indi-relay-gui"] + argv)
    app.setApplicationName("indi-relay-boxes")
    window = MainWindow(config, os.path.abspath(args.config))
    window.resize(1200, 720)
    window.show()
    return app.exec_()


def _TEMPLATE():
    from importlib.resources import files
    return files("indi_relay_gui").joinpath("config.example.yaml").read_text()


if __name__ == "__main__":
    sys.exit(main())
