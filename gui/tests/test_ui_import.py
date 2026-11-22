import sys
import types
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def mock_qt(monkeypatch):
    qtcore = types.ModuleType("PyQt5.QtCore")
    qtcore.Qt = MagicMock()
    qtcore.QTimer = MagicMock()
    qtcore.QPointF = MagicMock()
    qtcore.QRectF = MagicMock()
    qtcore.pyqtSignal = lambda *args: MagicMock()
    qtgui = types.ModuleType("PyQt5.QtGui")
    qtgui.QColor = MagicMock()
    qtgui.QPalette = MagicMock()
    qtgui.QBrush = MagicMock()
    qtgui.QPainter = MagicMock()
    qtgui.QPen = MagicMock()
    qtgui.QPolygonF = MagicMock()
    qtgui.QKeySequence = MagicMock()
    qtgui.QImage = MagicMock()
    qtgui.QPixmap = MagicMock()
    qtwidgets = types.ModuleType("PyQt5.QtWidgets")
    for name in ("QAbstractItemView", "QComboBox", "QFormLayout", "QGridLayout", "QGroupBox",
                 "QHBoxLayout", "QLabel", "QMainWindow", "QMessageBox", "QPlainTextEdit",
                 "QPushButton", "QShortcut", "QSplitter", "QTreeWidget", "QTreeWidgetItem",
                 "QVBoxLayout", "QWidget", "QApplication"):
        setattr(qtwidgets, name, MagicMock())
    monkeypatch.setitem(sys.modules, "PyQt5", types.ModuleType("PyQt5"))
    monkeypatch.setitem(sys.modules, "PyQt5.QtCore", qtcore)
    monkeypatch.setitem(sys.modules, "PyQt5.QtGui", qtgui)
    monkeypatch.setitem(sys.modules, "PyQt5.QtWidgets", qtwidgets)
    return sys.modules


def test_ui_modules_import(mock_qt):
    for name in ("indi_relay_gui.main_window", "indi_relay_gui.main", "indi_relay_gui.controller",
                 "indi_relay_gui.checks", "indi_relay_gui.sequence", "indi_relay_gui.model",
                 "indi_relay_gui.config", "indi_relay_gui.indi"):
        __import__(name)
        assert name in sys.modules
