from PySide6 import QtWidgets, QtCore
import maya.cmds as cmds
import maya.OpenMayaUI as omui

class GeneratePluginUI(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(GeneratePluginUI, self).__init__(parent)
        self.setWindowTitle("Generate Plugin")
        self.setGeometry(100, 100, 270, 300)
        self.selected_object = None
        self.init_ui()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        # Mesh Selection
        self.selected_object_label = QtWidgets.QLabel("Selected Object: None")
        self.select_button = QtWidgets.QPushButton("Select Mesh")
        self.select_button.clicked.connect(self.select_object)

        layout.addWidget(QtWidgets.QLabel("Mesh Selection"))
        layout.addWidget(self.select_button)
        layout.addWidget(self.selected_object_label)

        # Voxelization
        layout.addWidget(QtWidgets.QLabel("Voxelization"))
        self.resolution_spinbox = QtWidgets.QSpinBox()
        self.resolution_spinbox.setRange(1, 2048)
        self.resolution_spinbox.setValue(512)
        layout.addWidget(QtWidgets.QLabel("Resolution:"))
        layout.addWidget(self.resolution_spinbox)

        # Morphological
        layout.addWidget(QtWidgets.QLabel("Morphological"))
        self.base_scale_spinbox = QtWidgets.QSpinBox()
        self.base_scale_spinbox.setRange(1, 10)
        self.base_scale_spinbox.setValue(1)
        layout.addWidget(QtWidgets.QLabel("Base Scale:"))
        layout.addWidget(self.base_scale_spinbox)

        # Edit Scale Field Checkbox
        self.edit_scale_checkbox = QtWidgets.QCheckBox("Edit Scale Field")
        layout.addWidget(self.edit_scale_checkbox)

        # Brush Size Slider
        layout.addWidget(QtWidgets.QLabel("Brush Size:"))
        self.brush_size_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.brush_size_slider.setRange(1, 10)
        self.brush_size_slider.setValue(5)
        layout.addWidget(self.brush_size_slider)

        # Brush Mode (Radio Buttons)
        layout.addWidget(QtWidgets.QLabel("Brush Mode:"))
        self.brush_mode_group = QtWidgets.QButtonGroup(self)

        self.increase_radio = QtWidgets.QRadioButton("Increase")
        self.decrease_radio = QtWidgets.QRadioButton("Decrease")
        self.erase_radio = QtWidgets.QRadioButton("Erase")

        self.brush_mode_group.addButton(self.increase_radio)
        self.brush_mode_group.addButton(self.decrease_radio)
        self.brush_mode_group.addButton(self.erase_radio)

        layout.addWidget(self.increase_radio)
        layout.addWidget(self.decrease_radio)
        layout.addWidget(self.erase_radio)

        # Buttons for Actions
        self.scale_field_button = QtWidgets.QPushButton("Reset Scale Field")
        self.scale_field_button.clicked.connect(self.scale_field_action)

        self.voxel_button = QtWidgets.QPushButton("Show Voxel")
        self.voxel_button.clicked.connect(self.voxel_action)

        self.generate_button = QtWidgets.QPushButton("Generate")
        self.generate_button.clicked.connect(self.generate_action)

        layout.addWidget(self.scale_field_button)
        layout.addWidget(self.voxel_button)
        layout.addWidget(self.generate_button)

        self.setLayout(layout)

    def select_object(self):
        """Gets the currently selected object in Maya and updates the label."""
        selection = cmds.ls(selection=True)
        if selection:
            self.selected_object = selection[0]
            self.selected_object_label.setText(f"Selected Object: {self.selected_object}")
        else:
            self.selected_object = None
            self.selected_object_label.setText("Selected Object: None")

    def check_selection(self):
        """Checks if an object is selected, otherwise shows a warning."""
        if not self.selected_object:
            cmds.warning("Please select an object before running this action.")
            return False
        return True

    def scale_field_action(self):
        if not self.check_selection():
            return
        cmds.evalDeferred('cmds.BoundingProxyCmd("scale_field")')

    def voxel_action(self):
        if not self.check_selection():
            return
        resolution = self.resolution_spinbox.value()
        cmds.evalDeferred(f'cmds.BoundingProxyCmd("show_voxel", {resolution})')

    def generate_action(self):
        if not self.check_selection():
            return
        resolution = self.resolution_spinbox.value()
        cmds.evalDeferred(f'cmds.BoundingProxyCmd("generate", {resolution})')

def show():
    global generate_plugin_ui
    try:
        generate_plugin_ui.close()
    except:
        pass
    generate_plugin_ui = GeneratePluginUI()
    generate_plugin_ui.show()