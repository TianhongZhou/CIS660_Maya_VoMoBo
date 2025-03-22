from PySide6 import QtWidgets, QtCore
import maya.cmds as cmds
import maya.OpenMayaUI as omui
from shiboken6 import wrapInstance

def get_maya_main_window():
    main_window_ptr = omui.MQtUtil.mainWindow()
    if main_window_ptr:
        return wrapInstance(int(main_window_ptr), QtWidgets.QMainWindow)
    return None

class GeneratePluginUI(QtWidgets.QWidget):
    def __init__(self, parent=None):
        maya_main_window = get_maya_main_window()
        super(GeneratePluginUI, self).__init__(maya_main_window if maya_main_window else parent)

        self.setWindowTitle("Generate Plugin")
        self.setGeometry(100, 100, 270, 300)
        self.selected_object = None
        self.setWindowFlags(QtCore.Qt.Dialog)
        self.init_ui()

        self.action_executed = False

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
        self.resolution_dropdown = QtWidgets.QComboBox()
        self.resolution_values = [8, 16, 32, 64, 128, 256, 512, 1024, 2048]
        self.resolution_dropdown.addItems(map(str, self.resolution_values))
        self.resolution_dropdown.setCurrentText("16")
        layout.addWidget(QtWidgets.QLabel("Resolution:"))
        layout.addWidget(self.resolution_dropdown)

        # Morphological
        layout.addWidget(QtWidgets.QLabel("Morphological"))
        self.base_scale_spinbox = QtWidgets.QDoubleSpinBox()
        self.base_scale_spinbox.setRange(0.1, 99.9)
        self.base_scale_spinbox.setValue(2.0)
        self.base_scale_spinbox.setDecimals(2)
        self.base_scale_spinbox.setSingleStep(0.1)
        layout.addWidget(QtWidgets.QLabel("Base Scale:"))
        layout.addWidget(self.base_scale_spinbox)

        # Edit Scale Field Checkbox
        self.edit_scale_checkbox = QtWidgets.QCheckBox("Edit Scale Field")
        layout.addWidget(self.edit_scale_checkbox)

        # Brush Size
        layout.addWidget(QtWidgets.QLabel("Brush Size:"))
        self.brush_size_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.brush_size_slider.setRange(1, 3000)
        self.brush_size_slider.setValue(10)
        layout.addWidget(self.brush_size_slider)

        # Target Scale
        layout.addWidget(QtWidgets.QLabel("Target Scale:"))
        self.target_scale_spinbox = QtWidgets.QDoubleSpinBox()
        self.target_scale_spinbox.setRange(0.1, 99.9)
        self.target_scale_spinbox.setValue(2.0)
        self.target_scale_spinbox.setDecimals(2)
        self.target_scale_spinbox.setSingleStep(0.1)
        layout.addWidget(self.target_scale_spinbox)

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

        # Button Reset Scale
        self.scale_field_button = QtWidgets.QPushButton("Reset Scale Field")
        self.scale_field_button.clicked.connect(self.reset_scale_field_action)

        # CPU/GPU Selection
        layout.addWidget(QtWidgets.QLabel("Processing Mode:"))
        self.processing_mode_group = QtWidgets.QButtonGroup(self)
        self.cpu_radio = QtWidgets.QRadioButton("CPU")
        self.gpu_radio = QtWidgets.QRadioButton("GPU")
        self.cpu_radio.setChecked(True)  # Default to CPU
        self.processing_mode_group.addButton(self.cpu_radio)
        self.processing_mode_group.addButton(self.gpu_radio)
        layout.addWidget(self.cpu_radio)
        layout.addWidget(self.gpu_radio)

        # Button Show Voxel
        self.voxel_button = QtWidgets.QPushButton("Show Voxel")
        self.voxel_button.clicked.connect(self.voxel_action)

        # Dilation Shape Selection
        layout.addWidget(QtWidgets.QLabel("Dilation Shape:"))
        self.dilation_shape_group = QtWidgets.QButtonGroup(self)
        self.sphere_radio = QtWidgets.QRadioButton("Sphere SE")
        self.cube_radio = QtWidgets.QRadioButton("Cube SE")
        self.sphere_radio.setChecked(True)
        self.dilation_shape_group.addButton(self.sphere_radio)
        self.dilation_shape_group.addButton(self.cube_radio)
        layout.addWidget(self.sphere_radio)
        layout.addWidget(self.cube_radio)

        # Max Error Spinbox (scaled by 1e-2)
        layout.addWidget(QtWidgets.QLabel("Meshing"))
        self.max_err_spinbox = QtWidgets.QDoubleSpinBox()
        self.max_err_spinbox.setRange(0.001, 99.999)
        self.max_err_spinbox.setDecimals(3)
        self.max_err_spinbox.setSingleStep(0.001)
        self.max_err_spinbox.setValue(5.000)
        layout.addWidget(QtWidgets.QLabel("Max Error (x0.01):"))
        layout.addWidget(self.max_err_spinbox)

        # Button Generate
        self.generate_button = QtWidgets.QPushButton("Generate")
        self.generate_button.clicked.connect(self.generate_action)

        layout.addWidget(self.scale_field_button)
        layout.addWidget(self.voxel_button)
        layout.addWidget(self.generate_button)

        self.setLayout(layout)

        self.resolution_dropdown.currentIndexChanged.connect(self.auto_action)
        self.cpu_radio.toggled.connect(self.auto_action)
        self.gpu_radio.toggled.connect(self.auto_action)
        self.scale_field_button.clicked.connect(self.auto_action)
        self.sphere_radio.toggled.connect(self.auto_action)
        self.cube_radio.toggled.connect(self.auto_action)

    def auto_action(self):
        """if action_executed is true, call action again"""
        if self.action_executed:
            self.generate_action()

    def select_object(self):
        """Gets the currently selected object in Maya and updates the label."""
        selection = cmds.ls(selection=True)
        if selection:
            self.selected_object = selection[0]
            self.selected_object_label.setText(f"Selected Object: {self.selected_object}")
            self.mesh_selected = True
        else:
            self.selected_object = None
            self.selected_object_label.setText("Selected Object: None")

    def check_selection(self):
        """Checks if an object is selected, otherwise shows a warning."""
        if not self.selected_object:
            cmds.warning("Please select an object before running this action.")
            return False
        return True

    def reset_scale_field_action(self):
        if not self.check_selection():
            return
        cmds.evalDeferred('cmds.BoundingProxyCmd("reset_scale_field")')

    def voxel_action(self):
        if not self.check_selection():
            return
        resolution = int(self.resolution_dropdown.currentText())
        mode = "cpu" if self.cpu_radio.isChecked() else "gpu"
        seMode = "cube" if self.cube_radio.isChecked() else "sphere"
        baseScale = self.base_scale_spinbox.value()
        self.action_executed = True
        cmds.evalDeferred(f'cmds.BoundingProxyCmd("show_voxel", {resolution}, "{mode}", "{seMode}", {baseScale}, "{self.selected_object}")')

    def generate_action(self):
        if not self.check_selection():
            return
        resolution = int(self.resolution_dropdown.currentText())
        mode = "cpu" if self.cpu_radio.isChecked() else "gpu"
        seMode = "cube" if self.cube_radio.isChecked() else "sphere"
        baseScale = self.base_scale_spinbox.value()
        self.action_executed = True
        maxError = self.max_err_spinbox.value()
        cmds.evalDeferred(f'cmds.BoundingProxyCmd("generate", {resolution}, "{mode}", "{seMode}", {baseScale}, "{self.selected_object}", {maxError})')

def show():
    global generate_plugin_ui
    try:
        generate_plugin_ui.close()
    except:
        pass
    generate_plugin_ui = GeneratePluginUI()
    generate_plugin_ui.show()