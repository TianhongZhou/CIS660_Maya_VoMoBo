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

        # ---- Mesh Selection ----
        mesh_group = QtWidgets.QGroupBox("Mesh Selection")
        mesh_layout = QtWidgets.QVBoxLayout()
        self.select_button = QtWidgets.QPushButton("Select Mesh")
        self.select_button.clicked.connect(self.select_object)
        self.selected_object_label = QtWidgets.QLabel("Selected Object: None")
        mesh_layout.addWidget(self.select_button)
        mesh_layout.addWidget(self.selected_object_label)
        mesh_group.setLayout(mesh_layout)
        layout.addWidget(mesh_group)

        # ---- Voxelization Block ----
        voxel_group = QtWidgets.QGroupBox("Voxelization")
        voxel_layout = QtWidgets.QVBoxLayout()
        self.resolution_dropdown = QtWidgets.QComboBox()
        self.resolution_values = [8, 16, 32, 64, 128, 256, 512, 1024, 2048]
        self.resolution_dropdown.addItems(map(str, self.resolution_values))
        self.resolution_dropdown.setCurrentText("16")
        voxel_layout.addWidget(QtWidgets.QLabel("Resolution:"))
        voxel_layout.addWidget(self.resolution_dropdown)
        voxel_group.setLayout(voxel_layout)
        layout.addWidget(voxel_group)

        # ---- Morphological & Brush block ----
        morph_group = QtWidgets.QGroupBox("Morphological & Brush")
        morph_layout = QtWidgets.QVBoxLayout()
        self.base_scale_spinbox = QtWidgets.QDoubleSpinBox()
        self.base_scale_spinbox.setRange(0.1, 99.9)
        self.base_scale_spinbox.setValue(2.0)
        self.base_scale_spinbox.setDecimals(2)
        self.base_scale_spinbox.setSingleStep(0.1)
        morph_layout.addWidget(QtWidgets.QLabel("Base Scale:"))
        morph_layout.addWidget(self.base_scale_spinbox)

        self.edit_scale_checkbox = QtWidgets.QCheckBox("Edit Scale Field")
        self.edit_scale_checkbox.stateChanged.connect(self.edit_scale_field_changed)
        morph_layout.addWidget(self.edit_scale_checkbox)

        morph_layout.addWidget(QtWidgets.QLabel("Brush Size:"))
        self.brush_size_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.brush_size_slider.setRange(1, 3000)
        self.brush_size_slider.setValue(10)
        morph_layout.addWidget(self.brush_size_slider)

        morph_layout.addWidget(QtWidgets.QLabel("Target Scale:"))
        self.target_scale_spinbox = QtWidgets.QDoubleSpinBox()
        self.target_scale_spinbox.setRange(0.1, 99.9)
        self.target_scale_spinbox.setValue(2.0)
        self.target_scale_spinbox.setDecimals(2)
        self.target_scale_spinbox.setSingleStep(0.1)
        morph_layout.addWidget(self.target_scale_spinbox)

        morph_layout.addWidget(QtWidgets.QLabel("Brush Mode:"))
        self.brush_mode_group = QtWidgets.QButtonGroup(self)
        self.increase_radio = QtWidgets.QRadioButton("Increase")
        self.decrease_radio = QtWidgets.QRadioButton("Decrease")
        self.erase_radio = QtWidgets.QRadioButton("Erase")
        self.brush_mode_group.addButton(self.increase_radio)
        self.brush_mode_group.addButton(self.decrease_radio)
        self.brush_mode_group.addButton(self.erase_radio)
        morph_layout.addWidget(self.increase_radio)
        morph_layout.addWidget(self.decrease_radio)
        morph_layout.addWidget(self.erase_radio)

        morph_group.setLayout(morph_layout)
        layout.addWidget(morph_group)

        # ---- Meshing block ----
        mesh_block = QtWidgets.QGroupBox("Meshing")
        mesh_block_layout = QtWidgets.QVBoxLayout()

        mesh_block_layout.addWidget(QtWidgets.QLabel("Dilation Shape:"))
        self.dilation_shape_group = QtWidgets.QButtonGroup(self)
        self.sphere_radio = QtWidgets.QRadioButton("Sphere SE")
        self.cube_radio = QtWidgets.QRadioButton("Cube SE")
        self.sphere_radio.setChecked(True)
        self.dilation_shape_group.addButton(self.sphere_radio)
        self.dilation_shape_group.addButton(self.cube_radio)
        mesh_block_layout.addWidget(self.sphere_radio)
        mesh_block_layout.addWidget(self.cube_radio)

        self.max_err_spinbox = QtWidgets.QDoubleSpinBox()
        self.max_err_spinbox.setRange(0.001, 99.999)
        self.max_err_spinbox.setDecimals(3)
        self.max_err_spinbox.setSingleStep(0.001)
        self.max_err_spinbox.setValue(5.000)
        mesh_block_layout.addWidget(QtWidgets.QLabel("Max Error (x0.01):"))
        mesh_block_layout.addWidget(self.max_err_spinbox)

        mesh_block_layout.addWidget(QtWidgets.QLabel("Simplify Method:"))
        self.simplify_group = QtWidgets.QButtonGroup(self)
        self.cqem_radio = QtWidgets.QRadioButton("CQEM")
        self.qem_radio = QtWidgets.QRadioButton("QEM")
        self.cqem_radio.setChecked(True)
        self.simplify_group.addButton(self.cqem_radio)
        self.simplify_group.addButton(self.qem_radio)
        mesh_block_layout.addWidget(self.cqem_radio)
        mesh_block_layout.addWidget(self.qem_radio)

        mesh_block.setLayout(mesh_block_layout)
        layout.addWidget(mesh_block)

        # ---- Buttons block ----
        button_group = QtWidgets.QGroupBox("Actions")
        button_layout = QtWidgets.QVBoxLayout()

        button_layout.addWidget(QtWidgets.QLabel("Processing Mode:"))
        self.processing_mode_group = QtWidgets.QButtonGroup(self)
        self.cpu_radio = QtWidgets.QRadioButton("CPU")
        self.gpu_radio = QtWidgets.QRadioButton("GPU")
        self.cpu_radio.setChecked(True)
        self.processing_mode_group.addButton(self.cpu_radio)
        self.processing_mode_group.addButton(self.gpu_radio)
        button_layout.addWidget(self.cpu_radio)
        button_layout.addWidget(self.gpu_radio)

        self.scale_field_button = QtWidgets.QPushButton("Reset Scale Field")
        self.scale_field_button.clicked.connect(self.reset_scale_field_action)
        self.voxel_button = QtWidgets.QPushButton("Show Voxel")
        self.voxel_button.clicked.connect(self.voxel_action)
        self.generate_button = QtWidgets.QPushButton("Generate")
        self.generate_button.clicked.connect(self.generate_action)
        button_layout.addWidget(self.scale_field_button)
        # button_layout.addWidget(self.voxel_button)
        button_layout.addWidget(self.generate_button)
        button_group.setLayout(button_layout)
        layout.addWidget(button_group)

        self.setLayout(layout)

        # self.resolution_dropdown.currentIndexChanged.connect(self.auto_action)
        # self.cpu_radio.toggled.connect(self.auto_action)
        # self.gpu_radio.toggled.connect(self.auto_action)
        # self.scale_field_button.clicked.connect(self.auto_action)
        # self.sphere_radio.toggled.connect(self.auto_action)
        # self.cube_radio.toggled.connect(self.auto_action)

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
        simplify = "cqem" if self.cqem_radio.isChecked() else "qem"
        cmds.evalDeferred(f'cmds.BoundingProxyCmd("generate", {resolution}, "{mode}", "{seMode}", {baseScale}, "{self.selected_object}", {maxError}, "{simplify}")')

    def edit_scale_field_changed(self, state):
        if not self.check_selection():
            return
        resolution = int(self.resolution_dropdown.currentText())
        baseScale = self.base_scale_spinbox.value()
        if state > 0:
            cmds.setAttr(self.selected_object + ".displayColors", 1)
            cmds.evalDeferred(f'cmds.BoundingProxyCmd("show_scale_field", {resolution}, {baseScale}, "{self.selected_object}")')
        else:
            cmds.setAttr(self.selected_object + ".displayColors", 0)

def show():
    global generate_plugin_ui
    try:
        generate_plugin_ui.close()
    except:
        pass
    generate_plugin_ui = GeneratePluginUI()
    generate_plugin_ui.show()