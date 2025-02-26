import maya.cmds as cmds

def show():
    if cmds.window("GenerateWindow", exists=True):
        cmds.deleteUI("GenerateWindow")

    cmds.window("GenerateWindow", title="Generate Plugin", widthHeight=(200, 100))
    cmds.columnLayout(adjustableColumn=True)
    
    cmds.button(label="Generate", command=lambda x: cmds.generate())

    cmds.showWindow("GenerateWindow")