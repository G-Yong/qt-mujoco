import QtQuick 2.15
import Mujoco 1.0

Item {
    id: root
    width: 1280
    height: 768
    visible: true

    readonly property int mjVIS_CONTACTPOINT: 14
    readonly property int mjVIS_CONTACTFORCE: 16
    readonly property int mjVIS_TRANSPARENT: 18
    readonly property int mjVIS_COM: 21
    readonly property int mjRND_SHADOW: 0
    readonly property int mjRND_WIREFRAME: 1
    readonly property int mjRND_REFLECTION: 2
    readonly property int mjDSBL_CONSTRAINT: 1
    readonly property int mjDSBL_CONTACT: 16
    readonly property int mjDSBL_GRAVITY: 128
    readonly property int mjDSBL_ACTUATION: 2048
    readonly property int mjDSBL_SENSOR: 8192
    readonly property int mjENBL_OVERRIDE: 1
    readonly property int mjENBL_ENERGY: 2
    readonly property int mjENBL_FWDINV: 4

    property int keyframeIndex: 0

    function refocusMujoco() {
        mujoco.forceActiveFocus()
    }

    // 默认加载的模型路径（可由命令行 / C++ 端注入覆盖）
    property string initialXml: typeof initialXmlPath !== "undefined"
                                ? initialXmlPath : ""

    MujocoView {
        id: mujoco
        objectName: "mujocoView"
        anchors.fill: parent
        focus: true
        leftUiVisible: false
        rightUiVisible: false

        Component.onCompleted: {
            if (root.initialXml.length > 0)
                start(root.initialXml)
            else
                start()
        }
    }

    // 拖入 .xml / .mjb 即加载
    DropArea {
        anchors.fill: parent
        onDropped: (drop) => {
            if (!drop.hasUrls) return;
            for (var i = 0; i < drop.urls.length; ++i) {
                var u = drop.urls[i].toString();
                var lower = u.toLowerCase();
                if (lower.endsWith(".xml") || lower.endsWith(".mjb")) {
                    var path = u.startsWith("file:///")
                                ? u.substring(8) : u;
                    mujoco.loadModel(decodeURIComponent(path));
                    drop.acceptProposedAction();
                    return;
                }
            }
        }
    }

    Rectangle {
        id: controlPanel
        width: 300
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        color: "#78151a1f"
        opacity: 0.94

        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.AllButtons
            onClicked: mouse.accepted = true
            onPressed: mouse.accepted = true
            onReleased: mouse.accepted = true
        }

        Flickable {
            id: panelScroll
            anchors.fill: parent
            anchors.margins: 12
            contentWidth: width
            contentHeight: panelColumn.height
            clip: true

            Column {
                id: panelColumn
                width: panelScroll.width
                spacing: 10

                Text {
                    width: parent.width
                    text: "MuJoCo 控制"
                    color: "#f0f3f6"
                    font.pixelSize: 18
                    font.bold: true
                }

                PanelSection {
                    title: "仿真"
                    PanelToggle {
                        label: "运行"
                        checked: mujoco.simulationRunning
                        onToggled: function(value) {
                            mujoco.simulationRunning = value
                            root.refocusMujoco()
                        }
                    }
                    Row {
                        width: parent.width
                        spacing: 6
                        PanelButton { width: (parent.width - 12) / 3; label: "后退"; onClicked: { mujoco.stepSimulationBackward(); root.refocusMujoco() } }
                        PanelButton { width: (parent.width - 12) / 3; label: "前进"; onClicked: { mujoco.stepSimulationForward(); root.refocusMujoco() } }
                        PanelButton { width: (parent.width - 12) / 3; label: "重置"; onClicked: { mujoco.resetSimulation(); root.refocusMujoco() } }
                    }
                    Row {
                        width: parent.width
                        spacing: 6
                        PanelButton { width: (parent.width - 12) / 3; label: "清零"; onClicked: { mujoco.zeroControls(); root.refocusMujoco() } }
                        PanelButton { width: (parent.width - 12) / 3; label: "减速"; onClicked: { mujoco.slowDownSimulation(); root.refocusMujoco() } }
                        PanelButton { width: (parent.width - 12) / 3; label: "加速"; onClicked: { mujoco.speedUpSimulation(); root.refocusMujoco() } }
                    }
                    Row {
                        width: parent.width
                        spacing: 6
                        PanelButton { width: 52; label: "Key-"; onClicked: { root.keyframeIndex = Math.max(0, root.keyframeIndex - 1); mujoco.setKeyframeIndex(root.keyframeIndex); root.refocusMujoco() } }
                        Rectangle {
                            width: 52
                            height: 30
                            radius: 4
                            color: "#20262d"
                            border.color: "#3a4652"
                            Text { anchors.centerIn: parent; text: root.keyframeIndex; color: "#f0f3f6"; font.pixelSize: 13 }
                        }
                        PanelButton { width: 52; label: "Key+"; onClicked: { root.keyframeIndex += 1; mujoco.setKeyframeIndex(root.keyframeIndex); root.refocusMujoco() } }
                        PanelButton { width: 44; label: "载入"; onClicked: { mujoco.loadKeyframe(); root.refocusMujoco() } }
                        PanelButton { width: 44; label: "保存"; onClicked: { mujoco.saveKeyframe(); root.refocusMujoco() } }
                    }
                }

                PanelSection {
                    title: "相机"
                    Row {
                        width: parent.width
                        spacing: 6
                        PanelButton { width: (parent.width - 12) / 3; label: "自由"; onClicked: { mujoco.setFreeCamera(); root.refocusMujoco() } }
                        PanelButton { width: (parent.width - 12) / 3; label: "跟踪"; onClicked: { mujoco.setTrackingCamera(-1); root.refocusMujoco() } }
                        PanelButton { width: (parent.width - 12) / 3; label: "对齐"; onClicked: { mujoco.alignView(); root.refocusMujoco() } }
                    }
                    Row {
                        width: parent.width
                        spacing: 6
                        PanelButton { width: (parent.width - 6) / 2; label: "固定相机 -"; onClicked: { mujoco.cycleFixedCamera(-1); root.refocusMujoco() } }
                        PanelButton { width: (parent.width - 6) / 2; label: "固定相机 +"; onClicked: { mujoco.cycleFixedCamera(1); root.refocusMujoco() } }
                    }
                }

                PanelSection {
                    title: "覆盖层 / UI"
                    PanelToggle { label: "Help"; checked: mujoco.helpVisible; onToggled: function(value) { mujoco.helpVisible = value; root.refocusMujoco() } }
                    PanelToggle { label: "Info"; checked: mujoco.infoVisible; onToggled: function(value) { mujoco.infoVisible = value; root.refocusMujoco() } }
                    PanelToggle { label: "Profiler"; checked: mujoco.profilerVisible; onToggled: function(value) { mujoco.profilerVisible = value; root.refocusMujoco() } }
                    PanelToggle { label: "Sensor"; checked: mujoco.sensorVisible; onToggled: function(value) { mujoco.sensorVisible = value; root.refocusMujoco() } }
                    PanelToggle { label: "Pause update"; checked: mujoco.pauseUpdateEnabled; onToggled: function(value) { mujoco.pauseUpdateEnabled = value; root.refocusMujoco() } }
                    PanelToggle { label: "Fullscreen"; checked: mujoco.fullscreenRequested; onToggled: function(value) { mujoco.fullscreenRequested = value; root.refocusMujoco() } }
                    PanelToggle { label: "VSync"; checked: mujoco.vSyncEnabled; onToggled: function(value) { mujoco.vSyncEnabled = value; root.refocusMujoco() } }
                    PanelToggle { label: "Busy wait"; checked: mujoco.busyWaitEnabled; onToggled: function(value) { mujoco.busyWaitEnabled = value; root.refocusMujoco() } }
                    PanelToggle { label: "左侧 MuJoCo UI"; checked: mujoco.leftUiVisible; onToggled: function(value) { mujoco.leftUiVisible = value; root.refocusMujoco() } }
                    PanelToggle { label: "右侧 MuJoCo UI"; checked: mujoco.rightUiVisible; onToggled: function(value) { mujoco.rightUiVisible = value; root.refocusMujoco() } }
                }

                PanelSection {
                    title: "渲染 / 可视化"
                    Row {
                        width: parent.width
                        spacing: 6
                        PanelButton { width: (parent.width - 6) / 2; label: "Frame 循环"; onClicked: { mujoco.cycleFrameVisualization(); root.refocusMujoco() } }
                        PanelButton { width: (parent.width - 6) / 2; label: "Label 循环"; onClicked: { mujoco.cycleLabelVisualization(); root.refocusMujoco() } }
                    }
                    PanelToggle { label: "Contact point"; onToggled: function(value) { mujoco.setVisualizationFlag(root.mjVIS_CONTACTPOINT, value); root.refocusMujoco() } }
                    PanelToggle { label: "Contact force"; onToggled: function(value) { mujoco.setVisualizationFlag(root.mjVIS_CONTACTFORCE, value); root.refocusMujoco() } }
                    PanelToggle { label: "Transparent"; onToggled: function(value) { mujoco.setVisualizationFlag(root.mjVIS_TRANSPARENT, value); root.refocusMujoco() } }
                    PanelToggle { label: "Center of mass"; onToggled: function(value) { mujoco.setVisualizationFlag(root.mjVIS_COM, value); root.refocusMujoco() } }
                    PanelToggle { label: "Shadow"; checked: true; onToggled: function(value) { mujoco.setRenderingFlag(root.mjRND_SHADOW, value); root.refocusMujoco() } }
                    PanelToggle { label: "Wireframe"; onToggled: function(value) { mujoco.setRenderingFlag(root.mjRND_WIREFRAME, value); root.refocusMujoco() } }
                    PanelToggle { label: "Reflection"; checked: true; onToggled: function(value) { mujoco.setRenderingFlag(root.mjRND_REFLECTION, value); root.refocusMujoco() } }
                }

                PanelSection {
                    title: "可视分组"
                    Repeater {
                        model: ListModel {
                            ListElement { caption: "Geom 0"; kind: "geom"; group: 0 }
                            ListElement { caption: "Geom 1"; kind: "geom"; group: 1 }
                            ListElement { caption: "Geom 2"; kind: "geom"; group: 2 }
                            ListElement { caption: "Site 0"; kind: "site"; group: 0 }
                            ListElement { caption: "Joint 0"; kind: "joint"; group: 0 }
                            ListElement { caption: "Tendon 0"; kind: "tendon"; group: 0 }
                            ListElement { caption: "Actuator 0"; kind: "actuator"; group: 0 }
                            ListElement { caption: "Flex 0"; kind: "flex"; group: 0 }
                            ListElement { caption: "Skin 0"; kind: "skin"; group: 0 }
                        }
                        delegate: PanelToggle {
                            label: caption
                            checked: true
                            onToggled: function(value) {
                                if (kind === "geom") mujoco.setGeomGroupVisible(group, value)
                                else if (kind === "site") mujoco.setSiteGroupVisible(group, value)
                                else if (kind === "joint") mujoco.setJointGroupVisible(group, value)
                                else if (kind === "tendon") mujoco.setTendonGroupVisible(group, value)
                                else if (kind === "actuator") mujoco.setActuatorGroupVisible(group, value)
                                else if (kind === "flex") mujoco.setFlexGroupVisible(group, value)
                                else if (kind === "skin") mujoco.setSkinGroupVisible(group, value)
                                root.refocusMujoco()
                            }
                        }
                    }
                }

                PanelSection {
                    title: "物理选项"
                    PanelToggle { label: "Disable constraint"; onToggled: function(value) { mujoco.setPhysicsDisableFlag(root.mjDSBL_CONSTRAINT, value); root.refocusMujoco() } }
                    PanelToggle { label: "Disable contact"; onToggled: function(value) { mujoco.setPhysicsDisableFlag(root.mjDSBL_CONTACT, value); root.refocusMujoco() } }
                    PanelToggle { label: "Disable gravity"; onToggled: function(value) { mujoco.setPhysicsDisableFlag(root.mjDSBL_GRAVITY, value); root.refocusMujoco() } }
                    PanelToggle { label: "Disable actuation"; onToggled: function(value) { mujoco.setPhysicsDisableFlag(root.mjDSBL_ACTUATION, value); root.refocusMujoco() } }
                    PanelToggle { label: "Disable sensor"; onToggled: function(value) { mujoco.setPhysicsDisableFlag(root.mjDSBL_SENSOR, value); root.refocusMujoco() } }
                    PanelToggle { label: "Enable override"; onToggled: function(value) { mujoco.setPhysicsEnableFlag(root.mjENBL_OVERRIDE, value); root.refocusMujoco() } }
                    PanelToggle { label: "Enable energy"; onToggled: function(value) { mujoco.setPhysicsEnableFlag(root.mjENBL_ENERGY, value); root.refocusMujoco() } }
                    PanelToggle { label: "Enable fwd/inv"; onToggled: function(value) { mujoco.setPhysicsEnableFlag(root.mjENBL_FWDINV, value); root.refocusMujoco() } }

                    Text { width: parent.width; text: "Actuator groups"; color: "#aeb8c2"; font.pixelSize: 12 }
                    Repeater {
                        model: 6
                        delegate: PanelToggle {
                            label: "Actuator group " + index
                            checked: true
                            onToggled: function(value) {
                                mujoco.setActuatorGroupEnabled(index, value)
                                root.refocusMujoco()
                            }
                        }
                    }
                }
            }
        }
    }

    component PanelSection: Column {
        property string title: ""
        width: parent ? parent.width : 260
        spacing: 6

        Text {
            width: parent.width
            text: parent.title
            color: "#8fd3ff"
            font.pixelSize: 13
            font.bold: true
        }
    }

    component PanelButton: Rectangle {
        id: button
        property string label: ""
        signal clicked()
        width: parent ? parent.width : 120
        height: 30
        radius: 4
        color: mouseArea.containsPress ? "#3e5f77" : (mouseArea.containsMouse ? "#34414d" : "#242a31")
        border.color: "#465563"

        Text {
            anchors.centerIn: parent
            text: button.label
            color: "#eef3f7"
            font.pixelSize: 12
            elide: Text.ElideRight
            width: parent.width - 8
            horizontalAlignment: Text.AlignHCenter
        }

        MouseArea {
            id: mouseArea
            anchors.fill: parent
            hoverEnabled: true
            onClicked: button.clicked()
        }
    }

    component PanelToggle: Rectangle {
        id: toggle
        property string label: ""
        property bool checked: false
        signal toggled(bool value)
        width: parent ? parent.width : 260
        height: 28
        radius: 4
        color: checked ? "#21465c" : "#242a31"
        border.color: checked ? "#5bb6e6" : "#465563"

        Text {
            anchors.left: parent.left
            anchors.leftMargin: 9
            anchors.verticalCenter: parent.verticalCenter
            width: parent.width - 54
            text: toggle.label
            color: "#eef3f7"
            font.pixelSize: 12
            elide: Text.ElideRight
        }

        Rectangle {
            width: 34
            height: 16
            radius: 8
            anchors.right: parent.right
            anchors.rightMargin: 8
            anchors.verticalCenter: parent.verticalCenter
            color: toggle.checked ? "#5bb6e6" : "#53606b"

            Rectangle {
                width: 12
                height: 12
                radius: 6
                y: 2
                x: toggle.checked ? 20 : 2
                color: "#f7fafc"
            }
        }

        MouseArea {
            anchors.fill: parent
            hoverEnabled: true
            onClicked: {
                toggle.checked = !toggle.checked
                toggle.toggled(toggle.checked)
            }
        }
    }
}
