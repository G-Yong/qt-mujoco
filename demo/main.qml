import QtQuick 2.15
import Mujoco 1.0

Item {
    id: root
    width: 1280
    height: 768
    visible: true

    // 默认加载的模型路径（可由命令行 / C++ 端注入覆盖）
    property string initialXml: typeof initialXmlPath !== "undefined"
                                ? initialXmlPath : ""

    MujocoView {
        id: mujoco
        anchors.fill: parent
        focus: true

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
}
