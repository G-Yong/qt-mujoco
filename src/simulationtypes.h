#pragma once
#include <QString>
#include <QVector3D>
#include <QMetaType>
#include <QList>

// ---------------------------------------------------------------------------
// JointInfo — 关节固有属性描述。支持 Q_GADGET，可在 QML 中直接读取属性。
//
// type 对应 mjtJoint：
//   0 = free  (7 qpos：3 位置 + 4 四元数)
//   1 = ball  (4 qpos：四元数)
//   2 = slide (1 qpos：滑移距离，单位米)
//   3 = hinge (1 qpos：旋转角度，单位弧度)
// ---------------------------------------------------------------------------
struct JointInfo {
    Q_GADGET
    Q_PROPERTY(QString name      MEMBER name      CONSTANT)
    Q_PROPERTY(int     type      MEMBER type      CONSTANT)
    Q_PROPERTY(QString typeName  MEMBER typeName  CONSTANT)
    Q_PROPERTY(int     qposDim   MEMBER qposDim   CONSTANT)
    Q_PROPERTY(bool    limited   MEMBER limited   CONSTANT)
    Q_PROPERTY(double  rangeMin  MEMBER rangeMin  CONSTANT)
    Q_PROPERTY(double  rangeMax  MEMBER rangeMax  CONSTANT)
    Q_PROPERTY(double  stiffness MEMBER stiffness CONSTANT)
    Q_PROPERTY(int     qposadr   MEMBER qposadr   CONSTANT)
public:
    QString name;
    int     type      = 0;    // mjtJoint 枚举值
    QString typeName;         // "free" / "ball" / "slide" / "hinge"
    int     qposDim   = 1;    // qpos 维度
    bool    limited   = false;
    double  rangeMin  = 0.0;  // limited=false 时为 0
    double  rangeMax  = 0.0;
    double  stiffness = 0.0;
    int     qposadr   = 0;    // 在全局 qpos 数组中的起始下标
};
Q_DECLARE_METATYPE(JointInfo)

// ---------------------------------------------------------------------------
// ContactInfo — 单次接触（contact）的快照数据。支持 Q_GADGET，可在 QML 中读取。
//
// 由 MujocoQuickItem::contact(int) / contacts() 返回，每帧在物理锁内采样。
//
// 字段说明：
//   geom0Id / geom1Id   — 两个碰撞 geom 的 ID（geom[0]/geom[1]），-1 表示无效。
//   body0Id / body1Id   — 对应 body 的 ID；geom<0 时为 -1。
//   geom0Name / geom1Name — geom 名称；无名时用 "#<id>"。
//   body0Name / body1Name — body 名称；无名时用 "#<id>"。
//   dist                — 接触距离（< 0 表示穿透）。
//   active              — 是否为有效接触：exclude==0 且 efc_address>=0。
//   penetrating         — dist < 0。
//   normalForce         — 法向力大小（由 mj_contactForce 计算，单位 N）。
//   position            — 接触点在世界坐标系中的位置。
//   normal              — 接触法向量（从 geom1 指向 geom0，由接触帧首行给出）。
// ---------------------------------------------------------------------------
struct ContactInfo {
    Q_GADGET
    Q_PROPERTY(int      geom0Id     MEMBER geom0Id     CONSTANT)
    Q_PROPERTY(int      geom1Id     MEMBER geom1Id     CONSTANT)
    Q_PROPERTY(int      body0Id     MEMBER body0Id     CONSTANT)
    Q_PROPERTY(int      body1Id     MEMBER body1Id     CONSTANT)
    Q_PROPERTY(QString  geom0Name   MEMBER geom0Name   CONSTANT)
    Q_PROPERTY(QString  geom1Name   MEMBER geom1Name   CONSTANT)
    Q_PROPERTY(QString  body0Name   MEMBER body0Name   CONSTANT)
    Q_PROPERTY(QString  body1Name   MEMBER body1Name   CONSTANT)
    Q_PROPERTY(double   dist        MEMBER dist        CONSTANT)
    Q_PROPERTY(bool     active      MEMBER active      CONSTANT)
    Q_PROPERTY(bool     penetrating MEMBER penetrating CONSTANT)
    Q_PROPERTY(double   normalForce MEMBER normalForce CONSTANT)
    Q_PROPERTY(QVector3D position   MEMBER position    CONSTANT)
    Q_PROPERTY(QVector3D normal     MEMBER normal      CONSTANT)
public:
    int      geom0Id     = -1;
    int      geom1Id     = -1;
    int      body0Id     = -1;
    int      body1Id     = -1;
    QString  geom0Name;
    QString  geom1Name;
    QString  body0Name;
    QString  body1Name;
    double   dist        = 0.0;   // 接触距离（< 0 = 穿透）
    bool     active      = false; // exclude==0 && efc_address>=0
    bool     penetrating = false; // dist < 0
    double   normalForce = 0.0;   // 法向力大小（N）
    QVector3D position;           // 接触点世界坐标
    QVector3D normal;             // 接触法向量（接触帧首行）
};
Q_DECLARE_METATYPE(ContactInfo)
