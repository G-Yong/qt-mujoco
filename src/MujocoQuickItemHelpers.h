#pragma once
// ---------------------------------------------------------------------------
// MujocoQuickItemHelpers.h
//
// 集中存放 MujocoQuickItem.cpp 内部使用的辅助函数 / 小工具：
//   * 通用的小工具 (boolToInt / isValidIndex / bitFlagIndex / positiveOr ...)
//   * MuJoCo / Qt 类型互转 (copyVec3 / vectorFrom3 / variantToVector3 ...)
//   * 几何 / spec 操作辅助 (primitiveGeomType / setPrimitiveSize /
//     fillPrimitiveSize / uniqueObjectName / freeJointIndexForBody ...)
//   * Simulate 状态刷新 (markUiRefresh)
//
// 仅供 MujocoQuickItem.cpp 内部使用；所有符号放在匿名 namespace 之外的
// `mqi_detail` 命名空间，函数全部声明为 inline / template，
// 可被多个 .cpp 重复包含。
// ---------------------------------------------------------------------------

#include "MujocoQuickItem.h"
#include "simulate.h"
#include <mujoco/mujoco.h>

#include <QString>
#include <QStringList>
#include <QVariant>
#include <QVariantList>
#include <QVector3D>
#include <QVector4D>

#include <vector>

namespace mqi_detail {

// --- 基础工具 --------------------------------------------------------------

inline int boolToInt(bool value) { return value ? 1 : 0; }

inline bool isValidIndex(int index, int count) {
    return index >= 0 && index < count;
}

inline int bitFlagIndex(int bit, int count) {
    if (bit <= 0) return -1;
    for (int i = 0; i < count; ++i) {
        if (bit == (1 << i)) return i;
    }
    return -1;
}

inline double positiveOr(double value, double fallback) {
    return value > 0.0 ? value : fallback;
}

// --- 向量互转 --------------------------------------------------------------

template <typename T>
inline void copyVec3(T dest[3], const QVector3D& src) {
    dest[0] = static_cast<T>(src.x());
    dest[1] = static_cast<T>(src.y());
    dest[2] = static_cast<T>(src.z());
}

inline QVector3D vectorFrom3(const mjtNum* values) {
    return QVector3D(static_cast<float>(values[0]),
                     static_cast<float>(values[1]),
                     static_cast<float>(values[2]));
}

// --- 名称 / 几何类型 -------------------------------------------------------

inline QString objectName(const mjModel* model, int objType, int id) {
    if (!model || id < 0) return QString();
    const char* name = mj_id2name(model, objType, id);
    return (name && name[0] != '\0') ? QString::fromUtf8(name)
                                     : QStringLiteral("#%1").arg(id);
}

inline QString geomTypeName(int type) {
    switch (type) {
    case mjGEOM_PLANE: return QStringLiteral("plane");
    case mjGEOM_HFIELD: return QStringLiteral("hfield");
    case mjGEOM_SPHERE: return QStringLiteral("sphere");
    case mjGEOM_CAPSULE: return QStringLiteral("capsule");
    case mjGEOM_ELLIPSOID: return QStringLiteral("ellipsoid");
    case mjGEOM_CYLINDER: return QStringLiteral("cylinder");
    case mjGEOM_BOX: return QStringLiteral("box");
    case mjGEOM_MESH: return QStringLiteral("mesh");
    case mjGEOM_SDF: return QStringLiteral("sdf");
    default: return QStringLiteral("unknown");
    }
}

inline QString primitiveTypeName(MujocoQuickItem::PrimitiveType type) {
    switch (type) {
    case MujocoQuickItem::PrimitiveBox: return QStringLiteral("PrimitiveBox");
    case MujocoQuickItem::PrimitiveSphere: return QStringLiteral("PrimitiveSphere");
    case MujocoQuickItem::PrimitiveCapsule: return QStringLiteral("PrimitiveCapsule");
    case MujocoQuickItem::PrimitiveCylinder: return QStringLiteral("PrimitiveCylinder");
    case MujocoQuickItem::PrimitiveEllipsoid: return QStringLiteral("PrimitiveEllipsoid");
    }
    return QStringLiteral("UnknownPrimitive(%1)").arg(static_cast<int>(type));
}

inline int primitiveGeomType(MujocoQuickItem::PrimitiveType type) {
    switch (type) {
    case MujocoQuickItem::PrimitiveBox: return mjGEOM_BOX;
    case MujocoQuickItem::PrimitiveSphere: return mjGEOM_SPHERE;
    case MujocoQuickItem::PrimitiveCapsule: return mjGEOM_CAPSULE;
    case MujocoQuickItem::PrimitiveCylinder: return mjGEOM_CYLINDER;
    case MujocoQuickItem::PrimitiveEllipsoid: return mjGEOM_ELLIPSOID;
    }
    return mjGEOM_NONE;
}

// --- spec geom 尺寸填充 ----------------------------------------------------

inline void setPrimitiveSize(mjsGeom* geom, int geomType, const QVector3D& requestedSize) {
    const double x = positiveOr(requestedSize.x(), 0.1);
    const double y = positiveOr(requestedSize.y(), x);
    const double z = positiveOr(requestedSize.z(), y);

    switch (geomType) {
    case mjGEOM_SPHERE:
        geom->size[0] = x;
        break;
    case mjGEOM_CAPSULE:
    case mjGEOM_CYLINDER:
        geom->size[0] = x;
        geom->size[1] = y;
        break;
    default:
        geom->size[0] = x;
        geom->size[1] = y;
        geom->size[2] = z;
        break;
    }
}

// 针对 mjv_initGeom 使用的 size[3] 数组：根据几何类型填充对应分量。
// 模板支持 mjtNum (double) 或 float（mjvGeom::size 是 float[3]）。
template <typename T>
inline void fillPrimitiveSize(T dest[3], int geomType, const QVector3D& requestedSize) {
    const double x = positiveOr(requestedSize.x(), 0.1);
    const double y = positiveOr(requestedSize.y(), x);
    const double z = positiveOr(requestedSize.z(), y);

    dest[0] = static_cast<T>(x);
    dest[1] = static_cast<T>(0.0);
    dest[2] = static_cast<T>(0.0);
    switch (geomType) {
    case mjGEOM_SPHERE:
        break;
    case mjGEOM_CAPSULE:
    case mjGEOM_CYLINDER:
        dest[1] = static_cast<T>(y);
        break;
    default:
        dest[1] = static_cast<T>(y);
        dest[2] = static_cast<T>(z);
        break;
    }
}

// --- QVariant 解析 ---------------------------------------------------------

inline bool variantNumberAt(const QVariantList& values, int index, double* result) {
    if (index < 0 || index >= values.size() || !result) return false;
    bool ok = false;
    const double value = values.at(index).toDouble(&ok);
    if (!ok) return false;
    *result = value;
    return true;
}

inline bool variantToVector3(const QVariant& value, QVector3D* result) {
    if (!result) return false;
    if (value.canConvert<QVector3D>()) {
        *result = value.value<QVector3D>();
        return true;
    }

    const QVariantList values = value.toList();
    if (values.isEmpty() || values.size() > 3) return false;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!variantNumberAt(values, 0, &x)) return false;
    if (values.size() > 1 && !variantNumberAt(values, 1, &y)) return false;
    if (values.size() > 2 && !variantNumberAt(values, 2, &z)) return false;
    *result = QVector3D(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    return true;
}

inline bool variantToVector4(const QVariant& value, QVector4D* result) {
    if (!result) return false;
    if (value.canConvert<QVector4D>()) {
        *result = value.value<QVector4D>();
        return true;
    }

    const QVariantList values = value.toList();
    if (values.size() != 4) return false;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 0.0;
    if (!variantNumberAt(values, 0, &x)) return false;
    if (!variantNumberAt(values, 1, &y)) return false;
    if (!variantNumberAt(values, 2, &z)) return false;
    if (!variantNumberAt(values, 3, &w)) return false;
    *result = QVector4D(static_cast<float>(x), static_cast<float>(y),
                        static_cast<float>(z), static_cast<float>(w));
    return true;
}

inline bool variantToPrimitiveType(const QVariant& value, MujocoQuickItem::PrimitiveType* result) {
    if (!result) return false;
    bool ok = false;
    const int typeValue = value.toInt(&ok);
    if (!ok) return false;

    const auto type = static_cast<MujocoQuickItem::PrimitiveType>(typeValue);
    if (primitiveGeomType(type) == mjGEOM_NONE) return false;
    *result = type;
    return true;
}

// --- 批量请求构造 ----------------------------------------------------------

struct PrimitiveRequest {
    int geomType = mjGEOM_NONE;
    QVector3D position;
    QVector3D size;
    QVector4D rgba;
    QString name;
};

inline bool buildPrimitiveRequests(const QVariantList& positions,
                                   const QVariantList& types,
                                   const QVariantList& sizes,
                                   const QVariantList& colors,
                                   const QVector4D& defaultColor,
                                   const QString& namePrefix,
                                   std::vector<PrimitiveRequest>* requests,
                                   QString* error) {
    if (!requests) return false;
    requests->clear();

    const int count = positions.size();
    if (types.size() != count || sizes.size() != count || (!colors.isEmpty() && colors.size() != count)) {
        if (error) *error = QStringLiteral("Primitive list sizes do not match");
        return false;
    }

    requests->reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        PrimitiveRequest request;
        MujocoQuickItem::PrimitiveType type = MujocoQuickItem::PrimitiveBox;
        if (!variantToPrimitiveType(types.at(i), &type)) {
            if (error) *error = QStringLiteral("Unsupported primitive type at index %1: %2")
                                .arg(i).arg(types.at(i).toString());
            return false;
        }
        request.geomType = primitiveGeomType(type);
        if (request.geomType == mjGEOM_NONE) {
            if (error) *error = QStringLiteral("Unsupported primitive type at index %1: %2")
                                .arg(i).arg(primitiveTypeName(type));
            return false;
        }
        if (!variantToVector3(positions.at(i), &request.position)) {
            if (error) *error = QStringLiteral("Invalid primitive position at index %1").arg(i);
            return false;
        }
        if (!variantToVector3(sizes.at(i), &request.size)) {
            if (error) *error = QStringLiteral("Invalid primitive size at index %1").arg(i);
            return false;
        }
        request.rgba = defaultColor;
        if (!colors.isEmpty() && !variantToVector4(colors.at(i), &request.rgba)) {
            if (error) *error = QStringLiteral("Invalid primitive color at index %1").arg(i);
            return false;
        }
        if (!namePrefix.isEmpty()) {
            request.name = QStringLiteral("%1_%2").arg(namePrefix).arg(i);
        }
        requests->push_back(request);
    }
    return true;
}

inline QString uniqueObjectName(const mjModel* model,
                                const std::vector<QString>& pendingNames,
                                int objectType,
                                const QString& baseName) {
    const QString base = baseName.trimmed().isEmpty() ? QStringLiteral("primitive") : baseName.trimmed();
    QString candidate = base;
    int suffix = 1;
    auto pendingContains = [&pendingNames](const QString& name) {
        for (const QString& pendingName : pendingNames) {
            if (pendingName == name) return true;
        }
        return false;
    };
    while ((model && mj_name2id(model, objectType, candidate.toUtf8().constData()) >= 0) || pendingContains(candidate)) {
        candidate = QStringLiteral("%1_%2").arg(base).arg(suffix++);
    }
    return candidate;
}

// --- free joint / body 位置 ----------------------------------------------

inline int freeJointIndexForBody(const mjModel* model, int bodyId) {
    if (!model || !isValidIndex(bodyId, model->nbody)) return -1;
    const int jointCount = model->body_jntnum[bodyId];
    const int firstJoint = model->body_jntadr[bodyId];
    for (int i = 0; i < jointCount; ++i) {
        const int jointId = firstJoint + i;
        if (model->jnt_type[jointId] == mjJNT_FREE) return jointId;
    }
    return -1;
}

// 在持有 sim.mtx 时调用：把新增 free joint 的 qpos 写到 d->qpos 与 sim 的
// 缓存向量（qpos_/qpos_prev_）。调用前需保证 cache 已经按 m->nq resize。
inline void setFreeJointPosition(mjModel* model, mjData* data,
                                 std::vector<mjtNum>& qposCache,
                                 std::vector<mjtNum>& qposPrevCache,
                                 int jointId, const QVector3D& position) {
    if (!model || !data || jointId < 0) return;
    const int adr = model->jnt_qposadr[jointId];
    const mjtNum vals[7] = {
        position.x(), position.y(), position.z(),
        1.0, 0.0, 0.0, 0.0
    };
    for (int i = 0; i < 7; ++i) data->qpos[adr + i] = vals[i];
    if (qposCache.size() >= static_cast<size_t>(adr + 7)) {
        for (int i = 0; i < 7; ++i) qposCache[adr + i] = vals[i];
    }
    if (qposPrevCache.size() >= static_cast<size_t>(adr + 7)) {
        for (int i = 0; i < 7; ++i) qposPrevCache[adr + i] = vals[i];
    }
}

// mj_recompile 之后，sim.qpos_ / qpos_prev_ 仍保持旧 nq。
// 重新 resize 并从 d_->qpos 同步，避免 Simulate::Sync() 越界读到的旧
// 缓存数据被回写到 d_->qpos，造成新加 free-joint body 的位置被
// "复位" 到 0/原点（这是 addPrimitive 调用后物体经常跑到原点的根因）。
inline void resyncSimulateQposCaches(mujoco::Simulate& sim) {
    if (!sim.m_ || !sim.d_) return;
    const int nq = sim.m_->nq;
    sim.qpos_.assign(sim.d_->qpos, sim.d_->qpos + nq);
    sim.qpos_prev_ = sim.qpos_;
}

inline void setBodyLocalPositionFromWorld(mjModel* model, mjData* data,
                                          int bodyId, const QVector3D& worldPosition) {
    const int parentId = model->body_parentid[bodyId];
    const mjtNum* parentPos = data->xpos + 3 * parentId;
    const mjtNum* parentMat = data->xmat + 9 * parentId;
    const mjtNum dx = worldPosition.x() - parentPos[0];
    const mjtNum dy = worldPosition.y() - parentPos[1];
    const mjtNum dz = worldPosition.z() - parentPos[2];
    mjtNum* bodyPos = model->body_pos + 3 * bodyId;

    bodyPos[0] = parentMat[0] * dx + parentMat[3] * dy + parentMat[6] * dz;
    bodyPos[1] = parentMat[1] * dx + parentMat[4] * dy + parentMat[7] * dz;
    bodyPos[2] = parentMat[2] * dx + parentMat[5] * dy + parentMat[8] * dz;
}

inline void markUiRefresh(mujoco::Simulate& sim) {
    sim.pending_.ui_update_simulation = true;
    sim.pending_.ui_update_physics = true;
    sim.pending_.ui_update_rendering = true;
    sim.pending_.ui_update_visualization = true;
}

} // namespace mqi_detail
