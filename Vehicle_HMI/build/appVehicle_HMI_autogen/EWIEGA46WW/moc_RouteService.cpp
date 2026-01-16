/****************************************************************************
** Meta object code from reading C++ file 'RouteService.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.10.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../RouteService.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'RouteService.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.10.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN12RouteServiceE_t {};
} // unnamed namespace

template <> constexpr inline auto RouteService::qt_create_metaobjectdata<qt_meta_tag_ZN12RouteServiceE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "RouteService",
        "globalPathChanged",
        "",
        "localPathChanged",
        "routeStatusChanged",
        "routeDistanceChanged",
        "routeDurationChanged",
        "waypointCountChanged",
        "isNavigatingChanged",
        "routeReady",
        "navigationStarted",
        "navigationCompleted",
        "currentSpeedChanged",
        "remainingDistanceChanged",
        "remainingTimeChanged",
        "progressPercentChanged",
        "nextInstructionChanged",
        "distanceToNextTurnChanged",
        "currentPositionChanged",
        "navigateTo",
        "x",
        "y",
        "cancelNavigation",
        "clearRoute",
        "globalPath",
        "QVariantList",
        "localPath",
        "routeStatus",
        "routeDistance",
        "routeDuration",
        "waypointCount",
        "isNavigating",
        "currentSpeed",
        "remainingDistance",
        "remainingTime",
        "progressPercent",
        "nextInstruction",
        "distanceToNextTurn",
        "currentX",
        "currentY"
    };

    QtMocHelpers::UintData qt_methods {
        // Signal 'globalPathChanged'
        QtMocHelpers::SignalData<void()>(1, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'localPathChanged'
        QtMocHelpers::SignalData<void()>(3, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'routeStatusChanged'
        QtMocHelpers::SignalData<void()>(4, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'routeDistanceChanged'
        QtMocHelpers::SignalData<void()>(5, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'routeDurationChanged'
        QtMocHelpers::SignalData<void()>(6, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'waypointCountChanged'
        QtMocHelpers::SignalData<void()>(7, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'isNavigatingChanged'
        QtMocHelpers::SignalData<void()>(8, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'routeReady'
        QtMocHelpers::SignalData<void()>(9, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'navigationStarted'
        QtMocHelpers::SignalData<void()>(10, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'navigationCompleted'
        QtMocHelpers::SignalData<void()>(11, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'currentSpeedChanged'
        QtMocHelpers::SignalData<void()>(12, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'remainingDistanceChanged'
        QtMocHelpers::SignalData<void()>(13, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'remainingTimeChanged'
        QtMocHelpers::SignalData<void()>(14, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'progressPercentChanged'
        QtMocHelpers::SignalData<void()>(15, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'nextInstructionChanged'
        QtMocHelpers::SignalData<void()>(16, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'distanceToNextTurnChanged'
        QtMocHelpers::SignalData<void()>(17, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'currentPositionChanged'
        QtMocHelpers::SignalData<void()>(18, 2, QMC::AccessPublic, QMetaType::Void),
        // Slot 'navigateTo'
        QtMocHelpers::SlotData<void(double, double)>(19, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Double, 20 }, { QMetaType::Double, 21 },
        }}),
        // Slot 'cancelNavigation'
        QtMocHelpers::SlotData<void()>(22, 2, QMC::AccessPublic, QMetaType::Void),
        // Slot 'clearRoute'
        QtMocHelpers::SlotData<void()>(23, 2, QMC::AccessPublic, QMetaType::Void),
    };
    QtMocHelpers::UintData qt_properties {
        // property 'globalPath'
        QtMocHelpers::PropertyData<QVariantList>(24, 0x80000000 | 25, QMC::DefaultPropertyFlags | QMC::EnumOrFlag, 0),
        // property 'localPath'
        QtMocHelpers::PropertyData<QVariantList>(26, 0x80000000 | 25, QMC::DefaultPropertyFlags | QMC::EnumOrFlag, 1),
        // property 'routeStatus'
        QtMocHelpers::PropertyData<QString>(27, QMetaType::QString, QMC::DefaultPropertyFlags, 2),
        // property 'routeDistance'
        QtMocHelpers::PropertyData<double>(28, QMetaType::Double, QMC::DefaultPropertyFlags, 3),
        // property 'routeDuration'
        QtMocHelpers::PropertyData<double>(29, QMetaType::Double, QMC::DefaultPropertyFlags, 4),
        // property 'waypointCount'
        QtMocHelpers::PropertyData<int>(30, QMetaType::Int, QMC::DefaultPropertyFlags, 5),
        // property 'isNavigating'
        QtMocHelpers::PropertyData<bool>(31, QMetaType::Bool, QMC::DefaultPropertyFlags, 6),
        // property 'currentSpeed'
        QtMocHelpers::PropertyData<double>(32, QMetaType::Double, QMC::DefaultPropertyFlags, 10),
        // property 'remainingDistance'
        QtMocHelpers::PropertyData<double>(33, QMetaType::Double, QMC::DefaultPropertyFlags, 11),
        // property 'remainingTime'
        QtMocHelpers::PropertyData<double>(34, QMetaType::Double, QMC::DefaultPropertyFlags, 12),
        // property 'progressPercent'
        QtMocHelpers::PropertyData<int>(35, QMetaType::Int, QMC::DefaultPropertyFlags, 13),
        // property 'nextInstruction'
        QtMocHelpers::PropertyData<QString>(36, QMetaType::QString, QMC::DefaultPropertyFlags, 14),
        // property 'distanceToNextTurn'
        QtMocHelpers::PropertyData<double>(37, QMetaType::Double, QMC::DefaultPropertyFlags, 15),
        // property 'currentX'
        QtMocHelpers::PropertyData<double>(38, QMetaType::Double, QMC::DefaultPropertyFlags, 16),
        // property 'currentY'
        QtMocHelpers::PropertyData<double>(39, QMetaType::Double, QMC::DefaultPropertyFlags, 16),
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<RouteService, qt_meta_tag_ZN12RouteServiceE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject RouteService::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN12RouteServiceE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN12RouteServiceE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN12RouteServiceE_t>.metaTypes,
    nullptr
} };

void RouteService::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<RouteService *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->globalPathChanged(); break;
        case 1: _t->localPathChanged(); break;
        case 2: _t->routeStatusChanged(); break;
        case 3: _t->routeDistanceChanged(); break;
        case 4: _t->routeDurationChanged(); break;
        case 5: _t->waypointCountChanged(); break;
        case 6: _t->isNavigatingChanged(); break;
        case 7: _t->routeReady(); break;
        case 8: _t->navigationStarted(); break;
        case 9: _t->navigationCompleted(); break;
        case 10: _t->currentSpeedChanged(); break;
        case 11: _t->remainingDistanceChanged(); break;
        case 12: _t->remainingTimeChanged(); break;
        case 13: _t->progressPercentChanged(); break;
        case 14: _t->nextInstructionChanged(); break;
        case 15: _t->distanceToNextTurnChanged(); break;
        case 16: _t->currentPositionChanged(); break;
        case 17: _t->navigateTo((*reinterpret_cast<std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast<std::add_pointer_t<double>>(_a[2]))); break;
        case 18: _t->cancelNavigation(); break;
        case 19: _t->clearRoute(); break;
        default: ;
        }
    }
    if (_c == QMetaObject::IndexOfMethod) {
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::globalPathChanged, 0))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::localPathChanged, 1))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::routeStatusChanged, 2))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::routeDistanceChanged, 3))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::routeDurationChanged, 4))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::waypointCountChanged, 5))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::isNavigatingChanged, 6))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::routeReady, 7))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::navigationStarted, 8))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::navigationCompleted, 9))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::currentSpeedChanged, 10))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::remainingDistanceChanged, 11))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::remainingTimeChanged, 12))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::progressPercentChanged, 13))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::nextInstructionChanged, 14))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::distanceToNextTurnChanged, 15))
            return;
        if (QtMocHelpers::indexOfMethod<void (RouteService::*)()>(_a, &RouteService::currentPositionChanged, 16))
            return;
    }
    if (_c == QMetaObject::ReadProperty) {
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast<QVariantList*>(_v) = _t->globalPath(); break;
        case 1: *reinterpret_cast<QVariantList*>(_v) = _t->localPath(); break;
        case 2: *reinterpret_cast<QString*>(_v) = _t->routeStatus(); break;
        case 3: *reinterpret_cast<double*>(_v) = _t->routeDistance(); break;
        case 4: *reinterpret_cast<double*>(_v) = _t->routeDuration(); break;
        case 5: *reinterpret_cast<int*>(_v) = _t->waypointCount(); break;
        case 6: *reinterpret_cast<bool*>(_v) = _t->isNavigating(); break;
        case 7: *reinterpret_cast<double*>(_v) = _t->currentSpeed(); break;
        case 8: *reinterpret_cast<double*>(_v) = _t->remainingDistance(); break;
        case 9: *reinterpret_cast<double*>(_v) = _t->remainingTime(); break;
        case 10: *reinterpret_cast<int*>(_v) = _t->progressPercent(); break;
        case 11: *reinterpret_cast<QString*>(_v) = _t->nextInstruction(); break;
        case 12: *reinterpret_cast<double*>(_v) = _t->distanceToNextTurn(); break;
        case 13: *reinterpret_cast<double*>(_v) = _t->currentX(); break;
        case 14: *reinterpret_cast<double*>(_v) = _t->currentY(); break;
        default: break;
        }
    }
}

const QMetaObject *RouteService::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RouteService::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN12RouteServiceE_t>.strings))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int RouteService::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 20)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 20;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 20)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 20;
    }
    if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::BindableProperty
            || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void RouteService::globalPathChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void RouteService::localPathChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void RouteService::routeStatusChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void RouteService::routeDistanceChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void RouteService::routeDurationChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void RouteService::waypointCountChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void RouteService::isNavigatingChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 6, nullptr);
}

// SIGNAL 7
void RouteService::routeReady()
{
    QMetaObject::activate(this, &staticMetaObject, 7, nullptr);
}

// SIGNAL 8
void RouteService::navigationStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 8, nullptr);
}

// SIGNAL 9
void RouteService::navigationCompleted()
{
    QMetaObject::activate(this, &staticMetaObject, 9, nullptr);
}

// SIGNAL 10
void RouteService::currentSpeedChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 10, nullptr);
}

// SIGNAL 11
void RouteService::remainingDistanceChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 11, nullptr);
}

// SIGNAL 12
void RouteService::remainingTimeChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 12, nullptr);
}

// SIGNAL 13
void RouteService::progressPercentChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 13, nullptr);
}

// SIGNAL 14
void RouteService::nextInstructionChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 14, nullptr);
}

// SIGNAL 15
void RouteService::distanceToNextTurnChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 15, nullptr);
}

// SIGNAL 16
void RouteService::currentPositionChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 16, nullptr);
}
QT_WARNING_POP
