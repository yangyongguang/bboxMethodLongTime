/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.6.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/qt/widgets/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.6.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[18];
    char stringdata0[229];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 17), // "fullScreenSignals"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 18), // "onOpenFolderToRead"
QT_MOC_LITERAL(4, 49, 12), // "onPlayClouds"
QT_MOC_LITERAL(5, 62, 15), // "onSliderMovedTo"
QT_MOC_LITERAL(6, 78, 12), // "cloud_number"
QT_MOC_LITERAL(7, 91, 7), // "onReset"
QT_MOC_LITERAL(8, 99, 12), // "onUpdateShow"
QT_MOC_LITERAL(9, 112, 3), // "num"
QT_MOC_LITERAL(10, 116, 12), // "isFullScreen"
QT_MOC_LITERAL(11, 129, 8), // "onUpdate"
QT_MOC_LITERAL(12, 138, 10), // "onParamSet"
QT_MOC_LITERAL(13, 149, 16), // "onClearSelection"
QT_MOC_LITERAL(14, 166, 12), // "CloudToBBoxs"
QT_MOC_LITERAL(15, 179, 17), // "std::vector<BBox>"
QT_MOC_LITERAL(16, 197, 23), // "std::vector<Cloud::Ptr>"
QT_MOC_LITERAL(17, 221, 7) // "bboxPts"

    },
    "MainWindow\0fullScreenSignals\0\0"
    "onOpenFolderToRead\0onPlayClouds\0"
    "onSliderMovedTo\0cloud_number\0onReset\0"
    "onUpdateShow\0num\0isFullScreen\0onUpdate\0"
    "onParamSet\0onClearSelection\0CloudToBBoxs\0"
    "std::vector<BBox>\0std::vector<Cloud::Ptr>\0"
    "bboxPts"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    1,   77,    2, 0x08 /* Private */,
       7,    0,   80,    2, 0x08 /* Private */,
       8,    0,   81,    2, 0x08 /* Private */,
       8,    1,   82,    2, 0x08 /* Private */,
       8,    1,   85,    2, 0x08 /* Private */,
      11,    0,   88,    2, 0x08 /* Private */,
      12,    0,   89,    2, 0x08 /* Private */,
      13,    0,   90,    2, 0x08 /* Private */,
      14,    1,   91,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 15, 0x80000000 | 16,   17,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->fullScreenSignals(); break;
        case 1: _t->onOpenFolderToRead(); break;
        case 2: _t->onPlayClouds(); break;
        case 3: _t->onSliderMovedTo((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->onReset(); break;
        case 5: _t->onUpdateShow(); break;
        case 6: _t->onUpdateShow((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->onUpdateShow((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->onUpdate(); break;
        case 9: _t->onParamSet(); break;
        case 10: _t->onClearSelection(); break;
        case 11: { std::vector<BBox> _r = _t->CloudToBBoxs((*reinterpret_cast< const std::vector<Cloud::Ptr>(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< std::vector<BBox>*>(_a[0]) = _r; }  break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::fullScreenSignals)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &BaseViewerWidget::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return BaseViewerWidget::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = BaseViewerWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::fullScreenSignals()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
