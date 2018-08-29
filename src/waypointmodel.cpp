#include "waypointmodel.h"

WaypointModel::WaypointModel(QObject *parent)
    : QAbstractTableModel(parent)
{
}

WaypointModel::WaypointModel(QList<Waypoint> waypoints, QObject *parent)
    : QAbstractTableModel(parent)
    , waypoints(waypoints)
{
}

int WaypointModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return waypoints.size();
}

int WaypointModel::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 7;
}

QVariant WaypointModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (index.row() >= waypoints.size() || index.row() < 0)
        return QVariant();

    if (role == Qt::DisplayRole) {
        const Waypoint &waypoint = waypoints.at(index.row());

        if (index.column() == 0)
            return waypoint.x;
        else if (index.column() == 1)
            return waypoint.y;
        else if (index.column() == 2)
            return waypoint.z;
        else if (index.column() == 3)
            return waypoint.qx;
        else if (index.column() == 4)
            return waypoint.qy;
        else if (index.column() == 5)
            return waypoint.qz;
        else if (index.column() == 6)
            return waypoint.qw;

    }
    return QVariant();
}

QVariant WaypointModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal) {
        switch (section) {
            case 0:
                return tr("X");

            case 1:
                return tr("Y");

            case 2:
                return tr("Z");

            case 3:
                return tr("qX");

            case 4:
                return tr("qY");

            case 5:
                return tr("qZ");

            case 6:
                return tr("qW");

            default:
                return QVariant();
        }
    }
    return QVariant();
}

bool WaypointModel::insertRows(int position, int rows, const QModelIndex &index)
{
    Q_UNUSED(index);
    beginInsertRows(QModelIndex(), position, position + rows - 1);
    Waypoint waypoint = {};

    for (int row = 0; row < rows; ++row)
        waypoints.insert(position, waypoint);

    endInsertRows();
    return true;
}

bool WaypointModel::removeRows(int position, int rows, const QModelIndex &index)
{
    Q_UNUSED(index);
    beginRemoveRows(QModelIndex(), position, position + rows - 1);

    for (int row = 0; row < rows; ++row)
        waypoints.removeAt(position);

    endRemoveRows();
    return true;
}

bool WaypointModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (index.isValid() && role == Qt::EditRole) {
        int row = index.row();

        Waypoint waypoint = waypoints.value(row);

        if (index.column() == 0)
            waypoint.x = value.toFloat();
        else if (index.column() == 1)
            waypoint.y = value.toFloat();
        else if (index.column() == 2)
            waypoint.z = value.toFloat();
        else if (index.column() == 3)
            waypoint.qx = value.toFloat();
        else if (index.column() == 4)
            waypoint.qy = value.toFloat();
        else if (index.column() == 5)
            waypoint.qz = value.toFloat();
        else if (index.column() == 6)
            waypoint.qw = value.toFloat();
        else
            return false;

        waypoints.replace(row, waypoint);
        Q_EMIT (dataChanged(index, index));

        return true;
    }

    return false;
}

Qt::ItemFlags WaypointModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    return QAbstractTableModel::flags(index) | Qt::ItemIsEditable;
}

QList<Waypoint> WaypointModel::getWaypoints() const
{
    return waypoints;
}


QModelIndex WaypointModel::index(int row, int column, const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return createIndex(row, column);
}