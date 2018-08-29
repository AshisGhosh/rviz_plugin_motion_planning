#ifndef WAYPOINTMODEL_H
#define WAYPOINTMODEL_H

#include <QAbstractTableModel>
#include <QList>

struct Waypoint
{
    float x, y, z, qx, qy, qz, qw;

}; 


class WaypointModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    WaypointModel(QObject *parent = 0);
    WaypointModel(QList<Waypoint> waypoints, QObject *parent = 0);
    QList<Waypoint> waypoints;

    int rowCount(const QModelIndex &parent) const;
    int columnCount(const QModelIndex &parent) const;
    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
    Qt::ItemFlags flags(const QModelIndex &index) const;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole);
    bool insertRows(int position, int rows, const QModelIndex &index = QModelIndex());
    bool removeRows(int position, int rows, const QModelIndex &index = QModelIndex());
    QList<Waypoint> getWaypoints() const;
    QModelIndex index(int row, int column, const QModelIndex &parent) const;

private:
};

#endif // WAYPOINTMODEL_H