#ifndef TABLEMODEL_H
#define TABLEMODEL_H

#include <QAbstractTableModel>
#include <QList>

struct Contact
{
    QString name;
    QString address;

    bool operator==(const Contact &other) const
    {
        return name == other.name && address == other.address;
    }
};

inline QDataStream &operator<<(QDataStream &stream, const Contact &contact)
{
    return stream << contact.name << contact.address;
}

inline QDataStream &operator>>(QDataStream &stream, Contact &contact)
{
    return stream >> contact.name >> contact.address;
}

class TableModel : public QAbstractTableModel
{
    Q_OBJECT

public:
    TableModel(QObject *parent = 0);
    TableModel(QList<Contact> contacts, QObject *parent = 0);

    int rowCount(const QModelIndex &parent) const;
    int columnCount(const QModelIndex &parent) const;
    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
    Qt::ItemFlags flags(const QModelIndex &index) const;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole);
    bool insertRows(int position, int rows, const QModelIndex &index = QModelIndex());
    bool removeRows(int position, int rows, const QModelIndex &index = QModelIndex());
    QList<Contact> getContacts() const;
    QModelIndex index(int row, int column, const QModelIndex &parent) const;

private:
    QList<Contact> contacts;
};

#endif // TABLEMODEL_H