#include "tablemodel.h"

TableModel::TableModel(QObject *parent)
    : QAbstractTableModel(parent)
{
}

TableModel::TableModel(QList<Contact> contacts, QObject *parent)
    : QAbstractTableModel(parent)
    , contacts(contacts)
{
}

int TableModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return contacts.size();
}

int TableModel::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 2;
}

QVariant TableModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (index.row() >= contacts.size() || index.row() < 0)
        return QVariant();

    if (role == Qt::DisplayRole) {
        const Contact &contact = contacts.at(index.row());

        if (index.column() == 0)
            return contact.name;
        else if (index.column() == 1)
            return contact.address;
    }
    return QVariant();
}

QVariant TableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal) {
        switch (section) {
            case 0:
                return tr("Name");

            case 1:
                return tr("Address");

            default:
                return QVariant();
        }
    }
    return QVariant();
}

bool TableModel::insertRows(int position, int rows, const QModelIndex &index)
{
    Q_UNUSED(index);
    beginInsertRows(QModelIndex(), position, position + rows - 1);

    for (int row = 0; row < rows; ++row)
        contacts.insert(position, { QString(), QString() });

    endInsertRows();
    return true;
}

bool TableModel::removeRows(int position, int rows, const QModelIndex &index)
{
    Q_UNUSED(index);
    beginRemoveRows(QModelIndex(), position, position + rows - 1);

    for (int row = 0; row < rows; ++row)
        contacts.removeAt(position);

    endRemoveRows();
    return true;
}

bool TableModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (index.isValid() && role == Qt::EditRole) {
        int row = index.row();

        Contact contact = contacts.value(row);

        if (index.column() == 0)
            contact.name = value.toString();
        else if (index.column() == 1)
            contact.address = value.toString();
        else
            return false;

        contacts.replace(row, contact);
        Q_EMIT (dataChanged(index, index));

        return true;
    }

    return false;
}

Qt::ItemFlags TableModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return Qt::ItemIsEnabled;

    return QAbstractTableModel::flags(index) | Qt::ItemIsEditable;
}

QList<Contact> TableModel::getContacts() const
{
    return contacts;
}

QModelIndex TableModel::index(int row, int column, const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return createIndex(row, column);
}