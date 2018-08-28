#include "waypoint_table_model.h"
#include <sstream>
 
WaypointModel::WaypointModel() : QAbstractTableModel()
{
  std::vector<float> row1;
  row1.push_back(10);
  row1.push_back(20);
  row1.push_back(30);
  row1.push_back(40);
 
  Rows.push_back(row1);
 
  std::vector<float> row2;
  row2.push_back(50);
  row2.push_back(60);
  row2.push_back(70);
  row2.push_back(80);
 
  Rows.push_back(row2);
}
 
int WaypointModel::rowCount(const QModelIndex& parent) const
{
  return Rows.size();
}
 
int WaypointModel::columnCount(const QModelIndex& parent) const
{
  return Rows[0].size();
}
 
QVariant WaypointModel::data(const QModelIndex& index, int role) const
{
  if(role == Qt::DisplayRole)
    {
    return Rows[index.column()][index.row()];
    }
  return QVariant::Invalid;
}

bool WaypointModel::setData(const QModelIndex &index, const QVariant &value, int role) const
 {
         if (index.isValid() && role == Qt::EditRole) {
                 int row = index.row();

                 std::vector<float> v = Rows.value(row)
                 
                 if (index.column() == 0)
                         v[0] = value.toFloat();
                 else if (index.column() == 1)
                         v[1] = value.toFloat();
         else
             return false;

         Rows.replace(row, v);
         emit dataChanged(index, index);

         return true;
         }

         return false;
 }
 
QVariant WaypointModel::headerData(int section, Qt::Orientation orientation, int role) const
{
 
  if(role == Qt::DisplayRole)
    {
    std::stringstream ss;
    if(orientation == Qt::Horizontal)
      {
      ss << "H_" << section;
      return QString(ss.str().c_str());
      }
    else if(orientation == Qt::Vertical)
      {
      ss << "V_" << section;
      return QString(ss.str().c_str());
      }
 
    }
 
  return QVariant::Invalid;
}