#include "waypoint_table_model.h"
#include <sstream>
 
WaypointModel::WaypointModel() : QAbstractTableModel()
{
  std::vector<float> column1;
  column1.push_back(10);
  column1.push_back(20);
  column1.push_back(30);
  column1.push_back(40);
 
  Columns.push_back(column1);
 
  std::vector<float> column2;
  column2.push_back(50);
  column2.push_back(60);
  column2.push_back(70);
  column2.push_back(80);
 
  Columns.push_back(column2);
}
 
int WaypointModel::rowCount(const QModelIndex& parent) const
{
  return Columns[0].size();
}
 
int WaypointModel::columnCount(const QModelIndex& parent) const
{
  return Columns.size();
}
 
QVariant WaypointModel::data(const QModelIndex& index, int role) const
{
  if(role == Qt::DisplayRole)
    {
    return Columns[index.column()][index.row()];
    }
  return QVariant::Invalid;
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

