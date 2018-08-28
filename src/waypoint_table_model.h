#include <QtWidgets>

class WaypointModel : public QAbstractTableModel
{
public:
  WaypointModel();
 
  int rowCount(const QModelIndex& parent) const;
  int columnCount(const QModelIndex& parent) const;
  QVariant data(const QModelIndex& index, int role) const;
  bool setData(const QModelIndex &index, const QVariant &value, int role) const;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const;
 
protected:
  std::vector<std::vector<float> > Rows;
};