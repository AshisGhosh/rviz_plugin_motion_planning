#include <QtWidgets>

#include <rviz/panel.h>

class PluginPanel: public rviz::Panel
{
    Q_OBJECT

public:
    PluginPanel(QWidget* parent = 0);

protected:
    QVBoxLayout* _vbox;
    QHBoxLayout* _hbox1;

    QPushButton* _button1;
    QPushButton* _button2;

    QListView* _listview;
    QStringListModel* _stringlist1;

private Q_SLOTS:
    void button1_on_click();
    void button2_on_click();
};