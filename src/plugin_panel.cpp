#include <pluginlib/class_list_macros.h>

#include "plugin_panel.h"

PluginPanel::PluginPanel(QWidget* parent):
    rviz::Panel(parent)
{
    _vbox = new QVBoxLayout();
    _hbox1 = new QHBoxLayout();

    _button1 = new QPushButton(tr("Add"));
    _button2 = new QPushButton(tr("Delete"));

    _button1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _button2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    connect(_button1, SIGNAL (released()), this, SLOT (button1_on_click()));
    connect(_button2, SIGNAL (released()), this, SLOT (button2_on_click()));

    _listview = new QListView();

    _stringlist1 = new QStringListModel();
    QStringList list;
    list << "a" << "b" << "c";
    _stringlist1->setStringList(list);

    _listview->setModel(_stringlist1);
    _listview->
            setEditTriggers(QAbstractItemView::AnyKeyPressed |
                            QAbstractItemView::DoubleClicked);

    _hbox1->addWidget(_button1);
    _hbox1->addWidget(_button2);
    
    _vbox->addWidget(_listview);
    _vbox->addLayout(_hbox1);

    setLayout(_vbox);
}

void PluginPanel::button1_on_click()
{
    int row = _stringlist1->rowCount();

    // Enable add one or more rows
    _stringlist1->insertRows(row,1);

    // Get the row for Edit mode
    QModelIndex index = _stringlist1->index(row);

    // Enable item selection and put it edit mode
    _listview->setCurrentIndex(index);
    _listview->edit(index);
}

void PluginPanel::button2_on_click()
{
    _stringlist1->removeRows(_listview->currentIndex().row(),1);
}


PLUGINLIB_EXPORT_CLASS(PluginPanel, rviz::Panel)