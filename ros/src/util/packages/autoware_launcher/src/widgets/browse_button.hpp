#ifndef AUTOWARE_LAUNCHER_BROWSE_BUTTON_HPP_
#define AUTOWARE_LAUNCHER_BROWSE_BUTTON_HPP_

#include <QPushButton>

namespace autoware_launcher {

class AwBrowseButton : public QPushButton
{
    Q_OBJECT

    public:

        AwBrowseButton(const QString& text = "Browse", QWidget* parent = nullptr);

        enum class BrowseTarget { File, FileList, SaveFile, Direcroty };
        void setBrowseTarget(BrowseTarget target);

    Q_SIGNALS:

        void fileBrowsed(const QString& path);
        void listBrowsed(const QStringList& paths);

    private Q_SLOTS:

        void onClicked(bool checked);

    private:

        BrowseTarget target_;
};

}

#endif // INCLUDE GUARD