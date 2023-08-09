#include "nViewer.h"


nViewer::nViewer(QWidget* parent) :
	QMainWindow(parent)
{

	ui.setupUi(this);
	this->setWindowTitle("Navigation");
	QPalette pal = this->palette();
	pal.setColor(QPalette::Base, QColor(212, 212, 212));
	pal.setColor(QPalette::Window, QColor(242, 242, 242));
	pal.setColor(QPalette::Window, QColor(53, 53, 53));
	pal.setColor(QPalette::WindowText, Qt::white);
	pal.setColor(QPalette::Base, QColor(35, 35, 35));
	pal.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
	pal.setColor(QPalette::ToolTipBase, QColor(25, 25, 25));
	pal.setColor(QPalette::ToolTipText, Qt::white);
	pal.setColor(QPalette::Text, Qt::white);
	pal.setColor(QPalette::Button, QColor(53, 53, 53));
	pal.setColor(QPalette::ButtonText, Qt::white);
	pal.setColor(QPalette::BrightText, Qt::red);
	pal.setColor(QPalette::Link, QColor(42, 130, 218));
	pal.setColor(QPalette::Highlight, QColor(42, 130, 218));
	pal.setColor(QPalette::HighlightedText, QColor(35, 35, 35));
	pal.setColor(QPalette::Active, QPalette::Button, QColor(53, 53, 53));
	pal.setColor(QPalette::Disabled, QPalette::ButtonText, Qt::darkGray);
	pal.setColor(QPalette::Disabled, QPalette::WindowText, Qt::darkGray);
	pal.setColor(QPalette::Disabled, QPalette::Text, Qt::darkGray);
	pal.setColor(QPalette::Disabled, QPalette::Light, QColor(53, 53, 53));

	this->setAutoFillBackground(true);
	this->setPalette(pal);

	this->showMaximized();


	//Engine init
	auto getdatapath = []()
	{
		using namespace std;
		char ownPth[2048];
		GetModuleFileNameA(NULL, ownPth, (sizeof(ownPth)));
		string exe_path = ownPth;
		string exe_path_;
		size_t pos = 0;
		std::string token;
		string delimiter = "\\"; // windows
		while ((pos = exe_path.find(delimiter)) != std::string::npos) {
			token = exe_path.substr(0, pos);
			if (token.find(".exe") != std::string::npos) break;
			exe_path += token + "/";
			exe_path_ += token + "/";
			exe_path.erase(0, pos + delimiter.length());
		}
		return exe_path_ + "../../data/";
	};

	this->initLayout();

	m_folder_data = QString::fromStdString(getdatapath());
	m_engine = new Engine(m_mainFrame, m_folder_data, m_folder_data + "Motive Profile - 2023-05-20.motive", m_folder_data + "System Calibration.cal");

}
nViewer::~nViewer()
{

	SAFE_DELETE_OBJECT(m_engine);
	SAFE_DELETE_OBJECT(m_mainFrame);
	SAFE_DELETE_OBJECT(m_thumbnails);
}

void nViewer::initLayout()
{

	m_mainFrame = new ViewLayout(this);


	QHBoxLayout* hboxLayout = new QHBoxLayout(this->centralWidget());
	m_thumbnails = new QListWidget(); // thumbnail view

	m_thumbnails->setViewMode(QListWidget::IconMode);
	m_thumbnails->setIconSize(QSize(200, 150));
	m_thumbnails->setResizeMode(QListWidget::Adjust);

	hboxLayout->addWidget(m_mainFrame, 90);
	hboxLayout->addWidget(m_thumbnails, 10);


}

