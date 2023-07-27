#include "nViewer.h"


nViewer::nViewer(QWidget* parent) :
	QMainWindow(parent)
{
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

	m_folder_data = QString::fromStdString(getdatapath());

	m_engine = new Engine(m_folder_data + "Motive Profile - 2023-05-20.motive", m_folder_data + "System Calibration.cal");


}
nViewer::~nViewer()
{
}

