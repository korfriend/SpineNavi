// Dear ImGui: standalone example application for DirectX 11
// If you are new to Dear ImGui, read documentation from the docs/ folder + read the top of imgui.cpp.
// Read online: https://github.com/ocornut/imgui/tree/master/docs
#include "OperationTask.hpp"
\
#include "framework.h"
#include "SNaviWin.h"

#define MAX_LOADSTRING 100
typedef unsigned long long u64;


// Forward declarations of helper functions
bool CreateDeviceD3D(HWND hWnd);
void CleanupDeviceD3D();
void CreateRenderTarget();
void CleanupRenderTarget();
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

HWND hwnd;

auto ___LoadMyVariable = [](const std::string& pname, const int idx) {
	cv::Mat ocvVec3(1, 3, CV_32FC1);
	cv::FileStorage fs(__gc.g_folder_trackingInfo + "my_test_variables.txt", cv::FileStorage::Mode::READ);
	fs[pname] >> ocvVec3;
	glm::fvec3 p1;
	memcpy(&p1, ocvVec3.ptr(), sizeof(float) * 3);
	fs.release();

	return ((float*)&p1)[idx];
};

BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData) {
	static int monitor_idx = 0;
	MONITORINFOEX monitorInfo;
	monitorInfo.cbSize = sizeof(MONITORINFOEX);
	if (GetMonitorInfo(hMonitor, &monitorInfo)) {
		std::cout << "Monitor Name: " << monitorInfo.szDevice << std::endl;
		std::cout << "Monitor Resolution: " << monitorInfo.rcMonitor.right - monitorInfo.rcMonitor.left
			<< "x" << monitorInfo.rcMonitor.bottom - monitorInfo.rcMonitor.top << std::endl;
		std::cout << "Monitor Position: (" << monitorInfo.rcMonitor.left << "," << monitorInfo.rcMonitor.top << ")"
			<< std::endl;
		std::cout << "------------------------" << std::endl;
		//if (monitor_idx == 1) 
		{
			g_posWinX = monitorInfo.rcMonitor.left;
			g_posWinY = monitorInfo.rcMonitor.top;
			g_sizeWinX = monitorInfo.rcMonitor.right - monitorInfo.rcMonitor.left;
			g_sizeWinY = monitorInfo.rcMonitor.bottom - monitorInfo.rcMonitor.top;
		}
	}
	monitor_idx++;
	return TRUE;
}

void CallBackMouseWheel(vmgui::ImGuiVzmWindow* pThis) {

	int sid, cid;
	pThis->GetSceneAndCamera(sid, cid);

	vzm::CameraParameters cp;
	vzm::GetCameraParams(cid, cp);

	ImGuiIO& io = ImGui::GetIO();
	if (io.MouseWheel > 0)
		*(glm::fvec3*)cp.pos += 0.05f * (*(glm::fvec3*)cp.view);
	else
		*(glm::fvec3*)cp.pos -= 0.05f * (*(glm::fvec3*)cp.view);
	vzm::SetCameraParams(cid, cp);

}
void CallBackMouseDown(vmgui::ImGuiVzmWindow* pThis) {
	
	if (!__gc.g_markerSelectionMode)
		return;

	ImGuiIO& io = ImGui::GetIO();
	glm::fvec2 mPos = pThis->GetMousePos();

	// renderer does not need to be called
	glm::ivec2 pos_ss = glm::ivec2(mPos);

	if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
		int aidPicked = 0;
		glm::fvec3 posPick;
		int cidRender1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
		if (vzm::PickActor(aidPicked, __FP posPick, pos_ss.x, pos_ss.y, cidRender1)) {
			std::string actorName;
			vzm::GetSceneItemName(aidPicked, actorName);
			if (__gc.g_selectedMkNames.find(actorName) == __gc.g_selectedMkNames.end()) {
				int idx = __gc.g_selectedMkNames.size();
				__gc.g_selectedMkNames[actorName] = idx;
				__gc.g_engineLogger->info("# of Picking Markers : {}", __gc.g_selectedMkNames.size());
			}
			pThis->SetStartEvent(vmgui::ImGuiVzmWindow::OnEvents::CUSTOM_DOWN);
		}
	}
}

void mygui(ImGuiIO& io) {

	if (!__gc.g_track_que.empty())
	{
		__gc.g_track_que.wait_and_pop(opcode::trackInfo);
	}

	opcode::sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	opcode::cidRender1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
	opcode::cidRender2 = vzmutils::GetSceneItemIdByName(__gc.g_camName2);
	opcode::buttonHeight = 30;

	if (opcode::trackInfo.IsValidTracking()) {
		opcode::TrackInfo2Scene();
	}

	vzm::CameraParameters cpCam1, cpCam2;
	vzm::GetCameraParams(opcode::cidRender1, opcode::cpCam1);
	vzm::GetCameraParams(opcode::cidRender2, opcode::cpCam2);

	static bool viewInit = false;
	static vmgui::ImGuiVzmWindow view1("View 1", opcode::sidScene, opcode::cidRender1, g_pd3dDevice);
	static vmgui::ImGuiVzmWindow view2("View 2", opcode::sidScene, opcode::cidRender2, g_pd3dDevice);

	if (!viewInit) {
		viewInit = true;
		view1.SetStageCenterScale(glm::fvec3(0, 0, 1.2f), 1.f);
		view2.SetStageCenterScale(glm::fvec3(0, 0, 1.2f), 1.f);
		view1.AddCallback(vmgui::ImGuiVzmWindow::OnEvents::CUSTOM_DOWN, CallBackMouseDown);
		view1.AddCallback(vmgui::ImGuiVzmWindow::OnEvents::CUSTOM_WHEEL, CallBackMouseWheel);
		view2.AddCallback(vmgui::ImGuiVzmWindow::OnEvents::CUSTOM_WHEEL, CallBackMouseWheel);
		view1.EnableCustomWHEEL(true);
		view2.EnableCustomWHEEL(true);
		view1.EnableMouseZoom(false);
		view2.EnableMouseZoom(false);
		view1.SkipRender(true);
		view2.SkipRender(true);
	}

	static ImVec4 clear_color = ImVec4(0.f, 0.2f, 0.3f, 0.95f);

	ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.5, 0.3, 0.1, 1));
	static ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove //| ImGuiWindowFlags_NoSavedSettings
		| ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoBackground; // ImGuiWindowFlags_NoDecoration | 
	ImGui::SetNextWindowPos(ImVec2(g_sizeWinX - g_sizeRightPanelW + 20, 5), ImGuiCond_Once);
	ImGui::SetNextWindowSize(ImVec2(g_sizeRightPanelW - 40, g_sizeWinY - 100), ImGuiCond_Once);
	ImGui::Begin("Control Widgets");
	{
		// tree view 넣기                        // Create a window called "Hello, world!" and append into it.
		ImGui::Spacing();
		if (ImGui::CollapsingHeader("Render Option", ImGuiTreeNodeFlags_DefaultOpen)) {
			opcode::ControlWidgets(clear_color);
		}

		if (__gc.g_optiRecordMode == OPTTRK_RECMODE::NONE || __gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD) {
			ImGui::Spacing();
			if (ImGui::CollapsingHeader("Tracker Operation", ImGuiTreeNodeFlags_DefaultOpen)) {
				opcode::TrackerOperation(view1, view2);
			}
		}

		if (__gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD) {

		}

		if (__gc.g_optiRecordMode != OPTTRK_RECMODE::LOAD) {
			ImGui::Spacing();
			if (ImGui::CollapsingHeader("Calibration (Intrinsics)", ImGuiTreeNodeFlags_DefaultOpen)) {
				opcode::CalibrateIntrinsics();
			}

			ImGui::Spacing();
			if (ImGui::CollapsingHeader("Calibration (Extrinsics)", ImGuiTreeNodeFlags_DefaultOpen)) {
				opcode::CalibrateExtrinsics();
			}
		}

		ImGui::Spacing();
		if (ImGui::CollapsingHeader("System Information", ImGuiTreeNodeFlags_DefaultOpen)) {
			opcode::SystemInfo();
		}
	}
	ImGui::End();
	ImGui::PopStyleColor();

	if (1) // ImGui::CollapsingHeader(view2.GetWindowName().c_str(), ImGuiTreeNodeFlags_DefaultOpen)
	{
		ImGui::SetNextWindowPos(ImVec2(5, 5), ImGuiCond_Once);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
		ImGui::PushStyleColor(ImGuiCol_WindowBg, clear_color); // Set window background to 
		ImGui::Begin(view1.GetWindowName().c_str());   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::PopStyleVar();
		ImGui::PopStyleColor();
		view1.View(NULL);
		ImGui::End();

		ImGui::SetNextWindowPos(ImVec2((g_sizeWinX - g_sizeRightPanelW) / 2 + 10, 5), ImGuiCond_Once);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
		ImGui::PushStyleColor(ImGuiCol_WindowBg, clear_color); // Set window background to 
		ImGui::Begin(view2.GetWindowName().c_str());   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::PopStyleVar();
		ImGui::PopStyleColor();
		view2.View(NULL);
		ImGui::End();
	}

	rendertask::AnimateFrame(opcode::cidRender1, opcode::cidRender2);
	vzm::RenderScene(opcode::sidScene, opcode::cidRender1);
	vzm::RenderScene(opcode::sidScene, opcode::cidRender2);
}

int main()
{
	// test code //
	//std::vector<cv::Point3f> pp = { cv::Point3f(1.1), cv::Point3f(2.2), cv::Point3f(3.3), cv::Point3f(4.4) };
	//cv::FileStorage fs1("test___.txt", cv::FileStorage::Mode::WRITE);
	//fs1 << "test" << cv::Mat(pp);
	//fs1.release();
	//cv::FileStorage fs2("test___.txt", cv::FileStorage::Mode::READ);
	//cv::Mat ocvTest;
	//fs2["test"] >> ocvTest;
	//fs2.release();
	//std::vector<cv::Point3f> pp2 = ocvTest;


	EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, 0);
	WNDCLASSEXW wc = { sizeof(wc), CS_CLASSDC, WndProc, 0L, 0L, GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr, L"ImGui Example", nullptr };
	::RegisterClassExW(&wc);
	hwnd = ::CreateWindowW(wc.lpszClassName, L"SNAVI SW ver 1.0", WS_OVERLAPPEDWINDOW, g_posWinX, g_posWinY, g_sizeWinX, g_sizeWinY - 50, nullptr, nullptr, wc.hInstance, nullptr);

	// Initialize Direct3D
	if (!CreateDeviceD3D(hwnd))
	{
		CleanupDeviceD3D();
		::UnregisterClassW(wc.lpszClassName, wc.hInstance);
		return 1;
	}

	// Show the window
	::ShowWindow(hwnd, SW_SHOWDEFAULT);
	::UpdateWindow(hwnd);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsLight();

	// Setup Platform/Renderer backends
	ImGui_ImplWin32_Init(hwnd);
	ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

	// Load Fonts
	// - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
	// - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
	// - If the file cannot be loaded, the function will return a nullptr. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
	// - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
	// - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
	// - Read 'docs/FONTS.md' for more instructions and details.
	// - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
	//io.Fonts->AddFontDefault();
	//io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
	//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
	//ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());
	//IM_ASSERT(font != nullptr);

	vmgui::SetImGuiContext(ImGui::GetCurrentContext());
	vmgui::ImTermWindow viewTerm;

	// Our state
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	// https://fsunuc.physics.fsu.edu/git/gwm17/imgui/src/commit/7ac16c02cc1e1c6bcbdecdd5ac0746b157395084/docs/FONTS.md
	//ImFont* font1 = io.Fonts->AddFontFromFileTTF("c:/windows/fonts/arial.ttf", 20);
	ImFontConfig config;
	config.OversampleH = 2;
	config.OversampleV = 2;
	//config.SizePixels
	config.GlyphExtraSpacing.x = 1.0f;
	config.RasterizerMultiply = 1.2f;
	//ImFont* font1 = io.Fonts->AddFontDefault();
	ImFont* font2 = io.Fonts->AddFontFromFileTTF("c:/windows/fonts/arial.ttf", 13, &config);
	//ImFont* font3 = io.Fonts->AddFontFromFileTTF("c:/windows/fonts/arial.ttf", 30);//, &config);
	io.Fonts->Build();

	__gc.Init();
	__gc.g_folder_data = GetDataPath();
	__gc.g_configFileName = __gc.g_folder_data + "config.txt";

	cv::FileStorage config_fs(__gc.g_configFileName, cv::FileStorage::Mode::READ);
	std::string configLoad, motiveProfile, trackDataFolder;
	if (config_fs.isOpened()) {
		config_fs["TrackDataFolder"] >> trackDataFolder;
		config_fs["MotiveProfile"] >> motiveProfile;
		config_fs["RecordMode"] >> configLoad;
		config_fs["RecordPeriod"] >> __gc.g_optiRecordPeriod;
		config_fs["PivotSamples"] >> __gc.g_optiPivotSamples;
	}
	else {
		trackDataFolder = "Tracking 2023-10-05";
		motiveProfile = "Motive Profile - 2023-09-15.motive";
	}
	config_fs.release();

	if (configLoad == "LOAD") {
		__gc.g_optiRecordMode = OPTTRK_RECMODE::LOAD;
	}
	else if (configLoad == "CALIB_EDIT") {
		__gc.g_optiRecordMode = OPTTRK_RECMODE::CALIB_EDIT;
	}
	__gc.g_profileFileName = __gc.g_folder_data + motiveProfile;
	__gc.g_folder_trackingInfo = __gc.g_folder_data + trackDataFolder + "/";
	__gc.g_recFileName = __gc.g_folder_trackingInfo + "TrackingRecord.csv";
	__gc.g_recScanName = __gc.g_folder_trackingInfo + "ScanRecord.csv";

	rendertask::InitializeTask(&__gc);
	nettask::InitializeTask(&__gc);
	trackingtask::InitializeTask(&__gc);
	calibtask::InitializeTask(&__gc);

	vzm::InitEngineLib("SpineNavi");

	std::shared_ptr<void> nativeLogger;
	vzm::GetLoggerPointer(nativeLogger);
	__gc.g_engineLogger = std::static_pointer_cast<spdlog::logger, void>(nativeLogger);

	if (__gc.g_optiRecordMode == OPTTRK_RECMODE::NONE) {
		
		bool optitrkMode = optitrk::InitOptiTrackLib();

		optitrk::LoadProfileAndCalibInfo(__gc.g_profileFileName, "");
		//optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-08-06.motive", folder_data + "System Calibration.cal");
		//optitrk::LoadProfileAndCalibInfo(folder_data + "Motive Profile - 2023-07-04.motive", folder_data + "System Calibration.cal");
		optitrk::SetCameraSettings(0, 4, 16, 200);
		optitrk::SetCameraSettings(1, 4, 16, 200);
		optitrk::SetCameraSettings(2, 4, 16, 200);
	}

	rendertask::SceneInit((g_sizeWinX - g_sizeRightPanelW) / 2, g_sizeWinY - g_sizeConsolePanelH - 30);
	{
		cv::FileStorage fs(__gc.g_folder_trackingInfo + "carm_checker.txt", cv::FileStorage::Mode::READ);
		if (fs.isOpened()) {
			fs["DIST_REF"] >> __gc.g_refCArmRbDist;
			fs["ANGLE_REF"] >> __gc.g_refCArmRbAngle;
			fs.release();
		}
	}

	int sidScene = vzmutils::GetSceneItemIdByName(__gc.g_sceneName);
	int cidRender1 = vzmutils::GetSceneItemIdByName(__gc.g_camName);
	int cidRender2 = vzmutils::GetSceneItemIdByName(__gc.g_camName2);
	vzm::RenderScene(sidScene, cidRender1);
	vzm::RenderScene(sidScene, cidRender2);

	std::thread tracker_processing_thread;
	if (__gc.g_optiRecordMode != OPTTRK_RECMODE::CALIB_EDIT) {
		tracker_processing_thread = std::thread(trackingtask::OptiTrackingProcess);
	}

	std::thread network_processing_thread;
	if (__gc.g_optiRecordMode == OPTTRK_RECMODE::NONE || __gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD) {
		network_processing_thread = std::thread(nettask::NetworkProcess);
	}
	// Main loop
	bool done = false;
	while (!done)
	{
		// Poll and handle messages (inputs, window resize, etc.)
		// See the WndProc() function below for our to dispatch events to the Win32 backend.
		MSG msg;
		while (::PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE))
		{
			::TranslateMessage(&msg);
			::DispatchMessage(&msg);
			if (msg.message == WM_QUIT)
				done = true;
		}
		if (done)
			break;

		// Handle window resize (we don't resize directly in the WM_SIZE handler)
		if (g_ResizeWidth != 0 && g_ResizeHeight != 0)
		{
			CleanupRenderTarget();
			g_pSwapChain->ResizeBuffers(0, g_ResizeWidth, g_ResizeHeight, DXGI_FORMAT_UNKNOWN, 0);
			g_ResizeWidth = g_ResizeHeight = 0;
			CreateRenderTarget();
		}

		// Start the Dear ImGui frame
		ImGui_ImplDX11_NewFrame();
		ImGui_ImplWin32_NewFrame();
		ImGui::NewFrame();

		mygui(io);

		ImGui::SetNextWindowPos(ImVec2(5, g_sizeWinY - g_sizeConsolePanelH), ImGuiCond_Once);
		//ImGui::SetNextWindowSize(ImVec2(g_sizeWinX - 10, g_sizeConsolePanelH - 50), ImGuiCond_Once); // not work
		viewTerm.ShowTerminal();

		// Rendering
		ImGui::Render();
		const float clear_color_with_alpha[4] = { clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w };
		g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView, nullptr);
		g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView, clear_color_with_alpha);
		ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

		g_pSwapChain->Present(1, 0); // Present with vsync
		//g_pSwapChain->Present(0, 0); // Present without vsync
	}

	if (__gc.g_optiRecordMode != OPTTRK_RECMODE::CALIB_EDIT) {
		__gc.g_tracker_alive = false; // make the thread finishes, this setting should be located right before the thread join
		tracker_processing_thread.join();
	}

	if (__gc.g_optiRecordMode == OPTTRK_RECMODE::NONE || __gc.g_optiRecordMode == OPTTRK_RECMODE::RECORD) {
		__gc.g_network_alive = false;
		network_processing_thread.join();
	}

	if (__gc.g_optiRecordMode != OPTTRK_RECMODE::LOAD) {
		optitrk::DeinitOptiTrackLib();
	}

	vzm::DeinitEngineLib();
	__gc.Deinit();

	// Cleanup
	ImGui_ImplDX11_Shutdown();
	ImGui_ImplWin32_Shutdown();
	ImGui::DestroyContext();

	for (auto kv : g_mapImgSRVs)
		kv.second->Release();

	CleanupDeviceD3D();
	::DestroyWindow(hwnd);
	::UnregisterClassW(wc.lpszClassName, wc.hInstance);

	return 0;
}

// Helper functions

bool CreateDeviceD3D(HWND hWnd)
{
	// Setup swap chain
	DXGI_SWAP_CHAIN_DESC sd;
	ZeroMemory(&sd, sizeof(sd));
	sd.BufferCount = 2;
	sd.BufferDesc.Width = 0;
	sd.BufferDesc.Height = 0;
	sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	sd.BufferDesc.RefreshRate.Numerator = 60;
	sd.BufferDesc.RefreshRate.Denominator = 1;
	sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
	sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	sd.OutputWindow = hWnd;
	sd.SampleDesc.Count = 1;
	sd.SampleDesc.Quality = 0;
	sd.Windowed = TRUE;
	sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

	UINT createDeviceFlags = 0;
	//createDeviceFlags |= D3D11_CREATE_DEVICE_DEBUG;
	D3D_FEATURE_LEVEL featureLevel;
	const D3D_FEATURE_LEVEL featureLevelArray[2] = { D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0, };
	HRESULT res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
	if (res == DXGI_ERROR_UNSUPPORTED) // Try high-performance WARP software driver if hardware is not available.
		res = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_WARP, nullptr, createDeviceFlags, featureLevelArray, 2, D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &featureLevel, &g_pd3dDeviceContext);
	if (res != S_OK)
		return false;

	CreateRenderTarget();
	return true;
}

void CleanupDeviceD3D()
{
	CleanupRenderTarget();
	if (g_pSwapChain) { g_pSwapChain->Release(); g_pSwapChain = nullptr; }
	if (g_pd3dDeviceContext) { g_pd3dDeviceContext->Release(); g_pd3dDeviceContext = nullptr; }
	if (g_pd3dDevice) { g_pd3dDevice->Release(); g_pd3dDevice = nullptr; }
}

void CreateRenderTarget()
{
	ID3D11Texture2D* pBackBuffer;
	g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
	g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr, &g_mainRenderTargetView);
	pBackBuffer->Release();
}

void CleanupRenderTarget()
{
	if (g_mainRenderTargetView) { g_mainRenderTargetView->Release(); g_mainRenderTargetView = nullptr; }
}

// Forward declare message handler from imgui_impl_win32.cpp
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

// Win32 message handler
// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
		return true;

	switch (msg)
	{
	case WM_SIZE:
		if (wParam == SIZE_MINIMIZED)
			return 0;
		g_ResizeWidth = (UINT)LOWORD(lParam); // Queue resize
		g_ResizeHeight = (UINT)HIWORD(lParam);
		return 0;
	case WM_SYSCOMMAND:
		if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
			return 0;
		break;
	case WM_DESTROY:
		::PostQuitMessage(0);
		return 0;
	}
	return ::DefWindowProcW(hWnd, msg, wParam, lParam);
}
