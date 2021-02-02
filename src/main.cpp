#include <stdafx.hpp>
#include "imgui_helper.hpp"
#include "SlamLauncher.hpp"
#include "ScanMatcher.hpp"
#include "PointCloudMap.hpp"
#include "RefScanMaker.hpp"
#include "DataAssociator.hpp"

void drawCorrespondData(ScanMatcher& sm, Scan2D& scan) {

	RefScanMaker rsm;
	rsm.makeRefScan();
	Scan2D ref;
	rsm.getRefScan(ref);

	DataAssociator da;
	da.setRefBase(ref.lps);
	Pose2D pose = GetPCM().getLastPose();
	da.findCorrespondence(scan, pose);

	ImDrawList *draw_list = ImGui::GetWindowDrawList();
	std::cout << da.curLps.size() << std::endl;
	double scale = 1.0;
	double max_x = __DBL_MIN__;
	double min_x = __DBL_MAX__;
	double max_y = __DBL_MIN__;
	double min_y = __DBL_MAX__;
	double ave_x = 0;
	double ave_y = 0;
	for (size_t i = 0; i < da.curLps.size(); i++)
	{
		max_x = std::max(da.curLps[i].x, max_x);
		min_x = std::min(da.curLps[i].x, min_x);
		max_y = std::max(da.curLps[i].y, max_y);
		min_y = std::min(da.curLps[i].y, min_y);
		ave_x += da.curLps[i].x;
		ave_y += da.curLps[i].y;
	}
	scale = 1/std::max((max_x-min_x)/1300, (max_y-min_y)/800);
	ave_x /= da.curLps.size();
	ave_y /= da.curLps.size();
	std::cout << scale << std::endl;

	for (size_t i = 0; i < da.curLps.size(); i++)	{
		draw_list->AddCircle(
			ImVec2(
					(da.curLps[i].x - ave_x) * scale + ImGui::GetWindowWidth() / 2.0,
					(da.curLps[i].y - ave_y) * scale + ImGui::GetWindowHeight() / 2.0
				),
			3, 
			ImColor(
				ImVec4(1.0f, 0.0f, 0.0f, 1.00f)
			)
		);
		draw_list->AddCircle(
			ImVec2(
					(da.refLps[i].x - ave_x) * scale + ImGui::GetWindowWidth() / 2.0,
					(da.refLps[i].y - ave_y) * scale + ImGui::GetWindowHeight() / 2.0
			),
			2,
			ImColor(
				ImVec4(0.0f, 0.0f, 1.0f, 1.00f))
		);
		draw_list->AddLine(
			ImVec2((da.curLps[i].x - ave_x) * scale + ImGui::GetWindowWidth() / 2.0, (da.curLps[i].y - ave_y) * scale + ImGui::GetWindowHeight() / 2.0),
			ImVec2((da.refLps[i].x - ave_x) * scale + ImGui::GetWindowWidth() / 2.0, (da.refLps[i].y - ave_y) * scale + ImGui::GetWindowHeight() / 2.0),
			ImColor(ImVec4(1.0f, 1.0f, 1.0f, 1.00f))
		);
	}
}

void drawRefScan(ScanMatcher& sm, Scan2D& scan) {

	ImDrawList *draw_list = ImGui::GetWindowDrawList();
	sm.growMap(scan, scan.pose);

	RefScanMaker rsm;
	rsm.makeRefScan();
	Scan2D ref;
	rsm.getRefScan(ref);

	ImGui::Text("Ref Scan Size: %lu", ref.lps.size());
	for (size_t i = 0; i < ref.lps.size(); i++)
	{
		draw_list->AddCircle(
			ImVec2(
					ref.lps[i].x * 10 + ImGui::GetWindowWidth() / 2.0,
					ref.lps[i].y * 10 + ImGui::GetWindowHeight() / 2.0
				),
			2, 
			ImColor(
				ImVec4(1.0f, 0.0f, 0.0f, 1.00f)
			)
		);
	}
}

void drawScan(std::vector<LPoint2D>& glps, Scan2D& scan) {

		ImDrawList *draw_list = ImGui::GetWindowDrawList();
		int step = 100;
		for (size_t i = 0; i < glps.size(); i+=step)
		{
			draw_list->AddCircle(
				ImVec2(
						glps[i].x * 10 + ImGui::GetWindowWidth() / 2.0,
						glps[i].y * 10 + ImGui::GetWindowHeight() / 2.0
					),
				2, 
				ImColor(
					ImVec4(0.3f, 0.3f, 0.3f, 1.00f)
				)
			);
		}

		for (size_t i = 0; i < scan.lps.size(); i++) {
			LPoint2D glp;
			scan.pose.getGlobalPoint(scan.lps[i], glp);
			glps.emplace_back(glp);
			draw_list->AddCircle(
				ImVec2(
						glp.x * 10 + ImGui::GetWindowWidth() / 2.0,
						glp.y * 10 + ImGui::GetWindowHeight() / 2.0
					),
				2, 
				ImColor(
					ImVec4(0.0f, 0.0f, 0.0f, 1.00f)
				)
			);
		}
}

int main(int argc, char const *argv[]) {

	int w = 1280;
	int h = 720;
	std::string title = "LittleSLAM";
	initImgui(w, h, title);

	// Our state
	bool show_demo_window = true;
	bool show_another_window = false;
	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	SensorDataReader sReader;
	std::string data_file = "../data/corridor.lsc";
	sReader.init(data_file);
	ScanMatcher sm;

	// Main loop
	int cnt = 0;
	std::vector<LPoint2D> glps;
	Scan2D scan;
	sReader.loadData(cnt, scan);
	cnt++;
	bool next = false;
	while (!glfwWindowShouldClose(window))
	{
		// Poll and handle events (inputs, window resize, etc.)
		// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
		// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
		// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
		// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
		glfwPollEvents();

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		bool dataLoad = true;
		if (next) {
			sm.growMap(scan, scan.pose);
			dataLoad = sReader.loadData(cnt, scan);
		}
		if (dataLoad)	{
			if (next) cnt++;
		}
		else{
			break;
		};
		next = false;
		clock_t start = clock();
		// sm.matchScan(scan);
    clock_t end = clock();

    const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
    printf("matchScan time %lf[ms]\n", time);

		ImGuiWindowFlags window_flags = 0;
		window_flags |= ImGuiWindowFlags_NoMove;
		window_flags |= ImGuiWindowFlags_NoResize;
		window_flags |= ImGuiWindowFlags_NoCollapse;
		window_flags |= ImGuiWindowFlags_NoBackground;
		window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus;
		ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(w, h), ImGuiCond_FirstUseEver);

		ImGui::Begin("Scan Data", nullptr, window_flags);
		std::string frame_title = data_file + "  id: " + std::to_string(cnt);
		ImGui::Text("%s", frame_title.c_str());
		ImGui::Text("pose (%lf, %lf)", scan.pose.x, scan.pose.y);
		ImGui::Text("est pose (%lf, %lf)", GetPCM().getLastPose().x, GetPCM().getLastPose().y);

		if (ImGui::Button("next")) {
			next = true;
		}

		ImDrawList *draw_list = ImGui::GetWindowDrawList();

//		drawScan(glps, scan);
//		drawRefScan(sm, scan);
		drawCorrespondData( sm, scan);
		/*
		int step = 100;
		const auto &matchedMap = GetPCM().globalMap;
		for (size_t i = 0; i < GetPCM().globalMap.size(); i+=step) {
			draw_list->AddCircle(
				ImVec2(
						matchedMap[i].x * 10 + ImGui::GetWindowWidth() / 2.0,
						matchedMap[i].y * 10 + ImGui::GetWindowHeight() / 2.0
					),
				2, 
				ImColor(
					ImVec4(0.0f, 0.0f, 1.0f, 1.00f)
				)
			);
		}
		*/

		ImGui::End();

		// Rendering
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
	}

	sReader.term();

	termImgui();
	return 0;
}
