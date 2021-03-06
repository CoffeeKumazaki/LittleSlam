#include <stdafx.hpp>
#include "imgui_helper.hpp"
#include "SlamLauncher.hpp"
#include "ScanMatcher.hpp"
#include "PointCloudMap.hpp"
#include "RefScanMaker.hpp"
#include "ScanPointResampler.hpp"
#include "DataAssociator.hpp"


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
	ScanPointResampler spr;

	// Main loop
	int cnt = 0;
	std::vector<LPoint2D> glps;
	std::vector<Pose2D> estPoses;
	std::vector<Pose2D> odPoses;
	Scan2D scan;
	sReader.loadData(cnt, scan);
	estPoses.emplace_back(scan.pose);
	odPoses.emplace_back(scan.pose);
	cnt++;
	bool read_contenious = false;
	bool next = read_contenious;
	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		bool dataLoad = true;
		if (next) {
			dataLoad = sReader.loadData(cnt, scan);
			odPoses.emplace_back(scan.pose);
			clock_t start = clock();
			sm.matchScan(scan);
			sm.growMap(scan, scan.pose);
			clock_t end = clock();
			const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
			printf("matchScan time %lf[ms]\n", time);
			estPoses.emplace_back(GetPCM().getLastPose());
		}
		if (dataLoad)	{
			if (next) cnt++;
		}
		else{
			break;
		};
		next = read_contenious;

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
		ImGui::Text("odometry (%lf, %lf)", scan.pose.x, scan.pose.y);
		ImGui::Text("slam     (%lf, %lf)", GetPCM().getLastPose().x, GetPCM().getLastPose().y);
		ImGui::Text("diff     (%lf, %lf)", GetPCM().getLastPose().x - scan.pose.x, GetPCM().getLastPose().y - scan.pose.y);


    if (ImGui::Button(read_contenious ? "Pose" : "Continuous")) {
      read_contenious = !read_contenious;
    }
		ImGui::SameLine();
		if (ImGui::Button("next")) {
			next = true;
		}

		ImDrawList *draw_list = ImGui::GetWindowDrawList();

		int step = GetPCM().globalMap.size() > 10000 ? 100 : 1;
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

		for (size_t i = 1; i < odPoses.size(); i++)
		{
			draw_list->AddLine(
				ImVec2(
					odPoses[i].x*10 + ImGui::GetWindowWidth() / 2.0, odPoses[i].y*10+ ImGui::GetWindowHeight() / 2.0), 
				ImVec2(
					odPoses[i-1].x*10 + ImGui::GetWindowWidth() / 2.0, odPoses[i-1].y*10+ ImGui::GetWindowHeight() / 2.0), 
					ImColor(ImVec4(1.0, 0.0, 0.0, 1.0)), 1.5);
		}
		for (size_t i = 1; i < estPoses.size(); i++)
		{
			draw_list->AddLine(
				ImVec2(
					estPoses[i].x*10 + ImGui::GetWindowWidth() / 2.0, estPoses[i].y*10+ ImGui::GetWindowHeight() / 2.0), 
				ImVec2(
					estPoses[i-1].x*10 + ImGui::GetWindowWidth() / 2.0, estPoses[i-1].y*10+ ImGui::GetWindowHeight() / 2.0), 
					ImColor(ImVec4(1.0, 1.0, 1.0, 1.0)), 1.0);
		}

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
