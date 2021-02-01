#include <stdafx.hpp>
#include "imgui_helper.hpp"
#include "SlamLauncher.hpp"
#include "ScanMatcher.hpp"
#include "PointCloudMap.hpp"

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

		Scan2D scan;
		if (!sReader.loadData(cnt, scan)) {
			break;
		};
    clock_t start = clock();
		sm.matchScan(scan);
		cnt++;
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

		const auto &matchedMap = GetPCM().globalMap;
		for (size_t i = 0; i < GetPCM().globalMap.size(); i+=step) {
			draw_list->AddCircle(
				ImVec2(
						matchedMap[i].x * 10 + ImGui::GetWindowWidth() / 2.0,
						matchedMap[i].y * 10 + ImGui::GetWindowHeight() / 2.0
					),
				2, 
				ImColor(
					ImVec4(1.0f, 0.0f, 0.0f, 1.00f)
				)
			);
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
