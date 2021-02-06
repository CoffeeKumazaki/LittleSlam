#include <stdafx.hpp>
#include <imgui_helper.hpp>
#include <SensorDataReader.hpp>
#include <PointCloudMapGT.hpp>


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

	// Main loop
	int cnt = 0;
	std::vector<LPoint2D> glps;
	Scan2D scan;
	sReader.loadData(cnt, scan);
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
      GetPCMGT().clear();
      GetPCMGT().addPoints(scan.lps, 1);
      GetPCMGT().makeGlobalMap();
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

		ImGui::Begin("Data Resampling", nullptr, window_flags);
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    // draw grid.
    double rsize = 40;
    double csize = 0.5;
    int tsize = static_cast<int>(rsize/csize);
    int w = static_cast<int>(2*tsize+1);
    for (int c = -w; c < w; c++) {
      double x = c * csize;
      double y = -rsize;
      draw_list->AddLine(
        ImVec2(x*80 + ImGui::GetWindowWidth() / 2.0,  -y*80 + ImGui::GetWindowHeight() / 2.0),
        ImVec2(x*80 + ImGui::GetWindowWidth() / 2.0,  y*80 + ImGui::GetWindowHeight() / 2.0),
        ImColor(
          ImVec4(0.7f, 0.7f, 0.7f, 1.00f)
        ), 
        0.5
      );
    }
    for (int r = -w; r < w; r++) {
      double x = -rsize;
      double y = r * csize;
      draw_list->AddLine(
        ImVec2(-x*80 + ImGui::GetWindowWidth() / 2.0,  y*80 + ImGui::GetWindowHeight() / 2.0),
        ImVec2(x*80 + ImGui::GetWindowWidth() / 2.0, y*80 + ImGui::GetWindowHeight() / 2.0),
        ImColor(
          ImVec4(0.7f, 0.7f, 0.7f, 1.00f)
        ), 
        0.5
      );
    }

		std::string frame_title = data_file + "  id: " + std::to_string(cnt);
		ImGui::Text("%s", frame_title.c_str());

		if (ImGui::Button("next")) {
			next = true;
		}
    ImGui::SameLine();
    if (ImGui::Button("Continuous")) {
      read_contenious = !read_contenious;
    }

		std::vector<LPoint2D> resampled;
    static bool showOrg = false;

    ImGui::Checkbox("Show Org", &showOrg);
		if (showOrg) {
			for (size_t i = 0; i < scan.lps.size(); i++) {
				draw_list->AddCircle(
					ImVec2(
							scan.lps[i].x * 80 + ImGui::GetWindowWidth() / 2.0,
							scan.lps[i].y * 80 + ImGui::GetWindowHeight() / 2.0
						),
					3, 
					ImColor(
						ImVec4(1.0f, 1.0f, 1.0f, 1.00f)
					)
				);
			}
		}

    ImColor grid_colors[2] = {
      ImColor(
        ImVec4(1.0f, 0.0f, 0.0f, 1.00f)
      ),
      ImColor(
        ImVec4(1.0f, 1.0f, 0.0f, 1.00f)
      )
    };

		for (size_t i = 0; i < GetPCMGT().gt.table.size(); i++) {
      const auto grid = GetPCMGT().gt.table[i];
      for (size_t j=0; j < grid.lps.size(); j++) {
        const auto lp = grid.lps[j];
        draw_list->AddCircle(
            ImVec2(
                lp.x * 80 + ImGui::GetWindowWidth() / 2.0,
                lp.y * 80 + ImGui::GetWindowHeight() / 2.0),
            2,
            grid_colors[i%2]
          );
      }
		}
		for (size_t i = 0; i < GetPCMGT().globalMap.size(); i++) {
      const auto map = GetPCMGT().globalMap[i];
			draw_list->AddCircle(
					ImVec2(
							map.x * 80 + ImGui::GetWindowWidth() / 2.0,
							map.y * 80 + ImGui::GetWindowHeight() / 2.0),
					4,
					ImColor(
							ImVec4(0.0f, 0.0f, 1.0f, 1.00f)));
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
