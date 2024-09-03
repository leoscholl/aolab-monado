/*!
 * @file
 * @brief  Information and configuration window for CAVE-specific options.
 *
 *
 * @author Jeremy Auzou <jeremy.auzou@insa-rouen.fr>
 * @author Swan Remacle <swanremacle@hotmail.fr>
 * @author Jean-Marc Cherfils <jean-marc.cherfils@insa-rouen.fr>
 * @ingroup drv_cavexr
 */


#include "cavexr.h"
#include "cavexr_flystick.h"
#include "cavexr_debug.h"
#include <assert.h>

#include "os/os_time.h"

#include <glad/gl.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl.h"
#include "imgui/imgui_impl_opengl3.h"

typedef int32_t i32;
typedef uint32_t u32;

#define WIDTH 1280
#define HEIGHT 720

bool running = true;

uint64_t reset_frame;
uint64_t reset_time;

void rotate(xrt_vec3 in, xrt_quat *out) {
    // Assuming the angles are in radians.
    float c1 = cosf(in.x * (M_PI / 180.f));
    float s1 = sinf(in.x * (M_PI / 180.f));
    float c2 = cosf(in.y * (M_PI / 180.f));
    float s2 = sinf(in.y * (M_PI / 180.f));
    float c3 = cosf(in.z * (M_PI / 180.f));
    float s3 = sinf(in.z * (M_PI / 180.f));
    out->w = sqrtf(1.0f + c1 * c2 + c1 * c3 - s1 * s2 * s3 + c2 * c3) / 2.0f;
    float w4 = (4.0f * out->w);
    out->x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4;
    out->y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4;
    out->z = (-s1 * s3 + c1 * s2 * c3 + s2) / w4;
}

int cavexr_debug_window(struct cavexr *cave) {

    int width = WIDTH, height = HEIGHT;

    // Set up window
    u32 WindowFlags = SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE;
    SDL_Window *Window = SDL_CreateWindow("Informations CaveXR", 64, 64, width, height, WindowFlags);
    assert(Window);
    SDL_GLContext Context = SDL_GL_CreateContext(Window);
    SDL_GL_MakeCurrent(Window, Context);
    SDL_GL_SetSwapInterval(1); // Synchro verticale

    // Init GLAD context
    int version = gladLoadGL((GLADloadfunc) SDL_GL_GetProcAddress);
    printf("GL %d.%d\n", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));

    GLint major, minor;
    glGetIntegerv(GL_MAJOR_VERSION, &major);
    glGetIntegerv(GL_MINOR_VERSION, &minor);

    SDL_GL_SetSwapInterval(1);

    // Set up IMGUI
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void) io;
    io.IniFilename = "imgui_cavexr.ini";
    ImGui::StyleColorsLight();

    ImGui_ImplSDL2_InitForOpenGL(Window, Context);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    ImVec4 clearColor = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // State
    bool showDemoWindow = false;
    xrt_vec3 angles;
    angles.x = angles.y = angles.z = 0;

    reset_time = cave->created_ns;
    reset_frame = 0;

    while (running) {
        SDL_Event event;

        // Handle SDL events
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                running = false;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(Window))
                running = false;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame(Window);
        ImGui::NewFrame();

        {
            ImGui::SetNextWindowPos(ImVec2(400, 16), ImGuiCond_FirstUseEver, ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImVec2(150, 60), ImGuiCond_FirstUseEver);
            ImGui::Begin("Debug ImGui");

            ImGui::Checkbox("Demo Window", &showDemoWindow);

            ImGui::End();
        }

        // CAVE window
        {
            ImGui::SetNextWindowPos(ImVec2(16, 16), ImGuiCond_FirstUseEver, ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImVec2(368, 500), ImGuiCond_FirstUseEver);
            ImGui::Begin("Cave");

            ImGui::Text("Frame: %llu", cave->frame_count);

            uint64_t current_time = os_monotonic_get_ns();
            float total_time_ns = (current_time - reset_time);
            float total_time_s = time_ns_to_s(total_time_ns);
            float mean_frametime = total_time_s / (cave->frame_count - reset_frame);

            ImGui::Text("Avg framerate: %.1f FPS (%.1f ms)", 1 / mean_frametime, mean_frametime * 1000);
            if (ImGui::Button("Reset")) {
                reset_frame = cave->frame_count;
                reset_time = os_monotonic_get_ns();
            }

            ImGui::Separator();

            ImGui::Text("CONFIGURATION");

            ImGui::SliderFloat3("Dimensions", (float *) (&(cave->dimensions)), 1.f, 10.f, "%.2f m");

            ImGui::Checkbox("Enable 3D", &cave->enable_3d);
            ImGui::Checkbox("Swap eyes", &cave->invert_eyes);
            ImGui::SliderFloat("Eye distance", &cave->ipd, 0.050f, 0.072f, "%0.3f m");

            ImGui::End();
        }

        // Tracking window
        {
            ImGui::SetNextWindowPos(ImVec2(400, 92), ImGuiCond_FirstUseEver, ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImVec2(358, 424), ImGuiCond_FirstUseEver);
            ImGui::Begin("Tracking");

            float xEdge = cave->dimensions.x / 2;

            // Status
            {
                ImGui::Text(cave->dtrack->isTracking ? "DTrack detected" : "DTrack disabled or incorrectly set up");
            }

            ImGui::Separator();

            // Head
            {
                ImGui::Text(cave->dtrack->headVisible ? "Head visible" : "Head not visible");

                ImGui::Text("Position:");

                if (cave->dtrack->headVisible) {
                    ImGui::Text("Position: %.3f m / %.3f / m %.3f m",
                                cave->dtrack->headPos[0] * 0.001,
                                cave->dtrack->headPos[1] * 0.001,
                                cave->dtrack->headPos[2] * 0.001);
                } else {
                    ImGui::SliderFloat("Position X##head", (float *) (&(cave->pose.position.x)), -xEdge, xEdge, "%.2f m");
                    ImGui::SliderFloat("Position Y##head", (float *) (&(cave->pose.position.y)), 0.01f, cave->dimensions.y,
                                       "%.2f m");
                    ImGui::SliderFloat("Position Z##head", (float *) (&(cave->pose.position.z)), 0.01f, cave->dimensions.z,
                                       "%.2f m");
                }
            }

            ImGui::Separator();

            // FlyStick
            if (cave->controller)
            {
                if (cave->status.flystick_visible) {
                    ImGui::Text("FlyStick visible");
                } else {
                    ImGui::Text("FlyStick not visible");
                }

                ImGui::Text("Analog stick: %5.3f %5.3f (clic: %s)",
                            cave->controller->base.inputs[DTRACK_THUMBSTICK].value.vec2.x,
                            cave->controller->base.inputs[DTRACK_THUMBSTICK].value.vec2.y,
                            cave->controller->base.inputs[DTRACK_THUMBSTICK_CLICK].value.boolean ? "yes" : "no");

                ImGui::Text("Blue buttons: %s %s %s %s",
                            cave->controller->base.inputs[DTRACK_4].value.boolean ? "yes" : "no",
                            cave->controller->base.inputs[DTRACK_3].value.boolean ? "yes" : "no",
                            cave->controller->base.inputs[DTRACK_2].value.boolean ? "yes" : "no",
                            cave->controller->base.inputs[DTRACK_1].value.boolean ? "yes" : "no");

                ImGui::Text("Trigger: %d",
                            cave->controller->base.inputs[DTRACK_TRIGGER].value.boolean);



                if (cave->status.flystick_visible) {
                    ImGui::Text("Position: %.3f m / %.3f / m %.3f m",
                                cave->dtrack->flystickPos[0] * 0.001,
                                cave->dtrack->flystickPos[1] * 0.001,
                                cave->dtrack->flystickPos[2] * 0.001);
                } else {
                    ImGui::SliderFloat("Position X##flystick", (float *) (&(cave->controller->pose.position.x)), -xEdge, xEdge, "%.2f m");
                    ImGui::SliderFloat("Position Y##flystick", (float *) (&(cave->controller->pose.position.y)), 0.0f, cave->dimensions.y,
                                       "%.2f m");
                    ImGui::SliderFloat("Position Z##flystick", (float *) (&(cave->controller->pose.position.z)), 0.0f, cave->dimensions.z,
                                       "%.2f m");
                }
            }

            ImGui::End();
        }

        if (showDemoWindow) {
            ImGui::ShowDemoWindow();
        }

        ImGui::Render();
        glViewport(0, 0, io.DisplaySize.x, io.DisplaySize.y);
        glClearColor(clearColor.x, clearColor.y, clearColor.z, 0.f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(Window);
        SDL_Delay(16);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(Context);
    SDL_DestroyWindow(Window);
    SDL_Quit();

    return 0;
}

void cavexr_close_debug_window() {
    running = false;
}
